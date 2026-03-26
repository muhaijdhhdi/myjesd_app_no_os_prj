#include <stdio.h>
#include <inttypes.h>
#include "app_config.h"
#include "parameters.h"
#include <xparameters.h>
#include <xil_printf.h>
#include <xil_cache.h>
#include "axi_adxcvr.h"
#include "no_os_spi.h"
#include "no_os_gpio.h"
#include "xilinx_spi.h"
#include "xilinx_gpio.h"
#include "no_os_delay.h"
#include "no_os_clk.h"
#include "no_os_axi_io.h"
#include "no_os_error.h"
#include "ad9152.h"
#include "ad9528.h"
#include "ad9680.h"
#include "axi_adc_core.h"
#include "axi_dac_core.h"
#include "axi_dmac.h"
#include "axi_jesd204_tx.h"
#include "axi_jesd204_rx.h"
//#include "xilinx_uart.h"
static uint16_t adc_buffer[BUFFER_SAMPLES] __attribute__((aligned(1024)));

#define JESD204_RX_REG_LINK_CLK_RATIO	0x0C8
#define JESD204_RX_REG_SYSREF_CONF	0x100
#define JESD204_RX_REG_SYSREF_LMFC_OFFSET	0x104
#define JESD204_RX_REG_SYSREF_STATUS	0x108
#define JESD204_RX_REG_LINK_CONF0	0x210
#define JESD204_RX_REG_LINK_STATUS	0x280
#define JESD204_RX_REG_LANE_STATUS(x)	(((x) * 32) + 0x300)
#define JESD204_RX_REG_LANE_LATENCY(x)	(((x) * 32) + 0x304)
#define JESD204_RX_REG_LANE_ERRORS(x)	(((x) * 32) + 0x308)
#define AD9680_REG_JESD204B_LANE_POWERDOWN	0x5B0

/*
 * FMC1 debug build:
 * use a single AD9680 converter over two JESD lanes at 12.5 Gbps.
 * The PL is expected to remap the physically connected pair as logical lane0/1.
 */
#define FH8052_ADC_ACTIVE_LANES		2
#define FH8052_ADC_ACTIVE_CHANNELS	1
#define FH8052_RX_LANE_RATE_KHZ		12333333U
#define FH8052_RX_REF_CLK_KHZ		616667U
#define FH8052_RX_DEVICE_CLK_KHZ	(FH8052_RX_LANE_RATE_KHZ / 40U)
#define FH8052_RX_1G_LANE_RATE_KHZ	10000000U
#define FH8052_RX_1G_REF_CLK_KHZ	500000U
#define FH8052_RX_1G_DEVICE_CLK_KHZ	(FH8052_RX_1G_LANE_RATE_KHZ / 40U)
#define FH8052_RX_HALF_LANE_RATE_KHZ	6166667U
#define FH8052_RX_HALF_REF_CLK_KHZ	308333U
#define FH8052_RX_HALF_DEVICE_CLK_KHZ	(FH8052_RX_HALF_LANE_RATE_KHZ / 40U)

static void ad9528_config_channel(struct ad9528_channel_spec *channel,
				  uint8_t channel_num,
				  uint16_t channel_divider,
				  uint8_t signal_source)
{
	channel->channel_num = channel_num;
	channel->channel_divider = channel_divider;
	channel->output_dis = 0;
	channel->driver_mode = DRIVER_MODE_LVDS;
	channel->signal_source = signal_source;
	channel->divider_phase = 0;
	channel->sync_ignore_en = 0;
}

static void ad9528_log_output_rate(struct ad9528_dev *dev,
				   uint8_t channel_num,
				   const char *label)
{
	const struct ad9528_channel_spec *channel = NULL;
	uint64_t rate_hz = 0;
	uint32_t source_rate_hz = 0;
	uint32_t i;
	int32_t ret;

	if (dev->clk_desc && dev->clk_desc[channel_num]) {
		ret = no_os_clk_recalc_rate(dev->clk_desc[channel_num], &rate_hz);
		if (!ret) {
			printf("AD9528 %-18s ch%-2u rate=%"PRIu64" Hz\n",
			       label, channel_num, rate_hz);
			return;
		}
	}

	for (i = 0; i < dev->pdata->num_channels; i++) {
		if (dev->pdata->channels[i].channel_num == channel_num) {
			channel = &dev->pdata->channels[i];
			break;
		}
	}

	if (!channel || !channel->channel_divider) {
		printf("AD9528 %-18s ch%-2u rate read failed: %"PRIi32"\n",
		       label, channel_num, -ENOENT);
		return;
	}

	switch (channel->signal_source) {
	case SOURCE_VCO:
		source_rate_hz = dev->ad9528_st.vco_out_freq[AD9528_VCO];
		break;
	case SOURCE_VCXO:
		source_rate_hz = dev->ad9528_st.vco_out_freq[AD9528_VCXO];
		break;
	case SOURCE_SYSREF_VCO:
		source_rate_hz = dev->ad9528_st.vco_out_freq[AD9528_SYSREF];
		break;
	default:
		printf("AD9528 %-18s ch%-2u rate read failed: %"PRIi32"\n",
		       label, channel_num, -EINVAL);
		return;
	}

	rate_hz = source_rate_hz / channel->channel_divider;
	printf("AD9528 %-18s ch%-2u rate=%"PRIu64" Hz\n",
	       label, channel_num, rate_hz);
}

static int32_t ad9680_log_lane_assign(struct ad9680_dev *dev, const char *tag)
{
	uint8_t lane_pd = 0;
	uint8_t serd0 = 0;
	uint8_t serd1 = 0;
	uint8_t serd2 = 0;
	uint8_t serd3 = 0;
	int32_t ret;

	ret = ad9680_spi_read(dev, AD9680_REG_JESD204B_LANE_POWERDOWN, &lane_pd);
	if (ret)
		return ret;

	ret = ad9680_spi_read(dev, AD9680_REG_JESD204B_LANE_SERD_OUT0_ASSIGN, &serd0);
	if (ret)
		return ret;

	ret = ad9680_spi_read(dev, AD9680_REG_JESD204B_LANE_SERD_OUT1_ASSIGN, &serd1);
	if (ret)
		return ret;

	ret = ad9680_spi_read(dev, AD9680_REG_JESD204B_LANE_SERD_OUT2_ASSIGN, &serd2);
	if (ret)
		return ret;

	ret = ad9680_spi_read(dev, AD9680_REG_JESD204B_LANE_SERD_OUT3_ASSIGN, &serd3);
	if (ret)
		return ret;

	printf("AD9680 lane assign (%s): PD=0x%02x S0=0x%02x S1=0x%02x S2=0x%02x S3=0x%02x\n",
	       tag, lane_pd, serd0, serd1, serd2, serd3);

	return 0;
}

static void jesd204_rx_log_debug(struct axi_jesd204_rx *jesd)
{
	uint32_t link_conf0 = 0;
	uint32_t sysref_conf = 0;
	uint32_t sysref_status = 0;
	uint32_t clk_ratio = 0;
	uint32_t measured_link_clk_khz = 0;
	uint32_t octets_per_multiframe;
	uint32_t octets_per_frame;
	int32_t ret;

	ret = no_os_axi_io_read(jesd->base, JESD204_RX_REG_LINK_CONF0, &link_conf0);
	if (ret)
		goto read_error;

	ret = no_os_axi_io_read(jesd->base, JESD204_RX_REG_SYSREF_CONF, &sysref_conf);
	if (ret)
		goto read_error;

	ret = no_os_axi_io_read(jesd->base, JESD204_RX_REG_SYSREF_STATUS, &sysref_status);
	if (ret)
		goto read_error;

	ret = no_os_axi_io_read(jesd->base, JESD204_RX_REG_LINK_CLK_RATIO, &clk_ratio);
	if (ret)
		goto read_error;

	octets_per_multiframe = (link_conf0 & 0xff) + 1;
	octets_per_frame = ((link_conf0 >> 16) & 0xff) + 1;

	if (clk_ratio != 0)
		measured_link_clk_khz = NO_OS_DIV_ROUND_CLOSEST_ULL(100000ULL * clk_ratio,
						1ULL << 16);

	printf("%s debug regs:\n", jesd->name);
	printf("\tLINK_CONF0      [0x%03x] = 0x%08"PRIx32"\n",
	       JESD204_RX_REG_LINK_CONF0, link_conf0);
	printf("\t  octets/frame=%"PRIu32", octets/multiframe=%"PRIu32"\n",
	       octets_per_frame, octets_per_multiframe);
	printf("\tSYSREF_CONF     [0x%03x] = 0x%08"PRIx32"\n",
	       JESD204_RX_REG_SYSREF_CONF, sysref_conf);
	printf("\tSYSREF_STATUS   [0x%03x] = 0x%08"PRIx32"\n",
	       JESD204_RX_REG_SYSREF_STATUS, sysref_status);
	printf("\tLINK_CLK_RATIO  [0x%03x] = 0x%08"PRIx32,
	       JESD204_RX_REG_LINK_CLK_RATIO, clk_ratio);
	if (clk_ratio != 0)
		printf(" (%"PRIu32".%.3"PRIu32" MHz)\n",
		       measured_link_clk_khz / 1000,
		       measured_link_clk_khz % 1000);
	else
		printf(" (off)\n");

	return;

read_error:
	printf("%s debug regs read failed: %"PRIi32"\n", jesd->name, ret);
}

static const char *jesd204_rx_link_state_label(uint32_t link_status)
{
	switch (link_status & 0x3) {
	case 0:
		return "RESET";
	case 1:
		return "WAIT FOR PHY";
	case 2:
		return "CGS";
	case 3:
		return "DATA";
	default:
		return "UNKNOWN";
	}
}

static const char *jesd204_rx_lane_state_label(uint32_t lane_status)
{
	switch (lane_status & 0x3) {
	case 0:
		return "INIT";
	case 1:
		return "CHECK";
	case 2:
		return "DATA";
	default:
		return "UNKNOWN";
	}
}

static void jesd204_rx_log_lane_summary(struct axi_jesd204_rx *jesd,
					uint32_t lane_count)
{
	uint32_t lane;
	uint32_t lane_status;
	uint32_t lane_latency;
	uint32_t lane_errors;
	int32_t ret;

	printf("%s lane summary:\n", jesd->name);

	for (lane = 0; lane < lane_count && lane < jesd->num_lanes; lane++) {
		ret = no_os_axi_io_read(jesd->base, JESD204_RX_REG_LANE_STATUS(lane),
					&lane_status);
		if (ret)
			goto read_error;

		ret = no_os_axi_io_read(jesd->base, JESD204_RX_REG_LANE_LATENCY(lane),
					&lane_latency);
		if (ret)
			goto read_error;

		ret = no_os_axi_io_read(jesd->base, JESD204_RX_REG_LANE_ERRORS(lane),
					&lane_errors);
		if (ret)
			goto read_error;

		printf("\tlane%-2"PRIu32" status=0x%08"PRIx32" (%s), latency=%"PRIu32", errors=%"PRIu32"\n",
		       lane, lane_status, jesd204_rx_lane_state_label(lane_status),
		       lane_latency, lane_errors);
	}

	return;

read_error:
	printf("%s lane summary read failed at lane %"PRIu32": %"PRIi32"\n",
	       jesd->name, lane, ret);
}

static void jesd204_rx_scan_lmfc_offset(struct axi_jesd204_rx *jesd,
					uint16_t max_offset)
{
	uint32_t original_offset = 0;
	uint32_t best_offset = UINT32_MAX;
	uint32_t offset;
	uint32_t link_status;
	uint32_t sysref_status;
	int32_t ret;

	ret = no_os_axi_io_read(jesd->base, JESD204_RX_REG_SYSREF_LMFC_OFFSET,
				&original_offset);
	if (ret) {
		printf("%s LMFC offset read failed: %"PRIi32"\n", jesd->name, ret);
		return;
	}

	printf("%s SYSREF_LMFC_OFFSET scan (current=%"PRIu32"):\n",
	       jesd->name, original_offset);

	for (offset = 0; offset <= max_offset; offset++) {
		axi_jesd204_rx_lane_clk_disable(jesd);
		ret = no_os_axi_io_write(jesd->base, JESD204_RX_REG_SYSREF_LMFC_OFFSET,
					 offset);
		if (ret) {
			printf("\toffset=%"PRIu32" write failed: %"PRIi32"\n",
			       offset, ret);
			continue;
		}

		no_os_mdelay(2);
		axi_jesd204_rx_lane_clk_enable(jesd);
		no_os_mdelay(20);

		ret = no_os_axi_io_read(jesd->base, JESD204_RX_REG_LINK_STATUS,
					&link_status);
		if (ret) {
			printf("\toffset=%"PRIu32" link status read failed: %"PRIi32"\n",
			       offset, ret);
			continue;
		}

		ret = no_os_axi_io_read(jesd->base, JESD204_RX_REG_SYSREF_STATUS,
					&sysref_status);
		if (ret) {
			printf("\toffset=%"PRIu32" sysref status read failed: %"PRIi32"\n",
			       offset, ret);
			continue;
		}

		printf("\toffset=%2"PRIu32" -> link=%s, sysref_status=0x%08"PRIx32"\n",
		       offset, jesd204_rx_link_state_label(link_status), sysref_status);

		if ((sysref_status & 0x1) && !(sysref_status & 0x2) &&
		    best_offset == UINT32_MAX)
			best_offset = offset;
	}

	if (best_offset == UINT32_MAX)
		best_offset = original_offset;

	axi_jesd204_rx_lane_clk_disable(jesd);
	no_os_axi_io_write(jesd->base, JESD204_RX_REG_SYSREF_LMFC_OFFSET,
			   best_offset);
	no_os_mdelay(2);
	axi_jesd204_rx_lane_clk_enable(jesd);
	no_os_mdelay(20);

	printf("%s SYSREF_LMFC_OFFSET selected=%"PRIu32"%s\n",
	       jesd->name, best_offset,
	       (best_offset == original_offset) ? " (original)" : "");
}


static void fh8052_select_sampling_mode(struct ad9528_platform_data *ad9528_pdata,
					struct ad9680_init_param *ad9680_param,
					struct adxcvr_init *ad9680_xcvr_param,
					struct jesd204_rx_init *ad9680_jesd_param)
{
	uint8_t mode = 0;

	printf("Select ADC sampling mode:\n");
	printf("\t1 - 1233.333 MSPS (lane rate 12.333333 Gbps)\n");
	printf("\t3 - 1000.000 MSPS (lane rate 10.000000 Gbps)\n");
	printf("\t2 - 616.666 MSPS (lane rate 6.166667 Gbps)\n");
	printf("Choice [1/2/3]: ");

	mode = getc(stdin);
	printf("%c\n", mode);

	switch (mode) {
	case '2':
		printf("Selected 616.666 MSPS mode\n");
		ad9528_pdata->pll2_vco_div_m1 = 3;
		ad9528_pdata->pll2_n2_div = 37;
		ad9680_param->lane_rate_kbps = FH8052_RX_HALF_LANE_RATE_KHZ;
		ad9680_xcvr_param->lane_rate_khz = FH8052_RX_HALF_LANE_RATE_KHZ;
		ad9680_xcvr_param->ref_rate_khz = FH8052_RX_HALF_REF_CLK_KHZ;
		ad9680_jesd_param->lane_clk_khz = FH8052_RX_HALF_LANE_RATE_KHZ;
		ad9680_jesd_param->device_clk_khz = FH8052_RX_HALF_DEVICE_CLK_KHZ;
		ad9528_pdata->channels[0].channel_divider = 2;
		ad9528_pdata->channels[2].channel_divider = 4;
		ad9528_pdata->channels[4].channel_divider = 2;
		ad9528_pdata->channels[6].channel_divider = 4;
		break;
	case '3':
		printf("Selected 1000.000 MSPS mode\n");
		ad9528_pdata->pll2_vco_div_m1 = 4;
		ad9528_pdata->pll2_n2_div = 30;
		ad9680_param->lane_rate_kbps = FH8052_RX_1G_LANE_RATE_KHZ;
		ad9680_xcvr_param->lane_rate_khz = FH8052_RX_1G_LANE_RATE_KHZ;
		ad9680_xcvr_param->ref_rate_khz = FH8052_RX_1G_REF_CLK_KHZ;
		ad9680_jesd_param->lane_clk_khz = FH8052_RX_1G_LANE_RATE_KHZ;
		ad9680_jesd_param->device_clk_khz = FH8052_RX_1G_DEVICE_CLK_KHZ;
		ad9528_pdata->channels[0].channel_divider = 1;
		ad9528_pdata->channels[2].channel_divider = 2;
		ad9528_pdata->channels[4].channel_divider = 1;
		ad9528_pdata->channels[6].channel_divider = 2;
		break;
	default:
		printf("Selected 1233.333 MSPS mode\n");
		ad9528_pdata->pll2_vco_div_m1 = 3;
		ad9528_pdata->pll2_n2_div = 37;
		ad9680_param->lane_rate_kbps = FH8052_RX_LANE_RATE_KHZ;
		ad9680_xcvr_param->lane_rate_khz = FH8052_RX_LANE_RATE_KHZ;
		ad9680_xcvr_param->ref_rate_khz = FH8052_RX_REF_CLK_KHZ;
		ad9680_jesd_param->lane_clk_khz = FH8052_RX_LANE_RATE_KHZ;
		ad9680_jesd_param->device_clk_khz = FH8052_RX_DEVICE_CLK_KHZ;
		ad9528_pdata->channels[0].channel_divider = 1;
		ad9528_pdata->channels[2].channel_divider = 2;
		ad9528_pdata->channels[4].channel_divider = 1;
		ad9528_pdata->channels[6].channel_divider = 2;
		break;
	}
}

/***************************************************************************//**
 * @brief main
 ******************************************************************************/
int main(void)
{

	int32_t status;

	/* Initialize SPI structures */
	struct no_os_spi_init_param ad9528_spi_param = {
		.device_id = SPI_DEVICE_ID,
		.max_speed_hz = 2000000u,
		.chip_select = 0,
		.mode = NO_OS_SPI_MODE_0
	};

//	struct no_os_spi_init_param ad9152_spi_param = {
//		.device_id = SPI_DEVICE_ID,
//		.max_speed_hz = 2000000u,
//		.chip_select = 1,
//		.mode = NO_OS_SPI_MODE_0
//	};

	struct no_os_spi_init_param ad9680_spi_param = {
		.device_id = SPI_DEVICE_ID,
		.max_speed_hz = 2000000u,
		.chip_select = 2,
		.mode = NO_OS_SPI_MODE_0
	};


	struct xil_spi_init_param xil_spi_param = {
#ifdef PLATFORM_MB
		.type = SPI_PL,
#else
		.type = SPI_PS,
#endif
	};
	ad9528_spi_param.platform_ops = &xil_spi_ops;
	ad9528_spi_param.extra = &xil_spi_param;
//	ad9152_spi_param.platform_ops = &xil_spi_ops;
//	ad9152_spi_param.extra = &xil_spi_param;
	ad9680_spi_param.platform_ops = &xil_spi_ops;
	ad9680_spi_param.extra = &xil_spi_param;


	/* Initialize GPIO structures */
	struct no_os_gpio_init_param dac_txen_param = {
		.number = GPIO_DAC_TXEN,
	};

	struct no_os_gpio_init_param adc_pd_param = {
		.number = GPIO_ADC_PD
	};



	struct xil_gpio_init_param xil_gpio_param = {
#ifdef PLATFORM_MB
		.type = GPIO_PL,
#else
		.type = GPIO_PS,
#endif
		.device_id = GPIO_DEVICE_ID
	};
	dac_txen_param.extra = &xil_gpio_param;
	dac_txen_param.platform_ops = &xil_gpio_ops;
	adc_pd_param.extra = &xil_gpio_param;
	adc_pd_param.platform_ops = &xil_gpio_ops;

	struct no_os_gpio_desc *dac_txen;
	struct no_os_gpio_desc *adc_pd;

	struct ad9528_dev* ad9528_device;
//	struct ad9152_dev* ad9152_device;
	struct ad9680_dev* ad9680_device;

	struct ad9528_channel_spec ad9528_channels[8] = {0};
	struct ad9528_platform_data ad9528_pdata = {0};

	struct ad9528_init_param ad9528_param = {0};
	//struct ad9152_init_param ad9152_param;
	struct ad9680_init_param ad9680_param = {0};

	ad9528_param.spi_init = ad9528_spi_param;
	//ad9152_param.spi_init = ad9152_spi_param;
	ad9680_param.spi_init = ad9680_spi_param;


//	struct adxcvr_init ad9152_xcvr_param = {
//		.name = "ad9152_xcvr",
//		.base = XPAR_AXI_AD9152_XCVR_BASEADDR,
//		.sys_clk_sel = ADXCVR_SYS_CLK_QPLL0,
//		.out_clk_sel = ADXCVR_REFCLK_DIV2,
//		.lpm_enable = 1,
//		.ref_rate_khz = 616500,
//		.lane_rate_khz = 12330000,
//	};

	struct adxcvr_init ad9680_xcvr_param = {
		.name = "ad9680_xcvr",
		.base = XPAR_AXI_AD9680_XCVR_BASEADDR,
		.sys_clk_sel = ADXCVR_SYS_CLK_CPLL,
		.out_clk_sel = ADXCVR_REFCLK_DIV2,
		.lpm_enable = 1,
		.ref_rate_khz = FH8052_RX_REF_CLK_KHZ,
		.lane_rate_khz = FH8052_RX_LANE_RATE_KHZ
	};


	//struct adxcvr	*ad9152_xcvr;
	struct adxcvr	*ad9680_xcvr;

//	/* JESD initialization */
//	struct jesd204_tx_init ad9152_jesd_param = {
//		.name = "ad9152_jesd",
//		.base = TX_JESD_BASEADDR,
//		.octets_per_frame = 1,
//		.frames_per_multiframe = 32,
//		.converters_per_device = 2,
//		.converter_resolution = 16,
//		.bits_per_sample = 16,
//		.high_density = false,
//		.control_bits_per_sample = 0,
//		.subclass = 1,
//		.device_clk_khz = 12330000 / 40,
//		.lane_clk_khz = 12330000
//	};

	struct jesd204_rx_init  ad9680_jesd_param = {
		.name = "ad9680_jesd",
		.base = RX_JESD_BASEADDR,
		.octets_per_frame = 1,
		.frames_per_multiframe = 32,
		.subclass = 1,
		.device_clk_khz = FH8052_RX_DEVICE_CLK_KHZ,
		.lane_clk_khz = FH8052_RX_LANE_RATE_KHZ
	};

	//struct axi_jesd204_tx *ad9152_jesd;
	struct axi_jesd204_rx *ad9680_jesd;

	/* ADC Core */
	struct axi_adc_init ad9680_core_param = {
		.name = "ad9680_adc",
		.base = RX_CORE_BASEADDR,
		.num_channels = FH8052_ADC_ACTIVE_CHANNELS
	};
	struct axi_adc	*ad9680_core;

	/* DAC (AD9152) channels configuration */
	//struct axi_dac_channel ad9152_channels[2];
//	ad9152_channels[0].dds_dual_tone = 0;
//	ad9152_channels[0].dds_frequency_0 = 10*1000*1000;
//	ad9152_channels[0].dds_phase_0 = 0;
//	ad9152_channels[0].dds_scale_0 = 500000;
//	ad9152_channels[0].sel = AXI_DAC_DATA_SEL_DDS;
//	ad9152_channels[1].dds_dual_tone = 0;
//	ad9152_channels[1].dds_frequency_0 = 20*1000*1000;
//	ad9152_channels[1].dds_phase_0 = 0;
//	ad9152_channels[1].dds_scale_0 = 500000;
//	ad9152_channels[0].pat_data = 0xb1b0a1a0;
//	ad9152_channels[1].pat_data = 0xd1d0c1c0;
//	ad9152_channels[1].sel = AXI_DAC_DATA_SEL_DDS;

	/* DAC Core */
//	struct axi_dac_init ad9152_core_param = {
//		.name = "ad9152_dac",
//		.base =	TX_CORE_BASEADDR,
//		.num_channels = 2,
//		.channels = &ad9152_channels[0],
//		.rate = 3
//	};
//	struct axi_dac	*ad9152_core;

	struct axi_dmac_init ad9680_dmac_param = {
		.name = "ad9680_dmac",
		.base = RX_DMA_BASEADDR,
		.irq_option = IRQ_DISABLED
	};
	struct axi_dmac *ad9680_dmac;

	// ad9528 defaults
	ad9528_param.gpio_resetb = NULL;
	ad9528_param.pdata = &ad9528_pdata;
	ad9528_param.pdata->num_channels = 8;
	ad9528_param.pdata->channels = &ad9528_channels[0];
	ad9528_init(&ad9528_param);



	// adc dev sysref from the dedicated internal SYSREF generator
	ad9528_config_channel(&ad9528_channels[3], 9, 1, SOURCE_SYSREF_VCO);

	// adc sysref from the dedicated internal SYSREF generator
	ad9528_config_channel(&ad9528_channels[1], 1, 1, SOURCE_SYSREF_VCO);

	// adc-fpga-clock (625M)
	ad9528_config_channel(&ad9528_channels[2], 13, 2, SOURCE_VCO);

	// adc-device-clock (1.25G)
	ad9528_config_channel(&ad9528_channels[4], 2, 1, SOURCE_VCO);

	// dac dev sysref from the dedicated internal SYSREF generator
	ad9528_config_channel(&ad9528_channels[7], 8, 1, SOURCE_SYSREF_VCO);

	// dac sysref from the dedicated internal SYSREF generator
	ad9528_config_channel(&ad9528_channels[5], 5, 1, SOURCE_SYSREF_VCO);

	// dac-fpga-fmc (625M)
	ad9528_config_channel(&ad9528_channels[6], 7, 2, SOURCE_VCO);

	// dac-device-clock (1.25G)
	ad9528_config_channel(&ad9528_channels[0], 4, 1, SOURCE_VCO);

	// PLL2 plan for the FMC1 HDL:
	// VCXO = 100 MHz (external hardware input, not generated by AD9528)
	// internal PLL2 VCO = 100 MHz * (M1 * N2) / R1 = 100 * (3 * 25) / 2 = 3.75 GHz
	// distributed VCO output = 3.75 GHz / M1 = 1.25 GHz
	// output dividers then generate 625 MHz (/2) and 4.8828125 MHz (/256)
	ad9528_param.pdata->spi3wire = 1;
	ad9528_param.pdata->vcxo_freq = 100000000;
	ad9528_param.pdata->osc_in_diff_en = 1;
	ad9528_param.pdata->pll2_charge_pump_current_nA = 35000;
	ad9528_param.pdata->pll2_vco_div_m1 = 3;
	ad9528_param.pdata->pll2_r1_div = 3;
	ad9528_param.pdata->pll2_ndiv_a_cnt = 3;
	ad9528_param.pdata->pll2_ndiv_b_cnt = 27;
	ad9528_param.pdata->pll2_n2_div = 37;
	ad9528_param.pdata->sysref_src = SYSREF_SRC_INTERNAL;
	ad9528_param.pdata->sysref_k_div = 128;
	ad9528_param.pdata->rpole2 = RPOLE2_900_OHM;
	ad9528_param.pdata->rzero = RZERO_1850_OHM;
	ad9528_param.pdata->cpole1 = CPOLE1_16_PF;
	ad9528_param.pdata->sysref_pattern_mode = SYSREF_PATTERN_CONTINUOUS;
	ad9528_param.pdata->sysref_nshot_mode = SYSREF_NSHOT_4_PULSES;
	ad9528_param.pdata->sysref_req_en = false;
	ad9528_param.pdata->pll1_bypass_en = true;
	ad9528_param.pdata->pll2_bypass_en = false;

//	ad9152_param.stpl_samples[0][0] =
//		(ad9152_channels[0].pat_data >> 0)  & 0xffff;
//	ad9152_param.stpl_samples[0][1] =
//		(ad9152_channels[0].pat_data >> 16) & 0xffff;
//	ad9152_param.stpl_samples[0][2] =
//		(ad9152_channels[0].pat_data >> 0)  & 0xffff;
//	ad9152_param.stpl_samples[0][3] =
//		(ad9152_channels[0].pat_data >> 16) & 0xffff;
//	ad9152_param.stpl_samples[1][0] =
//		(ad9152_channels[1].pat_data >> 0)  & 0xffff;
//	ad9152_param.stpl_samples[1][1] =
//		(ad9152_channels[1].pat_data >> 16) & 0xffff;
//	ad9152_param.stpl_samples[1][2] =
//		(ad9152_channels[1].pat_data >> 0)  & 0xffff;
//	ad9152_param.stpl_samples[1][3] =
//		(ad9152_channels[1].pat_data >> 16) & 0xffff;
//	ad9152_param.interpolation = 1;
//	ad9152_param.lane_rate_kbps = 12330000;

	// adc settings
	ad9680_param.lane_rate_kbps = FH8052_RX_LANE_RATE_KHZ;
	ad9680_param.quick_config = AD9680_QUICK_CFG_M1_L2_F1;

#ifndef ALTERA_PLATFORM
	/* Enable the instruction cache. */
	Xil_ICacheEnable();
	/* Enable the data cache. */
	Xil_DCacheEnable();
#endif

	/* set GPIOs */
	no_os_gpio_get(&dac_txen,  &dac_txen_param);
	no_os_gpio_get(&adc_pd,    &adc_pd_param);

	no_os_gpio_direction_output(dac_txen,  1);
	no_os_gpio_direction_output(adc_pd,    0);

	fh8052_select_sampling_mode(ad9528_param.pdata,
				    &ad9680_param,
				    &ad9680_xcvr_param,
				    &ad9680_jesd_param);

	/* Reconfigure the default JESD configurations */
	ad9680_jesd_param.lane_clk_khz = ad9680_xcvr_param.lane_rate_khz;
	ad9680_jesd_param.device_clk_khz = ad9680_xcvr_param.lane_rate_khz / 40;
//	ad9152_jesd_param.lane_clk_khz = ad9152_xcvr_param.lane_rate_khz;
//	ad9152_jesd_param.device_clk_khz = ad9152_xcvr_param.lane_rate_khz / 40;

	status = ad9528_setup(&ad9528_device, ad9528_param);
	if (status != 0) {
		printf("error: ad9528_setup() failed\n");
	} else {
		uint32_t readback = 0;

		status = ad9528_spi_read_n(ad9528_device, AD9528_READBACK, &readback);
		if (status == 0) {
			printf("AD9528 readback=0x%04"PRIx32" VCXO_OK=%u PLL2_OK=%u PLL1_OK=%u PLL2_LOCK=%u PLL1_LOCK=%u\n",
			       readback,
			       !!(readback & AD9528_VCXO_OK),
			       !!(readback & AD9528_PLL2_OK),
			       !!(readback & AD9528_PLL1_OK),
			       !!(readback & AD9528_PLL2_LOCKED),
			       !!(readback & AD9528_PLL1_LOCKED));
		}

		ad9528_log_output_rate(ad9528_device, 2, "adc-device-clock");
		ad9528_log_output_rate(ad9528_device, 13, "adc-fpga-clock");
		ad9528_log_output_rate(ad9528_device, 1, "adc-sysref");
		ad9528_log_output_rate(ad9528_device, 9, "adc-dev-sysref");
		ad9528_log_output_rate(ad9528_device, 4, "dac-device-clock");
		ad9528_log_output_rate(ad9528_device, 7, "dac-fpga-clock");
		ad9528_log_output_rate(ad9528_device, 5, "dac-sysref");
		ad9528_log_output_rate(ad9528_device, 8, "dac-dev-sysref");
	}

	// Recommended DAC JESD204 link startup sequence
	//   1. FPGA JESD204 Link Layer
	//   2. FPGA JESD204 PHY Layer
	//   3. DAC

	status = ad9680_setup(&ad9680_device, &ad9680_param);
	if (status != 0) {
		printf("error: ad9680_setup() failed\n");
		return status;
	} else {
		status = ad9680_log_lane_assign(ad9680_device, "default");
		if (status)
			printf("AD9680 lane assign read failed: %"PRIi32"\n", status);
	}
//	status = axi_jesd204_tx_init(&ad9152_jesd, &ad9152_jesd_param);
//	if (status != 0) {
//		printf("error: %s: axi_jesd204_rx_init() failed\n", ad9152_jesd->name);
//	}
//	status = axi_jesd204_tx_lane_clk_enable(ad9152_jesd);
//	if (status != 0) {
//		printf("error: %s: axi_jesd204_tx_lane_clk_enable() failed\n",
//		       ad9152_jesd->name);
//	}
//	status = adxcvr_init(&ad9152_xcvr, &ad9152_xcvr_param);
//	if (status != 0) {
//		printf("error: %s: adxcvr_init() failed\n", ad9152_xcvr->name);
//	}
//#ifndef ALTERA_PLATFORM
//	status = adxcvr_clk_enable(ad9152_xcvr);
//	if (status != 0) {
//		printf("error: %s: adxcvr_clk_enable() failed\n", ad9152_xcvr->name);
//	}
//#endif
	status = adxcvr_init(&ad9680_xcvr, &ad9680_xcvr_param);
	if (status != 0) {
		printf("error: ad9680_xcvr: adxcvr_init() failed (%"PRIi32")\n", status);
		return status;
	}
#ifndef ALTERA_PLATFORM
	status = adxcvr_clk_enable(ad9680_xcvr);
	if (status != 0) {
		printf("error: ad9680_xcvr: adxcvr_clk_enable() failed (%"PRIi32")\n", status);
		return status;
	}
#endif
	status = axi_jesd204_rx_init_legacy(&ad9680_jesd, &ad9680_jesd_param);
	if (status != 0) {
		printf("error: %s: axi_jesd204_rx_init_legacy() failed\n",
		       ad9680_jesd->name);
	}
	status = axi_jesd204_rx_lane_clk_enable(ad9680_jesd);
	if (status != 0) {
		printf("error: %s: axi_jesd204_tx_lane_clk_enable() failed\n",
		       ad9680_jesd->name);
	}
//	status = ad9152_setup(&ad9152_device, ad9152_param);
//	if (status != 0) {
//		printf("error: ad9152_setup() failed\n");
//	}
	status = axi_adc_init(&ad9680_core,  &ad9680_core_param);
	if (status != 0) {
		printf("axi_adc_init() error: %s\n", ad9680_core->name);
	}
//	status = axi_dac_init(&ad9152_core, &ad9152_core_param);
//	if (status != 0) {
//		printf("axi_dac_init() error: %s\n", ad9152_core->name);
//	}

	jesd204_rx_log_debug(ad9680_jesd);

	status = axi_jesd204_rx_status_read(ad9680_jesd);
	if (status != 0) {
		printf("axi_jesd204_rx_status_read() error: %"PRIi32"\n", status);
	}

	jesd204_rx_log_lane_summary(ad9680_jesd, FH8052_ADC_ACTIVE_LANES);
	jesd204_rx_scan_lmfc_offset(ad9680_jesd, 31);
	jesd204_rx_log_debug(ad9680_jesd);

	status = axi_jesd204_rx_status_read(ad9680_jesd);
	if (status != 0) {
		printf("axi_jesd204_rx_status_read() error after LMFC scan: %"PRIi32"\n",
		       status);
	}

	jesd204_rx_log_lane_summary(ad9680_jesd, FH8052_ADC_ACTIVE_LANES);

//	status = axi_jesd204_tx_status_read(ad9152_jesd);
//	if (status != 0) {
//		printf("axi_jesd204_tx_status_read() error: %"PRIi32"\n", status);
//	}


	//ad9152_status(ad9152_device);

	// ad9152-xN (n > 1) supports data path prbs

//	ad9152_channels[0].sel = AXI_DAC_DATA_SEL_PN23;
//	ad9152_channels[1].sel = AXI_DAC_DATA_SEL_PN23;
//	axi_dac_data_setup(ad9152_core);
//	ad9152_param.prbs_type = AD9152_TEST_PN7;
//	ad9152_datapath_prbs_test(ad9152_device, ad9152_param);



//	ad9152_channels[0].sel = AXI_DAC_DATA_SEL_PN31;
//	ad9152_channels[1].sel = AXI_DAC_DATA_SEL_PN31;
//	axi_dac_data_setup(ad9152_core);
//	ad9152_param.prbs_type = AD9152_TEST_PN15;
//	ad9152_datapath_prbs_test(ad9152_device, ad9152_param);
//
//
//	ad9152_channels[0].sel = AXI_DAC_DATA_SEL_DDS;
//	ad9152_channels[1].sel = AXI_DAC_DATA_SEL_DDS;
//	axi_dac_data_setup(ad9152_core);

	ad9680_test(ad9680_device, AD9680_TEST_OFF);

	// capture data with DMA
	/* Initialize the DMAC and transfer 16384 samples from ADC to MEM */
	axi_dmac_init(&ad9680_dmac, &ad9680_dmac_param);

	struct axi_dma_transfer transfer_rx = {
		// Number of bytes to write/read
		.size = 16384 * 2,
		// Transfer done flag
		.transfer_done = 0,
		// Signal transfer mode
		.cyclic = NO,
		// Address of data source
		.src_addr = 0,
		// Address of data destination
			.dest_addr = (uintptr_t)adc_buffer
	};
	while(1)
	{
		axi_dmac_transfer_start(ad9680_dmac, &transfer_rx);
		status = axi_dmac_transfer_wait_completion(ad9680_dmac, 500);
			if(status)
				return status;
			#ifdef XILINX_PLATFORM
				Xil_DCacheInvalidateRange((uintptr_t)adc_buffer,
							  16384 * sizeof(adc_buffer[0]));
			#endif
	}







	printf("daq3: setup and configuration is done\n");



	return(0);
}
