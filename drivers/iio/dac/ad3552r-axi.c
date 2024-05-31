// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices AD3552R
 * Digital to Analog converter driver, axi version
 *
 * Copyright 2024 Analog Devices Inc.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/regmap.h>
#include <linux/iopoll.h>
#include <linux/gpio/consumer.h>
#include <linux/platform_device.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/iio/backend.h>

#include "ad3552r.h"

#define AXI_REG_RSTN			0x40
#define AXI_MSK_RSTN			BIT(0)
#define AXI_MSK_MMCM_RSTN		BIT(1)

#define AXI_REG_CNTRL_1			0x44

#define AXI_REG_CNTRL_2			0x48
#define AXI_MSK_USIGN_DATA		BIT(4)
#define AXI_MSK_SYMB_8B			BIT(14)
#define AXI_MSK_SDR_DDR_N		BIT(16)

#define AXI_REG_CNTRL_DATA_RD		0x80
#define AXI_MSK_DATA_RD_8		GENMASK(7, 0)
#define AXI_MSK_DATA_RD_16		GENMASK(15, 0)

#define AXI_REG_CNTRL_DATA_WR		0x84
#define AXI_MSK_DATA_WR_8		GENMASK(23, 16)
#define AXI_MSK_DATA_WR_16		GENMASK(23, 8)

#define AXI_REG_UI_STATUS		0x88
#define AXI_MSK_BUSY			BIT(4)

#define AXI_REG_CNTRL_CSTM		0x8C
#define AXI_MSK_TRANSFER_DATA		BIT(0)
#define AXI_MSK_STREAM			BIT(1)
#define AXI_MSK_SYNCED_TRANSFER		BIT(2)
#define AXI_MSK_ADDRESS			GENMASK(31, 24)

#define AXI_REG_CHAN_CNTRL_7_CH0	0x418
#define AXI_REG_CHAN_CNTRL_7_CH1	0x458
#define AXI_EXT_SYNC_ARM		0x02
#define AXI_SEL_SRC_DMA			0x02
#define AXI_SEL_SRC_ADC			0x08
#define AXI_SEL_SRC_DDS			0x0b

#define AXI_RST				(AXI_MSK_RSTN | \
					 AXI_MSK_MMCM_RSTN)

#define AXI_TFER_8BIT_SDR		(AXI_MSK_SYMB_8B | \
					 AXI_MSK_SDR_DDR_N)
#define AXI_TFER_8BIT_DDR		AXI_MSK_SYMB_8B
#define AXI_TFER_16BIT_SDR		AXI_MSK_SDR_DDR_N
#define AXI_TFER_16BIT_DDR		0x00

#define AXI_STREAM_START		(AXI_MSK_TRANSFER_DATA | \
					 AXI_MSK_STREAM)

#define RD_ADDR(x)			(BIT(7) | (x))
#define SET_CH0_RANGE(x)		FIELD_PREP(AD3552R_MASK_CH0_RANGE, x)
#define SET_CH1_RANGE(x)		FIELD_PREP(AD3552R_MASK_CH1_RANGE, x)
#define GET_CH0_RANGE(x)		FIELD_GET(AD3552R_MASK_CH0_RANGE, x)
#define GET_CH1_RANGE(x)		FIELD_GET(AD3552R_MASK_CH1_RANGE, x)
#define CNTRL_CSTM_ADDR(x)		FIELD_PREP(AXI_MSK_ADDRESS, x)
#define CNTRL_DATA_WR_8(x)		FIELD_PREP(AXI_MSK_DATA_WR_8, x)
#define CNTRL_DATA_WR_16(x)		FIELD_PREP(AXI_MSK_DATA_WR_16, x)

struct ad3552r_axi_state {
	struct gpio_desc *reset_gpio;
	struct clk *ref_clk;
	struct device *dev;
	bool ddr;
	bool single_channel;
	bool synced_transfer;
	struct iio_backend *back;
	struct regmap *axi_regs;
};

enum ad35525_stream_status {
	AD3552R_STOP_STREAM,
	AD3552R_START_STREAM,
	AD3552R_START_STREAM_SYNCED,
};

enum ad35525_source {
	AD3552R_ADC,
	AD3552R_DMA,
	AD3552R_RAMP,
};

static const char *const stream_status[] = {
	[AD3552R_STOP_STREAM] = "stop_stream",
	[AD3552R_START_STREAM] = "start_stream",
	[AD3552R_START_STREAM_SYNCED] = "start_stream_synced"
};

static const char *const input_source[] = {
	[AD3552R_ADC] = "adc_input",
	[AD3552R_DMA] = "dma_input",
	[AD3552R_RAMP] = "ramp_input"
};

static const char *const output_range[] = {
	[AD3552R_CH_OUTPUT_RANGE_0__2P5V] = "0/2.5V",
	[AD3552R_CH_OUTPUT_RANGE_0__5V] = "0/5V",
	[AD3552R_CH_OUTPUT_RANGE_0__10V] = "0/10V",
	[AD3552R_CH_OUTPUT_RANGE_NEG_5__5V]   = "-5/+5V",
	[AD3552R_CH_OUTPUT_RANGE_NEG_10__10V] = "-10/+10V"
};

static void ad3552r_axi_spi_write(struct ad3552r_axi_state *st, u32 reg,
				  u32 val, u32 transfer_params)
{
	if (transfer_params & AXI_MSK_SYMB_8B)
		regmap_write(st->axi_regs, AXI_REG_CNTRL_DATA_WR,
				  CNTRL_DATA_WR_8(val));
	else
		regmap_write(st->axi_regs, AXI_REG_CNTRL_DATA_WR,
				  CNTRL_DATA_WR_16(val));

	regmap_write(st->axi_regs, AXI_REG_CNTRL_2, transfer_params);

	regmap_update_bits(st->axi_regs, AXI_REG_CNTRL_CSTM, AXI_MSK_ADDRESS,
			   CNTRL_CSTM_ADDR(reg));
	regmap_update_bits(st->axi_regs, AXI_REG_CNTRL_CSTM,
			   AXI_MSK_TRANSFER_DATA, AXI_MSK_TRANSFER_DATA);
	mdelay(100);
	regmap_update_bits(st->axi_regs, AXI_REG_CNTRL_CSTM,
			   AXI_MSK_TRANSFER_DATA, 0);
}

static u32 ad3552r_axi_spi_read(struct ad3552r_axi_state *st, u32 reg,
				u32 transfer_params)
{
	u32 check;
	unsigned int val;

	ad3552r_axi_spi_write(st, RD_ADDR(reg), 0x00, transfer_params);

	regmap_read_poll_timeout(st->axi_regs, AXI_REG_UI_STATUS,
				 check, check == AXI_MSK_BUSY, 10, 100);

	regmap_read(st->axi_regs, AXI_REG_CNTRL_DATA_RD, &val);

	return val;
}

static void ad3552r_axi_spi_update_bits(struct ad3552r_axi_state *st, u32 reg,
					u32 mask, u32 val, u32 transfer_params)
{
	u32 tmp, orig;

	orig = ad3552r_axi_spi_read(st, reg, transfer_params);
	tmp = orig & ~mask;
	tmp |= val & mask;

	if (tmp != orig)
		ad3552r_axi_spi_write(st, reg, tmp, transfer_params);
}

static int ad3552r_axi_read_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int *val, int *val2, long mask)
{
	struct ad3552r_axi_state *st = iio_priv(indio_dev);
	int ch = chan->channel;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (st->single_channel)
			*val = DIV_ROUND_UP(clk_get_rate(st->ref_clk), 4);
		else
			*val = DIV_ROUND_UP(clk_get_rate(st->ref_clk), 8);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_RAW:
		*val = ad3552r_axi_spi_read(st, AD3552R_REG_ADDR_CH_DAC_16B(ch),
					    AXI_TFER_16BIT_SDR);
		return IIO_VAL_INT;
	}

	return -EINVAL;
}

static int ad3552r_axi_write_raw(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan,
				 int val, int val2, long mask)
{
	struct ad3552r_axi_state *st = iio_priv(indio_dev);
	int ch = chan->channel;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ad3552r_axi_spi_write(st, AD3552R_REG_ADDR_CH_DAC_16B(ch),
				      (u32)val, AXI_TFER_16BIT_SDR);
		return 0;
	}

	return -EINVAL;
}

static int ad3552r_axi_reg_access(struct iio_dev *indio_dev,
				  unsigned int reg, unsigned int writeval,
				  unsigned int *readval)
{
	struct ad3552r_axi_state *st = iio_priv(indio_dev);

	if (readval)
		return regmap_read(st->axi_regs, reg, readval);

	regmap_write(st->axi_regs, reg, writeval);

	return 0;
}

static int ad3552r_axi_update_scan_mode(struct iio_dev *indio_dev,
					const unsigned long *active_scan_mask)
{
	struct ad3552r_axi_state *st = iio_priv(indio_dev);
	u32 loop_len, val;

	st->ddr = true;
	ad3552r_axi_spi_update_bits(st, AD3552R_REG_ADDR_INTERFACE_CONFIG_D,
				    AD3552R_MASK_SPI_CONFIG_DDR,
				    AD3552R_MASK_SPI_CONFIG_DDR,
				    AXI_TFER_8BIT_SDR);

	switch (*active_scan_mask) {
	case AD3552R_CH0_ACTIVE:
		st->single_channel = true;
		loop_len = AD3552R_STREAM_2BYTE_LOOP;
		val = AD3552R_REG_ADDR_CH_DAC_16B(0);
		break;
	case AD3552R_CH1_ACTIVE:
		st->single_channel = true;
		loop_len = AD3552R_STREAM_2BYTE_LOOP;
		val = AD3552R_REG_ADDR_CH_DAC_16B(1);
		break;
	case AD3552R_CH0_CH1_ACTIVE:
		st->single_channel = false;
		loop_len = AD3552R_STREAM_4BYTE_LOOP;
		val = AD3552R_REG_ADDR_CH_DAC_16B(1);
		break;
	default:
		return -EINVAL;
	}

	ad3552r_axi_spi_write(st, AD3552R_REG_ADDR_STREAM_MODE, loop_len,
			      AXI_TFER_8BIT_DDR);
	regmap_update_bits(st->axi_regs, AXI_REG_CNTRL_CSTM, AXI_MSK_ADDRESS,
				CNTRL_CSTM_ADDR(val));

	return 0;
}

static int ad3552r_axi_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ad3552r_axi_state *st = iio_priv(indio_dev);

	return 0;
}

static int ad3552r_axi_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct ad3552r_axi_state *st = iio_priv(indio_dev);

	return 0;
}

static int ad3552r_axi_set_output_range(struct iio_dev *indio_dev,
					const struct iio_chan_spec *chan,
					unsigned int mode)
{
	struct ad3552r_axi_state *st = iio_priv(indio_dev);

	ad3552r_axi_spi_update_bits(st, AD3552R_REG_ADDR_CH0_CH1_OUTPUT_RANGE,
				    AD3552R_MASK_CH_OUTPUT_RANGE,
				    SET_CH1_RANGE(mode) | SET_CH0_RANGE(mode),
				    AXI_TFER_8BIT_SDR);
	mdelay(100);

	return 0;
}

static int ad3552r_axi_get_output_range(struct iio_dev *indio_dev,
					const struct iio_chan_spec *chan)
{
	struct ad3552r_axi_state *st = iio_priv(indio_dev);
	u32 val;

	val = ad3552r_axi_spi_read(st, AD3552R_REG_ADDR_CH0_CH1_OUTPUT_RANGE,
				   AXI_TFER_8BIT_SDR);

	return GET_CH0_RANGE(val);
}

static int ad3552r_axi_set_input_source(struct iio_dev *indio_dev,
					const struct iio_chan_spec *chan,
					unsigned int mode)
{
	struct ad3552r_axi_state *st = iio_priv(indio_dev);
	int source;

	switch (mode) {
	default:
	case AD3552R_ADC:
		source = AXI_SEL_SRC_ADC;
		break;
	case AD3552R_DMA:
		source = AXI_SEL_SRC_DMA;
		break;
	case AD3552R_RAMP:
		source = AXI_SEL_SRC_DDS;
		break;
	}

	regmap_write(st->axi_regs, AXI_REG_CHAN_CNTRL_7_CH0, source);
	regmap_write(st->axi_regs, AXI_REG_CHAN_CNTRL_7_CH1, source);

	return 0;
}

static int ad3552r_axi_get_input_source(struct iio_dev *indio_dev,
					const struct iio_chan_spec *chan)
{
	struct ad3552r_axi_state *st = iio_priv(indio_dev);
	unsigned int source;

	regmap_read(st->axi_regs, AXI_REG_CHAN_CNTRL_7_CH0, &source);

	switch (source) {
	default:
	case AXI_SEL_SRC_ADC:
		source = AD3552R_ADC;
		break;
	case AXI_SEL_SRC_DMA:
		source = AD3552R_DMA;
		break;
	case AXI_SEL_SRC_DDS:
		source = AD3552R_RAMP;
		break;
	}

	return source;
}

static int ad3552r_axi_set_stream_state(struct iio_dev *indio_dev,
					const struct iio_chan_spec *chan,
					unsigned int mode)
{
	struct ad3552r_axi_state *st = iio_priv(indio_dev);

	if (mode == 2) {
		st->synced_transfer = true;
		regmap_write(st->axi_regs, AXI_REG_CNTRL_1, AXI_EXT_SYNC_ARM);
		regmap_write(st->axi_regs, AXI_REG_CNTRL_2,
			     AXI_MSK_USIGN_DATA | ~AXI_MSK_SDR_DDR_N);
		regmap_update_bits(st->axi_regs, AXI_REG_CNTRL_CSTM,
				   AXI_STREAM_START, AXI_STREAM_START);
	} else if (mode == 1) {
		st->synced_transfer = false;
		regmap_write(st->axi_regs, AXI_REG_CNTRL_2,
			     AXI_MSK_USIGN_DATA | ~AXI_MSK_SDR_DDR_N);
		regmap_update_bits(st->axi_regs, AXI_REG_CNTRL_CSTM,
				   AXI_STREAM_START, AXI_STREAM_START);
	} else  {
		st->synced_transfer = false;
		regmap_update_bits(st->axi_regs, AXI_REG_CNTRL_CSTM,
				   AXI_STREAM_START, 0);
	}

	return 0;
}

static int ad3552r_axi_get_stream_state(struct iio_dev *indio_dev,
					const struct iio_chan_spec *chan)
{
	struct ad3552r_axi_state *st = iio_priv(indio_dev);
	unsigned int val;

	regmap_read(st->axi_regs, AXI_REG_CNTRL_CSTM, &val);

	if ((val & AXI_MSK_STREAM) == 2 && st->synced_transfer) {
		return 2;
	} else if ((val & AXI_MSK_STREAM) == 2) {
		return 1;
	}

	return 0;
}

static int ad3552r_axi_reset(struct ad3552r_axi_state *st)
{
	regmap_update_bits(st->axi_regs, AXI_REG_RSTN, AXI_RST, 0x00);
	regmap_update_bits(st->axi_regs, AXI_REG_RSTN, AXI_RST, AXI_RST);

	st->reset_gpio = devm_gpiod_get_optional(st->dev,
						 "reset", GPIOD_OUT_LOW);
	if (IS_ERR(st->reset_gpio))
		return PTR_ERR(st->reset_gpio);

	if (st->reset_gpio) {
		gpiod_set_value_cansleep(st->reset_gpio, 1);
		usleep_range(1, 10);
		gpiod_set_value_cansleep(st->reset_gpio, 0);
	} else {
		ad3552r_axi_spi_update_bits(st,
					    AD3552R_REG_ADDR_INTERFACE_CONFIG_A,
					    AD3552R_MASK_SOFTWARE_RESET,
					    AD3552R_MASK_SOFTWARE_RESET,
					    AXI_TFER_8BIT_SDR);
	}
	msleep_interruptible(100);

	return 0;
}

static int ad3552r_axi_setup(struct ad3552r_axi_state *st)
{
	u8 val;
	u16 id;
	int ret;

	ret = ad3552r_axi_reset(st);
	if (ret)
		return ret;

	ad3552r_axi_spi_write(st, AD3552R_REG_ADDR_SCRATCH_PAD,
			      AD3552R_SCRATCH_PAD_TEST_VAL1,
			      AXI_TFER_8BIT_SDR);
	val = ad3552r_axi_spi_read(st, AD3552R_REG_ADDR_SCRATCH_PAD,
				   AXI_TFER_8BIT_SDR);
	if (val != AD3552R_SCRATCH_PAD_TEST_VAL1) {
		dev_err(st->dev,
			"SCRATCH_PAD_TEST mismatch. Expected 0x%x, Read 0x%x\n",
			AD3552R_SCRATCH_PAD_TEST_VAL1, val);
		return -EIO;
	}

	ad3552r_axi_spi_write(st, AD3552R_REG_ADDR_SCRATCH_PAD,
			      AD3552R_SCRATCH_PAD_TEST_VAL2,
		       AXI_TFER_8BIT_SDR);
	val = ad3552r_axi_spi_read(st, AD3552R_REG_ADDR_SCRATCH_PAD,
				   AXI_TFER_8BIT_SDR);
	if (val != AD3552R_SCRATCH_PAD_TEST_VAL2) {
		dev_err(st->dev,
			"SCRATCH_PAD_TEST mismatch. Expected 0x%x, Read 0x%x\n",
			AD3552R_SCRATCH_PAD_TEST_VAL2, val);
		return -EIO;
	}

	val = ad3552r_axi_spi_read(st, AD3552R_REG_ADDR_PRODUCT_ID_L,
				   AXI_TFER_8BIT_SDR);

	id = val;
	mdelay(100);

	val = ad3552r_axi_spi_read(st, AD3552R_REG_ADDR_PRODUCT_ID_H,
				   AXI_TFER_8BIT_SDR);

	id |= val << 8;
	if (id != AD3552R_ID) {
		dev_err(st->dev, "Chip ID mismatch. Expected 0x%x, Read 0x%x\n",
			AD3552R_ID, id);
		return -ENODEV;
	}

	ad3552r_axi_spi_write(st, AD3552R_REG_ADDR_SH_REFERENCE_CONFIG,
			      AD3552R_REF_INIT,
			      AXI_TFER_8BIT_SDR);
	ad3552r_axi_spi_write(st, AD3552R_REG_ADDR_TRANSFER_REGISTER,
			      AD3552R_TRANSFER_INIT,
			      AXI_TFER_8BIT_SDR);

	regmap_write(st->axi_regs, AXI_REG_CHAN_CNTRL_7_CH0, AXI_SEL_SRC_ADC);
	regmap_write(st->axi_regs, AXI_REG_CHAN_CNTRL_7_CH1, AXI_SEL_SRC_ADC);

	return 0;
}

static const struct iio_buffer_setup_ops ad3552r_axi_buffer_setup_ops = {
	.postenable = ad3552r_axi_buffer_postenable,
	.postdisable = ad3552r_axi_buffer_postdisable,
};

static const struct iio_enum ad35525_axi_source_enum = {
	.items = input_source,
	.num_items = ARRAY_SIZE(input_source),
	.get = ad3552r_axi_get_input_source,
	.set = ad3552r_axi_set_input_source,
};

static const struct iio_enum ad35525_axi_output_enum = {
	.items = output_range,
	.num_items = ARRAY_SIZE(output_range),
	.get = ad3552r_axi_get_output_range,
	.set = ad3552r_axi_set_output_range,
};

static const struct iio_enum ad35525_axi_stream_enum = {
	.items = stream_status,
	.num_items = ARRAY_SIZE(stream_status),
	.get = ad3552r_axi_get_stream_state,
	.set = ad3552r_axi_set_stream_state,
};

static const struct iio_chan_spec_ext_info ad3552r_axi_ext_info[] = {
	IIO_ENUM("input_source", IIO_SHARED_BY_ALL, &ad35525_axi_source_enum),
	IIO_ENUM_AVAILABLE("input_source", IIO_SHARED_BY_ALL,
			   &ad35525_axi_source_enum),
	IIO_ENUM("stream_status", IIO_SHARED_BY_ALL, &ad35525_axi_stream_enum),
	IIO_ENUM_AVAILABLE("stream_status", IIO_SHARED_BY_ALL,
			   &ad35525_axi_stream_enum),
	IIO_ENUM("output_range", IIO_SHARED_BY_ALL, &ad35525_axi_output_enum),
	IIO_ENUM_AVAILABLE("output_range", IIO_SHARED_BY_ALL,
			   &ad35525_axi_output_enum),
	{},
};

#define AD3552R_CHANNEL(ch) { \
	.type = IIO_VOLTAGE, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
	.info_mask_shared_by_all = (((ch) == 0) ? \
		BIT(IIO_CHAN_INFO_SAMP_FREQ) : 0), \
	.output = 1, \
	.indexed = 1, \
	.channel = (ch), \
	.scan_index = (ch), \
	.scan_type = { \
		.sign = 'u', \
		.realbits = 16, \
		.storagebits = 16, \
		.shift = 0, \
		.endianness = IIO_BE, \
	}, \
	.ext_info = ad3552r_axi_ext_info, \
}

static struct iio_chan_spec ad3552r_axi_channels[] = {
	AD3552R_CHANNEL(0),
	AD3552R_CHANNEL(1),
};

static const struct iio_info ad3552r_axi_info = {
	.read_raw = &ad3552r_axi_read_raw,
	.write_raw = &ad3552r_axi_write_raw,
	.debugfs_reg_access = &ad3552r_axi_reg_access,
	.update_scan_mode = ad3552r_axi_update_scan_mode,
};

static int ad3552r_axi_probe(struct platform_device *pdev)
{
	struct ad3552r_axi_state *st;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	st->back = devm_iio_backend_get(&pdev->dev, NULL);
	if (IS_ERR(st->back))
		return PTR_ERR(st->back);

	ret = devm_iio_backend_request_buffer(&pdev->dev, st->back, indio_dev);
	if (ret)
		return ret;

	st->axi_regs = iio_backend_get_regmap(st->back);
	if (IS_ERR(st->axi_regs))
		return PTR_ERR(st->axi_regs);

	st->ref_clk = devm_clk_get_prepared(&pdev->dev, NULL);
	if (IS_ERR(st->ref_clk))
		return PTR_ERR(st->ref_clk);

	ret = devm_iio_backend_enable(&pdev->dev, st->back);
	if (ret)
		return ret;

	dev_info(&pdev->dev, "using AXI DAC backend\n");

	st->dev = &pdev->dev;

	indio_dev->name = pdev->dev.of_node->name;
	indio_dev->modes = INDIO_BUFFER_HARDWARE;
	indio_dev->setup_ops = &ad3552r_axi_buffer_setup_ops;
	indio_dev->channels = ad3552r_axi_channels;
	indio_dev->num_channels = ARRAY_SIZE(ad3552r_axi_channels);
	indio_dev->info = &ad3552r_axi_info;

	ret = ad3552r_axi_setup(st);
	if (ret)
		return ret;

	return devm_iio_device_register(&pdev->dev, indio_dev);
}

static const struct of_device_id ad3552r_axi_of_id[] = {
	{ .compatible = "adi,ad3552r-axi" },
	{}
};
MODULE_DEVICE_TABLE(of, ad3552r_axi_of_id);

static struct platform_driver axi_ad3552r_driver = {
	.driver = {
		.name = "ad3552r-axi",
		.owner = THIS_MODULE,
		.of_match_table = ad3552r_axi_of_id,
	},
	.probe = ad3552r_axi_probe,
};
module_platform_driver(axi_ad3552r_driver);

MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_AUTHOR("Angelo Dureghello <adueghello@baylibre.com>");
MODULE_DESCRIPTION("AD3552R Driver - AXI IP version");
MODULE_LICENSE("GPL");
