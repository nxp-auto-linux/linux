// SPDX-License-Identifier: GPL-2.0
/*
 * NXP S32 SAR-ADC driver (adapted from Freescale Vybrid vf610 ADC
 * driver by Fugang Duan <B38611@freescale.com>)
 *
 * Copyright 2013 Freescale Semiconductor, Inc.
 * Copyright 2017, 2020-2023 NXP
 */

#include <linux/circ_buf.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/regulator/consumer.h>
#include <linux/of_platform.h>
#include <linux/err.h>
#include <linux/iopoll.h>
#include <linux/version.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>

#define INDIO_UNINITIALIZED	0

/* This will be the driver name the kernel reports */
#define DRIVER_NAME "s32-adc"

/* S32 ADC registers */
#define REG_ADC_MCR			0x00
#define REG_ADC_MSR			0x04
#define REG_ADC_ISR			0x10
#define REG_ADC_CEOCFR(g)	(0x14 + ((g) << 2))
#define REG_ADC_IMR			0x20
#define REG_ADC_CIMR(g)		(0x24 + ((g) << 2))
#define REG_ADC_DMAE		0x40
#define REG_ADC_DMAR(g)		(0x44 + ((g) << 2))
#define REG_ADC_CTR(g)		(0x94 + ((g) << 2))
#define REG_ADC_NCMR(g)		(0xa4 + ((g) << 2))
#define REG_ADC_CDR(c)		(0x100 + ((c) << 2))
#define REG_ADC_CALSTAT		0x39c

/* Main Configuration Register field define */
#define ADC_PWDN		BIT(0)
#define ADC_ACKO		BIT(5)
#define ADC_ADCLKSEL		BIT(8)
#define ADC_TSAMP_MASK		GENMASK(10, 9)
#define ADC_NRSMPL_32		BIT(11)
#define ADC_NRSMPL_128		BIT(12)
#define ADC_NRSMPL_512		GENMASK(12, 11)
#define ADC_NRSMPL_MASK		GENMASK(12, 11)
#define ADC_AVGEN		BIT(13)
#define ADC_CALSTART		BIT(14)
#define ADC_NSTART		BIT(24)
#define ADC_MODE		BIT(29)
#define ADC_OWREN		BIT(31)

#define DMAEN			BIT(0)

/* Main Status Register field define */
#define ADC_CALBUSY		BIT(29)
#define ADC_CALFAIL		BIT(30)

/* Interrupt Status Register field define */
#define ADC_ECH			BIT(1)

/* Channel Pending Register field define */
#define ADC_EOC_CH(c)		BIT((c) % 32)


/* Channel Interrupt Mask Register field define */
#define ADC_CIM(c)		BIT((c) % 32)
#define ADC_CIM_MASK		GENMASK(7, 0)

/* Conversion Timing Register field define */
#define ADC_INPSAMP_MIN		8
#define ADC_INPSAMP_MAX		0xFF

/* Normal Conversion Mask Register field define */
#define ADC_CH(c)			BIT((c) % 32)
#define ADC_CH_MASK			GENMASK(7, 0)

/* Channel Data Register field define */
#define ADC_CDATA_MASK			GENMASK(11, 0)
#define ADC_VALID			BIT(19)

/* Calibration Status Register field define */
#define ADC_TEST_RESULT(x)		((x) >> 16)
#define ADC_STAT_n(x, n)		((x) & 1 << ((n) - 1))

/* Other field define */
#define ADC_CLK_FREQ_40MHz		40000000
#define ADC_CLK_FREQ_80MHz		80000000
#define ADC_CLK_FREQ_160MHz		160000000
#define ADC_CONV_TIMEOUT		100 /* ms */
#define ADC_CAL_TIMEOUT				100000 /* us */
#define ADC_WAIT				2000   /* us */
#define ADC_NUM_CAL_STEPS		14
#define ADC_NUM_GROUPS			2
#define ADC_RESOLUTION			12

/* Duration of conversion phases */
#define ADC_TPT			2
#define ADC_CT			((ADC_RESOLUTION + 2) * 4)
#define ADC_DP			2

#define BUFFER_ECH_NUM_OK	2
#define ADC_NUM_CHANNELS	8

#define ADC_IIO_BUFF_SZ		(ADC_NUM_CHANNELS + (sizeof(u64) / sizeof(u16)))

#define ADC_DMA_SAMPLE_SZ	DMA_SLAVE_BUSWIDTH_4_BYTES
#define ADC_DMA_BUFF_SZ		(PAGE_SIZE * ADC_DMA_SAMPLE_SZ)
#define ADC_DMA_SAMPLE_CNT	(ADC_DMA_BUFF_SZ / ADC_DMA_SAMPLE_SZ)

enum freq_sel {
	ADC_BUSCLK_EQUAL,
	ADC_BUSCLK_HALF,
	ADC_BUSCLK_FOURTH,
};

enum average_sel {
	ADC_SAMPLE_16,
	ADC_SAMPLE_32,
	ADC_SAMPLE_128,
	ADC_SAMPLE_512,
};

struct s32cc_adc_feature {
	enum freq_sel	freq_sel;

	int	sampling_duration[ADC_NUM_GROUPS];
	int	sample_num;

	bool	auto_clk_off;
	bool	calibration;
	bool	ovwren;
};

struct s32cc_adc {
	struct device *dev;
	void __iomem *regs;
	phys_addr_t regs_phys;
	struct clk *clk;

	u16 value;
	u32 vref;
	u8 current_channel;
	u8 channels_used;
	struct s32cc_adc_feature adc_feature;

	struct completion completion;

	u16 buffer[ADC_IIO_BUFF_SZ];
	u16 buffered_chan[ADC_NUM_CHANNELS];

	struct circ_buf dma_buf;
	struct dma_chan	*dma_chan;
	struct dma_slave_config dma_config;
	dma_addr_t rx_dma_buf;
	dma_cookie_t cookie;

	/* Protect circular buffers access. */
	spinlock_t lock;
};

#define ADC_CHAN(_idx, _chan_type) {			\
	.type = (_chan_type),					\
	.indexed = 1,						\
	.channel = (_idx),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |	\
				BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = (_idx),	\
	.scan_type = {			\
		.sign = 'u',		\
		.realbits = 12,		\
		.storagebits = 16,	\
	},						\
}

static const struct iio_chan_spec s32cc_adc_iio_channels[] = {
	ADC_CHAN(0, IIO_VOLTAGE),
	ADC_CHAN(1, IIO_VOLTAGE),
	ADC_CHAN(2, IIO_VOLTAGE),
	ADC_CHAN(3, IIO_VOLTAGE),
	ADC_CHAN(4, IIO_VOLTAGE),
	ADC_CHAN(5, IIO_VOLTAGE),
	ADC_CHAN(6, IIO_VOLTAGE),
	ADC_CHAN(7, IIO_VOLTAGE),
	IIO_CHAN_SOFT_TIMESTAMP(32),
};

static inline int group_idx(unsigned int channel)
{
	return (channel < 32) ? 0 : 1;
}

static inline unsigned long s32cc_adc_clk_rate(struct s32cc_adc *info)
{
	unsigned long ret = clk_get_rate(info->clk);
	struct s32cc_adc_feature *adc_feature = &info->adc_feature;

	if (adc_feature->freq_sel == ADC_BUSCLK_HALF)
		ret >>= 1;
	else if (adc_feature->freq_sel == ADC_BUSCLK_FOURTH)
		ret >>= 2;

	return ret;
}

static inline void s32cc_adc_cfg_init(struct s32cc_adc *info)
{
	struct s32cc_adc_feature *adc_feature = &info->adc_feature;

	/* set default Configuration for ADC controller */
	adc_feature->freq_sel = ADC_BUSCLK_EQUAL;

	adc_feature->calibration = true;
	adc_feature->ovwren = false;

	adc_feature->sampling_duration[0] =
		adc_feature->sampling_duration[1] = 20;
	adc_feature->sample_num = ADC_SAMPLE_512;
}

static void s32cc_adc_disable_channels(struct s32cc_adc *info, int iio_mode)
{
	int i;

	for (i = 0; i < ADC_NUM_GROUPS; i++) {
		writel(0, info->regs + REG_ADC_CIMR(i));
		writel(0, info->regs + REG_ADC_NCMR(i));
		writel(ADC_CH_MASK, info->regs + REG_ADC_CEOCFR(i));

		if (iio_mode == INDIO_BUFFER_SOFTWARE)
			writel(0, info->regs + REG_ADC_DMAR(i));
	}
}

static void s32cc_adc_irq_cfg(struct s32cc_adc *info, bool enable)
{
	if (enable)
		writel(ADC_ECH, info->regs + REG_ADC_IMR);
	else
		writel(0, info->regs + REG_ADC_IMR);
}

static void s32cc_adc_cfg_post_set(struct s32cc_adc *info)
{
	struct s32cc_adc_feature *adc_feature = &info->adc_feature;
	int mcr_data = 0;

	/* auto-clock-off mode enable */
	if (adc_feature->auto_clk_off)
		mcr_data |= ADC_ACKO;

	/* data overwrite enable */
	if (adc_feature->ovwren)
		mcr_data |= ADC_OWREN;

	writel(mcr_data, info->regs + REG_ADC_MCR);
}

static void s32cc_adc_calibration(struct s32cc_adc *info)
{
	struct s32cc_adc_feature *adc_feature = &info->adc_feature;
	int mcr_data, msr_data, calstat_data;
	int step;
	unsigned long clk_rate;

	if (!info->adc_feature.calibration)
		return;

	mcr_data = readl(info->regs + REG_ADC_MCR);

	/* default sample period (22 cycles of ADC clk) */
	mcr_data &= ~ADC_TSAMP_MASK;

	/* update hardware average selection */
	mcr_data |= ADC_AVGEN;
	mcr_data &= ~ADC_NRSMPL_MASK;
	switch (adc_feature->sample_num) {
	case ADC_SAMPLE_16:
		break;
	case ADC_SAMPLE_32:
		mcr_data |= ADC_NRSMPL_32;
		break;
	case ADC_SAMPLE_128:
		mcr_data |= ADC_NRSMPL_128;
		break;
	case ADC_SAMPLE_512:
		mcr_data |= ADC_NRSMPL_512;
		break;
	default:
		dev_err(info->dev,
			"error hardware sample average select\n");
	}

	/* set AD_clk frequency to 40 MHz */
	mcr_data &= ~ADC_ADCLKSEL;
	clk_rate = clk_get_rate(info->clk);
	if (clk_rate == ADC_CLK_FREQ_40MHz) {
		/* AD_CLK frequency is equal to bus clock frequency */
		mcr_data |= ADC_ADCLKSEL;
	} else if (clk_rate != ADC_CLK_FREQ_80MHz) {
		dev_err(info->dev, "Bad bus clock frequency\n");
	}

	mcr_data &= ~ADC_PWDN;
	writel(mcr_data, info->regs + REG_ADC_MCR);

	mcr_data |= ADC_CALSTART;
	writel(mcr_data, info->regs + REG_ADC_MCR);

	if (read_poll_timeout(readl, msr_data, !(msr_data & ADC_CALBUSY),
			      ADC_WAIT, ADC_CAL_TIMEOUT, true, info->regs + REG_ADC_MSR))
		dev_err(info->dev, "SAR ADC Calibration failed\n");

	if (msr_data & ADC_CALBUSY) {
		dev_err(info->dev, "Timeout for adc calibration\n");
	} else if (msr_data & ADC_CALFAIL) {
		dev_err(info->dev, "ADC calibration failed\nStep status:\n");
		calstat_data = readl(info->regs + REG_ADC_CALSTAT);
		for (step = 1; step <= ADC_NUM_CAL_STEPS; step++)
			dev_err(info->dev, "Step %d: %s\n", step,
				ADC_STAT_n(calstat_data, step) ?
				"failed" : "passed");
		dev_err(info->dev, "Result for the last failed test: %d\n",
			ADC_TEST_RESULT(calstat_data));
	}

	info->adc_feature.calibration = false;
}

static void s32cc_adc_sample_set(struct s32cc_adc *info)
{
	struct s32cc_adc_feature *adc_feature = &info->adc_feature;
	enum freq_sel freq_sel = adc_feature->freq_sel;
	int mcr_data, ctr_data = 0, group;

	/* configure AD_clk frequency */
	mcr_data = readl(info->regs + REG_ADC_MCR);
	mcr_data |= ADC_PWDN;
	writel(mcr_data, info->regs + REG_ADC_MCR);

	/* restore preferred AD_clk frequency */
	if (freq_sel == ADC_BUSCLK_EQUAL)
		mcr_data |= ADC_ADCLKSEL;
	else if (freq_sel != ADC_BUSCLK_HALF)
		dev_err(info->dev, "error frequency selection\n");

	writel(mcr_data, info->regs + REG_ADC_MCR);

	mcr_data &= ~ADC_PWDN;

	writel(mcr_data, info->regs + REG_ADC_MCR);

	/* sampling phase duration set */
	for (group = 0; group < ADC_NUM_GROUPS; group++) {
		ctr_data |= min(adc_feature->sampling_duration[group],
			ADC_INPSAMP_MAX);
		writel(ctr_data, info->regs + REG_ADC_CTR(group));
	}
}

static void s32cc_adc_hw_init(struct s32cc_adc *info)
{
	/* CFG: Feature set */
	s32cc_adc_cfg_post_set(info);

	/* adc calibration */
	s32cc_adc_calibration(info);

	/* sampling speed set */
	s32cc_adc_sample_set(info);
}

static void s32cc_adc_read_notify(struct s32cc_adc *info)
{
	int group;

	for (group = 0; group < ADC_NUM_GROUPS; group++)
		writel(ADC_CH_MASK, info->regs + REG_ADC_CEOCFR(group));
}

static int s32cc_adc_read_data(struct s32cc_adc *info,
			       unsigned int chan)
{
	u32 ceocfr_data, cdr_data;
	int group;

	group = group_idx(chan);
	ceocfr_data = readl(info->regs + REG_ADC_CEOCFR(group));
	if (!(ceocfr_data & ADC_EOC_CH(chan)))
		return -EIO;

	cdr_data = readl(info->regs + REG_ADC_CDR(chan));
	if (!(cdr_data & ADC_VALID)) {
		dev_err(info->dev, "error invalid data\n");
		return -EIO;
	}

	return cdr_data & ADC_CDATA_MASK;
}

static void s32cc_adc_isr_buffer(struct iio_dev *indio_dev)
{
	struct s32cc_adc *info = iio_priv(indio_dev);
	int i, ret;

	for (i = 0; i < info->channels_used; i++) {
		ret = s32cc_adc_read_data(info, info->buffered_chan[i]);
		if (ret < 0) {
			s32cc_adc_read_notify(info);
			return;
		}

		info->buffer[i] = ret;
	}

	s32cc_adc_read_notify(info);
	iio_push_to_buffers_with_timestamp(indio_dev,
					   info->buffer,
					   iio_get_time_ns(indio_dev));
	iio_trigger_notify_done(indio_dev->trig);
}

static void s32cc_adc_isr_read_raw(struct iio_dev *indio_dev)
{
	struct s32cc_adc *info = iio_priv(indio_dev);
	int ret;

	ret = s32cc_adc_read_data(info, info->current_channel);
	s32cc_adc_read_notify(info);
	if (ret < 0)
		return;

	info->value = ret;
	complete(&info->completion);
}

static irqreturn_t s32cc_adc_isr(int irq, void *dev_id)
{
	struct iio_dev *indio_dev = (struct iio_dev *)dev_id;
	struct s32cc_adc *info = iio_priv(indio_dev);
	int isr_data;

	isr_data = readl(info->regs + REG_ADC_ISR);
	if (!(isr_data & ADC_ECH))
		return IRQ_NONE;

	if (iio_buffer_enabled(indio_dev))
		s32cc_adc_isr_buffer(indio_dev);
	else
		s32cc_adc_isr_read_raw(indio_dev);

	writel(ADC_ECH, info->regs + REG_ADC_ISR);

	return IRQ_HANDLED;
}

static void s32cc_adc_chan_enable(struct s32cc_adc *info,
				  unsigned int chan, bool enable)
{
	u32 ncmr_data, cimr_data;
	int group;

	group = group_idx(chan);
	ncmr_data = readl(info->regs + REG_ADC_NCMR(group));
	cimr_data = readl(info->regs + REG_ADC_CIMR(group));
	if (enable) {
		ncmr_data |= ADC_CH(chan);
		cimr_data |= ADC_CH(chan);
	} else {
		ncmr_data &= ~ADC_CH(chan);
		cimr_data &= ~ADC_CH(chan);
	}

	writel(ncmr_data, info->regs + REG_ADC_NCMR(group));
	writel(cimr_data, info->regs + REG_ADC_CIMR(group));
}

static void s32cc_adc_dma_chan_enable(struct s32cc_adc *info,
				      unsigned int chan, bool enable)
{
	u32 dmar_data;
	int group;

	group = group_idx(chan);
	dmar_data = readl(info->regs + REG_ADC_DMAR(group));
	if (enable)
		dmar_data |= BIT(chan);
	else
		dmar_data &= ~BIT(chan);

	writel(dmar_data, info->regs + REG_ADC_DMAR(group));
}

static void s32cc_adc_dma_cfg(struct s32cc_adc *info, bool enable)
{
	u32 dmae_data;

	dmae_data = readl(info->regs + REG_ADC_DMAE);
	if (enable)
		dmae_data |= DMAEN;
	else
		dmae_data &= ~DMAEN;
	writel(dmae_data, info->regs + REG_ADC_DMAE);
}

static void s32cc_adc_enable(struct s32cc_adc *info, bool enable)
{
	u32 mcr_data;

	mcr_data = readl(info->regs + REG_ADC_MCR);
	if (enable) {
		mcr_data &= ~ADC_PWDN;
	} else {
		mcr_data |= ADC_PWDN;
		/* Stop any conversion. */
		mcr_data &= ~ADC_NSTART;
	}

	writel(mcr_data, info->regs + REG_ADC_MCR);

	/* Ensure there are at least three cycles between the
	 * configuration of NCMR and the setting of NSTART
	 */
	if (enable)
		ndelay(div64_u64(NSEC_PER_SEC,
				 s32cc_adc_clk_rate(info)) * 3U);
}

static int s32cc_adc_start_conversion(struct s32cc_adc *info, int currentmode)
{
	u32 mcr_data;

	mcr_data = readl(info->regs + REG_ADC_MCR);
	mcr_data |= ADC_NSTART;

	switch (currentmode) {
	case INDIO_UNINITIALIZED:
	case INDIO_DIRECT_MODE:
	case INDIO_BUFFER_TRIGGERED:
		mcr_data &= ~ADC_MODE;
		break;
	case INDIO_BUFFER_SOFTWARE:
		mcr_data |= ADC_MODE;
		break;
	default:
		dev_warn(info->dev, "invalid IIO mode %d\n", currentmode);
		return -EINVAL;
	}
	writel(mcr_data, info->regs + REG_ADC_MCR);

	return 0;
}

static int s32cc_read_raw(struct iio_dev *indio_dev,
			  struct iio_chan_spec const *chan,
			  int *val,
			  int *val2,
			  long mask)
{
	struct s32cc_adc *info = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&indio_dev->mlock);
		if (iio_buffer_enabled(indio_dev)) {
			ret = -EBUSY;
			goto out_iio_chan_info_raw;
		}

		info->current_channel = chan->channel;
		s32cc_adc_chan_enable(info, chan->channel, true);
		s32cc_adc_irq_cfg(info, true);
		s32cc_adc_enable(info, true);

		reinit_completion(&info->completion);
		ret = s32cc_adc_start_conversion(info, indio_dev->currentmode);
		if (ret < 0)
			goto out_iio_chan_info_raw;

		ret = wait_for_completion_interruptible_timeout
			(&info->completion,
			msecs_to_jiffies(ADC_CONV_TIMEOUT));

		s32cc_adc_chan_enable(info, chan->channel, false);
		s32cc_adc_irq_cfg(info, false);
		s32cc_adc_enable(info, false);

		if (ret == 0) {
			ret = -ETIMEDOUT;
			goto out_iio_chan_info_raw;
		}

		if (ret < 0)
			goto out_iio_chan_info_raw;

		*val = info->value;
		ret = IIO_VAL_INT;

out_iio_chan_info_raw:
		mutex_unlock(&indio_dev->mlock);
		return ret;

	case IIO_CHAN_INFO_SCALE:
		*val = info->vref;
		*val2 = ADC_RESOLUTION;
		return IIO_VAL_FRACTIONAL_LOG2;

	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = s32cc_adc_clk_rate(info) / (ADC_TPT +
				info->adc_feature.sampling_duration[0] +
				ADC_CT +
				ADC_DP);
		return IIO_VAL_INT;

	default:
		break;
	}

	return -EINVAL;
}

static int s32cc_write_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int val,
			   int val2,
			   long mask)
{
	struct s32cc_adc *info = iio_priv(indio_dev);
	int samp_time;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		samp_time = s32cc_adc_clk_rate(info) / val - (ADC_TPT +
				ADC_CT +
				ADC_DP);
		samp_time = max(samp_time, ADC_INPSAMP_MIN);
		samp_time = min(samp_time, ADC_INPSAMP_MAX);

		info->adc_feature.sampling_duration[0] = samp_time;
		s32cc_adc_sample_set(info);
		return 0;

	default:
		break;
	}

	return -EINVAL;
}

static void s32cc_adc_dma_cb(void *data)
{
	struct s32cc_adc *info = iio_priv((struct iio_dev *)data);
	struct iio_dev *indio_dev = data;
	struct dma_tx_state state;
	struct circ_buf *dma_buf;
	unsigned long flags;
	u32 *dma_samples;
	s64 timestamp;
	int idx, ret;

	dma_buf = &info->dma_buf;
	dma_samples = (u32 *)dma_buf->buf;

	spin_lock_irqsave(&info->lock, flags);
	dmaengine_tx_status(info->dma_chan,
			    info->cookie, &state);
	dma_sync_single_for_cpu(&indio_dev->dev, info->rx_dma_buf,
				ADC_DMA_BUFF_SZ, DMA_FROM_DEVICE);
	/* Current head position */
	dma_buf->head = (ADC_DMA_BUFF_SZ - state.residue) /
			ADC_DMA_SAMPLE_SZ;
	/* Make sure that head is multiple of info->channels_used */
	dma_buf->head -= dma_buf->head % info->channels_used;

	/* dma_buf->tail != dma_buf->head condition will become false
	 * because dma_buf->tail will be incremented with 1.
	 */
	while (dma_buf->tail != dma_buf->head) {
		idx = dma_buf->tail % info->channels_used;
		info->buffer[idx] = dma_samples[dma_buf->tail];
		dma_buf->tail = (dma_buf->tail + 1) % ADC_DMA_SAMPLE_CNT;
		if (idx != info->channels_used - 1)
			continue;

		/* iio_push_to_buffers_with_timestamp should not be called
		 * with dma_samples as parameter. The samples will be smashed
		 * if timestamp is enabled.
		 */
		timestamp = iio_get_time_ns(indio_dev);
		ret = iio_push_to_buffers_with_timestamp(indio_dev,
							 info->buffer,
							 timestamp);
		if (ret < 0 && ret != -EBUSY)
			dev_err_ratelimited(&indio_dev->dev,
					    "failed to push iio buffer: %d",
					    ret);
	}

	dma_buf->tail = dma_buf->head;
	dma_sync_single_for_device(&indio_dev->dev, info->rx_dma_buf,
				   ADC_DMA_BUFF_SZ, DMA_FROM_DEVICE);
	spin_unlock_irqrestore(&info->lock, flags);
}

static int s32cc_adc_start_cyclic_dma(struct iio_dev *indio_dev)
{
	struct s32cc_adc *info = iio_priv(indio_dev);
	struct dma_slave_config *config = &info->dma_config;
	struct dma_async_tx_descriptor *desc;
	int ret;

	info->dma_buf.head = 0;
	info->dma_buf.tail = 0;

	config->src_addr = info->regs_phys +
		REG_ADC_CDR(info->buffered_chan[0]);
	config->src_port_window_size = info->channels_used;
	ret = dmaengine_slave_config(info->dma_chan, config);
	if (ret < 0) {
		dev_err(info->dev, "failed to configure DMA slave\n");
		return ret;
	}

	desc = dmaengine_prep_dma_cyclic(info->dma_chan,
					 info->rx_dma_buf,
					 ADC_DMA_BUFF_SZ,
					 ADC_DMA_BUFF_SZ / 2,
					 DMA_DEV_TO_MEM, DMA_PREP_INTERRUPT);
	if (!desc) {
		dev_err(info->dev,
			"failed to prepare DMA cyclic transaction\n");
		return -EINVAL;
	}

	desc->callback = s32cc_adc_dma_cb;
	desc->callback_param = indio_dev;
	info->cookie = dmaengine_submit(desc);
	ret = dma_submit_error(info->cookie);
	if (ret) {
		dev_err(&indio_dev->dev, "failed to submit DMA cyclic\n");
		dmaengine_terminate_async(info->dma_chan);
		return ret;
	}

	dma_async_issue_pending(info->dma_chan);

	return 0;
}

static int s32cc_adc_buffer_postenable(struct iio_dev *indio_dev)
{
	struct s32cc_adc *info = iio_priv(indio_dev);
	int iio_mode = indio_dev->currentmode;
	unsigned long channel;
	int ret;

	info->channels_used = 0;

	for_each_set_bit(channel, indio_dev->active_scan_mask,
			 ADC_NUM_CHANNELS) {
		info->buffered_chan[info->channels_used++] = channel;
		s32cc_adc_chan_enable(info, channel, true);

		if (iio_mode == INDIO_BUFFER_SOFTWARE)
			s32cc_adc_dma_chan_enable(info, channel, true);
	}

	s32cc_adc_enable(info, true);
	if (iio_mode == INDIO_BUFFER_SOFTWARE) {
		s32cc_adc_dma_cfg(info, true);
		ret = s32cc_adc_start_cyclic_dma(indio_dev);
		if (ret)
			goto buffer_postenable_clean_up;
		return s32cc_adc_start_conversion(info, iio_mode);
	}

	s32cc_adc_irq_cfg(info, true);
	return 0;

buffer_postenable_clean_up:
	s32cc_adc_enable(info, false);
	s32cc_adc_dma_cfg(info, false);
	s32cc_adc_disable_channels(info, iio_mode);

	return ret;
}

static int s32cc_adc_buffer_predisable(struct iio_dev *indio_dev)
{
	struct s32cc_adc *info = iio_priv(indio_dev);

	s32cc_adc_enable(info, false);

	if (indio_dev->currentmode == INDIO_BUFFER_SOFTWARE) {
		/* The ADC DMAEN bit should be cleared before DMA transaction
		 * is canceled.
		 */
		s32cc_adc_dma_cfg(info, false);
		dmaengine_terminate_sync(info->dma_chan);
	} else {
		s32cc_adc_irq_cfg(info, false);
	}

	s32cc_adc_disable_channels(info, indio_dev->currentmode);

	return 0;
}

static int s32cc_adc_preenable(struct iio_dev *indio_dev)
{
	unsigned long first_bit, set_bits = *indio_dev->active_scan_mask;

	/* SAR_ADC permits any combination of the
	 * available channels in software triggered mode.
	 */
	if (indio_dev->currentmode == INDIO_BUFFER_TRIGGERED)
		return 0;

	first_bit = find_first_bit(indio_dev->active_scan_mask,
				   ADC_NUM_CHANNELS);
	/* Assumption: 8 bits max */
	set_bits = (set_bits >> first_bit) & GENMASK(7, 0);
	switch (set_bits) {
	case BIT(0):
		/* one bit */
	case GENMASK(1, 0):
		/* two bits */
	case GENMASK(3, 0):
		/* four consecutive bits */
	case GENMASK(7, 0):
		/* eight consecutive bits */
		return 0;
	default:
		dev_err(&indio_dev->dev, "Invalid active scan mask\n");
		return -EINVAL;
	}
}

static irqreturn_t s32cc_adc_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct s32cc_adc *info = iio_priv(indio_dev);
	int ret;

	ret = s32cc_adc_start_conversion(info, indio_dev->currentmode);
	if (ret < 0)
		return IRQ_NONE;

	return IRQ_HANDLED;
}

static const struct iio_buffer_setup_ops iio_triggered_buffer_setup_ops = {
	.preenable = s32cc_adc_preenable,
	.postenable = &s32cc_adc_buffer_postenable,
	.predisable = &s32cc_adc_buffer_predisable,
};

static const struct iio_info s32cc_adc_iio_info = {
	.read_raw = &s32cc_read_raw,
	.write_raw = &s32cc_write_raw,
};

static const struct of_device_id s32cc_adc_match[] = {
	{ .compatible = "nxp,s32cc-adc", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, s32cc_adc_match);

static int s32cc_adc_dma_probe(struct s32cc_adc *info)
{
	struct device *dev_dma;
	u8 *rx_buf;
	int ret;

	info->dma_chan = dma_request_chan(info->dev, "rx");
	if (IS_ERR(info->dma_chan))  {
		dev_err(info->dev, "can't get DMA channel\n");
		return PTR_ERR(info->dma_chan);
	}

	dev_dma = info->dma_chan->device->dev;
	rx_buf = dma_alloc_coherent(dev_dma, ADC_DMA_BUFF_SZ,
				    &info->rx_dma_buf,
				    GFP_KERNEL);
	if (!rx_buf) {
		dev_err(info->dev, "can't allocate coherent DMA area\n");
		ret = -ENOMEM;
		goto dma_release;
	}
	info->dma_buf.buf = rx_buf;

	info->dma_config.direction = DMA_DEV_TO_MEM;
	info->dma_config.src_addr_width = ADC_DMA_SAMPLE_SZ;
	info->dma_config.src_maxburst = 1;
	info->dma_config.dst_maxburst = 1;

	dev_info(info->dev, "using %s for ADC DMA transfers\n",
		 dma_chan_name(info->dma_chan));

	return 0;

dma_release:
	dma_release_channel(info->dma_chan);

	return ret;
}

static int s32cc_adc_probe(struct platform_device *pdev)
{
	struct s32cc_adc *info;
	struct iio_dev *indio_dev;
	struct resource *mem;
	int irq;
	int ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(struct s32cc_adc));
	if (!indio_dev) {
		dev_err(&pdev->dev, "Failed allocating iio device\n");
		return -ENOMEM;
	}

	info = iio_priv(indio_dev);
	info->dev = &pdev->dev;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	info->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(info->regs))
		return PTR_ERR(info->regs);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no irq resource?\n");
		return irq;
	}

	ret = devm_request_irq(info->dev, irq, s32cc_adc_isr, 0,
			       dev_name(&pdev->dev), indio_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed requesting irq, irq = %d\n", irq);
		return ret;
	}

	info->regs_phys = mem->start;
	spin_lock_init(&info->lock);

	info->clk = devm_clk_get(&pdev->dev, "adc");
	if (IS_ERR(info->clk)) {
		dev_err(&pdev->dev, "failed getting clock, err = %ld\n",
			PTR_ERR(info->clk));
		return PTR_ERR(info->clk);
	}

	if (!pdev->dev.of_node)
		return -EINVAL;

	ret = of_property_read_u32(pdev->dev.of_node, "vref", &info->vref);
	if (ret) {
		dev_err(&pdev->dev, "no vref property in device tree\n");
		return ret;
	}

	platform_set_drvdata(pdev, indio_dev);

	init_completion(&info->completion);

	indio_dev->name = dev_name(&pdev->dev);
	indio_dev->dev.of_node = pdev->dev.of_node;
	indio_dev->info = &s32cc_adc_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_SOFTWARE;
	indio_dev->channels = s32cc_adc_iio_channels;
	indio_dev->num_channels = ARRAY_SIZE(s32cc_adc_iio_channels);

	memset(info->buffer, 0, sizeof(info->buffer));
	memset(info->buffered_chan, 0, sizeof(info->buffered_chan));

	ret = clk_prepare_enable(info->clk);
	if (ret) {
		dev_err(&pdev->dev,
			"Could not prepare or enable the clock.\n");
		return ret;
	}

	s32cc_adc_cfg_init(info);
	s32cc_adc_hw_init(info);

	ret = s32cc_adc_dma_probe(info);
	if (ret)
		goto s32cc_adc_clock_disable;

	ret = devm_iio_triggered_buffer_setup(&pdev->dev, indio_dev,
					      &iio_pollfunc_store_time,
					      &s32cc_adc_trigger_handler,
					      &iio_triggered_buffer_setup_ops);
	if (ret < 0) {
		dev_err(&pdev->dev, "Couldn't initialise the buffer\n");
		goto s32cc_adc_dma_clean_up;
	}

	ret = iio_device_register(indio_dev);
	if (ret) {
		dev_err(&pdev->dev, "Couldn't register the device.\n");
		goto s32cc_adc_dma_clean_up;
	}

	dev_info(&pdev->dev, "Device initialized successfully.\n");

	return 0;

s32cc_adc_dma_clean_up:
	dma_free_coherent(info->dma_chan->device->dev, ADC_DMA_BUFF_SZ,
			  info->dma_buf.buf, info->rx_dma_buf);
	dma_release_channel(info->dma_chan);
s32cc_adc_clock_disable:
	clk_disable_unprepare(info->clk);

	return ret;
}

static int s32cc_adc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct s32cc_adc *info = iio_priv(indio_dev);

	s32cc_adc_enable(info, false);
	dmaengine_terminate_sync(info->dma_chan);
	dma_free_coherent(info->dma_chan->device->dev, ADC_DMA_BUFF_SZ,
			  info->dma_buf.buf, info->rx_dma_buf);
	dma_release_channel(info->dma_chan);
	iio_device_unregister(indio_dev);
	clk_disable_unprepare(info->clk);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int s32cc_adc_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct s32cc_adc *info = iio_priv(indio_dev);
	int mcr_data;

	/* ADC controller and analog part enter to stop mode */
	mcr_data = readl(info->regs + REG_ADC_MCR);
	mcr_data |= ADC_PWDN;
	writel(mcr_data, info->regs + REG_ADC_MCR);

	clk_disable_unprepare(info->clk);

	return 0;
}

static int s32cc_adc_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct s32cc_adc *info = iio_priv(indio_dev);
	int ret;

	ret = clk_prepare_enable(info->clk);
	if (ret)
		return ret;

	s32cc_adc_hw_init(info);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(s32cc_adc_pm_ops, s32cc_adc_suspend, s32cc_adc_resume);

static struct platform_driver s32cc_adc_driver = {
	.probe          = s32cc_adc_probe,
	.remove         = s32cc_adc_remove,
	.driver         = {
		.name   = DRIVER_NAME,
		.of_match_table = s32cc_adc_match,
		.pm     = &s32cc_adc_pm_ops,
	},
};

module_platform_driver(s32cc_adc_driver);

MODULE_AUTHOR("NXP");
MODULE_DESCRIPTION("NXP S32CC SAR-ADC driver");
MODULE_LICENSE("GPL v2");
