/*
 * NXP S32 SAR-ADC driver (adapted from Freescale Vybrid vf610 ADC
 * driver by Fugang Duan <B38611@freescale.com>)
 *
 * Copyright 2013 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 */

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
#include <linux/version.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/driver.h>

#include <linux/time64.h>

/* This will be the driver name the kernel reports */
#define DRIVER_NAME "s32-adc"

/* S32 ADC registers */
#define REG_ADC_MCR		0x00
#define REG_ADC_MSR		0x04
#define REG_ADC_ISR		0x10
#define REG_ADC_CEOCFR(g)		(0x14 + ((g) << 2))
#define REG_ADC_IMR		0X20
#define REG_ADC_CIMR(g)		(0x24 + ((g) << 2))
#define REG_ADC_CTR(g)		(0x94 + ((g) << 2))
#define REG_ADC_NCMR(g)		(0xa4 + ((g) << 2))
#define REG_ADC_CDR(c)		(0x100 + ((c) << 2))
#define REG_ADC_CALCFG(g)		(0x3a0 + ((g) << 2))
#define REG_ADC_CALSTAT		0x39c

/* Main Configuration Register field define */
#define ADC_PWDN			0x01
#define ADC_ADCLKDIV		0x10
#define ADC_ACKO			0x20
#define ADC_ADCLKSEL		0x100
#define ADC_TSAMP_MASK		0x600
#define ADC_NRSMPL_32		0X800
#define ADC_NRSMPL_128		0X1000
#define ADC_NRSMPL_512		0X1800
#define ADC_NRSMPL_MASK		0x1800
#define ADC_AVGEN			0x2000
#define ADC_CALSTART		0x4000
#define ADC_NSTART			0x1000000
#define ADC_MODE			0x20000000
#define ADC_OWREN			0x80000000

/* Main Status Register field define */
#define ADC_CALBUSY		0x20000000
#define ADC_CALFAIL		0x40000000

/* Interrupt Status Register field define */
#define ADC_ECH			0x01
#define ADC_EOC			0x02

/* Channel Pending Register field define */
#define ADC_EOC_CH(c)		(1 << (c) % 32)

/* Interrupt Mask Register field define */
#define ADC_MSKECH			0x01

/* Channel Interrupt Mask Register field define */
#define ADC_CIM(c)			(1 << (c) % 32)
#define ADC_CIM_MASK		0xFF

/* Conversion Timing Register field define */
#define ADC_INPSAMP_MIN		8
#define ADC_INPSAMP_MAX		0xFF

/* Normal Conversion Mask Register field define */
#define ADC_CH(c)			(1 << (c) % 32)
#define ADC_CH_MASK		0xFF

/* Channel Data Register field define */
#define ADC_CDATA_MASK		0xFFF
#define ADC_VALID			0x80000

/* Calibration Status Register field define */
#define ADC_TEST_RESULT(x)		((x) >> 16)
#define ADC_STAT_n(x, n)		((x) & 1 << ((n) - 1))

/* Other field define */
#define ADC_CLK_FREQ_40MHz		40000000
#define ADC_CLK_FREQ_80MHz		80000000
#define ADC_CLK_FREQ_160MHz	160000000
#define ADC_CONV_TIMEOUT		100 /* ms */
#define ADC_CAL_TIMEOUT		100 /* ms */
#define ADC_WAIT			2   /* ms */
#define ADC_NSEC_PER_SEC		1000000000
#define ADC_NUM_CAL_STEPS		14
#define ADC_NUM_GROUPS		2
#define ADC_RESOLUTION		12

/* Duration of conversion phases */
#define ADC_TPT			2
#define ADC_CT			((ADC_RESOLUTION + 2) * 4)
#define ADC_DP			2

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

struct s32_adc_feature {
	enum freq_sel	freq_sel;

	int	sampling_duration[ADC_NUM_GROUPS];
	int	sample_num;

	bool	auto_clk_off;
	bool	calibration;
	bool	ovwren;
};

struct s32_adc {
	struct device *dev;
	void __iomem *regs;
	struct clk *clk;

	u16 value;
	u32 vref;
	int current_channel;
	struct s32_adc_feature adc_feature;

	struct completion completion;
};

#define ADC_CHAN(_idx, _chan_type) {			\
	.type = (_chan_type),					\
	.indexed = 1,						\
	.channel = (_idx),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |	\
				BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
}

static const struct iio_chan_spec s32_adc_iio_channels[] = {
	ADC_CHAN(0, IIO_VOLTAGE),
	ADC_CHAN(1, IIO_VOLTAGE),
	ADC_CHAN(2, IIO_VOLTAGE),
	ADC_CHAN(3, IIO_VOLTAGE),
	ADC_CHAN(4, IIO_VOLTAGE),
	ADC_CHAN(5, IIO_VOLTAGE),
	ADC_CHAN(6, IIO_VOLTAGE),
	ADC_CHAN(7, IIO_VOLTAGE),
	/* sentinel */
};

static inline int group_idx(int channel)
{
	if (channel >= 0 && channel <= 7)
		return 0;
	if (channel >= 32 && channel <= 38)
		return 1;
	return -ECHRNG;
}

static inline unsigned long s32_adc_clk_rate(struct s32_adc *info)
{
	unsigned long ret = clk_get_rate(info->clk);
	struct s32_adc_feature *adc_feature = &info->adc_feature;

	if (adc_feature->freq_sel == ADC_BUSCLK_HALF)
		ret >>= 1;
	else if (adc_feature->freq_sel == ADC_BUSCLK_FOURTH)
		ret >>= 2;

	return ret;
}

static inline void s32_adc_cfg_init(struct s32_adc *info)
{
	struct s32_adc_feature *adc_feature = &info->adc_feature;

	/* set default Configuration for ADC controller */
	adc_feature->freq_sel = ADC_BUSCLK_EQUAL;

	adc_feature->calibration = true;
	adc_feature->ovwren = false;

	adc_feature->sampling_duration[0] =
		adc_feature->sampling_duration[1] = 20;
	adc_feature->sample_num = ADC_SAMPLE_512;
}

static void s32_adc_cfg_post_set(struct s32_adc *info)
{
	struct s32_adc_feature *adc_feature = &info->adc_feature;
	int mcr_data = 0, imr_data = 0;

	/* auto-clock-off mode enable */
	if (adc_feature->auto_clk_off)
		mcr_data |= ADC_ACKO;

	/* data overwrite enable */
	if (adc_feature->ovwren)
		mcr_data |= ADC_OWREN;

	writel(mcr_data, info->regs + REG_ADC_MCR);

	/* End of Conversion Chain interrupt enable */
	imr_data |= ADC_MSKECH;
	writel(imr_data, info->regs + REG_ADC_IMR);
}

static void s32_adc_calibration(struct s32_adc *info)
{
	struct s32_adc_feature *adc_feature = &info->adc_feature;
	int mcr_data, msr_data, calstat_data;
	int ms_passed, step;
	struct timespec64 tv_start, tv_current;
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
	mcr_data &= ~ADC_ADCLKSEL & ~ADC_ADCLKDIV;
	clk_rate = clk_get_rate(info->clk);
	switch (clk_rate) {
	case ADC_CLK_FREQ_40MHz:
		mcr_data |= ADC_ADCLKSEL;
		break;
	case ADC_CLK_FREQ_80MHz:
		break;
	case ADC_CLK_FREQ_160MHz:
		mcr_data |= ADC_ADCLKDIV;
		break;
	default:
		dev_err(info->dev, "Bad bus clock frequency\n");
	}

	mcr_data &= ~ADC_PWDN;
	writel(mcr_data, info->regs + REG_ADC_MCR);

	/* remove for production silicon where these values will be auto-loaded
	 * from fuses */
	writel(0x371b4fee, info->regs + REG_ADC_CALCFG(0));
	writel(0x00000000, info->regs + REG_ADC_CALCFG(1));

	mcr_data |= ADC_CALSTART;
	writel(mcr_data, info->regs + REG_ADC_MCR);

	do_gettimeofday(&tv_start);
	do {
		msleep(ADC_WAIT);
		msr_data = readl(info->regs + REG_ADC_MSR);
		do_gettimeofday(&tv_current);
		ms_passed = (tv_current.tv_sec - tv_start.tv_sec) * 1000 +
			(tv_current.tv_usec - tv_start.tv_usec) / 1000;
	} while (msr_data & ADC_CALBUSY &&
		ms_passed < ADC_CAL_TIMEOUT);

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

static void s32_adc_sample_set(struct s32_adc *info)
{
	struct s32_adc_feature *adc_feature = &info->adc_feature;
	int mcr_data, ctr_data = 0, group;

	/* configure AD_clk frequency */
	mcr_data = readl(info->regs + REG_ADC_MCR);
	mcr_data |= ADC_PWDN;
	writel(mcr_data, info->regs + REG_ADC_MCR);

	mcr_data &= ~ADC_ADCLKSEL & ~ADC_ADCLKDIV;

	switch (adc_feature->freq_sel) {
	case ADC_BUSCLK_EQUAL:
		mcr_data |= ADC_ADCLKSEL;
		break;
	case ADC_BUSCLK_HALF:
		break;
	case ADC_BUSCLK_FOURTH:
		mcr_data |= ADC_ADCLKDIV;
		break;
	default:
		dev_err(info->dev, "error frequency selection\n");
	}

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

static void s32_adc_hw_init(struct s32_adc *info)
{
	/* CFG: Feature set */
	s32_adc_cfg_post_set(info);

	/* adc calibration */
	s32_adc_calibration(info);

	/* sampling speed set */
	s32_adc_sample_set(info);
}

static irqreturn_t s32_adc_isr(int irq, void *dev_id)
{
	struct s32_adc *info = (struct s32_adc *)dev_id;
	int isr_data, ceocfr_data, cdr_data;
	int group;

	isr_data = readl(info->regs + REG_ADC_ISR);
	if (isr_data & ADC_ECH) {
		writel(ADC_ECH | ADC_EOC,
		       info->regs + REG_ADC_ISR);
		group = group_idx(info->current_channel);

		ceocfr_data = readl(info->regs + REG_ADC_CEOCFR(group));
		if (!(ceocfr_data & ADC_EOC_CH(info->current_channel)))
			return IRQ_HANDLED;

		writel(ADC_EOC_CH(info->current_channel),
		       info->regs + REG_ADC_CEOCFR(group));

		cdr_data = readl(info->regs +
			REG_ADC_CDR(info->current_channel));
		if (!(cdr_data & ADC_VALID)) {
			dev_err(info->dev, "error invalid data\n");
			return IRQ_HANDLED;
		}

		info->value = cdr_data & ADC_CDATA_MASK;
		complete(&info->completion);
	}

	return IRQ_HANDLED;
}

static int s32_read_raw(struct iio_dev *indio_dev,
			 struct iio_chan_spec const *chan,
			 int *val,
			 int *val2,
			 long mask)
{
	struct s32_adc *info = iio_priv(indio_dev);
	int group, i;
	int mcr_data, ncmr_data, cimr_data;
	long ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&indio_dev->mlock);
		reinit_completion(&info->completion);

		group = group_idx(chan->channel);
		if (group < 0) {
			mutex_unlock(&indio_dev->mlock);
			return group;
		}

		for (i = 0; i < ADC_NUM_GROUPS; i++) {
			ncmr_data = readl(info->regs + REG_ADC_NCMR(i));
			cimr_data = readl(info->regs + REG_ADC_CIMR(i));

			ncmr_data &= ~ADC_CH_MASK;
			cimr_data &= ~ADC_CIM_MASK;
			if (i == group) {
				ncmr_data |= ADC_CH(chan->channel);
				cimr_data |= ADC_CIM(chan->channel);
			}

			writel(ncmr_data, info->regs + REG_ADC_NCMR(i));
			writel(cimr_data, info->regs + REG_ADC_CIMR(i));
		}

		mcr_data = readl(info->regs + REG_ADC_MCR);
		mcr_data &= ~ADC_MODE;
		mcr_data &= ~ADC_PWDN;
		writel(mcr_data, info->regs + REG_ADC_MCR);

		info->current_channel = chan->channel;

		/* Ensure there are at least three cycles between the
		 * configuration of NCMR and the setting of NSTART */
		ndelay(ADC_NSEC_PER_SEC / s32_adc_clk_rate(info) * 3);

		mcr_data |= ADC_NSTART;
		writel(mcr_data, info->regs + REG_ADC_MCR);

		ret = wait_for_completion_interruptible_timeout
			(&info->completion,
			msecs_to_jiffies(ADC_CONV_TIMEOUT));

		ncmr_data &= ~ADC_CH(info->current_channel);
		cimr_data &= ~ADC_CIM(info->current_channel);
		writel(ncmr_data, info->regs + REG_ADC_NCMR(group));
		writel(cimr_data, info->regs + REG_ADC_CIMR(group));

		mcr_data = readl(info->regs + REG_ADC_MCR);
		mcr_data |= ADC_PWDN;
		writel(mcr_data, info->regs + REG_ADC_MCR);

		if (ret == 0) {
			mutex_unlock(&indio_dev->mlock);
			return -ETIMEDOUT;
		}
		if (ret < 0) {
			mutex_unlock(&indio_dev->mlock);
			return ret;
		}

		*val = info->value;

		mutex_unlock(&indio_dev->mlock);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = info->vref;
		*val2 = ADC_RESOLUTION;
		return IIO_VAL_FRACTIONAL_LOG2;

	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = s32_adc_clk_rate(info) / (ADC_TPT +
				info->adc_feature.sampling_duration[0] +
				ADC_CT +
				ADC_DP);
		return IIO_VAL_INT;

	default:
		break;
	}

	return -EINVAL;
}

static int s32_write_raw(struct iio_dev *indio_dev,
			  struct iio_chan_spec const *chan,
			  int val,
			  int val2,
			  long mask)
{
	struct s32_adc *info = iio_priv(indio_dev);
	int samp_time;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		samp_time = s32_adc_clk_rate(info) / val - (ADC_TPT +
				ADC_CT +
				ADC_DP);
		samp_time = max(samp_time, ADC_INPSAMP_MIN);
		samp_time = min(samp_time, ADC_INPSAMP_MAX);

		info->adc_feature.sampling_duration[0] = samp_time;
		s32_adc_sample_set(info);
		return 0;

	default:
		break;
	}

	return -EINVAL;
}

static const struct iio_info s32_adc_iio_info = {
	.read_raw = &s32_read_raw,
	.write_raw = &s32_write_raw,
};

static const struct of_device_id s32_adc_match[] = {
	{ .compatible = "fsl,s32v234-adc", },
	{ .compatible = "fsl,s32gen1-adc", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, s32_adc_match);

static int s32_adc_probe(struct platform_device *pdev)
{
	struct s32_adc *info;
	struct iio_dev *indio_dev;
	struct resource *mem;
	int irq;
	int ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(struct s32_adc));
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

	ret = devm_request_irq(info->dev, irq,
			       s32_adc_isr, 0,
			       dev_name(&pdev->dev), info);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed requesting irq, irq = %d\n", irq);
		return ret;
	}

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
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->dev.of_node = pdev->dev.of_node;
	indio_dev->info = &s32_adc_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = s32_adc_iio_channels;
	indio_dev->num_channels = ARRAY_SIZE(s32_adc_iio_channels);

	ret = clk_prepare_enable(info->clk);
	if (ret) {
		dev_err(&pdev->dev,
			"Could not prepare or enable the clock.\n");
		return ret;
	}

	s32_adc_cfg_init(info);
	s32_adc_hw_init(info);

	ret = iio_device_register(indio_dev);
	if (ret) {
		dev_err(&pdev->dev, "Couldn't register the device.\n");
		goto error_iio_device_register;
	}

	dev_info(&pdev->dev, "Device initialized successfully.\n");

	return 0;

error_iio_device_register:
	clk_disable_unprepare(info->clk);

	return ret;
}

static int s32_adc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct s32_adc *info = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	clk_disable_unprepare(info->clk);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int s32_adc_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct s32_adc *info = iio_priv(indio_dev);
	int mcr_data;

	/* ADC controller and analog part enter to stop mode */
	mcr_data = readl(info->regs + REG_ADC_MCR);
	mcr_data |= ADC_PWDN;
	writel(mcr_data, info->regs + REG_ADC_MCR);

	clk_disable_unprepare(info->clk);

	return 0;
}

static int s32_adc_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct s32_adc *info = iio_priv(indio_dev);
	int ret;

	ret = clk_prepare_enable(info->clk);
	if (ret)
		return ret;

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(s32_adc_pm_ops, s32_adc_suspend, s32_adc_resume);

static struct platform_driver s32_adc_driver = {
	.probe          = s32_adc_probe,
	.remove         = s32_adc_remove,
	.driver         = {
		.name   = DRIVER_NAME,
		.of_match_table = s32_adc_match,
		.pm     = &s32_adc_pm_ops,
	},
};

module_platform_driver(s32_adc_driver);

MODULE_AUTHOR("Stefan-Gabriel Mirea <stefan-gabriel.mirea@nxp.com>");
MODULE_DESCRIPTION("NXP S32 SAR-ADC driver");
MODULE_LICENSE("GPL v2");
