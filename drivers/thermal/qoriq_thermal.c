// SPDX-License-Identifier: GPL-2.0
//
// Copyright 2016 Freescale Semiconductor, Inc.
// Copyright 2023 NXP

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/sizes.h>
#include <linux/thermal.h>
#include <linux/units.h>
#include <soc/s32cc/nvmem_common.h>

#include "thermal_core.h"
#include "thermal_hwmon.h"

#define SITES_MAX		16
#define TEUMR0_V2		0x51009c00
#define TMSARA_V2		0xe
#define TMU_VER1		0x1
#define TMU_VER2		0x2

#define REGS_TMR	0x000	/* Mode Register */
#define TMR_DISABLE	0x0
#define TMR_ME		0x80000000
#define TMR_ALPF	0x0c000000
#define TMR_ALPF_V2	0x03000000

#define REGS_TMTMIR	0x008	/* Temperature measurement interval Register */
#define TMTMIR_DEFAULT	0x0000000f

#define REGS_V2_TMSR	0x008	/* monitor site register */
#define TMSR_SITES	GENMASK(2, 0)

#define REGS_V2_TMTMIR	0x00c	/* Temperature measurement interval Register */

#define REGS_TIER	0x020	/* Interrupt Enable Register */
#define TIER_DISABLE	0x0


#define REGS_TTCFGR	0x080	/* Temperature Configuration Register */
#define REGS_TSCFGR	0x084	/* Sensor Configuration Register */

#define REGS_TRITSR(n)	(0x100 + 16 * (n)) /* Immediate Temperature
					    * Site Register
					    */
#define TRITSR_V	BIT(31)
#define TRITSR_TP5	BIT(9)
#define REGS_V2_TMSAR(n)	(0x304 + 16 * (n))	/* TMU monitoring
						* site adjustment register
						*/
#define REGS_TTRnCR(n)	(0xf10 + 4 * (n)) /* Temperature Range n
					   * Control Register
					   */
#define TTRCR_V		BIT(31)
#define REGS_IPBRR(n)		(0xbf8 + 4 * (n)) /* IP Block Revision
						   * Register n
						   */
#define REGS_V2_TEUMR(n)	(0xf00 + 4 * (n))
#define REGS_V2_TCMCFG		0xf00

/* Experimentally determined */
#define S32CC_TCMCFG	(0x54004c00)

/*
 * Thermal zone data
 */
struct qoriq_sensor {
	int				id;
};

struct qoriq_tmu_data {
	int ver;
	struct regmap *regmap;
	struct clk *clk;
	struct qoriq_sensor	sensor[SITES_MAX];
	struct device *dev;
	u32 sites;
	void *plat_data;
	u8 sites_max;
};

struct s32cc_plat_data {
	u32 fuse_val;
	u32 *indexes;
	u32 *trcr;
	u32 *scfgr;
	u32 len;
};

static int qoriq_init_and_calib(struct qoriq_tmu_data *data);
static int s32cc_init_and_calib(struct qoriq_tmu_data *data);

static struct qoriq_tmu_data *qoriq_sensor_to_data(struct qoriq_sensor *s)
{
	return container_of(s, struct qoriq_tmu_data, sensor[s->id]);
}

static int tmu_get_temp(void *p, int *temp)
{
	struct qoriq_sensor *qsensor = p;
	struct qoriq_tmu_data *qdata = qoriq_sensor_to_data(qsensor);
	u32 val;
	/*
	 * REGS_TRITSR(id) has the following layout:
	 *
	 * For TMU Rev1:
	 * 31  ... 7 6 5 4 3 2 1 0
	 *  V          TEMP
	 *
	 * Where V bit signifies if the measurement is ready and is
	 * within sensor range. TEMP is an 8 bit value representing
	 * temperature in Celsius.

	 * For TMU Rev2:
	 * 31  ... 8 7 6 5 4 3 2 1 0
	 *  V          TEMP
	 *
	 * Where V bit signifies if the measurement is ready and is
	 * within sensor range. TEMP is an 9 bit value representing
	 * temperature in KelVin.
	 */

	regmap_read(qdata->regmap, REGS_TMR, &val);
	if (!(val & TMR_ME))
		return -EAGAIN;

	if (regmap_read_poll_timeout(qdata->regmap,
				     REGS_TRITSR(qsensor->id),
				     val,
				     val & TRITSR_V,
				     USEC_PER_MSEC,
				     10 * USEC_PER_MSEC))
		return -ENODATA;

	if (qdata->ver == TMU_VER1) {
		*temp = (val & GENMASK(7, 0)) * MILLIDEGREE_PER_DEGREE;
	} else {
		if (val & TRITSR_TP5)
			*temp = milli_kelvin_to_millicelsius((val & GENMASK(8, 0)) *
							     MILLIDEGREE_PER_DEGREE + 500);
		else
			*temp = kelvin_to_millicelsius(val & GENMASK(8, 0));
	}

	return 0;
}

static const struct thermal_zone_of_device_ops tmu_tz_ops = {
	.get_temp = tmu_get_temp,
};

static int qoriq_tmu_register_tmu_zone(struct device *dev,
				       struct qoriq_tmu_data *qdata)
{
	int id, sites = 0;

	for (id = 0; id < qdata->sites_max; id++) {
		struct thermal_zone_device *tzd;
		struct qoriq_sensor *sensor = &qdata->sensor[id];
		int ret;

		sensor->id = id;

		tzd = devm_thermal_zone_of_sensor_register(dev, id,
							   sensor,
							   &tmu_tz_ops);
		ret = PTR_ERR_OR_ZERO(tzd);
		if (ret) {
			if (ret == -ENODEV)
				continue;

			return ret;
		}

		if (qdata->ver == TMU_VER1)
			sites |= 0x1 << (15 - id);
		else
			sites |= 0x1 << id;

		if (devm_thermal_add_hwmon_sysfs(tzd))
			dev_warn(dev,
				 "Failed to add hwmon sysfs attributes\n");
	}

	if (sites) {
		if (qdata->ver == TMU_VER1) {
			regmap_write(qdata->regmap, REGS_TMR, TMR_ME | TMR_ALPF | sites);
		} else {
			regmap_write(qdata->regmap, REGS_V2_TMSR, sites);
			regmap_write(qdata->regmap, REGS_TMR, TMR_ME | TMR_ALPF_V2);
		}
	}

	if (sites > 0)
		qdata->sites = sites;

	return 0;
}

static int qoriq_tmu_calibration(struct device *dev,
				 struct qoriq_tmu_data *data)
{
	int i, val, len;
	u32 range[4];
	const u32 *calibration;
	struct device_node *np = dev->of_node;

	len = of_property_count_u32_elems(np, "fsl,tmu-range");
	if (len < 0 || len > 4) {
		dev_err(dev, "invalid range data.\n");
		return len;
	}

	val = of_property_read_u32_array(np, "fsl,tmu-range", range, len);
	if (val != 0) {
		dev_err(dev, "failed to read range data.\n");
		return val;
	}

	/* Init temperature range registers */
	for (i = 0; i < len; i++)
		regmap_write(data->regmap, REGS_TTRnCR(i), range[i]);

	calibration = of_get_property(np, "fsl,tmu-calibration", &len);
	if (calibration == NULL || len % 8) {
		dev_err(dev, "invalid calibration data.\n");
		return -ENODEV;
	}

	for (i = 0; i < len; i += 8, calibration += 2) {
		val = of_read_number(calibration, 1);
		regmap_write(data->regmap, REGS_TTCFGR, val);
		val = of_read_number(calibration + 1, 1);
		regmap_write(data->regmap, REGS_TSCFGR, val);
	}

	return 0;
}

static void qoriq_tmu_init_device(struct qoriq_tmu_data *data)
{
	/* Disable interrupt, using polling instead */
	regmap_write(data->regmap, REGS_TIER, TIER_DISABLE);

	/* Disable the module */
	regmap_write(data->regmap, REGS_TMR, TMR_DISABLE);

	/* Remove all monitored sites */
	if (data->ver != TMU_VER1)
		regmap_update_bits(data->regmap, REGS_V2_TMSR, TMSR_SITES, 0);

	/* Set update_interval */

	if (data->ver == TMU_VER1) {
		regmap_write(data->regmap, REGS_TMTMIR, TMTMIR_DEFAULT);
	} else {
		regmap_write(data->regmap, REGS_V2_TMTMIR, TMTMIR_DEFAULT);
	}
}

static const struct regmap_range qoriq_yes_ranges[] = {
	regmap_reg_range(REGS_TMR, REGS_TSCFGR),
	regmap_reg_range(REGS_TTRnCR(0), REGS_TTRnCR(15)),
	regmap_reg_range(REGS_V2_TEUMR(0), REGS_V2_TEUMR(2)),
	regmap_reg_range(REGS_V2_TMSAR(0), REGS_V2_TMSAR(15)),
	regmap_reg_range(REGS_IPBRR(0), REGS_IPBRR(1)),
	/* Read only registers below */
	regmap_reg_range(REGS_TRITSR(0), REGS_TRITSR(15)),
};

static const struct regmap_access_table qoriq_wr_table = {
	.yes_ranges	= qoriq_yes_ranges,
	.n_yes_ranges	= ARRAY_SIZE(qoriq_yes_ranges) - 1,
};

static const struct regmap_access_table qoriq_rd_table = {
	.yes_ranges	= qoriq_yes_ranges,
	.n_yes_ranges	= ARRAY_SIZE(qoriq_yes_ranges),
};

static const struct regmap_range s32cc_yes_ranges[] = {
	regmap_reg_range(REGS_TMR, REGS_TSCFGR),
	regmap_reg_range(REGS_TTRnCR(0), REGS_TTRnCR(15)),
	regmap_reg_range(REGS_V2_TCMCFG, REGS_V2_TCMCFG),
	/* Read-only */
	regmap_reg_range(REGS_IPBRR(0), REGS_IPBRR(1)),
	regmap_reg_range(REGS_TRITSR(0), REGS_TRITSR(2)),
};

static const struct regmap_access_table s32cc_rw_table = {
	.yes_ranges	= s32cc_yes_ranges,
	.n_yes_ranges	= ARRAY_SIZE(s32cc_yes_ranges),
};

static const struct regmap_access_table s32cc_ro_table = {
	.yes_ranges	= s32cc_yes_ranges,
	.n_yes_ranges	= ARRAY_SIZE(s32cc_yes_ranges),
};

struct qoirq_tmu_data {
	const struct regmap_access_table *ro_table;
	const struct regmap_access_table *rw_table;
	int (*init_and_calib)(struct qoriq_tmu_data *);
	u8 sites_max;
};

static const struct qoirq_tmu_data qoriq_data = {
	.ro_table = &qoriq_rd_table,
	.rw_table = &qoriq_wr_table,
	.init_and_calib = qoriq_init_and_calib,
	.sites_max = SITES_MAX,
};

static const struct qoirq_tmu_data s32cc_data = {
	.ro_table = &s32cc_ro_table,
	.rw_table = &s32cc_rw_table,
	.init_and_calib = s32cc_init_and_calib,
	.sites_max = 3,
};

static void qoriq_tmu_action(void *p)
{
	struct qoriq_tmu_data *data = p;

	regmap_write(data->regmap, REGS_TMR, TMR_DISABLE);
	clk_disable_unprepare(data->clk);
}

static int qoriq_init_and_calib(struct qoriq_tmu_data *data)
{
	if (data->ver != TMU_VER1)
		regmap_write(data->regmap, REGS_V2_TEUMR(0), TEUMR0_V2);

	return qoriq_tmu_calibration(data->dev, data);	/* TMU calibration */
}

static s32 complement_two(u32 num, unsigned int width)
{
	unsigned int sign_mask;
	unsigned int num_mask;

	if (width >= BITS_PER_LONG)
		return -1;

	sign_mask = BIT(width - 1);
	num_mask = GENMASK(width - 1, 0);

	if (!(num & sign_mask))
		return num;

	return -1 * (s32)(((~num) & num_mask) + 1);
}

static int s32cc_init_calib_arrays(struct qoriq_tmu_data *data)
{
	struct s32cc_plat_data *plat_data = data->plat_data;
	u32 *calib_array, nvmem_mask, nvmem_adj, mask_shift;
	struct device *dev = data->dev;
	struct device_node *np = dev->of_node;
	int n_ranges, calib_size, ret, i, j;
	unsigned int width;
	s32 scfgr_adj;

	n_ranges = of_property_count_u32_elems(np, "fsl,tmu-range");
	if (n_ranges < 0) {
		dev_err(dev, "Invalid TMU ranges\n");
		return n_ranges;
	}

	plat_data->len = n_ranges;

	plat_data->trcr = devm_kmalloc(dev, sizeof(*plat_data->trcr) * n_ranges,
				       GFP_KERNEL);
	if (!plat_data->trcr)
		return -ENOMEM;

	plat_data->scfgr = devm_kmalloc(dev, sizeof(*plat_data->scfgr) * n_ranges,
					GFP_KERNEL);
	if (!plat_data->scfgr)
		return -ENOMEM;

	plat_data->indexes = devm_kmalloc(dev, sizeof(*plat_data->indexes) * n_ranges,
					  GFP_KERNEL);
	if (!plat_data->indexes)
		return -ENOMEM;

	ret = of_property_read_u32_array(np, "fsl,tmu-range",
					 plat_data->trcr, n_ranges);
	if (ret) {
		dev_err(dev, "Failed to read range data (%d).\n", ret);
		return ret;
	}

	calib_size = of_property_count_u32_elems(np, "fsl,tmu-calibration");
	if (calib_size < 0) {
		dev_err(dev, "Invalid calibration array\n");
		return calib_size;
	}

	if (calib_size != 3 * n_ranges) {
		dev_err(dev, "Invalid number of calibrarion items: %d. Expected %d\n",
			calib_size, 3 * n_ranges);
		return -EINVAL;
	}

	calib_array = kmalloc(sizeof(*calib_array) * calib_size, GFP_KERNEL);
	if (!calib_array)
		return -ENOMEM;

	ret = of_property_read_u32_array(np, "fsl,tmu-calibration", calib_array,
					 calib_size);
	if (ret) {
		dev_err(dev, "Failed to read calibration data (%d).\n", ret);
		goto free_calibarray;
	}

	for (i = 0, j = 0; i < calib_size; i += 3, j++) {
		plat_data->indexes[j] = calib_array[i];
		plat_data->scfgr[j] = calib_array[i + 1];

		nvmem_mask = calib_array[i + 2];

		if (!nvmem_mask)
			continue;

		mask_shift = ffs(nvmem_mask) - 1;
		width = hweight32(nvmem_mask >> mask_shift);

		if (!width) {
			dev_err(dev, "Invalid mask width for:0x%x\n", nvmem_mask);
			ret = -EINVAL;
			break;
		}

		nvmem_adj = (plat_data->fuse_val & nvmem_mask) >> mask_shift;

		/* The trim values for TMU are in 2's complement */
		scfgr_adj = complement_two(nvmem_adj, width);

		if (scfgr_adj > 0) {
			if (unlikely(check_add_overflow(plat_data->scfgr[j],
							(u32)scfgr_adj,
							&plat_data->scfgr[j])))
				ret = -EOVERFLOW;
		} else {
			scfgr_adj *= -1;
			if (unlikely(check_sub_overflow(plat_data->scfgr[j],
							(u32)scfgr_adj,
							&plat_data->scfgr[j])))
				ret = -EOVERFLOW;
		}

		if (ret)
			break;
	}

free_calibarray:
	kfree(calib_array);

	return ret;
}

static void s32cc_calib(struct qoriq_tmu_data *data)
{
	struct s32cc_plat_data *plat_data = data->plat_data;
	u32 i;

	regmap_update_bits(data->regmap, REGS_V2_TCMCFG,
			   GENMASK(31, 8), S32CC_TCMCFG);

	for (i = 0u; i < plat_data->len; i++) {
		regmap_write(data->regmap, REGS_TTCFGR, plat_data->indexes[i]);
		regmap_write(data->regmap, REGS_TSCFGR, plat_data->scfgr[i]);
		regmap_write(data->regmap, REGS_TTRnCR(i), plat_data->trcr[i] | TTRCR_V);
	}
}

static int s32cc_get_calib_value(struct qoriq_tmu_data *data,
				 u32 *fuse_val)
{
	char *buf = NULL;

	buf = read_nvmem_cell(data->dev, "tmu_fuse_val");
	if (IS_ERR(buf)) {
		if (PTR_ERR(buf) != -EPROBE_DEFER)
			dev_err(data->dev, "Error reading fuse values\n");
		return PTR_ERR(buf);
	}

	*fuse_val = *(u32 *)buf;

	kfree(buf);

	return 0;
}

static int s32cc_init_and_calib(struct qoriq_tmu_data *data)
{
	struct s32cc_plat_data *plat_data;
	int ret;

	plat_data = devm_kmalloc(data->dev, sizeof(*plat_data), GFP_KERNEL);
	if (!plat_data)
		return -ENOMEM;

	data->plat_data = plat_data;

	ret = s32cc_get_calib_value(data, &plat_data->fuse_val);
	if (ret)
		return ret;

	ret = s32cc_init_calib_arrays(data);
	if (ret)
		return ret;

	s32cc_calib(data);

	return 0;
}

static int qoriq_tmu_probe(struct platform_device *pdev)
{
	int ret;
	u32 ver;
	struct qoriq_tmu_data *data;
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	const bool little_endian = of_property_read_bool(np, "little-endian");
	const enum regmap_endian format_endian =
		little_endian ? REGMAP_ENDIAN_LITTLE : REGMAP_ENDIAN_BIG;
	const struct qoirq_tmu_data *match_data = of_device_get_match_data(dev);
	const struct regmap_config regmap_config = {
		.reg_bits		= 32,
		.val_bits		= 32,
		.reg_stride		= 4,
		.rd_table		= match_data->ro_table,
		.wr_table		= match_data->rw_table,
		.val_format_endian	= format_endian,
		.max_register		= SZ_4K,
	};
	void __iomem *base;

	data = devm_kzalloc(dev, sizeof(struct qoriq_tmu_data),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = dev;
	data->sites_max = match_data->sites_max;
	if (data->sites_max > SITES_MAX) {
		dev_warn(dev, "driver supports maximum %d sites\n", SITES_MAX);
		data->sites_max = SITES_MAX;
	}

	base = devm_platform_ioremap_resource(pdev, 0);
	ret = PTR_ERR_OR_ZERO(base);
	if (ret) {
		dev_err(dev, "Failed to get memory region\n");
		return ret;
	}

	data->regmap = devm_regmap_init_mmio(dev, base, &regmap_config);
	ret = PTR_ERR_OR_ZERO(data->regmap);
	if (ret) {
		dev_err(dev, "Failed to init regmap (%d)\n", ret);
		return ret;
	}

	data->clk = devm_clk_get_optional(dev, NULL);
	if (IS_ERR(data->clk))
		return PTR_ERR(data->clk);

	ret = clk_prepare_enable(data->clk);
	if (ret) {
		dev_err(dev, "Failed to enable clock\n");
		return ret;
	}

	ret = devm_add_action_or_reset(dev, qoriq_tmu_action, data);
	if (ret)
		return ret;

	/* version register offset at: 0xbf8 on both v1 and v2 */
	ret = regmap_read(data->regmap, REGS_IPBRR(0), &ver);
	if (ret) {
		dev_err(dev, "Failed to read IP block version\n");
		return ret;
	}
	data->ver = (ver >> 8) & 0xff;

	qoriq_tmu_init_device(data);	/* TMU initialization */

	ret = match_data->init_and_calib(data);
	if (ret < 0)
		return ret;

	ret = qoriq_tmu_register_tmu_zone(dev, data);
	if (ret < 0) {
		dev_err(dev, "Failed to register sensors\n");
		return ret;
	}

	platform_set_drvdata(pdev, data);

	return 0;
}

static int __maybe_unused qoriq_tmu_suspend(struct device *dev)
{
	struct qoriq_tmu_data *data = dev_get_drvdata(dev);
	int ret;

	ret = regmap_update_bits(data->regmap, REGS_TMR, TMR_ME, 0);
	if (ret)
		return ret;

	clk_disable_unprepare(data->clk);

	return 0;
}

static int __maybe_unused qoriq_tmu_resume(struct device *dev)
{
	int ret;
	struct qoriq_tmu_data *data = dev_get_drvdata(dev);
	const struct qoirq_tmu_data *match_data = of_device_get_match_data(dev);

	ret = clk_prepare_enable(data->clk);
	if (ret)
		return ret;

	if (match_data == &s32cc_data) {
		s32cc_calib(data);

		regmap_write(data->regmap, REGS_V2_TMSR, data->sites);
		regmap_write(data->regmap, REGS_TMR, TMR_ME | TMR_ALPF_V2);

		return 0;
	}

	/* Enable monitoring */
	return regmap_update_bits(data->regmap, REGS_TMR, TMR_ME, TMR_ME);
}

static SIMPLE_DEV_PM_OPS(qoriq_tmu_pm_ops,
			 qoriq_tmu_suspend, qoriq_tmu_resume);

static const struct of_device_id qoriq_tmu_match[] = {
	{ .compatible = "fsl,qoriq-tmu", .data = &qoriq_data, },
	{ .compatible = "fsl,imx8mq-tmu", .data = &qoriq_data, },
	{ .compatible = "nxp,s32cc-tmu", .data = &s32cc_data, },
	{},
};
MODULE_DEVICE_TABLE(of, qoriq_tmu_match);

static struct platform_driver qoriq_tmu = {
	.driver	= {
		.name		= "qoriq_thermal",
		.pm		= &qoriq_tmu_pm_ops,
		.of_match_table	= qoriq_tmu_match,
	},
	.probe	= qoriq_tmu_probe,
};
module_platform_driver(qoriq_tmu);

MODULE_AUTHOR("Jia Hongtao <hongtao.jia@nxp.com>");
MODULE_DESCRIPTION("QorIQ Thermal Monitoring Unit driver");
MODULE_LICENSE("GPL v2");
