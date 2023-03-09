// SPDX-License-Identifier: GPL-2.0
/* Copyright 2017-2018, 2020-2023 NXP */

#include <linux/io.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/hwmon.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/mfd/syscon.h>
#include <linux/platform_device.h>
#include <soc/s32cc/fuse.h>
#include "s32cc_thermal.h"

#define DRIVER_NAME "s32cctmu"
#define TMU_SITE(n)			(1 << (n))
#define NO_INTERF_FILES		4

#define KELVIN_TO_CELSIUS_OFFSET	273

union TMU_TCFGR_S32CC_u  {
	u32 R;
	struct {
		u32 CAL_PT:4;
		u32 Reserved0:28;
	} B;
};

struct tmu_chip {
	u32 enable_mask;
	int		 sites;
	/* The hard-coded vectors are required to calibrate the thermal
	 * monitoring unit. They were determined by experiments in a
	 * thermally controlled environment and are specific to each
	 * revision of the SoC.
	 */
	u32 *calib_scfgr;
	u32 *calib_trcr;

	u8	calib_points_num;
	u32 *warm_idxes;
	u32 *cold_idxes;
	u8 trim_idxes_num;

	s16 trange_min;
	s16 trange_max;
};

static struct tmu_chip s32g2_tmu = {
	.enable_mask = 0x2,
	.sites = 3,
	.calib_scfgr = (u32 []){0x2C, 0x59, 0xC6, 0x167},
	.calib_trcr = (u32 []){0xE9, 0x101, 0x13A, 0x18E},
	.calib_points_num = 4,
	.warm_idxes = (u32 []){3},
	.cold_idxes = (u32 []){0},
	.trim_idxes_num = 1,
	.trange_min = -40,
	.trange_max = 125,
};

static struct tmu_chip s32g3_tmu = {
	.enable_mask = 0x2,
	.sites = 3,
	.calib_scfgr = (u32 []){0x1F, 0x28, 0x53, 0xA2, 0x162, 0x16C},
	.calib_trcr = (u32 []){0xE4, 0xE9, 0x100, 0x12A, 0x18E, 0x193},
	.calib_points_num = 6,
	.warm_idxes = (u32 []){4, 5},
	.cold_idxes = (u32 []){0, 1},
	.trim_idxes_num = 2,
	.trange_min = -45,
	.trange_max = 130,
};

static struct tmu_chip s32r45_tmu = {
	.enable_mask = 0x2,
	.sites = 3,
	.calib_scfgr = (u32 []){0x20, 0x29, 0x4E, 0xA3, 0x195, 0x19F},
	.calib_trcr = (u32 []){0xE4, 0xE9, 0xFD, 0x12A, 0x1A7, 0x1AC},
	.calib_points_num = 6,
	.warm_idxes = (u32 []){4, 5},
	.cold_idxes = (u32 []){0, 1},
	.trim_idxes_num = 2,
	.trange_min = -45,
	.trange_max = 155,
};

enum measurement_interval_t {
	mi_0_016s,
	mi_0_032s,
	mi_0_064s,
	mi_0_128s,
	mi_0_256s,
	mi_0_512s,
	mi_1_024s,
	mi_2_048s,
	mi_4_096s,
	mi_8_192s,
	mi_16_384s,
	mi_32_768s,
	mi_65_536s,
	mi_131_072s,
	mi_262_144s,
	mi_continuous };

/* Average Low Pass Filter settings */
enum alpf_t {
	alpf_1,
	alpf_0_5,
	alpf_0_25,
	alpf_0_125 };

struct tmu_driver_data {
	struct clk *clk_mod, *clk_reg;
	void __iomem *tmu_registers;
	union s32cc_tmu_fuse tmu_fuse_val;
	struct device *hwmon_device;
	s16 temp_offset;
	s16 trange_min;
	s16 trange_max;
};

static int get_site_idx_from_label(const char *label)
{
	int cmp_size = sizeof("temp1_label") - 1;

	if (!strncmp(label, "temp1_label", cmp_size) ||
	    !strncmp(label, "temp1_input", cmp_size) ||
	    !strncmp(label, "temp2_label", cmp_size) ||
	    !strncmp(label, "temp2_input", cmp_size))
		return 0;
	if (!strncmp(label, "temp3_label", cmp_size) ||
	    !strncmp(label, "temp3_input", cmp_size) ||
	    !strncmp(label, "temp4_label", cmp_size) ||
	    !strncmp(label, "temp4_input", cmp_size))
		return 1;
	if (!strncmp(label, "temp5_label", cmp_size) ||
	    !strncmp(label, "temp5_input", cmp_size) ||
	    !strncmp(label, "temp6_label", cmp_size) ||
	    !strncmp(label, "temp6_input", cmp_size))
		return 2;

	return -EINVAL;
}

static inline int is_out_of_range(struct tmu_driver_data *tmu_dd,
				  s16 temp)
{
	return (temp < tmu_dd->trange_min ||
		temp > tmu_dd->trange_max);
}

static int tmu_immediate_temperature(struct device *dev,
				     s16 *immediate_temperature, bool *point5,
				     int site)
{
	struct tmu_driver_data *tmu_dd = dev_get_drvdata(dev);
	union TMU_RITSR_u tmu_ritsr;

	tmu_ritsr.R = readl(tmu_dd->tmu_registers + TMU_RITSR(site));
	if (likely(tmu_ritsr.B.V == 0x1)) {
		*immediate_temperature = (s16)tmu_ritsr.B.TEMP;
		*immediate_temperature -= tmu_dd->temp_offset;
		*point5 = (bool)tmu_ritsr.B.TP5;
		return is_out_of_range(tmu_dd, *immediate_temperature);
	}

	return -EBUSY;
}

static int tmu_average_temperature(struct device *dev,
				   s16 *average_temperature, int site)
{
	struct tmu_driver_data *tmu_dd = dev_get_drvdata(dev);
	union TMU_RATSR_u tmu_ratsr;

	tmu_ratsr.R = readl(tmu_dd->tmu_registers + TMU_RATSR(site));
	if (likely(tmu_ratsr.B.V == 0x1)) {
		*average_temperature = (s16)tmu_ratsr.B.TEMP;
		*average_temperature -= tmu_dd->temp_offset;
		return is_out_of_range(tmu_dd, *average_temperature);
	}

	return -EBUSY;
}

static ssize_t tmu_show_immediate_label(struct device *dev,
					struct device_attribute *attr,
					char *buffer)
{
	int site;

	site = get_site_idx_from_label(attr->attr.name);
	if (site == -EINVAL)
		return snprintf(buffer, PAGE_SIZE, "Invalid site\n");
	return snprintf(buffer, PAGE_SIZE,
			"Immediate temperature for site %d\n", site);
}

static ssize_t tmu_show_immediate(struct device *dev,
				  struct device_attribute *attr,
				  char *buffer)
{
	s16 immediate_temperature;
	bool point5;
	int site;

	site = get_site_idx_from_label(attr->attr.name);
	if (site == -EINVAL)
		return snprintf(buffer, PAGE_SIZE, "Invalid site\n");
	if (tmu_immediate_temperature(dev, &immediate_temperature,
				      &point5, site))
		return snprintf(buffer, PAGE_SIZE,
				"Invalid temperature reading!\n");
	else
		return snprintf(buffer, PAGE_SIZE, "%d",
				immediate_temperature * 1000 + 500 * point5);
}

static ssize_t tmu_show_average_label(struct device *dev,
				      struct device_attribute *attr,
				      char *buffer)
{
	int site;

	site = get_site_idx_from_label(attr->attr.name);
	if (site == -EINVAL)
		return snprintf(buffer, PAGE_SIZE, "Invalid site\n");
	return snprintf(buffer, PAGE_SIZE,
			"Average temperature for site %d\n", site);
}

static ssize_t tmu_show_average(struct device *dev,
				struct device_attribute *attr,
				char *buffer)
{
	s16 average_temperature;
	int site;

	site = get_site_idx_from_label(attr->attr.name);
	if (site == -EINVAL)
		return snprintf(buffer, PAGE_SIZE, "Invalid site\n");
	if (tmu_average_temperature(dev, &average_temperature, site))
		return snprintf(buffer, PAGE_SIZE,
				"Invalid temperature reading!\n");
	else
		return snprintf(buffer, PAGE_SIZE, "%d",
				average_temperature * 1000);
}

static struct device_attribute dev_attrs[] = {
	/* First site for temperature detection */
	__ATTR(temp1_label, 0444, tmu_show_immediate_label, NULL),
	__ATTR(temp1_input, 0444, tmu_show_immediate,       NULL),
	__ATTR(temp2_label, 0444, tmu_show_average_label,   NULL),
	__ATTR(temp2_input, 0444, tmu_show_average,         NULL),
	/* Second site for temperature detection */
	__ATTR(temp3_label, 0444, tmu_show_immediate_label,	NULL),
	__ATTR(temp3_input, 0444, tmu_show_immediate,		NULL),
	__ATTR(temp4_label, 0444, tmu_show_average_label,	NULL),
	__ATTR(temp4_input, 0444, tmu_show_average,			NULL),
	/* Third site for temperature detection */
	__ATTR(temp5_label, 0444, tmu_show_immediate_label, NULL),
	__ATTR(temp5_input, 0444, tmu_show_immediate,		NULL),
	__ATTR(temp6_label, 0444, tmu_show_average_label,	NULL),
	__ATTR(temp6_input,	0444, tmu_show_average,			NULL)
};

static void tmu_monitor_enable(struct device *dev,
			       u32 enable_mask, bool do_enable)
{
	struct tmu_driver_data *tmu_dd = dev_get_drvdata(dev);
	union TMU_MR_u tmu_mr;

	tmu_mr.R = readl(tmu_dd->tmu_registers + TMU_MR);

	if (do_enable)
		tmu_mr.B.ME = enable_mask;
	else
		tmu_mr.B.ME = 0x0;

	writel(tmu_mr.R, tmu_dd->tmu_registers + TMU_MR);
}

static void tmu_measurement_interval(struct device *dev,
				     enum measurement_interval_t mi)
{
	struct tmu_driver_data *tmu_dd = dev_get_drvdata(dev);
	union TMU_MTMIR_u tmu_mtmir;

	tmu_mtmir.R = readl(tmu_dd->tmu_registers + TMU_MTMIR);
	tmu_mtmir.B.TMI = mi;
	writel(tmu_mtmir.R, tmu_dd->tmu_registers + TMU_MTMIR);
}

/* Average temperature is calculated as:
 * ALPF * Current_Temp + (1 - ALPF) * Average_Temp
 */
static void tmu_configure_alpf(struct device *dev, enum alpf_t alpf)
{
	struct tmu_driver_data *tmu_dd = dev_get_drvdata(dev);
	union TMU_MR_u tmu_mr;

	tmu_mr.R = readl(tmu_dd->tmu_registers + TMU_MR);
	tmu_mr.B.ALPF = alpf;
	writel(tmu_mr.R, tmu_dd->tmu_registers + TMU_MR);
}

static void tmu_enable_sites(struct device *dev)
{
	struct tmu_driver_data *tmu_dd = dev_get_drvdata(dev);
	union TMU_MSR_u tmu_msr;

	tmu_msr.R = readl(tmu_dd->tmu_registers + TMU_MSR);
	tmu_msr.B.SITE = TMU_SITE(0) | TMU_SITE(1) | TMU_SITE(2);
	writel(tmu_msr.R, tmu_dd->tmu_registers + TMU_MSR);
}

static void update_calib_table_with_fuses(struct device *dev)
{
	struct tmu_driver_data *tmu_dd = dev_get_drvdata(dev);
	const struct tmu_chip *tmu_chip =
		of_device_get_match_data(dev);
	u32 *calib_scfgr = tmu_chip->calib_scfgr;
	union s32cc_tmu_fuse tmu_fuse_val;
	const u32 *warm_idxes = tmu_chip->warm_idxes;
	const u32 *cold_idxes = tmu_chip->cold_idxes;
	size_t trim_idxes_num = tmu_chip->trim_idxes_num;
	size_t i;

	tmu_fuse_val = tmu_dd->tmu_fuse_val;

	for (i = 0; i < trim_idxes_num; i++)
		calib_scfgr[warm_idxes[i]] += tmu_fuse_val.B.CFG_DAC_TRIM0;
	for (i = 0; i < trim_idxes_num; i++)
		calib_scfgr[cold_idxes[i]] += tmu_fuse_val.B.CFG_DAC_TRIM1;
}

static void tmu_calibrate_s32cc(struct device *dev)
{
	struct tmu_driver_data *tmu_dd = dev_get_drvdata(dev);
	const struct tmu_chip *tmu_chip =
		of_device_get_match_data(dev);
	const u32 *calib_scfgr = tmu_chip->calib_scfgr;
	const u32 *calib_trcr = tmu_chip->calib_trcr;
	size_t calib_points_num = tmu_chip->calib_points_num;
	union TMU_TCFGR_S32CC_u tmu_tcfgr;
	union TMU_SCFGR_u tmu_scfgr;
	union TMU_TRCR_u tmu_trcr;
	union TMU_CMCFG_u tmu_cmcfg;
	int i;

	update_calib_table_with_fuses(dev);

	/* These values do look like magic numbers because
	 * they really are. They have been experimentally determined
	 * because there is no documentation for how to choose
	 * them.
	 */
	tmu_cmcfg.R = readl(tmu_dd->tmu_registers + TMU_CMCFG);
	tmu_cmcfg.B.OCS = 0;
	tmu_cmcfg.B.DFD = 3;
	tmu_cmcfg.B.RCTC = 4;
	tmu_cmcfg.B.CLK_DIV = 4;
	tmu_cmcfg.B.OCM = 1;
	tmu_cmcfg.B.DEMA = 1;
	writel(tmu_cmcfg.R, tmu_dd->tmu_registers + TMU_CMCFG);

	tmu_tcfgr.R = readl(tmu_dd->tmu_registers + TMU_TCFGR);
	tmu_scfgr.R = readl(tmu_dd->tmu_registers + TMU_SCFGR);

	for (i = 0; i < calib_points_num; i++) {
		tmu_trcr.R	= readl(tmu_dd->tmu_registers + TMU_TRCR(i));
		tmu_tcfgr.B.CAL_PT = i;
		tmu_scfgr.B.SENSOR = calib_scfgr[i];
		tmu_trcr.B.TEMP = calib_trcr[i];
		tmu_trcr.B.V = 1;
		writel(tmu_tcfgr.R, tmu_dd->tmu_registers + TMU_TCFGR);
		writel(tmu_scfgr.R, tmu_dd->tmu_registers + TMU_SCFGR);
		writel(tmu_trcr.R, tmu_dd->tmu_registers + TMU_TRCR(i));
	}
}

static void tmu_init_hw(struct device *dev,
			const struct tmu_chip *tmu_chip)
{
	tmu_monitor_enable(dev, tmu_chip->enable_mask, false);
	tmu_enable_sites(dev);

	tmu_calibrate_s32cc(dev);

	tmu_configure_alpf(dev, alpf_0_5);
	tmu_measurement_interval(dev, mi_2_048s);
	tmu_monitor_enable(dev, tmu_chip->enable_mask, true);
}

static int tmu_clk_prep_enable(struct tmu_driver_data *tmu_dd)
{
	int ret;

	ret = clk_prepare_enable(tmu_dd->clk_mod);
	if (ret)
		return ret;

	ret = clk_prepare_enable(tmu_dd->clk_reg);
	if (ret)
		return ret;

	return ret;
}

static void tmu_clk_disable_unprep(struct tmu_driver_data *tmu_dd)
{
	clk_disable_unprepare(tmu_dd->clk_mod);
	clk_disable_unprepare(tmu_dd->clk_reg);
}

static const struct of_device_id tmu_dt_ids[] = {
		{ .compatible = "nxp,s32g2-tmu", .data = &s32g2_tmu, },
		{ .compatible = "nxp,s32g3-tmu", .data = &s32g3_tmu, },
		{ .compatible = "nxp,s32r45-tmu", .data = &s32r45_tmu, },
		{ /* end */ }
	};
MODULE_DEVICE_TABLE(of, tmu_dt_ids);

static int tmu_probe(struct platform_device *pd)
{
	struct device *dev = &pd->dev;
	const struct of_device_id *of_matched_dt_id;
	const struct tmu_chip *tmu_chip;
	struct tmu_driver_data *tmu_dd;
	struct resource *tmu_resource;
	int device_files_created = 0;
	int ret;
	int i;

	of_matched_dt_id = of_match_device(tmu_dt_ids, dev);
	if (!of_matched_dt_id) {
		dev_err(dev, "Cannot find a compatible device.\n");
		return -ENODEV;
	}

	tmu_chip = of_device_get_match_data(dev);

	tmu_dd = devm_kzalloc(dev, sizeof(struct tmu_driver_data),
			      GFP_KERNEL);
	if (!tmu_dd)
		return -ENOMEM;

	tmu_dd->trange_min = tmu_chip->trange_min;
	tmu_dd->trange_max = tmu_chip->trange_max;
	dev_set_drvdata(dev, tmu_dd);

	ret = s32cc_ocotp_nvmem_get_tmu_fuse(dev, "tmu_fuse_val",
					     &tmu_dd->tmu_fuse_val);
	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Error reading fuse values\n");

		return ret;
	}

	tmu_resource = platform_get_resource(pd, IORESOURCE_MEM, 0);
	if (!tmu_resource) {
		dev_err(dev, "Cannot obtain TMU resource.\n");
		return -ENODEV;
	}

	tmu_dd->tmu_registers = devm_ioremap_resource(dev, tmu_resource);
	if (IS_ERR(tmu_dd->tmu_registers)) {
		dev_err(dev, "Cannot map TMU registers.\n");
		return PTR_ERR(tmu_dd->tmu_registers);
	}

	tmu_dd->clk_mod = devm_clk_get(dev, "tmu_module");
	if (IS_ERR(tmu_dd->clk_mod)) {
		dev_err(dev, "Cannot obtain 'tmu_module' clock: %ld\n",
			PTR_ERR(tmu_dd->clk_mod));
		return PTR_ERR(tmu_dd->clk_mod);
	}

	tmu_dd->clk_reg = devm_clk_get(dev, "tmu_reg");
	if (IS_ERR(tmu_dd->clk_reg)) {
		dev_err(dev, "Cannot obtain 'tmu_reg' clock: %ld\n",
			PTR_ERR(tmu_dd->clk_reg));
		return PTR_ERR(tmu_dd->clk_reg);
	}

	ret = tmu_clk_prep_enable(tmu_dd);
	if (ret) {
		dev_err(dev, "Cannot enable clock: %d\n", ret);
		return ret;
	}

	tmu_dd->temp_offset = KELVIN_TO_CELSIUS_OFFSET;
	tmu_init_hw(dev, tmu_chip);

	tmu_dd->hwmon_device =
		hwmon_device_register_with_info(dev, DRIVER_NAME,
						tmu_dd, NULL, NULL);
	if (IS_ERR(tmu_dd->hwmon_device)) {
		ret = PTR_ERR(tmu_dd->hwmon_device);
		dev_err(dev, "Cannot register hwmon device: %d\n", ret);
		goto hwmon_register_failed;
	}

	if (ARRAY_SIZE(dev_attrs) < tmu_chip->sites * NO_INTERF_FILES) {
		dev_err(dev, "Number of sysfs files does not map to device_attribute\n");
		goto hwmon_register_failed;
	}
	for (i = 0; i < (tmu_chip->sites * NO_INTERF_FILES); i++) {
		ret = device_create_file(tmu_dd->hwmon_device,
					 &dev_attrs[i]);
		if (ret)
			break;
		device_files_created++;
	}
	if (ret) {
		for (i = 0; i < device_files_created; i++)
			device_remove_file(tmu_dd->hwmon_device, &dev_attrs[i]);
		hwmon_device_unregister(tmu_dd->hwmon_device);
		goto hwmon_register_failed;
	}

	return 0;

hwmon_register_failed:
	tmu_clk_disable_unprep(tmu_dd);

	return ret;
}

static int tmu_remove(struct platform_device *pdev)
{
	struct tmu_driver_data *tmu_dd = platform_get_drvdata(pdev);
	const struct tmu_chip *tmu_chip;
	int i;

	tmu_chip = of_device_get_match_data(&pdev->dev);
	for (i = 0; i < (tmu_chip->sites * 4); i++)
		device_remove_file(tmu_dd->hwmon_device, &dev_attrs[i]);

	hwmon_device_unregister(tmu_dd->hwmon_device);
	tmu_clk_disable_unprep(tmu_dd);

	return 0;
}

static int __maybe_unused thermal_suspend(struct device *dev)
{
	struct tmu_driver_data *tmu_dd = dev_get_drvdata(dev);

	tmu_clk_disable_unprep(tmu_dd);

	return 0;
}

static int __maybe_unused thermal_resume(struct device *dev)
{
	struct tmu_driver_data *tmu_dd = dev_get_drvdata(dev);
	const struct tmu_chip *tmu_chip;
	int ret = 0;

	tmu_chip = of_device_get_match_data(dev);

	ret = tmu_clk_prep_enable(tmu_dd);
	if (ret) {
		dev_err(dev, "Cannot enable clock: %d\n", ret);
		return ret;
	}

	tmu_init_hw(dev, tmu_chip);

	return ret;
}

static SIMPLE_DEV_PM_OPS(thermal_pm_ops, thermal_suspend, thermal_resume);

static struct platform_driver tmu_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.of_match_table = tmu_dt_ids,
		.pm = &thermal_pm_ops,
	},
	.probe		= tmu_probe,
	.remove		= tmu_remove,
};
module_platform_driver(tmu_driver);

MODULE_AUTHOR("NXP");
MODULE_DESCRIPTION("Thermal driver for NXP S32CC");
MODULE_LICENSE("GPL v2");
