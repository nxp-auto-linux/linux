// SPDX-License-Identifier: GPL-2.0
/* Copyright 2017-2018,2020-2022 NXP */

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

#define DRIVER_NAME "s32tmu"

#include "s32gen1_thermal.h"

#define TMU_SITE(n)			(1 << n)
#define CALIB_POINTS		4
#define NO_INTERF_FILES		4

/* The hard-coded vectors are required to calibrate the thermal
 * monitoring unit. They were determined by experiments in a
 * thermally controlled environment and are specific to each
 * revision of the SoC.
 */
static const uint32_t rev2_calib_scfgr[] = {
	0x2C, 0x59, 0xC6, 0x167
};

static const uint32_t rev2_calib_trcr[] = {
	0xE9, 0x101, 0x13A, 0x18E
};

enum s32_calib_fuse {
	COLD_FUSE,
	WARM_FUSE
};

union CAL_FUSE_u {
	s32 R;
	struct {
		s32 CFG_DAC_TRIM0:5;
		s32 Reserved0:1;
		s32 CFG_DAC_TRIM1:5;
		s32 Reserved1:21;
	} B;
};

/* Due to the fact that TCFGR register is very different
 * between V234 and Gen1 we always include both of the
 * structures.
 * If we didn't, the compiler has no way to know that the
 * calibration function for V and for Gen1 will never
 * be both called in the resulting binary (namely, this driver
 * will be compiled for either V234 or for Gen1, never for
 * both simultaneously).
 */
union TMU_TCFGR_V234_u {
	uint32_t R;
	struct {
		uint32_t Data0:4;
		uint32_t Reserved0:12;
		uint32_t Data1:4;
		uint32_t Reserved1:12;
	} B;
};

union TMU_TCFGR_GEN1_u {
	uint32_t R;
	struct {
		uint32_t CAL_PT:4;
		uint32_t Reserved0:28;
	} B;
};

struct fsl_tmu_chip {
	bool	 has_clk;
	bool	 has_fuse;
	uint32_t enable_mask;
	int		 sites;
	int16_t	 offset_to_celsius;
};

static struct fsl_tmu_chip gen1_tmu = {
	.has_clk = false,
	.has_fuse = true,
	.enable_mask = 0x2,
	.sites = 3,
	/* For gen1, the TMU offers the value in Kelvin. 0 Celsius dgr. = 273K */
	.offset_to_celsius = 273,
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
	struct clk *clk;
	void __iomem *tmu_registers;
	void __iomem *fuse_base;
	struct device *hwmon_device;
	int16_t temp_offset;
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

	return -1;
}

static inline int is_out_of_range(int8_t temperature)
{
	return (temperature < -40 || temperature > 125);
}

/* 8 bit TEMP field of the RITSR and RATSR registers should be interpreted
 * as a signed integer when the temperature is below 25 degrees
 * Celsius and as an unsigned integer when the temperature is above 25
 * degrees. The fact that the sensor reading range is -40 to 125 degrees
 * allows us to simply cast to an int8_t, since within this range
 * interpreting the field as a signed integer always leads to the
 * correct result. If the value would ever fall outside this range,
 * the `Valid` bit of the registers would be cleared by hardware.
 */
static int tmu_immediate_temperature(struct device *dev,
				     s16 *immediate_temperature,
				     bool *point5, int site)
{
	struct tmu_driver_data *tmu_dd = dev_get_drvdata(dev);
	union TMU_RITSR_u tmu_ritsr;

	tmu_ritsr.R = readl(tmu_dd->tmu_registers + TMU_RITSR(site));
	if (likely(tmu_ritsr.B.V == 0x1)) {
		*immediate_temperature = (int16_t)tmu_ritsr.B.TEMP;
		*immediate_temperature -= tmu_dd->temp_offset;
		*point5 = (bool)tmu_ritsr.B.TP5;
		return is_out_of_range(*immediate_temperature);
	}

	return -1;
}

static int tmu_average_temperature(struct device *dev,
					int16_t *average_temperature, int site)
{
	struct tmu_driver_data *tmu_dd = dev_get_drvdata(dev);
	union TMU_RATSR_u tmu_ratsr;

	tmu_ratsr.R = readl(tmu_dd->tmu_registers + TMU_RATSR(site));
	if (likely(tmu_ratsr.B.V == 0x1)) {
		*average_temperature = (int16_t)tmu_ratsr.B.TEMP;
		*average_temperature -= tmu_dd->temp_offset;
		return is_out_of_range(*average_temperature);
	}

	return -1;
}

static ssize_t tmu_show_immediate_label(struct device *dev,
		struct device_attribute *attr, char *buffer)
{
	int site;

	site = get_site_idx_from_label(attr->attr.name);
	if (site == -1)
		return snprintf(buffer, PAGE_SIZE, "Invalid site\n");
	return snprintf(buffer, PAGE_SIZE,
			"Immediate temperature for site %d\n", site);
}

static ssize_t tmu_show_immediate(struct device *dev,
		struct device_attribute *attr, char *buffer)
{
	int16_t immediate_temperature;
	bool point5;
	int site;

	site = get_site_idx_from_label(attr->attr.name);
	if (site == -1)
		return snprintf(buffer, PAGE_SIZE, "Invalid site\n");
	if (tmu_immediate_temperature(dev, &immediate_temperature, &point5, site))
		return snprintf(buffer, PAGE_SIZE,
				"Invalid temperature reading!\n");
	else
		return snprintf(buffer, PAGE_SIZE, "%d",
				immediate_temperature * 1000 + 500 * point5);
}

static ssize_t tmu_show_average_label(struct device *dev,
		struct device_attribute *attr, char *buffer)
{
	int site;

	site = get_site_idx_from_label(attr->attr.name);
	if (site == -1)
		return snprintf(buffer, PAGE_SIZE, "Invalid site\n");
	return snprintf(buffer, PAGE_SIZE,
			"Average temperature for site %d\n", site);
}

static ssize_t tmu_show_average(struct device *dev,
		struct device_attribute *attr, char *buffer)
{
	int16_t average_temperature;
	int site;

	site = get_site_idx_from_label(attr->attr.name);
	if (site == -1)
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
		uint32_t enable_mask, bool do_enable)
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

static s32 get_calib_fuse_val(struct device *dev,
			      enum s32_calib_fuse fuse_type)
{
	struct tmu_driver_data *tmu_dd = dev_get_drvdata(dev);
	union CAL_FUSE_u calib_fuse;

	calib_fuse.R = readl(tmu_dd->fuse_base + CAL_FUSE);
	if (fuse_type == COLD_FUSE)
		return calib_fuse.B.CFG_DAC_TRIM1;
	return calib_fuse.B.CFG_DAC_TRIM0;
}

static void get_calib_with_fuses(struct device *dev,
		uint32_t *calib_scfgr, uint32_t *calib_trcr,
		const uint32_t *calib_scfgr_base,
		const uint32_t *calib_trcr_base,
		const uint32_t *warm_idxes, const uint32_t *cold_idxes,
		size_t warm_size, size_t cold_size, size_t table_size)
{
	size_t i;

	s32 fuse_val_cold = get_calib_fuse_val(dev, COLD_FUSE);
	s32 fuse_val_warm = get_calib_fuse_val(dev, WARM_FUSE);

	memcpy(calib_scfgr, calib_scfgr_base, table_size);
	for (i = 0; i < warm_size; i++)
		calib_scfgr[warm_idxes[i]] += fuse_val_warm;
	for (i = 0; i < cold_size; i++)
		calib_scfgr[cold_idxes[i]] += fuse_val_cold;

	memcpy(calib_trcr, calib_trcr_base, table_size);
}

static void get_calib_table(struct device *dev,
		uint32_t *calib_scfgr, uint32_t *calib_trcr)
{
	const static u32 warm_idxes[] = {3};
	const static u32 cold_idxes[] = {0};

	get_calib_with_fuses(dev,
			     calib_scfgr, calib_trcr,
			     rev2_calib_scfgr, rev2_calib_trcr,
			     warm_idxes, cold_idxes,
			     ARRAY_SIZE(warm_idxes), ARRAY_SIZE(cold_idxes),
			     sizeof(rev2_calib_scfgr));
}

static int tmu_calibrate_s32gen1(struct device *dev)
{
	struct tmu_driver_data *tmu_dd = dev_get_drvdata(dev);
	union TMU_TCFGR_GEN1_u tmu_tcfgr;
	union TMU_SCFGR_u tmu_scfgr;
	union TMU_TRCR_u tmu_trcr;
	union TMU_CMCFG_u tmu_cmcfg;
	int i;
	u32 calib_scfgr[CALIB_POINTS];
	u32 calib_trcr[CALIB_POINTS];

	get_calib_table(dev, calib_scfgr, calib_trcr);

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

	for (i = 0; i < CALIB_POINTS; i++) {
		tmu_trcr.R	= readl(tmu_dd->tmu_registers + TMU_TRCR(i));
		tmu_tcfgr.B.CAL_PT = i;
		tmu_scfgr.B.SENSOR = calib_scfgr[i];
		tmu_trcr.B.TEMP = calib_trcr[i];
		tmu_trcr.B.V = 1;
		writel(tmu_tcfgr.R, tmu_dd->tmu_registers + TMU_TCFGR);
		writel(tmu_scfgr.R, tmu_dd->tmu_registers + TMU_SCFGR);
		writel(tmu_trcr.R, tmu_dd->tmu_registers + TMU_TRCR(i));
	}

	return 0;
}

static int tmu_init_hw(struct device *dev,
		const struct fsl_tmu_chip *tmu_chip)
{
	int ret;

	tmu_monitor_enable(dev, tmu_chip->enable_mask, false);

	tmu_enable_sites(dev);

	ret = tmu_calibrate_s32gen1(dev);
	if (ret) {
		dev_err(dev, "TMU Calibration Failed\n");
		return ret;
	}

	tmu_configure_alpf(dev, alpf_0_5);
	tmu_measurement_interval(dev, mi_2_048s);
	tmu_monitor_enable(dev, tmu_chip->enable_mask, true);

	return 0;
}

static const struct of_device_id tmu_dt_ids[] = {
		{ .compatible = "nxp,s32cc-tmu", .data = &gen1_tmu, },
		{ /* end */ }
	};
MODULE_DEVICE_TABLE(of, tmu_dt_ids);

static int tmu_probe(struct platform_device *pd)
{
	const struct of_device_id *of_matched_dt_id;
	struct tmu_driver_data *tmu_dd;
	struct resource *tmu_resource;
	struct resource *fuse_resource;
	int device_files_created = 0;
	int return_code = 0;
	int i;
	const struct fsl_tmu_chip *tmu_chip;

	of_matched_dt_id = of_match_device(tmu_dt_ids, &pd->dev);
	if (!of_matched_dt_id) {
		dev_err(&pd->dev, "Cannot find a compatible device.\n");
		return -ENODEV;
	}

	tmu_chip = of_device_get_match_data(&pd->dev);

	tmu_dd = devm_kzalloc(&pd->dev, sizeof(struct tmu_driver_data),
			      GFP_KERNEL);
	if (!tmu_dd)
		return -ENOMEM;
	dev_set_drvdata(&pd->dev, tmu_dd);

	tmu_resource = platform_get_resource(pd, IORESOURCE_MEM, 0);
	if (!tmu_resource) {
		dev_err(&pd->dev, "Cannot obtain TMU resource.\n");
		return -ENODEV;
	}

	tmu_dd->tmu_registers = devm_ioremap_resource(&pd->dev, tmu_resource);
	if (IS_ERR(tmu_dd->tmu_registers)) {
		dev_err(&pd->dev, "Cannot map TMU registers.\n");
		return PTR_ERR(tmu_dd->tmu_registers);
	}

	if (tmu_chip->has_fuse) {
		fuse_resource = platform_get_resource(pd, IORESOURCE_MEM, 1);
		if (!fuse_resource) {
			dev_err(&pd->dev, "Cannot obtain TMU fuse resource.\n");
			return -ENODEV;
		}

		tmu_dd->fuse_base =
			devm_ioremap_resource(&pd->dev, fuse_resource);
		if (IS_ERR(tmu_dd->fuse_base)) {
			dev_err(&pd->dev, "Cannot map TMU fuse base.\n");
			return PTR_ERR(tmu_dd->fuse_base);
		}
	}

	if (tmu_chip->has_clk) {
		tmu_dd->clk = devm_clk_get(&pd->dev, "tsens");
		if (IS_ERR(tmu_dd->clk)) {
			dev_err(&pd->dev, "Cannot obtain clock: %d\n",
					return_code);
			return PTR_ERR(tmu_dd->clk);
		}

		return_code = clk_prepare_enable(tmu_dd->clk);
		if (return_code) {
			dev_err(&pd->dev, "Cannot enable clock: %d\n",
					return_code);
			return return_code;
		}
	}

	tmu_dd->temp_offset = tmu_chip->offset_to_celsius;

	tmu_dd->hwmon_device =
		hwmon_device_register_with_info(
			&pd->dev, DRIVER_NAME, tmu_dd, NULL, NULL);
	if (IS_ERR(tmu_dd->hwmon_device)) {
		return_code = PTR_ERR(tmu_dd->hwmon_device);
		dev_err(&pd->dev, "Cannot register hwmon device: %d\n",
			return_code);
		goto hwmon_register_failed;
	}

	for (i = 0; i < (tmu_chip->sites * NO_INTERF_FILES); i++) {
		return_code = device_create_file(tmu_dd->hwmon_device,
						 &dev_attrs[i]);
		if (return_code)
			goto device_create_file_failed;
		device_files_created++;
	}

	return_code = tmu_init_hw(&pd->dev, tmu_chip);
	if (return_code)
		goto calibration_failed;

	return 0;

calibration_failed:
device_create_file_failed:
	for (i = 0; i < device_files_created; i++)
		device_remove_file(tmu_dd->hwmon_device, &dev_attrs[i]);
	hwmon_device_unregister(tmu_dd->hwmon_device);
hwmon_register_failed:
	if (tmu_chip->has_clk)
		clk_disable_unprepare(tmu_dd->clk);

	return return_code;
}

static int tmu_remove(struct platform_device *pdev)
{
	struct tmu_driver_data *tmu_dd = platform_get_drvdata(pdev);
	int i;
	const struct fsl_tmu_chip *tmu_chip;

	tmu_chip = of_device_get_match_data(&pdev->dev);
	for (i = 0; i < (tmu_chip->sites * 4); i++)
		device_remove_file(tmu_dd->hwmon_device, &dev_attrs[i]);

	hwmon_device_unregister(tmu_dd->hwmon_device);
	if (tmu_chip->has_clk)
		clk_disable_unprepare(tmu_dd->clk);

	return 0;
}

static int __maybe_unused thermal_suspend(struct device *dev)
{
	struct tmu_driver_data *tmu_dd = dev_get_drvdata(dev);

	clk_disable_unprepare(tmu_dd->clk);

	return 0;
}

static int __maybe_unused thermal_resume(struct device *dev)
{
	struct tmu_driver_data *tmu_dd = dev_get_drvdata(dev);
	const struct fsl_tmu_chip *tmu_chip;
	int ret;

	tmu_chip = of_device_get_match_data(dev);

	ret = clk_prepare_enable(tmu_dd->clk);
	if (ret) {
		dev_err(dev, "Cannot enable clock: %d\n", ret);
		return ret;
	}

	return tmu_init_hw(dev, tmu_chip);
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

MODULE_AUTHOR("NXP Semiconductors, Inc.");
MODULE_DESCRIPTION("Thermal driver for NXP s32");
MODULE_LICENSE("GPL v2");
