// SPDX-License-Identifier: GPL-2.0
/* Copyright 2017-2018,2020 NXP */

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
#include <linux/mfd/syscon/s32v234-src.h>

#define DRIVER_NAME "s32tmu"

#ifdef CONFIG_S32GEN1_THERMAL
#include "s32gen1_thermal.h"
#elif CONFIG_S32V234_THERMAL
#include "s32v234_thermal.h"
#endif

#define TMU_SITE(n)			(1 << n)
#define CALIB_POINTS_MAX	12
#define NO_INTERF_FILES		4

/* SOC revision */
#define SIUL2_MIDR1_OFF				(0x00000004)
#define SIUL2_MIDR2_OFF				(0x00000008)

/* SIUL2_MIDR1 masks */
#define SIUL2_MIDR1_MINOR_MASK		(0xF << 0)
#define SIUL2_MIDR1_MAJOR_SHIFT		(4)
#define SIUL2_MIDR1_MAJOR_MASK		(0xF << SIUL2_MIDR1_MAJOR_SHIFT)

/* The hard-coded vectors are required to calibrate the thermal
 * monitoring unit. They were determined by experiments in a
 * thermally controlled environment and are specific to each
 * revision of the SoC.
 */
static const uint32_t rev1_calib_scfgr[] = {
	0x2C, 0x26, 0x3D, 0x6B, 0x9B, 0xC9, 0xF7, 0x112, 0x125,
	0x155, 0x16C, 0x172
};

static const uint32_t rev2_calib_scfgr[] = {
	0x2C, 0x59, 0xC6, 0x167
};

static const uint32_t rev1_calib_trcr[] = {
	0xE9, 0xE6, 0xF2, 0x10A, 0x123, 0x13B, 0x153, 0x161, 0x16B,
	0x184, 0x190, 0x193
};

static const uint32_t rev2_calib_trcr[] = {
	0xE9, 0x101, 0x13A, 0x18E
};

enum s32_calib_fuse {
	COLD_FUSE,
	WARM_FUSE
};

union CAL_FUSE_u {
	uint32_t R;
	struct {
		uint32_t CFG_DAC_TRIM0:5;
		uint32_t Reserved0:1;
		uint32_t CFG_DAC_TRIM1:5;
		uint32_t Reserved1:21;
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
	bool	 has_enable_bits;
	bool	 has_clk;
	bool	 has_sites;
	bool	 has_fuse;
	uint32_t enable_mask;
	int		 sites;
};

struct fsl_tmu_chip gen1_tmu = {
	.has_enable_bits = false,
	.has_clk = false,
	.has_sites = true, /* This is how you tell if Gen1 or V2 */
	.has_fuse = true,
	.enable_mask = 0x2,
	.sites = 3,
};

struct fsl_tmu_chip v234_tmu = {
	.has_enable_bits = true,
	.has_clk = true,
	.has_sites = false,
	.has_fuse = false,
	.enable_mask = 0x1,
	.sites = 1,
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
	uint8_t	revision;
	uint8_t calib_points;
};

static inline int get_siul2_midr1_minor(const void __iomem *siul20_base)
{
	return (readl(siul20_base + SIUL2_MIDR1_OFF) & SIUL2_MIDR1_MINOR_MASK);
}

static inline int get_siul2_midr1_major(const void __iomem *siul20_base)
{
	return ((readl(siul20_base + SIUL2_MIDR1_OFF) & SIUL2_MIDR1_MAJOR_MASK)
			>> SIUL2_MIDR1_MAJOR_SHIFT);
}

static u64 get_siul2_base_addr_from_fdt(char *node_name)
{
	struct device_node *node = NULL;
	const __be32 *siul2_base = NULL;
	u64 siul2_base_address = OF_BAD_ADDR;

	pr_debug("Searching %s MIDR registers in device-tree\n", node_name);
	node = of_find_node_by_name(NULL, node_name);
	if (!node) {
		pr_warn("Could not get %s node from device-tree\n", node_name);
		return siul2_base_address;
	}

	siul2_base = of_get_property(node, "midr-reg", NULL);
	if (siul2_base)
		siul2_base_address =
			of_translate_address(node, siul2_base);
	of_node_put(node);
	pr_debug("Resolved %s base address to 0x%llx\n", node_name,
			siul2_base_address);

	return siul2_base_address;
}

static int s32gen1_get_soc_revision(struct device *dev, uint8_t *rev)
{
	void __iomem *siul2_virt_addr;
	u32 raw_rev = 0;
	u64 siul2_base_address = OF_BAD_ADDR;

	siul2_base_address = get_siul2_base_addr_from_fdt("siul2_0");
	if (siul2_base_address == OF_BAD_ADDR) {
		dev_err(dev, "Can not obtain the base SIUL2 register\n");
		return -ENXIO;
	}
	siul2_virt_addr = ioremap_nocache(siul2_base_address, SZ_1K);
	if (siul2_virt_addr == NULL) {
		dev_err(dev, "Failed to map SIUL2 base address\n");
		return -EIO;
	}
	raw_rev =
		(get_siul2_midr1_major(siul2_virt_addr) << 8) |
		(get_siul2_midr1_minor(siul2_virt_addr) << 4);
	iounmap(siul2_virt_addr);

	if (raw_rev == 0)
		*rev = 1;
	else
		*rev = 2;

	return 0;
}

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

/* The Reference Manual Rev.2, 03/2017 states that the 8 bit
 * TEMP field of the RITSR and RATSR registers should be interpreted
 * as a signed integer when the temperature is below 25 degrees
 * Celsius and as an unsigned integer when the temperature is above 25
 * degrees. The fact that the sensor reading range is -40 to 125 degrees
 * allows us to simply cast to an int8_t, since within this range
 * interpreting the field as a signed integer always leads to the
 * correct result. If the value would ever fall outside this range,
 * the `Valid` bit of the registers would be cleared by hardware.
 * No mention about this for Gen1 (Documentation is wrong for TMU),
 * but the TMUs are similar enough to think it is the same, not to
 * mention that it seems to work in practice.
 */
static int tmu_immediate_temperature(struct device *dev,
					int8_t *immediate_temperature, int site)
{
	struct tmu_driver_data *tmu_dd = dev_get_drvdata(dev);
	union TMU_RITSR_u tmu_ritsr;

	tmu_ritsr.R = readl(tmu_dd->tmu_registers + TMU_RITSR(site));
	if (likely(tmu_ritsr.B.V == 0x1)) {
		*immediate_temperature = (int8_t)tmu_ritsr.B.TEMP;
		return is_out_of_range(*immediate_temperature);
	}

	return -1;
}

static int tmu_average_temperature(struct device *dev,
					int8_t *average_temperature, int site)
{
	struct tmu_driver_data *tmu_dd = dev_get_drvdata(dev);
	union TMU_RATSR_u tmu_ratsr;

	tmu_ratsr.R = readl(tmu_dd->tmu_registers + TMU_RATSR(site));
	if (likely(tmu_ratsr.B.V == 0x1)) {
		*average_temperature = (int8_t)tmu_ratsr.B.TEMP;
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
	int8_t immediate_temperature;
	int site;

	site = get_site_idx_from_label(attr->attr.name);
	if (site == -1)
		return snprintf(buffer, PAGE_SIZE, "Invalid site\n");
	if (tmu_immediate_temperature(dev, &immediate_temperature, site))
		return snprintf(buffer, PAGE_SIZE,
				"Invalid temperature reading!\n");
	else
		return snprintf(buffer, PAGE_SIZE, "%d",
				immediate_temperature * 1000);
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
	int8_t average_temperature;
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

static uint32_t get_calib_fuse_val(struct device *dev,
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

	uint32_t fuse_val_cold = get_calib_fuse_val(dev, COLD_FUSE);
	uint32_t fuse_val_warm = get_calib_fuse_val(dev, WARM_FUSE);

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
	struct tmu_driver_data *tmu_dd = dev_get_drvdata(dev);

	if (tmu_dd->revision == 1) {
		const static uint32_t warm_idxes[] = {7, 8, 9, 10, 11};
		const static uint32_t cold_idxes[] = {};

		get_calib_with_fuses(dev,
				calib_scfgr, calib_trcr,
				rev1_calib_scfgr, rev1_calib_trcr,
				warm_idxes, cold_idxes,
				ARRAY_SIZE(warm_idxes), ARRAY_SIZE(cold_idxes),
				sizeof(rev1_calib_scfgr));
	} else {
		const static uint32_t warm_idxes[] = {3};
		const static uint32_t cold_idxes[] = {0};

		get_calib_with_fuses(dev,
				calib_scfgr, calib_trcr,
				rev2_calib_scfgr, rev2_calib_trcr,
				warm_idxes, cold_idxes,
				ARRAY_SIZE(warm_idxes), ARRAY_SIZE(cold_idxes),
				sizeof(rev2_calib_scfgr));
	}
}

static int tmu_calibrate_s32gen1(struct device *dev)
{
	struct tmu_driver_data *tmu_dd = dev_get_drvdata(dev);
	union TMU_TCFGR_GEN1_u tmu_tcfgr;
	union TMU_SCFGR_u tmu_scfgr;
	union TMU_TRCR_u tmu_trcr;
	union TMU_CMCFG_u tmu_cmcfg;
	int i;
	uint32_t calib_scfgr[CALIB_POINTS_MAX];
	uint32_t calib_trcr[CALIB_POINTS_MAX];

	if (tmu_dd->calib_points > CALIB_POINTS_MAX) {
		dev_err(dev,
				"The allocated calibration tables are not large enough\n");
		return -ENOMEM;
	}
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

	for (i = 0; i < tmu_dd->calib_points; i++) {
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


static void tmu_calibrate_s32v234(struct device *dev)
{
	struct tmu_driver_data *tmu_dd = dev_get_drvdata(dev);
	union TMU_TCFGR_V234_u tmu_tcfgr;
	union TMU_SCFGR_u tmu_scfgr;

	int i, j;
	static int calibration_table[5][16] = {
		{  25, 30, 36, 42, 48, 54, 60, 66, 73,  80,  86, 93,  -1, -1,
		   -1, -1 },
		{  15, 22, 29, 36, 43, 51, 59, 67, 75,  83,  92, -1,  -1, -1,
		   -1, -1 },
		{  11, 19, 28, 37, 46, 56, 66, 76, 86,  -1,  -1, -1,  -1, -1,
		   -1, -1 },
		{   0, 9,  20, 30, 41, 53, 65, 77, 89, 102, 115, -1,  -1, -1,
		   -1, -1 },
		{  44, 48, 53, 57, 62, 67, 72, 77, 82,  88,  93, 99, 104, -1,
		   -1, -1 },
	};

	tmu_tcfgr.R = readl(tmu_dd->tmu_registers + TMU_TCFGR);
	tmu_scfgr.R = readl(tmu_dd->tmu_registers + TMU_SCFGR);

	for (i = 0; i < 5; i++) {
		tmu_tcfgr.B.Data1 = i;
		for (j = 0; calibration_table[i][j] != -1; j++) {

			tmu_tcfgr.B.Data0 = j;
			tmu_scfgr.B.SENSOR = calibration_table[i][j];

			writel(tmu_tcfgr.R, tmu_dd->tmu_registers + TMU_TCFGR);
			writel(tmu_scfgr.R, tmu_dd->tmu_registers + TMU_SCFGR);
		}
	}
}

static const struct of_device_id tmu_dt_ids[] = {
		{ .compatible = "fsl,s32v234-tmu", .data = &v234_tmu, },
		{ .compatible = "fsl,s32gen1-tmu", .data = &gen1_tmu, },
		{ /* end */ }
	};
MODULE_DEVICE_TABLE(of, tmu_dt_ids);

static int tmu_probe(struct platform_device *pd)
{
	const struct of_device_id *of_matched_dt_id;
	struct tmu_driver_data *tmu_dd;
	struct resource *tmu_resource;
	struct resource *fuse_resource;
	struct regmap *src_regmap = NULL;
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

	if (tmu_chip->has_enable_bits) {
		src_regmap =
			syscon_regmap_lookup_by_compatible("fsl,s32v234-src");
		if (!src_regmap) {
			dev_err(&pd->dev, "Cannot obtain SRC regmap.\n");
			return -ENODEV;
		}
	}

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

	if (s32gen1_get_soc_revision(&pd->dev, &tmu_dd->revision))
		goto revision_get_failed;
	if (tmu_dd->revision == 1)
		tmu_dd->calib_points = 12;
	else
		tmu_dd->calib_points = 4;

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

	if (tmu_chip->has_enable_bits) {
		return_code = regmap_update_bits(src_regmap, SRC_GPR4,
				   SRC_GPR4_TSENS_ENABLE_MASK, 0x1);
		if (return_code)
			goto regmap_update_bits_failed;
	}

	tmu_monitor_enable(&pd->dev, tmu_chip->enable_mask, false);
	if (tmu_chip->has_sites) {
		tmu_enable_sites(&pd->dev);
		if (tmu_calibrate_s32gen1(&pd->dev))
			goto calibration_failed;
	} else {
		tmu_calibrate_s32v234(&pd->dev);
	}
	tmu_configure_alpf(&pd->dev, alpf_0_5);
	tmu_measurement_interval(&pd->dev, mi_2_048s);
	tmu_monitor_enable(&pd->dev, tmu_chip->enable_mask, true);

	return 0;

calibration_failed:
regmap_update_bits_failed:
device_create_file_failed:
	for (i = 0; i < device_files_created; i++)
		device_remove_file(tmu_dd->hwmon_device, &dev_attrs[i]);
	hwmon_device_unregister(tmu_dd->hwmon_device);
revision_get_failed:
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

static struct platform_driver tmu_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.of_match_table = tmu_dt_ids,
	},
	.probe		= tmu_probe,
	.remove		= tmu_remove,
};
module_platform_driver(tmu_driver);

MODULE_AUTHOR("NXP Semiconductors, Inc.");
MODULE_DESCRIPTION("Thermal driver for NXP s32");
MODULE_LICENSE("GPL v2");
