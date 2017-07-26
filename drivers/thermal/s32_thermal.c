/* Copyright 2017-2018 NXP */

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

#define DRIVER_NAME "s32v234-tmu"

#define TMU_MR		0x0
#define TMU_SR		0x4
#define TMU_MTMIR	0x8
#define TMU_IER		0x20
#define TMU_IDR		0x24
#define TMU_MHTCR	0x40
#define TMU_MLTCR	0x44
#define TMU_MHTITR	0x50
#define TMU_MHTATR	0x54
#define TMU_MHTACTR	0x58
#define TMU_TCFGR	0x80
#define TMU_SCFGR	0x84
#define TMU_RITSR	0x100
#define TMU_RATSR	0x104
#define TMU_EUMR	0xF00

union TMU_MR_u {
	uint32_t R;
	struct {
		uint32_t Reserved0:26;
		uint32_t ALPF:2;
		uint32_t Reserved1:3;
		uint32_t ME:1;
	} B;
};

union TMU_SR_u {
	uint32_t R;
	struct {
		uint32_t Reserved0:28;
		uint32_t ORH:1;
		uint32_t ORL:1;
		uint32_t MIE:1;
		uint32_t Reserved1:1;
	} B;
};

union TMU_MTMIR_u {
	uint32_t R;
	struct {
		uint32_t TMI:4;
		uint32_t ORH:28;
	} B;
};

union TMU_IER_u {
	uint32_t R;
	struct {
		uint32_t Reserved0:29;
		uint32_t ATCTEIE:1;
		uint32_t ATTEIE:1;
		uint32_t ITTEIE:1;
	} B;
};

union TMU_IDR_u {
	uint32_t R;
	struct {
		uint32_t Reserved0:29;
		uint32_t ATCTE:1;
		uint32_t ATTE:1;
		uint32_t ITTE:1;
	} B;
};

union TMU_MHTCR_u {
	uint32_t R;
	struct {
		uint32_t TEMP:8;
		uint32_t Reserved0:23;
		uint32_t V:1;
	} B;
};

union TMU_MLTCR_u {
	uint32_t R;
	struct {
		uint32_t TEMP:8;
		uint32_t Reserved0:23;
		uint32_t V:1;
	} B;
};

union TMU_MHTITR_u {
	uint32_t R;
	struct {
		uint32_t TEMP:8;
		uint32_t Reserved0:23;
		uint32_t EN:1;
	} B;
};

union TMU_MHTATR_u {
	uint32_t R;
	struct {
		uint32_t TEMP:8;
		uint32_t Reserved0:23;
		uint32_t EN:1;
	} B;
};

union TMU_MHTACTR_u {
	uint32_t R;
	struct {
		uint32_t TEMP:8;
		uint32_t Reserved0:23;
		uint32_t EN:1;
	} B;
};

union TMU_TCFGR_u {
	uint32_t R;
	struct {
		uint32_t Data0:4;
		uint32_t Reserved0:12;
		uint32_t Data1:4;
		uint32_t Reserved1:12;
	} B;
};

union TMU_SCFGR_u {
	uint32_t R;
	struct {
		uint32_t SENSOR:7;
		uint32_t Reserved0:25;
	} B;
};

union TMU_RITSR_u {
	uint32_t R;
	struct {
		uint32_t TEMP:8;
		uint32_t Reserved0:23;
		uint32_t V:1;
	} B;
};

union TMU_RATSR_u {
	uint32_t R;
	struct {
		uint32_t TEMP:8;
		uint32_t Reserved0:23;
		uint32_t V:1;
	} B;
};

union TMU_EUMR_u {
	uint32_t R;
	struct {
		uint32_t Reserved0:21;
		uint32_t BG_CAL:2;
		uint32_t Reserved1:9;
	} B;
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
	struct device *hwmon_device;
};

static inline int is_out_of_range(int8_t temperature)
{
	return (temperature < -40 || temperature > 125);
}

/* The Reference Manual Rev.2, 03/2017 states that the 8 bit
TEMP field of the RITSR and RATSR registers should be interpreted
as a signed integer when the temperature is below 25 degrees
Celsius and as an unsigned integer when the temperature is above 25
degrees. The fact that the sensor reading range is -40 to 125 degrees
allows us to simply cast to an int8_t, since within this range
interpreting the field as a signed integer always leads to the
correct result. If the value would ever fall outside this range,
the `Valid` bit of the registers would be cleared by hardware.
*/

static int tmu_immediate_temperature(struct device *dev,
					int8_t *immediate_temperature)
{
	struct tmu_driver_data *tmu_dd = dev_get_drvdata(dev);
	union TMU_RITSR_u tmu_ritsr;

	tmu_ritsr.R = readl(tmu_dd->tmu_registers + TMU_RITSR);
	if (likely(tmu_ritsr.B.V == 0x1)) {
		*immediate_temperature = (int8_t)tmu_ritsr.B.TEMP;
		return is_out_of_range(*immediate_temperature);
	}

	return -1;
}

static int tmu_average_temperature(struct device *dev,
					int8_t *average_temperature)
{
	struct tmu_driver_data *tmu_dd = dev_get_drvdata(dev);
	union TMU_RATSR_u tmu_ratsr;

	tmu_ratsr.R = readl(tmu_dd->tmu_registers + TMU_RATSR);
	if (likely(tmu_ratsr.B.V == 0x1)) {
		*average_temperature = (int8_t)tmu_ratsr.B.TEMP;
		return is_out_of_range(*average_temperature);
	}

	return -1;
}

static ssize_t tmu_show_immediate_label(struct device *dev,
		struct device_attribute *attr, char *buffer)
{
	return snprintf(buffer, PAGE_SIZE, "Immediate temperature\n");
}

static ssize_t tmu_show_immediate(struct device *dev,
		struct device_attribute *attr, char *buffer)
{
	int8_t immediate_temperature;

	if (tmu_immediate_temperature(dev, &immediate_temperature))
		return snprintf(buffer, PAGE_SIZE,
				"Invalid temperature reading!\n");
	else
		return snprintf(buffer, PAGE_SIZE, "%d",
				immediate_temperature * 1000);
}

static ssize_t tmu_show_average_label(struct device *dev,
		struct device_attribute *attr, char *buffer)
{
	return snprintf(buffer, PAGE_SIZE, "Average temperature\n");
}

static ssize_t tmu_show_average(struct device *dev,
		struct device_attribute *attr, char *buffer)
{
	int8_t average_temperature;

	if (tmu_average_temperature(dev, &average_temperature))
		return snprintf(buffer, PAGE_SIZE,
				"Invalid temperature reading!\n");
	else
		return snprintf(buffer, PAGE_SIZE, "%d",
				average_temperature * 1000);
}

static struct device_attribute dev_attrs[] = {
	__ATTR(temp1_label, S_IRUGO, tmu_show_immediate_label, NULL),
	__ATTR(temp1_input, S_IRUGO, tmu_show_immediate,       NULL),
	__ATTR(temp2_label, S_IRUGO, tmu_show_average_label,   NULL),
	__ATTR(temp2_input, S_IRUGO, tmu_show_average,         NULL)
};

static void tmu_monitor_enable(struct device *dev, bool do_enable)
{
	struct tmu_driver_data *tmu_dd = dev_get_drvdata(dev);
	union TMU_MR_u tmu_mr;

	tmu_mr.R = readl(tmu_dd->tmu_registers + TMU_MR);

	if (do_enable)
		tmu_mr.B.ME = 0x1;
	else
		tmu_mr.B.ME = 0x0;

	writel(tmu_mr.R, tmu_dd->tmu_registers + TMU_MR);
}

static void tmu_measurement_interval(struct device *dev,
					enum measurement_interval_t mi)
{
	struct tmu_driver_data *tmu_dd = dev_get_drvdata(dev);
	union TMU_MTMIR_u tmu_mtmir;

	tmu_monitor_enable(dev, false);

	tmu_mtmir.R = readl(tmu_dd->tmu_registers + TMU_MTMIR);
	tmu_mtmir.B.TMI = mi;
	writel(tmu_mtmir.R, tmu_dd->tmu_registers + TMU_MTMIR);
}

/* Average temperature is calculated as:
   ALPF * Current_Temp + (1 - ALPF) * Average_Temp */
static void tmu_configure_alpf(struct device *dev, enum alpf_t alpf)
{
	struct tmu_driver_data *tmu_dd = dev_get_drvdata(dev);
	union TMU_MR_u tmu_mr;

	tmu_mr.R = readl(tmu_dd->tmu_registers + TMU_MR);
	tmu_mr.B.ALPF = alpf;
	writel(tmu_mr.R, tmu_dd->tmu_registers + TMU_MR);
}

static void tmu_calibrate(struct device *dev)
{
	struct tmu_driver_data *tmu_dd = dev_get_drvdata(dev);
	union TMU_TCFGR_u tmu_tcfgr;
	union TMU_SCFGR_u tmu_scfgr;

	int i, j;
	int calibration_table[5][16] = {
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

	tmu_monitor_enable(dev, false);

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
		{ .compatible = "fsl,s32v234-tmu", .data = NULL, },
		{ /* end */ }
	};
MODULE_DEVICE_TABLE(of, tmu_dt_ids);

static int tmu_probe(struct platform_device *pd)
{
	const struct of_device_id *of_matched_dt_id;
	struct tmu_driver_data *tmu_dd;
	struct resource *tmu_resource;
	struct regmap *src_regmap;
	int device_files_created = 0;
	int return_code = 0;
	int i;

	of_matched_dt_id = of_match_device(tmu_dt_ids, &pd->dev);
	if (!of_matched_dt_id) {
		dev_err(&pd->dev, "Cannot find a compatible device.\n");
		return -ENODEV;
	}

	tmu_dd = devm_kzalloc(&pd->dev, sizeof(struct tmu_driver_data),
			      GFP_KERNEL);
	if (!tmu_dd)
		return -ENOMEM;
	dev_set_drvdata(&pd->dev, tmu_dd);

	src_regmap = syscon_regmap_lookup_by_compatible("fsl,s32v234-src");
	if (!src_regmap) {
		dev_err(&pd->dev, "Cannot obtain SRC regmap.\n");
		return -ENODEV;
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

	tmu_dd->clk = devm_clk_get(&pd->dev, "tsens");
	if (IS_ERR(tmu_dd->clk)) {
		dev_err(&pd->dev, "Cannot obtain clock: %d\n", return_code);
		return PTR_ERR(tmu_dd->clk);
	}

	return_code = clk_prepare_enable(tmu_dd->clk);
	if (return_code) {
		dev_err(&pd->dev, "Cannot enable clock: %d\n", return_code);
		return return_code;
	}

	tmu_dd->hwmon_device =
		hwmon_device_register_with_info(
			&pd->dev, DRIVER_NAME, tmu_dd, NULL, NULL);
	if (IS_ERR(tmu_dd->hwmon_device)) {
		return_code = PTR_ERR(tmu_dd->hwmon_device);
		dev_err(&pd->dev, "Cannot register hwmon device: %d\n",
			return_code);
		goto hwmon_register_failed;
	}

	for (i = 0; i < ARRAY_SIZE(dev_attrs); i++) {
		return_code = device_create_file(tmu_dd->hwmon_device,
						 &dev_attrs[i]);
		if (return_code)
			goto device_create_file_failed;
		device_files_created++;
	}

	return_code = regmap_update_bits(src_regmap, SRC_GPR4,
			   SRC_GPR4_TSENS_ENABLE_MASK, 0x1);
	if (return_code)
		goto regmap_update_bits_failed;

	tmu_calibrate(&pd->dev);
	tmu_configure_alpf(&pd->dev, alpf_0_5);
	tmu_measurement_interval(&pd->dev, mi_2_048s);
	tmu_monitor_enable(&pd->dev, true);

	return 0;

regmap_update_bits_failed:
device_create_file_failed:
	for (i = 0; i < device_files_created; i++)
		device_remove_file(tmu_dd->hwmon_device, &dev_attrs[i]);
	hwmon_device_unregister(tmu_dd->hwmon_device);
hwmon_register_failed:
	clk_disable_unprepare(tmu_dd->clk);

	return return_code;
}

static int tmu_remove(struct platform_device *pdev)
{
	struct tmu_driver_data *tmu_dd = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < ARRAY_SIZE(dev_attrs); i++)
		device_remove_file(tmu_dd->hwmon_device, &dev_attrs[i]);

	hwmon_device_unregister(tmu_dd->hwmon_device);
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
MODULE_DESCRIPTION("Thermal driver for NXP s32v234");
MODULE_LICENSE("GPL v2");
