// SPDX-License-Identifier: GPL-2.0
//
// Copyright 2016 Freescale Semiconductor, Inc.
// Copyright 2023-2024 NXP

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/sizes.h>
#include <linux/mutex.h>
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
#define TMR_IDLE	0x0
#define TMR_ME		0x80000000
#define TMR_ALPF	0x0c000000
#define TMR_ALPF_V2_DEFAULT	3
#define TMR_ALPF_V2_MAX		3
#define TMR_ALPF_V2_OFFSET	24

#define REGS_TMTMIR	0x008	/* Temperature measurement interval Register */
#define TMTMIR_DEFAULT	0x0000000f

#define REGS_V2_TMSR	0x008	/* monitor site register */

#define REGS_V2_TMTMIR	0x00c	/* Temperature measurement interval Register */

#define REGS_TIER	0x020	/* Interrupt Enable Register */
#define TIER_MASK	GENMASK(31, 24)
#define TIER_DISABLE	0x0
#define TIER_IHTTIE	BIT(31)
#define TIER_AHTTIE	BIT(30)
#define TIER_ILTTIE	BIT(28)
#define TIER_ALTTIE	BIT(27)
#define TIER_RTRCTIE	BIT(25)
#define TIER_FTRCTIE	BIT(24)

#define REGS_TIDR	0x24	/* Interrupt Detect */
#define TIDR_MASK	GENMASK(31, 24)
#define TIDR_IHTT	BIT(31)
#define TIDR_AHTT	BIT(30)
#define TIDR_ILTT	BIT(28)
#define TIDR_ALTT	BIT(27)
#define TIDR_RTRCT	BIT(25)
#define TIDR_FTRCT	BIT(24)

#define REGS_TIISCR	0x30	/* Interrupt Immediate Site Capture */
#define REGS_TIASCR	0x34	/* Interrupt Average Site Capture */

#define REGS_TICSCR	0x38	/* Interrupt Critical Site Capture (TICSCR) */

#define REGS_TMHTCR	0x40	/* Monitor High Temperature Capture */

#define REGS_TMRTRCR	0x48
#define TMRTRCR_VALID	BIT(31)
#define TMRTRCR_TEMP_MASK	GENMASK(7, 0)

#define REGS_TMFTRCR	0x4C
#define TMFTRCR_VALID	BIT(31)
#define TMFTRCR_TEMP_MASK	GENMASK(7, 0)

#define REGS_TMHTITR	0x50	/* Monitor Low Temperature Immediate
				 * Threshold
				 */
#define TMHTITR_EN	BIT(31)
#define TMHTITR_TEMP_MASK	GENMASK(8, 0)

#define REGS_TMHTATR	0x54	/* Monitor High Temperature Average
				 * Threshold
				 */
#define REGS_TMHTACTR	0x58	/* Monitor High Temperature Immediate
				 * Threshold
				 */

#define TMHTATR_EN	BIT(31)
#define REGS_TMLTITR	0x60	/* Monitor Low Temperature Immediate
				 * Threshold
				 */
#define TMLTITR_EN	BIT(31)
#define TMLTITR_TEMP_MASK	GENMASK(8, 0)

#define REGS_TMLTATR	0x64	/* Monitor Low Temperature Average
				 * Threshold
				 */
#define REGS_TMLTACTR	0x68	/* Monitor Low Temperature Average
				 * Critical Threshold
				 */

#define TMLTATR_EN	BIT(31)
#define REGS_TMRTRCTR	0x70	/* Monitor Rising Temperature
				 * Rate Critical Threshold
				 */
#define TMRTRCTR_EN	BIT(31)
#define TMRTRCTR_TEMP_MASK GENMASK(7, 0)

#define REGS_TMFTRCTR	0x74	/* Monitor Falling Temperature
				 * Rate Critical Threshold
				 */
#define TMFTRCTR_EN	BIT(31)
#define TMFTRCTR_TEMP_MASK	GENMASK(7, 0)

#define REGS_TTCFGR	0x080	/* Temperature Configuration Register */
#define REGS_TSCFGR	0x084	/* Sensor Configuration Register */

#define REGS_TRITSR(n)	(0x100 + 16 * (n)) /* Immediate Temperature
					    * Site Register
					    */
#define TRITSR_V	BIT(31)
#define TRITSR_TP5	BIT(9)
#define REGS_TRATSR(n)	(0x104 + 16 * (n)) /* Average Temperature
					    * Site Register
					    */
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
#define TCMCFG_OCM(tcmcfg)	(((tcmcfg) & BIT(30)) >> 30)
#define TCMCFG_RCTC(tcmcfg)	(((tcmcfg) & GENMASK(26, 24)) >> 24)
#define TCMCFG_CLK_DIV(tcmcfg)	(((tcmcfg) & GENMASK(15, 12)) >> 12)
#define TCMCFG_DFD(tcmcfg)	(((tcmcfg) & GENMASK(11, 10)) >> 10)

/* Experimentally determined */
#define S32CC_TCMCFG	(0x54004c00)

#define TMU_INVALID_SITE	-1

#define TMU_RATES_DIFF		2
#define TMU_TRCTR_MIN		4
#define TMU_TRCTR_MAX		200

/*
 * Thermal zone data
 */
struct qoriq_sensor {
	/* Two types of ids are used. id is the hardware sensor id, while
	 * virt_id is the id used in device tree.
	 */
	int				id;
	int				virt_id;
	struct thermal_zone_device	*tzd;
	bool				polling;
};

struct qoriq_tmu_data {
	int ver;
	struct regmap *regmap;
	struct clk *clk;
	struct qoriq_sensor	sensor[SITES_MAX * 2];
	struct device *dev;
	u32 sites;
	void *plat_data;
	u8 sites_max;
	int monitored_irq_site;
	bool initialized;
	/* If one site is monitored with IRQs, when get_temp is called for
	 * the rest of the sites, TMR and TMSR will be modified.
	 * A locking mechanism is necessary to serialize the access to these
	 * registers when tmu_get_temp for two not monitored sites is called
	 * in the same time.
	 */
	struct mutex lock;
	/* It takes time to read the updated temperature when a site is
	 * just added to the monitored sites.
	 */
	u32 read_delay;
	u32 alpf;
	unsigned int irq;
	u32 trctr;
};

struct s32cc_plat_data {
	u32 fuse_val;
	u32 *indexes;
	u32 *trcr;
	u32 *scfgr;
	u32 len;
};

struct qoirq_tmu_data {
	const struct regmap_access_table *ro_table;
	const struct regmap_access_table *rw_table;
	int (*init_and_calib)(struct qoriq_tmu_data *qdata);
	int (*get_read_delay)(struct qoriq_tmu_data *qdata);
	u8 sites_max;
};

static int qoriq_init_and_calib(struct qoriq_tmu_data *data);
static int s32cc_init_and_calib(struct qoriq_tmu_data *data);

static inline bool tmu_is_average_sensor(struct qoriq_sensor *qsensor)
{
	return qsensor->id != qsensor->virt_id;
}

/* If the sensor measures the immediate temperature, id and virt_id
 * have the same value. In average temperature case, id = virt_id - SITES_MAX.
 */
static inline int tmu_virt_to_hw_sensor_id(int id)
{
	return id >= SITES_MAX ? id - SITES_MAX : id;
}

static inline int tmu_hw_to_virt_sensor_id(int id, bool average)
{
	return average ? id + SITES_MAX : id;
}

static struct qoriq_tmu_data *qoriq_sensor_to_data(struct qoriq_sensor *s)
{
	return container_of(s, struct qoriq_tmu_data, sensor[s->virt_id]);
}

static inline u32 tmu_get_sites_mask(struct qoriq_tmu_data *qdata)
{
	return GENMASK(qdata->sites_max - 1, 0);
}

static void tmu_set_site_monitoring(struct qoriq_tmu_data *qdata, u32 sites)
{
	/* Disable monitoring. */
	regmap_update_bits(qdata->regmap, REGS_TMR, TMR_ME, TMR_IDLE);

	/* Update monitored sites. */
	regmap_update_bits(qdata->regmap, REGS_V2_TMSR,
			   tmu_get_sites_mask(qdata), sites);

	/* Enable monitoring back. */
	regmap_update_bits(qdata->regmap, REGS_TMR, TMR_ME, TMR_ME);
}

static int tmu_get_temp(void *p, int *temp)
{
	struct qoriq_sensor *qsensor = p;
	struct qoriq_tmu_data *qdata = qoriq_sensor_to_data(qsensor);
	u32 val, delay;
	bool not_monitored, take_lock;
	bool average_sensor = tmu_is_average_sensor(qsensor);
	int ret = 0;
	unsigned int reg_addr;

	not_monitored = qsensor->polling &&
		(qdata->monitored_irq_site != TMU_INVALID_SITE);
	/* Serialize get_temp and tmu_set_thresholds. */
	take_lock = qdata->monitored_irq_site != TMU_INVALID_SITE;

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

	if (take_lock)
		mutex_lock(&qdata->lock);

	regmap_read(qdata->regmap, REGS_TMR, &val);
	if (!(val & TMR_ME)) {
		ret = -EAGAIN;
		goto tmu_get_temp_unlock;
	}

	if (not_monitored) {
		/* If IRQs are used, only one site is monitored continuously.
		 * For every site monitored using polling, the monitoring
		 * should be enabled now and disabled back, after the
		 * temperature is read. Also, the interrupts should be
		 * disabled to avoid receiving interrupts for the current site.
		 * There will be no conflicts with tmu_set_thresholds since
		 * the interrupt line is masked, but the locking is still
		 * necessary to make sure tmu_get_temp isn't called in the
		 * same time for two not monitored sites.
		 */
		disable_irq(qdata->irq);

		tmu_set_site_monitoring(qdata, BIT(qsensor->id) |
					BIT(qdata->monitored_irq_site));
		/* Give TMU some time to read the temperature for
		 * the current sensor since the monitoring for it
		 * was just started.
		 * Double read_delay since there are two monitored
		 * sites.
		 */
		delay = qdata->read_delay * 2;
		usleep_range(delay, delay + 1000);
	}

	/* Figure out which temperature is requested, average or immediate. */
	reg_addr = average_sensor ?
		REGS_TRATSR(qsensor->id) : REGS_TRITSR(qsensor->id);
	if (regmap_read_poll_timeout(qdata->regmap,
				     reg_addr,
				     val,
				     val & TRITSR_V,
				     USEC_PER_MSEC,
				     10 * USEC_PER_MSEC)) {

		ret = -ENODATA;
		goto tmu_get_temp_out;
	}

	if (qdata->ver == TMU_VER1) {
		*temp = (val & GENMASK(7, 0)) * MILLIDEGREE_PER_DEGREE;
	} else {
		if (!average_sensor && (val & TRITSR_TP5))
			*temp = milli_kelvin_to_millicelsius((val & GENMASK(8, 0)) *
							     MILLIDEGREE_PER_DEGREE + 500);
		else
			*temp = kelvin_to_millicelsius(val & GENMASK(8, 0));
	}

tmu_get_temp_out:
	if (not_monitored) {
		/* Enable back the interrupts and monitor only
		 * monitored_irq_site.
		 * If any glitch happens in this time, there are two cases:
		 * - the critical rising and falling rates both arrived until
		 *   this point - there is nothing to be done, since
		 *   tmu_set_site_monitoring will clear both while disabling
		 *   the monitorization.
		 * - only one critical rate arrives until this point and the
		 *   opposite will arrive after the interrupts are enabled -
		 *   there is also nothing to be done, since, even if the
		 *   interrupt handler doesn't identify it as a glitch, the
		 *   temperature will have a normal value.
		 */
		tmu_set_site_monitoring(qdata, BIT(qdata->monitored_irq_site));
		/* Enable interrupts back. */
		enable_irq(qdata->irq);
	}

tmu_get_temp_unlock:
	if (take_lock)
		mutex_unlock(&qdata->lock);

	return ret;
}

static void tmu_set_thresholds(struct qoriq_tmu_data *qdata, int low,
			      int high, u8 sensor_id, bool average)
{
	bool set_high = true, set_low = true;
	u32 reg = 0;
	u32 site = BIT(sensor_id);
	unsigned int reg_addr;

	if (high == INT_MAX)
		set_high = false;

	if (low <= -INT_MAX)
		set_low = false;

	if (unlikely(!set_high && !set_low))
		return;

	/* Disable module monitoring. */
	regmap_update_bits(qdata->regmap, REGS_TMR, TMR_ME, TMR_IDLE);

	/* Clear the interrupt detect register, Interrupt Detect (TIDR). */
	regmap_update_bits(qdata->regmap, REGS_TIDR, TIDR_MASK, TIDR_MASK);

	/* Clear thresholds registers. */
	regmap_write(qdata->regmap, REGS_TMHTITR, 0);
	regmap_write(qdata->regmap, REGS_TMLTITR, 0);
	regmap_write(qdata->regmap, REGS_TMHTATR, 0);
	regmap_write(qdata->regmap, REGS_TMLTATR, 0);

	/* Clear the interrupt capture registers. */
	regmap_update_bits(qdata->regmap, REGS_TIISCR,
			   tmu_get_sites_mask(qdata), 0);
	regmap_update_bits(qdata->regmap, REGS_TIASCR,
			   tmu_get_sites_mask(qdata), 0);
	regmap_update_bits(qdata->regmap, REGS_TICSCR,
			   tmu_get_sites_mask(qdata), 0);

	regmap_update_bits(qdata->regmap, REGS_TIER, TIER_MASK, 0);

	/* Enable interrupt handling. */
	/* Enable Rising/Falling Temperature Rate Critical Threshold
	 * Interrupt used as workaround for TKT0635774.
	 */
	if (qdata->trctr)
		reg = TIER_RTRCTIE | TIER_FTRCTIE;

	if (set_high && !average)
		reg |= TIER_IHTTIE;
	if (set_low && !average)
		reg |= TIER_ILTTIE;
	if (set_high && average)
		reg |= TIER_AHTTIE;
	if (set_low && average)
		reg |= TIER_ALTTIE;

	regmap_update_bits(qdata->regmap, REGS_TIER, TIER_MASK, reg);

	if (qdata->trctr) {
		/* TKT0635774 */
		regmap_update_bits(qdata->regmap, REGS_TMRTRCTR,
				   TMRTRCTR_TEMP_MASK | TMRTRCTR_EN,
				   qdata->trctr | TMRTRCTR_EN);
		regmap_update_bits(qdata->regmap, REGS_TMFTRCTR,
				   TMFTRCR_TEMP_MASK | TMFTRCTR_EN,
				   qdata->trctr | TMFTRCTR_EN);
	}

	/* Write the appropriate values to the temperature threshold registers. */
	if (set_high) {
		reg_addr = !average ? REGS_TMHTITR : REGS_TMHTATR;
		regmap_update_bits(qdata->regmap, reg_addr,
				   TMHTITR_TEMP_MASK,
				   millicelsius_to_kelvin(high));
		regmap_update_bits(qdata->regmap, reg_addr,
				   TMHTITR_EN, TMHTITR_EN);
	}

	if (set_low) {
		reg_addr = !average ? REGS_TMLTITR : REGS_TMLTATR;
		regmap_update_bits(qdata->regmap, reg_addr,
				   TMLTITR_TEMP_MASK,
				   millicelsius_to_kelvin(low));
		regmap_update_bits(qdata->regmap, reg_addr,
				   TMLTITR_EN, TMLTITR_EN);
	}

	/* The current site will be the only one monitored. */
	regmap_update_bits(qdata->regmap, REGS_V2_TMSR,
			   tmu_get_sites_mask(qdata), site);

	/* If set_trips is called during initialization
	 * (devm_thermal_zone_of_sensor_register), let
	 * qoriq_tmu_register_tmu_zone enable the module.
	 */
	if (qdata->initialized)
		/* Enable the monitor mode. */
		regmap_update_bits(qdata->regmap, REGS_TMR, TMR_ME, TMR_ME);
}

static int tmu_set_trips(void *data, int low, int high)
{
	struct qoriq_sensor *sensor = data;
	struct qoriq_tmu_data *qdata = qoriq_sensor_to_data(sensor);

	if (!sensor)
		return -EINVAL;

	/* The first time it is called is during
	 * devm_thermal_zone_of_sensor_register. At this point, tzd will be
	 * NULL.
	 */
	if (!sensor->tzd)
		return 0;

	/* Nothing to be done if polling is activated. */
	if (sensor->tzd->polling_delay_jiffies)
		return 0;

	if (unlikely(sensor->id > qdata->sites_max)) {
		dev_err(qdata->dev,
			"trying to set trips for unsupported sensor %d.",
			sensor->id);
		return -EINVAL;
	}

	dev_dbg(qdata->dev, "%d:low(mdegC):%d, high(mdegC):%d\n",
		sensor->id, low, high);

	tmu_set_thresholds(qdata, low, high, sensor->id,
			   tmu_is_average_sensor(sensor));

	return 0;
}

static const struct thermal_zone_of_device_ops tmu_tz_ops = {
	.get_temp = tmu_get_temp,
	.set_trips = tmu_set_trips,
};

static void tmu_print_reg_range(struct qoriq_tmu_data *qdata,
				unsigned int start, unsigned int end)
{
	u32 val = 0;
	unsigned int i;

	for (i = start; i <= end; i = i + 4) {
		regmap_read(qdata->regmap, i, &val);
		dev_emerg(qdata->dev, "0x%x: %x", i, val);
	}
}

static void tmu_dump(struct qoriq_tmu_data *qdata)
{
	const struct qoirq_tmu_data *match_data =
		of_device_get_match_data(qdata->dev);
	const struct regmap_access_table *ro_table =
		match_data->ro_table;
	int i;

	for (i = 0; i < ro_table->n_yes_ranges; i++)
		tmu_print_reg_range(qdata,
				    ro_table->yes_ranges[i].range_min,
				    ro_table->yes_ranges[i].range_max);
}

static void tmu_handle_immediate_irq(struct qoriq_tmu_data *qdata, u32 tidr)
{
	struct qoriq_sensor *sensor;
	bool upper_set, lower_set;
	int temp = 0, temp_crit = 0;

	lower_set = tidr & TIDR_ILTT;
	upper_set = tidr & TIDR_IHTT;

	/* If an immediate threshold is passed, we will use the sensor that
	 * is responsible for the immediate temperature.
	 */
	sensor = &qdata->sensor[qdata->monitored_irq_site];

	if (upper_set || lower_set) {
		/* set_trips gets the lock back.
		 * If get_temp, is called until that time, it doesn't matter.
		 */
		thermal_zone_device_update(sensor->tzd,
					   THERMAL_EVENT_UNSPECIFIED);
		if (upper_set) {
			tmu_get_temp(sensor, &temp);
			sensor->tzd->ops->get_crit_temp(sensor->tzd,
							&temp_crit);
			if (temp >= temp_crit)
				tmu_dump(qdata);
		}
	} else {
		dev_err(qdata->dev,
			"No immediate threshold was exceeded TIDR = %x\n",
			tidr);
	}

}

static void tmu_handle_average_irq(struct qoriq_tmu_data *qdata, u32 tidr)
{
	struct qoriq_sensor *sensor;
	bool upper_set, lower_set;
	int virt_id = tmu_hw_to_virt_sensor_id(qdata->monitored_irq_site,
					       true);

	lower_set = tidr & TIDR_ALTT;
	upper_set = tidr & TIDR_AHTT;

	/* If an average threshold is passed, we will use the sensor that
	 * is responsible for the average temperature.
	 */
	sensor = &qdata->sensor[virt_id];

	if (upper_set || lower_set)
		/* set_trips gets the lock back.
		 * If get_temp, is called until that time, it doesn't matter.
		 */
		thermal_zone_device_update(sensor->tzd,
					   THERMAL_EVENT_UNSPECIFIED);
	else
		dev_err(qdata->dev,
			"No average threshold was exceeded TIDR = %x\n", tidr);
}

static void tmu_read_rate(struct regmap *map, unsigned int reg, u32 mask,
			    unsigned int *val)
{
	regmap_read(map, reg, val);
	if (*val & TMRTRCR_VALID)
		*val &= mask;
	else
		*val = 0;
}

static bool tmu_check_glitch(struct qoriq_tmu_data *qdata, bool rising)
{
	u32 tmrtrcr = 0, tmftrcr = 0;

	if (rising)
		/* Check rising rate. */
		tmu_read_rate(qdata->regmap, REGS_TMRTRCR,
			      TMRTRCR_TEMP_MASK, &tmrtrcr);
	else
		/* Check falling rate. */
		tmu_read_rate(qdata->regmap, REGS_TMFTRCR,
			      TMFTRCR_TEMP_MASK, &tmftrcr);

	/* Wait for one more sample to make sure it is not a glitch.
	 * If this is a gltch, the opposite critical rate capture will
	 * contain a similar value after the next TMU sample.
	 */
	usleep_range(qdata->read_delay, qdata->read_delay + 1000);

	if (rising)
		/* Check falling rate. */
		tmu_read_rate(qdata->regmap, REGS_TMFTRCR,
			      TMFTRCR_TEMP_MASK, &tmftrcr);
	else
		/* Check rising rate. */
		tmu_read_rate(qdata->regmap, REGS_TMRTRCR,
			      TMRTRCR_TEMP_MASK, &tmrtrcr);

	/* Check if the difference between the two rates indicates a glitch. */
	if (abs(tmrtrcr - tmftrcr) <= TMU_RATES_DIFF)
		return true;

	return false;
}

static irqreturn_t tmu_alarm_irq_thread(int irq, void *data)
{
	struct qoriq_tmu_data *qdata = data;
	u32 tidr = 0, tiiscr = 0, tiascr = 0, ticscr = 0;
	bool glitch = false;

	regmap_read(qdata->regmap, REGS_TIDR, &tidr);

	regmap_read(qdata->regmap, REGS_TIISCR, &tiiscr);
	regmap_read(qdata->regmap, REGS_TIASCR, &tiascr);
	regmap_read(qdata->regmap, REGS_TICSCR, &ticscr);

	if (unlikely(!(tidr & TIDR_MASK)))
		return IRQ_HANDLED;

	if (unlikely(qdata->monitored_irq_site == TMU_INVALID_SITE)) {
		dev_err(qdata->dev,
			"monitored_irq_site is invalid! TIDR = %x TIISCR = %x TIASCR = %x\n",
			tidr, tiiscr, tiascr);
		goto tmu_alarm_err;
	}

	if (ticscr & BIT(qdata->monitored_irq_site)) {
		/* tmu_check_glitch will detect a temperature glitch isolated
		 * to one single TMU sample and update the immediate temperature
		 * indirectly.
		 */
		if (tidr & TIDR_RTRCT)
			glitch = tmu_check_glitch(qdata, true);
		else if (tidr & TIDR_FTRCT)
			glitch = tmu_check_glitch(qdata, false);

		if (glitch) {
			/* Discard Rising/Falling Temperature Rate Capture. */
			regmap_write(qdata->regmap, REGS_TMRTRCR, TMRTRCR_VALID);
			regmap_write(qdata->regmap, REGS_TMFTRCR, TMFTRCR_VALID);
			goto tmu_alarm_err;
		}
	}

	/* Verify if any immediate/average threshold was exceeded. */
	if (tiiscr & BIT(qdata->monitored_irq_site))
		tmu_handle_immediate_irq(qdata, tidr);
	else if (tiascr & BIT(qdata->monitored_irq_site))
		tmu_handle_average_irq(qdata, tidr);

tmu_alarm_err:
	regmap_update_bits(qdata->regmap, REGS_TIISCR,
			   tmu_get_sites_mask(qdata), 0);
	regmap_update_bits(qdata->regmap, REGS_TIASCR,
			   tmu_get_sites_mask(qdata), 0);
	regmap_update_bits(qdata->regmap, REGS_TICSCR,
			   tmu_get_sites_mask(qdata), 0);
	regmap_update_bits(qdata->regmap, REGS_TIDR, TIDR_MASK, TIDR_MASK);

	return IRQ_HANDLED;
}

static int tmu_register_irq(struct platform_device *pdev,
			    struct qoriq_tmu_data *qdata)
{
	int irq, ret;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = devm_request_threaded_irq(&pdev->dev, irq, NULL,
			       tmu_alarm_irq_thread, IRQF_ONESHOT,
			       dev_name(&pdev->dev), qdata);
	if (ret)
		dev_err(&pdev->dev, "%s: failed to get irq\n",
			__func__);
	else
		qdata->irq = irq;

	return ret;
}

static int qoriq_tmu_register_tmu_zone(struct platform_device *pdev,
				       struct qoriq_tmu_data *qdata)
{
	struct device *dev = qdata->dev;
	int id, sites = 0;

	/* IDs from 0 to SITES_MAX - 1 will be used for the immediate
	 * temperature and the rest of them for the average temperature.
	 * Eg: sensor[0] -> immediate temperature for site 0
	 *     sensor[SITES_MAX] -> average temperature for site 0
	 * Both sensor[0] and sensor[SITES_MAX] will have the same
	 * id but different virt_id.
	 */
	for (id = 0; id < SITES_MAX * 2; id++) {
		struct thermal_zone_device *tzd;
		struct qoriq_sensor *sensor = &qdata->sensor[id];
		int ret;

		sensor->virt_id = id;
		sensor->id = tmu_virt_to_hw_sensor_id(id);

		tzd = devm_thermal_zone_of_sensor_register(dev, id,
							   sensor,
							   &tmu_tz_ops);
		ret = PTR_ERR_OR_ZERO(tzd);
		if (ret) {
			sensor->tzd = NULL;
			if (ret == -ENODEV)
				continue;

			return ret;
		}

		sensor->tzd = tzd;
		sensor->polling = !!tzd->polling_delay_jiffies;

		if (qdata->ver == TMU_VER1)
			sites |= 0x1 << (15 - sensor->id);
		else
			sites |= 0x1 << sensor->id;

		if (!sensor->polling) {
			if (qdata->ver == TMU_VER1) {
				dev_err(dev, "this version of TMU doesn't support interrupts.\n");
				return -EINVAL;
			}
			if (qdata->monitored_irq_site != TMU_INVALID_SITE) {
				dev_err(dev, "only one thermal zone can handle interrupts.\n");
				return -EINVAL;
			}

			ret = tmu_register_irq(pdev, qdata);
			if (ret < 0)
				return ret;

			/* Store the hardware sensor id.
			 * The driver will choose which qoriq_sensor to use
			 * based on the triggered interrupt type.
			 */
			qdata->monitored_irq_site = sensor->id;

			/* Call again set_trips, since the first time it was
			 * called, nothing was done.
			 */
			tmu_set_trips(sensor, tzd->prev_low_trip,
				      tzd->prev_high_trip);
		}

		if (devm_thermal_add_hwmon_sysfs(tzd))
			dev_warn(dev,
				 "Failed to add hwmon sysfs attributes\n");
	}

	if (sites) {
		if (qdata->ver == TMU_VER1) {
			regmap_write(qdata->regmap, REGS_TMR, TMR_ME | TMR_ALPF | sites);
		} else {
			/* We can monitor all sites, when polling is used for all
			 * of them.
			 * If interrupts are used, we can monitor only one site
			 * because TMU doesn't support different thresholds per
			 * site. In this case, TMSR will be updated in
			 * tmu_set_thresholds.
			 */
			if (qdata->monitored_irq_site == TMU_INVALID_SITE)
				regmap_update_bits(qdata->regmap, REGS_V2_TMSR,
						   tmu_get_sites_mask(qdata),
						   sites);
			else
				sites = BIT(qdata->monitored_irq_site);
			regmap_write(qdata->regmap, REGS_TMR,
				     TMR_ME | qdata->alpf << TMR_ALPF_V2_OFFSET);
		}
	}

	if (sites > 0)
		qdata->sites = sites;

	qdata->initialized = true;

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
		regmap_update_bits(data->regmap, REGS_V2_TMSR,
				   tmu_get_sites_mask(data), 0);

	/* Set update_interval */

	if (data->ver == TMU_VER1) {
		regmap_write(data->regmap, REGS_TMTMIR, TMTMIR_DEFAULT);
	} else {
		regmap_write(data->regmap, REGS_V2_TMTMIR, TMTMIR_DEFAULT);
	}
}

/* Compute the time necessary for TMU to read the temperature for one
 * site when the monitorization starts.
 * This value should be multiplied with the number of sites enabled
 * in TMSR.
 * The formula applies for continuous monitoring mode (TMTMIR is 0xF).
 */
static int s32cc_get_read_delay(struct qoriq_tmu_data *qdata)
{
	unsigned long clkrate = clk_get_rate(qdata->clk);
	u32 tcmcfg = 0;
	u8 dfd, ocm;
	u32 clk_div, rctc;
	u8 ocm_index, rctc_index, clk_div_index, dfd_index;
	const u8 ocm_values[] = {1, 2};
	const u32 rctc_values[] = {8, 16, 32, 64, 128, 256, 512, 1024};
	const u32 clk_div_values[] = {16, 32, 64, 128, 256, 512, 1024,
				2048, 4096, 8192, 16384};
	const u8 dfd_values[] = {1, 2, 4, 8};

	regmap_read(qdata->regmap, REGS_V2_TCMCFG, &tcmcfg);

	ocm_index = TCMCFG_OCM(tcmcfg);
	rctc_index = TCMCFG_RCTC(tcmcfg);
	clk_div_index = TCMCFG_CLK_DIV(tcmcfg);
	dfd_index = TCMCFG_DFD(tcmcfg);

	if (ocm_index >= ARRAY_SIZE(ocm_values) ||
			rctc_index >= ARRAY_SIZE(rctc_values) ||
			clk_div_index >= ARRAY_SIZE(clk_div_values) ||
			dfd_index >= ARRAY_SIZE(dfd_values)) {
		dev_err(qdata->dev,
			"Can't compute delay based on TCMCFG value %x.\n", tcmcfg);
		return -EINVAL;
	}

	ocm = ocm_values[ocm_index];
	rctc = rctc_values[rctc_index];
	clk_div = clk_div_values[clk_div_index];
	dfd = dfd_values[dfd_index];

	/* Computed after the following formula:
	 * 1s/ (FREQ / CLK_DIV) * (RCTC_VAL + 10) * DFD_VAL * OCM_VAL.
	 */
	return (USEC_PER_SEC * clk_div * (rctc + 10) * dfd * ocm / clkrate + 1);
}

static int qoriq_get_read_delay(struct qoriq_tmu_data *qdata)
{
	return 0;
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
	regmap_reg_range(REGS_TMR, REGS_V2_TMTMIR),
	regmap_reg_range(REGS_TIER, REGS_TIDR),
	regmap_reg_range(REGS_TIISCR, REGS_TICSCR),
	regmap_reg_range(REGS_TMHTCR, REGS_TMHTACTR),
	regmap_reg_range(REGS_TMLTITR, REGS_TMLTACTR),
	regmap_reg_range(REGS_TMRTRCTR, REGS_TMFTRCTR),
	regmap_reg_range(REGS_TTCFGR, REGS_TSCFGR),
	regmap_reg_range(REGS_TTRnCR(0), REGS_TTRnCR(15)),
	regmap_reg_range(REGS_V2_TCMCFG, REGS_V2_TCMCFG),
	/* Read-only */
	regmap_reg_range(REGS_IPBRR(0), REGS_IPBRR(1)),
	regmap_reg_range(REGS_TRITSR(0), REGS_TRATSR(0)),
	regmap_reg_range(REGS_TRITSR(1), REGS_TRATSR(1)),
	regmap_reg_range(REGS_TRITSR(2), REGS_TRATSR(2)),
};

static const struct regmap_access_table s32cc_rw_table = {
	.yes_ranges	= s32cc_yes_ranges,
	.n_yes_ranges	= ARRAY_SIZE(s32cc_yes_ranges),
};

static const struct regmap_access_table s32cc_ro_table = {
	.yes_ranges	= s32cc_yes_ranges,
	.n_yes_ranges	= ARRAY_SIZE(s32cc_yes_ranges),
};

static const struct qoirq_tmu_data qoriq_data = {
	.ro_table = &qoriq_rd_table,
	.rw_table = &qoriq_wr_table,
	.init_and_calib = qoriq_init_and_calib,
	.get_read_delay = qoriq_get_read_delay,
	.sites_max = SITES_MAX,
};

static const struct qoirq_tmu_data s32cc_data = {
	.ro_table = &s32cc_ro_table,
	.rw_table = &s32cc_rw_table,
	.init_and_calib = s32cc_init_and_calib,
	.get_read_delay = s32cc_get_read_delay,
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
	int ret;

	ret = read_nvmem_cell(data->dev, "tmu_fuse_val", fuse_val);
	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_err(data->dev, "Error reading fuse values\n");
		return ret;
	}

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
	u32 alpf, trctr = 0;

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

	data->initialized = false;
	data->monitored_irq_site = TMU_INVALID_SITE;
	mutex_init(&data->lock);

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

	if (data->ver == TMU_VER2) {
		data->alpf = TMR_ALPF_V2_DEFAULT;
		/* Try reading ALPF from device tree. */
		ret = of_property_read_u32(np, "tmu-alpf", &alpf);
		if (!ret) {
			if (alpf > TMR_ALPF_V2_MAX)
				dev_err(dev,
					"Invalid ALPF value %d. Default value (%d) will be used\n",
					alpf, TMR_ALPF_V2_DEFAULT);
			else
				data->alpf = alpf;
		}

		/* Try reading rising and falling threshold used as an
		 * workaround for TKT0635774.
		 */
		data->trctr = 0;
		ret = of_property_read_u32(np, "tmu-rate-filter", &trctr);
		if (!ret) {
			if (trctr > TMU_TRCTR_MAX ||
					(trctr < TMU_TRCTR_MIN && trctr != 0))
				dev_err(dev,
					"Invalid rising/falling rate threshold value %d. Please use a value between %d and %d or 0 if \"tmu-rate-filter\" isn't necessary. Default value (0) will be used\n",
					trctr, TMU_TRCTR_MIN, TMU_TRCTR_MAX);
			else
				data->trctr = trctr;
		}
		dev_info(dev,
			 "Critical rising/falling temperature threshold = %d degree(s)\n",
			 trctr);
	}

	qoriq_tmu_init_device(data);	/* TMU initialization */

	ret = match_data->init_and_calib(data);
	if (ret < 0)
		return ret;

	ret = match_data->get_read_delay(data);
	if (ret < 0)
		return ret;
	data->read_delay = ret;

	ret = qoriq_tmu_register_tmu_zone(pdev, data);
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

	mutex_lock(&data->lock);
	ret = regmap_update_bits(data->regmap, REGS_TMR, TMR_ME, 0);
	mutex_unlock(&data->lock);
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

		if (data->monitored_irq_site == TMU_INVALID_SITE)
			regmap_write(data->regmap, REGS_V2_TMSR, data->sites);
		regmap_write(data->regmap, REGS_V2_TMTMIR, TMTMIR_DEFAULT);
		regmap_write(data->regmap, REGS_TMR,
			     TMR_ME | data->alpf << TMR_ALPF_V2_OFFSET);

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
