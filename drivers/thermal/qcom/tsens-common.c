// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
 */

#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/nvmem-consumer.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include "tsens.h"

/* IRQ state, mask and clear */
struct tsens_irq_data {
	u32 up_viol;
	int up_thresh;
	u32 up_irq_mask;
	u32 up_irq_clear;
	u32 low_viol;
	int low_thresh;
	u32 low_irq_mask;
	u32 low_irq_clear;
	u32 crit_viol;
	u32 crit_thresh;
	u32 crit_irq_mask;
	u32 crit_irq_clear;
};

char *qfprom_read(struct device *dev, const char *cname)
{
	struct nvmem_cell *cell;
	ssize_t data;
	char *ret;

	cell = nvmem_cell_get(dev, cname);
	if (IS_ERR(cell))
		return ERR_CAST(cell);

	ret = nvmem_cell_read(cell, &data);
	nvmem_cell_put(cell);

	return ret;
}

/*
 * Use this function on devices where slope and offset calculations
 * depend on calibration data read from qfprom. On others the slope
 * and offset values are derived from tz->tzp->slope and tz->tzp->offset
 * resp.
 */
void compute_intercept_slope(struct tsens_priv *priv, u32 *p1,
			     u32 *p2, u32 mode)
{
	int i;
	int num, den;

	for (i = 0; i < priv->num_sensors; i++) {
		dev_dbg(priv->dev,
			"sensor%d - data_point1:%#x data_point2:%#x\n",
			i, p1[i], p2[i]);

		priv->sensor[i].slope = SLOPE_DEFAULT;
		if (mode == TWO_PT_CALIB) {
			/*
			 * slope (m) = adc_code2 - adc_code1 (y2 - y1)/
			 *	temp_120_degc - temp_30_degc (x2 - x1)
			 */
			num = p2[i] - p1[i];
			num *= SLOPE_FACTOR;
			den = CAL_DEGC_PT2 - CAL_DEGC_PT1;
			priv->sensor[i].slope = num / den;
		}

		priv->sensor[i].offset = (p1[i] * SLOPE_FACTOR) -
				(CAL_DEGC_PT1 *
				priv->sensor[i].slope);
		dev_dbg(priv->dev, "offset:%d\n", priv->sensor[i].offset);
	}
}

static inline u32 degc_to_code(int degc, const struct tsens_sensor *sensor)
{
	u32 code = (degc * sensor->slope + sensor->offset) / SLOPE_FACTOR;

	if (code > THRESHOLD_MAX_ADC_CODE)
		code = THRESHOLD_MAX_ADC_CODE;
	else if (code < THRESHOLD_MIN_ADC_CODE)
		code = THRESHOLD_MIN_ADC_CODE;
	pr_debug("%s, raw_code: 0x%x, degc:%d\n", __func__, code, degc);
	return code;
}

static inline int code_to_degc(u32 adc_code, const struct tsens_sensor *s)
{
	int degc, num, den;

	num = (adc_code * SLOPE_FACTOR) - s->offset;
	den = s->slope;

	if (num > 0)
		degc = num + (den / 2);
	else if (num < 0)
		degc = num - (den / 2);
	else
		degc = num;

	degc /= den;

	return degc;
}

static inline unsigned int tsens_ver(struct tsens_priv *priv)
{
	return priv->feat->ver_major;
}

static inline u32 irq_mask(u32 hw_id)
{
	return 1 << hw_id;
}

/**
 * tsens_set_interrupt_v1 - Disable an interrupt (enable = false)
 *                          Re-enable an interrupt (enable = true)
 */
static void tsens_set_interrupt_v1(struct tsens_priv *priv, const struct tsens_irq_data d,
				   u32 hw_id, enum tsens_irq_type irq_type, bool enable)
{
	if (enable) {
		switch (irq_type) {
		case UPPER:
			regmap_field_write(priv->rf[UP_INT_CLEAR_0 + hw_id], 0);
			break;
		case LOWER:
			regmap_field_write(priv->rf[LOW_INT_CLEAR_0 + hw_id], 0);
			break;
		default:
			dev_err(priv->dev, "%s: Invalid irq_type\n", __func__);
			break;
		}
	} else {
		switch (irq_type) {
		case UPPER:
			regmap_field_write(priv->rf[UP_INT_CLEAR_0 + hw_id], 1);
			break;
		case LOWER:
			regmap_field_write(priv->rf[LOW_INT_CLEAR_0 + hw_id], 1);
			break;
		default:
			dev_err(priv->dev, "%s: Invalid irq_type\n", __func__);
			break;
		}
	}
}

/**
 * tsens_set_interrupt_v2 - Disable an interrupt (enable = false)
 *                          Re-enable an interrupt (enable = true)
 */
static void tsens_set_interrupt_v2(struct tsens_priv *priv, const struct tsens_irq_data d,
				   u32 hw_id, enum tsens_irq_type irq_type, bool enable)
{
	if (enable) {
		switch (irq_type) {
		case UPPER:
			regmap_field_write(priv->rf[UP_INT_MASK_0 + hw_id], 0);
			break;
		case LOWER:
			regmap_field_write(priv->rf[LOW_INT_MASK_0 + hw_id], 0);
			break;
		case CRITICAL:
			regmap_field_write(priv->rf[CRIT_INT_MASK_0 + hw_id], 0);
			break;
		default:
			dev_err(priv->dev, "%s: Invalid irq_type\n", __func__);
			break;
		}
	} else {
		/* To reset the interrupt flag for a sensor:
		 *  1. Mask further interrupts for this sensor
		 *  2. Write 1 followed by 0 to clear the interrupt
		 */
		switch (irq_type) {
		case UPPER:
			regmap_field_write(priv->rf[UP_INT_MASK_0 + hw_id], 1);
			regmap_field_write(priv->rf[UP_INT_CLEAR_0 + hw_id], 1);
			regmap_field_write(priv->rf[UP_INT_CLEAR_0 + hw_id], 0);
			break;
		case LOWER:
			regmap_field_write(priv->rf[LOW_INT_MASK_0 + hw_id], 1);
			regmap_field_write(priv->rf[LOW_INT_CLEAR_0 + hw_id], 1);
			regmap_field_write(priv->rf[LOW_INT_CLEAR_0 + hw_id], 0);
			break;
		case CRITICAL:
			regmap_field_write(priv->rf[CRIT_INT_MASK_0 + hw_id], 1);
			regmap_field_write(priv->rf[CRIT_INT_CLEAR_0 + hw_id], 1);
			regmap_field_write(priv->rf[CRIT_INT_CLEAR_0 + hw_id], 0);
			break;
		default:
			dev_err(priv->dev, "%s: Invalid irq_type\n", __func__);
			break;
		}
	}
}

/**
 * tsens_set_interrupt - Disable an interrupt (enable = false)
 *                       Re-enable an interrupt (enable = true)
 */
static void tsens_set_interrupt(struct tsens_priv *priv, const struct tsens_irq_data d,
				u32 hw_id, enum tsens_irq_type irq_type, bool enable)
{
	/* FIXME: remove tsens_irq_data */
	dev_err(priv->dev, "[%u] %s: %s -> %s\n", hw_id, __func__,
		irq_type ? ((irq_type == 1) ? "UP" : "CRITICAL") : "LOW",
		enable ? "en" : "dis");
	if (tsens_ver(priv) > VER_1_X)
		tsens_set_interrupt_v2(priv, d, hw_id, irq_type, enable);
	else
		tsens_set_interrupt_v1(priv, d, hw_id, irq_type, enable);
}

/**
 * tsens_hw_to_mC - Return properly sign extended temperature in mCelsius,
 * whether in ADC code or deciCelsius depending on IP version.
 * This function handles the different widths of the signed integer across IPs.
 */
static int tsens_hw_to_mC(char *str, struct tsens_sensor *s, int field, int temp) {
	struct tsens_priv *priv = s->priv;
	u32 mask;

	if (priv->feat->adc) {
		/* Convert temperature from ADC code to milliCelsius */
		return code_to_degc(temp, s) * 1000;
	} else {
		mask = GENMASK(priv->fields[field].msb,
			       priv->fields[field].lsb) >> priv->fields[field].lsb;
		dev_dbg(priv->dev, "%s: mask: %d\n", str, fls(mask));
		/* Convert temperature from deciCelsius to milliCelsius */
		return sign_extend32(temp, fls(mask) - 1) * 100;

	}
}

static int tsens_read_irq_state(struct tsens_priv *priv, u32 hw_id,
				struct tsens_sensor *s, struct tsens_irq_data *d)
{
	int ret, up_temp, low_temp;

	if (hw_id > priv->num_sensors) {
		dev_err(priv->dev, "%s Invalid hw_id\n", __func__);
		return -EINVAL;
	}

	ret = regmap_field_read(priv->rf[UPPER_STATUS_0 + hw_id], &d->up_viol);
	if (ret)
		return ret;
	ret = regmap_field_read(priv->rf[UP_THRESH_0 + hw_id], &up_temp);
	if (ret)
		return ret;
	ret = regmap_field_read(priv->rf[LOWER_STATUS_0 + hw_id], &d->low_viol);
	if (ret)
		return ret;
	ret = regmap_field_read(priv->rf[LOW_THRESH_0 + hw_id], &low_temp);
	if (ret)
		return ret;
	ret = regmap_field_read(priv->rf[UP_INT_CLEAR_0 + hw_id], &d->up_irq_clear);
	if (ret)
		return ret;
	ret = regmap_field_read(priv->rf[LOW_INT_CLEAR_0 + hw_id], &d->low_irq_clear);
	if (ret)
		return ret;
	if (tsens_ver(priv) > VER_1_X) {
		ret = regmap_field_read(priv->rf[UP_INT_MASK_0 + hw_id], &d->up_irq_mask);
		if (ret)
			return ret;
		ret = regmap_field_read(priv->rf[LOW_INT_MASK_0 + hw_id], &d->low_irq_mask);
		if (ret)
			return ret;
	} else {
		/* No mask register on older TSENS */
		d->up_irq_mask = 0;
		d->low_irq_mask = 0;
	}

	d->up_thresh = tsens_hw_to_mC("upthresh", s, UP_THRESH_0, up_temp);
	d->low_thresh = tsens_hw_to_mC("lowthresh", s, LOW_THRESH_0, low_temp);

	if (d->up_viol || d->low_viol) {
		dev_err(priv->dev, "[%u] %s (viol): status: low(%u), up(%u) "
			"| clr: low(%u), up(%u) | thresh: (%d:%d) | mask: low(%u), up(%u)\n",
			hw_id, __func__, d->low_viol, d->up_viol, d->low_irq_clear, d->up_irq_clear,
			d->low_thresh, d->up_thresh, d->low_irq_mask, d->up_irq_mask);
	}
	return 0;
}

static inline u32 masked_irq(u32 hw_id, u32 mask, enum tsens_ver ver)
{
	if (ver > VER_1_X) {
		return mask & (1 << hw_id);
	} else {
		/* v1, v0.1 don't have a irq mask register */
		return 0;
	}
}

irqreturn_t tsens_irq_thread(int irq, void *data)
{
	struct tsens_priv *priv = data;
	int temp, ret, i;
	unsigned long flags;
	bool enable = true, disable = false;

	/* Check if any sensor raised an IRQ - for each sensor
	 * connected to the TSENS block
	 */
	for (i = 0; i < priv->num_sensors; i++) {
		struct tsens_sensor *s = &priv->sensor[i];
		struct tsens_irq_data d;
		u32 hw_id = s->hw_id;
		bool trigger = 0;

		if (IS_ERR(priv->sensor[i].tzd))
			continue;
		ret = get_temp_tsens_valid(s, &temp);
		if (ret) {
			dev_err(priv->dev, "[%u] %s error reading sensor\n", hw_id, __func__);
			continue;
		}

		dev_err(priv->dev, "[%u] %s: temp: %d\n", hw_id, __func__, temp);

		spin_lock_irqsave(&priv->ul_lock, flags);

		tsens_read_irq_state(priv, hw_id, s, &d);

		dev_dbg(priv->dev, "[%u] irq_thread, temp: %d\n", hw_id, temp);

		if (d.up_viol &&
		    !masked_irq(hw_id, d.up_irq_mask, tsens_ver(priv))) {
			tsens_set_interrupt(priv, d, hw_id, UPPER, disable);
			if (d.up_thresh > temp) {
				dev_err(priv->dev, "[%u] %s re-arm upper\n",
					priv->sensor[i].hw_id, __func__);
				/* unmask the interrupt for this sensor */
				tsens_set_interrupt(priv, d, hw_id, UPPER, enable);
			} else {
				trigger = 1;
				/* Keep irq masked */
			}
		} else if (d.low_viol &&
			   !masked_irq(hw_id, d.low_irq_mask, tsens_ver(priv))) {
			tsens_set_interrupt(priv, d, hw_id, LOWER, disable);
			if (d.low_thresh < temp) {
				dev_err(priv->dev, "[%u] %s re-arm low\n",
					priv->sensor[i].hw_id, __func__);
				/* unmask the interrupt for this sensor */
				tsens_set_interrupt(priv, d, hw_id, LOWER, enable);
			} else {
				trigger = 1;
				/* Keep irq masked */
			}
		}

		spin_unlock_irqrestore(&priv->ul_lock, flags);

		if (trigger) {
			dev_err(priv->dev, "[%u] %s: TZ update trigger (%d mC)\n",
				hw_id, __func__, temp);
			thermal_zone_device_update(priv->sensor[i].tzd,
						   THERMAL_EVENT_UNSPECIFIED);
		}

		/* TODO: REALLY??? */
		mb();
	}
	return IRQ_HANDLED;
}

/**
 * tsens_mC_to_hw - Return correct value to be written to threshold
 * registers, whether in ADC code or deciCelsius depending on IP version
 */
static int tsens_mC_to_hw(struct tsens_sensor *s, int temp)
{
	struct tsens_priv *priv = s->priv;

	if (priv->feat->adc) {
		/* milli to C to adc code */
		return degc_to_code(temp / 1000, s);
	} else {
		/* milli to deci C */
		return (temp / 100);
	}
}

int tsens_set_trips(void *_sensor, int low, int high)
{
	struct tsens_sensor *s = _sensor;
	struct tsens_priv *priv = s->priv;
	struct device *dev = priv->dev;
	struct tsens_irq_data d;
	unsigned long flags;
	int high_val, low_val, cl_high, cl_low;
	bool enable = true, disable = false;
	u32 hw_id = s->hw_id;

	dev_err(dev, "[%u] %s: proposed thresholds: (%d:%d)\n",
		hw_id, __func__, low, high);
	if (high == INT_MAX) {
		dev_err(dev, "[%u] %s: Recvd INT_MAX\n", s->hw_id, __func__);
		tsens_set_interrupt(priv, d, hw_id, UPPER, disable);
	}
	if (low == -INT_MAX) {
		dev_err(dev, "[%u] %s: Recvd -INT_MAX\n", s->hw_id, __func__);
		tsens_set_interrupt(priv, d, hw_id, LOWER, disable);
	}

	cl_high = clamp_val(high, -40000, 120000);
	cl_low = clamp_val(low, -40000, 120000);

	high_val = tsens_mC_to_hw(s, cl_high);
	low_val = tsens_mC_to_hw(s, cl_low);

	spin_lock_irqsave(&priv->ul_lock, flags);

	tsens_read_irq_state(priv, hw_id, s, &d);

	/* Write the new thresholds and clear the status */
	if (d.low_thresh != low_val) {
		regmap_field_write(priv->rf[LOW_THRESH_0 + hw_id], low_val);
		tsens_set_interrupt(priv, d, hw_id, LOWER, enable);
	}
	if (d.up_thresh != high_val) {
		regmap_field_write(priv->rf[UP_THRESH_0 + hw_id], high_val);
		tsens_set_interrupt(priv, d, hw_id, UPPER, enable);
	}

	spin_unlock_irqrestore(&priv->ul_lock, flags);

	dev_err(dev, "[%u] %s: threshold change (%d:%d)->(%d:%d)\n",
		s->hw_id, __func__, d.low_thresh, d.up_thresh, cl_low, cl_high);
	return 0;
}

int tsens_enable_irq(struct tsens_priv *priv)
{
	int ret;
	int val = (tsens_ver(priv) > VER_1_X) ? 7 : 1;

	ret = regmap_field_write(priv->rf[INT_EN], val);
	if (ret < 0)
		dev_err(priv->dev, "failed to enable interrupts\n");

	return ret;
}

void tsens_disable_irq(struct tsens_priv *priv)
{
	regmap_field_write(priv->rf[INT_EN], 0);
}

int get_temp_tsens_valid(struct tsens_sensor *s, int *temp)
{
	struct tsens_priv *priv = s->priv;
	int hw_id = s->hw_id;
	u32 temp_idx = LAST_TEMP_0 + hw_id;
	u32 valid_idx = VALID_0 + hw_id;
	u32 last_temp = 0, valid;
	int ret;

	ret = regmap_field_read(priv->rf[valid_idx], &valid);
	if (ret)
		return ret;
	while (!valid) {
		/* Valid bit is 0 for 6 AHB clock cycles.
		 * At 19.2MHz, 1 AHB clock is ~60ns.
		 * We should enter this loop very, very rarely.
		 */
		ndelay(400);
		ret = regmap_field_read(priv->rf[valid_idx], &valid);
		if (ret)
			return ret;
	}

	/* Valid bit is set, OK to read the temperature */
	ret = regmap_field_read(priv->rf[temp_idx], &last_temp);
	if (ret)
		return ret;

	*temp = tsens_hw_to_mC("get_temp", s, LAST_TEMP_0, last_temp);

	return 0;
}

int get_temp_common(struct tsens_sensor *s, int *temp)
{
	struct tsens_priv *priv = s->priv;
	int hw_id = s->hw_id;
	int last_temp = 0, ret;

	ret = regmap_field_read(priv->rf[LAST_TEMP_0 + hw_id], &last_temp);
	if (ret)
		return ret;

	*temp = code_to_degc(last_temp, s) * 1000;

	return 0;
}

#ifdef CONFIG_DEBUG_FS
static int dbg_sensors_show(struct seq_file *s, void *data)
{
	struct platform_device *pdev = s->private;
	struct tsens_priv *priv = platform_get_drvdata(pdev);
	int i;

	seq_printf(s, "max: %2d\nnum: %2d\n\n",
		   priv->feat->max_sensors, priv->num_sensors);

	seq_puts(s, "      id   slope  offset\n------------------------\n");
	for (i = 0;  i < priv->num_sensors; i++) {
		seq_printf(s, "%8d%8d%8d\n", priv->sensor[i].hw_id,
			   priv->sensor[i].slope, priv->sensor[i].offset);
	}

	return 0;
}

static int dbg_version_show(struct seq_file *s, void *data)
{
	struct platform_device *pdev = s->private;
	struct tsens_priv *priv = platform_get_drvdata(pdev);
	u32 maj_ver, min_ver, step_ver;
	int ret;

	ret = regmap_field_read(priv->rf[VER_MAJOR], &maj_ver);
	if (ret)
		return ret;
	ret = regmap_field_read(priv->rf[VER_MINOR], &min_ver);
	if (ret)
		return ret;
	ret = regmap_field_read(priv->rf[VER_STEP], &step_ver);
	if (ret)
		return ret;
	if (tsens_ver(priv) > VER_0_1)
		seq_printf(s, "%d.%d.%d\n", maj_ver, min_ver, step_ver);
	else
		seq_puts(s, "0.1.0\n");

	return 0;
}

DEFINE_SHOW_ATTRIBUTE(dbg_version);
DEFINE_SHOW_ATTRIBUTE(dbg_sensors);

static void tsens_debug_init(struct platform_device *pdev)
{
	struct tsens_priv *priv = platform_get_drvdata(pdev);
	struct dentry *root, *instance, *file;

	root = debugfs_lookup("tsens", NULL);
	if (!root)
		root = debugfs_create_dir("tsens", NULL);
	if (!root) {
		dev_err(&pdev->dev, "failed to create root debugfs directory\n");
		return;
	}
	priv->debug_root = root;

	instance = debugfs_create_dir(dev_name(&pdev->dev), priv->debug_root);
	if (!instance) {
		dev_err(&pdev->dev, "failed to create instance debugfs directory\n");
		goto cleanup;
	}
	priv->debug = instance;

	file = debugfs_create_file("version", 0444, priv->debug_root,
				   pdev, &dbg_version_fops);
	if (!file) {
		dev_err(&pdev->dev, "failed to create debugfs file: version\n");
		goto cleanup;
	}

	file = debugfs_create_file("sensors", 0444, priv->debug,
				   pdev, &dbg_sensors_fops);
	if (!file) {
		dev_err(&pdev->dev, "failed to create debugfs file: sensors\n");
		goto cleanup;
	}

	return;
cleanup:
	debugfs_remove_recursive(priv->debug_root);
	priv->debug = NULL;
	priv->debug_root = NULL;
}
#else
static inline void tsens_debug_init(struct platform_device *pdev) {}
#endif

static const struct regmap_config tsens_config = {
	.name		= "tm",
	.reg_bits	= 32,
	.val_bits	= 32,
	.reg_stride	= 4,
};

static const struct regmap_config tsens_srot_config = {
	.name		= "srot",
	.reg_bits	= 32,
	.val_bits	= 32,
	.reg_stride	= 4,
};

int __init init_common(struct tsens_priv *priv)
{
	void __iomem *tm_base, *srot_base;
	struct device *dev = priv->dev;
	struct resource *res;
	u32 enabled;
	int ret, i, j;
	struct platform_device *op = of_find_device_by_node(priv->dev->of_node);

	if (!op)
		return -EINVAL;

	if (op->num_resources > 1) {
		/* DT with separate SROT and TM address space */
		priv->tm_offset = 0;
		res = platform_get_resource(op, IORESOURCE_MEM, 1);
		srot_base = devm_ioremap_resource(&op->dev, res);
		if (IS_ERR(srot_base)) {
			ret = PTR_ERR(srot_base);
			goto err_put_device;
		}

		priv->srot_map = devm_regmap_init_mmio(dev, srot_base,
							&tsens_srot_config);
		if (IS_ERR(priv->srot_map)) {
			ret = PTR_ERR(priv->srot_map);
			goto err_put_device;
		}
	} else {
		/* old DTs where SROT and TM were in a contiguous 2K block */
		priv->tm_offset = 0x1000;
	}

	res = platform_get_resource(op, IORESOURCE_MEM, 0);
	tm_base = devm_ioremap_resource(&op->dev, res);
	if (IS_ERR(tm_base)) {
		ret = PTR_ERR(tm_base);
		goto err_put_device;
	}

	priv->tm_map = devm_regmap_init_mmio(dev, tm_base, &tsens_config);
	if (IS_ERR(priv->tm_map)) {
		ret = PTR_ERR(priv->tm_map);
		goto err_put_device;
	}

	if (tsens_ver(priv) > VER_0_1) {
		for (i = VER_MAJOR; i <= VER_STEP; i++) {
			priv->rf[i] = devm_regmap_field_alloc(dev, priv->srot_map,
							      priv->fields[i]);
			if (IS_ERR(priv->rf[i]))
				return PTR_ERR(priv->rf[i]);
		}
	}

	priv->rf[TSENS_EN] = devm_regmap_field_alloc(dev, priv->srot_map,
						     priv->fields[TSENS_EN]);
	if (IS_ERR(priv->rf[TSENS_EN])) {
		ret = PTR_ERR(priv->rf[TSENS_EN]);
		goto err_put_device;
	}
	ret = regmap_field_read(priv->rf[TSENS_EN], &enabled);
	if (ret)
		goto err_put_device;
	if (!enabled) {
		dev_err(dev, "tsens device is not enabled\n");
		ret = -ENODEV;
		goto err_put_device;
	}

	priv->rf[SENSOR_EN] = devm_regmap_field_alloc(dev, priv->srot_map,
						      priv->fields[SENSOR_EN]);
	if (IS_ERR(priv->rf[SENSOR_EN])) {
		ret = PTR_ERR(priv->rf[SENSOR_EN]);
		goto err_put_device;
	}
	/* now alloc regmap_fields in tm_map */
	for (i = 0, j = LAST_TEMP_0; i < priv->feat->max_sensors; i++, j++) {
		priv->rf[j] = devm_regmap_field_alloc(dev, priv->tm_map,
						      priv->fields[j]);
		if (IS_ERR(priv->rf[j])) {
			ret = PTR_ERR(priv->rf[j]);
			goto err_put_device;
		}
	}
	for (i = 0, j = VALID_0; i < priv->feat->max_sensors; i++, j++) {
		priv->rf[j] = devm_regmap_field_alloc(dev, priv->tm_map,
						      priv->fields[j]);
		if (IS_ERR(priv->rf[j])) {
			ret = PTR_ERR(priv->rf[j]);
			goto err_put_device;
		}
	}
	for (i = 0, j = UPPER_STATUS_0; i < priv->feat->max_sensors; i++, j++) {
		priv->rf[j] = devm_regmap_field_alloc(dev, priv->tm_map,
						      priv->fields[j]);
		if (IS_ERR(priv->rf[j])) {
			ret = PTR_ERR(priv->rf[j]);
			goto err_put_device;
		}
	}
	for (i = 0, j = LOWER_STATUS_0; i < priv->feat->max_sensors; i++, j++) {
		priv->rf[j] = devm_regmap_field_alloc(dev, priv->tm_map,
						      priv->fields[j]);
		if (IS_ERR(priv->rf[j])) {
			ret = PTR_ERR(priv->rf[j]);
			goto err_put_device;
		}
	}
	for (i = 0, j = CRITICAL_STATUS_0; i < priv->feat->max_sensors; i++, j++) {
		priv->rf[j] = devm_regmap_field_alloc(dev, priv->tm_map,
						      priv->fields[j]);
		if (IS_ERR(priv->rf[j])) {
			ret = PTR_ERR(priv->rf[j]);
			goto err_put_device;
		}
	}
	for (i = 0, j = UP_THRESH_0; i < priv->feat->max_sensors; i++, j++) {
		priv->rf[j] = devm_regmap_field_alloc(dev, priv->tm_map,
						      priv->fields[j]);
		if (IS_ERR(priv->rf[j])) {
			ret = PTR_ERR(priv->rf[j]);
			goto err_put_device;
		}
	}
	for (i = 0, j = LOW_THRESH_0; i < priv->feat->max_sensors; i++, j++) {
		priv->rf[j] = devm_regmap_field_alloc(dev, priv->tm_map,
						      priv->fields[j]);
		if (IS_ERR(priv->rf[j])) {
			ret = PTR_ERR(priv->rf[j]);
			goto err_put_device;
		}
	}
	for (i = 0, j = UP_INT_CLEAR_0; i < priv->feat->max_sensors; i++, j++) {
		priv->rf[j] = devm_regmap_field_alloc(dev, priv->tm_map,
						      priv->fields[j]);
		if (IS_ERR(priv->rf[j])) {
			ret = PTR_ERR(priv->rf[j]);
			goto err_put_device;
		}
	}
	for (i = 0, j = LOW_INT_CLEAR_0; i < priv->feat->max_sensors; i++, j++) {
		priv->rf[j] = devm_regmap_field_alloc(dev, priv->tm_map,
						      priv->fields[j]);
		if (IS_ERR(priv->rf[j])) {
			ret = PTR_ERR(priv->rf[j]);
			goto err_put_device;
		}
	}
	for (i = 0, j = UP_INT_MASK_0; i < priv->feat->max_sensors; i++, j++) {
		priv->rf[j] = devm_regmap_field_alloc(dev, priv->tm_map,
						      priv->fields[j]);
		if (IS_ERR(priv->rf[j])) {
			ret = PTR_ERR(priv->rf[j]);
			goto err_put_device;
		}
	}
	for (i = 0, j = LOW_INT_MASK_0; i < priv->feat->max_sensors; i++, j++) {
		priv->rf[j] = devm_regmap_field_alloc(dev, priv->tm_map,
						      priv->fields[j]);
		if (IS_ERR(priv->rf[j])) {
			ret = PTR_ERR(priv->rf[j]);
			goto err_put_device;
		}
	}

	priv->rf[INT_EN] = devm_regmap_field_alloc(dev, priv->tm_map,
						   priv->fields[INT_EN]);
	if (IS_ERR(priv->rf[INT_EN])) {
		ret = PTR_ERR(priv->rf[INT_EN]);
		goto err_put_device;
	}

	tsens_enable_irq(priv);
	spin_lock_init(&priv->ul_lock);

	tsens_debug_init(op);

	return 0;

err_put_device:
	put_device(&op->dev);
	return ret;
}
