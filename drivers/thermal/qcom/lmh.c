// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2016-2017, The Linux Foundation. All rights reserved.
 * Copyright (c) 2018 Linaro Ltd.
 */

#define pr_fmt(fmt) "%s:%s " fmt, KBUILD_MODNAME, __func__

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/thermal.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/pm_opp.h>
#include <linux/cpu_cooling.h>
#include <linux/cpufreq.h>
#include <linux/atomic.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include <asm/smp_plat.h>
#include <asm/cacheflush.h>

#include <linux/qcom_scm.h>

#include "../thermal_core.h"
#include "lmh_dbg.h"

#define CREATE_TRACE_POINTS
#include <trace/events/lmh.h>

#define LMH_SUB_FN_THERMAL       0x54484D4C
#define LMH_SUB_FN_CRNT          0x43524E54
#define LMH_SUB_FN_REL           0x52454C00
#define LMH_SUB_FN_BCL           0x42434C00
#define LMH_SUB_FN_GENERAL       0x47454E00

#define LMH_ALGO_MODE_ENABLE     0x454E424C

#define LMH_HI_THRESHOLD         0x48494748
#define LMH_LOW_THRESHOLD        0x4C4F5700
#define LMH_ARM_THRESHOLD        0x41524D00

#define LMH_CLUSTER_0            0x6370302D
#define LMH_CLUSTER_1            0x6370312D

#define LMH_FREQ_CAP             0x46434150

#define LMH_TEMP_DEFAULT         75000
#define LMH_TEMP_HIGH_THRESH_MAX 120000
#define LMH_LOW_THRESHOLD_OFFSET 500
#define LMH_POLLING_DELAY_MS     10

/* Silver, aka LITTLE cluster */
#define LMH_CLUSTER_0_REQ        0x17D43704 /* OSM Rail 0, Clk 1 */
/* #define LMH_CLUSTER_0_INT_CLR    0x17D78808
   #define LMH_CLUSTER_0_MIN_FREQ   0x17D78BC0
*/

/* Gold, aka Big cluster */
#define LMH_CLUSTER_1_REQ        0x17D45F04 /* OSM Rail 1, Clk 0 */
/* #define LMH_CLUSTER_1_INT_CLR    0x17D70808
   #define LMH_CLUSTER_1_MIN_FREQ   0x17D70BC0
*/
#define LMH_INT_CLR_OFFSET	0x8
#define LMH_MIN_FREQ_OFFSET	0x3c0

#define dcvsh_get_frequency(_val, _max) do {	\
		_max = (_val) & 0x3FF;		\
		_max *= 19200;			\
	} while (0)
#define FREQ_KHZ_TO_HZ(_val) ((_val) * 1000)
#define FREQ_HZ_TO_KHZ(_val) ((_val) / 1000)

enum lmh_hw_trips {
	LMH_TRIP_ARM,
	LMH_TRIP_HI,
	LMH_TRIP_MAX,
};

struct __lmh_cdev_data {
	struct thermal_cooling_device *cdev;
	u32 max_freq;
	u32 min_freq;
};

struct lmh_thermal_hw {
	char			sensor_name[THERMAL_NAME_LENGTH];
	struct regmap		*map;
	u32			affinity;
	u32			temp_limits[LMH_TRIP_MAX];
	int			irq;
	void			*osm_hw_reg;
	cpumask_t		core_map;
	struct timer_list	poll_timer;
	unsigned long		max_freq;
	unsigned long		min_freq;
	unsigned long		hw_freq_limit;
	struct device_attribute lmh_freq_attr;
	struct list_head	list;
	atomic_t		is_irq_enabled;
	struct mutex		access_lock;
	struct __lmh_cdev_data	*cdev_data;
	struct regulator	*isens_reg;
};

LIST_HEAD(lmh_thermal_hw_list);
DEFINE_MUTEX(lmh_thermal_list_access);

static int lmh_thermal_get_freq_limits(uint32_t cpu, unsigned long *max_freq,
				       unsigned long *min_freq)
{
	unsigned long freq_ceil = UINT_MAX, freq_floor = 0;
	struct device *cpu_dev = NULL;
	struct dev_pm_opp *opp;
	int ret = 0;

	cpu_dev = get_cpu_device(cpu);
	if (!cpu_dev) {
		pr_err("Error getting CPU%d device\n", cpu);
		return -ENODEV;
	}

	rcu_read_lock();
	opp = dev_pm_opp_find_freq_floor(cpu_dev, &freq_ceil);
	if (!IS_ERR(opp)) {
		*max_freq = freq_ceil / 1000;
		dev_pm_opp_put(opp);
	}
	opp = dev_pm_opp_find_freq_ceil(cpu_dev, &freq_floor);
	if (!IS_ERR(opp)) {
		*min_freq = freq_floor / 1000;
		dev_pm_opp_put(opp);
	}
	rcu_read_unlock();

	return ret;
}

static unsigned long lmh_mitigation_notify(struct lmh_thermal_hw *hw)
{
	uint32_t val = 0;
	struct device *cpu_dev = NULL;
	unsigned long freq_val, max_limit = 0;
	struct dev_pm_opp *opp_entry;

	val = readl_relaxed(hw->osm_hw_reg);
	dcvsh_get_frequency(val, max_limit);
	cpu_dev = get_cpu_device(cpumask_first(&hw->core_map));
	if (!cpu_dev) {
		pr_err("Error in get CPU%d device\n",
		       cpumask_first(&hw->core_map));
		goto notify_exit;
	}

	pr_debug("CPU:%d max value read:%lu\n",
		 cpumask_first(&hw->core_map),
		 max_limit);
	freq_val = FREQ_KHZ_TO_HZ(max_limit);
	rcu_read_lock();
	opp_entry = dev_pm_opp_find_freq_floor(cpu_dev, &freq_val);
	/*
	 * Hardware mitigation frequency can be lower than the lowest
	 * possible CPU frequency. In that case freq floor call will
	 * fail with -ERANGE and we need to match to the lowest
	 * frequency using freq_ceil.
	 */
	if (IS_ERR(opp_entry) && PTR_ERR(opp_entry) == -ERANGE) {
		opp_entry = dev_pm_opp_find_freq_ceil(cpu_dev, &freq_val);
		if (IS_ERR(opp_entry))
			dev_err(cpu_dev, "frequency:%lu. opp error:%ld\n",
				freq_val, PTR_ERR(opp_entry));
	}
	rcu_read_unlock();
	max_limit = FREQ_HZ_TO_KHZ(freq_val);

	// TODO: update scheduler capacity values for the core
	// sched_update_cpu_freq_min_max(&hw->core_map, 0, max_limit);
	pr_debug("CPU:%d max limit:%lu\n", cpumask_first(&hw->core_map),
		 max_limit);
	trace_lmh_dcvs_freq(cpumask_first(&hw->core_map), max_limit);

 notify_exit:
	hw->hw_freq_limit = max_limit;
	return max_limit;
}

static void lmh_thermal_poll(struct timer_list *t)
{
	int ret;
	unsigned long max_limit = 0;
	struct lmh_thermal_hw *hw = from_timer(hw, t, poll_timer);

	if (hw->max_freq == UINT_MAX)
		lmh_thermal_get_freq_limits(cpumask_first(&hw->core_map),
					    &hw->max_freq, &hw->min_freq);
	max_limit = lmh_mitigation_notify(hw);
	if (max_limit >= hw->max_freq) {
		del_timer(&hw->poll_timer);
		ret = regmap_write(hw->map, LMH_INT_CLR_OFFSET, 0xFF);
		atomic_set(&hw->is_irq_enabled, 1);
		enable_irq(hw->irq);
	} else {
		mod_timer(&hw->poll_timer,
			  jiffies + msecs_to_jiffies(LMH_POLLING_DELAY_MS));
	}
}

static void lmh_thermal_notify(struct lmh_thermal_hw *hw)
{
	if (atomic_dec_and_test(&hw->is_irq_enabled)) {
		disable_irq_nosync(hw->irq);
		lmh_mitigation_notify(hw);
		mod_timer(&hw->poll_timer,
			  jiffies + msecs_to_jiffies(LMH_POLLING_DELAY_MS));
	}
}

static irqreturn_t lmh_thermal_handle_isr(int irq, void *data)
{
	struct lmh_thermal_hw *hw = data;

	lmh_thermal_notify(hw);

	return IRQ_HANDLED;
}

static int lmh_get_temp(void *data, int *val)
{
	/*
	 * LMH DCVSh hardware doesn't support temperature read.
	 * return a default value for the thermal core to aggregate
	 * the thresholds
	 */
	*val = LMH_TEMP_DEFAULT;

	return 0;
}

static int lmh_set_trips(void *data, int low, int high)
{
	struct lmh_thermal_hw *hw = (struct lmh_thermal_hw *)data;
	int ret = 0;

	if (high >= LMH_TEMP_HIGH_THRESH_MAX || low < 0) {
		pr_err("Value out of range low:%d high:%d\n",
		       low, high);
		return -EINVAL;
	}

	/* Sanity check limits before writing to the hardware */
	if (low >= high)
		return -EINVAL;

	hw->temp_limits[LMH_TRIP_HI] = (uint32_t)high;
	hw->temp_limits[LMH_TRIP_ARM] = (uint32_t)low;

	ret =  qcom_scm_lmh_write(hw->affinity, LMH_SUB_FN_THERMAL,
				  LMH_ARM_THRESHOLD, low, 0, 0);
	if (ret)
		return ret;
	ret =  qcom_scm_lmh_write(hw->affinity, LMH_SUB_FN_THERMAL,
				  LMH_HI_THRESHOLD, high, 0, 0);
	if (ret)
		return ret;
	ret =  qcom_scm_lmh_write(hw->affinity, LMH_SUB_FN_THERMAL,
				  LMH_LOW_THRESHOLD,
				  high - LMH_LOW_THRESHOLD_OFFSET,
				  0, 0);
	if (ret)
		return ret;

	return ret;
}

static struct lmh_thermal_hw *get_lmh_hw_from_cpu(int cpu)
{
	struct lmh_thermal_hw *hw;

	mutex_lock(&lmh_thermal_list_access);
	list_for_each_entry(hw, &lmh_thermal_hw_list, list) {
		if (cpumask_test_cpu(cpu, &hw->core_map)) {
			mutex_unlock(&lmh_thermal_list_access);
			return hw;
		}
	}
	mutex_unlock(&lmh_thermal_list_access);

	return NULL;
}

static int lmh_set_max_limit(int cpu, u32 freq)
{
	struct lmh_thermal_hw *hw = get_lmh_hw_from_cpu(cpu);
	int ret = 0, cpu_idx, idx = 0;
	u32 max_freq = U32_MAX;

	if (!hw)
		return -EINVAL;

	mutex_lock(&hw->access_lock);
	for_each_cpu(cpu_idx, &hw->core_map) {
		if (cpu_idx == cpu)
			/*
			 * If there is no limits restriction for CPU scaling max
			 * frequency, vote for a very high value. This will allow
			 * the CPU to use the boost frequencies.
			 */
			hw->cdev_data[idx].max_freq =
				(freq == hw->max_freq) ? U32_MAX : freq;
		if (max_freq > hw->cdev_data[idx].max_freq)
			max_freq = hw->cdev_data[idx].max_freq;
		idx++;
	}
	ret = qcom_scm_lmh_write(hw->affinity, LMH_SUB_FN_THERMAL,
				 LMH_FREQ_CAP, max_freq,
				 (max_freq == U32_MAX) ? 0 : 1, 1);
	mutex_unlock(&hw->access_lock);
	lmh_thermal_notify(hw);

	return ret;
}

static int lmh_set_min_limit(int cpu, u32 freq)
{
	struct lmh_thermal_hw *hw = get_lmh_hw_from_cpu(cpu);
	int cpu_idx, idx = 0;
	u32 min_freq = 0;
	int ret;

	if (!hw)
		return -EINVAL;

	mutex_lock(&hw->access_lock);
	for_each_cpu(cpu_idx, &hw->core_map) {
		if (cpu_idx == cpu)
			hw->cdev_data[idx].min_freq = freq;
		if (min_freq < hw->cdev_data[idx].min_freq)
			min_freq = hw->cdev_data[idx].min_freq;
		idx++;
	}
	if (min_freq != hw->min_freq)
		ret = regmap_write(hw->map, LMH_MIN_FREQ_OFFSET, 0x01);
	else
		ret = regmap_write(hw->map, LMH_MIN_FREQ_OFFSET, 0x00);
	mutex_unlock(&hw->access_lock);

	return 0;
}
static struct cpu_cooling_ops cd_ops = {
	.ceil_limit = lmh_set_max_limit,
	.floor_limit = lmh_set_min_limit,
};

static int lmh_cpu_online(unsigned int online_cpu)
{
	struct lmh_thermal_hw *hw = get_lmh_hw_from_cpu(online_cpu);
	struct cpufreq_policy *policy;
	unsigned int idx = 0, cpu = 0;

	if (!hw)
		return 0;

	for_each_cpu(cpu, &hw->core_map) {
		if (cpu != online_cpu) {
			idx++;
			continue;
		} else if (hw->cdev_data[idx].cdev) {
			return 0;
		}
		hw->cdev_data[idx].max_freq = U32_MAX;
		hw->cdev_data[idx].min_freq = 0;
		policy = cpufreq_cpu_get(cpu);
		if (!policy) {
			pr_debug("%s: CPUFreq policy not found\n", __func__);
			return -EPROBE_DEFER;
		}
		hw->cdev_data[idx].cdev = cpufreq_platform_cooling_register(policy, &cd_ops);
		if (IS_ERR_OR_NULL(hw->cdev_data[idx].cdev)) {
			pr_err("CPU:%u cooling device register error:%ld\n",
			       cpu, PTR_ERR(hw->cdev_data[idx].cdev));
			hw->cdev_data[idx].cdev = NULL;
		} else {
			pr_debug("CPU:%u cooling device registered\n", cpu);
		}
		break;
	}

	return 0;
}

static void lmh_isens_vref_ldo_init(struct platform_device *pdev,
				    struct lmh_thermal_hw *hw)
{
	int ret = 0;
	uint32_t settings[3];

	hw->isens_reg = devm_regulator_get(&pdev->dev, "isens_vref");
	if (IS_ERR_OR_NULL(hw->isens_reg)) {
		if (PTR_ERR(hw->isens_reg) == -ENODEV)
			return;

		pr_err("Regulator:isens_vref init error:%ld\n",
		       PTR_ERR(hw->isens_reg));
		return;
	}
	ret = of_property_read_u32_array(pdev->dev.of_node,
					 "isens-vref-settings",
					 settings, 3);
	if (ret) {
		pr_err("Regulator:isens_vref settings read error:%d\n",
		       ret);
		devm_regulator_put(hw->isens_reg);
		return;
	}
	ret = regulator_set_voltage(hw->isens_reg, settings[0], settings[1]);
	if (ret) {
		pr_err("Regulator:isens_vref set voltage error:%d\n", ret);
		devm_regulator_put(hw->isens_reg);
		return;
	}
	ret = regulator_set_load(hw->isens_reg, settings[2]);
	if (ret) {
		pr_err("Regulator:isens_vref set load error:%d\n", ret);
		devm_regulator_put(hw->isens_reg);
		return;
	}
	if (regulator_enable(hw->isens_reg)) {
		pr_err("Failed to enable regulator:isens_vref\n");
		devm_regulator_put(hw->isens_reg);
		return;
	}
}

static ssize_t
lmh_freq_limit_show(struct device *dev, struct device_attribute *devattr,
		    char *buf)
{
	struct lmh_thermal_hw *hw = container_of(devattr,
						 struct lmh_thermal_hw,
						 lmh_freq_attr);

	return snprintf(buf, PAGE_SIZE, "%lu\n", hw->hw_freq_limit);
}

static const struct regmap_config lmh_regconfig = {
	.reg_bits	= 32,
	.val_bits	= 32,
	.reg_stride	= 4,
};

static struct thermal_zone_of_device_ops lmh_sensor_ops = {
	.get_temp   = lmh_get_temp,
	.set_trips  = lmh_set_trips,
};

static int lmh_thermal_probe(struct platform_device *pdev)
{
	int ret;
	int cpu;
	int affinity = -1;
	struct resource *res;
	void __iomem *base;
	struct lmh_thermal_hw *hw;
	struct thermal_zone_device *tzdev;
	struct device_node *cpu_node, *lmh_node;
	struct device_node *dn = pdev->dev.of_node;
	uint32_t request_reg;
	unsigned long max_freq, min_freq;
	cpumask_t lmh_cpu_mask = { CPU_BITS_NONE };

	for_each_possible_cpu(cpu) {
		cpu_node = of_cpu_device_node_get(cpu);
		if (!cpu_node)
			continue;
		lmh_node = of_parse_phandle(cpu_node, "qcom,lmh-thermal", 0);
		if (lmh_node == dn) {
			cpumask_set_cpu(cpu, &(lmh_cpu_mask));
		}
		of_node_put(cpu_node);
		of_node_put(lmh_node);
	}

	/*
	 * We return error if none of the CPUs have
	 * reference to our LMH node
	 */
	if (cpumask_empty(&lmh_cpu_mask))
		return -EINVAL;

	/* FIXME: Make this work for cpus with different OPP tables */
	ret = lmh_thermal_get_freq_limits(cpumask_first(&lmh_cpu_mask), &max_freq,
					  &min_freq);
	if (ret)
		return ret;
	hw = devm_kzalloc(&pdev->dev, sizeof(*hw), GFP_KERNEL);
	if (!hw)
		return -ENOMEM;
	hw->cdev_data = devm_kcalloc(&pdev->dev, cpumask_weight(&lmh_cpu_mask),
				     sizeof(*hw->cdev_data),
				     GFP_KERNEL);
	if (!hw->cdev_data)
		return -ENOMEM;

	cpumask_copy(&hw->core_map, &lmh_cpu_mask);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);
	hw->map = devm_regmap_init_mmio(&pdev->dev, base, &lmh_regconfig);
	if (IS_ERR(hw->map))
		return PTR_ERR(hw->map);

	ret = of_property_read_u32(dn, "qcom,affinity", &affinity);
	if (ret)
		return -ENODEV;
	switch (affinity) {
	case 0:
		hw->affinity = LMH_CLUSTER_0;
		break;
	case 1:
		hw->affinity = LMH_CLUSTER_1;
		break;
	default:
		return -EINVAL;
	};

	/* Enable the thermal algorithm early */
	ret = qcom_scm_lmh_write(hw->affinity, LMH_SUB_FN_THERMAL,
				 LMH_ALGO_MODE_ENABLE, 1, 0, 0);
	if (ret)
		return ret;
	/* Enable the LMH outer loop algorithm */
	ret = qcom_scm_lmh_write(hw->affinity, LMH_SUB_FN_CRNT,
				 LMH_ALGO_MODE_ENABLE, 1, 0, 0);
	if (ret)
		return ret;
	/* Enable the Reliability algorithm */
	ret = qcom_scm_lmh_write(hw->affinity, LMH_SUB_FN_REL,
				 LMH_ALGO_MODE_ENABLE, 1, 0, 0);
	if (ret)
		return ret;
	/* Enable the BCL algorithm */
	ret = qcom_scm_lmh_write(hw->affinity, LMH_SUB_FN_BCL,
				 LMH_ALGO_MODE_ENABLE, 1, 0, 0);
	if (ret)
		return ret;
	ret = qcom_scm_lmh_enable();
	if (ret)
		return ret;

	/*
	 * Setup virtual thermal zones for each LMH-THERMAL block
	 * The sensor does not do actual thermal temperature readings
	 * but does support setting thresholds for trips.
	 * Let's register with thermal framework, so we have the ability
	 * to set low/high thresholds.
	 */
	hw->temp_limits[LMH_TRIP_HI] = INT_MAX;
	hw->temp_limits[LMH_TRIP_ARM] = 0;
	hw->hw_freq_limit = hw->max_freq = max_freq;
	hw->min_freq = min_freq;
	snprintf(hw->sensor_name, sizeof(hw->sensor_name), "lmh_sensor-%02d",
		 affinity);

	tzdev = devm_thermal_zone_of_sensor_register(&pdev->dev, 0, hw,
						     &lmh_sensor_ops);
	if (IS_ERR(tzdev))
		return PTR_ERR(tzdev);

	switch (affinity) {
	case 0:
		request_reg = LMH_CLUSTER_0_REQ;
		break;
	case 1:
		request_reg = LMH_CLUSTER_1_REQ;
		break;
	};

	mutex_init(&hw->access_lock);
	timer_setup(&hw->poll_timer, lmh_thermal_poll, TIMER_DEFERRABLE);

	hw->osm_hw_reg = devm_ioremap(&pdev->dev, request_reg, 0x4);
	if (!hw->osm_hw_reg) {
		dev_err(&pdev->dev, "osm_hw_reg register remap failed\n");
	}

	hw->irq = of_irq_get(pdev->dev.of_node, 0);
	if (hw->irq == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	if (hw->irq <= 0) {
		dev_err(&pdev->dev, "Error getting IRQ: %d\n", hw->irq);
		return hw->irq ?: -ENXIO;
	}
	atomic_set(&hw->is_irq_enabled, 1);
	ret = devm_request_threaded_irq(&pdev->dev, hw->irq, NULL,
					lmh_thermal_handle_isr,
					IRQF_TRIGGER_HIGH | IRQF_ONESHOT |
					IRQF_NO_SUSPEND, hw->sensor_name, hw);
	if (ret) {
		pr_err("Error registering for irq. err:%d\n", ret);
		return ret;
	}

	lmh_isens_vref_ldo_init(pdev, hw);

	hw->lmh_freq_attr.attr.name = "lmh_freq_limit";
	hw->lmh_freq_attr.show = lmh_freq_limit_show;
	hw->lmh_freq_attr.attr.mode = 0444;
	ret = device_create_file(&pdev->dev, &hw->lmh_freq_attr);
	if (ret)
		return ret;

	mutex_lock(&lmh_thermal_list_access);
	INIT_LIST_HEAD(&hw->list);
	list_add(&hw->list, &lmh_thermal_hw_list);
	mutex_unlock(&lmh_thermal_list_access);

	ret = cpuhp_setup_state(CPUHP_AP_ONLINE_DYN, "lmh-thermal/cdev:online",
				lmh_cpu_online, NULL);

	return ret;
}

static const struct of_device_id lmh_thermal_match[] = {
	{ .compatible = "qcom,lmh", },
	{},
};

static struct platform_driver lmh_thermal_driver = {
	.probe		= lmh_thermal_probe,
	.driver	= {
		.name = KBUILD_MODNAME,
		.of_match_table = lmh_thermal_match,
	},
};
builtin_platform_driver(lmh_thermal_driver);
