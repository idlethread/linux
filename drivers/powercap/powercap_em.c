// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2020 Linaro Limited
 *
 * Author: Daniel Lezcano <daniel.lezcano@linaro.org>
 *
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/cpumask.h>
#include <linux/cpufreq.h>
#include <linux/cpuhotplug.h>
#include <linux/device.h>
#include <linux/energy_model.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pm_qos.h>
#include <linux/powercap.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>

struct powercap_em_param {
	int cpu;
	u64 power_limit;
	struct freq_qos_request qos_req;
};

struct powercap_em {
	struct powercap_zone zone;
	struct list_head sibling;
	struct list_head children;
	spinlock_t lock;
	int weight;
};

static struct powercap_control_type *pct;
static struct powercap_em *pc_soc;
static struct powercap_em *pc_package;

struct powercap_em *to_powercap_em(struct powercap_zone *zone)
{
	return container_of(zone, struct powercap_em, zone);
}

static int powercap_em_rebalance_weight(struct powercap_em *pcem)
{
	struct powercap_em *child;
	u64 max_power, sum_max_power = 0;
	int ret = 0;

	spin_lock(&pcem->lock);

	list_for_each_entry(child, &pcem->children, sibling) {

		powercap_em_rebalance_weight(child);

		ret = child->zone.ops->get_max_power_range_uw(&child->zone, &max_power);
		if (ret)
			break;

		sum_max_power += max_power;
	}

	list_for_each_entry(child, &pcem->children, sibling) {

		ret = child->zone.ops->get_max_power_range_uw(&child->zone, &max_power);
		if (ret)
			break;

		child->weight = (max_power * 1024) / sum_max_power;

		printk("Computed weight for %s: %d\n", child->zone.name, child->weight);
	}

	spin_unlock(&pcem->lock);

	return ret;
}

static struct powercap_em *powercap_em_register(struct powercap_control_type *control_type,
						const char *name,
						struct powercap_em *parent,
						const struct powercap_zone_ops *ops,
						int nr_constraints,
						const struct powercap_zone_constraint_ops *const_ops)
{
	struct powercap_em *pcem;
	struct powercap_zone *pcz;

	pcem = kzalloc(sizeof(*pcem), GFP_KERNEL);
	if (!pcem)
		return NULL;

	INIT_LIST_HEAD(&pcem->children);
	INIT_LIST_HEAD(&pcem->sibling);
	spin_lock_init(&pcem->lock);

	pcz = powercap_register_zone(&pcem->zone, control_type, name,
				     parent ? &parent->zone : NULL,
				     ops, nr_constraints, const_ops);
	if (IS_ERR(pcz)) {
		kfree(pcem);
		return NULL;
	}

	if (parent) {
		spin_lock(&parent->lock);
		list_add_tail(&pcem->sibling, &parent->children);
		spin_unlock(&parent->lock);
	}

	return pcem;
}

static int set_children_power_limit(struct powercap_zone *pcz, int cid, u64 power_limit)
{
	struct powercap_em *pcem = to_powercap_em(pcz);
	struct powercap_em *child;
	u64 power;
	int ret = 0;

	spin_lock(&pcem->lock);
	list_for_each_entry(child, &pcem->children, sibling) {

		power = (power_limit * child->weight) / 1024;

		printk("Setting power limit for %s '%llu' (w=%d, limit=%llu)\n",
		       child->zone.name, power, child->weight, power_limit);
		
		ret = child->zone.constraints->ops->set_power_limit_uw(&child->zone, cid, power);
		if (ret)
			break;
	}
	spin_unlock(&pcem->lock);

	return ret;
}

static int get_children_power_limit(struct powercap_zone *pcz, int cid, u64 *data)
{
	struct powercap_em *pcem = to_powercap_em(pcz);
	struct powercap_em *child;
	u64 power;
	int ret = 0;

	*data = 0;
	
	spin_lock(&pcem->lock);
	list_for_each_entry(child, &pcem->children, sibling) {
		ret = child->zone.constraints->ops->get_power_limit_uw(&child->zone, cid, &power);
		if (ret)
			break;
		*data += power;
	}
	spin_unlock(&pcem->lock);

	return ret;
}

static const char *get_constraint_name(struct powercap_zone *pcz, int cid)
{
	return NULL;
}

static int get_children_max_power(struct powercap_zone *pcz, int cid, u64 *data)
{
	struct powercap_em *pcem = to_powercap_em(pcz);
	struct powercap_em *child;
	u64 power;
	int ret = 0;

	*data = 0;
	
	spin_lock(&pcem->lock);
	list_for_each_entry(child, &pcem->children, sibling) {
		ret = child->zone.constraints->ops->get_max_power_uw(&child->zone, cid, &power);
		if (ret)
			break;
		*data += power;
	}
	spin_unlock(&pcem->lock);

	return ret;
}

static int set_cpu_power_limit(struct powercap_zone *pcz, int cid,
			       u64 power_limit)
{
	struct em_perf_domain *pd;
	struct powercap_em_param *pcp;
	unsigned long frequency;
	int i;

	pcp = powercap_get_zone_data(pcz);
	if (!pcp)
		return -EINVAL;
	
	pd = em_cpu_get(pcp->cpu);

	for (i = 0, frequency = pd->table[0].frequency;
	     i < pd->nr_cap_states; i++) {

		if ((pd->table[i].power * 1000) > power_limit)
			break;

		frequency = pd->table[i].frequency;
	}

	freq_qos_update_request(&pcp->qos_req, frequency);

	pcp->power_limit = power_limit;

	return 0;
}

static int get_cpu_power_limit(struct powercap_zone *pcz, int cid, u64 *data)
{
	struct powercap_em_param *pcp;

	pcp = powercap_get_zone_data(pcz);
	if (!pcp)
		return -EINVAL;

	if (!pcp->power_limit)
		return pcz->ops->get_max_power_range_uw(pcz, data);
	else
		*data = pcp->power_limit;
	
	return 0;
}

static int set_time_window(struct powercap_zone *pcz, int cid, u64 window)
{
	return -ENOSYS;
}


static int get_time_window(struct powercap_zone *pcz, int cid, u64 *data)
{
	*data = 0;
	
	return 0;
}

static const char *get_cpu_constraint_name(struct powercap_zone *power_zone,
					   int cid)
{
	return "CPU frequency capping";
}

static int get_cpu_max_power(struct powercap_zone *pcz, int id, u64 *data)
{
	struct em_perf_domain *pd;
	struct powercap_em_param *pcp;

	pcp = powercap_get_zone_data(pcz);
	if (!pcp)
		return -EINVAL;
	
	pd = em_cpu_get(pcp->cpu);

	*data = pd->table[pd->nr_cap_states - 1].power * 1000;

	return 0;
}

static const struct powercap_zone_constraint_ops constraint_ops = {
	.set_power_limit_uw = set_children_power_limit,
	.get_power_limit_uw = get_children_power_limit,
	.set_time_window_us = set_time_window,
	.get_time_window_us = get_time_window,
	.get_max_power_uw = get_children_max_power,
	.get_name = get_constraint_name,
};

static const struct powercap_zone_constraint_ops cpu_constraint_ops = {
	.set_power_limit_uw = set_cpu_power_limit,
	.get_power_limit_uw = get_cpu_power_limit,
	.set_time_window_us = set_time_window,
	.get_time_window_us = get_time_window,
	.get_max_power_uw = get_cpu_max_power,
	.get_name = get_cpu_constraint_name,
};

static int get_children_max_power_range_uw(struct powercap_zone *pcz,
					   u64 *max_power_uw)
{
	struct powercap_em *pcem = to_powercap_em(pcz);
	struct powercap_em *child;
	u64 power;
	int ret = 0;

	*max_power_uw = 0;
	
	spin_lock(&pcem->lock);
	list_for_each_entry(child, &pcem->children, sibling) {
		ret = child->zone.ops->get_max_power_range_uw(&child->zone, &power);
		if (ret)
			break;
		*max_power_uw += power;
	}
	spin_unlock(&pcem->lock);

	return ret;
}

static int get_cpu_max_power_range_uw(struct powercap_zone *pcz,
				      u64 *max_power_uw)
{
	struct em_perf_domain *pd;
	struct powercap_em_param *pcp;

	pcp = powercap_get_zone_data(pcz);
	if (!pcp)
		return -EINVAL;
	
	pd = em_cpu_get(pcp->cpu);

	*max_power_uw = pd->table[pd->nr_cap_states - 1].power * 1000;

	return 0;
}

static int get_gpu_max_power_range_uw(struct powercap_zone *pcz,
				      u64 *max_power_uw)
{
	*max_power_uw = 0;

	return 0;
}

static int get_cpu_power_uw(struct powercap_zone *pcz, u64 *power_uw)
{
	int i;
	unsigned long freq;
	struct em_perf_domain *pd;
	struct powercap_em_param *pcp;
	
	pcp = powercap_get_zone_data(pcz);
	if (!pcp)
		return -EINVAL;

	freq = cpufreq_quick_get(pcp->cpu);
	pd = em_cpu_get(pcp->cpu);

	for (i = 0; i < pd->nr_cap_states; i++) {

		if (pd->table[i].frequency < freq)
			continue;

		*power_uw = pd->table[i].power * 1000;

		return 0;
	}

	return -EINVAL;
}

static int get_gpu_power_uw(struct powercap_zone *pcz, u64 *power_uw)
{
	*power_uw = 0;

	return 0;
}

static int get_children_power_uw(struct powercap_zone *pcz, u64 *power_uw)
{
	struct powercap_em *pcem = to_powercap_em(pcz);
	struct powercap_em *child;
	u64 power;
	int ret = 0;

	*power_uw = 0;
	
	spin_lock(&pcem->lock);
	list_for_each_entry(child, &pcem->children, sibling) {
		ret = child->zone.ops->get_power_uw(&child->zone, &power);
		if (ret)
			break;
		*power_uw += power;
	}
	spin_unlock(&pcem->lock);

	return ret;
}

static int release_zone(struct powercap_zone *power_zone)
{
	return 0;
}

static int set_domain_enable(struct powercap_zone *power_zone, bool mode)
{
	return 0;
}

static int get_domain_enable(struct powercap_zone *power_zone, bool *mode)
{
	return 0;
}

static const struct powercap_zone_ops zone_ops = {
	.get_max_power_range_uw = get_children_max_power_range_uw,
	.get_power_uw = get_children_power_uw,
	.release = release_zone,
	.set_enable = set_domain_enable,
	.get_enable = get_domain_enable,
};

static const struct powercap_zone_ops cpu_zone_ops = {
	.get_max_power_range_uw = get_cpu_max_power_range_uw,
	.get_power_uw = get_cpu_power_uw,
	.release = release_zone,
	.set_enable = set_domain_enable,
	.get_enable = get_domain_enable,
};

static const struct powercap_zone_ops gpu_zone_ops = {
	.get_max_power_range_uw = get_gpu_max_power_range_uw,
	.get_power_uw = get_gpu_power_uw,
	.release = release_zone,
	.set_enable = set_domain_enable,
	.get_enable = get_domain_enable,
};

static int cpuhp_powercap_em_online(unsigned int cpu)
{
	struct powercap_em *pc_perfdomain;
	struct powercap_em_param *pcp;
	struct cpufreq_policy *policy;
	int ret;
	static int id = 0;
	char name[20];

	policy = cpufreq_cpu_get(cpu);

	if (!policy || cpumask_first(policy->related_cpus) != cpu)
		return 0;

	sprintf(name, "perfdomain%d", id++);

	pc_perfdomain = powercap_em_register(pct, name, pc_package,
					     &zone_ops, 1, &constraint_ops);
	if (!pc_perfdomain)
		return -EINVAL;

	for_each_cpu(cpu, policy->related_cpus) {

		struct em_perf_domain *pd;
		struct powercap_em *pc_cpu;

		sprintf(name, "cpu%d", cpu);
					
		pc_cpu = powercap_em_register(pct, name, pc_perfdomain,
					      &cpu_zone_ops, 1, &cpu_constraint_ops);
		if (!pc_cpu)
			return -EINVAL;

		pcp = kzalloc(sizeof(*pcp), GFP_KERNEL);
		if (!pcp)
			return -ENOMEM;

		pd = em_cpu_get(cpu);
		
		ret = freq_qos_add_request(&policy->constraints,
					   &pcp->qos_req, FREQ_QOS_MAX,
					   pd->table[pd->nr_cap_states - 1].frequency);

		pcp->cpu = cpu;
		powercap_set_zone_data(&pc_cpu->zone, pcp);
	}

	return powercap_em_rebalance_weight(pc_soc);
}

static int cpuhp_powercap_em_offline(unsigned int cpu)
{
	return 0;
}

static int __init powercap_em_init(void)
{
	struct powercap_em *pc_gpu;

	pct = powercap_register_control_type(NULL, "energy_model", NULL);
	if (!pct) {
		pr_err("Failed to register control type\n");
		return -EINVAL;
	}

	pc_soc = powercap_em_register(pct, "soc", NULL,
				      &zone_ops, 1, &constraint_ops);
	if (!pc_soc)
		return -EINVAL;

	pc_gpu = powercap_em_register(pct, "gpu", pc_soc,
				      &gpu_zone_ops, 1, &constraint_ops);
	if (!pc_gpu)
		return -EINVAL;
	
	pc_package = powercap_em_register(pct, "package", pc_soc,
					  &zone_ops, 1, &constraint_ops);
	if (!pc_package)
		return -EINVAL;

	return  cpuhp_setup_state(CPUHP_AP_POWERCAP_EM_ONLINE,
				  "powercap_em:online",
				  cpuhp_powercap_em_online,
				  cpuhp_powercap_em_offline);
}
late_initcall(powercap_em_init);
