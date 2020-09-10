// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2020 Linaro Limited
 *
 * Author: Daniel Lezcano <daniel.lezcano@linaro.org>
 *
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#define MICROWATT_PER_MILLIWATT 1000

#include <linux/cpumask.h>
#include <linux/cpufreq.h>
#include <linux/cpuhotplug.h>
#include <linux/device.h>
#include <linux/energy_model.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pm_domain.h>
#include <linux/pm_qos.h>
#include <linux/powercap.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>
#include <linux/units.h>

struct powercap_em {
	struct powercap_zone zone;
	struct powercap_em *parent;
	struct list_head sibling;
	struct list_head children;
	struct freq_qos_request qos_req;
	spinlock_t lock;
	bool mode;
	u64 power_limit;
	u64 power_max;
	u64 power_min;
	int weight;
	int cpu;
};

static const char *constraint_name[] = {
	"Performance capping",
};

static struct powercap_control_type *pct;
static struct powercap_em *pc_soc;
static struct powercap_em *pc_package;

struct powercap_em *to_powercap_em(struct powercap_zone *zone)
{
	return container_of(zone, struct powercap_em, zone);
}

/*
 * Browse the powercap nodes of the tree and rebalance their
 * weigths. This function is called when a node is inserted or
 * deleted.
 */
static void powercap_em_rebalance_weight(struct powercap_em *pcem)
{
	struct powercap_em *child;

	spin_lock(&pcem->lock);
	list_for_each_entry(child, &pcem->children, sibling) {

		child->weight = (child->power_max * 1024) / pcem->power_max;

		powercap_em_rebalance_weight(child);
	}
	spin_unlock(&pcem->lock);
}

/*
 * Initialize the energy model powercap zone by calling the underlying
 * powercap register function followed by the specific allocations.
 */
static struct powercap_em *
powercap_em_register(struct powercap_control_type *control_type,
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

	/*
	 * The root node does not have a parent
	 */
	if (parent) {
		spin_lock(&parent->lock);
		list_add_tail(&pcem->sibling, &parent->children);
		spin_unlock(&parent->lock);
		pcem->parent = parent;
	}

	return pcem;
}

/*
 * When a new powercap zone is inserted, propagate its power numbers
 * to the parents.
 */
static int powercap_em_set_power_range(struct powercap_em *pcem,
				       struct em_perf_domain *em)
{
	struct powercap_em *parent = pcem->parent;
	int nr_cpus = cpumask_weight(to_cpumask(em->cpus));

	if (pcem->power_min || pcem->power_max)
		return -EINVAL;

	pcem->power_min = em->table[0].power;
	pcem->power_min *= MICROWATT_PER_MILLIWATT;
	pcem->power_min *= nr_cpus;

	pcem->power_max = em->table[em->nr_perf_states - 1].power;
	pcem->power_max *= MICROWATT_PER_MILLIWATT;
	pcem->power_max *= nr_cpus;

	while (parent) {
		spin_lock(&parent->lock);
		parent->power_min += pcem->power_min;
		parent->power_max += pcem->power_max;
		spin_unlock(&parent->lock);
		parent = parent->parent;
	}

	return 0;
}

static int get_max_power_range_uw(struct powercap_zone *pcz, u64 *max_power_uw)
{
	struct powercap_em *pcem = to_powercap_em(pcz);

	spin_lock(&pcem->lock);
	*max_power_uw = pcem->power_max;
	spin_unlock(&pcem->lock);

	return 0;
}

static int get_pd_power_uw(struct powercap_zone *pcz, u64 *power_uw)
{
	struct powercap_em *pcem = to_powercap_em(pcz);
	struct em_perf_domain *pd;
	unsigned long freq;
	int i, nr_cpus;

	freq = cpufreq_quick_get(pcem->cpu);
	pd = em_cpu_get(pcem->cpu);
	nr_cpus = cpumask_weight(to_cpumask(pd->cpus));

	for (i = 0; i < pd->nr_perf_states; i++) {

		if (pd->table[i].frequency < freq)
			continue;

		*power_uw = pd->table[i].power *
			MICROWATT_PER_MILLIWATT * nr_cpus;

		return 0;
	}

	return -EINVAL;
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

static int set_domain_enable(struct powercap_zone *pcz, bool mode)
{
	struct powercap_em *pcem = to_powercap_em(pcz);
	struct cpufreq_policy *policy;
	struct em_perf_domain *pd;
	int ret;

	if (mode) {
		policy = cpufreq_cpu_get(pcem->cpu);
		if (!policy)
			return -EINVAL;

		pd = em_cpu_get(pcem->cpu);
		if (!pd)
			return -EINVAL;

		ret = freq_qos_add_request(&policy->constraints,
					   &pcem->qos_req, FREQ_QOS_MAX,
					   pd->table[pd->nr_perf_states - 1].frequency);
		if (ret)
			return ret;

	} else {
		freq_qos_remove_request(&pcem->qos_req);
	}

	pcem->mode = mode;

	powercap_em_rebalance_weight(pc_soc);

	return 0;
}

static int get_domain_enable(struct powercap_zone *pcz, bool *mode)
{
	struct powercap_em *pcem = to_powercap_em(pcz);

	*mode = pcem->mode;

	return 0;
}

static int release_zone(struct powercap_zone *pcz)
{
	struct powercap_em *pcem = to_powercap_em(pcz);

	if (!list_empty(&pcem->children))
		return -EBUSY;

	freq_qos_remove_request(&pcem->qos_req);
	list_del(&pcem->sibling);
	kfree(pcem);
	powercap_em_rebalance_weight(pc_soc);

	return 0;
}

/*
 * Set the power limit on the nodes, the power limit is distributed
 * given the weight of the children.
 */
static int set_children_power_limit(struct powercap_zone *pcz, int cid,
				    u64 power_limit)
{
	struct powercap_em *pcem = to_powercap_em(pcz);
	struct powercap_em *child;
	u64 power;
	int ret = 0;

	/*
	 * Don't allow values outside of the power range previously
	 * set when initiliazing the powercap energy model zone
	 */
	pcem->power_limit = clamp_val(power_limit,
				      pcem->power_min,
				      pcem->power_max);

	spin_lock(&pcem->lock);
	list_for_each_entry(child, &pcem->children, sibling) {

		power = (pcem->power_limit * child->weight) / 1024;

		ret = child->zone.constraints->ops->set_power_limit_uw(
			&child->zone, cid, power);
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
		ret = child->zone.constraints->ops->get_power_limit_uw(
			&child->zone, cid, &power);
		if (ret)
			break;
		*data += power;
	}
	spin_unlock(&pcem->lock);

	return ret;
}

static const char *get_constraint_name(struct powercap_zone *pcz, int cid)
{
	return constraint_name[cid];
}

static int set_pd_power_limit(struct powercap_zone *pcz, int cid,
			       u64 power_limit)
{
	struct powercap_em *pcem = to_powercap_em(pcz);
	struct em_perf_domain *pd;
	unsigned long frequency;
	int i, nr_cpus;

	spin_lock(&pcem->lock);

	power_limit = clamp_val(power_limit, pcem->power_min, pcem->power_max);

	pd = em_cpu_get(pcem->cpu);

	nr_cpus = cpumask_weight(to_cpumask(pd->cpus));

	for (i = 0, frequency = pd->table[0].frequency; i < pd->nr_perf_states; i++) {

		u64 power = pd->table[i].power * MICROWATT_PER_MILLIWATT;

		if ((power * nr_cpus) > power_limit)
			break;

		frequency = pd->table[i].frequency;
	}

	freq_qos_update_request(&pcem->qos_req, frequency);

	pcem->power_limit = power_limit;

	spin_unlock(&pcem->lock);

	return 0;
}

static int get_pd_power_limit(struct powercap_zone *pcz, int cid, u64 *data)
{
	struct powercap_em *pcem = to_powercap_em(pcz);

	spin_lock(&pcem->lock);
	*data = pcem->power_limit ? pcem->power_limit : pcem->power_max;
	spin_unlock(&pcem->lock);

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

static int get_max_power_uw(struct powercap_zone *pcz, int id, u64 *data)
{
	return get_max_power_range_uw(pcz, data);
}

static const struct powercap_zone_constraint_ops constraint_ops = {
	.set_power_limit_uw = set_children_power_limit,
	.get_power_limit_uw = get_children_power_limit,
	.set_time_window_us = set_time_window,
	.get_time_window_us = get_time_window,
	.get_max_power_uw = get_max_power_uw,
	.get_name = get_constraint_name,
};

static const struct powercap_zone_constraint_ops pd_constraint_ops = {
	.set_power_limit_uw = set_pd_power_limit,
	.get_power_limit_uw = get_pd_power_limit,
	.set_time_window_us = set_time_window,
	.get_time_window_us = get_time_window,
	.get_max_power_uw = get_max_power_uw,
	.get_name = get_constraint_name,
};

static const struct powercap_zone_ops zone_ops = {
	.get_max_power_range_uw = get_max_power_range_uw,
	.get_power_uw = get_children_power_uw,
	.set_enable = set_domain_enable,
	.get_enable = get_domain_enable,
	.release = release_zone,
};

static const struct powercap_zone_ops pd_zone_ops = {
	.get_max_power_range_uw = get_max_power_range_uw,
	.get_power_uw = get_pd_power_uw,
	.set_enable = set_domain_enable,
	.get_enable = get_domain_enable,
	.release = release_zone,
};

static int cpuhp_powercap_em_online(unsigned int cpu)
{
        struct powercap_em *pcem;
	struct cpufreq_policy *policy;
	struct em_perf_domain *pd;
	char name[CPUFREQ_NAME_LEN];
	int ret;

	policy = cpufreq_cpu_get(cpu);

	if (!policy || cpumask_first(policy->related_cpus) != cpu)
		return 0;

	pd = em_cpu_get(cpu);
	if (!pd)
		return -EINVAL;

	sprintf(name, "policy%d", cpu);

	pcem = powercap_em_register(pct, name, pc_package,
				    &pd_zone_ops, 1, &pd_constraint_ops);
	if (!pcem)
		return -EINVAL;

	ret = powercap_em_set_power_range(pcem, pd);
	if (ret)
		return ret;

	pcem->cpu = cpu;

	ret = freq_qos_add_request(&policy->constraints,
				   &pcem->qos_req, FREQ_QOS_MAX,
				   pd->table[pd->nr_perf_states - 1].frequency);

	powercap_em_rebalance_weight(pc_soc);

	return ret;
}

static int __init powercap_em_init(void)
{
	struct generic_pm_domain *genpd;
	struct powercap_em *pc_dev;

	pct = powercap_register_control_type(NULL, "energy_model", NULL);
	if (!pct) {
		pr_err("Failed to register control type\n");
		return -EINVAL;
	}

	pc_soc = powercap_em_register(pct, "soc", NULL,
				      &zone_ops, 1, &constraint_ops);
	if (!pc_soc)
		return -EINVAL;

	pc_package = powercap_em_register(pct, "package", pc_soc,
					  &zone_ops, 1, &constraint_ops);
	if (!pc_package)
		return -EINVAL;

	mutex_lock(&gpd_list_lock);

	list_for_each_entry(genpd, &gpd_list, gpd_list_node) {
		dev_err(&genpd->dev, "registering powercap em zone");
		pc_dev = powercap_em_register(pct, dev_name(&genpd->dev), NULL,
					      &zone_ops, 1, &constraint_ops);

		if (!pc_dev) {
			dev_err(&genpd->dev, "error registering powercap em zone");
		}
	}

	mutex_unlock(&gpd_list_lock);


	return cpuhp_setup_state(CPUHP_AP_POWERCAP_EM_ONLINE,
				 "powercap_em:online",
				 cpuhp_powercap_em_online, NULL);
}
late_initcall(powercap_em_init);
