/* SPDX-License-Identifier: GPL-2.0-only */
/*
 *  linux/include/linux/cpu_cooling.h
 *
 *  Copyright (C) 2012	Samsung Electronics Co., Ltd(http://www.samsung.com)
 *  Copyright (C) 2012  Amit Daniel <amit.kachhap@linaro.org>
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#ifndef __CPU_COOLING_H__
#define __CPU_COOLING_H__

#include <linux/of.h>
#include <linux/thermal.h>
#include <linux/cpumask.h>

struct cpufreq_policy;

typedef int (*plat_mitig_t)(int cpu, u32 clip_freq);
struct cpu_cooling_ops {
	plat_mitig_t ceil_limit, floor_limit;
};

#ifdef CONFIG_CPU_THERMAL
/**
 * cpufreq_cooling_register - function to create cpufreq cooling device.
 * @policy: cpufreq policy.
 */
struct thermal_cooling_device *
cpufreq_cooling_register(struct cpufreq_policy *policy);

/**
 * cpufreq_cooling_unregister - function to remove cpufreq cooling device.
 * @cdev: thermal cooling device pointer.
 */
void cpufreq_cooling_unregister(struct thermal_cooling_device *cdev);

#else /* !CONFIG_CPU_THERMAL */
static inline struct thermal_cooling_device *
cpufreq_cooling_register(struct cpufreq_policy *policy)
{
	return ERR_PTR(-ENOSYS);
}

static inline
void cpufreq_cooling_unregister(struct thermal_cooling_device *cdev)
{
	return;
}
#endif	/* CONFIG_CPU_THERMAL */

#if defined(CONFIG_THERMAL_OF) && defined(CONFIG_CPU_THERMAL)

struct thermal_cooling_device *
cpufreq_platform_cooling_register(struct cpufreq_policy *policy,
				  struct cpu_cooling_ops *ops);

/**
 * of_cpufreq_cooling_register - create cpufreq cooling device based on DT.
 * @policy: cpufreq policy.
 */
struct thermal_cooling_device *
of_cpufreq_cooling_register(struct cpufreq_policy *policy);
#else

static inline struct thermal_cooling_device *
cpufreq_platform_cooling_register(const struct cpumask *clip_cpus,
				  struct cpu_cooling_ops *ops)
{
	return NULL;
}
static inline struct thermal_cooling_device *
of_cpufreq_cooling_register(struct cpufreq_policy *policy)
{
	return NULL;
}
#endif /* defined(CONFIG_THERMAL_OF) && defined(CONFIG_CPU_THERMAL) */

#endif /* __CPU_COOLING_H__ */
