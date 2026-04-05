// SPDX-License-Identifier: GPL-2.0
/*
 * thermal_bin.c - hardware-version bin-info registration for thermal zones
 *
 * Provides thermal_zone_set_supported_hw_bin() and friends, which allow platform
 * drivers to register SoC fuse / hardware-version information with a thermal
 * zone so that the OF trip-point parser can filter trips and select per-bin
 * temperatures via the 'thermal-hw-bin' and 'temperature-bin' DT properties.
 */

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/slab.h>
#include <linux/thermal.h>

#include "thermal_core.h"

/**
 * thermal_zone_set_supported_hw_bin - register hardware-version info with a zone
 * @tz:    thermal zone device
 * @vers:  array of hardware-version bitfields (one word per level)
 * @count: number of entries in @vers (must be <= THERMAL_BIN_HW_MAX_LEVELS)
 *
 * Stores a copy of @vers in @tz->hw_bin_info so that the thermal OF parser can
 * match trip-point thermal-hw-bin sub-groups against the running hardware.
 * The copy is managed by the zone's parent device via devm.
 *
 * Return: 0 on success, negative errno on failure.
 */
int thermal_zone_set_supported_hw_bin(struct thermal_zone_device *tz,
				      const u32 *vers, unsigned int count)
{
	u32 *copy;

	if (WARN_ON(!tz || !vers || !count || count > THERMAL_BIN_HW_MAX_LEVELS))
		return -EINVAL;

	copy = devm_kmemdup(tz->device.parent, vers, count * sizeof(u32),
			    GFP_KERNEL);
	if (!copy)
		return -ENOMEM;

	WRITE_ONCE(tz->hw_bin_info.supported_hw_bin, copy);
	WRITE_ONCE(tz->hw_bin_info.supported_hw_bin_count, count);

	return 0;
}
EXPORT_SYMBOL_GPL(thermal_zone_set_supported_hw_bin);

static void devm_thermal_zone_clear_supported_hw_bin(void *data)
{
	struct thermal_zone_device *tz = data;

	WRITE_ONCE(tz->hw_bin_info.supported_hw_bin, NULL);
	WRITE_ONCE(tz->hw_bin_info.supported_hw_bin_count, 0);
}

/**
 * devm_thermal_zone_set_supported_hw_bin - managed variant of thermal_zone_set_supported_hw_bin
 * @dev:   device whose lifetime governs the cleanup action
 * @tz:    thermal zone device
 * @vers:  array of hardware-version bitfields
 * @count: number of entries in @vers
 *
 * Like thermal_zone_set_supported_hw_bin(), but registers a devm cleanup action on
 * @dev that clears the hw_bin_info when @dev is unbound.
 *
 * Return: 0 on success, negative errno on failure.
 */
int devm_thermal_zone_set_supported_hw_bin(struct device *dev,
					   struct thermal_zone_device *tz,
					   const u32 *vers, unsigned int count)
{
	int ret;

	ret = thermal_zone_set_supported_hw_bin(tz, vers, count);
	if (ret)
		return ret;

	return devm_add_action_or_reset(dev,
					devm_thermal_zone_clear_supported_hw_bin,
					tz);
}
EXPORT_SYMBOL_GPL(devm_thermal_zone_set_supported_hw_bin);

/**
 * thermal_zone_get_hw_bin_info - retrieve hardware-version info from a zone
 * @tz: thermal zone device (may be NULL)
 *
 * Return: pointer to the zone's thermal_hw_bin_info if supported_hw_bin has been
 *         set, or NULL if @tz is NULL or no hardware-version info is present.
 */
const struct thermal_hw_bin_info *
thermal_zone_get_hw_bin_info(const struct thermal_zone_device *tz)
{
	if (!tz || !READ_ONCE(tz->hw_bin_info.supported_hw_bin))
		return NULL;

	return &tz->hw_bin_info;
}
EXPORT_SYMBOL_GPL(thermal_zone_get_hw_bin_info);
