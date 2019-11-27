// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2017-2019, Linaro Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/interconnect.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/random.h>
#include <linux/uaccess.h>

struct platform_device *sdm845_icc_pdev;

struct sdm845_icc_stress_data {
	struct icc_path *path;
	u32 src;
	u32 dst;
};

static struct sdm845_icc_stress_data data[] = {
	{ .src = 7, .dst = 111, }, // ufs-mem
	{ .src = 27, .dst = 99, }, // ufs-cfg
	{ .src = 45, .dst = 111, }, // video0
	{ .src = 13, .dst = 111, }, // crypto
	{ .src = 37, .dst = 111, }, // gfx3d
	{ .src = 8, .dst = 111, }, // pcie
	{ .src = 4, .dst = 111, }, // sdcc2
	{ .src = 17, .dst = 111, }, // usb
	{ .src = 5, .dst = 111, }, // sdcc4
};

static int sdm845_icc_stress_thread(void *data)
{
	struct sdm845_icc_stress_data *d = data;
	u32 avg_bw, peak_bw, tag = 0;
	int i = 0;

	do {
		avg_bw = get_random_u32() % 10000000 + 1000 ;
		peak_bw = get_random_u32() % 10000000 + 1000;
		tag = get_random_u32() % 100;

		if (IS_ERR_OR_NULL(d->path)) {
			d->path = icc_get(&sdm845_icc_pdev->dev, d->src, d->dst);
			if (IS_ERR(d->path) && PTR_ERR(d->path) != -EPROBE_DEFER)
			    pr_err("%s icc_get src=%u dst=%u error: %lu\n",
				   __func__, d->src, d->dst, PTR_ERR(d->path));
		}

		if (!IS_ERR(d->path)) {

			if (i % 7)
				icc_set_tag(d->path, tag);

			icc_set_bw(d->path, avg_bw, peak_bw);

			avg_bw--;
			peak_bw--;

			if (i++ % 5) {
				icc_put(d->path);
				d->path = NULL;
			}
		}

	} while (!kthread_should_stop());

	return 0;
}

static int __init sdm845_icc_stress_init(void)
{
	int cpu;

	sdm845_icc_pdev = platform_device_alloc("icc-sdm845-stress",
						PLATFORM_DEVID_AUTO);
	platform_device_add(sdm845_icc_pdev);

	for_each_online_cpu(cpu) {

		struct task_struct *k = kthread_create(sdm845_icc_stress_thread,
						       &data[cpu],
						       "icc_stress_%d", cpu);
		kthread_bind(k, cpu);
		wake_up_process(k);
		pr_info("Started %s on cpu%d\n", __func__, cpu);
	}
	/* TODO: stop threads and free resources */

	return 0;
}
late_initcall(sdm845_icc_stress_init);
MODULE_LICENSE("GPL v2");
