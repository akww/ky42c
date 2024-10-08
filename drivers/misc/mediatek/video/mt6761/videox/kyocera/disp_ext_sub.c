/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2016 KYOCERA Corporation
 * (C) 2017 KYOCERA Corporation
 * (C) 2022 KYOCERA Corporation
 */
/*
 * Copyright (C) 2007 Google Incorporated
 * Copyright (c) 2008-2013, The Linux Foundation. All rights reserved.
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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of_device.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include <linux/fb.h>

#include "disp_ext_sub_ctrl.h"

static int disp_ext_sub_probe(struct platform_device *pdev);
static int disp_ext_sub_remove(struct platform_device *pdev);
static int disp_ext_sub_open(struct fb_info *info, int user);
static int disp_ext_sub_release(struct fb_info *info, int user);
static int disp_ext_sub_mmap(struct fb_info *info, struct vm_area_struct *vma);
static void disp_ext_sub_shutdown(struct platform_device *pdev);
static int disp_ext_sub_ioctl(
	struct fb_info *info,
	unsigned int cmd,
	unsigned long arg );
static int disp_ext_sub_blank(int blank_mode, struct fb_info *info);
static int disp_ext_sub_pan_display(
	struct fb_var_screeninfo *var,
	struct fb_info *info );

static struct fb_ops disp_ext_sub_ops = {
	.owner = THIS_MODULE,
	.fb_open = disp_ext_sub_open,
	.fb_release = disp_ext_sub_release,
	.fb_mmap = disp_ext_sub_mmap,
	.fb_ioctl = disp_ext_sub_ioctl,
	.fb_blank = disp_ext_sub_blank,
	.fb_pan_display = disp_ext_sub_pan_display,
};

static const struct of_device_id disp_ext_sub_dt_match[] = {
	{.compatible = "kc,disp_ext_sub"},
	{}
};

static struct platform_driver disp_ext_sub_driver = {
	.probe = disp_ext_sub_probe,
	.remove = disp_ext_sub_remove,
	.shutdown = disp_ext_sub_shutdown,
	.driver = {
		.name = "disp_ext_sub",
		.of_match_table = disp_ext_sub_dt_match,
	},
};

extern struct fb_info *get_fb_info(unsigned int idx);

static int disp_ext_sub_open(struct fb_info *info, int user)
{
	int ret = 0;
	struct disp_ext_sub_info *sub_info_p;
	struct disp_ext_sub_pdata *pdata;

	pr_debug("%s start\n",__func__);

	if(!info){
		pr_err("%s end - null info\n",__func__);
		return -ENODEV;
	}

	sub_info_p = (struct disp_ext_sub_info*)(info->par);
	pdata = &(sub_info_p->pdata);

	if (DISP_SUB_STATE_PWR_OFF == pdata->state) {
		disp_ext_sub_panel_set_status(DISP_SUB_STATE_PWR_ON, pdata);
	}

	pr_debug("%s end\n",__func__);
	return ret;
}

static int disp_ext_sub_release(struct fb_info *info, int user)
{
	int ret = 0;
	pr_debug("%s start\n",__func__);
	pr_debug("%s end\n",__func__);
	return ret;
}

static int disp_ext_sub_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	int ret = 0;
	unsigned long start = info->fix.smem_start;
	u32 len = PAGE_ALIGN((start & ~PAGE_MASK) + info->fix.smem_len);
	unsigned long off = vma->vm_pgoff << PAGE_SHIFT;

	pr_debug("%s start\n",__func__);

	if (!start) {
		pr_err("No framebuffer memory is allocated.\n");
		return -ENOMEM;
	}

	start &= PAGE_MASK;
	if ((vma->vm_end <= vma->vm_start) ||
		(off >= len) ||
		((vma->vm_end - vma->vm_start) > (len - off))) {
		pr_debug("%s vm_area error\n",__func__);
		return -EINVAL;
	}
	off += start;
	if (off < start) {
		pr_debug("%s vm_area error\n",__func__);
		return -EINVAL;
	}
	vma->vm_pgoff = off >> PAGE_SHIFT;
	vma->vm_flags |= VM_IO | VM_DONTEXPAND | VM_DONTDUMP;

	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	ret = io_remap_pfn_range(vma, vma->vm_start, off >> PAGE_SHIFT,
					vma->vm_end - vma->vm_start,
					vma->vm_page_prot);

	pr_debug("%s end %d\n",__func__,ret);
	return ret;
}

static int disp_ext_sub_diag_status(struct fb_info *info, void __user *p)
{
	struct disp_ext_sub_info *sub_info_p = (struct disp_ext_sub_info*)(info->par);
	struct disp_ext_sub_pdata *pdata = &(sub_info_p->pdata);
	int ret;
	int state;

	switch(pdata->state) {
	case DISP_SUB_STATE_ON:
		state = 1;
		break;
	case DISP_SUB_STATE_OFF:
	case DISP_SUB_STATE_PWR_ON:
		state = 0;
		break;
#ifdef CONFIG_DISP_EXT_SUB_ALWAYS_ON
	case DISP_SUB_STATE_ALLWAYSON:
		state = 2;
		break;
#endif /* CONFIG_DISP_EXT_SUB_ALWAYS_ON */
	case DISP_SUB_STATE_PWR_OFF:
	default:
		state = -1;
		break;
	}

	ret = copy_to_user(p, (void*)&state, sizeof(state));

	return ret;
}

int disp_ext_sub_ioctl_set_subdispinfo(struct fb_info *info, void __user *p)
{
	struct disp_ext_sub_info *sub_info_p = (struct disp_ext_sub_info*)(info->par);
	struct disp_ext_sub_pdata *pdata = &(sub_info_p->pdata);
	struct fb_var_subdispinfo subdispinfo;
	int ret;

	ret = copy_from_user(&subdispinfo, p, sizeof(subdispinfo));
	if (ret)
		return ret;

	return disp_ext_sub_set_subdispinfo(pdata, &subdispinfo);
}

static int disp_ext_sub_ioctl(
	struct fb_info *info,
	unsigned int cmd,
	unsigned long arg )
{
	int ret = 0;
	void __user *argp = (void __user *)arg;

	if (!info || !info->par)
		return -EINVAL;

	pr_debug("%s start\n",__func__);
	switch (cmd) {
	case KC_DISP_EXT_SUB_STATUS_CMD:
		if (!lock_fb_info(info))
			return -ENODEV;
		ret = disp_ext_sub_diag_status(info, argp);
		unlock_fb_info(info);
		break;
	default:
		pr_err("%s start default\n",__func__);
		break;
	};
	pr_debug("%s end %d\n",__func__,ret);
	return ret;
}

static int disp_ext_sub_blank(int blank_mode, struct fb_info *info)
{
	int ret = 0;
	pr_debug("%s start mode=%d\n",__func__,blank_mode);

	ret = disp_ext_sub_blank_ctrl( blank_mode, info );

	pr_debug("%s end %d\n",__func__,ret);
	return ret;
}

static int disp_ext_sub_pan_display(
	struct fb_var_screeninfo *var,
	struct fb_info *info )
{
	int ret = 0;
	pr_debug("%s start\n",__func__);

	ret = disp_ext_sub_pan_display_ctrl(var,info);

	pr_debug("%s end %d\n",__func__,ret);
	return ret;
}

static ssize_t disp_ext_sub_sysfs_disp_stat(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct disp_ext_sub_info *sub_info_p = (struct disp_ext_sub_info*)(fbi->par);
	struct disp_ext_sub_pdata *pdata = &(sub_info_p->pdata);
	ssize_t ret;

	ret = sprintf(buf, "%d\n", pdata->state);

	return ret;
}

#if defined(CONFIG_DISP_EXT_SUB_CONTRAST)
ssize_t disp_ext_sub_dump_subdispinfo(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct disp_ext_sub_info *sub_info_p = (struct disp_ext_sub_info*)(fbi->par);
	struct disp_ext_sub_pdata *pdata = &(sub_info_p->pdata);

	if (!lock_fb_info(fbi))
		return count;
//	disp_ext_sub_get_subdispinfo(pdata);
	unlock_fb_info(fbi);

	return count;
}
#endif

static DEVICE_ATTR(disp_stat, S_IRUGO, disp_ext_sub_sysfs_disp_stat, NULL);
#if defined (CONFIG_DISP_EXT_SUB_ALWAYS_ON) || defined(CONFIG_DISP_EXT_SUB_CONTRAST)
static DEVICE_ATTR(subdispinfo, S_IWUSR, NULL, disp_ext_sub_dump_subdispinfo);
#endif /* CONFIG_DISP_EXT_SUB_ALWAYS_ON || CONFIG_DISP_EXT_SUB_CONTRAST */
#ifdef CONFIG_DISP_EXT_SUB_ALWAYS_ON
static DEVICE_ATTR(mdm_stat, S_IRUGO, disp_ext_sub_sysfs_mdm_stat, NULL);
static DEVICE_ATTR(mdm_svcdump, S_IWUSR|S_IWGRP, NULL, disp_ext_sub_sysfs_mdm_svcdump);
static DEVICE_ATTR(mdm_drvdump, S_IWUSR|S_IWGRP, NULL, disp_ext_sub_sysfs_mdm_drvdump);
static DEVICE_ATTR(mdm_svcdbglvl, S_IWUSR|S_IWGRP, NULL, disp_ext_sub_sysfs_mdm_svcdbglvl);
static DEVICE_ATTR(mdm_drvdbglvl, S_IWUSR|S_IWGRP, NULL, disp_ext_sub_sysfs_mdm_drvdbglvl);
static DEVICE_ATTR(mdm_timehist, S_IWUSR|S_IWGRP, NULL, disp_ext_sub_sysfs_mdm_timehist);
static DEVICE_ATTR(mdm_svcimage, S_IWUSR|S_IWGRP, NULL, disp_ext_sub_sysfs_mdm_svcimage);
static DEVICE_ATTR(mdm_drvimage, S_IWUSR|S_IWGRP, NULL, disp_ext_sub_sysfs_mdm_drvimage);
static DEVICE_ATTR(mdm_timeradj, S_IWUSR|S_IWGRP, NULL, disp_ext_sub_sysfs_mdm_timeradj);
#endif /* CONFIG_DISP_EXT_SUB_ALWAYS_ON */
static struct attribute *disp_ext_sub_attrs[] = {
	&dev_attr_disp_stat.attr,
#if defined (CONFIG_DISP_EXT_SUB_ALWAYS_ON) || defined(CONFIG_DISP_EXT_SUB_CONTRAST)
	&dev_attr_subdispinfo.attr,
#endif /* CONFIG_DISP_EXT_SUB_ALWAYS_ON || CONFIG_DISP_EXT_SUB_CONTRAST */
#ifdef CONFIG_DISP_EXT_SUB_ALWAYS_ON
	&dev_attr_mdm_stat.attr,
	&dev_attr_mdm_svcdump.attr,
	&dev_attr_mdm_drvdump.attr,
	&dev_attr_mdm_svcdbglvl.attr,
	&dev_attr_mdm_drvdbglvl.attr,
	&dev_attr_mdm_timehist.attr,
	&dev_attr_mdm_svcimage.attr,
	&dev_attr_mdm_drvimage.attr,
	&dev_attr_mdm_timeradj.attr,
#endif /* CONFIG_DISP_EXT_SUB_ALWAYS_ON */
	NULL,
};

static struct attribute_group disp_ext_sub_attr_group = {
	.attrs = disp_ext_sub_attrs
};

static int disp_ext_sub_create_sysfs(struct device *pdev)
{
	int rc;

	rc = sysfs_create_group(&pdev->kobj, &disp_ext_sub_attr_group);

	return rc;
}

static int disp_ext_sub_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct disp_ext_sub_info *drvdata;
	struct disp_ext_sub_pdata *pdata;
	pr_debug("%s start\n",__func__);

	ret = disp_ext_sub_drvdata_init(pdev);
	if(ret) {
		pr_err("%s data init error\n",__func__);
		return ret;
	}
	ret = disp_ext_sub_get_dt(pdev);
	if(ret) {
		pr_err("%s get dt error\n",__func__);
		return ret;
	}
	ret = disp_ext_sub_device_init(pdev);
	if(ret) {
		pr_err("%s device init error\n",__func__);
		return ret;
	}

	ret = disp_ext_sub_register_framebuffer(pdev, &disp_ext_sub_ops);
	if(ret) {
		pr_err("%s framebuffer register error\n",__func__);
		return ret;
	}

	drvdata = (struct disp_ext_sub_info *)platform_get_drvdata(pdev);
	if(!drvdata) {
		pr_err("%s end - null drvdata\n",__func__);
		return -ENODEV;
	}
	ret = disp_ext_sub_create_sysfs(drvdata->fbi->dev);
	if(ret) {
		pr_err("%s create sysfs error %d\n",__func__,ret);
	}
	pdata = &(drvdata->pdata);

	ret = disp_ext_sub_panel_set_status(DISP_SUB_STATE_PWR_ON, pdata);

	pr_debug("%s end %d\n",__func__,ret);
	return ret;
}


static int disp_ext_sub_remove(struct platform_device *pdev)
{
	pr_debug("%s start\n",__func__);
	pr_debug("%s end\n",__func__);
	return 0;
}

static void disp_ext_sub_shutdown(struct platform_device *pdev)
{

	struct fb_info *fbi;

	pr_err("%s start\n",__func__);
//	fbi = oem_fb_get_fb_info(2);
	fbi = get_fb_info(2);
	if (!fbi) {
		pr_err("%s end - null drvdata\n",__func__);
		return;
	}

	pr_debug("%s start\n",__func__);

	if (!lock_fb_info(fbi))
		return;
	disp_ext_sub_device_shutdown(pdev);
	unlock_fb_info(fbi);
	pr_err("%s end\n",__func__);
}

static int __init disp_ext_sub_driver_init(void)
{
	int ret;

	pr_debug("%s start\n",__func__);

	ret = platform_driver_register(&disp_ext_sub_driver);
	if (ret)
		pr_err("%s failed!\n",__func__);

	pr_debug("%s end\n",__func__);

	return ret;
}
late_initcall(disp_ext_sub_driver_init);
