#
# Copyright (C) 2016 MediaTek Inc.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See http://www.gnu.org/licenses/gpl-2.0.html for more details.
#

ifeq ($(CONFIG_MTK_ECCCI_DRIVER), y)

obj-$(CONFIG_MTK_ECCCI_DRIVER) := ccci.o
ccflags-y += -I$(srctree)/drivers/misc/mediatek/eccci/inc/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/eccci/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/eccci/$(MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/ccmni/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/conn_md/include/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/$(MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/scp/$(MTK_PLATFORM)

# common services
ifeq ($(CONFIG_MTK_MT6295_TMP_TEST), y)
ccci-y := ccci_core.o ccci_bm.o ccci_modem_tmp.o modem_sys1.o ccci_aee_handle.o
else
ccci-y := ccci_core.o ccci_bm.o ccci_modem.o modem_sys1.o ccci_aee_handle.o
endif

# sub modules
obj-y += fsm/
obj-y += hif/
obj-y += port/
obj-y += udc/
obj-$(CONFIG_MTK_MIX_DEVICES) += mixdev/

ifeq ($(CONFIG_MTK_ECCCI_C2K), y)
ccci-y += modem_sys3.o
endif

# platform code
obj-y += $(subst ",,$(CONFIG_MTK_PLATFORM))/

else
obj-  := dummy.o # avoid build error
endif
