#
# Copyright (C) 2015 MediaTek Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#

MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))

# mt6761 and mt6765 co-folder
ifeq ($(MTK_PLATFORM),mt6761)
MTK_PLATFORM := mt6765
endif

ifeq ($(CONFIG_SINGLE_PANEL_OUTPUT),y)
else ifneq ($(wildcard $(srctree)/drivers/misc/mediatek/ext_disp/$(MTK_PLATFORM)/),)
obj-y += $(MTK_PLATFORM)/
obj-y += common/
else
obj-y += dummy/
endif
