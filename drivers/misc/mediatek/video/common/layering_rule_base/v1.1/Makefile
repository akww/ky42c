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

obj-y += layering_rule_base.o

ccflags-y += \
				-I$(srctree)/drivers/misc/mediatek/video/common/layering_rule_base/v1.1 \
                                -I$(srctree)/drivers/misc/mediatek/video/include     \
                                -I$(srctree)/drivers/misc/mediatek/video/common/IPv1     \
                                -I$(srctree)/drivers/misc/mediatek/video/$(MTK_PLATFORM)/dispsys \
                                -I$(srctree)/drivers/misc/mediatek/video/$(MTK_PLATFORM)/videox \
                                -I$(srctree)/drivers/misc/mediatek/sync/      \
                                -I$(srctree)/drivers/misc/mediatek/lcm/inc/ \

ccflags-y += \
                                -I$(srctree)/drivers/misc/mediatek/cmdq/v3/

ifeq ($(CONFIG_MACH_MT6761),y)
    ccflags-y += -I$(srctree)/drivers/misc/mediatek/cmdq/v3/mt6765/
else
    ccflags-y += -I$(srctree)/drivers/misc/mediatek/cmdq/v3/$(MTK_PLATFORM)/
endif
