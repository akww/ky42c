ifneq ($(strip $(CONFIG_MTK_TINYSYS_SSPM_SUPPORT)$(CONFIG_MTK_TINYSYS_SSPM_PLT_SUPPORT)),)
obj-y += sspm.o
sspm-y := sspm_helper.o sspm_sysfs.o sspm_reservedmem.o sspm_plt.o

ccflags-y += -I$(srctree)/drivers/misc/mediatek/sspm/$(CONFIG_MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/$(CONFIG_MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sspm/
ccflags-$(CONFIG_MTK_ENG_BUILD) += -DDEBUG

obj-y += $(CONFIG_MTK_PLATFORM)/
else
obj-y +=  dummy.o
endif
