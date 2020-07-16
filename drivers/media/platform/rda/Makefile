#
# Makefile for the RDA camera driver
#
subdir-ccflags-y := -Werror
subdir-ccflags-y += -I$(ANDROID_BUILD_TOP)/$(RDA_TARGET_DEVICE_DIR)/include

obj-$(CONFIG_VIDEO_RDA) += isp_ae.o

obj-$(CONFIG_VIDEO_RDA) += rda_sensor_base.o
ifeq ($(CONFIG_MACH_RDA8810),y)
obj-$(CONFIG_VIDEO_RDA) += rda_camera_8810.o
else
ifeq ($(CONFIG_MACH_RDA8850E),y)
obj-$(CONFIG_VIDEO_RDA) += rda_camera_8850e.o
endif
endif

