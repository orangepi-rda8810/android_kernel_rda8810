
KERNEL_OUT := $(TARGET_OUT_INTERMEDIATES)/KERNEL
KERNEL_CONFIG := $(KERNEL_OUT)/.config
KERNEL_MODULES_OUT := $(TARGET_OUT_VENDOR)/modules

ifeq ($(USES_UNCOMPRESSED_KERNEL),true)
TARGET_PREBUILT_KERNEL := $(KERNEL_OUT)/arch/arm/boot/Image
else
TARGET_PREBUILT_KERNEL := $(KERNEL_OUT)/arch/arm/boot/zImage
endif

$(KERNEL_OUT):
	@echo "==== Start Kernel Compiling ... ===="

$(KERNEL_CONFIG): kernel/arch/arm/configs/$(KERNEL_DEFCONFIG)
	mkdir -p $(KERNEL_OUT)
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- \
	 $(KERNEL_DEFCONFIG)

.PHONY: $(TARGET_PREBUILT_KERNEL)
$(TARGET_PREBUILT_KERNEL) : $(KERNEL_CONFIG)
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- headers_install
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- zImage
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- INSTALL_MOD_STRIP=1 modules
	@-mkdir -p $(KERNEL_MODULES_OUT)
	@-find $(KERNEL_OUT) -name *.ko | xargs -I{} cp {} $(KERNEL_MODULES_OUT)

kernelheader:
	mkdir -p $(KERNEL_OUT)
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- headers_install
