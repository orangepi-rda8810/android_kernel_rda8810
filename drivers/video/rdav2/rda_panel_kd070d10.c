#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/gpio.h>
#include <linux/io.h>

#include <plat/devices.h>
#include <plat/rda_debug.h>
#include <mach/board.h>

#include "rda_panel.h"
#include "rda_lcdc.h"

static struct rda_spi_panel_device *rda_kd070d10_spi_dev= NULL;

#define rda_spi_dev rda_kd070d10_spi_dev


RDA_SPI_PARAMETERS kd070d10_spicfg = {
	.inputEn = true,
	.clkDelay = RDA_SPI_HALF_CLK_PERIOD_0,
	.doDelay = RDA_SPI_HALF_CLK_PERIOD_0,
	.diDelay = RDA_SPI_HALF_CLK_PERIOD_1,
	.csDelay = RDA_SPI_HALF_CLK_PERIOD_0,
	.csPulse = RDA_SPI_HALF_CLK_PERIOD_0,
	.frameSize = 9,
	.oeRatio = 0,
	.rxTrigger = RDA_SPI_RX_TRIGGER_4_BYTE,
	.txTrigger = RDA_SPI_TX_TRIGGER_1_EMPTY,
	.rxMode = RDA_SPI_DIRECT_POLLING,
	.txMode = RDA_SPI_DIRECT_POLLING,
	.mask = {0, 0, 0, 0, 0},
	.handler = NULL
};

/* ************************************************
   PLEASE NOTES:
	clk_divider: 0 mean, auto-cal by driver
		     non-zero, customer can fine tune.

	example:
		GOUDA_CLK == 400MHz
		clk_divider = 14
	while by auto-cal, clk_divider = 13
   *********************************************** */
static int kd070d10_reset_gpio(void)
{
	printk("%s\n",__func__);
	mdelay(1);
	gpio_set_value(GPIO_LCD_RESET, 0);
	mdelay(10);
	gpio_set_value(GPIO_LCD_RESET, 1);
	mdelay(10);

	return 0;
}


static int kd070d10_open(void)
{
	return 0;
}

static int kd070d10_sleep(void)
{
#if 0
	/*
	 * Set LCD to sleep mode if we don't control v_lcd pin.
	 * If we are using v_lcd, we'll not use this way.
	 */
	panel_spi_send_cmd(0x10);
#endif /* #if 0 */

	return 0;
}

static int kd070d10_wakeup(void)
{
	/*
	 * Note:
	 * If we use v_lcd pin, we'll not controll lcd to sleep mode.
	 */
#if 0
	/* Exist from sleep mode */
	panel_spi_send_cmd(0x11);
#endif /* #if 0 */
	kd070d10_reset_gpio();
	kd070d10_open();

	return 0;
}

static int kd070d10_match_id(void)
{
	return true;
}

static int kd070d10_set_active_win(struct lcd_img_rect *r)
{
	return 0;
}

static int kd070d10_set_rotation(int rotate)
{
	return 0;
}

static int kd070d10_close(void)
{
	return 0;
}

static struct rda_lcd_info kd070d10_info = {
	.name = KD070D10_PANEL_NAME,
	.ops = {
		.s_reset_gpio = kd070d10_reset_gpio,
		.s_open = kd070d10_open,
		.s_match_id = kd070d10_match_id,
		.s_active_win = kd070d10_set_active_win,
		.s_rotation = kd070d10_set_rotation,
		.s_sleep = kd070d10_sleep,
		.s_wakeup = kd070d10_wakeup,
		.s_close = kd070d10_close
	},
	.lcd = {
		.width = WVGA_LCDD_DISP_Y,
		.height = WVGA_LCDD_DISP_X,
		.lcd_interface = LCD_IF_DPI,
		.bpp = 32,
		.rgb_pinfo = {
			.pclk = 30000000,
			.pclk_divider = 12,
			.v_back_porch = 32,
			.v_front_porch = 13,
			.h_back_porch = 88,
			.h_front_porch = 40,
			.v_low = 5,
			.h_low = 5,
			.v_pol = false,
			.h_pol = false,
			.data_pol = false,
			.dot_clk_pol = false,
			.dpi_clk_adj = 0,/* 0-0xf */
			.rgb_if_fast_enable = false,
			.pixel_format = RGB_PIX_FMT_XRGB8888,
			.rgb_format_bits = RGB_FMT_24_BIT,
			.rgb_order = RGB_ORDER_BGR,
			.frame1 = false,
			.frame2 = true,
			.rgb_enable = true,
		},
	},
};

static ssize_t rda_kd070d10_gpio_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int gpiov = gpio_get_value(GPIO_LCD_RESET);
	return sprintf(buf, "gpio is %s level\n",gpiov ? "high" : "low");
}

static ssize_t rda_kd070d10_gpio_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	int set;

	printk("rda_kd070d10_gpio_store in\n");
	ret = kstrtoint(buf, 0, &set);
	if (ret < 0) {
		return ret;
	}

	set = !!set;

	gpio_set_value(GPIO_LCD_RESET,set);
	return count;
}

static DEVICE_ATTR(gpio, S_IWUSR | S_IWGRP | S_IRUGO,
	rda_kd070d10_gpio_show, rda_kd070d10_gpio_store);

static struct attribute *rda_kd070d10_attrs[] = {
	&dev_attr_gpio.attr,
	NULL
};

static const struct attribute_group rda_kd070d10_attr_group = {
	.attrs = rda_kd070d10_attrs,
};

static int rda_fb_panel_kd070d10_probe(struct spi_device *spi)
{
	struct rda_spi_panel_device *panel_dev;
	int ret = 0;

	printk("rda_fb_panel_kd070d10_probe\n");
	panel_dev = kzalloc(sizeof(*panel_dev), GFP_KERNEL);

	if (panel_dev == NULL) {
		dev_err(&spi->dev, "rda_fb_panel_kd070d10, out of memory\n");
		return -ENOMEM;
	}

	spi->mode = SPI_MODE_2;
	panel_dev->spi = spi;
	panel_dev->spi_xfer_num = 0;

	dev_set_drvdata(&spi->dev, panel_dev);

	spi->bits_per_word = 9;
	spi->max_speed_hz = 500000;
	spi->controller_data = &kd070d10_spicfg;

	ret = spi_setup(spi);

	if (ret < 0) {
		printk("error spi_setup failed\n");
		goto out_free_dev;
	}

	rda_kd070d10_spi_dev = panel_dev;

	rda_fb_register_panel(&kd070d10_info);

	ret = sysfs_create_group(&spi->dev.kobj, &rda_kd070d10_attr_group);
	if (ret)
		goto out_free_dev;

	dev_info(&spi->dev, "rda panel kd070d10 registered\n");
	return 0;

out_free_dev:

	kfree(panel_dev);
	panel_dev = NULL;

	return ret;
}

static int rda_fb_panel_kd070d10_remove(struct spi_device *spi)
{
	struct rda_spi_panel_device *panel_dev;
	panel_dev = dev_get_drvdata(&spi->dev);

	kfree(panel_dev);
	return 0;
}

#ifdef CONFIG_PM
static int rda_fb_panel_kd070d10_suspend(struct spi_device *spi, pm_message_t mesg)
{
	return 0;
}

static int rda_fb_panel_kd070d10_resume(struct spi_device *spi)
{
	return 0;
}
#else
#define rda_fb_panel_kd070d10_suspend NULL
#define rda_fb_panel_kd070d10_resume NULL
#endif /* CONFIG_PM */

/* The name is specific for each panel, it should be different from the
   abstract name "rda-fb-panel" */
static struct spi_driver rda_fb_panel_kd070d10_driver = {
	.driver = {
		.name = KD070D10_PANEL_NAME,
		.owner = THIS_MODULE,
	},
	.probe = rda_fb_panel_kd070d10_probe,
	.remove = rda_fb_panel_kd070d10_remove,
	.suspend = rda_fb_panel_kd070d10_suspend,
	.resume = rda_fb_panel_kd070d10_resume,
};

static struct rda_panel_driver kd070d10_mcu_panel_driver = {
	.panel_type = LCD_IF_DPI,
	.lcd_driver_info = &kd070d10_info,
	.rgb_panel_driver = &rda_fb_panel_kd070d10_driver,
};
