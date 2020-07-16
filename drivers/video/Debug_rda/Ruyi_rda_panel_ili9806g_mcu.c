#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/gpio.h>

#include <plat/devices.h>
#include <plat/rda_debug.h>
#include <mach/board.h>

#include "rda_gouda.h"
#include "rda_panel.h"

#define ILI9806G_MCU_CHIP_ID			0x9806
#ifdef CONFIG_PM
#include <linux/regulator/consumer.h>
#include <mach/regulator.h>
#endif /* CONFIG_PM */

//static u8 vcom=0x3b;
static struct rda_lcd_info ILI9806g_mcu_info;
static struct rda_panel_id_param ILI9806g_id_param;

/* wngl, for FPGA */

#if 1//def ILI9806G_MCU_397_XCX_K_20150813
#define ILI9806G_MCU_TIMING {\
	{\
		.tas = 7,\
		.tah = 7,\
		.pwl = 15,\
		.pwh = 15\
	}\
}
#else
#define ILI9806G_MCU_TIMING {\
	{\
		.tas = 15,\
		.tah = 15,\
		.pwl = 16,\
		.pwh = 16\
	}\
}
#endif
 
#define ILI9806G_MCU_CONFIG {\
	{\
		.cs = GOUDA_LCD_CS_0,\
		.output_fmt = GOUDA_LCD_OUTPUT_FORMAT_16_bit_RGB565,\
		.cs0_polarity = false,\
		.cs1_polarity = false,\
		.rs_polarity = false,\
		.wr_polarity = false,\
		.rd_polarity = false,\
		.te_en       =   1,\
		.tecon2	     = 0x100\
	}\
}

#define delayms(_ms_) msleep(_ms_)

static struct rda_lcd_info ILI9806g_mcu_info;

static int ILI9806g_mcu_init_gpio(void)
{

	gpio_request(GPIO_LCD_RESET, "lcd reset");
	gpio_direction_output(GPIO_LCD_RESET, 1);
	mdelay(1);
	gpio_set_value(GPIO_LCD_RESET, 0);
	mdelay(15);
	gpio_set_value(GPIO_LCD_RESET, 1);
	mdelay(10);

	return 0;
}

static int ili9806g_mcu_readid_sub(void)
{
	u16 data[6] = {0};
	u32 cmd = 0xd3;

	//LCD_MCU_CMD(0x11);
	//mdelay(20);
	/* read id */
	//while(1) 
//{ 
	LCD_MCU_CMD(0xD3);
	panel_mcu_read_id(cmd,&ILI9806g_id_param,data);

	printk(KERN_INFO "rda_fb: ILI9806g_mcu_lyq_20131211 ID:"
	       "%02x %02x %02x %02x %02x %02x\n",
	       data[0], data[1], data[2], data[3], data[4], data[5]);
//}
	if(data[2] == 0x98 && data[3] == 0x06)
		return 1;
	return 0;

}

static int ILI9806g_mcu_readid(void)
{
	struct regulator *lcd_reg;
	struct gouda_lcd *lcd = (void *)&(ILI9806g_mcu_info.lcd);
	int ret = 0;

#ifdef CONFIG_PM
	lcd_reg = regulator_get(NULL, LDO_LCD);
	if (IS_ERR(lcd_reg)) {
		printk(KERN_ERR"rda-fb not find lcd regulator devices\n");
		return 0;
	}

	if ( regulator_enable(lcd_reg)< 0) {
		printk(KERN_ERR"rda-fb lcd could not be enabled!\n");
		return 0;
	}
#endif /* CONFIG_PM */

	rda_gouda_pre_enable_lcd(lcd,1);
	ILI9806g_mcu_init_gpio();
	ret = ili9806g_mcu_readid_sub();
	rda_gouda_pre_enable_lcd(lcd,0);

#ifdef CONFIG_PM
	if ( regulator_disable(lcd_reg)< 0) {
		printk(KERN_ERR"rda-fb lcd could not be enabled!\n");
		//return 0;
	}
#endif /* CONFIG_PM */

	return ret;
}



static int ILI9806g_mcu_open(void)
{

	LCD_MCU_CMD(0x11);
	LCD_MCU_DATA(0x00);   
    	mdelay(120);
	LCD_MCU_CMD(0xB0);
	LCD_MCU_DATA(0x04);
	LCD_MCU_CMD(0xB3);
	LCD_MCU_DATA(0x02);
	LCD_MCU_DATA(0x00);
	LCD_MCU_CMD(0xB6);
	LCD_MCU_DATA(0x52);
	LCD_MCU_DATA(0x83);
	LCD_MCU_CMD(0xB7);
	LCD_MCU_DATA(0x80);
	LCD_MCU_DATA(0x72);
	LCD_MCU_DATA(0x11);
	LCD_MCU_DATA(0x25);
	LCD_MCU_CMD(0xB8);
	LCD_MCU_DATA(0x00);
	LCD_MCU_DATA(0x0F);
	LCD_MCU_DATA(0x0F);
	LCD_MCU_DATA(0xFF);
	LCD_MCU_DATA(0xFF);
	LCD_MCU_DATA(0xC8);
	LCD_MCU_DATA(0xC8);
	LCD_MCU_DATA(0x02);
	LCD_MCU_DATA(0x18);
	LCD_MCU_DATA(0x10);
	LCD_MCU_DATA(0x10);
	LCD_MCU_DATA(0x37);
	LCD_MCU_DATA(0x5A);
	LCD_MCU_DATA(0x87);
	LCD_MCU_DATA(0xBE);
	LCD_MCU_DATA(0xFF);
	LCD_MCU_DATA(0x00);
	LCD_MCU_DATA(0x00);
	LCD_MCU_DATA(0x00);
	LCD_MCU_DATA(0x00);
	LCD_MCU_CMD(0xB9);
	LCD_MCU_DATA(0x00);
	LCD_MCU_DATA(0x00);
	LCD_MCU_DATA(0x00);
	LCD_MCU_DATA(0x00);
	LCD_MCU_CMD(0xBD);
	LCD_MCU_DATA(0x00);
	LCD_MCU_CMD(0xC0);
	LCD_MCU_DATA(0x02);
	LCD_MCU_DATA(0x76);
	LCD_MCU_CMD(0xC1);
	LCD_MCU_DATA(0x63);
	LCD_MCU_DATA(0x31);
	LCD_MCU_DATA(0x00);
	LCD_MCU_DATA(0x27);
	LCD_MCU_DATA(0x27);
	LCD_MCU_DATA(0x32);
	LCD_MCU_DATA(0x12);
	LCD_MCU_DATA(0x28);
	LCD_MCU_DATA(0x4E);
	LCD_MCU_DATA(0x10);
	LCD_MCU_DATA(0xA5);
	LCD_MCU_DATA(0x0F);
	LCD_MCU_DATA(0x58);
	LCD_MCU_DATA(0x21);
	LCD_MCU_DATA(0x01);
	LCD_MCU_CMD(0xC2);
	LCD_MCU_DATA(0x28);
	LCD_MCU_DATA(0x06);
	LCD_MCU_DATA(0x06);
	LCD_MCU_DATA(0x01);
	LCD_MCU_DATA(0x03);
	LCD_MCU_DATA(0x00);
	LCD_MCU_CMD(0xC3);
	LCD_MCU_DATA(0x40);
	LCD_MCU_DATA(0x00);
	LCD_MCU_DATA(0x03);
	LCD_MCU_CMD(0xC4);
	LCD_MCU_DATA(0x00);
	LCD_MCU_DATA(0x01);
	LCD_MCU_CMD(0xC6);
	LCD_MCU_DATA(0x00);
	LCD_MCU_DATA(0x00);
	LCD_MCU_CMD(0xC7);
	LCD_MCU_DATA(0x11);
	LCD_MCU_DATA(0x8D);
	LCD_MCU_DATA(0xA0);
	LCD_MCU_DATA(0xF5);
	LCD_MCU_DATA(0x27);
	LCD_MCU_CMD(0xC8);
	LCD_MCU_DATA(0x02);
	LCD_MCU_DATA(0x13);
	LCD_MCU_DATA(0x18);
	LCD_MCU_DATA(0x25);
	LCD_MCU_DATA(0x34);
	LCD_MCU_DATA(0x4E);
	LCD_MCU_DATA(0x36);
	LCD_MCU_DATA(0x23);
	LCD_MCU_DATA(0x17);
	LCD_MCU_DATA(0x0E);
	LCD_MCU_DATA(0x0C);
	LCD_MCU_DATA(0x02);
	LCD_MCU_DATA(0x02);
	LCD_MCU_DATA(0x13);
	LCD_MCU_DATA(0x18);
	LCD_MCU_DATA(0x25);
	LCD_MCU_DATA(0x34);
	LCD_MCU_DATA(0x4E);LCD_MCU_DATA(0x36);LCD_MCU_DATA(0x23);LCD_MCU_DATA(0x17);LCD_MCU_DATA(0x0E);LCD_MCU_DATA(0x0C);LCD_MCU_DATA(0x02);
	LCD_MCU_CMD(0xC9);LCD_MCU_DATA(0x02);LCD_MCU_DATA(0x13);LCD_MCU_DATA(0x18);LCD_MCU_DATA(0x25);LCD_MCU_DATA(0x34);LCD_MCU_DATA(0x4E);LCD_MCU_DATA(0x36);LCD_MCU_DATA(0x23);LCD_MCU_DATA(0x17);LCD_MCU_DATA(0x0E);LCD_MCU_DATA(0x0C);LCD_MCU_DATA(0x02);LCD_MCU_DATA(0x02);LCD_MCU_DATA(0x13);LCD_MCU_DATA(0x18);LCD_MCU_DATA(0x25);LCD_MCU_DATA(0x34);LCD_MCU_DATA(0x4E);LCD_MCU_DATA(0x36);LCD_MCU_DATA(0x23);LCD_MCU_DATA(0x17);LCD_MCU_DATA(0x0E);LCD_MCU_DATA(0x0C);LCD_MCU_DATA(0x02);
	LCD_MCU_CMD(0xCA);LCD_MCU_DATA(0x02);LCD_MCU_DATA(0x13);LCD_MCU_DATA(0x18);LCD_MCU_DATA(0x25);LCD_MCU_DATA(0x34);LCD_MCU_DATA(0x4E);LCD_MCU_DATA(0x36);LCD_MCU_DATA(0x23);LCD_MCU_DATA(0x17);LCD_MCU_DATA(0x0E);LCD_MCU_DATA(0x0C);LCD_MCU_DATA(0x02);LCD_MCU_DATA(0x02);LCD_MCU_DATA(0x13);LCD_MCU_DATA(0x18);LCD_MCU_DATA(0x25);LCD_MCU_DATA(0x34);LCD_MCU_DATA(0x4E);LCD_MCU_DATA(0x36);LCD_MCU_DATA(0x23);LCD_MCU_DATA(0x17);LCD_MCU_DATA(0x0E);LCD_MCU_DATA(0x0C);LCD_MCU_DATA(0x02);

	LCD_MCU_CMD(0xD0);LCD_MCU_DATA(0xA9);LCD_MCU_DATA(0x03);LCD_MCU_DATA(0xCC);LCD_MCU_DATA(0xA5);LCD_MCU_DATA(0x00);LCD_MCU_DATA(0x53);LCD_MCU_DATA(0x20);LCD_MCU_DATA(0x10);LCD_MCU_DATA(0x01);LCD_MCU_DATA(0x00);LCD_MCU_DATA(0x01);LCD_MCU_DATA(0x01);LCD_MCU_DATA(0x00);LCD_MCU_DATA(0x03);LCD_MCU_DATA(0x01);LCD_MCU_DATA(0x00);
	LCD_MCU_CMD(0xD1);LCD_MCU_DATA(0x18);LCD_MCU_DATA(0x0C);LCD_MCU_DATA(0x23);LCD_MCU_DATA(0x03);LCD_MCU_DATA(0x75);LCD_MCU_DATA(0x02);LCD_MCU_DATA(0x50);
	LCD_MCU_CMD(0xD3);LCD_MCU_DATA(0x33);
	LCD_MCU_CMD(0xD5);LCD_MCU_DATA(0x2a);LCD_MCU_DATA(0x2a);

	LCD_MCU_CMD(0xD6);LCD_MCU_DATA(0x28);//a8
	LCD_MCU_CMD(0xD7);LCD_MCU_DATA(0x01);LCD_MCU_DATA(0x00);LCD_MCU_DATA(0xAA);LCD_MCU_DATA(0xC0);LCD_MCU_DATA(0x2A);LCD_MCU_DATA(0x2C);LCD_MCU_DATA(0x22);LCD_MCU_DATA(0x12);LCD_MCU_DATA(0x71);LCD_MCU_DATA(0x0A);LCD_MCU_DATA(0x12);LCD_MCU_DATA(0x00);LCD_MCU_DATA(0xA0);LCD_MCU_DATA(0x00);LCD_MCU_DATA(0x03);
	LCD_MCU_CMD(0xD8);LCD_MCU_DATA(0x44);LCD_MCU_DATA(0x44);LCD_MCU_DATA(0x22);LCD_MCU_DATA(0x44);LCD_MCU_DATA(0x21);LCD_MCU_DATA(0x46);LCD_MCU_DATA(0x42);LCD_MCU_DATA(0x40);
	LCD_MCU_CMD(0xD9);LCD_MCU_DATA(0xCF);LCD_MCU_DATA(0x2D);LCD_MCU_DATA(0x51);
	LCD_MCU_CMD(0xDA);LCD_MCU_DATA(0x01);

	LCD_MCU_CMD(0xDE);LCD_MCU_DATA(0x01);LCD_MCU_DATA(0x51);//58
	LCD_MCU_CMD(0xE1);LCD_MCU_DATA(0x00);LCD_MCU_DATA(0x00);LCD_MCU_DATA(0x00);LCD_MCU_DATA(0x00);LCD_MCU_DATA(0x00);LCD_MCU_DATA(0x00);
	LCD_MCU_CMD(0xE6);LCD_MCU_DATA(0x55);//58
	LCD_MCU_CMD(0xF3);LCD_MCU_DATA(0x06);LCD_MCU_DATA(0x00);LCD_MCU_DATA(0x00);LCD_MCU_DATA(0x24);LCD_MCU_DATA(0x00);
	LCD_MCU_CMD(0xF8);LCD_MCU_DATA(0x00);
	LCD_MCU_CMD(0xFA);LCD_MCU_DATA(0x01);
	LCD_MCU_CMD(0xFB);LCD_MCU_DATA(0x00);LCD_MCU_DATA(0x00);LCD_MCU_DATA(0x00);
	LCD_MCU_CMD(0xFC);LCD_MCU_DATA(0x00);LCD_MCU_DATA(0x00);LCD_MCU_DATA(0x00);LCD_MCU_DATA(0x00);LCD_MCU_DATA(0x00);
	LCD_MCU_CMD(0xFD);LCD_MCU_DATA(0x00);LCD_MCU_DATA(0x00);LCD_MCU_DATA(0x70);LCD_MCU_DATA(0x00);LCD_MCU_DATA(0x72);LCD_MCU_DATA(0x31);LCD_MCU_DATA(0x37);LCD_MCU_DATA(0x70);LCD_MCU_DATA(0x32);LCD_MCU_DATA(0x31);LCD_MCU_DATA(0x07);LCD_MCU_DATA(0x00);LCD_MCU_DATA(0x00);
	LCD_MCU_CMD(0xFE);LCD_MCU_DATA(0x00);LCD_MCU_DATA(0x00);LCD_MCU_DATA(0x00);LCD_MCU_DATA(0x00);LCD_MCU_DATA(0x20);
	LCD_MCU_CMD(0xB0);LCD_MCU_DATA(0x04); //04
   
	LCD_MCU_CMD(0x35);LCD_MCU_DATA(0x00);
	LCD_MCU_CMD(0x44);LCD_MCU_DATA(0x00);
	LCD_MCU_CMD(0x36);LCD_MCU_DATA(0x00);
	LCD_MCU_CMD(0x3A);LCD_MCU_DATA(0x55);
	LCD_MCU_CMD(0x2A);LCD_MCU_DATA(0x00);LCD_MCU_DATA(0x00);LCD_MCU_DATA(0x01);LCD_MCU_DATA(0xDF);
	LCD_MCU_CMD(0x2B);LCD_MCU_DATA(0x00);LCD_MCU_DATA(0x00);LCD_MCU_DATA(0x03);LCD_MCU_DATA(0x1F);

	LCD_MCU_CMD(0x29);LCD_MCU_DATA(0x00);     
     	mdelay(180);
 	LCD_MCU_CMD(0x2C);LCD_MCU_DATA(0x00); 
	
	return 0;
}




static int ILI9806g_mcu_sleep(void)
{

	return 0;
}



static int ILI9806g_mcu_wakeup(void)
{
	gpio_set_value(GPIO_LCD_RESET, 1);
	mdelay(1);
	gpio_set_value(GPIO_LCD_RESET, 0);
	mdelay(15);
	gpio_set_value(GPIO_LCD_RESET, 1);
	mdelay(10);
	ILI9806g_mcu_open();

	return 0;
}



static int ILI9806g_mcu_set_active_win(struct gouda_rect *r)
{
	return 0;
}



static int ILI9806g_mcu_set_rotation(int rotate)
{
	return 0;
}

static int ILI9806g_mcu_close(void)
{
	return 0;
}

static struct rda_panel_id_param ILI9806g_id_param = {
	.lcd_info = &ILI9806g_mcu_info,
	.per_read_bytes = 4,
};

static struct rda_lcd_info ILI9806g_mcu_info = {
	.ops = {
		.s_init_gpio = ILI9806g_mcu_init_gpio,
		.s_open = ILI9806g_mcu_open,
		.s_readid = ILI9806g_mcu_readid,
		.s_active_win = ILI9806g_mcu_set_active_win,
		.s_rotation = ILI9806g_mcu_set_rotation,
		.s_sleep = ILI9806g_mcu_sleep,
		.s_wakeup = ILI9806g_mcu_wakeup,
		.s_close = ILI9806g_mcu_close
	},
	.lcd = {
		.width = WVGA_LCDD_DISP_X,
		.height = WVGA_LCDD_DISP_Y,
		.lcd_interface = GOUDA_LCD_IF_DBI,
		.lcd_timing = ILI9806G_MCU_TIMING,
		.lcd_cfg = ILI9806G_MCU_CONFIG
	},
	.name = ILI9806G_MCU_PANEL_NAME,
};

/*--------------------Platform Device Probe-------------------------*/

static int rda_fb_panel_ILI9806g_mcu_probe(struct platform_device *pdev)
{
	rda_fb_register_panel(&ILI9806g_mcu_info);

	dev_info(&pdev->dev, "rda panel ILI9806g_mcu registered\n");

	return 0;
}

static int rda_fb_panel_ILI9806g_mcu_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver rda_fb_panel_ILI9806g_mcu_driver = {
	.probe = rda_fb_panel_ILI9806g_mcu_probe,
	.remove = rda_fb_panel_ILI9806g_mcu_remove,
	.driver = {
		.name = ILI9806G_MCU_PANEL_NAME
	}
};

static struct rda_panel_driver ili9806g_mcu_panel_driver = {
	.panel_type = GOUDA_LCD_IF_DBI,
	.lcd_driver_info = &ILI9806g_mcu_info,
	.pltaform_panel_driver = &rda_fb_panel_ILI9806g_mcu_driver,
};

static int __init rda_fb_panel_ILI9806g_mcu_init(void)
{
	rda_fb_probe_panel(&ILI9806g_mcu_info, &rda_fb_panel_ILI9806g_mcu_driver);
	return platform_driver_register(&rda_fb_panel_ILI9806g_mcu_driver);
}

static void __exit rda_fb_panel_ILI9806g_mcu_exit(void)
{
	platform_driver_unregister(&rda_fb_panel_ILI9806g_mcu_driver);
}

module_init(rda_fb_panel_ILI9806g_mcu_init);
module_exit(rda_fb_panel_ILI9806g_mcu_exit);
