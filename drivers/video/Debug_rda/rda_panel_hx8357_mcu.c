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

#define HX8357_MCU_CHIP_ID			0x8357

/* wngl, for FPGA */
#define HX8357_MCU_TIMING {						     \
	{.tas       =  6,                                            \
	.tah        =  6,                                            \
	.pwl        =  16,                                           \
	.pwh        =  16}}                                          \

#define HX8357_MCU_CONFIG {                                             \
	{.cs             =   GOUDA_LCD_CS_0,                         \
	.output_fmt      =   GOUDA_LCD_OUTPUT_FORMAT_16_bit_RGB565,   \
	.cs0_polarity    =   false,                                  \
	.cs1_polarity    =   false,                                  \
	.rs_polarity     =   false,                                  \
	.wr_polarity     =   false,                                  \
	.rd_polarity     =   false,									\
	.te_en           =   1,				     	     			\
	.tecon2		 = 0x100}}

static int hx8357_mcu_init_gpio(void)
{


	gpio_request(GPIO_LCD_RESET, "lcd reset");
	gpio_direction_output(GPIO_LCD_RESET, 1);
	mdelay(1);
	gpio_set_value(GPIO_LCD_RESET, 0);
	mdelay(100);
	gpio_set_value(GPIO_LCD_RESET, 1);
	mdelay(50);

	return 0;
}

static int hx8357_mcu_open(void)
{
	int i;

	LCD_MCU_CMD(0xB9); //Set_EXTC
	LCD_MCU_DATA(0xFF);
	LCD_MCU_DATA(0x83);
	LCD_MCU_DATA(0x69);

	LCD_MCU_CMD(0xB1); //Set Power
	LCD_MCU_DATA(0x01);
	LCD_MCU_DATA(0x00);
	LCD_MCU_DATA(0x34);
	LCD_MCU_DATA(0x0A);
	LCD_MCU_DATA(0x00);
	LCD_MCU_DATA(0x11);
	LCD_MCU_DATA(0x12);
	LCD_MCU_DATA(0x1e);//
	LCD_MCU_DATA(0x1f);//
	LCD_MCU_DATA(0x3F);
	LCD_MCU_DATA(0x3F);
	LCD_MCU_DATA(0x01);
	LCD_MCU_DATA(0x1A);
	LCD_MCU_DATA(0x01);
	LCD_MCU_DATA(0xE6);
	LCD_MCU_DATA(0xE6);
	LCD_MCU_DATA(0xE6);
	LCD_MCU_DATA(0xE6);
	LCD_MCU_DATA(0xE6);

	LCD_MCU_CMD(0xB2); /* SET Display 480x800 */
	LCD_MCU_DATA(0x00);
	LCD_MCU_DATA(0x20);  //0x2b;0x20-MCU;0x29-DPI;RM,DM; RM=0:DPI IF;  RM=1:RGB IF;
	LCD_MCU_DATA(0x03);
	LCD_MCU_DATA(0x03);
	LCD_MCU_DATA(0x70);
	LCD_MCU_DATA(0x00);
	LCD_MCU_DATA(0xFF);
	LCD_MCU_DATA(0x00);
	LCD_MCU_DATA(0x00);
	LCD_MCU_DATA(0x00);
	LCD_MCU_DATA(0x00);
	LCD_MCU_DATA(0x03);
	LCD_MCU_DATA(0x03);
	LCD_MCU_DATA(0x00);
	LCD_MCU_DATA(0x01);

	LCD_MCU_CMD(0xB4); // SET Display column inversion
	LCD_MCU_DATA(0x00);         // 2Dot inversion
	LCD_MCU_DATA(0x18);
	LCD_MCU_DATA(0x70);
	LCD_MCU_DATA(0x13);
	LCD_MCU_DATA(0x05);
	LCD_MCU_CMD(0xB6); // SET VCOM
	LCD_MCU_DATA(0x50);
	LCD_MCU_DATA(0x50);

	LCD_MCU_CMD(0xD5); //SET GIP
	LCD_MCU_DATA(0x00);  //SHR 8-11 
	LCD_MCU_DATA(0x01);  //SHR 0-7    6
	LCD_MCU_DATA(0x03);    //SHR1 8-11 
	LCD_MCU_DATA(0x25);  //SHR1 0-7    //reset 808
	LCD_MCU_DATA(0x01); //SPD    stv LCD_DelayMS
	LCD_MCU_DATA(0x02); //CHR         8
	LCD_MCU_DATA(0x28); //CON     ck LCD_DelayMS
	LCD_MCU_DATA(0x70); //COFF /////////
	LCD_MCU_DATA(0x11); //SHP  SCP  stv high 1 hsync  stv ÖÜÆÚ
	LCD_MCU_DATA(0x13); //CHP  CCP  CK HIGH 1 HSYNC CKÖÜÆÚ3
	LCD_MCU_DATA(0x00); //CGOUT10_L CGOUT9_L  ML=0 
	LCD_MCU_DATA(0x00); //CGOUT10_R   
	LCD_MCU_DATA(0x40); //CGOUT6_L CGOUT5_L    //40 
	LCD_MCU_DATA(0xe6); //CGOUT8_L CGOUT7_L   //26 
	LCD_MCU_DATA(0x51); //CGOUT6_R CGOUT5_R   //51 
	LCD_MCU_DATA(0xf7); //CGOUT8_R CGOUT7_R   //37 
	LCD_MCU_DATA(0x00); //CGOUT10_L CGOUT9_L ML=1   
	LCD_MCU_DATA(0x00); //CGOUT10_R CGOUT9_R   
	LCD_MCU_DATA(0x71); //CGOUT6_L CGOUT5_L   
	LCD_MCU_DATA(0x35); //CGOUT8_L CGOUT7_L   
	LCD_MCU_DATA(0x60); //CGOUT6_R CGOUT5_R   
	LCD_MCU_DATA(0x24); //CGOUT8_R CGOUT7_R 
	LCD_MCU_DATA(0x07);   // GTO
	LCD_MCU_DATA(0x0F);    // GNO
	LCD_MCU_DATA(0x04);    // EQ LCD_DelayMS
	LCD_MCU_DATA(0x04);  // GIP 

	LCD_MCU_CMD(0xE0); //SET GAMMA
	LCD_MCU_DATA(0x00);
	LCD_MCU_DATA(0x02);
	LCD_MCU_DATA(0x0b);
	LCD_MCU_DATA(0x0a);
	LCD_MCU_DATA(0x09);
	LCD_MCU_DATA(0x18);
	LCD_MCU_DATA(0x1d);
	LCD_MCU_DATA(0x2a);
	LCD_MCU_DATA(0x08);
	LCD_MCU_DATA(0x11);
	LCD_MCU_DATA(0x0d);
	LCD_MCU_DATA(0x13);
	LCD_MCU_DATA(0x15);
	LCD_MCU_DATA(0x14);
	LCD_MCU_DATA(0x15);
	LCD_MCU_DATA(0x0f);
	LCD_MCU_DATA(0x14);
	LCD_MCU_DATA(0x00);
	LCD_MCU_DATA(0x02);
	LCD_MCU_DATA(0x0b);
	LCD_MCU_DATA(0x0a);
	LCD_MCU_DATA(0x09);
	LCD_MCU_DATA(0x18);
	LCD_MCU_DATA(0x1d);
	LCD_MCU_DATA(0x2a);
	LCD_MCU_DATA(0x08);
	LCD_MCU_DATA(0x11);
	LCD_MCU_DATA(0x0d);
	LCD_MCU_DATA(0x13);
	LCD_MCU_DATA(0x15);
	LCD_MCU_DATA(0x14);
	LCD_MCU_DATA(0x15);
	LCD_MCU_DATA(0x0f);
	LCD_MCU_DATA(0x14);

	LCD_MCU_CMD(0x36);
	LCD_MCU_DATA(0x00);

	LCD_MCU_CMD(0x3A); //Set COLMOD
	LCD_MCU_DATA(0x55);

	LCD_MCU_CMD(0x35);
	LCD_MCU_DATA(0x00);

	LCD_MCU_CMD(0x2D); 
	for (i=0; i<=63; i++)               
	LCD_MCU_DATA(i*8);                
	for (i=0; i<=63; i++)              
	LCD_MCU_DATA(i*4);   
	for (i=0; i<=63; i++)          
	LCD_MCU_DATA(i*8);   


	LCD_MCU_CMD(0x21);

	LCD_MCU_CMD(0x11); //Sleep Out
//	LCD_DelayMS(150);
	LCD_MCU_CMD(0x29); //Display On
//	LCD_DelayMS(50);

	LCD_MCU_CMD(0x2C);
	return 0;
}

static int hx8357_mcu_sleep(void)
{
	gpio_set_value(GPIO_LCD_RESET, 0);
	mdelay(10);
	return 0;
}

static int hx8357_mcu_wakeup(void)
{
	gpio_direction_output(GPIO_LCD_RESET, 1);
	mdelay(1);
	gpio_set_value(GPIO_LCD_RESET, 0);
	mdelay(10);
	gpio_set_value(GPIO_LCD_RESET, 1);
	mdelay(5);
	hx8357_mcu_open();
	return 0;
}

static int hx8357_mcu_set_active_win(struct gouda_rect *r)
{
	LCD_MCU_CMD(0x2a);	/* Set Column Address */
	LCD_MCU_DATA(r->tlX >> 8);
	LCD_MCU_DATA(r->tlX & 0x00ff);
	LCD_MCU_DATA((r->brX) >> 8);
	LCD_MCU_DATA((r->brX) & 0x00ff);

	LCD_MCU_CMD(0x2b);	/* Set Page Address */
	LCD_MCU_DATA(r->tlY >> 8);
	LCD_MCU_DATA(r->tlY & 0x00ff);
	LCD_MCU_DATA((r->brY) >> 8);
	LCD_MCU_DATA((r->brY) & 0x00ff);

	LCD_MCU_CMD(0x2c);

	return 0;
}

static int hx8357_mcu_display_on(void)
{
	return 0;
}

static int hx8357_mcu_display_off(void)
{
	return 0;
}

static int hx8357_mcu_set_rotation(int rotate)
{
	return 0;
}

static int hx8357_mcu_close(void)
{
	return 0;
}

static struct rda_lcd_info hx8357_mcu_info = {
	.ops = {
		.s_init_gpio = hx8357_mcu_init_gpio,
		.s_open = hx8357_mcu_open,
		.s_active_win = hx8357_mcu_set_active_win,
		.s_rotation = hx8357_mcu_set_rotation,
		.s_sleep = hx8357_mcu_sleep,
		.s_wakeup = hx8357_mcu_wakeup,
		.s_close = hx8357_mcu_close,
		.s_display_on = hx8357_mcu_display_on,
		.s_display_off = hx8357_mcu_display_off
	},
	.lcd = {
		.width = HVGA_LCDD_DISP_X,
		.height = HVGA_LCDD_DISP_Y,
		.lcd_interface = GOUDA_LCD_IF_DBI,
		.lcd_timing = HX8357_MCU_TIMING,
		.lcd_cfg = HX8357_MCU_CONFIG},
	.name = HX8357_MCU_PANEL_NAME,
};

/*--------------------Platform Device Probe-------------------------*/

static int rda_fb_panel_hx8357_mcu_probe(struct platform_device *pdev)
{
	rda_fb_register_panel(&hx8357_mcu_info);

	dev_info(&pdev->dev, "rda panel hx8357_mcu registered\n");

	return 0;
}

static int rda_fb_panel_hx8357_mcu_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver rda_fb_panel_hx8357_mcu_driver = {
	.probe = rda_fb_panel_hx8357_mcu_probe,
	.remove = rda_fb_panel_hx8357_mcu_remove,
	.driver = {
		   .name = HX8357_MCU_PANEL_NAME}
};

static struct rda_panel_driver hx8357_mcu_panel_driver = {
	.panel_type = GOUDA_LCD_IF_DBI,
	.lcd_driver_info = &hx8357_mcu_info,
	.pltaform_panel_driver = &rda_fb_panel_hx8357_mcu_driver,
};
