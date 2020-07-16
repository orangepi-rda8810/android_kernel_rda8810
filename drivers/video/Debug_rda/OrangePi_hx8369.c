/* drivers/video/sc8810/lcd_hx8369.c
 *
 * Support for hx8369 LCD device
 *
 * Copyright (C) 2010 Spreadtrum
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

#include <linux/kernel.h>
#include <mach/lcd.h>

#include "runyee_drv.h"

#define  LCD_DEBUG
#ifdef LCD_DEBUG
#define LCD_PRINT printk
#else
#define LCD_PRINT(...)
#endif

static void init_hx8369(struct lcd_spec *self) 
{
	unsigned int i;

	Send_data send_cmd = self->info.mcu->ops->send_cmd;
	Send_data send_data = self->info.mcu->ops->send_data;

#if 0
	send_cmd(0xB9);
	send_data(0xFF); 
	send_data(0x83); 
	send_data(0x69); 

	send_cmd(0xB1);
	send_data(0x01);
	send_data(0x00);
	send_data(0x34);
	send_data(0x07);
	send_data(0x00);
	send_data(0x0E);
	send_data(0x0E);
	send_data(0x16); //VSPR 21
	send_data(0x24); //VSNR 29
	send_data(0x2F);//3f
	send_data(0x2F);//3f
	send_data(0x01);
	send_data(0x23);
	send_data(0x01);
	send_data(0xE6);
	send_data(0xE6);
	send_data(0xE6);
	send_data(0xE6);
	send_data(0xE6);

	send_cmd(0xB2);
	send_data(0x00);
	send_data(0x20); //23 3-wire, 20 cpu
	send_data(0x07);
	send_data(0x07);
	send_data(0x70);
	send_data(0x00);
	send_data(0xFF);
	send_data(0x00);
	send_data(0x00);
	send_data(0x00);
	send_data(0x00);
	send_data(0x03);
	send_data(0x03);
	send_data(0x00);
	send_data(0x01);

	send_cmd(0xB4);	
	send_data(0x0A); // 00: column, 05: 1dot, 0A: 2dot 
	send_data(0x0C);
	send_data(0x84);
	send_data(0x0C);
	send_data(0x01);

	send_cmd(0xB6);	
	send_data(0x1F); // SET VCOM
	send_data(0x1F);

	//send_cmd(0xCC);
	send_cmd(0xCC);
	send_data(0x00); // Normally White
				
	send_cmd(0xD5);	// GIP	
	send_data(0x00);  
	send_data(0x01);  
	send_data(0x00);  
	send_data(0x00);  
	send_data(0x01);  
	send_data(0x06);
	send_data(0x10);  
	send_data(0x80);  
	send_data(0x73);  
	send_data(0x37);  
	send_data(0x01);  
	send_data(0x22);  
	send_data(0xb9);  
	send_data(0x75);  
	send_data(0xa8);  
	send_data(0x64);  
	send_data(0x00);  
	send_data(0x00);  
	send_data(0x41);  
	send_data(0x06);  
	send_data(0x50);  
	send_data(0x07);  
	send_data(0x07);  
	send_data(0x0F);  
	send_data(0x07);  
	send_data(0x00);
	
	send_cmd(0xE0);
	send_data(0x00);
	send_data(0x03);
	send_data(0x00);
	send_data(0x09);
	send_data(0x09);
	send_data(0x21);
	send_data(0x1B);
	send_data(0x2D);
	send_data(0x06);
	send_data(0x0C);
	send_data(0x10);
	send_data(0x15);
	send_data(0x16);
	send_data(0x14);
	send_data(0x16);
	send_data(0x12);
	send_data(0x18);
	send_data(0x00);
	send_data(0x03);
	send_data(0x00);
	send_data(0x09);
	send_data(0x09);
	send_data(0x21);
	send_data(0x1B);
	send_data(0x2D);
	send_data(0x06);
	send_data(0x0C);
	send_data(0x10);
	send_data(0x15);
	send_data(0x16);
	send_data(0x14);
	send_data(0x16);
	send_data(0x12);
	send_data(0x18);

	send_cmd(0x3A);  // set CSEL
	send_data(0x55);   // CSEL=0x66, 16bit-color

	send_cmd(0x35);
	send_data(0x00);
	
	send_cmd(0x44);	
	send_data(0x55); // SET VCOM
	send_data(0x00);//22 33
 
	send_cmd(0x2D); //Look up table// 
	for(i=0;i<64;i++)
	send_data(8*i);//RED  
	for(i=0;i<64;i++)
	send_data(4*i);//Green 
	for(i=0;i<64;i++)
	send_data(8*i);//Blue


	send_cmd(0x11); //Sleep Out
	LCD_DelayMS(200);

	send_cmd(0x29); //Display On
	LCD_DelayMS(30);	 
#else
	send_cmd(0xB9); //Set_EXTC
	send_data(0xFF);
	send_data(0x83);
	send_data(0x69);

	send_cmd(0xB1); //Set Power
	send_data(0x01);
	send_data(0x00);
	send_data(0x34);
	send_data(0x0A);
	send_data(0x00);
	send_data(0x11);
	send_data(0x12);
	send_data(0x1e);//
	send_data(0x1f);//
	send_data(0x3F);
	send_data(0x3F);
	send_data(0x01);
	send_data(0x1A);
	send_data(0x01);
	send_data(0xE6);
	send_data(0xE6);
	send_data(0xE6);
	send_data(0xE6);
	send_data(0xE6);

	send_cmd(0xB2); /* SET Display 480x800 */
	send_data(0x00);
	send_data(0x20);  //0x2b;0x20-MCU;0x29-DPI;RM,DM; RM=0:DPI IF;  RM=1:RGB IF;
	send_data(0x03);
	send_data(0x03);
	send_data(0x70);
	send_data(0x00);
	send_data(0xFF);
	send_data(0x00);
	send_data(0x00);
	send_data(0x00);
	send_data(0x00);
	send_data(0x03);
	send_data(0x03);
	send_data(0x00);
	send_data(0x01);

	send_cmd(0xB4); // SET Display column inversion
	send_data(0x00);         // 2Dot inversion
	send_data(0x18);
	send_data(0x70);
	send_data(0x13);
	send_data(0x05);
	send_cmd(0xB6); // SET VCOM
	send_data(0x50);
	send_data(0x50);

	send_cmd(0xD5); //SET GIP
	send_data(0x00);  //SHR 8-11 
	send_data(0x01);  //SHR 0-7    6
	send_data(0x03);    //SHR1 8-11 
	send_data(0x25);  //SHR1 0-7    //reset 808
	send_data(0x01); //SPD    stv LCD_DelayMS
	send_data(0x02); //CHR         8
	send_data(0x28); //CON     ck LCD_DelayMS
	send_data(0x70); //COFF /////////
	send_data(0x11); //SHP  SCP  stv high 1 hsync  stv 周期
	send_data(0x13); //CHP  CCP  CK HIGH 1 HSYNC CK周期3
	send_data(0x00); //CGOUT10_L CGOUT9_L  ML=0 
	send_data(0x00); //CGOUT10_R   
	send_data(0x40); //CGOUT6_L CGOUT5_L    //40 
	send_data(0xe6); //CGOUT8_L CGOUT7_L   //26 
	send_data(0x51); //CGOUT6_R CGOUT5_R   //51 
	send_data(0xf7); //CGOUT8_R CGOUT7_R   //37 
	send_data(0x00); //CGOUT10_L CGOUT9_L ML=1   
	send_data(0x00); //CGOUT10_R CGOUT9_R   
	send_data(0x71); //CGOUT6_L CGOUT5_L   
	send_data(0x35); //CGOUT8_L CGOUT7_L   
	send_data(0x60); //CGOUT6_R CGOUT5_R   
	send_data(0x24); //CGOUT8_R CGOUT7_R 
	send_data(0x07);   // GTO
	send_data(0x0F);    // GNO
	send_data(0x04);    // EQ LCD_DelayMS
	send_data(0x04);  // GIP 

	send_cmd(0xE0); //SET GAMMA
	send_data(0x00);
	send_data(0x02);
	send_data(0x0b);
	send_data(0x0a);
	send_data(0x09);
	send_data(0x18);
	send_data(0x1d);
	send_data(0x2a);
	send_data(0x08);
	send_data(0x11);
	send_data(0x0d);
	send_data(0x13);
	send_data(0x15);
	send_data(0x14);
	send_data(0x15);
	send_data(0x0f);
	send_data(0x14);
	send_data(0x00);
	send_data(0x02);
	send_data(0x0b);
	send_data(0x0a);
	send_data(0x09);
	send_data(0x18);
	send_data(0x1d);
	send_data(0x2a);
	send_data(0x08);
	send_data(0x11);
	send_data(0x0d);
	send_data(0x13);
	send_data(0x15);
	send_data(0x14);
	send_data(0x15);
	send_data(0x0f);
	send_data(0x14);

	send_cmd(0x36);
	send_data(0x00);

	send_cmd(0x3A); //Set COLMOD
	send_data(0x55);

	send_cmd(0x35);
	send_data(0x00);

	send_cmd(0x2D); 
	for (i=0; i<=63; i++)               
	send_data(i*8);                
	for (i=0; i<=63; i++)              
	send_data(i*4);   
	for (i=0; i<=63; i++)          
	send_data(i*8);   


	send_cmd(0x21);

	send_cmd(0x11); //Sleep Out
	LCD_DelayMS(150);
	send_cmd(0x29); //Display On
	LCD_DelayMS(50);

	send_cmd(0x2C);
#endif


}

static int32_t hx8369_init(struct lcd_spec *self)
{
	init_hx8369(self);

	return 0;
}

static int32_t hx8369_set_window(struct lcd_spec *self,
		uint16_t left, uint16_t top, uint16_t right, uint16_t bottom)
{
	Send_data send_cmd = self->info.mcu->ops->send_cmd;
	Send_data send_data = self->info.mcu->ops->send_data;

	LCD_PRINT("hx8369_set_window\n");
    
	send_cmd(0x2A); // col
	send_data((left >> 8));
	send_data((left & 0xFF));
	send_data((right >> 8));
	send_data((right & 0xFF));

	send_cmd(0x2B); // row
	send_data((top >> 8));
	send_data((top & 0xFF));
	send_data((bottom >> 8));
	send_data((bottom & 0xFF));
	
	send_cmd(0x2C); //Write data

	return 0;
}


static int32_t hx8369_invalidate(struct lcd_spec *self)
{
	LCD_PRINT("hx8369_invalidate\n");

	return self->ops->lcd_set_window(self, 0, 0, 
			self->width-1, self->height-1);
	
}

static int32_t hx8369_invalidate_rect(struct lcd_spec *self,
				uint16_t left, uint16_t top,
				uint16_t right, uint16_t bottom)
{
	Send_data send_cmd = self->info.mcu->ops->send_cmd;
	Send_data send_data = self->info.mcu->ops->send_data;

	LCD_PRINT("hx8369_invalidate_rect : (%d, %d, %d, %d)\n",left, top, right, bottom);

	send_cmd(0x44); // TE scanline
	send_data((top >> 8));
	send_data((top & 0xFF));

	return self->ops->lcd_set_window(self, left, top, 
			right, bottom);
}

static int32_t hx8369_set_direction(struct lcd_spec *self, uint16_t direction)
{
	Send_data send_cmd = self->info.mcu->ops->send_cmd;
	Send_data send_data = self->info.mcu->ops->send_data;

	LCD_PRINT("hx8369_set_direction\n");
	send_cmd(0x36);

	switch (direction) {
	case LCD_DIRECT_NORMAL:
		send_data(0);
		break;
	case LCD_DIRECT_ROT_90:
		send_data(0xA0);
		break;
	case LCD_DIRECT_ROT_180:
		send_data(0x60);
		break;
	case LCD_DIRECT_ROT_270:
		send_data(0xB0);
		break;
	case LCD_DIRECT_MIR_H:
		send_data(0x40);
		break;
	case LCD_DIRECT_MIR_V:
		send_data(0x10);
		break;
	case LCD_DIRECT_MIR_HV:
		send_data(0xE0);
		break;
	default:
		LCD_PRINT("unknown lcd direction!\n");
		send_data(0x0);
		direction = LCD_DIRECT_NORMAL;
		break;
	}

	self->direction = direction;
	
	return 0;
}

static int32_t hx8369_enter_sleep(struct lcd_spec *self, uint8_t is_sleep)
{
	Send_data send_cmd = self->info.mcu->ops->send_cmd;

	if(is_sleep) {
		//Sleep In
		send_cmd(0x28);
		LCD_DelayMS(120); 
		send_cmd(0x10);
		LCD_DelayMS(120); 
	}
	else {
		hx8369_init(self);
	}
	return 0;
}

static uint32_t hx8369_read_id(struct lcd_spec *self)
{
#if 1
	uint16_t lcd_id = 0x0;

	Send_data send_cmd = self->info.mcu->ops->send_cmd;
	Send_data send_data = self->info.mcu->ops->send_data;
	Read_data read_data = self->info.mcu->ops->read_data;

	send_cmd(0xB9);
	send_data(0xFF);
	send_data(0x83);
	send_data(0x69);
	LCD_DelayMS(10); 

	send_cmd(0xF4);
	LCD_DelayMS(10); 
	read_data();
	LCD_DelayMS(10); 
	lcd_id = read_data(); 

	LCD_PRINT("hx8369_read_id:[ kernel ] lcd_id = %x\r\n", lcd_id);
	
	return lcd_id;
#else
	return 0x69;
#endif
}

static struct lcd_operations lcd_hx8369_operations = {
	.lcd_init            = hx8369_init,
	.lcd_set_window      = hx8369_set_window,
	.lcd_invalidate      = hx8369_invalidate,
	.lcd_invalidate_rect = hx8369_invalidate_rect,
	.lcd_set_direction   = hx8369_set_direction,
	.lcd_enter_sleep     = hx8369_enter_sleep,
	.lcd_readid          = hx8369_read_id,
};

static struct timing_mcu lcd_hx8369_timing[] = {
[LCD_REGISTER_TIMING] = {                // read/write register timing
		.rcss = 25, //25 // 25 ns
		.rlpw = 70,
		.rhpw = 70,
		.wcss = 10,
		.wlpw = 15,//15
		.whpw = 15,//15
	},
[LCD_GRAM_TIMING] = {// read/write gram timing
		.rcss = 25,  // 25 ns
		.rlpw = 70,
		.rhpw = 70,
		.wcss = 10,
		.wlpw = 10,//15
		.whpw = 10,//15
	}
};

static struct info_mcu lcd_hx8369_info = {
	.bus_mode = LCD_BUS_8080,
	.bus_width = 16,
	.timing = lcd_hx8369_timing,
	.ops = NULL,
};

struct lcd_spec lcd_panel_hx8369 = {
	.width = 480,
	.height = 800,
	.mode = LCD_MODE_MCU,
	.direction = LCD_DIRECT_NORMAL,
	.info = {.mcu = &lcd_hx8369_info},
	.ops = &lcd_hx8369_operations,
};

