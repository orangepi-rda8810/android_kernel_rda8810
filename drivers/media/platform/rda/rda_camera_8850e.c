/*
 * V4L2 driver for RDA camera host
 *
 * Copyright (C) 2014 Rda electronics, Inc.
 *
 * Contact: Xing Wei <xingwei@rdamicro.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 * This is only for sp2508 raw snsor
 */

#include <linux/clk.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include <media/soc_camera.h>
#include <media/soc_mediabus.h>
#include <media/videobuf2-dma-contig.h>

#include <mach/rda_clk_name.h>
#include <plat/devices.h>
#include <plat/rda_debug.h>
#include <plat/reg_cam_8850e.h>
#include <plat/pm_ddr.h>

#include <plat/reg_sysctrl_8850e.h>

#include "tgt_ap_board_config.h"
#include "tgt_ap_clock_config.h"
#include <rda_isp_reg.h>

/* Macros */
#define MAX_BUFFER_NUM		32
#define VID_LIMIT_BYTES		(16 * 1024 * 1024)
#define MAX_SUPPORT_WIDTH	2048
#define MAX_SUPPORT_HEIGHT	2048

#define RDA_CAM_MBUS_PARA	(V4L2_MBUS_MASTER		|\
				V4L2_MBUS_HSYNC_ACTIVE_HIGH	|\
				V4L2_MBUS_HSYNC_ACTIVE_LOW	|\
				V4L2_MBUS_VSYNC_ACTIVE_HIGH	|\
				V4L2_MBUS_VSYNC_ACTIVE_LOW	|\
				V4L2_MBUS_PCLK_SAMPLE_RISING	|\
				V4L2_MBUS_PCLK_SAMPLE_FALLING	|\
				V4L2_MBUS_DATA_ACTIVE_HIGH)

#define RDA_CAM_MBUS_CSI2	V4L2_MBUS_CSI2_LANES		|\
				V4L2_MBUS_CSI2_CONTINUOUS_CLOCK

#define CAM_OUT_MCLK		(_TGT_AP_PLL_BUS_FREQ >> 3)

/* Global Var */
#ifdef _TGT_AP_CAM_ISP_ENABLE
static int ISP_INIT = 0;
#endif
static void __iomem *cam_regs = NULL;

/* Structure */
//static struct tasklet_struct rcam_tasklet;

/* Capture Buffer */
struct cap_buffer {
	struct vb2_buffer vb;
	struct list_head list;
	unsigned int dma_addr;
};

/* RDA camera device */
struct rda_camera_dev {
	struct soc_camera_host soc_host;
	struct soc_camera_device *icd;

	struct list_head cap_buffer_list;
	struct cap_buffer *active;
	struct cap_buffer *next;
	struct vb2_alloc_ctx *alloc_ctx;
	struct delayed_work isr_work;
//	struct v4l2_rect crop_rect;
	struct  raw_sensor_info_data *raw_sensor;
	int state;
	int sequence;
	int width;
	int height;
	wait_queue_head_t vsync_wq;
	spinlock_t lock;

	struct rda_camera_device_data *pdata;

	struct clk *pclk;
	void __iomem *regs;
	unsigned int irq;
};

static const struct soc_mbus_pixelfmt rda_camera_formats[] = {
	{
		.fourcc = V4L2_PIX_FMT_YUYV,
		.name = "Packed YUV422 16 bit",
		.bits_per_sample = 8,
		.packing = SOC_MBUS_PACKING_2X8_PADHI,
		.order = SOC_MBUS_ORDER_LE,
		.layout = SOC_MBUS_LAYOUT_PACKED,
	},
};

/* camera states */
enum {
	CAM_STATE_IDLE = 0,
	CAM_STATE_ONESHOT,
	CAM_STATE_SUCCESS,
};

/* -----------------------------------------------------------------
 * Public functions for sensor
 * -----------------------------------------------------------------*/
void rcam_pdn(bool pdn, bool acth)
{
	HWP_CAMERA_T *hwp_cam = (HWP_CAMERA_T*)cam_regs;

	if (acth)
		hwp_cam->CTRL &= ~CAMERA_PWDN_POL_INVERT;
	else
		hwp_cam->CTRL |= CAMERA_PWDN_POL_INVERT;

	if (pdn)
		hwp_cam->CMD_SET = CAMERA_PWDN;
	else
		hwp_cam->CMD_CLR = CAMERA_PWDN;
}

void rcam_rst(bool rst, bool acth)
{
	HWP_CAMERA_T *hwp_cam = (HWP_CAMERA_T*)cam_regs;

	if (acth)
		hwp_cam->CTRL &= ~CAMERA_RESET_POL_INVERT;
	else
		hwp_cam->CTRL |= CAMERA_RESET_POL_INVERT;

	if (rst)
		hwp_cam->CMD_SET = CAMERA_RESET;
	else
		hwp_cam->CMD_CLR = CAMERA_RESET;
}

void rcam_clk(bool out, int freq)
{
	HWP_CAMERA_T *hwp_cam = (HWP_CAMERA_T*)cam_regs;
	unsigned int val = 0x1;
	int tmp;

	if (out) {
		if (freq == 13)
			val |= 0x1 << 4;
		else if (freq == 26)
			val |= 0x2 << 12;
		else {
			tmp = CAM_OUT_MCLK / freq;
			if (tmp < 2)
				tmp = 2;
			else if (tmp > 17)
				tmp = 17;
			val |= (tmp - 2) << 8;
		}
		hwp_cam->CLK_OUT = val;
	} else {
		hwp_cam->CLK_OUT = 0x3f00;
	}
}


#ifdef _TGT_AP_CAM_ISP_ENABLE
/*
extern void rda_sensor_update_gain(int val);
extern void rda_sensor_update_exp(int val);

void rcam_isp_set_awb();
void rcam_isp_set_anti_flicker();

*/

unsigned char rcam_isp_cur_bright_val_read(void)
{
	ISP_CAMERA_T *cam_isp = (ISP_CAMERA_T*)((void *)(cam_regs )+ REG_ISP_OFFS);
	return cam_isp->PAGE_X.ISP_PAGE0.YAVE_OUT;

}


unsigned char rcam_isp_reg_read(unsigned int addr_offset)
{

	unsigned char * cam_isp_regs = (void *)cam_regs;

	return cam_isp_regs[addr_offset] ;
}

/* write data to ISP register, the addr_offset is the offset to camera interface (CIF)*/
void rcam_isp_reg_write(unsigned int addr_offset, unsigned char data)
{

	unsigned char * cam_isp_regs = (void *)cam_regs;

	cam_isp_regs[addr_offset] = data;
	return;
}

/* write ISP data array, with register and the data */
void rcam_isp_reg_write_array(struct isp_reg_list *regs)
{
	unsigned char * cam_isp_regs = (void *)cam_regs;
	struct isp_reg *tmp = NULL;
	int i;

	for (i = 0; i < regs->size; i++) {
		tmp = regs->val + i;
		cam_isp_regs[tmp->addr_offset] = tmp->data;
	}

	return;
}

#endif

void rcam_config_csi(unsigned int d, unsigned int c,
		unsigned int line, unsigned int flag)
{
	HWP_CAMERA_T *hwp_cam = (HWP_CAMERA_T*)cam_regs;
	unsigned char d_term_en = (d >> 16) & 0xff;
	unsigned char d_hs_setl = d & 0xff;
	unsigned short c_term_en = c >> 16;
	unsigned short c_hs_setl = c & 0xffff;

	unsigned int frame_line_l = line & 0x3ff;
	unsigned int frame_line_h = (line >> 10) & 0x7;
	unsigned char ch_sel = flag & 0x1;
	unsigned char avdd = (flag >> 1) & 0x1;
	unsigned char lane = 2 - ((flag >> 2) & 0x1);
	unsigned char clk_edge_sel = (flag >> 3) & 0x1;

	hwp_cam->CAM_CSI_REG_0 = 0xA0000000 | (frame_line_l << 8) | d_term_en;
	hwp_cam->CAM_CSI_REG_1 = 0x00020000 | d_hs_setl;
	hwp_cam->CAM_CSI_REG_2 = (c_term_en << 16) | c_hs_setl;
	hwp_cam->CAM_CSI_REG_3 = 0x1E0A0000 | (lane << 30) | (clk_edge_sel << 29)
			| (ch_sel << 20) | (avdd << 11);
	hwp_cam->CAM_CSI_REG_4 = 0x0;
	hwp_cam->CAM_CSI_REG_5 = 0x40dc4200 | frame_line_h;
	hwp_cam->CAM_CSI_REG_6 = 0x800420ea;
	hwp_cam->CAM_CSI_ENABLE = 1;
}

/* -----------------------------------------------------------------
 * Private functions
 * -----------------------------------------------------------------*/
static void start_camera(struct rda_camera_dev *rcam)
{
	HWP_CAMERA_T *hwp_cam = (HWP_CAMERA_T*)rcam->regs;

	pm_ddr_get(PM_DDR_CAMERA_DMA);

	/* enable Camera controller & AXI */
	hwp_cam->CAM_AXI_CONFIG |= CAMERA_AXI_START;
#ifdef _TGT_AP_CAM_ISP_ENABLE
	hwp_cam->CTRL |= (CAMERA_ISP_ENABLE | CAMERA_LINEBUF_ISP);
#endif
	hwp_cam->CTRL |= CAMERA_ENABLE;

//	rda_dbg_camera("%s: begin.\n", __func__);
}

static void stop_camera(struct rda_camera_dev *rcam)
{
	HWP_CAMERA_T *hwp_cam = (HWP_CAMERA_T*)rcam->regs;

	hwp_cam->CTRL &= ~CAMERA_ENABLE;
#ifdef _TGT_AP_CAM_ISP_ENABLE
	hwp_cam->CTRL &= ~(CAMERA_ISP_ENABLE | CAMERA_LINEBUF_ISP);
#endif
	/* disable Camera controller & AXI */
	hwp_cam->CAM_AXI_CONFIG |= CAMERA_AXI_SWDIS;

	pm_ddr_put(PM_DDR_CAMERA_DMA);

//	rda_dbg_camera("%s: begin.\n", __func__);
}

#ifdef _TGT_AP_CAM_ISP_ENABLE
static void init_isp(struct rda_camera_dev *rcam)
{
	ISP_CAMERA_T *cam_isp = (ISP_CAMERA_T*)(rcam->regs + REG_ISP_OFFS);
	struct  isp_reg_list *isp_table;

	rda_dbg_camera("%s: begin.\n", __func__);
        //*****************************
        // init_general
        //*****************************
	cam_isp->ISP_SOFT_RST = 0x00;
	cam_isp->ISP_SOFT_RST = 0x01; //rst isp_reg

	cam_isp->ISP_PAGE = 0x00; //select page 0

	cam_isp->ISP_CTRL0 = 0x00; //for YUV mode  00->10 vsync_toggle
	cam_isp->ISP_CTRL1 = 0x80; //exp_chg_dont_care=1
	cam_isp->ISP_CTRL2 = 0x0f;

	cam_isp->TOP_DUMMY_REG0 = 0x04;
	cam_isp->LEFT_DUMMY_REG0 = 0x00;
	cam_isp->V_DUMMY_REG0 = 0x00; //v_dummy

	cam_isp->LINE_NUM_1_REG0 = 0xb0; //test 1280*960 capture 320*240
	cam_isp->PIX_NUM_1_REG0 = 0x20;  // 1280=0x500 960=0x3c0  320=0x140 24
	cam_isp->LINE_PIX_H_REG0 = 0x34;
	printk(KERN_ERR "%s: init nor to 1600*1200!\n",__func__);

	cam_isp->TOP_DUMMY_REG1 = 0x3c;//top_dummy
	cam_isp->LEFT_DUMMY_REG1 = 0x50; //left_dummy
	cam_isp->V_DUMMY_REG1 = 0x00; //v_dummy to enlarge gap of VSYNC f0->00

	cam_isp->LINE_NUM_1_REG1 = 0xe0;
	cam_isp->PIX_NUM_1_REG1  = 0x80;
	cam_isp->LINE_PIX_H_REG1 = 0x21;
	printk(KERN_ERR "%s: init sub to 640*480!\n",__func__);

//	cam_isp->ISP_SUB_MODE = 0x02;   //start with nor mode
//	printk(KERN_ERR "%s: init to sub mode!\n",__func__);
        //*****************************
        // init_bayer_top
        //*****************************
	cam_isp->PAGE_X.ISP_PAGE0.BLC_CTRL         = 0x00; //close BLC
	cam_isp->PAGE_X.ISP_PAGE0.DRC_CLAMP_CTRL   = 0x83; //DRC
	cam_isp->PAGE_X.ISP_PAGE0.LSC_CTRL         = 0x83; //enable LSC
	cam_isp->PAGE_X.ISP_PAGE0.LSC_P2_UP_R		   = 0x62;
	cam_isp->PAGE_X.ISP_PAGE0.LSC_P2_UP_G		   = 0x58;
	cam_isp->PAGE_X.ISP_PAGE0.LSC_P2_UP_B		   = 0x56;
	cam_isp->PAGE_X.ISP_PAGE0.LSC_P2_DOWN_R	   = 0x62;
	cam_isp->PAGE_X.ISP_PAGE0.LSC_P2_DOWN_G	   = 0x58;
	cam_isp->PAGE_X.ISP_PAGE0.LSC_P2_DOWN_B	   = 0x58;
	cam_isp->PAGE_X.ISP_PAGE0.LSC_P2_LEFT_R	   = 0x60;
	cam_isp->PAGE_X.ISP_PAGE0.LSC_P2_LEFT_G	   = 0x58;
	cam_isp->PAGE_X.ISP_PAGE0.LSC_P2_LEFT_B	   = 0x54;
	cam_isp->PAGE_X.ISP_PAGE0.LSC_P2_RIGHT_R	 = 0x62;
	cam_isp->PAGE_X.ISP_PAGE0.LSC_P2_RIGHT_G	 = 0x58;
	cam_isp->PAGE_X.ISP_PAGE0.LSC_P2_RIGHT_B	 = 0x58;
	cam_isp->PAGE_X.ISP_PAGE0.LSC_P4_Q1			   = 0x30;
	cam_isp->PAGE_X.ISP_PAGE0.LSC_P4_Q2			   = 0x30;
	cam_isp->PAGE_X.ISP_PAGE0.LSC_P4_Q3			   = 0x30;
	cam_isp->PAGE_X.ISP_PAGE0.LSC_P4_Q4			   = 0x30;


	//*****************************
	// init_awb&cc
	//*****************************
	cam_isp->ISP_PAGE = 0x01; //select page 1
	cam_isp->PAGE_X.ISP_PAGE1.EXP_CTRL         = 0x10; //awb_vld_thr={exp_ctrl_reg[7:0], 3'd7};
	cam_isp->ISP_PAGE = 0x00; //select page 0
	cam_isp->R_AWB_GAIN_IN_REG                 = 0x44;
	cam_isp->G_AWB_GAIN_IN_REG                 = 0x40;
	cam_isp->B_AWB_GAIN_IN_REG                 = 0x44;
	cam_isp->AWB_GAIN_L_REG                    = 0x38;
	cam_isp->AWB_GAIN_H_REG                    = 0x78;
	cam_isp->AWB_CTRL_REG                      = 0xa1; //awb_ofst=1
	cam_isp->AWB_STOP_REG                      = 0x3c; //0:1.5x 1:2x 2:3x 3:4x
	cam_isp->AWB_ALGO_REG                      = 0x78; //smaller will be red sometime, bigger will be blue
        //*****************************
        // init_Bayer_gamma
        //*****************************
	//cam_isp->PAGE_X.ISP_PAGE0.GAMMA_CTRL       = 0x83; //rgb_gamma_en=1 bayer_gamma_en=0
	cam_isp->PAGE_X.ISP_PAGE0.GAMMA_CTRL       = 0x0f; //rgb_gamma_en=0 bayer_gamma_en=1
	cam_isp->PAGE_X.ISP_PAGE0.BAYER_GAMMA_B0   = 0x02;
	cam_isp->PAGE_X.ISP_PAGE0.BAYER_GAMMA_B1   = 0x14;
	cam_isp->PAGE_X.ISP_PAGE0.BAYER_GAMMA_B2   = 0x26;
	cam_isp->PAGE_X.ISP_PAGE0.BAYER_GAMMA_B3   = 0x37;
	cam_isp->PAGE_X.ISP_PAGE0.BAYER_GAMMA_B4   = 0x47;
	cam_isp->PAGE_X.ISP_PAGE0.BAYER_GAMMA_B5   = 0x58;
	cam_isp->PAGE_X.ISP_PAGE0.BAYER_GAMMA_B6   = 0x67;
	cam_isp->PAGE_X.ISP_PAGE0.BAYER_GAMMA_B8   = 0x82;
	cam_isp->PAGE_X.ISP_PAGE0.BAYER_GAMMA_B10  = 0x98;
	cam_isp->PAGE_X.ISP_PAGE0.BAYER_GAMMA_B12  = 0xae;
	cam_isp->PAGE_X.ISP_PAGE0.BAYER_GAMMA_B14  = 0xbf;
	cam_isp->PAGE_X.ISP_PAGE0.BAYER_GAMMA_B16  = 0xcd;
	cam_isp->PAGE_X.ISP_PAGE0.BAYER_GAMMA_B18  = 0xd8;
	cam_isp->PAGE_X.ISP_PAGE0.BAYER_GAMMA_B20  = 0xe2;
	cam_isp->PAGE_X.ISP_PAGE0.BAYER_GAMMA_B24  = 0xef;
	cam_isp->PAGE_X.ISP_PAGE0.BAYER_GAMMA_B28  = 0xf9;
	cam_isp->PAGE_X.ISP_PAGE0.BAYER_GAMMA_B32  = 0xff;
        //*****************************
        // init_YNR&CNR
        //*****************************
	cam_isp->ISP_PAGE = 0x01; //select page 1
	cam_isp->PAGE_X.ISP_PAGE1.YCNR_CTRL             = 0x04; //nr_1nd_on      = 1
	cam_isp->PAGE_X.ISP_PAGE1.YNR_INT_CTRL          = 0x80; //ynr_int_en_reg = 0 & not_adp_med
	cam_isp->PAGE_X.ISP_PAGE1.YNR_LL_AREA_THR			  = 0x30;
	cam_isp->PAGE_X.ISP_PAGE1.YNR_LL_HF_STR         = 0x20;
	cam_isp->PAGE_X.ISP_PAGE1.YNR_LL_LF_STR         = 0x70;
	cam_isp->PAGE_X.ISP_PAGE1.YNR_LL_LF_METHOD_STR  = 0x80;
	cam_isp->PAGE_X.ISP_PAGE1.YNR_HL_AREA_THR		    = 0x10;
	cam_isp->PAGE_X.ISP_PAGE1.YNR_HL_HF_STR         = 0x10;
	cam_isp->PAGE_X.ISP_PAGE1.YNR_HL_LF_STR         = 0x50;
	cam_isp->PAGE_X.ISP_PAGE1.YNR_HL_LF_METHOD_STR  = 0x80;
	cam_isp->PAGE_X.ISP_PAGE1.CNR_1D_CTRL           = 0x05; //gaus cnr
	cam_isp->PAGE_X.ISP_PAGE1.CNR_1D_STR            = 0xc0; //cnr_1d_str

	//*****************************
	// init_EE
	//*****************************

	//*****************************
	// ycnr+yee
	//*****************************
	cam_isp->PAGE_X.ISP_PAGE1.EE_CTRL				   = 0x07; //sel cnr_1d_in, ee_on=1
        cam_isp->PAGE_X.ISP_PAGE1.EE_INT_CTRL      = 0x00; //ee_int_en_reg  = 0
        cam_isp->PAGE_X.ISP_PAGE1.EE_HL_THR			   = 0xb8;
        cam_isp->PAGE_X.ISP_PAGE1.EE_LL_THR			   = 0x28;
	cam_isp->PAGE_X.ISP_PAGE1.EE_EDGE_GAIN		 = 0x40; //gain
	cam_isp->PAGE_X.ISP_PAGE1.EE_H_LF_STR		   = 0x80;
	cam_isp->PAGE_X.ISP_PAGE1.EE_H_AREA_THR	   = 0x20;
	cam_isp->PAGE_X.ISP_PAGE1.EE_L_LF_STR	     = 0x40;
	cam_isp->PAGE_X.ISP_PAGE1.EE_L_AREA_THR    = 0x10;
	cam_isp->PAGE_X.ISP_PAGE1.EE_H_MINUS_STR	 = 0x28;
	cam_isp->PAGE_X.ISP_PAGE1.EE_H_PLUS_STR	   = 0x28;
	cam_isp->PAGE_X.ISP_PAGE1.EE_M_MINUS_STR	 = 0x28;
	cam_isp->PAGE_X.ISP_PAGE1.EE_M_PLUS_STR	   = 0x20;
	cam_isp->PAGE_X.ISP_PAGE1.EE_L_MINUS_STR	 = 0x08;
	cam_isp->PAGE_X.ISP_PAGE1.EE_L_PLUS_STR    = 0x08;
	cam_isp->PAGE_X.ISP_PAGE1.EE_B11           = 0x50;
	cam_isp->PAGE_X.ISP_PAGE1.EE_B12           = 0x48;
	cam_isp->PAGE_X.ISP_PAGE1.EE_B13           = 0x40;
	cam_isp->PAGE_X.ISP_PAGE1.EE_B14           = 0x38;
	cam_isp->PAGE_X.ISP_PAGE1.EE_B15           = 0x30;
	cam_isp->PAGE_X.ISP_PAGE1.EE_B16           = 0x28;
	cam_isp->ISP_PAGE = 0x00; //select page 0
	//*****************************
	// init_cc
	//****************************
	cam_isp->PAGE_X.ISP_PAGE0.CC_R_OFFSET      = 0x00;//0x0a; //0x06
	cam_isp->PAGE_X.ISP_PAGE0.CC_G_OFFSET      = 0x00;//0x08; //0x06
	cam_isp->PAGE_X.ISP_PAGE0.CC_B_OFFSET      = 0x00;//0x86; //0x06
	cam_isp->PAGE_X.ISP_PAGE0.CC_00            = 0x58;//0x7c; //0x50
	cam_isp->PAGE_X.ISP_PAGE0.CC_01            = 0x90;//0x8f; //0x9d
	cam_isp->PAGE_X.ISP_PAGE0.CC_02            = 0x88;//0xad; //0x0d
	cam_isp->PAGE_X.ISP_PAGE0.CC_10            = 0x88;//0xa3; //0x89
	cam_isp->PAGE_X.ISP_PAGE0.CC_11            = 0x50;//0x69; //0x4b
	cam_isp->PAGE_X.ISP_PAGE0.CC_12            = 0x88;//0x86; //0x82
	cam_isp->PAGE_X.ISP_PAGE0.CC_20            = 0x88;//0xa6; //0x84
	cam_isp->PAGE_X.ISP_PAGE0.CC_21            = 0x90;//0x9a; //0xa4
	cam_isp->PAGE_X.ISP_PAGE0.CC_22            = 0x58;//0x7f; //0x68


	//*****************************
        // init_y_gamma
        //*****************************
        cam_isp->SCG_REG                           = 0xa0;
	//*****************************
	// init_rgb_top
	//*****************************
	cam_isp->PAGE_X.ISP_PAGE0.INTP_CTRL        = 0x76;
	cam_isp->PAGE_X.ISP_PAGE0.DPC_CTRL0        = 0xc3; //cc_on=bnr_on=dpc_en=1 not use adp_med
	cam_isp->PAGE_X.ISP_PAGE0.DPC_CTRL1        = 0x1d; //check_round_en=0
	cam_isp->PAGE_X.ISP_PAGE0.DPC_AU1          = 0x00; //not check int_flg7
	cam_isp->PAGE_X.ISP_PAGE0.DPC_Y_THR_DATA   = 0x28; //dead pxiel thr
	cam_isp->PAGE_X.ISP_PAGE0.DPC_INT_THR      = 0x10; //diff of max and min
	cam_isp->PAGE_X.ISP_PAGE0.DPC_NR_LF_STR    = 0x10;
	cam_isp->PAGE_X.ISP_PAGE0.DPC_NR_HF_STR    = 0x10;
	cam_isp->PAGE_X.ISP_PAGE0.DPC_NR_AREA_THR  = 0x10;


	//*****************************
	// init_ae
	//*****************************
	cam_isp->ISP_PAGE = 0x01; //select page 1
	cam_isp->PAGE_X.ISP_PAGE1.AE_CTRL_5        = 0x10; //ae_ext_adj_on_reg=1

	cam_isp->PAGE_X.ISP_PAGE1.EXP_CHG_1        = 0x01; //exp_floor={exp_chg_1_reg[4:0],2'd0};
        cam_isp->PAGE_X.ISP_PAGE1.HIST_BP_LEVEL    = 0xC0; //hist_bp_level=0xc0
        cam_isp->PAGE_X.ISP_PAGE1.CONST_REG        = 0x98; //inc contrast
        cam_isp->PAGE_X.ISP_PAGE1.SATUR_REG        = 0xB0;
	cam_isp->PAGE_X.ISP_PAGE1.CONST_SATUR_OFF  = 0x00;

	cam_isp->ISP_PAGE = 0x00; //select page 0
	cam_isp->Y_AVE_TARGET_REG                  = rcam->raw_sensor->targetBV; //Yave_target
	cam_isp->AE_CTRL_REG                       = 0x01; //full image AE
	cam_isp->PAGE_X.ISP_PAGE0.AE_CTRL2         = 0x00; //int_thr_sel = nr_lf_str_sel = 0
	cam_isp->AE_ANA_GAIN_H_L_REG               = 0xe4; //yave_sel=0
	cam_isp->AWB_DRC_REG                       = 0x01; //dig_en=0, dig_gain_ofst=1
	cam_isp->PAGE_X.ISP_PAGE0.Y_CTRL_MISC      = 0xF2; //awb_adj=1
	cam_isp->PAGE_X.ISP_PAGE0.AE_DRC_GAIN_IN   = 0x40; //dig_gain_init
	cam_isp->PAGE_X.ISP_PAGE0.LUMA_OFFSET      = 0x00;

	/*
		hwp_cam->DCT_SHIFTR_Y_0 = 0x648;//1600->1616
		hwp_cam->DCT_SHIFTR_Y_1 = 0x4b8;//1200->1216
	*/

	// sensor specific initialization parameters
	isp_table = rcam->raw_sensor->isp_sensor_init;
	if (isp_table)
		rcam_isp_reg_write_array(isp_table);
}


extern struct  raw_sensor_info_data *rda_sensor_get_raw_sensor_info(struct v4l2_subdev *sd);
static void config_isp_geometry(struct rda_camera_dev *rcam, u32 w, u32 h)
{
	HWP_CAMERA_T *hwp_cam = (HWP_CAMERA_T*)rcam->regs;
	ISP_CAMERA_T *cam_isp = (ISP_CAMERA_T*)(rcam->regs + REG_ISP_OFFS);

        struct v4l2_subdev *sd = soc_camera_to_subdev(rcam->icd);

	u16 sensor_line_number;
	u16 sensor_frame_width, sensor_frame_height;
        int normal_mode = 0;
	rda_dbg_camera("%s: begin.\n", __func__);

	rcam->raw_sensor= rda_sensor_get_raw_sensor_info(sd);

        if (!rcam->raw_sensor) {

		rda_dbg_camera("%s: not raw sensor, return.\n", __func__);
               return;
	}

	sensor_line_number = rcam->raw_sensor->frame_line_num;
        sensor_frame_width =  rcam->raw_sensor->frame_width;
        sensor_frame_height =  rcam->raw_sensor->frame_height;

	if (!sensor_frame_width)
		sensor_frame_width = 1600;
        if (!sensor_frame_height)
		sensor_frame_height = 1200;

	hwp_cam->DCT_SHIFTR_Y_0 = sensor_frame_width; //1600->1616
	hwp_cam->DCT_SHIFTR_Y_1 = sensor_frame_height;//1200->1216


	if ((sensor_line_number > 900 )&& (h < 600)) {// >=1.3M pixel && in VGA or small, sub mod
		hwp_cam->CTRL &= ~CAMERA_DATAFORMAT_RESERVE;
		hwp_cam->CTRL |= CAMERA_DATAFORMAT_YUV422; //sub mode
		hwp_cam->CTRL &= ~CAMERA_REORDER(2) ;
		printk(KERN_ERR "%s: DATAFORMAT_YUV422!\n",__func__);

		cam_isp->ISP_SUB_MODE = 0x02;
		printk(KERN_ERR "%s: set to sub mode!\n",__func__);
		normal_mode =0 ;
	}
	else {// others all in normal mode
		hwp_cam->CTRL &= ~CAMERA_DATAFORMAT_YUV422;
		hwp_cam->CTRL |= CAMERA_DATAFORMAT_RESERVE;   //normal mode
		hwp_cam->CTRL |= CAMERA_REORDER(2) ;
		printk(KERN_ERR "%s: DATAFORMAT_NORMAL!\n",__func__);

		cam_isp->ISP_SUB_MODE = 0x00;
		printk(KERN_ERR "%s: set to normal mode!\n",__func__);

		normal_mode =1 ;
	}

	rda_dbg_camera("%s: width=%d height=%d\n", __func__, w, h);


	cam_isp->ISP_PAGE = 0x00; //select page 0

	if (normal_mode ==1) {
		cam_isp->LINE_NUM_1_REG0 = h & 0xff ;
		cam_isp->PIX_NUM_1_REG0 = (w>>1)&0xff;
		cam_isp->LINE_PIX_H_REG0 =(((w>>1)&0xf00)>>4)|((h &0xf00)>>8);

		cam_isp->ISP_SUB_MODE = 0x00;
		printk(KERN_ERR "set normal to %x * %x !\n", w , h);
	}

	else{

		cam_isp->LINE_NUM_1_REG1 = h & 0xff ;
		cam_isp->PIX_NUM_1_REG1 = w &0xff;
		cam_isp->LINE_PIX_H_REG1 =((w&0xf00)>>4)|((h &0xf00)>>8);

		cam_isp->ISP_SUB_MODE = 0x2;
		printk(KERN_ERR "set sub mode to %x * %x!\n", w ,h );
	}

}
#endif

static int configure_geometry(struct rda_camera_dev *rcam,
		enum v4l2_mbus_pixelcode code, u32 width, u32 height)
{
	HWP_CAMERA_T *hwp_cam = (HWP_CAMERA_T*)rcam->regs;

	switch (code) {
	case V4L2_MBUS_FMT_YUYV8_2X8:
		hwp_cam->CTRL |= CAMERA_REORDER_YUYV;
		break;
	case V4L2_MBUS_FMT_YVYU8_2X8:
		hwp_cam->CTRL |= CAMERA_REORDER_YVYU;
		break;
	case V4L2_MBUS_FMT_UYVY8_2X8:
		hwp_cam->CTRL |= CAMERA_REORDER_UYVY;
		break;
	case V4L2_MBUS_FMT_VYUY8_2X8:
		hwp_cam->CTRL |= CAMERA_REORDER_VYUY;
		break;
	default:
		rda_dbg_camera("%s: pixelcode: %x not support\n",
				__func__, code);
		return -EINVAL;
	}
	hwp_cam->CTRL |= CAMERA_DATAFORMAT_YUV422;
#ifdef _TGT_AP_CAM_ISP_ENABLE
	config_isp_geometry(rcam, width, height);
#endif
	return 0;
}

#ifdef _TGT_AP_CAM_ISP_ENABLE
extern void rda_sensor_reg_w(struct v4l2_subdev *sd, const u16 addr, const u8 data);
extern void rda_sensor_upd_gain_isp(struct v4l2_subdev *sd, int val);
extern void rda_sensor_upd_exp_isp(struct v4l2_subdev *sd, int val);
extern void AE(int cBV, int hist,  struct raw_sensor_info_data *p_raw, int first_frame );

static void adjust_exp(struct work_struct *wk)
{
        struct delayed_work *dwk = to_delayed_work(wk);
        struct rda_camera_dev *p = container_of(dwk,
        struct rda_camera_dev, isr_work);
        ISP_CAMERA_T *cam_isp = (ISP_CAMERA_T*)(p->regs + REG_ISP_OFFS);
        struct v4l2_subdev *sd = soc_camera_to_subdev(p->icd);

        static int pcounter =0, pre_gain, pre_exp;
        static int flag = 2;
        int cBV;
        bool sensor_stable =0;
        int hist;

        struct sensor_status  *sensor_state_info;

        cam_isp->ISP_PAGE = 0;

        cBV = cam_isp->PAGE_X.ISP_PAGE0.YAVE_OUT;
        hist = cam_isp->PAGE_X.ISP_PAGE0.AE_BRIGHT_HIST;
        rda_dbg_camera("%s: current BV is  %02x , weighted BV is %02x\n", __func__,cBV, cam_isp->PAGE_X.ISP_PAGE0.YWAVE_OUT);
        flag ++ ;
        if (flag ==5)
                flag =0;
//        return;

	if (!p->raw_sensor->ae_table){
		rda_dbg_camera("%s: AE table is empty\n", __func__);
		return;
	}

	sensor_state_info =&( p->raw_sensor->state_info);
        if((sensor_stable)||(flag>2)) {

                if(flag ==3) {//after adjust exposure & gain, 3 frames later to do the AE measuring.
                        AE(cBV,hist, p->raw_sensor,1);  //no adjust, since need to read out 2 frames to compare
                }
                else {
                        AE(cBV,hist, p->raw_sensor,0); //adjust

			sensor_state_info =&( p->raw_sensor->state_info);

			sensor_stable = sensor_state_info->sensor_stable;
                rda_dbg_camera("%s: gain =  %x , exp =  %x ,cBV = %x stable = %x \n", __func__,sensor_state_info->gain,sensor_state_info->exp,cBV,sensor_stable);
                        if (!sensor_stable) {
	                        rda_dbg_camera(" sensor not stable R= %x , B= %x \n",cam_isp->PAGE_X.ISP_PAGE0.R_AWB_GAIN,cam_isp->PAGE_X.ISP_PAGE0.B_AWB_GAIN);
                                if (pre_gain != sensor_state_info->gain) {
		                        rda_sensor_upd_gain_isp(sd, (int)sensor_state_info->gain);
                                        pre_gain = sensor_state_info->gain;

                                }
                                if (pre_exp!= sensor_state_info->exp) {
		                        rda_sensor_upd_exp_isp(sd, (int)sensor_state_info->exp);
                                        pre_exp = sensor_state_info->exp;
                                }

                                flag =0;

                                if(sensor_state_info->tBV_dec) {//target Brightness Value needed to be decreased
	                                     p->raw_sensor->targetBV =  p->raw_sensor->targetBV- sensor_state_info->tBV_dec; //Yave_target
                                }
                        }
                }
        }
#if 1  //for debug purpose
        pcounter++;
        if (pcounter ==10){
                pcounter =0 ;
                cam_isp->ISP_PAGE = 0;
	        rda_dbg_camera("%s: dh= %02x, bh=%02x, ywave= %02x, yave=%02x, awbd= %02x, dig=%02x,"\
                                "r= %02x, b= %02x, gain= %x, exp=%04x, cBV=%02x, targetBV=%02x \n",
				__func__,
				cam_isp->PAGE_X.ISP_PAGE0.AE_DARK_HIST,
				cam_isp->PAGE_X.ISP_PAGE0.AE_BRIGHT_HIST,
				cam_isp->PAGE_X.ISP_PAGE0.YWAVE_OUT,
				cam_isp->PAGE_X.ISP_PAGE0.YAVE_OUT,
				cam_isp->PAGE_X.ISP_PAGE0.AWB_DEBUG_OUT,
				cam_isp->PAGE_X.ISP_PAGE0.AE_DRC_GAIN,
				cam_isp->PAGE_X.ISP_PAGE0.R_AWB_GAIN,
				cam_isp->PAGE_X.ISP_PAGE0.B_AWB_GAIN,sensor_state_info->gain, sensor_state_info->exp, cBV,p->raw_sensor->targetBV);

	}
#endif
}
#endif
/*
static void handle_vsync(struct work_struct *wk)
{
	struct delayed_work *dwk = to_delayed_work(wk);
	struct rda_camera_dev *p = container_of(dwk,
			struct rda_camera_dev, isr_work);
	HWP_CAMERA_T *hwp_cam = (HWP_CAMERA_T*)p->regs;
	unsigned int tc = hwp_cam->CAM_TC_COUNT;

	if (!tc)
		schedule_delayed_work(dwk, msecs_to_jiffies(5));
	else if (p->next) {
		hwp_cam->CAM_FRAME_START_ADDR = p->next->dma_addr;
		rda_dbg_camera("%s: tc: %d, next dma_addr: 0x%x\n",
				__func__, tc, p->next->dma_addr);
	} else
		rda_dbg_camera("%s: p->next is NULL\n", __func__);
}

static void handle_vsync(unsigned long pcam)
{
	struct rda_camera_dev *p = (struct rda_camera_dev*)pcam;
	HWP_CAMERA_T *hwp_cam = (HWP_CAMERA_T*)p->regs;
	unsigned int tc = hwp_cam->CAM_TC_COUNT;

	if (!tc) {
		tasklet_schedule(&rcam_tasklet);
	} else if (p->next) {
		hwp_cam->CAM_FRAME_START_ADDR = p->next->dma_addr;
		rda_dbg_camera("%s: tc: %d, next dma_addr: 0x%x\n",
				__func__, tc, p->next->dma_addr);
	} else
		rda_dbg_camera("%s: p->next is NULL\n", __func__);
}
*/

static irqreturn_t handle_streaming(struct rda_camera_dev *rcam)
{
	struct cap_buffer *buf = rcam->active;
	struct vb2_buffer *vb;
	HWP_CAMERA_T *hwp_cam = (HWP_CAMERA_T*)rcam->regs;
	unsigned int tc = hwp_cam->CAM_TC_COUNT;

	if (!buf)
		return IRQ_HANDLED;

	if (tc)
		printk(KERN_ERR "%s: frame size=%d, tc=%d\n",
				__func__, rcam->icd->sizeimage, tc);

	list_del_init(&buf->list);
	if (list_empty(&rcam->cap_buffer_list)) {
		rcam->active = NULL;
		rcam->next = NULL;
	} else {
		rcam->active = list_entry(rcam->cap_buffer_list.next,
				struct cap_buffer, list);
		if (list_is_last(&rcam->active->list, &rcam->cap_buffer_list))
			rcam->next = NULL;
		else
			rcam->next = list_entry(rcam->active->list.next,
					struct cap_buffer, list);
	}

	vb = &buf->vb;
	do_gettimeofday(&vb->v4l2_buf.timestamp);
	vb->v4l2_buf.sequence = rcam->sequence++;
	vb2_buffer_done(vb, VB2_BUF_STATE_DONE);

	return IRQ_HANDLED;
}

static irqreturn_t rda_camera_isr(int irq, void *dev)
{
	struct rda_camera_dev *rcam = dev;
	HWP_CAMERA_T *hwp_cam = (HWP_CAMERA_T*)rcam->regs;
	unsigned int irq_cause = 0;
	unsigned int state = 0;
	unsigned int addr = 0;
	irqreturn_t ret = IRQ_NONE;
	unsigned long flags = 0;

	spin_lock_irqsave(&rcam->lock, flags);
	irq_cause = hwp_cam->IRQ_CAUSE;
	hwp_cam->IRQ_CLEAR |= irq_cause;
	state = rcam->state;

//	rda_dbg_camera("%s: begin, state is %d.\n", __func__, state);

	if (irq_cause & IRQ_VSYNC_R) {
		if (state == CAM_STATE_IDLE) {
			wake_up_interruptible(&rcam->vsync_wq);
			/* start from oneshot mode */
			rcam->state = CAM_STATE_ONESHOT;
		} else {
			/* Disable IRQ_VSYNC_R */
			hwp_cam->IRQ_MASK &= ~IRQ_VSYNC_R;
		}
		ret = IRQ_HANDLED;
	} else if (irq_cause & IRQ_OVFL) {
		printk(KERN_ERR "%s: overflow!\n", __func__);
#ifdef _TGT_AP_CAM_ISP_ENABLE
//	if sensor is  YUV_SENSOR, do nothing
	if( rcam->raw_sensor)
		hwp_cam->CTRL &= ~(CAMERA_ISP_ENABLE | CAMERA_LINEBUF_ISP);
#endif
	hwp_cam->CTRL &= ~CAMERA_ENABLE;

	ret = IRQ_HANDLED;
	} else if (irq_cause & IRQ_DMADONE) {
		if (!rcam->active) {
			ret = IRQ_HANDLED;
		} else if (state == CAM_STATE_ONESHOT) {
			addr = rcam->active->dma_addr;
			if (hwp_cam->CTRL & CAMERA_ENABLE)
				stop_camera(rcam);
			ret = handle_streaming(rcam);
			if (rcam->next) {
				/* change to success mode */
				rcam->state = CAM_STATE_SUCCESS;
				hwp_cam->CAM_AXI_CONFIG &= ~CAMERA_AXI_AUTO_DIS;
				hwp_cam->CAM_AXI_CONFIG |= CAMERA_AXI_PINGPONG;
				/* config ping-pong dma address */
				hwp_cam->CAM_FRAME_START_ADDR =
					rcam->active->dma_addr;
				hwp_cam->CAM_FRAME2_START_ADDR =
					rcam->next->dma_addr;
				start_camera(rcam);
			} else if (rcam->active) {
				/* update dma address */
				hwp_cam->CAM_FRAME_START_ADDR =
					rcam->active->dma_addr;
				start_camera(rcam);
			}
		} else if (state == CAM_STATE_SUCCESS) {
			addr = rcam->active->dma_addr;
			ret = handle_streaming(rcam);
			if (rcam->next) {
				/* update dma address for next buffer */
				if (addr == hwp_cam->CAM_FRAME_START_ADDR) {
					hwp_cam->CAM_FRAME_START_ADDR =
						rcam->next->dma_addr;
				} else {
					hwp_cam->CAM_FRAME2_START_ADDR =
						rcam->next->dma_addr;
				}
			} else if (rcam->active) {
				/* change to oneshot mode */
				rcam->state = CAM_STATE_ONESHOT;
				hwp_cam->CAM_AXI_CONFIG |= CAMERA_AXI_AUTO_DIS;
				hwp_cam->CAM_AXI_CONFIG &= ~CAMERA_AXI_PINGPONG;
				/* set same dma address for safety reason */
				hwp_cam->CAM_FRAME_START_ADDR =
					rcam->active->dma_addr;
				hwp_cam->CAM_FRAME2_START_ADDR =
					rcam->active->dma_addr;
			} else {
				/* change to oneshot mode & stop dma */
				rcam->state = CAM_STATE_ONESHOT;
				stop_camera(rcam);
			}
		}
#ifdef _TGT_AP_CAM_ISP_ENABLE
		if( rcam->raw_sensor)
			schedule_delayed_work(&rcam->isr_work, 0);
#endif
	}

	spin_unlock_irqrestore(&rcam->lock, flags);

	//rda_dbg_camera("%s: cause: %x, addr: 0x%x, state: %d, new state %d\n",
	//		__func__, irq_cause, addr, state, rcam->state);
	return ret;
}

/* -----------------------------------------------------------------
 * Videobuf operations
 * -----------------------------------------------------------------*/
static int queue_setup(struct vb2_queue *vq, const struct v4l2_format *fmt,
		unsigned int *nbuffers, unsigned int* nplanes,
		unsigned int sizes[], void *alloc_ctxs[])
{
	struct soc_camera_device *icd = soc_camera_from_vb2q(vq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct rda_camera_dev *rcam = ici->priv;
	unsigned int size;

	/* May need reset camera host */
	/* TODO: do hardware reset here */
	size = icd->sizeimage;

	if (!*nbuffers || *nbuffers > MAX_BUFFER_NUM)
		*nbuffers = MAX_BUFFER_NUM;

	if (size * *nbuffers > VID_LIMIT_BYTES)
		*nbuffers = VID_LIMIT_BYTES / size;

	*nplanes = 1;
	sizes[0] = size;
	alloc_ctxs[0] = rcam->alloc_ctx;

	rcam->sequence = 0;
	rcam->active = NULL;
	rcam->next = NULL;
	rda_dbg_camera("%s: count=%d, size=%d\n", __func__, *nbuffers, size);

	return 0;
}

static int buffer_init(struct vb2_buffer *vb)
{
	struct cap_buffer *buf = container_of(vb, struct cap_buffer, vb);

	INIT_LIST_HEAD(&buf->list);

	return 0;
}

static int buffer_prepare(struct vb2_buffer *vb)
{
	struct soc_camera_device *icd = soc_camera_from_vb2q(vb->vb2_queue);
	struct cap_buffer *buf = container_of(vb, struct cap_buffer, vb);
	unsigned long size;

	size = icd->sizeimage;

	if (vb2_plane_size(vb, 0) < size) {
		rda_dbg_camera("%s: data will not fit into plane(%lu < %lu)\n",
				__func__, vb2_plane_size(vb, 0), size);
		return -EINVAL;
	}

	vb2_set_plane_payload(&buf->vb, 0, size);

	return 0;
}

static void buffer_cleanup(struct vb2_buffer *vb)
{
}

static void buffer_queue(struct vb2_buffer *vb)
{
	struct soc_camera_device *icd = soc_camera_from_vb2q(vb->vb2_queue);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct rda_camera_dev *rcam = ici->priv;
	HWP_CAMERA_T *hwp_cam = (HWP_CAMERA_T*)rcam->regs;
	struct cap_buffer *buf = container_of(vb, struct cap_buffer, vb);
	unsigned long flags = 0;

	buf->dma_addr = vb2_dma_contig_plane_dma_addr(vb, 0);
	spin_lock_irqsave(&rcam->lock, flags);
	list_add_tail(&buf->list, &rcam->cap_buffer_list);


//	rda_dbg_camera("%s: enter buffer_queue, buf->dma_addr: 0x%x\n",
//				__func__, buf->dma_addr);

	if (rcam->active == NULL) {
		rcam->active = buf;
//		rda_dbg_camera("%s: rcam->active->dma_addr: 0x%x\n",
//				__func__, buf->dma_addr);
		if (vb2_is_streaming(vb->vb2_queue)) {
			/* update dma address */
			hwp_cam->CAM_FRAME_START_ADDR = buf->dma_addr;
			/* start from oneshot mode */
			hwp_cam->CAM_AXI_CONFIG |= CAMERA_AXI_AUTO_DIS;
			hwp_cam->CAM_AXI_CONFIG &= ~CAMERA_AXI_PINGPONG;
			start_camera(rcam);
		}
	} else if (rcam->next == NULL) {
		rcam->next = buf;
//		rda_dbg_camera("%s: rcam->next->dma_addr: 0x%x\n",
//				__func__, buf->dma_addr);
	}
	spin_unlock_irqrestore(&rcam->lock, flags);
}

static int start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct soc_camera_device *icd = soc_camera_from_vb2q(vq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct rda_camera_dev *rcam = ici->priv;
	HWP_CAMERA_T *hwp_cam = (HWP_CAMERA_T*)rcam->regs;
	struct cap_buffer *buf, *node;
	unsigned long flags = 0;
	int ret;


	rda_dbg_camera("%s: begin.\n", __func__);
#ifdef _TGT_AP_CAM_ISP_ENABLE
        if(!ISP_INIT){
		if( rcam->raw_sensor){
			rda_dbg_camera("%s: rcam->raw_sensor targetBV is %02x\n", __func__, rcam->raw_sensor->targetBV);
			init_isp(rcam);
		}
		ISP_INIT = 1;
	}
#endif

	spin_lock_irqsave(&rcam->lock, flags);
	rcam->state = CAM_STATE_IDLE;
	hwp_cam->IRQ_CLEAR |= IRQ_MASKALL;
	hwp_cam->IRQ_MASK |= IRQ_VSYNC_R | IRQ_DMADONE | IRQ_OVFL;
	/* enable camera before wait vsync */
	if (count) {
		/* config address & size for frame */
		hwp_cam->CAM_FRAME_SIZE = rcam->icd->sizeimage;
		hwp_cam->CAM_FRAME_START_ADDR = rcam->active->dma_addr;
		/* config AXI burst length */
		hwp_cam->CAM_AXI_CONFIG = CAMERA_AXI_BURST(0xf);
		/* start from oneshot mode */
		hwp_cam->CAM_AXI_CONFIG |= CAMERA_AXI_AUTO_DIS;
		hwp_cam->CAM_AXI_CONFIG &= ~CAMERA_AXI_PINGPONG;
		start_camera(rcam);
	}
	spin_unlock_irqrestore(&rcam->lock, flags);

	rda_dbg_camera("%s: Waiting for VSYNC\n", __func__);
	ret = wait_event_interruptible_timeout(rcam->vsync_wq,
			rcam->state != CAM_STATE_IDLE,
			msecs_to_jiffies(1500));
	if (ret == 0) {
		rda_dbg_camera("%s: timeout\n", __func__);
		ret = -ETIMEDOUT;
		goto err;
	} else if (ret == -ERESTARTSYS) {
		rda_dbg_camera("%s: Interrupted by a signal\n", __func__);
		goto err;
	}

	rda_dbg_camera("%s: VSYNC arrived, start streaming returned ok \n", __func__);
	return 0;
err:
	/* Clear & Disable interrupt */
	hwp_cam->IRQ_CLEAR |= IRQ_MASKALL;
	hwp_cam->IRQ_MASK &= ~IRQ_MASKALL;
	stop_camera(rcam);
	rcam->active = NULL;
	rcam->next = NULL;
	list_for_each_entry_safe(buf, node, &rcam->cap_buffer_list, list) {
		list_del_init(&buf->list);
		vb2_buffer_done(&buf->vb, VB2_BUF_STATE_ERROR);
	}
	INIT_LIST_HEAD(&rcam->cap_buffer_list);
	return ret;
}

static int stop_streaming(struct vb2_queue *vq)
{
	struct soc_camera_device *icd = soc_camera_from_vb2q(vq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct rda_camera_dev *rcam = ici->priv;
	HWP_CAMERA_T *hwp_cam = (HWP_CAMERA_T*)rcam->regs;
	struct cap_buffer *buf, *node;
	unsigned long flags = 0;

	rda_dbg_camera("%s: begin.\n", __func__);

#ifdef _TGT_AP_CAM_ISP_ENABLE
	cancel_delayed_work(&rcam->isr_work);
	flush_scheduled_work();
#endif
	spin_lock_irqsave(&rcam->lock, flags);
	/* Clear & Disable interrupt */
	hwp_cam->IRQ_CLEAR |= IRQ_MASKALL;
	hwp_cam->IRQ_MASK &= ~IRQ_MASKALL;
	stop_camera(rcam);

/*
#ifdef _TGT_AP_CAM_ISP_ENABLE
	hwp_sysCtrlAp->GCG_Rst_Set = (0x1 << 2);
	hwp_sysCtrlAp->GCG_Rst_Clr = (0x1 << 2);
#endif
*/

	/* Release all active buffers */
	rcam->active = NULL;
	rcam->next = NULL;
	list_for_each_entry_safe(buf, node, &rcam->cap_buffer_list, list) {
		list_del_init(&buf->list);
		vb2_buffer_done(&buf->vb, VB2_BUF_STATE_ERROR);
	}
	INIT_LIST_HEAD(&rcam->cap_buffer_list);
	spin_unlock_irqrestore(&rcam->lock, flags);

	return 0;
}

static struct vb2_ops rda_video_qops = {
	.queue_setup = queue_setup,
	.buf_init = buffer_init,
	.buf_prepare = buffer_prepare,
	.buf_cleanup = buffer_cleanup,
	.buf_queue = buffer_queue,
	.start_streaming = start_streaming,
	.stop_streaming = stop_streaming,
	.wait_prepare = soc_camera_unlock,
	.wait_finish = soc_camera_lock,
};

/* -----------------------------------------------------------------
 * SoC camera operation for the device
 * -----------------------------------------------------------------*/
static int rda_camera_init_videobuf(struct vb2_queue *q,
		struct soc_camera_device *icd)
{
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_USERPTR;
	q->drv_priv = icd;
	q->buf_struct_size = sizeof(struct cap_buffer);
	q->ops = &rda_video_qops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->timestamp_type = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;

	return vb2_queue_init(q);
}

static int rda_camera_try_fmt(struct soc_camera_device *icd,
			      struct v4l2_format *f)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	const struct soc_camera_format_xlate *xlate;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_mbus_framefmt mf;
	u32 pixfmt = pix->pixelformat;
	int ret;

	xlate = soc_camera_xlate_by_fourcc(icd, pixfmt);
	if (pixfmt && !xlate) {
		rda_dbg_camera("%s: Format %x not found\n", __func__, pixfmt);
		return -EINVAL;
	}

	/* limit to Atmel ISI hardware capabilities */
	if (pix->height > MAX_SUPPORT_HEIGHT)
		pix->height = MAX_SUPPORT_HEIGHT;
	if (pix->width > MAX_SUPPORT_WIDTH)
		pix->width = MAX_SUPPORT_WIDTH;

	/* limit to sensor capabilities */
	mf.width = pix->width;
	mf.height = pix->height;
	mf.field = pix->field;
	mf.colorspace = pix->colorspace;
	mf.code = xlate->code;

	ret = v4l2_subdev_call(sd, video, try_mbus_fmt, &mf);
	if (ret < 0)
		return ret;

	pix->width = mf.width;
	pix->height = mf.height;
	pix->colorspace = mf.colorspace;

	switch (mf.field) {
	case V4L2_FIELD_ANY:
		pix->field = V4L2_FIELD_NONE;
		break;
	case V4L2_FIELD_NONE:
		break;
	default:
		rda_dbg_camera("%s: Field type %d unsupported.\n",
				__func__, mf.field);
		ret = -EINVAL;
	}

	return ret;
}

static int rda_camera_set_fmt(struct soc_camera_device *icd,
		struct v4l2_format *f)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct rda_camera_dev *rcam = ici->priv;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	const struct soc_camera_format_xlate *xlate;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_mbus_framefmt mf;
	int ret;

	xlate = soc_camera_xlate_by_fourcc(icd, pix->pixelformat);
	if (!xlate) {
		rda_dbg_camera("%s: Format %x not found\n",
				__func__, pix->pixelformat);
		return -EINVAL;
	}

	rda_dbg_camera("%s: Plan to set format %dx%d\n",
			__func__, pix->width, pix->height);

	mf.width = pix->width;
	mf.height = pix->height;
	mf.field = pix->field;
	mf.colorspace = pix->colorspace;
	mf.code = xlate->code;

	ret = v4l2_subdev_call(sd, video, s_mbus_fmt, &mf);
	if (ret < 0 && ret != -ENODEV)
		return ret;

	if (mf.code != xlate->code)
		return -EINVAL;

	ret = configure_geometry(rcam, xlate->code, mf.width, mf.height);
	if (ret < 0)
		return ret;

	pix->width = mf.width;
	pix->height = mf.height;
	pix->field = mf.field;
	pix->colorspace = mf.colorspace;
	icd->current_fmt = xlate;

	rcam->width = mf.width;
	rcam->height = mf.height;
	rda_dbg_camera("%s: Finally set format %dx%d\n",
			__func__, pix->width, pix->height);

	return ret;
}

static int rda_camera_set_bus_param(struct soc_camera_device *icd)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct rda_camera_dev *rcam = ici->priv;
	HWP_CAMERA_T *hwp_cam = (HWP_CAMERA_T*)rcam->regs;
	unsigned int ctrl = hwp_cam->CTRL;
	struct v4l2_mbus_config cfg = {.type = V4L2_MBUS_PARALLEL,};
	unsigned int common_flags = RDA_CAM_MBUS_PARA;
	int ret;

	ret = v4l2_subdev_call(sd, video, g_mbus_config, &cfg);
	if (!ret) {
		if (cfg.type == V4L2_MBUS_CSI2)
			common_flags = RDA_CAM_MBUS_CSI2;
		common_flags = soc_mbus_config_compatible(&cfg,
				common_flags);
		if (!common_flags) {
			rda_dbg_camera("%s: Flags incompatible camera 0x%x, host 0x%x\n",
					__func__, cfg.flags, common_flags);
			return -EINVAL;
		}
	} else if (ret != -ENOIOCTLCMD) {
		return ret;
	}
	rda_dbg_camera("%s: Flags cam: 0x%x common: 0x%x\n",
			__func__, cfg.flags, common_flags);

	if (cfg.type == V4L2_MBUS_PARALLEL) {
		/* Make choises, based on platform preferences */
		if ((common_flags & V4L2_MBUS_HSYNC_ACTIVE_HIGH) &&
				(common_flags & V4L2_MBUS_HSYNC_ACTIVE_LOW)) {
			if (rcam->pdata->hsync_act_low)
				common_flags &= ~V4L2_MBUS_HSYNC_ACTIVE_HIGH;
			else
				common_flags &= ~V4L2_MBUS_HSYNC_ACTIVE_LOW;
		}

		if ((common_flags & V4L2_MBUS_VSYNC_ACTIVE_HIGH) &&
				(common_flags & V4L2_MBUS_VSYNC_ACTIVE_LOW)) {
			if (rcam->pdata->vsync_act_low)
				common_flags &= ~V4L2_MBUS_VSYNC_ACTIVE_HIGH;
			else
				common_flags &= ~V4L2_MBUS_VSYNC_ACTIVE_LOW;
		}

		if ((common_flags & V4L2_MBUS_PCLK_SAMPLE_RISING) &&
				(common_flags & V4L2_MBUS_PCLK_SAMPLE_FALLING)) {
			if (rcam->pdata->pclk_act_falling)
				common_flags &= ~V4L2_MBUS_PCLK_SAMPLE_RISING;
			else
				common_flags &= ~V4L2_MBUS_PCLK_SAMPLE_FALLING;
		}
		/* set bus param for host */
		if (common_flags & V4L2_MBUS_HSYNC_ACTIVE_HIGH)
			ctrl &= ~CAMERA_HREF_POL_INVERT;
		else
			ctrl |= CAMERA_HREF_POL_INVERT;
		if (common_flags & V4L2_MBUS_VSYNC_ACTIVE_HIGH)
			ctrl &= ~CAMERA_VSYNC_POL_INVERT;
		else
			ctrl |= CAMERA_VSYNC_POL_INVERT;
		if (common_flags & V4L2_MBUS_PCLK_SAMPLE_RISING)
			ctrl &= ~CAMERA_PIXCLK_POL_INVERT;
		else
			ctrl |= CAMERA_PIXCLK_POL_INVERT;
		hwp_cam->CTRL = ctrl;
	}

	cfg.flags = common_flags;
	ret = v4l2_subdev_call(sd, video, s_mbus_config, &cfg);
	if (ret < 0 && ret != -ENOIOCTLCMD) {
		rda_dbg_camera("%s: camera s_mbus_config(0x%x) returned %d\n",
				__func__, common_flags, ret);
		return ret;
	}

	return 0;
}

static int rda_camera_try_bus_param(struct soc_camera_device *icd,
		unsigned char buswidth)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct v4l2_mbus_config cfg = {.type = V4L2_MBUS_PARALLEL,};
	unsigned int common_flags = RDA_CAM_MBUS_PARA;
	int ret;

	ret = v4l2_subdev_call(sd, video, g_mbus_config, &cfg);
	if (!ret) {
		if (cfg.type == V4L2_MBUS_CSI2)
			common_flags = RDA_CAM_MBUS_CSI2;
		common_flags = soc_mbus_config_compatible(&cfg,
				common_flags);
		if (!common_flags) {
			rda_dbg_camera("%s: Flags incompatible camera 0x%x, host 0x%x\n",
					__func__, cfg.flags, common_flags);
			return -EINVAL;
		}
	} else if (ret != -ENOIOCTLCMD) {
		return ret;
	}

	return 0;
}

/* This will be corrected as we get more formats */
static bool rda_camera_packing_supported(const struct soc_mbus_pixelfmt *fmt)
{
	return fmt->packing == SOC_MBUS_PACKING_NONE ||
		(fmt->bits_per_sample == 8 &&
		 fmt->packing == SOC_MBUS_PACKING_2X8_PADHI) ||
		(fmt->bits_per_sample > 8 &&
		 fmt->packing == SOC_MBUS_PACKING_EXTEND16);
}

static int rda_camera_get_formats(struct soc_camera_device *icd,
		unsigned int idx,
		struct soc_camera_format_xlate *xlate)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	int formats = 0, ret;
	/* sensor format */
	enum v4l2_mbus_pixelcode code;
	/* soc camera host format */
	const struct soc_mbus_pixelfmt *fmt;

	ret = v4l2_subdev_call(sd, video, enum_mbus_fmt, idx, &code);
	if (ret < 0)
		/* No more formats */
		return 0;

	fmt = soc_mbus_get_fmtdesc(code);
	if (!fmt) {
		rda_dbg_camera("%s: Invalid format code #%u: %d\n",
				__func__, idx, code);
		return 0;
	}

	/* This also checks support for the requested bits-per-sample */
	ret = rda_camera_try_bus_param(icd, fmt->bits_per_sample);
	if (ret < 0) {
		rda_dbg_camera("%s: Fail to try the bus parameters.\n",
				__func__);
		return 0;
	}

	switch (code) {
	case V4L2_MBUS_FMT_UYVY8_2X8:
	case V4L2_MBUS_FMT_VYUY8_2X8:
	case V4L2_MBUS_FMT_YUYV8_2X8:
	case V4L2_MBUS_FMT_YVYU8_2X8:
		formats++;
		if (xlate) {
			xlate->host_fmt = &rda_camera_formats[0];
			xlate->code = code;
			xlate++;
			rda_dbg_camera("%s: Providing format %s using code %d\n",
					__func__, rda_camera_formats[0].name, code);
		}
		break;
	default:
		if (!rda_camera_packing_supported(fmt))
			return 0;
		if (xlate)
			rda_dbg_camera("%s: Providing format %s in pass-through mode\n",
					__func__, fmt->name);
	}

	/* Generic pass-through */
	formats++;
	if (xlate) {
		xlate->host_fmt = fmt;
		xlate->code = code;
		xlate++;
	}

	return formats;
}

/* Called with .host_lock held */
static int rda_camera_add_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct rda_camera_dev *rcam = ici->priv;
	int ret;

	if (rcam->icd)
		return -EBUSY;

	ret = clk_enable(rcam->pclk);
	if (ret)
		return ret;


	rcam->icd = icd;
	rda_dbg_camera("%s: Camera driver attached to camera %d\n",
			__func__, icd->devnum);
	return 0;
}

/* Called with .host_lock held */
static void rda_camera_remove_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct rda_camera_dev *rcam = ici->priv;

#ifdef _TGT_AP_CAM_ISP_ENABLE
	HWP_CAMERA_T *hwp_cam = (HWP_CAMERA_T*)rcam->regs;
	hwp_cam->CTRL &= ~(CAMERA_ISP_ENABLE | CAMERA_LINEBUF_ISP);
	ISP_INIT = 0;

	rcam->raw_sensor =0 ;
#endif
	BUG_ON(icd != rcam->icd);

	clk_disable(rcam->pclk);
	rcam->icd = NULL;

        hwp_sysCtrlAp->GCG_Rst_Set = SYS_CTRL_AP_SET_GCG_RST_CAMERA;
        hwp_sysCtrlAp->GCG_Rst_Clr = SYS_CTRL_AP_CLR_GCG_RST_CAMERA;

	rda_dbg_camera("%s: Camera driver detached from camera %d\n",
			__func__, icd->devnum);
}

static unsigned int rda_camera_poll(struct file *file, poll_table *pt)
{
	struct soc_camera_device *icd = file->private_data;

	return vb2_poll(&icd->vb2_vidq, file, pt);
}

static int rda_camera_querycap(struct soc_camera_host *ici,
		struct v4l2_capability *cap)
{
	strcpy(cap->driver, RDA_CAMERA_DRV_NAME);
	strcpy(cap->card, "RDA Camera Sensor Interface");
	cap->capabilities = (V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING);
	return 0;
}

static struct soc_camera_host_ops rda_soc_camera_host_ops = {
	.owner		= THIS_MODULE,
	.add		= rda_camera_add_device,
	.remove		= rda_camera_remove_device,
	.set_fmt	= rda_camera_set_fmt,
	.try_fmt	= rda_camera_try_fmt,
	.get_formats	= rda_camera_get_formats,
	.init_videobuf2	= rda_camera_init_videobuf,
	.poll		= rda_camera_poll,
	.querycap	= rda_camera_querycap,
	.set_bus_param	= rda_camera_set_bus_param,
};

static int rda_camera_remove(struct platform_device *pdev)
{
	struct soc_camera_host *soc_host = to_soc_camera_host(&pdev->dev);
	struct rda_camera_dev *rcam = container_of(soc_host,
			struct rda_camera_dev, soc_host);

	free_irq(rcam->irq, rcam);
	soc_camera_host_unregister(soc_host);
	vb2_dma_contig_cleanup_ctx(rcam->alloc_ctx);
	cam_regs = NULL;
	iounmap(rcam->regs);
	clk_unprepare(rcam->pclk);
	clk_put(rcam->pclk);
	kfree(rcam);

	return 0;
}

static int rda_camera_probe(struct platform_device *pdev)
{
	unsigned int irq;
	struct rda_camera_dev *rcam;
	struct clk *pclk;
	struct resource *regs;
	int ret;
	struct device *dev = &pdev->dev;
	struct soc_camera_host *soc_host;
	struct rda_camera_device_data *pdata;

	pdata = dev->platform_data;
	if (!pdata) {
		printk(KERN_ERR "%s: No platform data available\n", __func__);
		return -EINVAL;
	}

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs)
		return -ENXIO;

	pclk = clk_get(&pdev->dev, RDA_CLK_CAMERA);
	if (IS_ERR(pclk))
		return PTR_ERR(pclk);
	ret = clk_prepare(pclk);
	if (ret)
		goto err_clk_prepare;

	rcam = kzalloc(sizeof(struct rda_camera_dev), GFP_KERNEL);
	if (!rcam) {
		ret = -ENOMEM;
		printk(KERN_ERR "%s: Can't allocate interface!\n", __func__);
		goto err_alloc_rcam;
	}

	rcam->pclk = pclk;
	rcam->pdata = pdata;
	rcam->active = NULL;
	rcam->next = NULL;
	spin_lock_init(&rcam->lock);

#ifdef _TGT_AP_CAM_ISP_ENABLE //_TGT_AP_CAM_ISP_ENABLE
	INIT_DELAYED_WORK(&rcam->isr_work, adjust_exp);
#endif
//	INIT_DELAYED_WORK(&rcam->isr_work, handle_vsync);
//	tasklet_init(&rcam_tasklet, handle_vsync, (unsigned long)rcam);
	init_waitqueue_head(&rcam->vsync_wq);
	INIT_LIST_HEAD(&rcam->cap_buffer_list);

	rcam->alloc_ctx = vb2_dma_contig_init_ctx(&pdev->dev);
	if (IS_ERR(rcam->alloc_ctx)) {
		ret = PTR_ERR(rcam->alloc_ctx);
		goto err_alloc_ctx;
	}

	rcam->regs = ioremap(regs->start, resource_size(regs));
	if (!rcam->regs) {
		ret = -ENOMEM;
		goto err_ioremap;
	}
	cam_regs = rcam->regs;

	irq = platform_get_irq(pdev, 0);
	if (IS_ERR_VALUE(irq)) {
		ret = irq;
		goto err_req_irq;
	}
	ret = request_irq(irq, rda_camera_isr, IRQF_SHARED, pdev->name, rcam);
	if (ret) {
		printk(KERN_ERR "%s: Unable to request irq %d\n",
				__func__, irq);
		goto err_req_irq;
	}
	rcam->irq = irq;

	soc_host = &rcam->soc_host;
	soc_host->drv_name = RDA_CAMERA_DRV_NAME;
	soc_host->ops = &rda_soc_camera_host_ops;
	soc_host->priv = rcam;
	soc_host->v4l2_dev.dev = &pdev->dev;
	soc_host->nr = pdev->id;

	ret = soc_camera_host_register(soc_host);
	if (ret) {
		printk(KERN_ERR "%s: Unable to register soc camera host\n",
				__func__);
		goto err_register_soc_camera_host;
	}
	return 0;

err_register_soc_camera_host:
	free_irq(rcam->irq, rcam);
err_req_irq:
	cam_regs = NULL;
	iounmap(rcam->regs);
err_ioremap:
	vb2_dma_contig_cleanup_ctx(rcam->alloc_ctx);
err_alloc_ctx:
//	tasklet_kill(&rcam_tasklet);
	kfree(rcam);
err_alloc_rcam:
	clk_unprepare(pclk);
err_clk_prepare:
	clk_put(pclk);

	return ret;
}

static struct platform_driver rda_camera_driver = {
	.probe = rda_camera_probe,
	.remove = rda_camera_remove,
	.driver = {
		.name = RDA_CAMERA_DRV_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init rda_camera_init_module(void)
{
	return platform_driver_probe(&rda_camera_driver, &rda_camera_probe);
}

static void __exit rda_camera_exit(void)
{
	platform_driver_unregister(&rda_camera_driver);
}

module_init(rda_camera_init_module);
module_exit(rda_camera_exit);

MODULE_AUTHOR("Wei Xing <xingwei@rdamicro.com>");
MODULE_DESCRIPTION("The V4L2 driver for RDA camera");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("video");

