/*
 * Kuke Ant GPMC-FPGA Access Driver
 *
 * Copyright (c) 2019 wangping
 *
 * This program is through gpmc interface to access fpga resource;
 * use dma to get sysinfo, wbc, hgb data, and use cpu interface to get 
 * fpga control regs.
 */

/*
 * FPGA Memory Resouces
 *
 * fpga connect to gpmc cs0
 */


#ifndef __DRIVERS_FPGA_INTERFACE_H
#define __DRIVERS_FPGA_INTERFACE_H
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/miscdevice.h>
#include <linux/vmalloc.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/kref.h>
#include <linux/io.h>
#include <linux/sizes.h>
#include <linux/delay.h>

extern int fpga_msg_level;

#define FPGA_CONTROL_PATH 0
#define FPGA_DATA_PATH 1
#define FPGA_MONI_PATH 2



enum DevType
{
	// To FPGA
	VALVE = 1,
	PUMP,
	LASER,
	CONSTANT,
	BURN_JEWEL_HOLE,
	LED,

	MOTOR_CTRL,
	MOTOR_INIT,
	MOTOR_CYCLE,
	INJECT_CTRL,
	INJECT_INIT,
	INJECT_STAIR,

	SET_REG,
	VAC_SETUP,

	DETC_VAC,
	DETC_PRESS,
	DETC_TEMP,
	DETC_POOL_LEVEL,
	DETC_AIR_BUBBLE,

	DATA_HGB_BLANK,
	DATA_HGB_BLOOD,
	DATA_DIFF,
	DATA_WBC,
	DATA_RBC,

	// To serial
	SAMPLER_CTRL,

	// To userspace
	EVENT_UNLIMITED 	= 100,
	EVENT_LIMITED_START,
	EVENT_LIMITED_END,
	EVENT_START,
	EVENT_END,
	EVENT_ASP_COMPLETE,

};


#define FPGA_CTRL_DEV_TYPE_MIN 1
#define FPGA_CTRL_DEV_TYPE_MAX 19
#define FPGA_MONI_DEV_TYPE_MIN 1
#define FPGA_MONI_DEV_TYPE_MAX 5


#define FPGA_CHMON_REG_BASE 0x00000000

#define FPGA_SYS_GLOBAL_CNT_FLAG (FPGA_CHMON_REG_BASE + 0x2e)

#define FPGA_SYS_CHAN_HGB  (FPGA_CHMON_REG_BASE + 0x2)
#define FPGA_SYS_CHAN_RBC_MON (FPGA_CHMON_REG_BASE + 0x4)
#define FPGA_SYS_CHAN_WBC_MON (FPGA_CHMON_REG_BASE + 0x6)
#define FPGA_SYS_CHAN_A56V_MON (FPGA_CHMON_REG_BASE + 0x8)
#define FPGA_SYS_CHAN_A12V_MON (FPGA_CHMON_REG_BASE + 0xa)
#define FPGA_SYS_CHAN_AN12V_MON (FPGA_CHMON_REG_BASE + 0xc)
#define FPGA_SYS_CHAN_P24V_MON (FPGA_CHMON_REG_BASE + 0xe)
#define FPGA_SYS_CHAN_LIQUIDPRESS (FPGA_CHMON_REG_BASE + 0x10)
#define FPGA_SYS_CHAN_AIRPRESS (FPGA_CHMON_REG_BASE + 0x12)
#define FPGA_SYS_CHAN_LIQ1 (FPGA_CHMON_REG_BASE + 0x14)
#define FPGA_SYS_CHAN_LIQ2 (FPGA_CHMON_REG_BASE + 0x16)
#define FPGA_SYS_CHAN_LIQ3 (FPGA_CHMON_REG_BASE + 0x18)
#define FPGA_SYS_CHAN_PMT_HV_MON (FPGA_CHMON_REG_BASE + 0x1a)
#define FPGA_SYS_CHAN_LD_CURT_MON (FPGA_CHMON_REG_BASE + 0x1c)

#define FPGA_SYS_CHAN_TEMP1 (FPGA_CHMON_REG_BASE + 0x1e)
#define FPGA_SYS_CHAN_TEMP2 (FPGA_CHMON_REG_BASE + 0x20)
#define FPGA_SYS_CHAN_TEMP3 (FPGA_CHMON_REG_BASE + 0x22)
#define FPGA_SYS_CHAN_TEMP4 (FPGA_CHMON_REG_BASE + 0x24)
#define FPGA_SYS_CHAN_TEMP5 (FPGA_CHMON_REG_BASE + 0x26)
#define FPGA_SYS_CHAN_TEMP6 (FPGA_CHMON_REG_BASE + 0x28)

#define FPGA_SYS_FLOAT (FPGA_CHMON_REG_BASE + 0x34)


#define FPGA_RBC_REG_BASE 0x00010000


#define FPGA_FIFO_RBC_START (FPGA_RBC_REG_BASE + 0x2)
#define FPGA_FIFO_RBC_FILT_EN (FPGA_RBC_REG_BASE + 0x4)
#define FPGA_FIFO_RBC_SAMPTIME_H (FPGA_RBC_REG_BASE + 0x6)
#define FPGA_FIFO_RBC_SAMPTIME_L (FPGA_RBC_REG_BASE + 0x8)
#define FPGA_FIFO_RBC_FILT_GATE (FPGA_RBC_REG_BASE + 0xa)
#define FPGA_FIFO_RBC_STATUS (FPGA_RBC_REG_BASE + 0xc)
#define FPGA_FIFO_RBC_STATUS_FIFO_FULL_BIT 14
#define FPGA_FIFO_RBC_STATUS_FIFO_EMP_BIT 13
#define FPGA_FIFO_RBC_STATUS_FIFO_LENGTH 12
#define FPGA_FIFO_RBC_ADDR (FPGA_RBC_REG_BASE + 0x22)

#define FPGA_FIFO_RBC_INT_MASK 0x40002
#define FPGA_FIFO_RBC_INT_SOURCE 0x40004



#define FPGA_FIFO_WBC_REG_BASE 0x00070000
#define FPGA_FIFO_WBC_START (FPGA_FIFO_WBC_REG_BASE + 0x2)
#define FPGA_FIFO_WBC_FILT_EN (FPGA_FIFO_WBC_REG_BASE + 0x4)
#define FPGA_FIFO_WBC_SAMPTIME_H (FPGA_FIFO_WBC_REG_BASE + 0x6)
#define FPGA_FIFO_WBC_SAMPTIME_L (FPGA_FIFO_WBC_REG_BASE + 0x8)
#define FPGA_FIFO_WBC_FILT_GATE (FPGA_FIFO_WBC_REG_BASE + 0xa)
#define FPGA_FIFO_WBC_STATUS (FPGA_FIFO_WBC_REG_BASE + 0xc)
#define FPGA_FIFO_WBC_STATUS_FIFO_FULL_BIT 14
#define FPGA_FIFO_WBC_STATUS_FIFO_EMP_BIT 13
#define FPGA_FIFO_WBC_STATUS_FIFO_LENGTH 12

#define FPGA_FIFO_WBC_ADDR (FPGA_FIFO_WBC_REG_BASE + 0x22)



#define FPGA_FIFO_DIFF_REG_BASE 0x00080000
#define FPGA_FIFO_DIFF_START (FPGA_FIFO_DIFF_REG_BASE + 0x2)
#define FPGA_FIFO_DIFF_SAMPTIME_H (FPGA_FIFO_DIFF_REG_BASE + 0x4)
#define FPGA_FIFO_DIFF_SAMPTIME_L (FPGA_FIFO_DIFF_REG_BASE + 0x6)
#define FPGA_FIFO_DIFF_FILT_GATE (FPGA_FIFO_DIFF_REG_BASE + 0x28)
#define FPGA_FIFO_DIFF_STATUS (FPGA_FIFO_DIFF_REG_BASE + 0x26)
#define FPGA_FIFO_DIFF_STATUS_FIFO_FULL_BIT 14
#define FPGA_FIFO_DIFF_STATUS_FIFO_EMP_BIT 13
#define FPGA_FIFO_DIFF_STATUS_FIFO_LENGTH 12
#define FPGA_FIFO_DIFF_ADDR (FPGA_FIFO_DIFF_REG_BASE + 0x24)




//interrupt reg
#define FPGA_INTER_REG_BASE  0x00040000

#define FPGA_INTER_RBC_MASK  FPGA_INTER_REG_BASE + 0x2
#define FPGA_INTER_RBC_SOURCE  FPGA_INTER_REG_BASE + 0x4
#define INTERRUPT_RBC_MASK_NORMAL_INT_BIT_6 6
#define INTERRUPT_RBC_MASK_END_INT_BIT_7 7

#define INTERRUPT_RBC_SOURCE_NORMAL_INT_BIT_6 6
#define INTERRUPT_RBC_SOURCE_END_INT_BIT_7 7



#define FPGA_INTER_WBC_MASK  FPGA_INTER_REG_BASE + 0xa
#define FPGA_INTER_WBC_SOURCE  FPGA_INTER_REG_BASE + 0xc
#define INTERRUPT_WBC_MASK_NORMAL_INT_BIT_6 6
#define INTERRUPT_WBC_MASK_END_INT_BIT_7 7

#define INTERRUPT_WBC_SOURCE_NORMAL_INT_BIT_6 6
#define INTERRUPT_WBC_SOURCE_END_INT_BIT_7 7


#define FPGA_INTER_DIFF_MASK  FPGA_INTER_REG_BASE + 0xe
#define FPGA_INTER_DIFF_SOURCE  FPGA_INTER_REG_BASE + 0x10
#define INTERRUPT_DIFF_MASK_NORMAL_INT_BIT_6 6
#define INTERRUPT_DIFF_MASK_END_INT_BIT_7 7

#define INTERRUPT_DIFF_SOURCE_NORMAL_INT_BIT_6 6
#define INTERRUPT_DIFF_SOURCE_END_INT_BIT_7 7





#define SYS_CHAN_HGB 0
#define SYS_CHAN_RBC_MON 1
#define SYS_CHAN_WBC_MON 2
#define SYS_CHAN_A56V_MON 3
#define SYS_CHAN_A12V_MON 4
#define SYS_CHAN_AN12V_MON 5
#define SYS_CHAN_P24V_MON 6
#define SYS_CHAN_LIQUIDPRESS 7
#define SYS_CHAN_AIRPRESS 8
#define SYS_CHAN_LIQ1 9
#define SYS_CHAN_LIQ2 10
#define SYS_CHAN_LIQ3 11
#define SYS_CHAN_PMT_HV_MON 12
#define SYS_CHAN_LD_CURT_MON 13
#define SYS_CHAN_TEMP1 14
#define SYS_CHAN_TEMP2 15


//io control reg
#define FPGA_IOCTRL_REG_BASE 0x00030000
#define FPGA_NEGA_PUMP_EN (FPGA_IOCTRL_REG_BASE+0x34)
#define FPGA_NEGA_PRESSURE_MIN (FPGA_IOCTRL_REG_BASE+0x2)
#define FPGA_NEGA_PRESSURE_TARGET (FPGA_IOCTRL_REG_BASE+0x4)
#define FPGA_NEGA_PRESSURE_MAX (FPGA_IOCTRL_REG_BASE+0x36)
#define FPGA_NEGA_PRESSURE_ACTUAL (FPGA_CHMON_REG_BASE+0x12)


#define FPGA_FAN1_PWM (FPGA_IOCTRL_REG_BASE+0x6)
#define FPGA_VALVE_A (FPGA_IOCTRL_REG_BASE+0x8)
#define  FPGA_VALVE_B (FPGA_IOCTRL_REG_BASE+0xa)
#define FPGA_VALVE_C (FPGA_IOCTRL_REG_BASE+0xc)

#define HEAT_D_EN (FPGA_IOCTRL_REG_BASE + 0xe)
#define HEAT_DIFF_TEMP (FPGA_IOCTRL_REG_BASE + 0x10)

#define HEAT_O_EN (FPGA_IOCTRL_REG_BASE + 0x18)
#define HEAT_O_TEMP (FPGA_IOCTRL_REG_BASE + 0x1a)

#define MIN_VALVE_A 1
#define MAX_VALVE_A 16
#define MIN_VALVE_B 17
#define MAX_VALVE_B 32
#define MIN_VALVE_C 33
#define MAX_VALVE_C 36
#define MAX_VALVE_C_EXT 61


#define FPGA_HEAT_D_EN (FPGA_IOCTRL_REG_BASE+0xe)
#define FPGA_HEAT_D_TEMP (FPGA_IOCTRL_REG_BASE+0x10)
#define FPGA_HEAT_D_THRE (FPGA_IOCTRL_REG_BASE+0x12)
#define FPGA_HEAT_D_Kp (FPGA_IOCTRL_REG_BASE+0x14)
#define FPGA_HEAT_D_Kd (FPGA_IOCTRL_REG_BASE+0x16)
#define FPGA_HEAT_O_EN (FPGA_IOCTRL_REG_BASE+0x18)
#define FPGA_HEAT_O_TEMP (FPGA_IOCTRL_REG_BASE+0x1a)
#define FPGA_HEAT_O_THRE (FPGA_IOCTRL_REG_BASE+0x1c)
#define FPGA_HEAT_O_Kp (FPGA_IOCTRL_REG_BASE+0x1e)
#define FPGA_HEAT_O_Kd (FPGA_IOCTRL_REG_BASE+0x20)
#define FPGA_LAMP (FPGA_IOCTRL_REG_BASE+0x22)
#define FPGA_BEEP (FPGA_IOCTRL_REG_BASE+0x26)
#define FPGA_RBC_BURN (FPGA_IOCTRL_REG_BASE+0x28)
#define FPGA_WBC_BURN (FPGA_IOCTRL_REG_BASE+0x2a)
#define FPGA_CONST_CTRL (FPGA_IOCTRL_REG_BASE+0x2c)
#define FPGA_HGB_LED (FPGA_IOCTRL_REG_BASE+0x2e)
#define FPGA_LIQ_LED (FPGA_IOCTRL_REG_BASE+0x30)
#define FPGA_LASER_DIODE (FPGA_IOCTRL_REG_BASE+0x32)







#define FPGA_INT1_MASK_RBC  (FPGA_INTER_REG_BASE + 0x2)
#define FPGA_INT1_SOURCE_RBC  (FPGA_INTER_REG_BASE + 0x2)
#define FPGA_INT2_MASK_WBC  (FPGA_INTER_REG_BASE + 0x2)
#define FPGA_INT2_SOURCE_WBC  (FPGA_INTER_REG_BASE + 0x2)
#define FPGA_INT3_MASK_OPT  (FPGA_INTER_REG_BASE + 0x2)
#define FPGA_INT3_SOURCE_OPT  (FPGA_INTER_REG_BASE + 0x2)
#define FPGA_INT4_MASK_SWITCH  (FPGA_INTER_REG_BASE + 0x2)
#define FPGA_INT4_SOURCE_SWITCH  (FPGA_INTER_REG_BASE + 0x2)

#define FPGA_INT5_MASK_MOTOR (FPGA_INTER_REG_BASE + 0x2)
#define FPGA_INT5_SOURCE_MOTOR (FPGA_INTER_REG_BASE + 0x2)

//interrupt source sys/rbc/wbc/opt info
#define INT_SOURCE_FIFO_EMPTY_BIT  (0x1 << 0)
#define INT_SOURCE_FIFO_FULL_BIT  (0x1 << 1)
#define INT_SOURCE_FIFO_HALFFULL_BIT  (0x1 << 2)

//interrupt source float en/dis
#define INT_SOURCE_FLOAT_BIT (0x1 << 0)

//interrupt source motor info
#define INT_SOURCE_MOTOR_SAMPH_BIT  (0x1 << 0)
#define INT_SOURCE_MOTOR_SAMPV_BIT (0x1 << 1)
#define INT_SOURCE_MOTOR_MIXH_BIT (0x1 << 2)
#define INT_SOURCE_MOTOR_MIXV_BIT  (0x1 << 3)
#define INT_SOURCE_MOTOR_MIXR_BIT (0x1 << 4)
#define INT_SOURCE_MOTOR_INJ1_BIT (0x1 << 5)
#define INT_SOURCE_MOTOR_INJ2_BIT (0x1 << 6)
#define INT_SOURCE_MOTOR_INJ3_BIT (0x1 << 7)

#define INT_SOURCE_MOTOR_SAMPH  0
#define INT_SOURCE_MOTOR_SAMPV 1
#define INT_SOURCE_MOTOR_MIXH  2
#define INT_SOURCE_MOTOR_MIXV 3
#define INT_SOURCE_MOTOR_MIXR  4
#define INT_SOURCE_MOTOR_INJ1 5
#define INT_SOURCE_MOTOR_INJ2 6
#define INT_SOURCE_MOTOR_INJ3 7


#define FPGA_MOTOR_TYPE_INJ 1

#define FPGA_MOTOR_TYPE_MAIN 2


#define FPGA_MAIN_MOTOR_SAMPLE_V 	1
#define FPGA_MAIN_MOTOR_SAMPLE_H 	2
#define FPGA_MAIN_MOTOR_MIX_V 	3
#define FPGA_MAIN_MOTOR_MIX_R 	4
#define FPGA_MAIN_MOTOR_MIX_H 	5


#define FPGA_MAIN_MOTOR_MAX	6

#define FPGA_MAIN_MOTOR_POS_MAX	12


#define FPGA_INJ1_MOTOR	1
#define FPGA_INJ2_MOTOR	2
#define FPGA_INJ3_MOTOR	3
#define FPGA_INJ_MOTOR_MAX 	4


#define FPGA_INJ1_MOTOR_TEMP	6
#define FPGA_INJ2_MOTOR_TEMP	7
#define FPGA_INJ3_MOTOR_TEMP	8


#define FPGA_MOTOR_REG_BASE 0x00050000
#define   FPGA_INJ1_MOTOR_BASE (FPGA_MOTOR_REG_BASE + 0x6000)
#define   FPGA_INJ2_MOTOR_BASE (FPGA_MOTOR_REG_BASE + 0x7000)
#define   FPGA_INJ3_MOTOR_BASE (FPGA_MOTOR_REG_BASE + 0x8000)




#define   FPGA_SAMPH_MOTOR_BASE (FPGA_MOTOR_REG_BASE + 0x2000)
#define   FPGA_SAMPV_MOTOR_BASE (FPGA_MOTOR_REG_BASE + 0x1000)
#define   FPGA_MIXH_MOTOR_BASE (FPGA_MOTOR_REG_BASE + 0x5000)
#define   FPGA_MIXV_MOTOR_BASE (FPGA_MOTOR_REG_BASE + 0x3000)
#define   FPGA_MIXR_MOTOR_BASE (FPGA_MOTOR_REG_BASE + 0x4000)


#define  FPGA_MOTOR_STEP_MAX 0x0
#define  FPGA_MOTOR_STEP_ADD 0x2
#define  FPGA_MOTOR_STEP_ALL 0x4
#define  FPGA_MOTOR_DIR 0x6
#define  FPGA_MOTOR_HOLD 0x8
#define  FPGA_MOTOR_SPEED 0xa
#define  FPGA_MOTOR_PHOTO_NUM 0xc
#define  FPGA_MOTOR_PHOTO_WIDTH 0xe

#define FPGA_MOTOR_CHANGE_POINT 0x10
#define FPGA_MOTOR_CHANGE_SPEED 0x12

#define  FPGA_MOTOR_MODE  0x14

#define  FPGA_MOTOR_RUN 0x16
#define FPGA_MOTOR_SPD_TAB15 0x84


#define  FPGA_MOTOR_IGNORE  0x1e
#define  FPGA_MOTOR_MICROSTEP  0x20
#define  FPGA_MOTOR_ACC_INTER 0x22
#define FPGA_MOTOR_DEC_INTER 0x24

#define FPGA_MOTOR_RUN_TIME 0x28


#define  FPGA_MOTOR_INDEX_START 0x64


#define  FPGA_MOTOR_INIT_ENABLE 0x88
#define  FPGA_MOTOR_INIT_SPEED 0x26



#define FPGA_RBC_REG  0x00040000
#define FPGA_WBC_REG 0x00040000
#define FPGA_DIFF_REG 0x00040000


#define FPGA_BUBBLE 0x00040000




struct fpga_ctrl_fp
{
	//u32 dev_type;
	void (*pfun)(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri);

};

struct fpga_moni_fp
{
	//u32 dev_type;
	void (*pfun)(u16 dev_no, int p1, int p2);
};


struct fpga_motor_fp
{
	//u32 dev_type;
	void (*pfun)(u16 dev_no, int p1, int p2);
};



/************************control*****************************/
void fpga_set_valve(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri);
void fpga_set_pump(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri);
void  fpga_set_laser_ctrl(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri);

void fpga_set_cnst_ctrl(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri);
void fpga_set_burn_ctrl(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri);

void fpga_set_led(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri);

void fpga_set_collect_reg(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri);

void fpga_setup_vac(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri);

/***************************moni***************************************/
void fpga_moni_vac(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri);
void fpga_moni_press(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri);
void fpga_moni_temp(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri);
void fpga_moni_pool(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri);
void fpga_moni_bubble(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri);

void fpga_type_invalid(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri);


void fpga_inject_motor_ctrl(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri); //step <0:out ; step >0 :in
void fpga_inject_motor_init(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri);
void fpga_inject_motor_stair(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri);

void fpga_motor_ctrl(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri);
void fpga_motor_init(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri);
void fpga_motor_cycle(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri);


void fpga_get_hgb_blank(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri);
void fpga_get_hgb_blood(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri);


void fpga_get_fifo_rbc(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri);
void fpga_get_fifo_wbc(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri);
void fpga_get_fifo_diff(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri);

#if 0
void fpga_get_sys_hgb(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri);
void fpga_get_sys_rbc(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri);
void fpga_get_sys_wbc(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri);
void fpga_get_sys_liqpress(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri) ;
void fpga_get_sys_airpress(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri);
void fpga_get_sys_liq1(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri);
void fpga_get_sys_pmt(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri);
void fpga_get_sys_ld(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri);
void fpga_get_sys_temp1(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri);
void fpga_get_sys_temp2(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri);
#endif

void fpga_sampler_ctrl(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri);

void fpga_event_unlimited(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri);
void fpga_event_limited_start(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri);
void fpga_event_limited_end(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri);
void fpga_event_start(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri);
void fpga_event_end(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri);
void fpga_event_asp_complete(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri);


void fpga_timer_init(struct fpga_dev *pdev);

int load_motor_pos_steps(void);
void set_motor_init_accspeed(void);

void netlink_other_ctrl_type_handle(char *ctrldata, int ctrl_type);
int update_motor_pos_steps(int row, int col);


/*********************************** * SYSFS Attributes************************/
ssize_t fpga_netlink_cmd_set(struct device *dev, struct device_attribute *attr,const char *buf, size_t count);
ssize_t gpmc_dumpreg_show(struct device *dev, struct device_attribute *attr,char *buf);
ssize_t gpmc_dump_motor1_reg_show(struct device *dev, struct device_attribute *attr,
			    char *buf);
ssize_t gpmc_dump_motor2_reg_show(struct device *dev, struct device_attribute *attr,
			    char *buf);
ssize_t gpmc_dump_motor3_reg_show(struct device *dev, struct device_attribute *attr,
			    char *buf);


ssize_t gpmc_fpga_reg_set(struct device *dev, struct device_attribute *attr,  const char *buf, size_t count);
ssize_t gpmc_fpga_reg_show(struct device *dev, struct device_attribute *attr,char *buf);
ssize_t sys_motor_ctrl_set(struct device *dev, struct device_attribute *attr,const char *buf, size_t count);
ssize_t sys_inject_motor_ctrl(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

ssize_t sys_valve_ctrl_set(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count);
ssize_t sys_pump_ctrl_set(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count);
ssize_t sys_nega_pressure_set(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count);

ssize_t sys_inject_motor_init(struct device *dev, struct device_attribute *attr,
	   				const char *buf, size_t count);
ssize_t sys_motor_init(struct device *dev, struct device_attribute *attr,
	   				const char *buf, size_t count);

ssize_t sys_update_motor_pos_steps(struct device *dev, struct device_attribute *attr,
	   				const char *buf, size_t count);

ssize_t sys_get_fifo_rbc(struct device *dev, struct device_attribute *attr,
	   				const char *buf, size_t count);
ssize_t sys_get_fifo_wbc(struct device *dev, struct device_attribute *attr,
	   				const char *buf, size_t count);
ssize_t sys_get_fifo_diff(struct device *dev, struct device_attribute *attr,
	   				const char *buf, size_t count);



ssize_t sys_cpu_read_rbc(struct device *dev, struct device_attribute *attr,
	   				const char *buf, size_t count);

ssize_t sys_trigger_rbc_dma(struct device *dev, struct device_attribute *attr,
	   				const char *buf, size_t count);

#endif
