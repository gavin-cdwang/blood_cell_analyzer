/*
 * Kuake Ant GPMC-FPGA(FPGA_INTERFACE) Access Driver
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
*  the fpag interface used for access the fpag register resources
 */


#include <linux/of_address.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/dma-mapping.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/dmaengine.h>
#include <linux/seq_file.h>
#include <linux/highmem.h>
#include <linux/debugfs.h>
#include <linux/vmalloc.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/kref.h>
#include <linux/io.h>
#include <linux/sizes.h>
#include <linux/delay.h>
#include <linux/netlink.h>
#include <net/sock.h>
#include <linux/rtc.h>

#include <linux/kthread.h>
#include "gpmc_fpga.h"
#include "fpga_timing_parser.h"
#include "fpga_interface.h"
#include "gpmc_fpga.h"




unsigned long g_time1,g_time2,g_time4,g_time5;

extern int lenarr[10];
extern int cnt;


extern int netlink_port;
extern int debug_fpga_reg;


extern int rbc_fisrt_flag;
extern int rbc_per_len;
extern int rbc_end_rpt_flag;
extern int rbc_total;
extern int rbc_irq;
extern int rbctime[8000];
extern int rbc_xfer_len[6000];
extern int cpycnt;
extern int rbc_cpy_time[6000];


extern int wbc_fisrt_flag;
extern int wbc_per_len;
extern int wbc_end_rpt_flag;
extern int wbc_total;
extern int wbc_irq;
extern int wbctime[8000];
extern int wbc_xfer_len[6000];


extern int diff_fisrt_flag;
extern int diff_per_len;
extern int diff_end_rpt_flag;
extern int diff_total;
extern int diff_irq;
extern int difftime[8000];
extern int diff_xfer_len[6000];




//read hgb reg : 16 
#define HGB_READ_CNT 10

//u16 hgb_buf[HGB_READ_CNT+1];
u16  hgb_buf[HGB_READ_CNT];

extern struct fpga_dev *g_fpga_dev;

int rbc_time ;

static int main_pre_steps[FPGA_MAIN_MOTOR_MAX] = {0};
//for other ctrl type info:
static int main_curr_steps[FPGA_MAIN_MOTOR_MAX] = {0};
static int main_steps[FPGA_MAIN_MOTOR_MAX]= {0};
static int main_speed[FPGA_MAIN_MOTOR_MAX]= {0};
int main_abs_steps [FPGA_MAIN_MOTOR_MAX]= {0};


int main_current_pos = 0, inj_current_pos=0;


static int main_devno;

static int inj_pre_steps[FPGA_INJ_MOTOR_MAX] = {0};
static int abs_steps[FPGA_INJ_MOTOR_MAX] = {0};

static int inj_steps[FPGA_MAIN_MOTOR_MAX]= {0};
static int inj_speed[FPGA_MAIN_MOTOR_MAX]= {0};


int main_motor_pos_map[FPGA_MAIN_MOTOR_MAX][FPGA_MAIN_MOTOR_POS_MAX] = {
		{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0 }, //invalid;
		{ 300, 350, 400, 450, 500, 550, 600, 0, 0, 0, 0,0 },	//FPGA_MAIN_MOTOR_SAMPLE_V
		{ 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0 },	//FPGA_MAIN_MOTOR_SAMPLE_H
		{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0 },	//FPGA_MAIN_MOTOR_MIX_V
		{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0},	//FPGA_MAIN_MOTOR_MIX_R
		{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0},	//FPGA_MAIN_MOTOR_MIX_H
};




int inj_motor_init_pos [FPGA_INJ_MOTOR_MAX]={0,0,0,0};

struct fpga_ctrl_fp fpga_ctrl[]={
/*** **********io control************/	
	{.pfun = fpga_type_invalid },	
	{.pfun = fpga_set_valve },  //type valve
	{.pfun = fpga_set_pump },
	{.pfun = fpga_set_laser_ctrl },   //devtype 3
	{.pfun = fpga_set_cnst_ctrl },
	{.pfun = fpga_set_burn_ctrl },
	{.pfun = fpga_set_led },
/*** **********motor control************/	\
	{.pfun = fpga_motor_ctrl},   //7
	{.pfun = fpga_motor_init }, //8
	{.pfun = fpga_motor_cycle }, //9

	{.pfun = fpga_inject_motor_ctrl }, //10
	{.pfun = fpga_inject_motor_init },  //11
	{.pfun = fpga_inject_motor_stair },//12
	
	{.pfun = fpga_set_collect_reg },
	{.pfun = fpga_setup_vac },

/*************monitor temp,vac,pool...**************/
	{.pfun = fpga_moni_vac },	//devtype 15
	{.pfun = fpga_moni_press },
	{.pfun = fpga_moni_temp },
	{.pfun = fpga_moni_pool },
	{.pfun = fpga_moni_bubble },

/*****************data sample**********************/
	{.pfun = fpga_get_hgb_blank }, //20
	{.pfun = fpga_get_hgb_blood },
	{.pfun = fpga_get_fifo_diff },	
	{.pfun = fpga_get_fifo_wbc },
	{.pfun = fpga_get_fifo_rbc },
	
/*******to serial: MCU contrl the sampler to move to the pos*************/
	{.pfun = fpga_sampler_ctrl },

/*******************to userspace event********************/
	{.pfun = fpga_event_unlimited },
	{.pfun = fpga_event_limited_start },
	{.pfun = fpga_event_limited_end },
	{.pfun = fpga_event_start },   //29
	{.pfun = fpga_event_end },
	{.pfun = fpga_event_asp_complete },
};

extern struct timespec tc_old;
extern long delay_time;
extern long timarry[50];



int gpmc_fpga_rbc_fifo_init(struct fpga_dev * priv)
{

	fpga_write_reg(priv,FPGA_FIFO_RBC_SAMPTIME_L, 0);
	fpga_write_reg(priv,FPGA_FIFO_RBC_SAMPTIME_H, 0);
	
	fpga_write_reg(g_fpga_dev, FPGA_FIFO_RBC_START, 0x1);
		 
}


int gpmc_fpga_rbc_fifo_enable(struct fpga_dev * priv, int time_delay)
{
	u16 time_l, time_h;
	u32 tick;


	//time_delay = rbc_time;
	
	tick = time_delay * 90000;
	
	fpga_info(("%s:time:%dms  tick:0x%x \n", __func__, time_delay, tick));
	
	time_l = tick& 0xffff;
	time_h = (tick & 0xffff0000) >> 16 ;
	printk("time_l:0x%x  time_h:0x%x \n", time_l, time_h);
	
	fpga_write_reg(priv,FPGA_FIFO_RBC_SAMPTIME_L, time_l);
	fpga_write_reg(priv,FPGA_FIFO_RBC_SAMPTIME_H, time_h);


	//fpga_write_reg(priv,FPGA_FIFO_RBC_SAMPTIME_L, 0x1378);
	//fpga_write_reg(priv,FPGA_FIFO_RBC_SAMPTIME_H, 0xb);

	fpga_write_reg(priv,FPGA_FIFO_RBC_FILT_GATE, 0x1);

	fpga_write_reg(g_fpga_dev, FPGA_FIFO_RBC_INT_SOURCE, 0); 
	fpga_write_reg(g_fpga_dev, FPGA_FIFO_RBC_INT_MASK, 0);

	//enable rbc sample
	fpga_write_reg(g_fpga_dev, FPGA_FIFO_RBC_START, 0x1);
}



int gpmc_fpga_wbc_fifo_init(struct fpga_dev * priv)
{

	fpga_write_reg(priv,FPGA_FIFO_WBC_SAMPTIME_L, 0);
	fpga_write_reg(priv,FPGA_FIFO_WBC_SAMPTIME_H, 0);
	
	fpga_write_reg(g_fpga_dev, FPGA_FIFO_WBC_START, 0x1);
		 
}


int gpmc_fpga_wbc_fifo_enable(struct fpga_dev * priv, int time_delay)
{
	u16 time_l, time_h;
	u32 tick;

	
	tick = time_delay * 90000;
	
	printk("%s:time:%dms  tick:0x%x \n", __func__, time_delay, tick);
	
	time_l = tick& 0xffff;
	time_h = (tick & 0xffff0000) >> 16 ;
	//printk("time_l:0x%x  time_h:0x%x \n", time_l, time_h);
	
	fpga_write_reg(priv,FPGA_FIFO_WBC_SAMPTIME_L, time_l);
	fpga_write_reg(priv,FPGA_FIFO_WBC_SAMPTIME_H, time_h);

	fpga_write_reg(priv,FPGA_FIFO_WBC_FILT_GATE, 0x1);

	fpga_write_reg(g_fpga_dev, FPGA_INTER_WBC_MASK, 0); 
	fpga_write_reg(g_fpga_dev, FPGA_INTER_WBC_SOURCE, 0);

	//enable rbc sample
	fpga_write_reg(g_fpga_dev, FPGA_FIFO_WBC_START, 0x1);
}


int gpmc_fpga_diff_fifo_init(struct fpga_dev * priv)
{

	fpga_write_reg(priv,FPGA_FIFO_DIFF_SAMPTIME_L, 0);
	fpga_write_reg(priv,FPGA_FIFO_DIFF_SAMPTIME_H, 0);
	
	fpga_write_reg(g_fpga_dev, FPGA_FIFO_DIFF_START, 0x1);
		 
}


int gpmc_fpga_diff_fifo_enable(struct fpga_dev * priv, int time_delay)
{
	u16 time_l, time_h;
	u32 tick;

	
	tick = time_delay * 90000;
	
	printk("%s:time:%dms  tick:0x%x \n", __func__, time_delay, tick);
	
	time_l = tick& 0xffff;
	time_h = (tick & 0xffff0000) >> 16 ;
	//printk("time_l:0x%x  time_h:0x%x \n", time_l, time_h);
	
	fpga_write_reg(priv,FPGA_FIFO_DIFF_SAMPTIME_L, time_l);
	fpga_write_reg(priv,FPGA_FIFO_DIFF_SAMPTIME_H, time_h);

	fpga_write_reg(priv,FPGA_FIFO_DIFF_FILT_GATE, 0x1);

	fpga_write_reg(g_fpga_dev, FPGA_INTER_DIFF_MASK, 0); 
	fpga_write_reg(g_fpga_dev, FPGA_INTER_DIFF_SOURCE, 0);

	//enable rbc sample
	fpga_write_reg(g_fpga_dev, FPGA_FIFO_DIFF_START, 0x1);
}



#define DUMP_POS_FILE


int get_pos_by_row(int file_buf, int row)
{
	char *tempbuf;
	
	switch(row) {
		
		case FPGA_MAIN_MOTOR_SAMPLE_V:
			return 0;
			
		case FPGA_MAIN_MOTOR_SAMPLE_H:
			 tempbuf =  strchr(file_buf, '\n');
			 if(tempbuf != NULL)
			 	return tempbuf - file_buf +1;

		 case FPGA_MAIN_MOTOR_MIX_V:
				  tempbuf =  strchr(file_buf, '\n');
				  tempbuf += 1;
				  tempbuf =  strchr(tempbuf, '\n');
				  
				  if(tempbuf != NULL)
					  	return tempbuf - file_buf +1;
				
		
		 case FPGA_MAIN_MOTOR_MIX_R:
				  tempbuf =  strchr(file_buf, '\n');
				  tempbuf += 1;
				  tempbuf =  strchr(tempbuf, '\n');
				  tempbuf += 1;
				  tempbuf =  strchr(tempbuf, '\n');
				  if(tempbuf != NULL)
					  	return tempbuf - file_buf +1;

		case FPGA_MAIN_MOTOR_MIX_H:
				 tempbuf =	strchr(file_buf, '\n');
				 tempbuf += 1;
				 tempbuf =	strchr(tempbuf, '\n');
				 tempbuf += 1;
				 tempbuf =	strchr(tempbuf, '\n');
				 tempbuf += 1;
				 tempbuf =	strchr(tempbuf, '\n');
				 if(tempbuf != NULL)
					   return tempbuf - file_buf +1;
		case FPGA_INJ1_MOTOR_TEMP:
				tempbuf =  strchr(file_buf, '\n');
				tempbuf += 1;
				tempbuf =  strchr(tempbuf, '\n');
				tempbuf += 1;
				tempbuf =  strchr(tempbuf, '\n');
				tempbuf += 1;
				tempbuf =  strchr(tempbuf, '\n');
				tempbuf += 1;
				tempbuf =  strchr(tempbuf, '\n');
				
				if(tempbuf != NULL)
					return tempbuf - file_buf +1;		   
				 
		case FPGA_INJ2_MOTOR_TEMP:
				tempbuf = strchr(file_buf, '\n');
				tempbuf += 1;
				tempbuf = strchr(tempbuf, '\n');
				tempbuf += 1;
				tempbuf = strchr(tempbuf, '\n');
				tempbuf += 1;
				tempbuf = strchr(tempbuf, '\n');
				tempbuf += 1;
				tempbuf = strchr(tempbuf, '\n');
				tempbuf += 1;
				tempbuf = strchr(tempbuf, '\n');						 
				if(tempbuf != NULL)
					 return tempbuf - file_buf +1;		
				 
		case FPGA_INJ3_MOTOR_TEMP:
				tempbuf = strchr(file_buf, '\n');
				tempbuf += 1;
				tempbuf = strchr(tempbuf, '\n');
				tempbuf += 1;
				tempbuf = strchr(tempbuf, '\n');
				tempbuf += 1;
				tempbuf = strchr(tempbuf, '\n');
				tempbuf += 1;
				tempbuf = strchr(tempbuf, '\n');
				tempbuf += 1;
				tempbuf = strchr(tempbuf, '\n');		
				tempbuf += 1;
				tempbuf = strchr(tempbuf, '\n');					
				if(tempbuf != NULL)
					 return tempbuf - file_buf +1;						

		}		  
	
	return 0;


}


int update_motor_pos_steps(int row, int col)
{
		struct file *filep = NULL;
		struct kstat stat;
		mm_segment_t fs;
		char *raw_fmts;
		int tdata_size = 0;
		int update;
		char *pos_buf;
		int i,error;
		char *tmpstr1, *tmpstr2, *tmpstr3, *tmpstr4,*tmpstr5,*tmpstr6, *tmpstr;
		char strbuf[5] = {0};
		int len, pos_steps,tmplen,j;
		char pos1[40], pos2[40], pos3[40], pos4[40], pos5[40];
		char *tmp;
		int temppos, pos, ret;
		char test[3]= "043";
		int buf[] = {'1','2','3','4','5','9'};
		char format_pos[4];

		
		set_fs(KERNEL_DS);
		fs = get_fs();
		filep = filp_open("/home/ant/examples/kernelmod/motor_pos_map", O_RDWR, 777);
		if (IS_ERR(filep)) {
			fpga_error(("Failed to open the motor_pos_map file	in %s", __FUNCTION__));
			goto fail;
		}
		error = vfs_stat("/home/ant/examples/kernelmod/motor_pos_map", &stat);
		if (error) {
			fpga_error(("Failed in %s to find file stat", __FUNCTION__));
			goto fail;
		}
		tdata_size = (int) stat.size;
	
		pos_buf = kzalloc(tdata_size , GFP_KERNEL);
		if (pos_buf == NULL) {
			fpga_error(("Failed to allocate pos_buf memory"));
			goto fail;
		}
		
		if (vfs_read(filep, pos_buf, tdata_size, &filep->f_pos) !=	tdata_size) {
			fpga_error(("Error: motor_pos_map file read failed"));
			goto fail;
		}

		//ret = kernel_write(filep, buf ,sizeof(buf), 0);
		printk("main_pre_steps[%d]=%d\n", row, main_pre_steps[row]);

		main_motor_pos_map[row][col] = main_pre_steps[row];
		
		sprintf(format_pos, "%04d",  main_motor_pos_map[row][col]);
		
		printk("update pos[%d][%d]  steps :%s\n", row, col, format_pos);
		temppos = get_pos_by_row(pos_buf, row);
		pos = temppos + col * 5;
		ret = kernel_write(filep, format_pos ,4, pos);
		if(ret < 0){
			printk("write error\n");
			goto fail;
		}

				

fail:
	if (pos_buf) {
		kfree(pos_buf);
		pos_buf = NULL;
	}
	if (!IS_ERR(filep))
		filp_close(filep, NULL);
	set_fs(fs);

#ifdef DUMP_POS_FILE
	load_motor_pos_steps();
#endif
	
	return 0;

}


int update_inj_pos_steps(int row, int col)
{
		struct file *filep = NULL;
		struct kstat stat;
		mm_segment_t fs;
		char *raw_fmts;
		int tdata_size = 0;
		int update;
		char *pos_buf;
		int i,error;
		char *tmpstr1, *tmpstr2, *tmpstr3, *tmpstr4,*tmpstr5,*tmpstr6, *tmpstr;
		char strbuf[5] = {0};
		int len, pos_steps,tmplen,j;
		char pos1[40], pos2[40], pos3[40], pos4[40], pos5[40];
		char *tmp;
		int temppos, pos, ret;
		char test[3]= "043";
		int buf[] = {'1','2','3','4','5','9'};
		char format_pos[4];

		
		set_fs(KERNEL_DS);
		fs = get_fs();
		filep = filp_open("/home/ant/examples/kernelmod/motor_pos_map", O_RDWR, 777);
		if (IS_ERR(filep)) {
			fpga_error(("Failed to open the motor_pos_map file	in %s", __FUNCTION__));
			goto fail;
		}
		error = vfs_stat("/home/ant/examples/kernelmod/motor_pos_map", &stat);
		if (error) {
			fpga_error(("Failed in %s to find file stat", __FUNCTION__));
			goto fail;
		}
		tdata_size = (int) stat.size;
	
		pos_buf = kzalloc(tdata_size , GFP_KERNEL);
		if (pos_buf == NULL) {
			fpga_error(("Failed to allocate pos_buf memory"));
			goto fail;
		}
		
		if (vfs_read(filep, pos_buf, tdata_size, &filep->f_pos) !=	tdata_size) {
			fpga_error(("Error: motor_pos_map file read failed"));
			goto fail;
		}

		//ret = kernel_write(filep, buf ,sizeof(buf), 0);

		inj_motor_init_pos[row] = inj_pre_steps[row] ;
		
		sprintf(format_pos, "%04d",  inj_motor_init_pos[row]);
		
		printk("update pos[%d][%d]  steps :%s\n", row, col, format_pos);
		temppos = get_pos_by_row(pos_buf, row+5);
		pos = temppos + col * 5;
		ret = kernel_write(filep, format_pos ,4, pos);
		if(ret < 0){
			printk("write error\n");
			goto fail;
		}

				

fail:
	if (pos_buf) {
		kfree(pos_buf);
		pos_buf = NULL;
	}
	if (!IS_ERR(filep))
		filp_close(filep, NULL);
	set_fs(fs);

#ifdef DUMP_POS_FILE
	load_motor_pos_steps();
#endif
	
	return 0;

}



int load_motor_pos_steps(void)
{
		struct file *filep = NULL;
		struct kstat stat;
		mm_segment_t fs;
		char *raw_fmts;
		int tdata_size = 0;
		int update;
		char *pos_buf;
		int i,error;
		char *tmpstr1, *tmpstr2, *tmpstr3, *tmpstr4,*tmpstr5,*tmpstr6, *tmpstr;
		char strbuf[5] = {0};
		int len, pos_steps,tmplen,j;
		char pos1[40], pos2[40], pos3[40], pos4[40], pos5[40],pos6[40];
		char *tmp;
		
		
		set_fs(KERNEL_DS);
		fs = get_fs();
		filep = filp_open("/home/ant/examples/kernelmod/motor_pos_map", O_RDONLY, 0);
		if (IS_ERR(filep)) {
			fpga_error(("Failed to open the motor_pos_map file	in %s", __FUNCTION__));
			goto fail;
		}
		error = vfs_stat("/home/ant/examples/kernelmod/motor_pos_map", &stat);
		if (error) {
			fpga_error(("Failed in %s to find file stat", __FUNCTION__));
			goto fail;
		}
		tdata_size = (int) stat.size;
	
		pos_buf = kzalloc(tdata_size+1 , GFP_KERNEL);
		if (pos_buf == NULL) {
			fpga_error(("Failed to allocate pos_buf memory"));
			goto fail;
		}
	
		if (vfs_read(filep, pos_buf, tdata_size, &filep->f_pos) !=	tdata_size) {
			fpga_error(("Error: motor_pos_map file read failed"));
			goto fail;
		}
	
		if (!IS_ERR(filep))
			filp_close(filep, NULL);
		
		set_fs(fs);
		
		pos_buf[tdata_size]='\0';

		printk("pos_buf:%s\n", pos_buf);
	
		tmpstr1 = strchr(pos_buf, '\n');
		if(tmpstr1 != NULL) {	
			len = tmpstr1 - pos_buf;
			*tmpstr1  = ',';
			strncpy(pos1, pos_buf,len + 1);
			
		}
	
		printk("motor sample_v pos param: ");
		tmp = pos1;
		for(i = 0, j = 0; i < len+1; j++ ) {
			tmpstr = strchr(tmp, ',');
			if(tmpstr != NULL){
				tmplen = tmpstr - tmp; 
				strncpy(strbuf, tmp, tmplen);			
				strbuf[tmplen] = '\0';
				pos_steps = simple_strtol(strbuf,NULL,10);
				main_motor_pos_map[FPGA_MAIN_MOTOR_SAMPLE_V][j] = pos_steps;
				printk("%d ",pos_steps);
				i += tmplen +1;
				tmp  = pos1 + i;
			}
		}
		
	
		printk("\n");
	
		tmpstr2 = strchr(tmpstr1+1, '\n');
		if(tmpstr2 != NULL) {
			len = tmpstr2 - (tmpstr1+1);
			*tmpstr2 = ',';
			strncpy(pos2, tmpstr1+1,len+1);
			
		}
	
		
		printk("motor sample_h pos param: ");
	
		tmp = pos2;
	
		for(i = 0, j=0; i < len+1; j++ ) {
			tmpstr = strchr(tmp, ',');
			if(tmpstr != NULL){
				tmplen = tmpstr - tmp; 
				strncpy(strbuf, tmp, tmplen);	
				strbuf[tmplen] = '\0';
				pos_steps = simple_strtol(strbuf,NULL,10);				
				main_motor_pos_map[FPGA_MAIN_MOTOR_SAMPLE_H][j] = pos_steps;
				printk("%d ",pos_steps);
				i += tmplen +1;
				tmp  = pos2 + i;
			}
		}
		printk("\n");
	
	
		tmpstr3 = strchr(tmpstr2+1, '\n');
		if(tmpstr3 != NULL) {
			len = tmpstr3 - (tmpstr2+1);
			*tmpstr3 = ',';
			strncpy(pos3, tmpstr2+1,len+1);
		}
	
	
		printk("motor mix_v pos param: ");
	
		tmp = pos3;
	
		for(i = 0, j=0; i < len+1; j++ ) {
			tmpstr = strchr(tmp, ',');
			if(tmpstr != NULL){
				tmplen = tmpstr - tmp; 
				strncpy(strbuf, tmp, tmplen);	
				strbuf[tmplen] = '\0';			
				pos_steps = simple_strtol(strbuf,NULL,10);
				main_motor_pos_map[FPGA_MAIN_MOTOR_MIX_V][j] = pos_steps;			
				printk("%d ",pos_steps);
				i += tmplen +1;
				tmp  = pos3 + i;
			}
		}
		printk("\n");
	
		tmpstr4 = strchr(tmpstr3+1, '\n');
		if(tmpstr4 != NULL) {
			len = tmpstr4 - (tmpstr3+1);
			*tmpstr4 = ',';
			strncpy(pos4, tmpstr3+1,len+1);
		}
	
		
		printk("motor mix_r pos param: ");
	
		tmp = pos4;
	
		for(i = 0, j=0; i < len+1; j++ ) {
			tmpstr = strchr(tmp, ',');
			if(tmpstr != NULL){
				tmplen = tmpstr - tmp; 
				strncpy(strbuf, tmp, tmplen);	
				strbuf[tmplen] = '\0';			
				pos_steps = simple_strtol(strbuf,NULL,10);		
				main_motor_pos_map[FPGA_MAIN_MOTOR_MIX_R][j] = pos_steps;						
				printk("%d ",pos_steps);
				i += tmplen +1;
				tmp  = pos4 + i;
			}
		}
		printk("\n");
	
		tmpstr5 = strchr(tmpstr4+1, '\n');
		if(tmpstr5 != NULL) {
			len = tmpstr5 - (tmpstr4+1);
			*tmpstr5 = ',';
			strncpy(pos5, tmpstr4+1,len+1);
		}
	
		printk("motor mix_r pos param: ");
		tmp = pos5;
	
		for(i = 0, j=0; i < len+1; j++ ) {
			tmpstr = strchr(tmp, ',');
			if(tmpstr != NULL){
				tmplen = tmpstr - tmp; 
				strncpy(strbuf, tmp, tmplen);
				strbuf[tmplen] = '\0';			
				pos_steps = simple_strtol(strbuf,NULL,10);	
				main_motor_pos_map[FPGA_MAIN_MOTOR_MIX_H][j] = pos_steps;			
				printk("%d ",pos_steps);
				i += tmplen +1;
				tmp  = pos5 + i;
			}
		}
		printk("\n");

		printk("inj motor init pos : param: ");
		tmpstr = tmpstr5+1;
		for(i = 0; i < 3; i++){
			strncpy(strbuf, tmpstr, 4);
			strbuf[4] = '\0';			
			pos_steps = simple_strtol(strbuf,NULL,10);	
			inj_motor_init_pos[i+1] = pos_steps;
			tmpstr += 5;
		}
	
		printk("inj_init_pos1:%d pos2: %d  pos3:%d\n", inj_motor_init_pos[1], inj_motor_init_pos[2],inj_motor_init_pos[3]);

		if (pos_buf) {
			kfree(pos_buf);
			pos_buf = NULL;
		}
	

fail:
	if (pos_buf) {
		kfree(pos_buf);
		pos_buf = NULL;
	}
	if (!IS_ERR(filep))
		filp_close(filep, NULL);
	set_fs(fs);
	
	return -1;

}


void report_exe_time(void)
{
# if 0
	struct timespec exe_time;
	struct rtc_time tm;
	long diff_time;
	static int i = 0;
	
	getnstimeofday(&exe_time);

	
	diff_time = ((exe_time.tv_sec*1000*1000 + exe_time.tv_nsec/1000 ) -  (tc_old.tv_sec*1000*1000 + tc_old.tv_nsec/1000 )) - delay_time*1000;

	timarry[i++] =diff_time;

	//fpga_info(("report_time :ts_delay:%d ->(nowtime:%lds .%ldus)-(pretime:%ld s.%ld us) ->interval_diff=%ldus\n", delay_time, exe_time.tv_sec, exe_time.tv_nsec/1000, tc_old.tv_sec, tc_old.tv_nsec/1000,	((exe_time.tv_sec*1000*1000 + exe_time.tv_nsec/1000 )	-  (tc_old.tv_sec*1000*1000 + tc_old.tv_nsec/1000 )) - delay_time*1000));
	//rtc_time_to_tm(exe_time.tv_sec, &tm);
	//fpga_info(( "%s: %4d-%02d-%02d %02d:%02d:%02d -> diff_time(%ld)\n", "readtime",
	//	1900 + tm.tm_year, tm.tm_mon, tm.tm_mday,
	//	tm.tm_hour, tm.tm_min, tm.tm_sec, diff_time));
#endif
}


void fpga_set_intsys_onoff(int on)
{
	u32 reg;
	u16 value;

	spin_lock(&g_fpga_dev->lock);
	
	//value = fpga_read_reg(g_fpga_dev,FPGA_INT0_MASK_SYS);

	if(on){
		value &= ~(INT_SOURCE_FIFO_EMPTY_BIT | INT_SOURCE_FIFO_FULL_BIT| INT_SOURCE_FIFO_HALFFULL_BIT);
	}else
		value |= (INT_SOURCE_FIFO_EMPTY_BIT | INT_SOURCE_FIFO_FULL_BIT| INT_SOURCE_FIFO_HALFFULL_BIT);

	//fpga_write_reg(g_fpga_dev,FPGA_INT0_MASK_SYS,value);
	spin_unlock(&g_fpga_dev->lock);

}


void fpga_set_intrbc_onoff(int on)
{
	u32 reg;
	u16 value;

	spin_lock(&g_fpga_dev->lock);
	
	value = fpga_read_reg(g_fpga_dev,FPGA_INT1_MASK_RBC);

	if(on){
		value &= ~(INT_SOURCE_FIFO_EMPTY_BIT | INT_SOURCE_FIFO_FULL_BIT| INT_SOURCE_FIFO_HALFFULL_BIT);
	}else
		value |= (INT_SOURCE_FIFO_EMPTY_BIT | INT_SOURCE_FIFO_FULL_BIT| INT_SOURCE_FIFO_HALFFULL_BIT);

	fpga_write_reg(g_fpga_dev,FPGA_INT1_MASK_RBC,value);
	spin_unlock(&g_fpga_dev->lock);

}

void fpga_set_intwbc_onoff(int on)
{
	u32 reg;
	u16 value;
	
	spin_lock(&g_fpga_dev->lock);
	
	value = fpga_read_reg(g_fpga_dev,FPGA_INT2_MASK_WBC);

	if(on){
		value &= ~(INT_SOURCE_FIFO_EMPTY_BIT | INT_SOURCE_FIFO_FULL_BIT| INT_SOURCE_FIFO_HALFFULL_BIT);
	}else
		value |= (INT_SOURCE_FIFO_EMPTY_BIT | INT_SOURCE_FIFO_FULL_BIT| INT_SOURCE_FIFO_HALFFULL_BIT);

	fpga_write_reg(g_fpga_dev,FPGA_INT2_MASK_WBC,value);
	spin_unlock(&g_fpga_dev->lock);

}

void fpga_set_intopt_onoff(int on)
{
	u32 reg;
	u16 value;

	spin_lock(&g_fpga_dev->lock);
	
	value = fpga_read_reg(g_fpga_dev,FPGA_INT3_MASK_OPT);

	if(on){
		value &= ~(INT_SOURCE_FIFO_EMPTY_BIT | INT_SOURCE_FIFO_FULL_BIT| INT_SOURCE_FIFO_HALFFULL_BIT);
	}else
		value |= (INT_SOURCE_FIFO_EMPTY_BIT | INT_SOURCE_FIFO_FULL_BIT| INT_SOURCE_FIFO_HALFFULL_BIT);

	fpga_write_reg(g_fpga_dev,FPGA_INT3_MASK_OPT,value);
	spin_unlock(&g_fpga_dev->lock);

}


void fpga_set_intswitch_onoff(int on)
{
	u32 reg;
	u16 value;
	
	spin_lock(&g_fpga_dev->lock);
	
	value = fpga_read_reg(g_fpga_dev,FPGA_INT4_MASK_SWITCH);

	if(on){
		value &= ~(INT_SOURCE_FIFO_EMPTY_BIT | INT_SOURCE_FIFO_FULL_BIT| INT_SOURCE_FIFO_HALFFULL_BIT);
	}else
		value |= (INT_SOURCE_FIFO_EMPTY_BIT | INT_SOURCE_FIFO_FULL_BIT| INT_SOURCE_FIFO_HALFFULL_BIT);

	fpga_write_reg(g_fpga_dev,FPGA_INT4_MASK_SWITCH,value);
	
	spin_unlock(&g_fpga_dev->lock);
}


void fpga_set_intmotor_onoff(int on, int dev_no) //dev_no: 0~7
{
	u32 reg;
	u16 value;

	spin_lock(&g_fpga_dev->lock);
	value = fpga_read_reg(g_fpga_dev,FPGA_INT5_MASK_MOTOR);

	if(on){
		value &= ~(0x1 << dev_no);
	}else
		value |= (0x1 << dev_no);

	fpga_write_reg(g_fpga_dev,FPGA_INT5_MASK_MOTOR,value);
	
	spin_unlock(&g_fpga_dev->lock);

}


void set_motor_init_accspeed(void)
{
	fpga_write_reg(g_fpga_dev,FPGA_SAMPH_MOTOR_BASE+0x22,0);
	fpga_write_reg(g_fpga_dev,FPGA_SAMPV_MOTOR_BASE+0x22,0);
	fpga_write_reg(g_fpga_dev,FPGA_MIXH_MOTOR_BASE+0x22,0);
	fpga_write_reg(g_fpga_dev,FPGA_MIXV_MOTOR_BASE+0x22,0);
	fpga_write_reg(g_fpga_dev,FPGA_MIXR_MOTOR_BASE+0x22,0);
	fpga_write_reg(g_fpga_dev,FPGA_INJ1_MOTOR_BASE+0x22,0);
	fpga_write_reg(g_fpga_dev,FPGA_INJ2_MOTOR_BASE+0x22,0);
	fpga_write_reg(g_fpga_dev,FPGA_INJ3_MOTOR_BASE+0x22,0);

	return;
}

int get_main_motor_pos_speed(int dev_no, int pos)
{
	if(pos<0 || pos > 11) {
		fpga_error(("%s: pos invalid", __func__));
		return -1;
	}
		
	
	return main_motor_pos_map[dev_no][pos];
	
#if 0
	
	switch(dev_no) {
		case FPGA_MAIN_MOTOR_SAMPLE_H: 
			switch(pos) {
			case  0:
				pos_steps = 300;
				break;
			case  1:
				pos_steps = 350;
				break;
			case  2:
				pos_steps = 400;
				break;
			case  3:
				pos_steps = 450;
				break;					
			case  4:
				pos_steps = 500;
				break;
			case  5:
				pos_steps = 550;
				break;
			case  6:
				pos_steps = 600;
				break;	
			}
			
			break;
		case FPGA_MAIN_MOTOR_SAMPLE_V:
			switch(pos) {
				case  0:
					pos_steps = 300;
					break;
				case  1:
					pos_steps = 350;
					break;
				case  2:
					pos_steps = 400;
					break;
				case  3:
					pos_steps = 450;
					break;					
				case  4:
					pos_steps = 500;
					break;
				case  5:
					pos_steps = 550;
					break;
			}
			
			break;
		case FPGA_MAIN_MOTOR_MIX_H:
			
			switch(pos) {
				case  0:
					pos_steps = 300;
					break;
				case  1:
					pos_steps = 350;
					break;
				case  2:
					pos_steps = 400;
					break;
				}
			break;
		case FPGA_MAIN_MOTOR_MIX_V:
			switch(pos) {
				case  0:
					pos_steps = 300;
					break;
				case  1:
					pos_steps = 350;
					break;
				case  2:
					pos_steps = 400;
					break;
				}
			break;
		case FPGA_MAIN_MOTOR_MIX_R:
			switch(pos) {
				case  0:
					pos_steps = 300;
					break;
				case  1:
					pos_steps = 350;
					break;
				case  2:
					pos_steps = 400;
					break;
				}
			break;			
		}
#endif
		
}


int get_inj_motor_pos_speed(int dev_no, int pos)
{
	if(pos<0 || pos > 3) {
		fpga_error(("%s: pos invalid", __func__));
		return -1;
	}
		
	
	return inj_motor_init_pos[dev_no];
	
}


int comm_main_motor_set(int dev_no, int speed, int pos)
{
	int motor_hold, index_start, ignor_step, chg_point, chg_speed, dir, pos_steps, temp_steps, tmp_speed; 
	int regbase;

	if(dev_no < FPGA_MAIN_MOTOR_SAMPLE_V || dev_no > FPGA_MAIN_MOTOR_MIX_H){
		fpga_error(("%s: dev_no invalid", __func__));
		return -1;
	}
	
	switch(dev_no) {
		case FPGA_MAIN_MOTOR_SAMPLE_V:
			regbase = FPGA_SAMPV_MOTOR_BASE;
			break;		
		case FPGA_MAIN_MOTOR_SAMPLE_H: 
			regbase = FPGA_SAMPH_MOTOR_BASE;
			break;
		case FPGA_MAIN_MOTOR_MIX_V:
			regbase = FPGA_MIXV_MOTOR_BASE;
			break;	
		case FPGA_MAIN_MOTOR_MIX_R:
			regbase = FPGA_MIXR_MOTOR_BASE;
			break;				
		case FPGA_MAIN_MOTOR_MIX_H:
			regbase = FPGA_MIXH_MOTOR_BASE;
			break;
		}


	pos_steps = get_main_motor_pos_speed(dev_no, pos);

	temp_steps = pos_steps - main_pre_steps[dev_no];

		//int motor:
	if(speed == 0){
		fpga_info(("!!!!speed is zero!!!!"));
		return;
	}
	
	tmp_speed = 520 - 11250/speed;

	fpga_info(("%s: speed:%d  real_speed:%d pos:%d    pos_steps:%d  pre_pos_step:%d  real_steps:%d\n", __func__, speed, tmp_speed, pos, pos_steps, main_pre_steps[dev_no],  temp_steps));

	
	if(temp_steps > 0)
		dir = 0;
	else {
		temp_steps = abs(temp_steps);
		dir = 1;
	}
	
	motor_hold = 0x2;
	index_start = tmp_speed/4;
	chg_point = temp_steps/8;
	chg_speed =  tmp_speed/8;


	//set step max: 0x0
	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_STEP_MAX, temp_steps);
	//set step add 0x2
	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_HOLD, motor_hold);
	//set step add 0x6
	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_DIR, dir);

	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_PHOTO_NUM, 9);
	
	//fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_ACC_INTER, 0x1);

	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_INDEX_START, index_start);

	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_SPEED, 15);


	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_MICROSTEP, 0x1);


	//fpga_write_reg(g_fpga_dev, reg+FPGA_MOTOR_IGNORE, ignor_step);

	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_MODE, 0x1);


// set dec motor speed:
	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_DEC_INTER, 0x1);

	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_CHANGE_POINT, chg_point );

	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_CHANGE_SPEED, 15 );
	
	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_SPD_TAB15, tmp_speed );

//enable motor:
	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_RUN, 0x1);

	//main_devno = dev_no;

	main_pre_steps[dev_no] = pos_steps;
	main_curr_steps[dev_no] = pos;
	main_steps[dev_no] = temp_steps;
	main_speed[dev_no] = tmp_speed;

	return 0;

}


void comm_motor_init(int dev_no, int speed)
{
	int regbase, chg_speed, tmp_speed; 
	int init_pos_step;

	switch(dev_no) {
		case FPGA_MAIN_MOTOR_SAMPLE_V:
			regbase = FPGA_SAMPV_MOTOR_BASE;
			break;		
		case FPGA_MAIN_MOTOR_SAMPLE_H: 
			regbase = FPGA_SAMPH_MOTOR_BASE;
			break;
		case FPGA_MAIN_MOTOR_MIX_V:
			regbase = FPGA_MIXV_MOTOR_BASE;
			break;	
		case FPGA_MAIN_MOTOR_MIX_R:
			regbase = FPGA_MIXR_MOTOR_BASE;
			break;				
		case FPGA_MAIN_MOTOR_MIX_H:
			regbase = FPGA_MIXH_MOTOR_BASE;
			break;
		}

	//int motor:
	if(speed == 0){
		fpga_info(("!!!!speed is zero!!!!"));
		return;
	}
	tmp_speed = 520 - 11250/speed;

	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_INIT_SPEED, tmp_speed);
	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_INIT_ENABLE, 0x1);

	main_current_pos = 0;

	
	init_pos_step = get_main_motor_pos_speed(dev_no, 0);

	main_pre_steps[dev_no] = init_pos_step;  //for eng maintaince

	fpga_info(("%s:main_pre_steps[%d]=%d", __func__, dev_no, main_pre_steps[dev_no] ));


}

void comm_inj_motor_set( int dev_no, int speed, int steps)
{
	int motor_hold, index_start, ignor_step, chg_point, chg_speed, dir, temp_steps, tmp_speed; 
	int regbase;
	static int pre_pos;
	static int pr_steps, next_steps;
	//int abs_steps;

		//do some  motor regs setting;
	switch(dev_no) {
		case FPGA_INJ1_MOTOR: 
			regbase = FPGA_INJ1_MOTOR_BASE;
			break;
		case FPGA_INJ2_MOTOR:
			regbase = FPGA_INJ2_MOTOR_BASE;
			break;
		case FPGA_INJ3_MOTOR:
			regbase = FPGA_INJ3_MOTOR_BASE;
			break;
		}

	inj_pre_steps[dev_no] = steps + abs_steps[dev_no];
	 abs_steps[dev_no] =inj_pre_steps[dev_no];

	if(steps == 0) { // reset steps;
		steps =  -abs_steps[dev_no];
		abs_steps[dev_no] = 0;
		fpga_info(("steps == 0"));
	}

	if(speed == 0) {	
		fpga_info(("speed == 0"));
		return 0;
	}
	
	tmp_speed = 520 - 11250/speed;
	
	fpga_info(("comm_inj_motor_set:regbase:0x%x speed:%d steps:%d  abs_steps:%d\n",regbase, tmp_speed, steps,  abs_steps[dev_no]));

	motor_hold = 0x2;
	index_start = tmp_speed/4;
	chg_point = steps/8;
	chg_speed =  tmp_speed/8;

	if(steps > 0)
		dir = 1;
	else {
		steps = abs(steps);
		dir = 0;
	}
		

	//set step max: 0x0
	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_STEP_MAX, steps);
	//set step add 0x2
	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_HOLD, motor_hold);
	//set step add 0x6
	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_DIR, dir);

	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_PHOTO_NUM, 9);
	
	//fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_ACC_INTER, 0x1);

	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_INDEX_START, index_start);

	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_SPEED, 15);


	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_MICROSTEP, 0x1);


	//fpga_write_reg(g_fpga_dev, reg+FPGA_MOTOR_IGNORE, ignor_step);

	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_MODE, 0x1);


// set dec motor speed:
	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_DEC_INTER, 0x1);

	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_CHANGE_POINT, chg_point );

	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_CHANGE_SPEED, 15 );
	
	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_SPD_TAB15, tmp_speed );

//enable motor:
	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_RUN, 0x1);

	inj_steps[dev_no] = steps;
	inj_speed[dev_no] = tmp_speed;
	

}

void comm_inj_motor_init(int dev_no, int speed)
{
	int regbase, chg_speed, tmp_speed; 	
	int init_pos_step;

	fpga_info(("%s:dev_no:%d speed=%d  ",dev_no, speed));

	switch(dev_no) {
		case FPGA_INJ1_MOTOR: 
			regbase = FPGA_INJ1_MOTOR_BASE;
			break;
		case FPGA_INJ2_MOTOR:
			regbase = FPGA_INJ2_MOTOR_BASE;
			break;
		case FPGA_INJ3_MOTOR:
			regbase = FPGA_INJ3_MOTOR_BASE;
			break;
		}
	//int motor:
	if(speed == 0) {	
		fpga_info(("speed == 0"));
		return 0;
	}
	tmp_speed = 520 - 11250/speed;
	
	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_INIT_SPEED, tmp_speed);
	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_INIT_ENABLE, 0x1);

	inj_current_pos = 0; //for eng matain use;
	init_pos_step = get_inj_motor_pos_speed(dev_no, 0);

	inj_pre_steps[dev_no] = init_pos_step;  //for eng maintaince

	abs_steps[dev_no]=0;
	
}


static enum hrtimer_restart m_inj1_timer_handler(struct hrtimer *hrtimer)
{
	struct fpga_dev *pdev = container_of(hrtimer, struct fpga_dev, m_inj1_timer);

	fpga_info(("inject_motor_timer_handler"));
	
	ktime_t expires,now,diff;


	expires = hrtimer_get_expires(hrtimer);
	now = hrtimer_cb_get_time(hrtimer);
	diff = ktime_sub(now, expires);



	fpga_info(("timediff:%ul",	ktime_to_ns(diff)));
	
	if(atomic_read(&pdev->m_inj1_t) == 0){ //overtime , send warnning  to user;
		fpga_info(("m_inj1 set failed,send warning to user......"));
	}else{
		atomic_set(&pdev->m_inj1_t,0);
		fpga_info(("m_inj1 set success"));
	}		

	//disable motor interrupt
	fpga_set_intmotor_onoff(0, INT_SOURCE_MOTOR_INJ1);

	return HRTIMER_NORESTART;
}

static enum hrtimer_restart m_inj2_timer_handler(struct hrtimer *hrtimer)
{
	struct fpga_dev *pdev = container_of(hrtimer, struct fpga_dev, m_inj2_timer);

	fpga_info(("inject_motor_timer_handler"));
	
	
	if(atomic_read(&pdev->m_inj2_t) == 0){ //overtime , send warnning  to user;
		fpga_info(("m_inj2 set failed,send warning to user......"));
	}else{
		atomic_set(&pdev->m_inj2_t,0);
		fpga_info(("m_inj2 set success"));
	}	

	//disable motor interrupt
	fpga_set_intmotor_onoff(0, INT_SOURCE_MOTOR_INJ2);
	return HRTIMER_NORESTART;
}


static enum hrtimer_restart m_inj3_timer_handler(struct hrtimer *hrtimer)
{
	struct fpga_dev *pdev = container_of(hrtimer, struct fpga_dev, m_inj3_timer);

	fpga_info(("inject_motor_timer_handler"));
	
	if(atomic_read(&pdev->m_inj3_t) == 0){ //overtime , send warnning  to user;
		fpga_info(("m_inj3 set failed,send warning to user......"));
	}else{
		atomic_set(&pdev->m_inj3_t,0);
		fpga_info(("m_inj3 set success"));
	}

	//disable motor interrupt
	fpga_set_intmotor_onoff(0, INT_SOURCE_MOTOR_INJ3);

	return HRTIMER_NORESTART;
}


static enum hrtimer_restart m_samph_timer_handler(struct hrtimer *hrtimer)
{
	struct fpga_dev *pdev = container_of(hrtimer, struct fpga_dev, m_samph_timer);

	fpga_info(("m_samph_timer_handler"));
	
		
	if(atomic_read(&pdev->m_samph_t) == 0){ //overtime , send warnning  to user;
		fpga_info(("m_samph set failed,send warning to user......"));
	}else{
		atomic_set(&pdev->m_samph_t,0);
		fpga_info(("m_samph set success"));
	}

	//disable motor interrupt
	fpga_set_intmotor_onoff(0, INT_SOURCE_MOTOR_SAMPH);	
	return HRTIMER_NORESTART;
}



static enum hrtimer_restart m_sampv_timer_handler(struct hrtimer *hrtimer)
{
	struct fpga_dev *pdev = container_of(hrtimer, struct fpga_dev, m_sampv_timer);

	fpga_info(("m_sampv_timer_handler"));
	
		
	if(atomic_read(&pdev->m_sampv_t) == 0){ //overtime , send warnning  to user;
		fpga_info(("m_sampv set failed,send warning to user......"));
	}else{
		atomic_set(&pdev->m_samph_t,0);
		fpga_info(("m_sampv set success"));
	}
	//disable motor interrupt
	fpga_set_intmotor_onoff(0, INT_SOURCE_MOTOR_SAMPV);	
	
	return HRTIMER_NORESTART;
}



static enum hrtimer_restart m_mixh_timer_handler(struct hrtimer *hrtimer)
{
	struct fpga_dev *pdev = container_of(hrtimer, struct fpga_dev, m_mixh_timer);


	ktime_t expires,now,diff;

	fpga_info(("m_mixh_timer_handler"));
	
	expires = hrtimer_get_expires(hrtimer);
	now = hrtimer_cb_get_time(hrtimer);
	diff = ktime_sub(now, expires);
	
	fpga_info(("timediff:%ul",	ktime_to_us(diff)));
	
	if(atomic_read(&pdev->m_mixh_t) == 0){ //overtime , send warnning  to user;
		fpga_info(("m_mixh set failed,send warning to user......"));
	}else{
		atomic_set(&pdev->m_mixh_t,0);
		fpga_info(("m_mixh set success"));
	}	
	//disable motor interrupt
	fpga_set_intmotor_onoff(0, INT_SOURCE_MOTOR_MIXH);	
	
	return HRTIMER_NORESTART;
}


static enum hrtimer_restart m_mixv_timer_handler(struct hrtimer *hrtimer)
{
	struct fpga_dev *pdev = container_of(hrtimer, struct fpga_dev, m_mixv_timer);

	fpga_info(("m_mixv_timer_handler"));
	
		
	if(atomic_read(&pdev->m_mixv_t) == 0){ //overtime , send warnning  to user;
		fpga_info(("m_mixv set failed,send warning to user......"));
	}else{
		atomic_set(&pdev->m_mixv_t,0);
		fpga_info(("m_mixv set success"));
	}
	//disable motor interrupt
	fpga_set_intmotor_onoff(0, INT_SOURCE_MOTOR_MIXV);	
	
	return HRTIMER_NORESTART;
}


static enum hrtimer_restart m_mixr_timer_handler(struct hrtimer *hrtimer)
{
	struct fpga_dev *pdev = container_of(hrtimer, struct fpga_dev, m_mixr_timer);

	fpga_info(("m_mixr_timer_handler"));
	
		
	if(atomic_read(&pdev->m_mixr_t) == 0){ //overtime , send warnning  to user;
		fpga_info(("m_mixr set failed,send warning to user......"));
	}else{
		atomic_set(&pdev->m_mixr_t,0);
		fpga_info(("m_mixr set success"));
	}
		//disable motor interrupt
	fpga_set_intmotor_onoff(0, INT_SOURCE_MOTOR_MIXR);	
	return HRTIMER_NORESTART;
}


static enum hrtimer_restart sys_hgb_timer_handler(struct hrtimer *hrtimer)
{
	struct fpga_dev *pdev = container_of(hrtimer, struct fpga_dev, sys_hgb_timer);

	fpga_info(("sys_hgb_timer_handler"));
	
		//disable motor interrupt
	fpga_set_intsys_onoff(0);

	//set atomic to avoid report interrupt data to user again and again;
	atomic_set(&g_fpga_dev->sys_hgb_t,0);

	return HRTIMER_NORESTART;
}

static enum hrtimer_restart sys_rbc_timer_handler(struct hrtimer *hrtimer)
{
	struct fpga_dev *pdev = container_of(hrtimer, struct fpga_dev, sys_rbc_timer);

	fpga_info(("sys_rbc_timer_handler"));

	//disable int sys interrupt
	fpga_set_intsys_onoff(0);	

	atomic_set(&g_fpga_dev->sys_rbc_t,0);

	return HRTIMER_NORESTART;
}

static enum hrtimer_restart sys_wbc_timer_handler(struct hrtimer *hrtimer)
{
	struct fpga_dev *pdev = container_of(hrtimer, struct fpga_dev, sys_wbc_timer);

	fpga_info(("sys_rbc_timer_handler"));
	
		//disable motor interrupt
	fpga_set_intsys_onoff(0);	

	atomic_set(&g_fpga_dev->sys_wbc_t,0);
		
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart sys_liqpress_timer_handler(struct hrtimer *hrtimer)
{
	struct fpga_dev *pdev = container_of(hrtimer, struct fpga_dev, sys_liqpress_timer);

	fpga_info(("sys_liqpress_timer_handler"));
	
		//disable motor interrupt
	fpga_set_intsys_onoff(0);	

	atomic_set(&g_fpga_dev->sys_liqpress_t,0);
		
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart sys_airpress_timer_handler(struct hrtimer *hrtimer)
{
	struct fpga_dev *pdev = container_of(hrtimer, struct fpga_dev, sys_airpress_timer);

	fpga_info(("sys_airpress_timer_handler"));
	
		//disable motor interrupt
	fpga_set_intsys_onoff(0);	

	atomic_set(&g_fpga_dev->sys_airpress_t,0);
		
	return HRTIMER_NORESTART;
}


static enum hrtimer_restart sys_liq1_timer_handler(struct hrtimer *hrtimer)
{
	struct fpga_dev *pdev = container_of(hrtimer, struct fpga_dev, sys_liq1_timer);

	fpga_info(("sys_liq1_timer_handler"));
	
		//disable motor interrupt
	fpga_set_intsys_onoff(0);

	atomic_set(&g_fpga_dev->sys_liq1_t,0);
		
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart sys_pmt_timer_handler(struct hrtimer *hrtimer)
{
	struct fpga_dev *pdev = container_of(hrtimer, struct fpga_dev, sys_pmt_timer);

	fpga_info(("sys_pmt_timer_handler"));
	
		//disable motor interrupt
	fpga_set_intsys_onoff(0);	

	atomic_set(&g_fpga_dev->sys_pmt_t,0);	
	
	return HRTIMER_NORESTART;
}


static enum hrtimer_restart sys_ld_timer_handler(struct hrtimer *hrtimer)
{
	struct fpga_dev *pdev = container_of(hrtimer, struct fpga_dev, sys_ld_timer);

	fpga_info(("sys_ld_timer_handler"));
	
		//disable motor interrupt
	fpga_set_intsys_onoff(0);	

	atomic_set(&g_fpga_dev->sys_ld_t,0);	
	
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart sys_temp1_timer_handler(struct hrtimer *hrtimer)
{
	struct fpga_dev *pdev = container_of(hrtimer, struct fpga_dev, sys_temp1_timer);

	fpga_info(("sys_temp1_timer_handler"));
	
		//disable motor interrupt
	fpga_set_intsys_onoff(0);	

	atomic_set(&g_fpga_dev->sys_temp1_t,0);	
		
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart sys_temp2_timer_handler(struct hrtimer *hrtimer)
{
	struct fpga_dev *pdev = container_of(hrtimer, struct fpga_dev, sys_temp2_timer);

	fpga_info(("sys_temp2_timer_handler"));
	
		//disable motor interrupt
	fpga_set_intsys_onoff(0);	

	atomic_set(&g_fpga_dev->sys_temp2_t,0);	
		
	return HRTIMER_NORESTART;
}


extern struct timespec tm1, tm2, tm3, tm4,tm5,tm6;
extern int lenarr[10], timearr[10], difftm1, difftm2, intcnt;

static enum hrtimer_restart fifo_rbc_timer_handler(struct hrtimer *hrtimer)
{
	struct fpga_dev *pdev = container_of(hrtimer, struct fpga_dev, fifo_rbc_timer);

	fpga_info(("\n%s: rbc sample end\n",  __func__));
	int i;

#if 0
	for(i=0;i<6;i++)
		printk("%d ", lenarr[i]);
	printk("\n");

	printk(" dma time diff && work_sch\n");

	for(i=0; i< 8; i++)
		printk("%d ", timearr[i]);
	
	printk("\n");
	
	printk(" dma time diff =%ld work_sch:%ld us\n",  ((tm3.tv_sec*1000*1000 + tm3.tv_nsec/1000 ) -	(tm1.tv_sec*1000*1000 + tm1.tv_nsec/1000 )), ((tm2.tv_sec*1000*1000 + tm2.tv_nsec/1000 ) -	(tm1.tv_sec*1000*1000 + tm1.tv_nsec/1000 )));

#endif

		//disable rbc sample
	//fpga_write_reg(g_fpga_dev, FPGA_FIFO_RBC_START, 0);

	//fpga_disable_irq(g_fpga_dev); //diable irq ,so we can do not use spin_lock to enable preempt;
	//pdev->pmri->m_unique_key = 536870931;
	atomic_set(&g_fpga_dev->fifo_rbc_t,0);

		
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart fifo_wbc_timer_handler(struct hrtimer *hrtimer)
{
	struct fpga_dev *pdev = container_of(hrtimer, struct fpga_dev, fifo_wbc_timer);

	fpga_info(("fifo_wbc_timer_handler"));
	
		//disable motor interrupt
	//fpga_set_intwbc_onoff(0);	

	atomic_set(&g_fpga_dev->fifo_wbc_t,0);	
		
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart fifo_diff_timer_handler(struct hrtimer *hrtimer)
{
	struct fpga_dev *pdev = container_of(hrtimer, struct fpga_dev, fifo_rbc_timer);

	fpga_info(("fifo_diff_timer_handler"));
	
		//disable motor interrupt
	//fpga_set_intopt_onoff(0);	

	atomic_set(&g_fpga_dev->fifo_diff_t,0);	
		
	return HRTIMER_NORESTART;
}


/********************************************timer handler*************************************/



/***********************************************timer init**************************************/
void m_inj1_timer_init(struct fpga_dev *pdev)
{
	hrtimer_init(&pdev->m_inj1_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pdev->m_inj1_timer.function = m_inj1_timer_handler;
}

void m_inj2_timer_init(struct fpga_dev *pdev)
{
	hrtimer_init(&pdev->m_inj2_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pdev->m_inj2_timer.function = m_inj2_timer_handler;
	atomic_set(&pdev->m_inj2_t, 0); //this flag need set in  motor interrupt handler;
}

void m_inj3_timer_init(struct fpga_dev *pdev)
{
	hrtimer_init(&pdev->m_inj3_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pdev->m_inj3_timer.function = m_inj3_timer_handler;
	atomic_set(&pdev->m_inj3_t, 0); //this flag need set in  motor interrupt handler;
}



void m_samph_timer_init(struct fpga_dev *pdev)
{
	hrtimer_init(&pdev->m_samph_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pdev->m_samph_timer.function = m_samph_timer_handler;
}

void m_sampv_timer_init(struct fpga_dev *pdev)
{
	hrtimer_init(&pdev->m_sampv_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pdev->m_sampv_timer.function = m_sampv_timer_handler;
}

void m_mixh_timer_init(struct fpga_dev *pdev)
{
	hrtimer_init(&pdev->m_mixh_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pdev->m_mixh_timer.function = m_mixh_timer_handler;
	atomic_set(&pdev->m_mixh_t, 0); //this flag need set in  motor interrupt handler;
}

void m_mixv_timer_init(struct fpga_dev *pdev)
{
	hrtimer_init(&pdev->m_mixv_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pdev->m_mixv_timer.function = m_mixv_timer_handler;
	atomic_set(&pdev->m_mixv_t, 0); //this flag need set in  motor interrupt handler;
}

void m_mixr_timer_init(struct fpga_dev *pdev)
{
	hrtimer_init(&pdev->m_mixr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pdev->m_mixr_timer.function = m_mixr_timer_handler;
	atomic_set(&pdev->m_mixr_t, 0); //this flag need set in  motor interrupt handler;
}

void sys_hgb_timer_init(struct fpga_dev *pdev)
{
	hrtimer_init(&pdev->sys_hgb_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pdev->sys_hgb_timer.function = sys_hgb_timer_handler;

	//set sys_xxx_t to 0 ,avoid all sys interrupt will report to user ; interrupt sys report to user should check this flag 
	atomic_set(&g_fpga_dev->sys_hgb_t,0);
}


void sys_rbc_timer_init(struct fpga_dev *pdev)
{
	hrtimer_init(&pdev->sys_rbc_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pdev->sys_rbc_timer.function = sys_rbc_timer_handler;
	
	//set sys_xxx_t to 0 ,avoid all sys interrupt will report to user ; interrupt sys report to user should check this flag 
	atomic_set(&g_fpga_dev->sys_rbc_t,0);
}


void sys_wbc_timer_init(struct fpga_dev *pdev)
{
	hrtimer_init(&pdev->sys_wbc_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pdev->sys_wbc_timer.function = sys_wbc_timer_handler;
	
	atomic_set(&g_fpga_dev->sys_wbc_t,0);
}

void sys_liqpress_timer_init(struct fpga_dev *pdev)
{
	hrtimer_init(&pdev->sys_liqpress_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pdev->sys_liqpress_timer.function = sys_liqpress_timer_handler;
	
	atomic_set(&g_fpga_dev->sys_liqpress_t,0);
}



void sys_airpress_timer_init(struct fpga_dev *pdev)
{
	hrtimer_init(&pdev->sys_airpress_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pdev->sys_airpress_timer.function = sys_airpress_timer_handler;
	
	atomic_set(&g_fpga_dev->sys_airpress_t,0);
}


void sys_liq1_timer_init(struct fpga_dev *pdev)
{
	hrtimer_init(&pdev->sys_liq1_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pdev->sys_liq1_timer.function = sys_liq1_timer_handler;
	
	atomic_set(&g_fpga_dev->sys_liq1_t,0);
}

void sys_pmt_timer_init(struct fpga_dev *pdev)
{
	hrtimer_init(&pdev->sys_pmt_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pdev->sys_pmt_timer.function = sys_pmt_timer_handler;
	
	atomic_set(&g_fpga_dev->sys_pmt_t,0);
}

void sys_ld_timer_init(struct fpga_dev *pdev)
{
	hrtimer_init(&pdev->sys_ld_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pdev->sys_ld_timer.function = sys_ld_timer_handler;
	
	atomic_set(&g_fpga_dev->sys_ld_t,0);
}

void sys_temp1_timer_init(struct fpga_dev *pdev)
{
	hrtimer_init(&pdev->sys_temp1_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pdev->sys_temp1_timer.function = sys_temp1_timer_handler;
	atomic_set(&g_fpga_dev->sys_temp1_t,0);
}


void sys_temp2_timer_init(struct fpga_dev *pdev)
{
	hrtimer_init(&pdev->sys_temp2_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pdev->sys_temp2_timer.function = sys_temp2_timer_handler;
	
	atomic_set(&g_fpga_dev->sys_temp2_t,0);
}

//rbc out chan;
void fifo_rbc_timer_init(struct fpga_dev *pdev)
{
	hrtimer_init(&pdev->fifo_rbc_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pdev->fifo_rbc_timer.function = fifo_rbc_timer_handler;
	
	atomic_set(&g_fpga_dev->fifo_rbc_t,0);
}


void fifo_wbc_timer_init(struct fpga_dev *pdev)
{
	hrtimer_init(&pdev->fifo_wbc_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pdev->fifo_wbc_timer.function = fifo_wbc_timer_handler;
	
	atomic_set(&g_fpga_dev->fifo_wbc_t,0);
}


void fifo_diff_timer_init(struct fpga_dev *pdev)
{
	hrtimer_init(&pdev->fifo_diff_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pdev->fifo_diff_timer.function = fifo_diff_timer_handler;
	
	atomic_set(&g_fpga_dev->fifo_diff_t,0);
}


void fpga_timer_init(struct fpga_dev *pdev)
{
	int sys_gcnt_flag;
	
	m_inj1_timer_init(pdev);
	m_inj2_timer_init(pdev);
	m_inj3_timer_init(pdev);
	m_samph_timer_init(pdev);
	m_sampv_timer_init(pdev);
	m_mixh_timer_init(pdev);
	m_mixv_timer_init(pdev);
	m_mixr_timer_init(pdev);

	sys_hgb_timer_init(pdev);
	sys_rbc_timer_init(pdev);
	sys_wbc_timer_init(pdev);
	sys_liqpress_timer_init(pdev);
	sys_airpress_timer_init(pdev);
	sys_liq1_timer_init(pdev);
	sys_pmt_timer_init(pdev);
	sys_ld_timer_init(pdev);
	sys_temp1_timer_init(pdev);
	sys_temp2_timer_init(pdev);

	fifo_rbc_timer_init(pdev);
	fifo_wbc_timer_init(pdev);
	fifo_diff_timer_init(pdev);

	sys_gcnt_flag = fpga_read_reg(pdev,FPGA_SYS_GLOBAL_CNT_FLAG);

	sys_gcnt_flag |= 0x1;
	fpga_write_reg(pdev,FPGA_SYS_GLOBAL_CNT_FLAG,sys_gcnt_flag); //enable fifo data sample;
	
	fpga_info(("FPGA timer init success"));
}


void fpga_set_valve(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4,struct measure_related_info * mri)
{
	u32 reg;
	u16 val;
	
	fpga_info(("fpga_set_valve : dev_no :%d action :%d \n", dev_no, action));

	if( dev_no >= MIN_VALVE_A && dev_no <=MAX_VALVE_A)
		reg =FPGA_VALVE_A;
	else if( dev_no >= MIN_VALVE_B && dev_no <=MAX_VALVE_B){
		reg = FPGA_VALVE_B;
		dev_no -= 16;
	}
	else if( dev_no >= MIN_VALVE_C && dev_no <=MAX_VALVE_C){
		reg = FPGA_VALVE_C;
		dev_no -=32;
	}
	

	val = fpga_read_reg(g_fpga_dev, reg);

	if(action)
		val |= 1 << (dev_no-1);
	else
		val &=  ~(1 << (dev_no-1));
	
	fpga_write_reg(g_fpga_dev, reg, val);

}



void fpga_set_pump(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4,struct measure_related_info * mri)
{
	u16 val;

	//report_exe_time();

	fpga_info(("fpga_set_pump"));

	//val = fpga_read_reg(g_fpga_dev, FPGA_NEGA_PUMP_EN);
	
	fpga_write_reg(g_fpga_dev, FPGA_NEGA_PUMP_EN, action);

}



void  fpga_set_laser_ctrl(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4,struct measure_related_info * mri)
{	

	struct timespec exe_time;
	long diff_time;

	getnstimeofday(&exe_time);


	diff_time = ((exe_time.tv_sec*1000*1000 + exe_time.tv_nsec/1000 ) -  (tc_old.tv_sec*1000*1000 + tc_old.tv_nsec/1000 )) - delay_time*1000;

	timarry[4] =diff_time;


	report_exe_time();

	fpga_info(("fpga_set_laser_ctrl"));
	fpga_write_reg(g_fpga_dev, FPGA_LASER_DIODE, action);

}


void fpga_set_cnst_ctrl(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri)
{	

	struct timespec exe_time;
	long diff_time;

	getnstimeofday(&exe_time);


	diff_time = ((exe_time.tv_sec*1000*1000 + exe_time.tv_nsec/1000 ) -  (tc_old.tv_sec*1000*1000 + tc_old.tv_nsec/1000 )) - delay_time*1000;

	timarry[5] =diff_time;



	report_exe_time();
	fpga_info(("fpga_set_cnst_ctrl"));

	fpga_write_reg(g_fpga_dev, FPGA_CONST_CTRL, action);

}

void fpga_set_burn_ctrl(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri)
{


	u32 reg;


	fpga_info(("fpga_set_burn"));

	if(dev_no == 1)
		reg = FPGA_RBC_BURN;
	else if(dev_no == 2)
		reg = FPGA_WBC_BURN;
		
	fpga_write_reg(g_fpga_dev, reg, action);

}


void fpga_set_led(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4,  struct measure_related_info * mri)
{

	u32 reg;
	
	fpga_info(("fpga_set_hgb_led"));

	if(dev_no == 1)
		reg = FPGA_HGB_LED;
	else if(dev_no == 2)
		reg = FPGA_HGB_LED;
	
	fpga_write_reg(g_fpga_dev, reg, action);
	
}


//motor move (samph , sampv,mixh,mixv,mixr): dev_no : 1,2,3,4,5 map to interrupt sample motor:0,1,2,3,4
void fpga_motor_ctrl(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri)
{
	int speed = p1;
	int pos = p2;
	static int flag =0;
	
	//report_exe_time();
	
	fpga_info(("fpga_motor_ctrl:timout:%d devnum:%d, speed:%d pos=%d",timout, dev_no,speed,pos));

#if 0
	struct timespec exe_time;
	long diff_time;
	
	getnstimeofday(&exe_time);

	
	diff_time = ((exe_time.tv_sec*1000*1000 + exe_time.tv_nsec/1000 ) -  (tc_old.tv_sec*1000*1000 + tc_old.tv_nsec/1000 )) - delay_time*1000;

	if(flag) {
		timarry[9] =diff_time;
	}else
		timarry[1] =diff_time;
	
	flag =1;

#endif

	comm_main_motor_set(dev_no, speed, pos);

	
	//enable motor interrupt;
	//fpga_set_intmotor_onoff(1, dev_no-1);

		
	switch(dev_no){
	 	case FPGA_MAIN_MOTOR_SAMPLE_V:
			atomic_set(&g_fpga_dev->m_sampv_t, 0); //this flag need set in  motor interrupt handler;
			hrtimer_start(&g_fpga_dev->m_sampv_timer, ms_to_ktime(timout), HRTIMER_MODE_REL);
			break;
		
		case FPGA_MAIN_MOTOR_SAMPLE_H:
			atomic_set(&g_fpga_dev->m_sampv_t, 0); //this flag need set in  motor interrupt handler;
			hrtimer_start(&g_fpga_dev->m_samph_timer, ms_to_ktime(timout), HRTIMER_MODE_REL);
			break;
			
	 	case FPGA_MAIN_MOTOR_MIX_V:			
			atomic_set(&g_fpga_dev->m_mixv_t, 0); //this flag need set in  motor interrupt handler;
			hrtimer_start(&g_fpga_dev->m_mixv_timer, ms_to_ktime(timout), HRTIMER_MODE_REL);
			break;	

		case FPGA_MAIN_MOTOR_MIX_R:			
			atomic_set(&g_fpga_dev->m_mixr_t, 0); //this flag need set in  motor interrupt handler;
			hrtimer_start(&g_fpga_dev->m_mixr_timer, ms_to_ktime(timout), HRTIMER_MODE_REL);
			break;	
			
	 	case FPGA_MAIN_MOTOR_MIX_H:			
			atomic_set(&g_fpga_dev->m_mixh_t, 0); //this flag need set in  motor interrupt handler;
			hrtimer_start(&g_fpga_dev->m_mixh_timer, ms_to_ktime(timout), HRTIMER_MODE_REL);
			break;		
	}	
	

}

void fpga_motor_init(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri)
{

	int speed = p1;

	report_exe_time();
	
	fpga_info(("fpga_motor_init:timout:%d devnum:%d, speed:%d",timout, dev_no,speed));


	comm_motor_init(dev_no, speed);

	//fpga_set_intmotor_onoff(1, dev_no-1);

		
		switch(dev_no){
			case FPGA_MAIN_MOTOR_SAMPLE_H:
				atomic_set(&g_fpga_dev->m_samph_t, 0); //this flag need set in  motor interrupt handler;
				hrtimer_start(&g_fpga_dev->m_samph_timer, ms_to_ktime(timout), HRTIMER_MODE_REL);
				break;
			case FPGA_MAIN_MOTOR_SAMPLE_V:				
				atomic_set(&g_fpga_dev->m_sampv_t, 0); //this flag need set in  motor interrupt handler;
				hrtimer_start(&g_fpga_dev->m_sampv_timer, ms_to_ktime(timout), HRTIMER_MODE_REL);
				break;
			case FPGA_MAIN_MOTOR_MIX_H:				
				atomic_set(&g_fpga_dev->m_mixh_t, 0); //this flag need set in  motor interrupt handler;
				hrtimer_start(&g_fpga_dev->m_mixh_timer, ms_to_ktime(timout), HRTIMER_MODE_REL);
				break;	
			case FPGA_MAIN_MOTOR_MIX_V:				
				atomic_set(&g_fpga_dev->m_mixv_t, 0); //this flag need set in  motor interrupt handler;
				hrtimer_start(&g_fpga_dev->m_mixv_timer, ms_to_ktime(timout), HRTIMER_MODE_REL);
				break;	
			case FPGA_MAIN_MOTOR_MIX_R:				
				atomic_set(&g_fpga_dev->m_mixr_t, 0); //this flag need set in  motor interrupt handler;
				hrtimer_start(&g_fpga_dev->m_mixr_timer, ms_to_ktime(timout), HRTIMER_MODE_REL);
				break;			
		}	
		


}


//(u16 dev_no, u16 timout, u16 away_step, u16 close_step, u16 speed, u16 wait_time, u16 mix_times
void fpga_motor_cycle(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri)
{
	char *pstr ="\n";
	int mix_times = action;
	int away_step = p1;
	int close_step = p2;
	int speed = p3;
	int wait_time = p4;

#if 0
	struct timespec exe_time;
	long diff_time;

	getnstimeofday(&exe_time);


	diff_time = ((exe_time.tv_sec*1000*1000 + exe_time.tv_nsec/1000 ) -  (tc_old.tv_sec*1000*1000 + tc_old.tv_nsec/1000 )) - delay_time*1000;

	timarry[8] =diff_time;
	report_exe_time();
#endif

	

	fpga_info(("fpga_motor_cycle:timout:%d devnum:%d, away_step:%d close_step:%d speed:%d wait_time:%d mix_times:%d",timout, dev_no,away_step,close_step,speed,wait_time,mix_times));

	/****do  some motor regs setting****/
	
	//fpga_set_intmotor_onoff(1, dev_no-1);
	
	switch(dev_no){
		case FPGA_MAIN_MOTOR_SAMPLE_H:		
			atomic_set(&g_fpga_dev->m_samph_t, 0); //this flag need set in  motor interrupt handler;
			hrtimer_start(&g_fpga_dev->m_samph_timer, ms_to_ktime(timout), HRTIMER_MODE_REL);
			break;
		case FPGA_MAIN_MOTOR_SAMPLE_V:			
			atomic_set(&g_fpga_dev->m_sampv_t, 0); //this flag need set in  motor interrupt handler;
			hrtimer_start(&g_fpga_dev->m_sampv_timer, ms_to_ktime(timout), HRTIMER_MODE_REL);
			break;
		case FPGA_MAIN_MOTOR_MIX_H:			
			atomic_set(&g_fpga_dev->m_mixh_t, 0); //this flag need set in  motor interrupt handler;
			hrtimer_start(&g_fpga_dev->m_mixh_timer, ms_to_ktime(timout), HRTIMER_MODE_REL);
			break;	
		case FPGA_MAIN_MOTOR_MIX_V:			
			atomic_set(&g_fpga_dev->m_mixv_t, 0); //this flag need set in  motor interrupt handler;
			hrtimer_start(&g_fpga_dev->m_mixv_timer, ms_to_ktime(timout), HRTIMER_MODE_REL);
			break;	
		case FPGA_MAIN_MOTOR_MIX_R:			
			atomic_set(&g_fpga_dev->m_mixr_t, 0); //this flag need set in  motor interrupt handler;
			hrtimer_start(&g_fpga_dev->m_mixr_timer, ms_to_ktime(timout), HRTIMER_MODE_REL);
			break;			
		}	

}

/************************* motor control  start***********************/
//timout:ms format;
//dev_no:1,2,3 ----> map to intterupt inj:5,6,7
void fpga_inject_motor_ctrl(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri)
{
	ktime_t inject_interval;
	
	int speed = p1;
	int steps = p2;
	int reg;
	int dir;

#if 0
	static int flag =0;

	struct timespec exe_time;
	long diff_time;

	getnstimeofday(&exe_time);


	diff_time = ((exe_time.tv_sec*1000*1000 + exe_time.tv_nsec/1000 ) -  (tc_old.tv_sec*1000*1000 + tc_old.tv_nsec/1000 )) - delay_time*1000;

	if(flag){
		timarry[10] =diff_time;
	}else
		timarry[2] =diff_time;

	report_exe_time();

	flag=1;
#endif
	
	fpga_info(("fpga_inject_motor_ctrl:timout:%d devnum:%d,speed:%d step:%d",timout, dev_no,speed,steps));

	comm_inj_motor_set(dev_no, speed, steps);

	switch(dev_no){
		case FPGA_INJ1_MOTOR:	
			atomic_set(&g_fpga_dev->m_inj1_t, 0); //this flag need set in  motor interrupt handler;
			hrtimer_start(&g_fpga_dev->m_inj1_timer, ms_to_ktime(timout), HRTIMER_MODE_REL);
			break;
	 	case FPGA_INJ2_MOTOR:		
			atomic_set(&g_fpga_dev->m_inj2_t, 0); //this flag need set in  motor interrupt handler;
			hrtimer_start(&g_fpga_dev->m_inj2_timer, ms_to_ktime(timout), HRTIMER_MODE_REL);
			break;
	 	case FPGA_INJ3_MOTOR:
			atomic_set(&g_fpga_dev->m_inj3_t, 0); //this flag need set in  motor interrupt handler;
			hrtimer_start(&g_fpga_dev->m_inj3_timer, ms_to_ktime(timout), HRTIMER_MODE_REL);
			break;			
	}

	
}

void fpga_inject_motor_stair(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri)
{
	ktime_t inject_interval;
	int first_seed = p1;
	int first_steps = p2;
	int second_speed = p3;
	int second_steps = p4;

	
	report_exe_time();
	fpga_info(("fpga_inject_motor_stair:timout:%d devnum:%d,speed1:%d step1:%d,speed2:%d step2:%d",timout, dev_no,first_seed,first_steps,second_speed,second_steps));

	//do some  motor regs setting;

	//enable motor interrupt;
	//fpga_set_intmotor_onoff(1, dev_no+ 4);	

	inject_interval = ms_to_ktime(timout);
		
	switch(dev_no){
		case 1:
			atomic_set(&g_fpga_dev->m_inj1_t, 0); //this flag need set in  motor interrupt handler;
			hrtimer_start(&g_fpga_dev->m_inj1_timer, ms_to_ktime(timout), HRTIMER_MODE_REL);
			break;
	 	case 2:
			atomic_set(&g_fpga_dev->m_inj2_t, 0); //this flag need set in  motor interrupt handler;
			hrtimer_start(&g_fpga_dev->m_inj2_timer, ms_to_ktime(timout), HRTIMER_MODE_REL);
			break;
	 	case 3:
			atomic_set(&g_fpga_dev->m_inj3_t, 0); //this flag need set in  motor interrupt handler;			
			hrtimer_start(&g_fpga_dev->m_inj3_timer, ms_to_ktime(timout), HRTIMER_MODE_REL);
			break;			
	}
	
	
}


//dev_no:1,2,3
void fpga_inject_motor_init(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri)
{
	ktime_t inject_interval = ms_to_ktime(timout);
	int speed = p1;

	report_exe_time();
	
	fpga_info(("fpga_inject_motor_init:timout:%d devnum:%d,speed:%d",timout, dev_no,speed));

	//do some  motor regs setting;
	comm_inj_motor_init(dev_no, speed);

	//enable motor interrupt;
	//fpga_set_intmotor_onoff(1, dev_no+ 4);
		
	switch(dev_no){
		case FPGA_INJ1_MOTOR:
			atomic_set(&g_fpga_dev->m_inj1_t, 0); //this flag need set in  motor interrupt handler;			
			hrtimer_start(&g_fpga_dev->m_inj1_timer, ms_to_ktime(timout), HRTIMER_MODE_REL);
			break;
	 	case FPGA_INJ2_MOTOR:
			atomic_set(&g_fpga_dev->m_inj2_t, 0); //this flag need set in  motor interrupt handler;
			hrtimer_start(&g_fpga_dev->m_inj2_timer, ms_to_ktime(timout), HRTIMER_MODE_REL);
			break;
	 	case FPGA_INJ3_MOTOR:			
			atomic_set(&g_fpga_dev->m_inj3_t, 0); //this flag need set in  motor interrupt handler;
			hrtimer_start(&g_fpga_dev->m_inj3_timer, ms_to_ktime(timout), HRTIMER_MODE_REL);
			break;			
	}	

}


void fpga_set_collect_reg(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri)
{
	u32 reg;

	report_exe_time();

	fpga_info(("fpga_set_collect_reg"));

	switch(dev_no) {
		case 1:
			reg = FPGA_RBC_REG;
			break;
		case 2:
			reg = FPGA_WBC_REG;
			break;
		case 3:
			reg = FPGA_DIFF_REG;
			break;
	}
	
	fpga_write_reg(g_fpga_dev, reg, action);
}

void fpga_setup_vac(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri)
{
	u16  min, press_min = p1;
	u16 max, press_max = p2;
	u16 press_target = p3;

	
	fpga_info(("fpga_set_vac: press_mix:%d press_max:%d press_target:%d", press_min, press_max, press_target));



	//min = 4096/5*(4.5*abs(press_min)/100 + 0.2);
	
	//max = 4096/5*(4.5*abs(press_max)/100 + 0.2);

	fpga_info(("decode: min:%d max:%d", min, max));

	
	fpga_write_reg(g_fpga_dev, FPGA_IOCTRL_REG_BASE, 1);


	fpga_write_reg(g_fpga_dev, FPGA_NEGA_PRESSURE_MIN, press_min);
	fpga_write_reg(g_fpga_dev, FPGA_NEGA_PRESSURE_MAX, press_max);
	fpga_write_reg(g_fpga_dev, FPGA_NEGA_PRESSURE_TARGET, press_target);


}

void fpga_type_invalid(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4,struct measure_related_info * mri)
{

	fpga_info(("fpga_type_invalid"));

}

/************************* motor control  end***********************/



/************************* hgb, rbc, wbc data sample start***********************/


void fpga_get_hgb_blank(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri)
{
	u16 value,hgb_cnt=0;

	report_exe_time();


	fpga_info(("fpga_get_hgb_blank"));
	memset((u8 *)hgb_buf, 0, 20);

	printk("fpga_get_hgb_blank\n");
	while(hgb_cnt < HGB_READ_CNT) {
		value = fpga_read_reg(g_fpga_dev,FPGA_SYS_CHAN_HGB);
		hgb_buf[hgb_cnt++] = value;		
		usleep_range(300,400); //A/D change need use 0.016ms * 16ch = 0.3ms;
		printk("fpga_get_hgb_blank:%d \n", value);
	}

	encode_tlv_netlink_send_data_rpt(MSG_DATA_REPORT, SUBTYPE_DATA_REPORT_HGB,mri, hgb_buf,HGB_READ_CNT*2);
}

//read 10 count hgb blood 
void fpga_get_hgb_blood(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri)
{
	u16 value,hgb_cnt=0;

	report_exe_time();

	fpga_info(("fpga_get_hgb_blood"));

	printk("fpga_get_hgb_blood\n");
	memset((u8 *)hgb_buf, 0, 20);
#if 1
	while(hgb_cnt < HGB_READ_CNT) {
		value = fpga_read_reg(g_fpga_dev,FPGA_SYS_CHAN_HGB);
		hgb_buf[hgb_cnt++] = value;		
		usleep_range(300,400); //A/D change need use 0.016ms * 16ch = 0.3ms;
		
		printk("%d \n", value);
	}
#endif	
	encode_tlv_netlink_send_data_rpt(MSG_DATA_REPORT, SUBTYPE_DATA_REPORT_HGB,mri, hgb_buf,HGB_READ_CNT*2 );
	
}


//get sys diff fifo value;
void fpga_get_fifo_rbc(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri)
{
	u16 value,diff_cnt=0;
	int i;
	

	fpga_info(("%s : timout:%d devno:%d",__func__, timout, dev_no));


	g_fpga_dev->test_flag = 0;
	intcnt = 0;
	memset(timearr, 0, 10);
	memset(lenarr, 0, 10);

	cnt=0;

	g_fpga_dev->pmri = mri;
	//set atomic indacate interrupt can check this flag to wether to report to user;
	atomic_set(&g_fpga_dev->fifo_rbc_t,1);	
	rbc_fisrt_flag = 1;
	rbc_per_len = 0;
	rbc_end_rpt_flag = 0;
	rbc_total = 0;
	rbc_irq = 0;
	cpycnt=0;
	

	memset(rbc_cpy_time, 0, 6000);
	
	memset(rbctime, 0, 8000);
	
	memset(rbc_xfer_len, 0, 8000);

	memset(g_fpga_dev->rbcout_addr, 0,   2*FIFO_DATA_DMA_SIZE);
		
 	gpmc_fpga_rbc_fifo_init(g_fpga_dev);

	gpmc_fpga_rbc_fifo_enable(g_fpga_dev, timout);

	hrtimer_start(&g_fpga_dev->fifo_rbc_timer, ms_to_ktime(timout), HRTIMER_MODE_REL);
	
}

//get sys diff fifo value;
void fpga_get_fifo_wbc(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri)
{
	u16 value,diff_cnt=0;

	fpga_info(("fpga_get_fifo_wbc:timout:%d devno:%d",timout, dev_no));

	//enable sys chan  interrupt;
	//fpga_set_intsys_onoff(1);
	g_fpga_dev->pmri = mri;

	//set atomic indacate interrupt can check this flag to wether to report to user;
	atomic_set(&g_fpga_dev->fifo_wbc_t,1);	
	wbc_fisrt_flag = 1;
	wbc_per_len = 0;
	wbc_end_rpt_flag = 0;
	wbc_total = 0;
	wbc_irq = 0;


	memset(g_fpga_dev->wbcout_addr, 0,   2*FIFO_DATA_DMA_SIZE);
		
 	gpmc_fpga_wbc_fifo_init(g_fpga_dev);

	gpmc_fpga_wbc_fifo_enable(g_fpga_dev, timout);

	hrtimer_start(&g_fpga_dev->fifo_wbc_timer, ms_to_ktime(timout), HRTIMER_MODE_REL);
	
}

//get sys diff fifo value;
void fpga_get_fifo_diff(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri)
{
	u16 value,diff_cnt=0;

	return ;
	
	fpga_info(("fpga_get_diff:timout:%d devno:%d",timout, dev_no));

	//enable sys chan  interrupt;
	//fpga_set_intopt_onoff(1);
	g_fpga_dev->pmri = mri;

	//set atomic indacate interrupt can check this flag to wether to report to user;
	atomic_set(&g_fpga_dev->fifo_diff_t,1);		
	diff_fisrt_flag = 1;
	diff_per_len = 0;
	diff_end_rpt_flag = 0;
	diff_total = 0;
	diff_irq = 0;


	memset(g_fpga_dev->diffout_addr, 0,   2*SYS_DATA_DMA_SIZE);


 	gpmc_fpga_diff_fifo_init(g_fpga_dev);

	gpmc_fpga_diff_fifo_enable(g_fpga_dev, timout);


	hrtimer_start(&g_fpga_dev->fifo_diff_timer, ms_to_ktime(timout), HRTIMER_MODE_REL);
	
}


/************************* hgb, rbc, wbc data, sample end***********************/

void fpga_moni_vac(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri)
{
	u16 value1, value2 ;
	int press_mix = p1;
	int press_max = p2;
	
	char *pwstr = "vac warning\n";

	report_exe_time();
	
	fpga_info(("fpga_moni_vac: press_mix:%d press_max:%d",press_mix, press_max));

	value1 = fpga_read_reg(g_fpga_dev,FPGA_NEGA_PRESSURE_MIN);
	value2 = fpga_read_reg(g_fpga_dev,FPGA_NEGA_PRESSURE_MAX);

	//value1 = -45000;
	//value1 = -40000;


	if(value1 < press_mix || value2 >  press_max){
		encode_tlv_netlink_send_data_rpt(MSG_TYPE_OTHER_REPORT, SUBTYPE_OTHERREPORT_INFO_PRESS,mri, pwstr,strlen(pwstr) );
	}

}


void fpga_moni_press(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri)
{
	u16 value1, value2 ;
	int press_mix = p1;
	int press_max = p2;
	
	char *pwstr = "press warning\n";

	report_exe_time();

	fpga_info(("fpga_moni_press"));

	value1 = fpga_read_reg(g_fpga_dev,FPGA_NEGA_PRESSURE_MIN);
	//msleep(10);
	value2 = fpga_read_reg(g_fpga_dev,FPGA_NEGA_PRESSURE_MAX);

	//value1 = 30;
	//value2 = 100;

	
	//if(value1 < press_mix || value2 >  press_max) {
	//	encode_tlv_netlink_send_data_rpt(MSG_TYPE_OTHER_REPORT, SUBTYPE_OTHERREPORT_INFO_PRESS,mri, pwstr,strlen(pwstr) );
	//}

}


void fpga_moni_temp(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri)
{
	u16 value1 ;
	int temp_min = p1;
	int temp_max = p2;
	
	char *pwstr = "temp warning\n";

	report_exe_time();

	fpga_info(("fpga_moni_temp"));

	value1 = fpga_read_reg(g_fpga_dev,FPGA_SYS_CHAN_TEMP1);
	
	//value1 = 20;
	
	if(value1 < temp_min || value1 >  temp_max) {
		encode_tlv_netlink_send_data_rpt(MSG_TYPE_OTHER_REPORT, SUBTYPE_OTHERREPORT_INFO_TEMP,mri, pwstr,strlen(pwstr) );
	}

}

void fpga_moni_pool(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri)
{
	u32 reg;
	u16 value;
	int expect_state = action;
	
	char *pwstr1 = "pool full warning\n";
	char *pwstr2 = "pool not full warning\n";

	report_exe_time();

	fpga_info(("fpga_moni_pool"));
	
	if(dev_no)
		reg = FPGA_SYS_FLOAT;

	value = fpga_read_reg(g_fpga_dev,reg);
	//value = 0;
	
	if(value != expect_state) {
		if(expect_state == 0){
			encode_tlv_netlink_send_data_rpt(MSG_TYPE_OTHER_REPORT, SUBTYPE_OTHERREPORT_INFO_SENSOR,mri, pwstr1,strlen(pwstr1) );
		}else		
			encode_tlv_netlink_send_data_rpt(MSG_TYPE_OTHER_REPORT, SUBTYPE_OTHERREPORT_INFO_SENSOR,mri, pwstr2,strlen(pwstr2) );
	}


}



void fpga_moni_bubble(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri)
{
	u32 reg;
	u16 value;
	char pwstr[] = "bubble:1\n";

	report_exe_time();
		
	fpga_info(("fpga_moni_bubble"));
	
	if(dev_no)
		reg = FPGA_BUBBLE;

	value = fpga_read_reg(g_fpga_dev,reg);
	//value = 1;
	
	if(value == 0){
		pwstr[7] = '0';
	}
	else{
		pwstr[7] = '1';
	}


	encode_tlv_netlink_send_data_rpt(MSG_TYPE_OTHER_REPORT, SUBTYPE_OTHERREPORT_INFO_SENSOR,mri, pwstr,strlen(pwstr) );
	
}

void fpga_sampler_ctrl(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri)
{
	int postion = action;

#if 0	
	struct timespec exe_time;
	long diff_time;
	
	getnstimeofday(&exe_time);

	
	diff_time = ((exe_time.tv_sec*1000*1000 + exe_time.tv_nsec/1000 ) -  (tc_old.tv_sec*1000*1000 + tc_old.tv_nsec/1000 )) - delay_time*1000;

	timarry[3] =diff_time;

#else
	report_exe_time();
	fpga_info(("fpga_sampler_ctrl, pos:%d",postion));
#endif	
}

void fpga_event_unlimited(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri)
{
	report_exe_time();

	fpga_info(("fpga_event_unlimited"));

}

void fpga_event_limited_start(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri)
{
	report_exe_time();

	fpga_info(("fpga_event_limited_start"));

}

void fpga_event_limited_end(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri)
{
	report_exe_time();

	fpga_info(("fpga_event_limited_end"));

}

void fpga_event_start(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri)
{

	struct timespec exe_time;
	long diff_time;
	
	getnstimeofday(&exe_time);

	
	diff_time = ((exe_time.tv_sec*1000*1000 + exe_time.tv_nsec/1000 ) -  (tc_old.tv_sec*1000*1000 + tc_old.tv_nsec/1000 )) - delay_time*1000;

	timarry[0] =diff_time;
	
	fpga_info(("fpga_event_start"));

}

void fpga_event_end(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri)
{
	report_exe_time();

	fpga_info(("fpga_event_end"));

}

void fpga_event_asp_complete(u16 dev_no, u16 timout, int action, int p1, int p2, int p3,  int p4, struct measure_related_info * mri)
{
	report_exe_time();

	fpga_info(("fpga_event_asp_complete"));

}

/**********************************************************************************************/
// ctrl type event handle
/************************************************************************************************/
void fpga_ctrltype_get_valve(struct sensor_info *psi)
{
	u32 reg_a, reg_b, reg_c;
	u16 val_a, val_b, val_c;

	
	fpga_info(("%s", __func__));
	
	reg_a =FPGA_VALVE_A;
	reg_b = FPGA_VALVE_B;
	reg_c = FPGA_VALVE_C;

	val_a = fpga_read_reg(g_fpga_dev, reg_a);
	val_b = fpga_read_reg(g_fpga_dev, reg_b);
	val_c = fpga_read_reg(g_fpga_dev, reg_c);
	
	psi->m_bitmap[0]  = val_b << 16 | val_a; 
	psi->m_bitmap[1]  = val_c;
	
	fpga_info(("m_mask0:0x%x m_mask1:0x%x  m_bitmap[0]=0x%x m_bitmap[1]=0x%x",  psi->m_mask[0],  psi->m_mask[1], psi->m_bitmap[0], psi->m_bitmap[1]));
	
}

void fpga_ctrltype_set_valve(struct sensor_info *psi)
{
	u32 reg_a, reg_b, reg_c, reg;
	u16 val_a, val_b, val_c, val;
	int dev_no = 0;
	u32 m_mask0, m_mask1, m_mask2;
	u64 m_mask, m_bitmap;
	int flag;
	
	fpga_info(("%s", __func__));



	m_mask =   ((u64)psi->m_mask[1]) << 32 | psi->m_mask[0];
	m_bitmap =   ((u64)psi->m_bitmap[1]) << 32 | psi->m_bitmap[0] ;

	fpga_info(("m_mask:0x%llx   m_bitmap:0x%llx    m_mask0:0x%x m_mask1:0x%x  m_bitmap[0]=0x%x m_bitmap[1]=0x%x", m_mask,  m_bitmap, psi->m_mask[0],  psi->m_mask[1], psi->m_bitmap[0], psi->m_bitmap[1]));

	val_a = fpga_read_reg(g_fpga_dev, FPGA_VALVE_A);
	val_b = fpga_read_reg(g_fpga_dev, FPGA_VALVE_B);
	val_c = fpga_read_reg(g_fpga_dev, FPGA_VALVE_C);

	printk("val_a:0x%x  val_b:0x%x val_c:0x%x \n" , val_a, val_b, val_c);


	while( m_mask != 0) {
		if(test_bit(0, &m_mask)){
			//printk("dev_no:%d\n", dev_no);
			if(dev_no <= MAX_VALVE_A-1){
				//clear_bit(dev_no, &val_a);
				val_a &= ~(1 << dev_no);
				val_a |=(( m_bitmap >> (dev_no)) & 0x1) << dev_no;
			}else if(dev_no >= MIN_VALVE_B-1 && dev_no <=MAX_VALVE_B-1){
				//dev_no -= 16;
				val_b &= ~(1 << (dev_no-16));
				val_b |=(( m_bitmap >> (dev_no)) & 0x1) << (dev_no-16);
			}else if(dev_no >= MIN_VALVE_C-1 && dev_no <=MAX_VALVE_C-1){
				//dev_no -=31;
				val_c &= ~(1 << (dev_no-32));
				val_c |= (( m_bitmap >> (dev_no)) & 0x1) << (dev_no-32);
			}
			
		}
			
		m_mask = m_mask >> 1;
		
		dev_no++;
	}
	fpga_info(("val_a:0x%x val_b:0x%x  val_c:0x%x\n", val_a, val_b, val_c));
	fpga_write_reg(g_fpga_dev, FPGA_VALVE_A, val_a);
	fpga_write_reg(g_fpga_dev, FPGA_VALVE_B, val_b);
	fpga_write_reg(g_fpga_dev, FPGA_VALVE_C, val_c);
	
}


void fpga_ctrltype_get_pump(struct sensor_info *psi)
{
	u32 reg;
	u16 val;


	val = fpga_read_reg(g_fpga_dev, FPGA_NEGA_PUMP_EN);

	psi->m_bitmap[0]  = val; 
	
	fpga_info(("%s: m_mask0:0x%x m_bitmap[0]=0x%x", __func__, psi->m_mask[0] , psi->m_bitmap[0] ));

}

void fpga_ctrltype_set_pump(struct sensor_info *psi)
{
	u32 reg_a, reg_b, reg_c, reg;
	u16 val_a, val_b, val_c, val;
	int dev_no = 0;
	int m_mask0, m_mask1, m_mask2;
	u64 m_mask, m_bitmap;
	
	fpga_info(("%s: m_mask0:0x%x m_bitmap[0]=0x%x", __func__, psi->m_mask[0] , psi->m_bitmap[0] ));



	val = fpga_read_reg(g_fpga_dev, FPGA_NEGA_PUMP_EN);


	 while( psi->m_mask[0] != 0) {
		if(test_bit(0, &psi->m_mask[0])){
			if(dev_no <= 15){
				 val &= ~(1 << dev_no);
				 val |=(( psi->m_bitmap[0]  >> (dev_no)) & 0x1) << dev_no;
			 }
		}	 
		if(dev_no >3)
			break;
		psi->m_mask[0] = psi->m_mask[0] >> 1;
		dev_no++;
	 }

	printk("val:0x%x\n", val);
         fpga_write_reg(g_fpga_dev, FPGA_NEGA_PUMP_EN, val);

}


void fpga_ctrltype_get_all_sensor(struct sensor_info *psi)
{
	u32 reg;
	u16 val;


	fpga_info(("%s", __func__));
}


void fpga_ctrltype_set_all_sensor(struct sensor_info *psi)
{
	u32 reg_a, reg_b, reg_c, reg;
	u16 val_a, val_b, val_c, val;
	int dev_no = 1;
	u32 m_mask0, m_mask1, m_mask2;
	u64 m_mask, m_bitmap;

	fpga_info(("%s", __func__));

	
}



void fpga_ctrltype_get_press(struct range_info *pri)
{
	 u16 value1, value2, value3, value4;
	 u16 min,max,actual,target;

	 fpga_info(("%s", __func__));
	
	value1 = fpga_read_reg(g_fpga_dev,FPGA_NEGA_PRESSURE_MIN);

	value2 = fpga_read_reg(g_fpga_dev,FPGA_NEGA_PRESSURE_MAX);


	value3 =  fpga_read_reg(g_fpga_dev,FPGA_NEGA_PRESSURE_TARGET);

	value4 =  fpga_read_reg(g_fpga_dev,FPGA_NEGA_PRESSURE_ACTUAL);

	pri->m_min =value1 ; 
	pri->m_max =  value2; 
	pri->m_target=  value3; 
	pri->m_actual =  value4; 
	 fpga_info(("decode :m_min:%d m_max:%d target:%d actual:%d", pri->m_min, pri->m_max ,pri->m_target, pri->m_actual));	
}

void fpga_ctrltype_set_press(struct range_info *pri)
{

	 
	fpga_info(("%s: min:%d  max:%d target:%d", __func__, pri->m_min, pri->m_max, pri->m_target));	 

	fpga_write_reg(g_fpga_dev, FPGA_IOCTRL_REG_BASE, 1);


	fpga_write_reg(g_fpga_dev, FPGA_NEGA_PRESSURE_MIN, pri->m_min);
	fpga_write_reg(g_fpga_dev, FPGA_NEGA_PRESSURE_MAX, pri->m_max);
	fpga_write_reg(g_fpga_dev, FPGA_NEGA_PRESSURE_TARGET, pri->m_target);
	
}


u16 temp_min[3], temp_max[3];

void fpga_ctrltype_get_temp(struct range_info *pri)
{
	 u16 act_temp, value2,  reg_target, reg_en, target_temp;
	 u32 reg_act;

	fpga_info(("%s: id=%d", __func__, pri->m_id));	 
	switch(pri->m_id) {
		case 1:
			reg_act= FPGA_SYS_CHAN_TEMP1;	
			reg_target = HEAT_DIFF_TEMP;
			reg_en = HEAT_D_EN;
		case 2:
			reg_act = FPGA_SYS_CHAN_TEMP2;
		case 3:
			reg_act = FPGA_SYS_CHAN_TEMP2;
			reg_target = HEAT_O_TEMP;
			reg_en = HEAT_O_EN;
		case 4:
			reg_act = FPGA_SYS_CHAN_TEMP4;
		case 5:
			reg_act = FPGA_SYS_CHAN_TEMP5;
		case 6:
			reg_act = FPGA_SYS_CHAN_TEMP6;			
	}
	
	act_temp = fpga_read_reg(g_fpga_dev,reg_act); //actual temp
	target_temp =  fpga_read_reg(g_fpga_dev,reg_target); //target temp

	pri->m_actual = act_temp; 
	pri->m_target = target_temp;
	pri->m_min = temp_min[pri->m_id];
	pri->m_max = temp_max[pri->m_id];

	
	printk(" get acttemp:%d  target_temp:%d  m_min:%d m_max:%d\n", pri->m_actual, pri->m_target, pri->m_min, pri->m_max);

}

void fpga_ctrltype_set_temp(struct range_info *pri)
{
	 u16 reg_temp, reg_en, target;
	 u32 reg;

	fpga_info(("%s: id=%d  temp_target:%d  min:%d  max:%d", __func__, pri->m_id, pri->m_target, pri->m_min, pri->m_max));	 
	switch(pri->m_id) {
		case 1:
			reg_temp = HEAT_DIFF_TEMP;
			reg_en = HEAT_D_EN;
			break;
		case 2:
			reg_temp = FPGA_SYS_CHAN_TEMP2;
			break;
		case 3:
			reg_temp = HEAT_O_TEMP;
			reg_en = HEAT_O_EN;
			break;
		case 4:
			reg_temp = FPGA_SYS_CHAN_TEMP4;
			break;
		case 5:
			reg_temp = FPGA_SYS_CHAN_TEMP5;
			break;
		case 6:
			reg_temp = FPGA_SYS_CHAN_TEMP6;
			break;
	}

	fpga_write_reg(g_fpga_dev, reg_en, 1);
	
	fpga_write_reg(g_fpga_dev, reg_temp, pri->m_target);
	

	temp_min[pri->m_id] = pri->m_min;
	temp_max[pri->m_id] = pri->m_max;


}



void fpga_get_motor_run_time(struct motor_info *pmi, int motor_type)
{
	int regbase;
	u16 val;
	char str[10];
	
	if( motor_type == 1 && (pmi->m_id < FPGA_MAIN_MOTOR_SAMPLE_V || pmi->m_id > FPGA_MAIN_MOTOR_MIX_H)){
		fpga_error(("%s: dev_no:%d invalid", __func__, pmi->m_id));
		return -1;
	}else if ( motor_type == 2 && (pmi->m_id < FPGA_INJ1_MOTOR || pmi->m_id > FPGA_INJ3_MOTOR_BASE)){
		fpga_error(("%s: dev_no:%d  invalid", __func__, pmi->m_id));
		return -1;
	}

	if( motor_type == 1) {
		switch(pmi->m_id) {
			case FPGA_MAIN_MOTOR_SAMPLE_V:
				regbase = FPGA_SAMPV_MOTOR_BASE;
				break;		
			case FPGA_MAIN_MOTOR_SAMPLE_H: 
				regbase = FPGA_SAMPH_MOTOR_BASE;
				break;
			case FPGA_MAIN_MOTOR_MIX_V:
				regbase = FPGA_MIXV_MOTOR_BASE;
				break;	
			case FPGA_MAIN_MOTOR_MIX_R:
				regbase = FPGA_MIXR_MOTOR_BASE;
				break;				
			case FPGA_MAIN_MOTOR_MIX_H:
				regbase = FPGA_MIXH_MOTOR_BASE;
				break;
			}
	}else if( motor_type == 2) {
			//do some  motor regs setting;
		switch(pmi->m_id) {
			case FPGA_INJ1_MOTOR: 
				regbase = FPGA_INJ1_MOTOR_BASE;
				break;
			case FPGA_INJ2_MOTOR:
				regbase = FPGA_INJ2_MOTOR_BASE;
				break;
			case FPGA_INJ3_MOTOR:
				regbase = FPGA_INJ3_MOTOR_BASE;
				break;
			}
	}


	val = fpga_read_reg(g_fpga_dev,regbase + FPGA_MOTOR_RUN_TIME);
	sprintf(str, "%d", val);

	val = simple_strtol(str, NULL, 16);
	
	pmi->m_execute_time = val;
	
	printk("after format exe val:%d \n", pmi->m_execute_time);

}




void fpga_ctrltype_get_motor_info(struct motor_info *pmi)
{
	fpga_info(("%s: id:%d  currpos:%d", __func__, pmi->m_id, main_current_pos));

	pmi->m_pos_current = main_current_pos;
	
	printk("main_pre_steps[%d]=%d\n", main_current_pos, main_pre_steps[main_current_pos]);

	pmi->m_steps = main_motor_pos_map[pmi->m_id][pmi->m_pos_current];
	//main_pre_steps[main_current_pos] =  main_motor_pos_map[pmi->m_id][main_current_pos];
	fpga_get_motor_run_time(pmi, 1);
}


void main_motor_step_set( int dev_no, int speed, int steps, int pos)
{
	int motor_hold, index_start, ignor_step, chg_point, chg_speed, dir, temp_steps, tmp_speed; 
	int regbase;
	static int pre_pos;
	static int pr_steps, next_steps;
	int pos_steps,steps1, tmpstep;
	
	//int abs_steps;

	if(dev_no < FPGA_MAIN_MOTOR_SAMPLE_V || dev_no > FPGA_MAIN_MOTOR_MIX_H){
		fpga_error(("%s: dev_no invalid", __func__));
		return -1;
	}
	
	switch(dev_no) {
		case FPGA_MAIN_MOTOR_SAMPLE_V:
			regbase = FPGA_SAMPV_MOTOR_BASE;
			break;		
		case FPGA_MAIN_MOTOR_SAMPLE_H: 
			regbase = FPGA_SAMPH_MOTOR_BASE;
			break;
		case FPGA_MAIN_MOTOR_MIX_V:
			regbase = FPGA_MIXV_MOTOR_BASE;
			break;	
		case FPGA_MAIN_MOTOR_MIX_R:
			regbase = FPGA_MIXR_MOTOR_BASE;
			break;				
		case FPGA_MAIN_MOTOR_MIX_H:
			regbase = FPGA_MIXH_MOTOR_BASE;
			break;
		}


	if(speed == 0){
		fpga_info(("!!!!speed is zero!!!!"));
		return;
	}

	tmp_speed = 520 - 11250/speed;

	fpga_info(("main_motor_step_set:regbase:0x%x speed:%d steps:%d  abs_steps:%d\n",regbase, tmp_speed, steps,  abs_steps[dev_no]));

	motor_hold = 0x2;
	index_start = tmp_speed/4;
	chg_point = steps/8;
	chg_speed =  tmp_speed/8;
	tmpstep = steps;

	if(steps > 0)
		dir = 0;
	else {
		steps = abs(steps);
		dir = 1;
	}
		

	//set step max: 0x0
	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_STEP_MAX, steps);
	//set step add 0x2
	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_HOLD, motor_hold);
	//set step add 0x6
	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_DIR, dir);

	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_PHOTO_NUM, 9);
	
	//fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_ACC_INTER, 0x1);

	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_INDEX_START, index_start);

	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_SPEED, 15);


	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_MICROSTEP, 0x1);


	//fpga_write_reg(g_fpga_dev, reg+FPGA_MOTOR_IGNORE, ignor_step);

	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_MODE, 0x1);


// set dec motor speed:
	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_DEC_INTER, 0x1);

	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_CHANGE_POINT, chg_point );

	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_CHANGE_SPEED, 15 );
	
	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_SPD_TAB15, tmp_speed );

//enable motor:
	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_RUN, 0x1);
	main_pre_steps[dev_no] += tmpstep;
	printk("main_pre_steps[%d]=%d\n", dev_no, main_pre_steps[dev_no]);

	

}

int main_motor_pos_set(int dev_no, int speed, int pos)
{
	int motor_hold, index_start, ignor_step, chg_point, chg_speed, dir, pos_steps, temp_steps, tmp_speed; 
	int regbase;

	if(dev_no < FPGA_MAIN_MOTOR_SAMPLE_V || dev_no > FPGA_MAIN_MOTOR_MIX_H){
		fpga_error(("%s: dev_no invalid", __func__));
		return -1;
	}

	
	switch(dev_no) {
		case FPGA_MAIN_MOTOR_SAMPLE_V:
			regbase = FPGA_SAMPV_MOTOR_BASE;
			break;		
		case FPGA_MAIN_MOTOR_SAMPLE_H: 
			regbase = FPGA_SAMPH_MOTOR_BASE;
			break;
		case FPGA_MAIN_MOTOR_MIX_V:
			regbase = FPGA_MIXV_MOTOR_BASE;
			break;	
		case FPGA_MAIN_MOTOR_MIX_R:
			regbase = FPGA_MIXR_MOTOR_BASE;
			break;				
		case FPGA_MAIN_MOTOR_MIX_H:
			regbase = FPGA_MIXH_MOTOR_BASE;
			break;
		}


	pos_steps = get_main_motor_pos_speed(dev_no, pos);

	temp_steps = pos_steps - main_pre_steps[dev_no];

		//int motor:
	if(speed == 0){
		fpga_info(("!!!!speed is zero!!!!"));
		return;
	}
	
	tmp_speed = 520 - 11250/speed;

	fpga_info(("%s: speed:%d  real_speed:%d pos:%d    pos_steps:%d  main_pre_steps:%d  real_steps:%d\n", __func__, speed, tmp_speed, pos, pos_steps, main_pre_steps[dev_no],  temp_steps));

	
	if(temp_steps > 0)
		dir = 0;
	else {
		temp_steps = abs(temp_steps);
		dir = 1;
	}
	
	motor_hold = 0x2;
	index_start = tmp_speed/4;
	chg_point = temp_steps/8;
	chg_speed =  tmp_speed/8;


	//set step max: 0x0
	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_STEP_MAX, temp_steps);
	//set step add 0x2
	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_HOLD, motor_hold);
	//set step add 0x6
	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_DIR, dir);

	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_PHOTO_NUM, 9);
	
	//fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_ACC_INTER, 0x1);

	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_INDEX_START, index_start);

	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_SPEED, 15);


	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_MICROSTEP, 0x1);


	//fpga_write_reg(g_fpga_dev, reg+FPGA_MOTOR_IGNORE, ignor_step);

	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_MODE, 0x1);


// set dec motor speed:
	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_DEC_INTER, 0x1);

	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_CHANGE_POINT, chg_point );

	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_CHANGE_SPEED, 15 );
	
	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_SPD_TAB15, tmp_speed );

//enable motor:
	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_RUN, 0x1);
	main_current_pos = pos;
	main_pre_steps[dev_no] = pos_steps;

	//main_devno = dev_no;

	//main_pre_steps[dev_no] = pos_steps;
	//main_curr_steps[dev_no] = pos;
	//main_steps[dev_no] = temp_steps;
	//main_speed[dev_no] = tmp_speed;

	//main_abs_steps[dev_no] = main_motor_pos_map[dev_no][pos];
	printk("main_pre_steps[%d]=%d\n", dev_no, main_pre_steps[dev_no]);

	return 0;

}



void fpga_ctrltype_set_motor_move(struct motor_info *pmi)
{
	fpga_info(("%s: id:%d  speed:%d m_pos_target:%d  m_pos_current:%d  steps:%d pmi->m_type:%d", __func__, pmi->m_id, pmi->m_speed, pmi->m_pos_target,  pmi->m_pos_current, pmi->m_steps,   pmi->m_type));

	if(pmi->m_type == 1)
		main_motor_step_set(pmi->m_id, pmi->m_speed, pmi->m_steps, pmi->m_pos_target);
	else if(pmi->m_type == 2)
 		main_motor_pos_set(pmi->m_id, pmi->m_speed, pmi->m_pos_target);
	else 	if(pmi->m_type == 3){
		comm_motor_init(pmi->m_id, pmi->m_speed);
	}else 	if(pmi->m_type == 4){
		update_motor_pos_steps(pmi->m_id, pmi->m_pos_target );
	}
	return ;
}

void fpga_ctrltype_get_inject_info(struct motor_info *pmi)
{
	
	fpga_info(("%s", __func__));

	pmi->m_pos_current = inj_current_pos;
	pmi->m_steps = inj_motor_init_pos[pmi->m_id];
	

	fpga_get_motor_run_time(pmi, 2);
}


void inj_motor_step_set( int dev_no, int speed, int steps, int pos)
{
	int motor_hold, index_start, ignor_step, chg_point, chg_speed, dir, temp_steps, tmp_speed; 
	int regbase;
	static int pre_pos;
	static int pr_steps, next_steps;
	int pos_steps,steps1;
	
	//int abs_steps;

	if(dev_no < FPGA_INJ1_MOTOR || dev_no > FPGA_INJ3_MOTOR){
		fpga_error(("%s: dev_no invalid", __func__));
		return -1;
	}
	

	switch(dev_no) {
		case FPGA_INJ1_MOTOR: 
			regbase = FPGA_INJ1_MOTOR_BASE;
			break;
		case FPGA_INJ2_MOTOR:
			regbase = FPGA_INJ2_MOTOR_BASE;
			break;
		case FPGA_INJ3_MOTOR:
			regbase = FPGA_INJ3_MOTOR_BASE;
			break;
		}

	if(speed == 0){
		fpga_info(("speed == 0"));
		return;
	}
	
	tmp_speed = 520 - 11250/speed;

	fpga_info(("main_motor_step_set:regbase:0x%x speed:%d steps:%d  abs_steps:%d\n",regbase, tmp_speed, steps,  abs_steps[dev_no]));

	motor_hold = 0x2;
	index_start = tmp_speed/4;
	chg_point = steps/8;
	chg_speed =  tmp_speed/8;

	temp_steps = steps;

	if(steps > 0)
		dir = 1;
	else {
		steps = abs(steps);
		dir = 0;
	}
		

	//set step max: 0x0
	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_STEP_MAX, steps);
	//set step add 0x2
	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_HOLD, motor_hold);
	//set step add 0x6
	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_DIR, dir);

	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_PHOTO_NUM, 9);
	
	//fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_ACC_INTER, 0x1);

	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_INDEX_START, index_start);

	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_SPEED, 15);


	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_MICROSTEP, 0x1);


	//fpga_write_reg(g_fpga_dev, reg+FPGA_MOTOR_IGNORE, ignor_step);

	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_MODE, 0x1);


// set dec motor speed:
	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_DEC_INTER, 0x1);

	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_CHANGE_POINT, chg_point );

	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_CHANGE_SPEED, 15 );
	
	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_SPD_TAB15, tmp_speed );

//enable motor:
	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_RUN, 0x1);
	inj_pre_steps[dev_no] += temp_steps;


}

int inj_motor_pos_set(int dev_no, int speed, int pos)
{
	int motor_hold, index_start, ignor_step, chg_point, chg_speed, dir, pos_steps, temp_steps, tmp_speed; 
	int regbase;

	switch(dev_no) {
		case FPGA_INJ1_MOTOR: 
			regbase = FPGA_INJ1_MOTOR_BASE;
			break;
		case FPGA_INJ2_MOTOR:
			regbase = FPGA_INJ2_MOTOR_BASE;
			break;
		case FPGA_INJ3_MOTOR:
			regbase = FPGA_INJ3_MOTOR_BASE;
			break;
		}


	pos_steps = get_inj_motor_pos_speed(dev_no+5, 0);

	temp_steps = pos_steps - inj_pre_steps[dev_no];
	//temp_steps = pos_steps;

		//int motor:
	if(speed == 0){
		fpga_info(("!!!!speed is zero!!!!"));
		return;
	}
	
	tmp_speed = 520 - 11250/speed;

	fpga_info(("%s: speed:%d  real_speed:%d pos:%d    pos_steps:%d  pre_pos_step:%d  real_steps:%d\n", __func__, speed, tmp_speed, pos, pos_steps, main_pre_steps[dev_no],  temp_steps));

	
	if(temp_steps > 0)
		dir = 0;
	else {
		temp_steps = abs(temp_steps);
		dir = 1;
	}
	
	motor_hold = 0x2;
	index_start = tmp_speed/4;
	chg_point = temp_steps/8;
	chg_speed =  tmp_speed/8;


	//set step max: 0x0
	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_STEP_MAX, temp_steps);
	//set step add 0x2
	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_HOLD, motor_hold);
	//set step add 0x6
	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_DIR, dir);

	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_PHOTO_NUM, 9);
	
	//fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_ACC_INTER, 0x1);

	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_INDEX_START, index_start);

	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_SPEED, 15);


	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_MICROSTEP, 0x1);


	//fpga_write_reg(g_fpga_dev, reg+FPGA_MOTOR_IGNORE, ignor_step);

	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_MODE, 0x1);


// set dec motor speed:
	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_DEC_INTER, 0x1);

	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_CHANGE_POINT, chg_point );

	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_CHANGE_SPEED, 15 );
	
	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_SPD_TAB15, tmp_speed );

//enable motor:
	fpga_write_reg(g_fpga_dev, regbase+FPGA_MOTOR_RUN, 0x1);
	inj_current_pos = pos;
	inj_pre_steps[dev_no] = pos_steps;

	return 0;

}




void fpga_ctrltype_set_inject_move(struct motor_info *pmi)
{

	fpga_info(("%s: id:%d  speed:%d m_pos_target:%d  m_pos_current:%d  steps:%d pmi->m_type:%d", __func__, pmi->m_id, pmi->m_speed, pmi->m_pos_target,  pmi->m_pos_current, pmi->m_steps,   pmi->m_type));

	if(pmi->m_type == 1)
		inj_motor_step_set(pmi->m_id, pmi->m_speed, pmi->m_steps, pmi->m_pos_target);
	else if(pmi->m_type == 2)
 		inj_motor_pos_set(pmi->m_id, pmi->m_speed, pmi->m_pos_target);
	else 	if(pmi->m_type == 3){
		comm_inj_motor_init(pmi->m_id, pmi->m_speed);
	}else 	if(pmi->m_type == 4){
		update_inj_pos_steps(pmi->m_id, pmi->m_pos_target );
	}

	
}

void fpga_ctrltype_get_ts_name_list(void)
{
	fpga_info(("%s", __func__));
}



void fpga_ctrltype_set_sampler_move(struct motor_info *pmi)
{
 	printk("%s\n", __func__);
}

/*
SUBTYPE_OTHERCTRL_GET_VALVE = 1000,
SUBTYPE_OTHERCTRL_GET_PUMP,
SUBTYPE_OTHERCTRL_GET_TEMP,
SUBTYPE_OTHERCTRL_GET_PRESS,
SUBTYPE_OTHERCTRL_GET_MOTOR,
SUBTYPE_OTHERCTRL_GET_INJECT,
SUBTYPE_OTHERCTRL_GET_TS_NAME_LIST,
SUBTYPE_OTHERCTRL_GET_ALL_SENSOR,

SUBTYPE_OTHERCTRL_SET_VALVE,
SUBTYPE_OTHERCTRL_SET_PUMP,
SUBTYPE_OTHERCTRL_SET_TEMP,
SUBTYPE_OTHERCTRL_SET_PRESS,
SUBTYPE_OTHERCTRL_SET_MOTOR_MOVE,
SUBTYPE_OTHERCTRL_SET_INJECT_MOVE,

SUBTYPE_OTHERCTRL_SET_SAMPLER_MOVE,
*/
void netlink_other_ctrl_type_handle(char *ctrldata, int ctrl_type)
{

	struct range_info *pr_info = (struct range_info *)ctrldata;
	struct motor_info *pm_info = (struct motor_info *)ctrldata;
	struct sensor_info *ps_info = (struct sensor_info *)ctrldata;
	int i;
	
	switch(ctrl_type){
// CTRL TYPE FOR GET INFO:		
		case SUBTYPE_OTHERCTRL_GET_VALVE:
			fpga_ctrltype_get_valve(ps_info);	
			encode_tlv_netlink_send_other_rpt(MSG_TYPE_OTHER_REPORT, SUBTYPE_OTHERREPORT_INFO_VALVE, OTHERREPORT_THIRD_TYPE_SENSOR_INFO,(struct sensor_info *)ps_info);
			break;
		case SUBTYPE_OTHERCTRL_GET_PUMP:
			fpga_ctrltype_get_pump(ps_info);	
			encode_tlv_netlink_send_other_rpt(MSG_TYPE_OTHER_REPORT, SUBTYPE_OTHERREPORT_INFO_PUMP, OTHERREPORT_THIRD_TYPE_SENSOR_INFO,(struct sensor_info *)ps_info);
			break;
		case SUBTYPE_OTHERCTRL_GET_ALL_SENSOR:
			fpga_ctrltype_get_all_sensor(ps_info);	
			encode_tlv_netlink_send_other_rpt(MSG_TYPE_OTHER_REPORT, SUBTYPE_OTHERREPORT_INFO_SENSOR, OTHERREPORT_THIRD_TYPE_SENSOR_INFO,(struct sensor_info *)ps_info);
			break;				
		case SUBTYPE_OTHERCTRL_GET_TEMP:
			fpga_ctrltype_get_temp(pr_info);	
			encode_tlv_netlink_send_other_rpt(MSG_TYPE_OTHER_REPORT, SUBTYPE_OTHERREPORT_INFO_TEMP, OTHERREPORT_THIRD_TYPE_RANGE_INFO,(struct range_info *)pr_info);
			break;
		case SUBTYPE_OTHERCTRL_GET_PRESS:
			fpga_ctrltype_get_press(pr_info);	
			encode_tlv_netlink_send_other_rpt(MSG_TYPE_OTHER_REPORT, SUBTYPE_OTHERREPORT_INFO_PRESS, OTHERREPORT_THIRD_TYPE_RANGE_INFO,(struct range_info *)pr_info);
			break;
		case SUBTYPE_OTHERCTRL_GET_MOTOR:
			fpga_ctrltype_get_motor_info(pm_info);	
			encode_tlv_netlink_send_other_rpt(MSG_TYPE_OTHER_REPORT, SUBTYPE_OTHERREPORT_INFO_MOTOR, OTHERREPORT_THIRD_TYPE_MOTOR_INFO,(struct motor_info *)pm_info);
			break;

		case SUBTYPE_OTHERCTRL_GET_INJECT:
			fpga_ctrltype_get_inject_info(pm_info);	
			encode_tlv_netlink_send_other_rpt(MSG_TYPE_OTHER_REPORT, SUBTYPE_OTHERREPORT_INFO_INJECT, OTHERREPORT_THIRD_TYPE_MOTOR_INFO,(struct motor_info *)pm_info);
			break;
		case SUBTYPE_OTHERCTRL_GET_TS_NAME_LIST:
			fpga_ctrltype_get_ts_name_list();			
			encode_tlv_netlink_send_other_rpt(MSG_TYPE_OTHER_REPORT, SUBTYPE_OTHERREPORT_INFO_TS_NAME_LIST, OTHERREPORT_THIRD_TYPE_MOTOR_INFO,(struct motor_info *)pm_info);
			break;	
// CTRL TYPE FOR SET INFO:
		case SUBTYPE_OTHERCTRL_SET_VALVE:
			fpga_ctrltype_set_valve(ps_info);	
			break;

		case SUBTYPE_OTHERCTRL_SET_PUMP:
			fpga_ctrltype_set_pump(ps_info);	
			break;

		case SUBTYPE_OTHERCTRL_SET_TEMP:
			for(i=0; i<60;i++)
				printk("%d ", ctrldata[i]);
			printk("\n");
			fpga_ctrltype_set_temp(pr_info);	
			break;
		case SUBTYPE_OTHERCTRL_SET_PRESS:
			fpga_ctrltype_set_press(pr_info);	
			break;
		case SUBTYPE_OTHERCTRL_SET_MOTOR_MOVE:
			fpga_ctrltype_set_motor_move(pm_info);	
			break;
		case SUBTYPE_OTHERCTRL_SET_INJECT_MOVE:
			fpga_ctrltype_set_inject_move(pm_info);	
			break;

		case SUBTYPE_OTHERCTRL_SET_SAMPLER_MOVE:
			fpga_ctrltype_set_sampler_move(pm_info);		
			break;
		default:
			fpga_info(("%s: unsupport type:%d", __func__, ctrl_type));
				
	}

	return;

}






/************************************************************************************************
 * 								SYSFS Attributes
 ************************************************************************************************/
/* use echo  > fpga_reg_set to set fpga reg */
ssize_t fpga_netlink_cmd_set(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
		struct fpga_dev *priv = dev_get_drvdata(dev);
		char msg[20];

		if(strncmp(buf, "trigger",7) == 0)
			netlink_send_timing_cmd();
		else
			fpga_netlink_send(buf, count, netlink_port);
	
		return count;
}


ssize_t gpmc_fpga_reg_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct fpga_dev *priv = dev_get_drvdata(dev);
	int ret,regval;
	int i=100;
	

	regval = fpga_read_reg(priv, debug_fpga_reg);

	return  sprintf(buf,  "0x%x\n", regval);
}

/* use echo reg val > fpga_reg_set to set fpga reg */
ssize_t gpmc_fpga_reg_set(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct fpga_dev *priv = dev_get_drvdata(dev);
	unsigned long enable;
	u32 ret,reg,val;
	char strreg[11],strval[11];

	ret = sscanf(buf,  "%s %s", strreg, strval);
	
	reg = simple_strtol(strreg, NULL, 0);
	if (reg < 0)
		return reg;

	val = simple_strtol(strval, NULL, 0);
	if (val < 0)
		return val;
	
	fpga_write_reg(priv, reg, val);	

	debug_fpga_reg = reg;
	
	return count;
}

ssize_t gpmc_dumpreg_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct fpga_dev *priv = dev_get_drvdata(dev);
	u16 ret,regval;
	int i=0,len=0;


	for(i = 0; i< 0x40; i=i+2){
		regval = fpga_read_reg(priv, i);
		len += snprintf(buf+len,20,"reg0x%x :0x%x\n",i, regval);
	}
	
	for(i = 0x10000; i< 0x10024; i=i+2){
		regval = fpga_read_reg(priv, i);
		usleep_range(1000,1000);
		len += snprintf(buf+len,20,"reg0x%x :0x%x\n",i, regval);
	}

	
	regval = fpga_read_reg(priv, 0x20000);
	len += snprintf(buf+len,20,"reg0x%x :0x%x\n",0x20000, regval);

	regval = fpga_read_reg(priv, 0x20002);
	len += snprintf(buf+len,20,"reg0x%x :0x%x\n",0x20002, regval);

	regval = fpga_read_reg(priv, 0x20004);
	len += snprintf(buf+len,20,"reg0x%x :0x%x\n",0x20004, regval);

	for(i = 0x30000; i< 0x30034; i=i+2){
		regval = fpga_read_reg(priv, i);
		len += snprintf(buf+len,20,"reg0x%x :0x%x\n",i, regval);

	}

	for(i = 0x40000; i< 0x40020; i=i+2){
		regval = fpga_read_reg(priv, i);
		len += snprintf(buf+len,20,"reg0x%x :0x%x\n",i, regval);

	}


	for(i = 0x70000; i< 0x70026; i=i+2){
		regval = fpga_read_reg(priv, i);
		len += snprintf(buf+len,20,"reg0x%x :0x%x\n",i, regval);
	}

	
	for(i = 0x80000; i< 0x80030; i=i+2){
		regval = fpga_read_reg(priv, i);
		len += snprintf(buf+len,20,"reg0x%x :0x%x\n",i, regval);
	}
	
	return  len;
}

ssize_t gpmc_dump_motor1_reg_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct fpga_dev *priv = dev_get_drvdata(dev);
	u16 ret,regval;
	int i=0,len=0;

	regval = fpga_read_reg(priv, 0x50000);
	len += snprintf(buf+len,20,"reg0x%x :0x%x\n",0x50000, regval);

	for(i = 0x51000; i< 0x51088; i=i+2){
		regval = fpga_read_reg(priv, i);
		len += snprintf(buf+len,20,"reg0x%x :0x%x\n",i, regval);

	}


	for(i = 0x52000; i< 0x52088; i=i+2){
		regval = fpga_read_reg(priv, i);
		len += snprintf(buf+len,20,"reg0x%x :0x%x\n",i, regval);

	}

	for(i = 0x53000; i< 0x53088; i=i+2){
		regval = fpga_read_reg(priv, i);
		len += snprintf(buf+len,20,"reg0x%x :0x%x\n",i, regval);

	}



	return len;

}


ssize_t gpmc_dump_motor2_reg_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{

	struct fpga_dev *priv = dev_get_drvdata(dev);
	u16 ret,regval;
	int i=0,len=0;


	for(i = 0x54000; i< 0x54088; i=i+2){
		regval = fpga_read_reg(priv, i);
		len += snprintf(buf+len,20,"reg0x%x :0x%x\n",i, regval);

	}

	for(i = 0x55000; i< 0x55088; i=i+2){
		regval = fpga_read_reg(priv, i);
		len += snprintf(buf+len,20,"reg0x%x :0x%x\n",i, regval);

	}

	for(i = 0x56000; i< 0x56088; i=i+2){
		regval = fpga_read_reg(priv, i);
		len += snprintf(buf+len,20,"reg0x%x :0x%x\n",i, regval);

	}	

	return len;

}


ssize_t gpmc_dump_motor3_reg_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{

	struct fpga_dev *priv = dev_get_drvdata(dev);
	u16 ret,regval;
	int i=0,len=0;



	for(i = 0x57000; i< 0x57088; i=i+2){
		regval = fpga_read_reg(priv, i);
		len += snprintf(buf+len,20,"reg0x%x :0x%x\n",i, regval);

	}	


	for(i = 0x58000; i< 0x58088; i=i+2){
		regval = fpga_read_reg(priv, i);
		len += snprintf(buf+len,20,"reg0x%x :0x%x\n",i, regval);
	}	

	return len;


}




/************************* motor control  start***********************/

//echo num  enable > valve
ssize_t sys_valve_ctrl_set(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	int ret;
	int dev_no, enable;
	
	char str1[10],str2[10], str3[10], str4[10],  str5[10];

	ret = sscanf(buf,  "%s %s", str1, str2);	

	dev_no = simple_strtol(str1, NULL, 0);
	if (dev_no < 0)
		return dev_no;

	enable = simple_strtol(str2, NULL, 0);
	if (enable < 0)
		return enable;

	fpga_set_valve(dev_no, 0, enable, 0,0, 0, 0,(void *)0);

	return count;	

}

ssize_t sys_pump_ctrl_set(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	int ret;
	int dev_no, enable;
	
	char str1[10],str2[10];

	ret = sscanf(buf,  "%s %s", str1, str2);	

	dev_no = simple_strtol(str1, NULL, 0);
	if (dev_no < 0)
		return dev_no;

	enable = simple_strtol(str2, NULL, 0);
	if (enable < 0)
		return enable;

	fpga_set_pump(dev_no, 0, enable, 0,0, 0, 0,(void *)0);

	return count;	

}


void fpga_set_nega_press(int min, int max)
{
	fpga_write_reg(g_fpga_dev, FPGA_IOCTRL_REG_BASE, 1);

	fpga_write_reg(g_fpga_dev, FPGA_NEGA_PRESSURE_MIN, min);
	fpga_write_reg(g_fpga_dev, FPGA_NEGA_PRESSURE_MAX, max);
	return;
}

//echo num  enable > valve
ssize_t sys_nega_pressure_set(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	int ret;
	int min, max;
	
	char str1[10],str2[10];

	ret = sscanf(buf,  "%s %s", str1, str2);	

	min = simple_strtol(str1, NULL, 0);
	if (min < 0)
		return min;

	max = simple_strtol(str2, NULL, 0);
	if (max < 0)
		return max;

	fpga_setup_vac(0, 0, 0, min, max, 0, 0, (void *)0);

	return count;	

}



//echo dev_no speed steps > motor_ctrl
ssize_t sys_motor_ctrl_set(struct device *dev, struct device_attribute *attr,
	   				const char *buf, size_t count)
{
	int ret;
	int dev_no, speed, pos, dir, enable;
	char str1[10],str2[10], str3[10], str4[10],  str5[10];

	ret = sscanf(buf,  "%s %s %s", str1, str2, str3);	

	dev_no = simple_strtol(str1, NULL, 0);
	if (dev_no < 0)
		return dev_no;

	speed = simple_strtol(str2, NULL, 0);
	if (speed < 0)
		return speed;

	pos = simple_strtol(str3, NULL, 0);
	if (pos < 0)
		return pos;
	
	comm_main_motor_set( dev_no, speed, pos);

	return count;

}


ssize_t sys_motor_init(struct device *dev, struct device_attribute *attr,
	   				const char *buf, size_t count)
{

	int ret;
	int dev_no, speed;
	char str1[10],str2[10];

	ret = sscanf(buf,  "%s %s", str1, str2);	

	dev_no = simple_strtol(str1, NULL, 0);
	if (dev_no < 0)
		return dev_no;

	speed = simple_strtol(str2, NULL, 0);
	if (speed < 0)
		return speed;

	
	comm_motor_init(dev_no, speed);

	return count;

}


/************************* motor control  start***********************/


//echo dev_no speed step dir enable > motor_ctrl
ssize_t sys_inject_motor_ctrl(struct device *dev, struct device_attribute *attr,
	   				const char *buf, size_t count)
{
	int ret;
	long dev_no, speed, steps, dir, enable;
	char str1[10],str2[10], str3[10], str4[10],  str5[10];
	long tmpval = -300;

	ret = sscanf(buf,  "%s %s %s", str1, str2, str3);	

	dev_no = simple_strtoul(str1, NULL, 0);
	if (dev_no < 0)
		return dev_no;

	speed = simple_strtoul(str2, NULL, 0);
	if (speed < 0)
		return speed;

	if(*str3 == '-') {
		steps = simple_strtoul(str3+1, NULL, 0);
		steps= -steps;
	}else 
		steps = simple_strtoul(str3, NULL, 0);


	comm_inj_motor_set( dev_no, speed, steps);

	return count;

}


ssize_t sys_inject_motor_init(struct device *dev, struct device_attribute *attr,
	   				const char *buf, size_t count)
{

	int ret;
	int dev_no, speed;
	char str1[10],str2[10];

	ret = sscanf(buf,  "%s %s", str1, str2);	

	dev_no = simple_strtol(str1, NULL, 0);
	if (dev_no < 0)
		return dev_no;

	speed = simple_strtol(str2, NULL, 0);
	if (speed < 0)
		return speed;

	comm_inj_motor_init(dev_no, speed);

	return count;

}



ssize_t sys_update_motor_pos_steps(struct device *dev, struct device_attribute *attr,
	   				const char *buf, size_t count)
{
	int ret;
	int val, row, col;
	char str1[10],str2[10], str3[10];

	ret = sscanf(buf,  "%s %s %s", str1, str2, str3);	

	val = simple_strtol(str1, NULL, 0);
	if (val < 0)
		return val;

	row = simple_strtol(str2, NULL, 0);
	if (row < 0)
		return row;

	col = simple_strtol(str3, NULL, 0);
	if (col < 0)
		return col;
	


	if(val == 1)
		load_motor_pos_steps();
	else
		update_motor_pos_steps(row, col);
	
	return count;

}




ssize_t sys_get_fifo_rbc(struct device *dev, struct device_attribute *attr,
	   				const char *buf, size_t count)
{

	int ret,i;

	struct measure_related_info  mri;

	char str1[10];
	int time;
	
	ret = sscanf(buf,  "%s", str1);	

	time = simple_strtol(str1, NULL, 0);
	if (time < 0)
		return time;

	mri.m_ts_id=0x3344;

	fpga_get_fifo_rbc(2,time,1,0,0,0,0, &mri);


	printk("rbc time:%d\n" ,time );
	return count;
}


ssize_t sys_get_fifo_wbc(struct device *dev, struct device_attribute *attr,
	   				const char *buf, size_t count)
{

	int ret,i;

	struct measure_related_info  mri;


	char str1[10];
	int time;
	
	ret = sscanf(buf,  "%s", str1);	

	time = simple_strtol(str1, NULL, 0);
	if (time < 0)
		return time;

	mri.m_ts_id=0x3344;

	fpga_get_fifo_wbc(2,time,1,0,0,0,0, &mri);

	printk("wbc time:%d\n" ,time );
		
	return count;
}

ssize_t sys_get_fifo_diff(struct device *dev, struct device_attribute *attr,
	   				const char *buf, size_t count)
{

	int ret;

	struct measure_related_info  mri;

	//fpga_get_hgb_blank(2,1000,1,0,0,0,0, &mri);
		
	//msleep(10000);

	
	char str1[10];
	int time;
	
	ret = sscanf(buf,  "%s", str1);	

	time = simple_strtol(str1, NULL, 0);
	if (time < 0)
		return time;


	fpga_get_fifo_diff(2,time,1,0,0,0,0, &mri);
	
	return count;
}


ssize_t sys_cpu_read_rbc(struct device *dev, struct device_attribute *attr,
	   				const char *buf, size_t count)
{

	int i;
	u16 val;
	u16 tmpbuf[1024];
	struct timespec ts1, ts3;

	printk("start read\n");
	
	getnstimeofday(&ts1);	
	for(i = 0; i<1024; i++){
		val = fpga_read_reg(g_fpga_dev,0x10022);
		tmpbuf[i] = val;
	}
		getnstimeofday(&ts3);	
	printk(" cpu dma time diff =%ld us  \n",  ((ts3.tv_sec*1000*1000 + ts3.tv_nsec/1000 ) -  (ts1.tv_sec*1000*1000 + ts1.tv_nsec/1000 )));
	


	printk("start end\n");	

	for(i = 0; i<1024; i++){
		printk("0x%x ", tmpbuf[i]);
	}

	

		
	return count;
}

ssize_t sys_trigger_rbc_dma(struct device *dev, struct device_attribute *attr,
	   				const char *buf, size_t count)
{
	//memset(g_fpga_dev->rbcout_per_buf, 0,   FIFO_DATA_SEND_MAX_SIZE *sizeof(u16));
	//memset(g_fpga_dev->rbcout_addr, 0,   SYS_DATA_DMA_SIZE *sizeof(u16));

	//gpmc_fpga_fifo_init(g_fpga_dev);
	//fpga_get_fifo_rbc(2,5000,1,0,0,0,0, (void *)0);
	int i;

	memset(g_fpga_dev->wbcout_addr, 0,   2*FIFO_DATA_DMA_SIZE);
	memset(g_fpga_dev->rbcout_addr, 0,   2*FIFO_DATA_DMA_SIZE);

	for(i = 0; i< 5000000;i++)
		((u32 *)((unsigned int)g_fpga_dev->wbcout_addr))[i] = i+4;

	printk("rbc buf:\n");
	
	//for(i = 0; i< 1000000;i++)
		//printk("%u " ,((u32 *)((unsigned int)g_fpga_dev->wbcout_addr))[i] );
	
	printk("\n");


	queue_work(g_fpga_dev->wqfifo,&g_fpga_dev->fifo_rbc); //read wq to handle dma data;
	return count;
}



