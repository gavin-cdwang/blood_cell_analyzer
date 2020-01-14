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


#ifndef __DRIVERS_GPMC_FPGA_H
#define __DRIVERS_GPMC_FPGA_H
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


//#define printf(format, args...) printk("ant-fpga: %s(): " format, __func__, ## args)
#define	printf(fmt, args...)	printk(fmt , ## args);printk("\n")

/* message levels */
#define FPGA_ERROR_VAL	(0x0001)
#define FPGA_WARN_VAL	(0x0002 | FPGA_ERROR_VAL)
#define FPGA_NOTICE_VAL	(0x0004 | FPGA_WARN_VAL)
#define FPGA_INFO_VAL	(0x0008 | FPGA_NOTICE_VAL)

#define fpga_error(args)		do {if (fpga_msg_level & FPGA_ERROR_VAL) printf args;} while (0)
#define fpga_warn(args)		do {if (fpga_msg_level & FPGA_WARN_VAL) printf args;} while (0)
#define fpga_notice(args)		do {if (fpga_msg_level & FPGA_NOTICE_VAL) printf args;} while (0)
#if 0
#define fpga_info(args)			do {if (fpga_msg_level & FPGA_INFO_VAL) printf args;} while (0)
#else
#define fpga_info(args)			do { } while (0)
#endif

/* The crypto netlink socket */

#define NETLINK_FPGA 30
//static struct sock *fpga_nlsk = NULL;

#define DMA_PER_TRANSFER_BUF  3000// fifo 2000*2;
#define FIFO_DATA_SEND_MAX_SIZE    1000*30 //1000*150 //7000 // 2048*1024 //max fifo size


#define FIFO_DATA_DMA_SIZE 3000*1024//3000*1024//2048 //max fifo size

#define SYS_DATA_DMA_SIZE  2000//2048 //max fifo size

#define FPGA_INT_MASK_SYS 0x00040002
#define FPGA_INT_SOURCE_SYS 0x00040004
#define FPGA_INT_SOURCE_SYS_BIT0 1<<0 //sysrd empty
#define FPGA_INT_SOURCE_SYS_BIT1 1<<1 //sysrd full
#define FPGA_INT_SOURCE_SYS_BIT2 1<<2 //sysrd half full ,valid

#define FPGA_SYS_INFO_LEN 0x00000030
#define FPGA_SYS_FIFO_ADDR 0x00000032



struct fpga_dev {
	/* character device */
	struct miscdevice miscdev;
	struct device *dev;
	struct mutex mutex;

	/* reference count */
	struct kref ref;

	/* FPGA registers and information */

	/* fpga base address */
	void __iomem *regs; 
	int irq;
	int rbc_irq, wbc_irq, diff_irq;
	int fpga_rst;
	int fpga_ready;
	
		
	/* FPGA Physical Address/Size Information */
	resource_size_t phys_addr;
	size_t phys_size;


	/* Protection for all members below */
	spinlock_t lock;

	/* Device enable/disable flag */
	bool enabled;

	/* Correlation data buffers */
	//wait_queue_head_t wait;
	struct list_head free;
	struct list_head used;
	struct data_buf *inflight;

	size_t bufsize;
	struct dentry *dbg_entry;

	wait_queue_head_t fpga_rq;
	struct workqueue_struct *wqfifo;
	struct work_struct fifo_sys;
	struct work_struct fifo_wbc;	
	struct work_struct fifo_rbc;
	struct work_struct fifo_diff;
	struct work_struct sys_float;
		
	atomic_t fpga_scheduled;

	//dma info 
	struct dma_chan *fpga_rbc_dma_chan;
	struct dma_chan *fpga_wbc_dma_chan;
	struct dma_chan *fpga_diff_dma_chan;
	//struct dma_slave_config dma_sconfig;

	u16 *sys_hgb_addr;	
	u16 *sys_rbcmon_addr;
	u16 *sys_wbcmon_addr;
	u16 *sys_a56v_addr;
	u16 *sys_a12v_addr;
	u16 *sys_an12v_addr;
	u16 *sys_p24v_addr;
	u16 *sys_liquidpess_addr;
	u16 *sys_airpress_addr;
	u16 *sys_liq1_addr;
	u16 *sys_liq2_addr;
	u16 *sys_liq3_addr;

	u16 *sys_pmt_hv_addr;
	u16 *sys_ld_curt_addr;
	u16 *sys_temp1_addr;
	u16 *sys_temp2_addr;
	u16 *sys_temp3_addr;

	void *rbcout_addr;
	void *wbcout_addr;
	void *diffout_addr;

	
	dma_addr_t sys_hgb_dma_addr;	
	dma_addr_t sys_rbcmon_dma_addr;
	dma_addr_t sys_wbcmon_dma_addr;
	dma_addr_t sys_a56v_dma_addr;
	dma_addr_t sys_a12v_dma_addr;
	dma_addr_t sys_an12v_dma_addr;
	dma_addr_t sys_p24v_dma_addr;
	dma_addr_t sys_liquidpess_dma_addr;
	dma_addr_t sys_airpress_dma_addr;
	dma_addr_t sys_liq1_dma_addr;
	dma_addr_t sys_liq2_dma_addr;
	dma_addr_t sys_liq3_dma_addr;
	dma_addr_t sys_pmt_hv_dma_addr;
	dma_addr_t sys_ld_curt_dma_addr;
	dma_addr_t sys_temp1_dma_addr;
	dma_addr_t sys_temp2_dma_addr;
	dma_addr_t sys_temp3_dma_addr;

	dma_addr_t rbcout_dma_addr;
	dma_addr_t wbcout_dma_addr;
	dma_addr_t diffout_dma_addr;

#if 0
	u16 sys_hgb_per_buf[2*SYS_DATA_DMA_SIZE];	
	u16 sys_rbcmon_per_buf[2*SYS_DATA_DMA_SIZE];
	u16 sys_wbcmon_per_buf[2*SYS_DATA_DMA_SIZE];
	u16 sys_a56v_per_buf[2*SYS_DATA_DMA_SIZE];
	u16 sys_a12v_per_buf[2*SYS_DATA_DMA_SIZE];
	u16 sys_an12v_per_buf[2*SYS_DATA_DMA_SIZE];
	u16 sys_p24v_per_buf[2*SYS_DATA_DMA_SIZE];
	u16 sys_liquidpess_per_buf[2*SYS_DATA_DMA_SIZE];
	u16 sys_airpress_per_buf[2*SYS_DATA_DMA_SIZE];
	u16 sys_liq1_per_buf[2*SYS_DATA_DMA_SIZE];
	u16 sys_liq2_per_buf[2*SYS_DATA_DMA_SIZE];
	u16 sys_liq3_per_buf[2*SYS_DATA_DMA_SIZE];

	u16 sys_pmt_hv_per_buf[2*SYS_DATA_DMA_SIZE];
	u16 sys_ld_curt_per_buf[2*SYS_DATA_DMA_SIZE];
	u16 sys_temp1_per_buf[2*SYS_DATA_DMA_SIZE];
	u16 sys_temp2_per_buf[2*SYS_DATA_DMA_SIZE];
	u16 sys_temp3_per_buf[2*SYS_DATA_DMA_SIZE];
#endif
	
	//u16 wbcout_per_buf[2*SYS_DATA_DMA_SIZE];
	//u16 *rbcout_per_buf;

	//u16 opt_fsout_per_buf[2*SYS_DATA_DMA_SIZE];
	//u16 opt_ssout_per_buf[2*SYS_DATA_DMA_SIZE];
	//u16 opt_sfout_per_buf[2*SYS_DATA_DMA_SIZE];
	

	//inject motor set
	struct hrtimer m_inj1_timer;  //inject motor timer
	struct hrtimer m_inj2_timer; //inject init motor timer
	struct hrtimer m_inj3_timer; //inject init motor timer

	//main motor set	
	struct hrtimer m_samph_timer;  //main motor timer
	struct hrtimer m_sampv_timer;   //main init motor timer

	struct hrtimer m_mixh_timer;   //sample motor timer
	struct hrtimer m_mixv_timer;   //sample motor timer
	struct hrtimer m_mixr_timer;   //sample motor timer

	//sys chan hgb,rbc,wbc,temp timer;
	struct hrtimer sys_hgb_timer;
	struct hrtimer sys_rbc_timer;
	struct hrtimer sys_wbc_timer;
	struct hrtimer  sys_liqpress_timer;
	struct hrtimer  sys_airpress_timer;
	struct hrtimer  sys_liq1_timer;
	struct hrtimer  sys_pmt_timer;
	struct hrtimer  sys_ld_timer;
	struct hrtimer  sys_temp1_timer;
	struct hrtimer  sys_temp2_timer;

	struct hrtimer  fifo_rbc_timer;
	struct hrtimer  fifo_wbc_timer;
	struct hrtimer  fifo_diff_timer;

	atomic_t m_inj1_t; //inject motor1 work done interrupt 
	atomic_t m_inj2_t; // inject motor2 work done interrupt 
	atomic_t m_inj3_t; //inject motor3 work done interrupt 
	atomic_t m_samph_t; //sample motor work done interrupt
	atomic_t m_sampv_t; //sample motor work done interrupt
	atomic_t m_mixh_t;  //mix pool h
	atomic_t m_mixv_t; //mix pool  v
	atomic_t m_mixr_t; //mix pool 

	atomic_t sys_hgb_t;
	atomic_t sys_rbc_t;	
	atomic_t sys_wbc_t;
	atomic_t sys_liqpress_t;
	atomic_t sys_airpress_t;
	atomic_t sys_liq1_t;
	atomic_t sys_pmt_t;
	atomic_t sys_ld_t;
	atomic_t sys_temp1_t;
	atomic_t sys_temp2_t;

	atomic_t fifo_rbc_t;
	atomic_t fifo_wbc_t;
	atomic_t fifo_diff_t;
	
	
	int int_mask_flag;
	int test_flag;

	int sche_wa_flga; //workaround flag to call worker;
	struct sock *fpga_nlsk;

	struct measure_related_info * pmri;   //for wbc,rbc, diff fifo data report;
};

struct func_ts_timer {

	struct hrtimer ts_timer;  //inject motor timer
	struct completion *ts_complete;
};


enum FPGA_DATA_TYPE
{
	FPGA_SYS_DATA,
	FPGA_HGB_DATA,
	FPGA_WBC_DATA,
	FPGA_RBC_DATA,
};


u16 fpga_read_reg(struct fpga_dev *priv, unsigned int reg);

u16 fpga_write_reg(struct fpga_dev *priv, unsigned int reg, u16 val);

int fpga_netlink_send(char *toubuf, u32 len, u32 port);


void netlink_send_timing_cmd(void);

void fpga_enable_irq(struct fpga_dev *priv);
void fpga_disable_irq(struct fpga_dev *priv);

int get_func_ts_id_index(int func_ts_id);
int fpga_exec_functs_thread(void *p);


#endif
