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
#include <linux/irq.h>

#include <linux/kthread.h>
#include "gpmc_fpga.h"
#include "fpga_timing_parser.h"
#include "fpga_interface.h"

/* system controller registers */

extern int fpga_msg_level;
extern struct timing_exec_hdr g_tehdr;
char *tdata_path = "/home/ant/examples/usermod/timingparse/timseq.bin";
extern struct timing_exec_hdr *g_func_tshdr;
extern int g_func_num;
dma_cookie_t cookie;
static volatile int irqraised1;
int rbc_fisrt_flag = 1;
int rbc_per_len=0;
int rbc_end_rpt_flag = 0;
int rbc_total = 0;
int rbc_irq = 0;
int rbc_xfer_len[6000];
int rbc_cpy_time[6000];
int cpycnt;


int wbc_fisrt_flag = 1;
int wbc_per_len=0;
int wbc_end_rpt_flag = 0;
int wbc_total = 0;
int wbc_irq = 0;
int wbc_xfer_len[6000];

int diff_fisrt_flag = 1;
int diff_per_len=0;
int diff_end_rpt_flag = 0;
int diff_total = 0;
int diff_irq = 0;
int diff_xfer_len[6000];

int rbctime[8000];





int netlink_port = 0;

extern struct dma_chan *of_dma_request_slave_channel(struct device_node *np,
					      const char *name);


/* FPGA registers */
#define GPMC_FPGA_VERSION 0x00

struct task_struct **ts_thread1,*ts_thread2;
struct task_struct **func_ts_thread;

extern struct fpga_ctrl_fp fpga_ctrl[];
extern struct fpga_moni_fp fpga_moni[];

int debug_fpga_reg;

static const char drv_name[] = "gpmc-fpga";

struct fpga_dev *g_fpga_dev;




//sysdata  dma info


static u16 sysinfo_len;



//static DECLARE_COMPLETION(sysinfo_wait);

struct completion rbcinfo_wait;
struct completion wbcinfo_wait;
struct completion diffinfo_wait;

//rbcdata dma info
static u16 *rbcdata_buf;
static dma_addr_t rbcdata_buf_phys;
static u16 rbcdata_len;


//static  atomic_t fpga_scheduled;
static spinlock_t fpga_rlock;

static DEFINE_MUTEX(fpga_mutex);


u16 fpga_read_reg(struct fpga_dev *priv, unsigned int reg)
{
	return ioread16(priv->regs + 2*reg);
}

u16 fpga_write_reg(struct fpga_dev *priv, unsigned int reg, u16 val)
{
	iowrite16(val, priv->regs + 2*reg);
}

static void gpmc_fpga_device_release(struct kref *ref)
{
	struct fpga_dev *priv = container_of(ref, struct fpga_dev, ref);

	/* the last reader has exited, cleanup the last bits */
	mutex_destroy(&priv->mutex);
	kfree(priv);
}

static void fpag_set_irq_mask(struct fpga_dev *priv, int disable)
{
	u16 reg_val;

	
	reg_val = fpga_read_reg(priv, FPGA_INTER_RBC_MASK);

	if(disable) {		
		reg_val |= BIT_MASK(INTERRUPT_RBC_MASK_NORMAL_INT_BIT_6) | BIT_MASK(INTERRUPT_RBC_MASK_END_INT_BIT_7);
	}else
		reg_val &= ~(BIT_MASK(INTERRUPT_RBC_MASK_NORMAL_INT_BIT_6) | BIT_MASK(INTERRUPT_RBC_MASK_END_INT_BIT_7));

	fpga_write_reg(priv,FPGA_INTER_RBC_MASK,reg_val); 	

	
}

static void fpag_set_irq_source(struct fpga_dev *priv, int disable)
{
	u16 reg_val;

	
	reg_val = fpga_read_reg(priv, FPGA_INTER_RBC_SOURCE);

	if(disable) {		
		reg_val |= BIT_MASK(INTERRUPT_RBC_SOURCE_NORMAL_INT_BIT_6) | BIT_MASK(INTERRUPT_RBC_SOURCE_END_INT_BIT_7);
	}else
		reg_val &= ~(BIT_MASK(INTERRUPT_RBC_SOURCE_NORMAL_INT_BIT_6) | BIT_MASK(INTERRUPT_RBC_SOURCE_END_INT_BIT_7));

	fpga_write_reg(priv,FPGA_INTER_RBC_SOURCE,reg_val); 	

	
}


void fpga_enable_irq(struct fpga_dev *priv)
{
	fpag_set_irq_mask(priv,0);

}

void fpga_disable_irq(struct fpga_dev *priv)
{
	fpag_set_irq_mask(priv,1);
	//fpag_set_irq_source(priv,1);
}


struct timespec tm1, tm2, tm3, tm4,tm5,tm6;
int lenarr[10],timearr[10], difftm1, difftm2,diffcpy;
int intcnt = 0;


static int gpmc_fpga_debug_show(struct seq_file *f, void *offset)
{
	struct fpga_dev *priv = f->private;

	spin_lock_irq(&priv->lock);

	seq_printf(f, "enabled: %d\n", priv->enabled);


	spin_unlock_irq(&priv->lock);
	return 0;
}

static int gpmc_fpga_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, gpmc_fpga_debug_show, inode->i_private);
}

static const struct file_operations gpmc_fpga_debug_fops = {
	.owner		= THIS_MODULE,
	.open		= gpmc_fpga_debug_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int gpmc_fpga_debugfs_init(struct fpga_dev *priv)
{
	priv->dbg_entry = debugfs_create_file(drv_name, S_IRUGO, NULL, priv,
					      &gpmc_fpga_debug_fops);
	return PTR_ERR_OR_ZERO(priv->dbg_entry);
}

static void gpmc_fpga_debugfs_exit(struct fpga_dev *priv)
{
	debugfs_remove(priv->dbg_entry);
}


static DEVICE_ATTR(fpga_reg, S_IWUSR | S_IRUGO, gpmc_fpga_reg_show, gpmc_fpga_reg_set);
static DEVICE_ATTR(netlink_cmd, S_IWUSR | S_IRUGO, NULL, fpga_netlink_cmd_set);
static DEVICE_ATTR(dumpreg, S_IWUSR | S_IRUGO, gpmc_dumpreg_show, NULL);
static DEVICE_ATTR(dump_motor1_reg, S_IWUSR | S_IRUGO, gpmc_dump_motor1_reg_show, NULL);
static DEVICE_ATTR(dump_motor2_reg, S_IWUSR | S_IRUGO, gpmc_dump_motor2_reg_show, NULL);
static DEVICE_ATTR(dump_motor3_reg, S_IWUSR | S_IRUGO, gpmc_dump_motor3_reg_show, NULL);

static DEVICE_ATTR(main_motor_ctrl, S_IWUSR | S_IRUGO, NULL, sys_motor_ctrl_set);
static DEVICE_ATTR(main_motor_init, S_IWUSR | S_IRUGO, NULL, sys_motor_init);

static DEVICE_ATTR(inject_ctrl, S_IWUSR | S_IRUGO, NULL, sys_inject_motor_ctrl);
static DEVICE_ATTR(inject_init, S_IWUSR | S_IRUGO, NULL, sys_inject_motor_init);

static DEVICE_ATTR(pump_ctrl, S_IWUSR | S_IRUGO, NULL, sys_pump_ctrl_set);
static DEVICE_ATTR(valve_ctrl, S_IWUSR | S_IRUGO, NULL, sys_valve_ctrl_set);
static DEVICE_ATTR(nega_press_set, S_IWUSR | S_IRUGO, NULL, sys_nega_pressure_set);

static DEVICE_ATTR(update_motor_pos, S_IWUSR | S_IRUGO, NULL, sys_update_motor_pos_steps);

static DEVICE_ATTR(get_rbc, S_IWUSR | S_IRUGO, NULL, sys_get_fifo_rbc);
static DEVICE_ATTR(get_wbc, S_IWUSR | S_IRUGO, NULL, sys_get_fifo_wbc);
static DEVICE_ATTR(get_diff, S_IWUSR | S_IRUGO, NULL, sys_get_fifo_diff);

static DEVICE_ATTR(cpu_read_rbc, S_IWUSR | S_IRUGO, NULL, sys_cpu_read_rbc);
static DEVICE_ATTR(trigger_rbc_dma, S_IWUSR | S_IRUGO, NULL, sys_trigger_rbc_dma);




static struct attribute *gpmc_fpga_sysfs_attrs[] = {
	&dev_attr_fpga_reg.attr,
	&dev_attr_netlink_cmd.attr,
	&dev_attr_dumpreg.attr,
	&dev_attr_dump_motor1_reg.attr,
	&dev_attr_dump_motor2_reg.attr,
	&dev_attr_dump_motor3_reg.attr,
	&dev_attr_main_motor_ctrl.attr,
	&dev_attr_main_motor_init.attr,
	&dev_attr_inject_ctrl.attr,		
	&dev_attr_inject_init.attr,	
	&dev_attr_valve_ctrl.attr,
	&dev_attr_pump_ctrl.attr,
	&dev_attr_nega_press_set.attr,
	&dev_attr_update_motor_pos.attr,
	&dev_attr_get_rbc.attr,
	&dev_attr_get_wbc.attr,		
	&dev_attr_get_diff.attr,		
	&dev_attr_cpu_read_rbc.attr,	
	&dev_attr_trigger_rbc_dma.attr,	
	NULL,
};

static const struct attribute_group fpga_sysfs_attr_group = {
	.attrs = gpmc_fpga_sysfs_attrs,
};

/*
 * FPGA Realtime Data Character Device
 */


int cnt = 0;

static void fpga_rbc_dma_complete(void *args)
{
	struct fpga_dev *priv = (struct fpga_dev *)args;
	struct dma_chan *chan = priv->fpga_rbc_dma_chan;


	complete(&rbcinfo_wait);
}


static void fpga_wbc_dma_complete(void *args)
{
	struct fpga_dev *priv = (struct fpga_dev *)args;
	struct dma_chan *chan = priv->fpga_wbc_dma_chan;


	complete(&wbcinfo_wait);
}


static void fpga_diff_dma_complete(void *args)
{
	struct fpga_dev *priv = (struct fpga_dev *)args;
	struct dma_chan *chan = priv->fpga_diff_dma_chan;


	complete(&diffinfo_wait);
}


static void fpga_rbc_dma_memcpy_complete(void *args)
{
	struct fpga_dev *priv = (struct fpga_dev *)args;
	struct dma_chan *chan = priv->fpga_rbc_dma_chan;

	printk("fpga_rbc_dma_memcpy_complete\n");
	complete(&rbcinfo_wait);
}




static int fpga_rbc_dma_transfer(struct fpga_dev *priv, dma_addr_t dma_dst,
			     dma_addr_t dma_src, size_t len)
{
	struct dma_chan *chan = priv->fpga_rbc_dma_chan;
	struct dma_device *dma_dev = chan->device;
	dma_addr_t tmp_dma_dst = dma_dst ;
		
	dma_cookie_t cookie;
	enum dma_ctrl_flags flags = DMA_CTRL_ACK | DMA_PREP_INTERRUPT;
	struct dma_async_tx_descriptor *tx = NULL;
	int ret,i;
	int xfer_len=0;
	
	struct dma_slave_config sconf;


	sconf.direction = DMA_DEV_TO_MEM;
	sconf.src_addr = priv->phys_addr + 2*FPGA_FIFO_RBC_ADDR; 
	
	sconf.dst_addr =priv->rbcout_dma_addr;
	sconf.src_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	sconf.dst_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	sconf.dst_maxburst = DMA_PER_TRANSFER_BUF;
	sconf.src_maxburst = DMA_PER_TRANSFER_BUF;
	//sconf.device_fc = 1;
	dmaengine_slave_config(priv->fpga_rbc_dma_chan, &sconf);


	while(len != 0){
		xfer_len = min(len, DMA_PER_TRANSFER_BUF);
		//printk("@@@dma xfer_len=%d \n\n", xfer_len);

		if(xfer_len < DMA_PER_TRANSFER_BUF){
			sconf.dst_maxburst = xfer_len;
			sconf.src_maxburst = xfer_len;
			//sconf.device_fc = 1;
			dmaengine_slave_config(priv->fpga_rbc_dma_chan, &sconf);
		}


		
		//dma_sync_single_for_device(priv->dev, priv->rbcout_dma_addr, 120, DMA_FROM_DEVICE);

		tx = dmaengine_prep_slave_single(chan, tmp_dma_dst, xfer_len*sizeof(u16), DMA_DEV_TO_MEM, flags);
		if (!tx) {
			dev_err(priv->dev, "dmaengine_prep_slave_single error\n");
			ret = -EIO;
			goto err;
		}



		tx->callback = fpga_rbc_dma_complete;
		tx->callback_param = priv;



		dmaengine_submit(tx);


		dma_async_issue_pending(chan);

		len -= xfer_len;
		tmp_dma_dst += xfer_len*2;

#if 0
		ret = wait_for_completion_timeout(&sysinfo_wait,msecs_to_jiffies(6000));
		if (ret <= 0) {
			dmaengine_terminate_all(chan);
			dev_err(priv->dev, "DMA wait_for_completion_timeout\n");
			if (!ret)
				ret = -ETIMEDOUT;
			goto err;
		}
#endif
		wait_for_completion(&rbcinfo_wait);
		//getnstimeofday(&ts3);	

		
		//printk(" dma time diff =%ld us  \n",  ((ts3.tv_sec*1000*1000 + ts3.tv_nsec/1000 ) -  (ts1.tv_sec*1000*1000 + ts1.tv_nsec/1000 )));

			
	}
	

	ret = 0;

err:
	return ret;
}


static int fpga_wbc_dma_transfer(struct fpga_dev *priv, dma_addr_t dma_dst,
			     dma_addr_t dma_src, size_t len)
{
	struct dma_chan *chan = priv->fpga_wbc_dma_chan;
	struct dma_device *dma_dev = chan->device;
	dma_addr_t tmp_dma_dst = dma_dst ;
		
	dma_cookie_t cookie;
	enum dma_ctrl_flags flags = DMA_CTRL_ACK | DMA_PREP_INTERRUPT;
	struct dma_async_tx_descriptor *tx = NULL;
	int ret,i;
	int xfer_len=0;
	
	struct dma_slave_config sconf;


	sconf.direction = DMA_DEV_TO_MEM;
	sconf.src_addr = priv->phys_addr + 2*FPGA_FIFO_WBC_ADDR; 
	
	sconf.dst_addr =priv->rbcout_dma_addr;
	sconf.src_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	sconf.dst_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	sconf.dst_maxburst = DMA_PER_TRANSFER_BUF;
	sconf.src_maxburst = DMA_PER_TRANSFER_BUF;
	//sconf.device_fc = 1;
	dmaengine_slave_config(priv->fpga_wbc_dma_chan, &sconf);



	//printk("rbcout_dma_addr:0x%x \n", rbcout_dma_addr);

	
	while(len != 0){
		xfer_len = min(len, DMA_PER_TRANSFER_BUF);
		//printk("@@@dma xfer_len=%d \n\n", xfer_len);

		if(xfer_len < DMA_PER_TRANSFER_BUF){
			sconf.dst_maxburst = xfer_len;
			sconf.src_maxburst = xfer_len;
			//sconf.device_fc = 1;
			dmaengine_slave_config(priv->fpga_wbc_dma_chan, &sconf);
		}


		
		//dma_sync_single_for_device(priv->dev, priv->rbcout_dma_addr, 120, DMA_FROM_DEVICE);

		tx = dmaengine_prep_slave_single(chan, tmp_dma_dst, xfer_len*sizeof(u16), DMA_DEV_TO_MEM, flags);
		if (!tx) {
			dev_err(priv->dev, "dmaengine_prep_slave_single error\n");
			ret = -EIO;
			goto err;
		}



		tx->callback = fpga_wbc_dma_complete;
		tx->callback_param = priv;



		dmaengine_submit(tx);


		dma_async_issue_pending(chan);

		len -= xfer_len;
		tmp_dma_dst += xfer_len*2;

#if 0
		ret = wait_for_completion_timeout(&sysinfo_wait,msecs_to_jiffies(6000));
		if (ret <= 0) {
			dmaengine_terminate_all(chan);
			dev_err(priv->dev, "DMA wait_for_completion_timeout\n");
			if (!ret)
				ret = -ETIMEDOUT;
			goto err;
		}
#endif
		wait_for_completion(&wbcinfo_wait);
		//getnstimeofday(&ts3);	

		
		//printk(" dma time diff =%ld us  \n",  ((ts3.tv_sec*1000*1000 + ts3.tv_nsec/1000 ) -  (ts1.tv_sec*1000*1000 + ts1.tv_nsec/1000 )));

			
	}
	

	ret = 0;

err:
	return ret;
}

static int fpga_diff_dma_transfer(struct fpga_dev *priv, dma_addr_t dma_dst,
			     dma_addr_t dma_src, size_t len)
{
	struct dma_chan *chan = priv->fpga_diff_dma_chan;
	struct dma_device *dma_dev = chan->device;
	dma_addr_t tmp_dma_dst = dma_dst ;
		
	dma_cookie_t cookie;
	enum dma_ctrl_flags flags = DMA_CTRL_ACK | DMA_PREP_INTERRUPT;
	struct dma_async_tx_descriptor *tx = NULL;
	int ret,i;
	int xfer_len=0;
	
	struct dma_slave_config sconf;


	sconf.direction = DMA_DEV_TO_MEM;
	sconf.src_addr = priv->phys_addr + 2*FPGA_FIFO_DIFF_ADDR; 
	
	sconf.dst_addr =priv->rbcout_dma_addr;
	sconf.src_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	sconf.dst_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	sconf.dst_maxburst = DMA_PER_TRANSFER_BUF;
	sconf.src_maxburst = DMA_PER_TRANSFER_BUF;
	//sconf.device_fc = 1;
	dmaengine_slave_config(priv->fpga_diff_dma_chan, &sconf);



	//printk("rbcout_dma_addr:0x%x \n", rbcout_dma_addr);

	
	while(len != 0){
		xfer_len = min(len, DMA_PER_TRANSFER_BUF);
		//printk("@@@dma xfer_len=%d \n\n", xfer_len);

		if(xfer_len < DMA_PER_TRANSFER_BUF){
			sconf.dst_maxburst = xfer_len;
			sconf.src_maxburst = xfer_len;
			//sconf.device_fc = 1;
			dmaengine_slave_config(priv->fpga_diff_dma_chan, &sconf);
		}


		
		//dma_sync_single_for_device(priv->dev, priv->rbcout_dma_addr, 120, DMA_FROM_DEVICE);

		tx = dmaengine_prep_slave_single(chan, tmp_dma_dst, xfer_len*sizeof(u16), DMA_DEV_TO_MEM, flags);
		if (!tx) {
			dev_err(priv->dev, "dmaengine_prep_slave_single error\n");
			ret = -EIO;
			goto err;
		}



		tx->callback = fpga_diff_dma_complete;
		tx->callback_param = priv;



		dmaengine_submit(tx);


		dma_async_issue_pending(chan);

		len -= xfer_len;
		tmp_dma_dst += xfer_len*2;

#if 0
		ret = wait_for_completion_timeout(&sysinfo_wait,msecs_to_jiffies(6000));
		if (ret <= 0) {
			dmaengine_terminate_all(chan);
			dev_err(priv->dev, "DMA wait_for_completion_timeout\n");
			if (!ret)
				ret = -ETIMEDOUT;
			goto err;
		}
#endif
		wait_for_completion(&diffinfo_wait);
		//getnstimeofday(&ts3);	

		
		//printk(" dma time diff =%ld us  \n",  ((ts3.tv_sec*1000*1000 + ts3.tv_nsec/1000 ) -  (ts1.tv_sec*1000*1000 + ts1.tv_nsec/1000 )));

			
	}
	

	ret = 0;

err:
	return ret;
}


static int fpga_dma_rbc_mcpy_transfer(struct fpga_dev *priv, dma_addr_t dma_dst, dma_addr_t dma_src, size_t len)
{

	struct dma_chan *chan = priv->fpga_rbc_dma_chan;
	struct dma_device *dma_dev = chan->device;
	enum dma_ctrl_flags flags = DMA_CTRL_ACK | DMA_PREP_INTERRUPT;
	enum dma_status status;
	struct dma_async_tx_descriptor *tx = NULL;
	int ret=0,i;

#if 0
	struct dma_slave_config sconf;


	sconf.direction = DMA_DEV_TO_MEM;
	sconf.src_addr = priv->phys_addr + 2*FPGA_FIFO_DIFF_ADDR; 
	
	sconf.dst_addr =priv->rbcout_dma_addr;
	sconf.src_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	sconf.dst_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	sconf.dst_maxburst = DMA_PER_TRANSFER_BUF;
	sconf.src_maxburst = DMA_PER_TRANSFER_BUF;
	//sconf.device_fc = 1;
	dmaengine_slave_config(priv->fpga_diff_dma_chan, &sconf);

#endif
	getnstimeofday(&tm2);	

	tx = dma_dev->device_prep_dma_memcpy(chan, (unsigned long)dma_dst, (unsigned long)dma_src,
					     len, flags);
	if (!tx) {
		dev_err(priv->dev, "device_prep_dma_memcpy error\n");
		ret = -EIO;
		goto err;
	}

	
 //change other way to  refer to omap_dmm_tiler.c
	tx->callback = fpga_rbc_dma_memcpy_complete;
	tx->callback_param = priv;

	dmaengine_submit(tx);


	dma_async_issue_pending(chan);


	//ret = wait_for_completion_timeout(&sysinfo_wait, msecs_to_jiffies(50));
	wait_for_completion(&rbcinfo_wait);
	printk("dma success\n");

	getnstimeofday(&tm3);	
	difftm1 =  ((tm3.tv_sec*1000*1000 + tm3.tv_nsec/1000 ) -	(tm2.tv_sec*1000*1000 + tm2.tv_nsec/1000 ));

	printk("dma cpy time:%d\n", difftm1);
	for(i = 0; i< 100;i++)
		printk("%u " ,((u32 *)((unsigned int)g_fpga_dev->rbcout_addr))[i] );
	printk("\n");

	ret = 0;
err:
	return ret;


}



static irqreturn_t gpmc_fpga_rbc_irq(int irq, void *dev_id)
{
	struct fpga_dev *priv = dev_id;
	u16  int5_m,int0_sys, int1_rbc, int2_wbc,int3_opt,int4_sw;//motor source reg val
	int reg_val_len,reg_val;
	int sys_gcnt_flag, rbc_end_flag;
	u16 rbc_status;

	//printk("gpmc_fpga_rbc_irq\n");
	rbc_irq++;

	priv->test_flag = 0;
	/* handle irq event */
	/* this is a no mask interrupt source ,can't use disable_irq() to disable cpu interrupt controller */
	//fpga_disable_irq(priv); //diable irq ,so we can do not use spin_lock to enable preempt;

	//spin_lock(&priv->lock);
	
	fpga_write_reg(g_fpga_dev, FPGA_INTER_RBC_MASK, 0xff);
	
	getnstimeofday(&tm1);	
	intcnt += 2;


	int1_rbc = fpga_read_reg(priv,FPGA_INTER_RBC_SOURCE);

	if(int1_rbc != NULL) {
		//fpga_info(("interrupt source rbc"));
		if( likely(int1_rbc & BIT_MASK(INTERRUPT_RBC_SOURCE_NORMAL_INT_BIT_6))) {//fifo rbc normal int ;
			queue_work(priv->wqfifo,&priv->fifo_rbc); //read wq to handle dma data;
		}else if(int1_rbc & BIT_MASK(INTERRUPT_RBC_SOURCE_END_INT_BIT_7)) {
			//fpga_info(("interrupt source rbc end flag"));	
			rbc_end_rpt_flag = 1;
			//printk("rbc_end\n");
			queue_work(priv->wqfifo,&priv->fifo_rbc); //read wq to handle dma data;
				priv->test_flag = 1;
		}else{
			fpga_info(("interrupt source NULL flag"));	
			fpga_write_reg(g_fpga_dev, FPGA_INTER_RBC_MASK, 0);
			fpga_write_reg(g_fpga_dev, FPGA_INTER_RBC_SOURCE, 0);

		}
			
	}else{
		fpga_info(("int1_rbc == NULL\n"));		
		intcnt -= 2;
		fpga_write_reg(g_fpga_dev, FPGA_INTER_RBC_MASK, 0);
		fpga_write_reg(g_fpga_dev, FPGA_INTER_RBC_SOURCE, 0);

	}

	priv->int_mask_flag =0;

irq_out:
	return IRQ_HANDLED;
}


static irqreturn_t gpmc_fpga_wbc_irq(int irq, void *dev_id)
{
	struct fpga_dev *priv = dev_id;
	u16  int5_m,int0_sys, int1_rbc, int2_wbc,int3_opt,int4_sw;//motor source reg val
	int reg_val_len,reg_val;
	int sys_gcnt_flag, rbc_end_flag;
	u16 rbc_status;

	//printk("gpmc_fpga_wbc_irq\n");
	wbc_irq++;

	priv->test_flag = 0;
	/* handle irq event */
	/* this is a no mask interrupt source ,can't use disable_irq() to disable cpu interrupt controller */
	//fpga_disable_irq(priv); //diable irq ,so we can do not use spin_lock to enable preempt;

	//spin_lock(&priv->lock);
	
	fpga_write_reg(g_fpga_dev, FPGA_INTER_WBC_MASK, 0xff);
	

	int1_rbc = fpga_read_reg(priv,FPGA_INTER_WBC_SOURCE);

	if(int1_rbc != NULL) {
		//fpga_info(("interrupt source rbc"));
		if( likely(int1_rbc & BIT_MASK(INTERRUPT_WBC_SOURCE_NORMAL_INT_BIT_6))) {//fifo rbc normal int ;
			queue_work(priv->wqfifo,&priv->fifo_wbc); //read wq to handle dma data;
		}else if(int1_rbc & BIT_MASK(INTERRUPT_WBC_SOURCE_END_INT_BIT_7)) {
			//fpga_info(("interrupt source rbc end flag"));	
			wbc_end_rpt_flag = 1;
			queue_work(priv->wqfifo,&priv->fifo_wbc); //read wq to handle dma data;
				priv->test_flag = 1;
		}else{
			fpga_info(("interrupt source NULL flag"));	
			fpga_write_reg(g_fpga_dev, FPGA_INTER_WBC_MASK, 0);
			fpga_write_reg(g_fpga_dev, FPGA_INTER_WBC_SOURCE, 0);

		}
			
	}else{
		fpga_info(("int1_wbc == NULL\n"));		
		intcnt -= 2;
		fpga_write_reg(g_fpga_dev, FPGA_INTER_WBC_MASK, 0);
		fpga_write_reg(g_fpga_dev, FPGA_INTER_WBC_SOURCE, 0);

	}
	

	priv->int_mask_flag =0;

irq_out:
	return IRQ_HANDLED;
}


static irqreturn_t gpmc_fpga_diff_irq(int irq, void *dev_id)
{
	struct fpga_dev *priv = dev_id;
	u16  int5_m,int0_sys, int_diff, int2_wbc,int3_opt,int4_sw;//motor source reg val
	int reg_val_len,reg_val;
	int sys_gcnt_flag, rbc_end_flag;
	u16 rbc_status;

	printk("gpmc_fpga_diff_irq\n");
	diff_irq++;

	priv->test_flag = 0;
	/* handle irq event */
	/* this is a no mask interrupt source ,can't use disable_irq() to disable cpu interrupt controller */
	//fpga_disable_irq(priv); //diable irq ,so we can do not use spin_lock to enable preempt;

	//spin_lock(&priv->lock);
	
	fpga_write_reg(g_fpga_dev, FPGA_INTER_DIFF_MASK, 0xff);
	

	int_diff = fpga_read_reg(priv, FPGA_INTER_DIFF_SOURCE);

	if(int_diff != NULL) {
		//fpga_info(("interrupt source rbc"));
		if( likely(int_diff & BIT_MASK(INTERRUPT_DIFF_SOURCE_NORMAL_INT_BIT_6))) {//fifo rbc normal int ;
			queue_work(priv->wqfifo,&priv->fifo_diff); //read wq to handle dma data;
		}else if(int_diff & BIT_MASK(INTERRUPT_DIFF_SOURCE_END_INT_BIT_7)) {
			//fpga_info(("interrupt source rbc end flag"));	

			queue_work(priv->wqfifo,&priv->fifo_diff); //read wq to handle dma data;
				priv->test_flag = 1;
		}else{
			fpga_info(("interrupt source NULL flag"));	
			fpga_write_reg(g_fpga_dev, FPGA_INTER_DIFF_MASK, 0);
			fpga_write_reg(g_fpga_dev,	FPGA_INTER_DIFF_SOURCE, 0);

		}
			
	}else{
		fpga_info(("int_diff == NULL\n"));		
		intcnt -= 2;
		fpga_write_reg(g_fpga_dev,  FPGA_INTER_DIFF_MASK, 0);
		fpga_write_reg(g_fpga_dev,  FPGA_INTER_DIFF_SOURCE, 0);

	}
	

	priv->int_mask_flag =0;

irq_out:
	return IRQ_HANDLED;
}


#if 1
static void fpga_fifo_rbc_work(struct work_struct  *work)
{
	struct fpga_dev *priv = container_of(work, struct fpga_dev, fifo_rbc);
	struct dma_async_tx_descriptor *dma_desc;
	dma_addr_t dma_src = priv->phys_addr + 2*FPGA_FIFO_RBC_ADDR;
	size_t xfer_len;
	int ret,i;
	u16 rbc_status;
	unsigned short *buf;
	static u32 tmp_dma_dst=0;
	int end_rpt_flag=0, full_flag=0;
	struct measure_related_info mri;


	getnstimeofday(&tm2);	

	if(priv->sche_wa_flga== 1){
			printk("probe workaround for schedle time long\n");
			priv->sche_wa_flga = 0;
			return ;
	}

	printk("rbc_work\n");

	
	//unsigned int to = (unsigned int)buf;

	if(rbc_fisrt_flag == 1 ){
		tmp_dma_dst = priv->rbcout_dma_addr;
		rbc_fisrt_flag= 0;
	}
		
	rbc_status =  fpga_read_reg(priv,FPGA_FIFO_RBC_STATUS); //read sysinfo data len;
	if(rbc_status != NULL){
		if(unlikely(rbc_status & BIT_MASK(FPGA_FIFO_RBC_STATUS_FIFO_FULL_BIT))) {
			//printk("FPGA_FIFO_RBC_STATUS_FIFO_FULL_BIT\n");
			xfer_len = 8000;
			//printk("xfer_len:%d\n", xfer_len);
			//return;
		}else if(unlikely(rbc_status & BIT_MASK(FPGA_FIFO_RBC_STATUS_FIFO_EMP_BIT))) {
			//printk("FPGA_FIFO_RBC_STATUS_FIFO_EMP_BIT priv->test_flag =%d\n", priv->test_flag);
			fpga_write_reg(g_fpga_dev, 0x40004, 0);	
			fpga_write_reg(g_fpga_dev, 0x40002, 0);
			end_rpt_flag = 1;
			xfer_len = 0;
			//return;
		}else{
			xfer_len = 0x1fff & rbc_status;	
			//lenarr[cnt++] = xfer_len;
			//printk("fifo_len:%d\n", xfer_len);
		}
	}else{
		printk("rbc_status == NULL\n");
		fpga_write_reg(g_fpga_dev, 0x40004, 0);	
		fpga_write_reg(g_fpga_dev, 0x40002, 0);
		return ;
	}

	//printk("tmp_dma_dst:0x%x\n", tmp_dma_dst);
	
	//buf  = (unsigned short *)kmalloc(xfer_len*sizeof(unsigned short),GFP_KERNEL);

	//ret = fpga_dma_transfer1(priv, priv->rbcout_dma_addr, dma_src, xfer_len);
	
	//ret = fpga_dma_transfer2(priv, priv->rbcout_dma_addr, xfer_len);

	
	//getnstimeofday(&ts1);	
	//xfer_len=2000;
	

	ret = fpga_rbc_dma_transfer(priv, tmp_dma_dst, dma_src, xfer_len);
	//getnstimeofday(&tm3);	
	//usleep_range(10000,11000); 


	rbc_per_len += xfer_len;
	tmp_dma_dst += 2*xfer_len;


	//difftm1 =  ((tm2.tv_sec*1000*1000 + tm2.tv_nsec/1000 ) -	(tm1.tv_sec*1000*1000 + tm1.tv_nsec/1000 ));
	//difftm2 =  ((tm3.tv_sec*1000*1000 + tm3.tv_nsec/1000 ) -	(tm1.tv_sec*1000*1000 + tm1.tv_nsec/1000 ));
	//timearr[intcnt-2] = difftm1;
	//timearr[intcnt-1] = difftm2;
	//printk("fifolen:\n");

	//for(i=0;i<3;i++)
	//	printk("%d ", lenarr[i]);
	//printk("\n");
	//printk(" dma time2 diff =%ld us  time3 diff =%ld us  time4 diff =%ld work_sch:%ld us\n",  ((ts2.tv_sec*1000*1000 + ts2.tv_nsec/1000 ) -	(ts1.tv_sec*1000*1000 + ts1.tv_nsec/1000 )), ((ts3.tv_sec*1000*1000 + ts3.tv_nsec/1000 ) -	(ts1.tv_sec*1000*1000 + ts1.tv_nsec/1000 )), ((ts4.tv_sec*1000*1000 + ts4.tv_nsec/1000 ) -	(ts1.tv_sec*1000*1000 + ts1.tv_nsec/1000 )), ((ts5.tv_sec*1000*1000 + ts5.tv_nsec/1000 ) -	(ts1.tv_sec*1000*1000 + ts1.tv_nsec/1000 )));
#if 0	

	getnstimeofday(&ts5);	
	usleep_range(1000,1000); 
	getnstimeofday(&ts6);	
	printk(" 1ms diff =%ld us \n",((ts6.tv_sec*1000*1000 + ts6.tv_nsec/1000 ) -	(ts5.tv_sec*1000*1000 + ts5.tv_nsec/1000 )));
#endif




#if  1
	if(g_fpga_dev->pmri == NULL)
		g_fpga_dev->pmri = &mri;
	


	
	//if(per_len >  50 /*&& !atomic_read(&priv->fifo_rbc_t)*/) { //2k fifo data per/time to send to userspace;
	if(rbc_per_len >  FIFO_DATA_SEND_MAX_SIZE  ||(rbc_end_rpt_flag == 1 && rbc_per_len!=0)/*&& !atomic_read(&priv->fifo_rbc_t)*/) { //2k fifo data per/time to send to userspace;
		//printk("rbc per_len:%d \n", rbc_per_len);
		getnstimeofday(&tm4);	
		encode_tlv_netlink_send_data_rpt(MSG_DATA_REPORT, SUBTYPE_DATA_REPORT_RBC,g_fpga_dev->pmri, (char *)priv->rbcout_addr, rbc_per_len*sizeof(u16));		
		getnstimeofday(&tm5);	
		
		diffcpy =  ((tm5.tv_sec*1000*1000 + tm5.tv_nsec/1000 ) -	(tm4.tv_sec*1000*1000 + tm4.tv_nsec/1000 ));
		rbc_cpy_time[cpycnt] = rbc_per_len;
		rbc_cpy_time[cpycnt+1] = diffcpy;
		cpycnt +=2;

		rbc_fisrt_flag = 1;
		rbc_per_len = 0;
		end_rpt_flag = 0;
		
	}
#endif

	fpga_write_reg(g_fpga_dev, 0x40004, 0); 
	fpga_write_reg(g_fpga_dev, 0x40002, 0);

	getnstimeofday(&tm3);	

	difftm1 =  ((tm2.tv_sec*1000*1000 + tm2.tv_nsec/1000 ) -	(tm1.tv_sec*1000*1000 + tm1.tv_nsec/1000 ));
	difftm2 =  ((tm3.tv_sec*1000*1000 + tm3.tv_nsec/1000 ) -	(tm2.tv_sec*1000*1000 + tm2.tv_nsec/1000 ));
	rbctime[intcnt-2] = difftm1;
	rbctime[intcnt-1] = difftm2;
	rbc_xfer_len[rbc_irq-1] = xfer_len;

	rbc_total +=  2*xfer_len;



	//printk("rbc_work end\n");

}
#else

static void fpga_fifo_rbc_work(struct work_struct  *work)
{
	struct fpga_dev *priv = container_of(work, struct fpga_dev, fifo_rbc);
	struct dma_async_tx_descriptor *dma_desc;
	dma_addr_t dma_src = priv->phys_addr + 2*FPGA_FIFO_RBC_ADDR;
	size_t xfer_len;
	int ret,i;
	u16 rbc_status;
	unsigned short *buf;
	static u32 tmp_dma_dst=0;
	int end_rpt_flag=0, full_flag=0;
	struct measure_related_info mri;


	//getnstimeofday(&tm2);	
	//xfer_len = 5000000;
	//xfer_len =5000000;
	xfer_len = 5000;

	ret = fpga_dma_rbc_mcpy_transfer(priv, priv->rbcout_dma_addr,  dma_src, xfer_len*sizeof(u32));
	

}


#endif

static void fpga_fifo_wbc_work(struct work_struct  *work)
{
	struct fpga_dev *priv = container_of(work, struct fpga_dev, fifo_wbc);
	struct dma_async_tx_descriptor *dma_desc;
	dma_addr_t dma_src = priv->phys_addr + 2*FPGA_FIFO_WBC_ADDR;
	size_t xfer_len;
	int ret,i;
	static int per_len=0;
	u16 wbc_status;
	unsigned short *buf;
	static u32 tmp_dma_dst=0;
	static int fisrt_flag = 1, end_rpt_flag=0, full_flag=0;
	struct measure_related_info mri;


	if(priv->sche_wa_flga== 1){
			printk("probe workaround for schedle time long\n");
			priv->sche_wa_flga = 0;
			return ;
	}

	
	//printk("wbc work \n");

	if(wbc_fisrt_flag == 1){
		tmp_dma_dst = priv->wbcout_dma_addr;
		wbc_fisrt_flag= 0;
	}
		
	
	wbc_status =  fpga_read_reg(priv,FPGA_FIFO_WBC_STATUS); //read sysinfo data len;
	if(wbc_status != NULL){
		if(unlikely(wbc_status & BIT_MASK(FPGA_FIFO_WBC_STATUS_FIFO_FULL_BIT))) {
			//printk("FPGA_FIFO_RBC_STATUS_FIFO_FULL_BIT\n");
			xfer_len = 8000;
			//printk("xfer_len:%d\n", xfer_len);
			//return;
		}else if(unlikely(wbc_status & BIT_MASK(FPGA_FIFO_WBC_STATUS_FIFO_EMP_BIT))) {
			//printk("FPGA_FIFO_RBC_STATUS_FIFO_EMP_BIT priv->test_flag =%d\n", priv->test_flag);
			fpga_write_reg(g_fpga_dev, FPGA_INTER_WBC_SOURCE, 0);	
			fpga_write_reg(g_fpga_dev, FPGA_INTER_WBC_MASK, 0);
			end_rpt_flag = 1;
			xfer_len = 0;
			//return;
		}else{
			xfer_len = 0x1fff & wbc_status;	
		}
	}else{
		printk("wbc_status == NULL\n");
		fpga_write_reg(g_fpga_dev, FPGA_INTER_WBC_SOURCE, 0);	
		fpga_write_reg(g_fpga_dev, FPGA_INTER_WBC_MASK, 0);
		return ;
	}



	ret = fpga_wbc_dma_transfer(priv, tmp_dma_dst, dma_src, xfer_len);

	wbc_per_len += xfer_len;
	tmp_dma_dst += 2*xfer_len;

#if  1
	if(g_fpga_dev->pmri == NULL)
		g_fpga_dev->pmri = &mri;

	fpga_write_reg(g_fpga_dev, FPGA_INTER_WBC_SOURCE, 0); 
	fpga_write_reg(g_fpga_dev, FPGA_INTER_WBC_MASK, 0);
	

	
	//if(per_len >  50 /*&& !atomic_read(&priv->fifo_rbc_t)*/) { //2k fifo data per/time to send to userspace;
	if(wbc_per_len >  FIFO_DATA_SEND_MAX_SIZE  ||(wbc_end_rpt_flag == 1 && wbc_per_len!=0)/*&& !atomic_read(&priv->fifo_rbc_t)*/) { //2k fifo data per/time to send to userspace;
		//printk("wbc per_len:%d \n", per_len);
		encode_tlv_netlink_send_data_rpt(MSG_DATA_REPORT, SUBTYPE_DATA_REPORT_WBC,g_fpga_dev->pmri, (char *)priv->wbcout_addr, wbc_per_len*sizeof(u16));
		wbc_fisrt_flag = 1;
		wbc_per_len = 0;
		end_rpt_flag = 0;
	}
#endif

	wbc_total +=  2*xfer_len;
	//printk("wbc_work end\n");

}


static void fpga_fifo_diff_work(struct work_struct  *work)
{
	struct fpga_dev *priv = container_of(work, struct fpga_dev, fifo_diff);
	struct dma_async_tx_descriptor *dma_desc;
	dma_addr_t dma_src = priv->phys_addr + 2*FPGA_FIFO_DIFF_ADDR;
	size_t xfer_len;
	int ret,i;
	static int per_len=0;
	u16 diff_status;
	unsigned short *buf;
	static u32 tmp_dma_dst=0;
	static int fisrt_flag = 1, end_rpt_flag=0, full_flag=0;
	struct measure_related_info mri;

	
	if(priv->sche_wa_flga== 1){
			printk("probe workaround for schedle time long\n");
			priv->sche_wa_flga = 0;
			return ;
	}
	
	printk("diff_work\n");
		
		//unsigned int to = (unsigned int)buf;
	
	if(diff_fisrt_flag == 1){
		tmp_dma_dst = priv->diffout_dma_addr;
		diff_fisrt_flag= 0;
	}
			
	//usleep_range(100,110); 
		
	diff_status =  fpga_read_reg(priv,FPGA_FIFO_DIFF_STATUS); //read sysinfo data len;
	if(diff_status != NULL){
		if(unlikely(diff_status & BIT_MASK(FPGA_FIFO_DIFF_STATUS_FIFO_FULL_BIT))) {
				//printk("FPGA_FIFO_RBC_STATUS_FIFO_FULL_BIT\n");
				xfer_len = 8000;
				//printk("xfer_len:%d\n", xfer_len);
				//return;
		}else if(unlikely(diff_status & BIT_MASK(FPGA_FIFO_DIFF_STATUS_FIFO_EMP_BIT))) {
				//printk("FPGA_FIFO_RBC_STATUS_FIFO_EMP_BIT priv->test_flag =%d\n", priv->test_flag);
				fpga_write_reg(g_fpga_dev, FPGA_INTER_DIFF_SOURCE, 0); 
				fpga_write_reg(g_fpga_dev, FPGA_INTER_DIFF_MASK, 0);
				end_rpt_flag = 1;
				xfer_len = 0;
		}else{
				xfer_len = 0x1fff & diff_status; 
		}
	}else{
			printk("rbc_status == NULL\n");
			fpga_write_reg(g_fpga_dev, FPGA_INTER_DIFF_SOURCE, 0); 
			fpga_write_reg(g_fpga_dev, FPGA_INTER_DIFF_MASK, 0);
			return ;
	}
	

	ret = fpga_diff_dma_transfer(priv, tmp_dma_dst, dma_src, xfer_len);
	getnstimeofday(&tm3);	
	
	diff_per_len += xfer_len;
	tmp_dma_dst += 2*xfer_len;


	if(g_fpga_dev->pmri == NULL)
		g_fpga_dev->pmri = &mri;
	
	
	fpga_write_reg(g_fpga_dev, FPGA_INTER_DIFF_SOURCE, 0); 
	fpga_write_reg(g_fpga_dev, FPGA_INTER_DIFF_MASK, 0);
		
		//if(per_len >	50 /*&& !atomic_read(&priv->fifo_rbc_t)*/) { //2k fifo data per/time to send to userspace;
	if(diff_per_len >  FIFO_DATA_SEND_MAX_SIZE  ||(diff_end_rpt_flag == 1 && diff_per_len!=0)/*&& !atomic_read(&priv->fifo_rbc_t)*/) { //2k fifo data per/time to send to userspace;
			//printk("diff per_len:%d \n", per_len);
		encode_tlv_netlink_send_data_rpt(MSG_DATA_REPORT, SUBTYPE_DATA_REPORT_DIFF,g_fpga_dev->pmri, (char *)priv->diffout_addr, diff_per_len*sizeof(u16));
			//usleep_range(600,700); 
		diff_fisrt_flag = 1;
		diff_per_len = 0;
		end_rpt_flag = 0;
	}


		diff_total +=  2*xfer_len;

}




static void fpga_sys_float_work(struct work_struct  *work)
{
	struct fpga_dev *priv = container_of(work, struct fpga_dev, wqfifo);

	mutex_lock(&priv->mutex);

	fpga_info(("fpga_sys_float_work"));

	mutex_unlock(&priv->mutex);

}

static int gpmc_fpga_open(struct inode *inode, struct file *filp)
{
	/*
	 * The miscdevice layer puts our struct miscdevice into the
	 * filp->private_data field. We use this to find our private
	 * data and then overwrite it with our own private structure.
	 */
	struct fpga_dev *priv = container_of(filp->private_data,
						struct fpga_dev, miscdev);

	return 0;
}

static int gpmc_fpga_release(struct inode *inode, struct file *filp)
{

	return 0;
}

static ssize_t gpmc_fpga_read(struct file *filp, char __user *ubuf, size_t count, loff_t *f_pos)
{
	int ret;
	unsigned long flags;
	char fpga_data[100];
	
	struct fpga_dev *priv = container_of(filp->private_data,
						struct fpga_dev, miscdev);
#if 0
	/* begain to set time seq for fpga */
	queue_work(priv->fpga_workq,&priv->fpga_setseq_wq);
	
	/* wait for dma handle ready read data  from dma buffer*/
	//ret = wait_event_interruptible_timeout(priv->fpga_rq,atomic_read(&priv->fpga_scheduled) == 1,HZ);
	ret = wait_event_interruptible(priv->fpga_rq,atomic_read(&priv->fpga_scheduled) == 1);
	if(ret)
		return -EINTR;

	//memset(fpga_data,0,sizeof(fpga_data));
	
	/* ok to copy fpga data from fifo to userspace  */
	if(copy_to_user(ubuf, sys_hgb_addr, sysinfo_len )) {
		printk("copy to user failed\n");
		return 0;
	}
#endif	
	return sysinfo_len;
}

static unsigned int gpmc_fpga_poll(struct file *filp, struct poll_table_struct *tbl)
{
	int ret = 0;


	return ret;
}

static int gpmc_fpga_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int ret = 0;



	return ret;
}

static const struct file_operations gpmc_fpga_fops = {
	.owner		= THIS_MODULE,
	.open		= gpmc_fpga_open,
	.release	= gpmc_fpga_release,
	.read		= gpmc_fpga_read,
	.poll		= gpmc_fpga_poll,
	.mmap		= gpmc_fpga_mmap,
	.llseek		= no_llseek,
};




static void timing_seq_action_init(void)
{
	timing_parser_init();
	timing_data_read(tdata_path);
	timing_seq_action_parser();
	
	fpga_info(( "Timing seq action parser init success\n"));
}


bool fpga_dma_filter_fn(struct dma_chan *chan, void *param)
{
	int chan_id = *(unsigned int *)param;
	printk("fpga_dma_filter_fn\n");
	if (chan_id == chan->chan_id)
		return true;
	
	return false;
}

static int fpga_dma_init(struct fpga_dev * priv, int rx_req)
{
	dma_cap_mask_t mask;
	int ret;
	int i;
	u16 *pbuf_addr[21];
	dma_addr_t pdma_addr[21];
	struct dma_slave_config conf;
	unsigned id = 4;
	unsigned rbc_req = 1, wbc_req=2, diff_req=3;


	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	//dma_cap_set(DMA_MEMCPY, mask);


	priv->fpga_rbc_dma_chan = dma_request_slave_channel_compat_reason(mask,
				NULL, &rbc_req, priv->dev, "rbc");


	if (IS_ERR(priv->fpga_rbc_dma_chan)) {
		dev_err(priv->dev,
					"No Rx DMA available, trying mmap mode\n");
		return  -ENODEV;
	}
	printk("@@@rbc->rx_chan:%d\n", priv->fpga_rbc_dma_chan->chan_id);



	priv->rbcout_addr = dma_alloc_coherent(priv->fpga_rbc_dma_chan->device->dev, FIFO_DATA_DMA_SIZE*sizeof(u16),
					&priv->rbcout_dma_addr,  GFP_KERNEL|GFP_DMA);

	if (!priv->rbcout_addr) {
		dev_err(priv->dev,
			"dma_alloc_coherent  rbcout_addr falied, using PIO mode\n");
		dma_release_channel(priv->fpga_rbc_dma_chan);
	}

	printk("priv->rbcout_addr:0x%x   rbcout_dma_addr:0x%x\n", priv->rbcout_addr, priv->rbcout_dma_addr);


	priv->fpga_wbc_dma_chan = dma_request_slave_channel_compat_reason(mask,
				NULL, &wbc_req, priv->dev, "wbc");

	if (IS_ERR(priv->fpga_wbc_dma_chan)) {
		dev_err(priv->dev,
					"No Rx DMA available, trying mmap mode\n");
		return  -ENODEV;
	}
	printk("@@@wbc->rx_chan:%d\n", priv->fpga_wbc_dma_chan->chan_id);


	priv->wbcout_addr = dma_alloc_coherent(priv->fpga_wbc_dma_chan->device->dev, FIFO_DATA_DMA_SIZE*sizeof(u16),
					&priv->wbcout_dma_addr,  GFP_KERNEL|GFP_DMA);

	if (!priv->wbcout_addr) {
		dev_err(priv->dev,
			"dma_alloc_coherent wbcout_addr falied, using PIO mode\n");
		dma_release_channel(priv->fpga_wbc_dma_chan);
	}

	printk("priv->wbcout_addr:0x%x   wbcout_dma_addr:0x%x\n", priv->wbcout_addr, priv->wbcout_dma_addr);

	priv->fpga_diff_dma_chan = dma_request_slave_channel_compat_reason(mask,
				NULL, &rbc_req, priv->dev, "diff");

	if (IS_ERR(priv->fpga_diff_dma_chan)) {
		dev_err(priv->dev,
					"No Rx DMA available, trying mmap mode\n");
		return  -ENODEV;
	}
	printk("@@@diff->rx_chan:%d\n", priv->fpga_diff_dma_chan->chan_id);


	priv->diffout_addr = dma_alloc_coherent(priv->fpga_diff_dma_chan->device->dev, FIFO_DATA_DMA_SIZE*sizeof(u16),
					&priv->diffout_dma_addr,  GFP_KERNEL|GFP_DMA);

	if (!priv->diffout_addr) {
		dev_err(priv->dev,
			"dma_alloc_coherent opt_fsout_addr falied, using PIO mode\n");
		dma_release_channel(priv->fpga_diff_dma_chan);
	}
	printk("priv->diffout_addr:0x%x   diffout_dma_addr:0x%x\n", priv->diffout_addr, priv->diffout_dma_addr);

	printk("fpga dma register success\n");

	return 0;
	
}

static int fpga_dma_memcpy_init(struct fpga_dev * priv, int datatype)
{
	dma_cap_mask_t mask;
	int ret;
	int i;
	u16 *pbuf_addr[21];
	dma_addr_t pdma_addr[21];
	struct dma_slave_config conf;

	dma_cap_zero(mask);

	dma_cap_set(DMA_MEMCPY, mask);


	/* Get control of DMA channel #0 */
	priv->fpga_rbc_dma_chan = dma_request_channel(mask, NULL, NULL);
	if (!priv->fpga_rbc_dma_chan) {
		dev_err(priv->dev, "Unable to acquire DMA channel #0\n");
		return  -ENODEV;
	}
	printk("rbc->rx_chan:%d\n", priv->fpga_rbc_dma_chan->chan_id);
	

	priv->rbcout_addr = dma_alloc_coherent(priv->fpga_rbc_dma_chan->device->dev, FIFO_DATA_DMA_SIZE*sizeof(u16),
					&priv->rbcout_dma_addr,  GFP_KERNEL|GFP_DMA);

	if (!priv->rbcout_addr) {
		dev_err(priv->dev,
			"dma_alloc_coherent  rbcout_addr falied, using PIO mode\n");
		dma_release_channel(priv->fpga_rbc_dma_chan);
	}

	printk("priv->rbcout_addr:0x%x   rbcout_dma_addr:0x%x\n", priv->rbcout_addr, priv->rbcout_dma_addr);


	priv->wbcout_addr = dma_alloc_coherent(priv->fpga_rbc_dma_chan->device->dev, FIFO_DATA_DMA_SIZE*sizeof(u16),
					&priv->wbcout_dma_addr,  GFP_KERNEL|GFP_DMA);

	if (!priv->wbcout_addr) {
		dev_err(priv->dev,
			"dma_alloc_coherent wbcout_addr falied, using PIO mode\n");
		dma_release_channel(priv->fpga_rbc_dma_chan);
	}

	printk("priv->wbcout_addr:0x%x   wbcout_dma_addr:0x%x\n", priv->wbcout_addr, priv->wbcout_dma_addr);


	priv->diffout_addr = dma_alloc_coherent(priv->fpga_rbc_dma_chan->device->dev, FIFO_DATA_DMA_SIZE*sizeof(u16),
					&priv->diffout_dma_addr,  GFP_KERNEL|GFP_DMA);

	if (!priv->diffout_addr) {
		dev_err(priv->dev,
			"dma_alloc_coherent opt_fsout_addr falied, using PIO mode\n");
		dma_release_channel(priv->fpga_rbc_dma_chan);
	}
	printk("priv->diffout_addr:0x%x   diffout_dma_addr:0x%x\n", priv->diffout_addr, priv->diffout_dma_addr);



	printk("fpga dma register success\n");
	return 0;
}




/*
* event ts ctrl & ts report no need to have raw data,so just to parser type element 
*/
void netlink_event_ts_handle(struct measure_related_info *mri, int type)
{
	int err;
	int func_ts_index;
	int count;
	struct sched_param param = { .sched_priority = 1 };
	struct measure_related_info *tmp_mri =(struct measure_related_info *)kzalloc(sizeof(*mri), GFP_KERNEL);

	memcpy(tmp_mri, mri, sizeof(struct measure_related_info));
	
	
	//fpga_info(("netlink_event_ts_handle: timing id:%d  unique key:%d subtype:%d",tmp_mri->m_ts_id, tmp_mri->m_unique_key, type));

	if( g_func_tshdr == NULL || g_func_num == 0 ) {
		fpga_info(("!!!!!!Func TS Head List IS Not Initialize, Please Check !!!!!!"));
		return -1;
	}
	

	func_ts_index = get_func_ts_id_index(tmp_mri->m_ts_id);
	if(func_ts_index < 0 && type != SUBTYPE_TS_CTRL_MEASURE_FORCE_STOP )
		return -1;

	switch(type) {
	case SUBTYPE_TS_CTRL_MEASURE_START: //ts ctrl  ICS->TS_EXE				
			func_ts_thread[func_ts_index] = kthread_create(fpga_exec_functs_thread, tmp_mri, "fpga_timseq_exethread");
			if (IS_ERR(func_ts_thread)) {
				err = PTR_ERR(func_ts_thread);
				return err;
			}
			sched_setscheduler(func_ts_thread[func_ts_index], SCHED_FIFO, &param);
			wake_up_process(func_ts_thread[func_ts_index]);
			
			//if(func_ts_thread[func_ts_index])
			//	printk("func_ts thread : pid:%d(func_ts_index:%d) started\n", func_ts_thread[func_ts_index]->pid, func_ts_index);
	
		break;
	
	case SUBTYPE_TS_CTRL_MEASURE_STOP: //ts ctrl  ICS->TS_EXE	
		func_ts_index = get_func_ts_id_index(tmp_mri->m_ts_id);
		printk("stop func_ts_index:%d\n", func_ts_index);
		if(func_ts_thread[func_ts_index]) {
			kthread_stop(func_ts_thread[func_ts_index]);
			func_ts_thread[func_ts_index]=NULL;	
			printk("kthread_stop success\n");
		}
		break;
	
	case SUBTYPE_TS_CTRL_MEASURE_FORCE_STOP: //ts ctrl  ICS->TS_EXE
		//stop all the func ts thread;
		for(count = 0; count < g_func_num; count++){
			printk("force stop func_ts_index:%d\n", count);
			if(func_ts_thread[count]) {
				kthread_stop(func_ts_thread[count]);
				func_ts_thread[count]=NULL;					
				printk("force stop kthread_stop success\n");
			}
		}

		break;

	}

	return ;
}




void netlink_send_timing_cmd(void)
{	
	struct measure_related_info mri;
	memset(&mri, 0, sizeof(struct measure_related_info));
	mri.m_unique_key = 23;
	mri.m_ts_id = 99;
	fpga_info(("Trigger get func ts signal..."));
	encode_tlv_netlink_send_ts_rpt(MSG_TYPE_TS_REPORT, SUBTYPE_TS_REPORT_TRIGGER, &mri, sizeof(struct measure_related_info));

}

int netlink_parser_data_from_ics(char *buf, int len)
{
	struct tlv_data * tlvdata, *sub_tlvdata;
	int value_len,sub_valuelen; 
	int i;
	
	tlvdata =(struct tlv_data *)kzalloc(sizeof(*tlvdata), GFP_KERNEL);
	sub_tlvdata =(struct tlv_data *)kzalloc(sizeof(*sub_tlvdata), GFP_KERNEL);

	deserialize_tlv(tlvdata, buf, &value_len);


	if(tlvdata->m_type  != MSG_TYPE_TS_INIT) {
		deserialize_tlv(sub_tlvdata, tlvdata->m_value,&sub_valuelen);
		//fpga_info(("sub_tlvdata: sub_type:%d  sub_valuelen:%d\n",sub_tlvdata->m_type, sub_tlvdata->m_len));
	}
	
	switch(tlvdata->m_type){
		case MSG_TYPE_TS_INIT :
			if(len != value_len+ 8){
				fpga_info(("get ts data invalid"));
			}
			ts_parser_data(tlvdata->m_value,value_len);
			//ts_parser_data(sub_tlvdata->m_value,sub_valuelen);
		
			if(g_func_num > 0) {
				func_ts_thread = (struct task_struct **)kzalloc(sizeof(struct task_struct *) * g_func_num, GFP_KERNEL);
				if(func_ts_thread) {
					fpga_info(("Create %d func ts thread success", g_func_num));
				}else {
					fpga_info(("Create  func ts thread failed"));	
					return -ENOMEM;
				}
				for(i = 0; i < g_func_num; i++)
					func_ts_thread[i] = NULL;
				
			}

			dump_func_ts();
		
			break;
			
		case MSG_TYPE_TS_CTRL:
			netlink_event_ts_handle((struct measure_related_info *)sub_tlvdata->m_value,sub_tlvdata->m_type);
			
			break;	
			
		case MSG_TYPE_OTHER_CTRL:
			fpga_info(("MSG_TYPE_OTHER_CTRL: subtype:%d\n", sub_tlvdata->m_type));
			netlink_other_ctrl_type_handle(sub_tlvdata->m_value,sub_tlvdata->m_type);
			
			break;	
	}

		
	return 0;

}



struct pfunc_data {
	int funcid;
};

static int fpga_netlink_rcv(struct sk_buff *skb)
{
	int type, err=0;
	char *umsg = NULL;
	char *kmsg = "hello user ---> from kernel!!!\n";
	char *connect_msg = "connect success\n";
	char *cmd = "run";
	int msg_len;
	int func_ts_index;
	static int i= 0,j=0;
	int len1, len2, len3;
	char * tmp ;
	struct tlv_data * tlvheader;
	int nl_msg_len, nl_msg_type;
	
	struct pfunc_data *pfunc_id = (struct pfunc_data *)kzalloc(sizeof(*pfunc_id),GFP_KERNEL);

#define FPGA_TIMING 4
#define FPGA_EXIT 3
#define FPGA_EXEC 1


	struct nlmsghdr *nlh = nlmsg_hdr(skb);


	//netlink_port = nlh->nlmsg_pid;
	//fpga_info(("fpga_netlink_rcv, netlink_port=%d\n",netlink_port));

	if (nlh->nlmsg_len < sizeof(*nlh)/* || skb->len < nlh->nlmsg_len*/)
		return err;
	

	msg_len = nlh->nlmsg_len -NLMSG_HDRLEN;


	tmp = NLMSG_DATA(nlh);

	
	tlvheader = (struct tlv_data *)tmp;
	nl_msg_len = tlvheader->m_len + 8;
	nl_msg_type = tlvheader->m_type;

	

	if(nl_msg_type > 5 || nl_msg_type < 1) {
		fpga_info(("!!!!!Not a valid supported protocol!!!!!! "));
		return 0;
	}
	
	//printk("skb raw data:\n");
	//for(i=0;i<60;i++)
	//	printk("%d ", tmp[i]);
	//printk("\n");
	netlink_parser_data_from_ics((char *)NLMSG_DATA(nlh), nl_msg_len);

#if 0	
	//fpga_netlink_send(kmsg, strlen(kmsg), nlh->nlmsg_pid);
	if(strncmp("getts",NLMSG_DATA(nlh),msg_len) == 0){	
		ts_parser_data_from_file();	

		if(g_func_num > 0)
			ts_thread1 = (struct task_struct *)kzalloc(sizeof(struct task_struct) * g_func_num, GFP_KERNEL);
	}

	if(strncmp("dumpts",NLMSG_DATA(nlh),msg_len) == 0){	
		i=0;
		j = 0;
		dump_func_ts();		
	}

	if(strncmp("run_functs",NLMSG_DATA(nlh),msg_len) == 0){	
		pfunc_id->funcid = 0x1870c;
		func_ts_index = get_func_ts_id_index(pfunc_id->funcid);
		func_ts_index = 0;
		printk("thread num :%d\n",i);
		ts_thread1[i] = kthread_run(fpga_exec_functs_thread, pfunc_id, "fpga_timseq_thread");
		if (IS_ERR(ts_thread1[i])) {
			err = PTR_ERR(ts_thread1[i]);
			return err;
		}
		printk("@@@@func id: %d  thread pid:%d\n",func_ts_index, ts_thread1[i]->pid);
		i++;
		msleep(4000);
	}

	
	if(strncmp("stop_functs",NLMSG_DATA(nlh),msg_len) == 0){	
		pfunc_id->funcid = 0x1870c;
		func_ts_index = get_func_ts_id_index(pfunc_id->funcid);
		if(ts_thread1[j]) {	
			printk("stop thread num:%d\n",j);
			kthread_stop(ts_thread1[j]);
			ts_thread1[j]=NULL;
		}
		j++;
	
	}
#endif

	return err;
}


/*
 * Module Init / Exit
 */

static int gpmc_fpga_of_probe(struct platform_device *op)
{
	struct device_node *of_node = op->dev.of_node;
	struct device *this_device;
	struct fpga_dev *priv;
	struct resource res;
	dma_cap_mask_t mask;
	int ret;
	u16 fpgaver;
	unsigned long fpgabase;
	struct resource *res1;
	struct task_struct **test_th;
	long test_diff_time;
	unsigned rx_req;
	
	struct dma_chan *fpga_chan;
	
	struct resource *resdma;

	struct sched_param param = {  .sched_priority = 1 };
	
	fpga_info(("Ant FPGA Driver Probe Start ...\n"));

	/* Allocate fpga device private data */
	g_fpga_dev = priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&op->dev, "Unable to allocate device private data\n");
		ret = -ENOMEM;
		goto out_return;
	}

	platform_set_drvdata(op, priv);
	priv->dev = &op->dev;
	kref_init(&priv->ref);
	mutex_init(&priv->mutex);

	dev_set_drvdata(priv->dev, priv);

	spin_lock_init(&priv->lock);
	INIT_LIST_HEAD(&priv->free);
	INIT_LIST_HEAD(&priv->used);
	
	/* Setup the misc device */
	priv->miscdev.minor = MISC_DYNAMIC_MINOR;
	priv->miscdev.name = drv_name;
	priv->miscdev.fops = &gpmc_fpga_fops;


	/* Get the physical address of the FPGA registers */
	ret = of_address_to_resource(of_node, 0, &res);
	if (ret) {
		dev_err(&op->dev, "Unable to find FPGA physical address\n");
		ret = -ENODEV;
		goto out_free_priv;
	}

#if 0
	printk("of_node: 0x%x\n  priv->dev_of_node:0x%x", of_node, priv->dev->of_node);
	fpga_chan = of_dma_request_slave_channel(of_node, "fpga-rx");
	if (IS_ERR(fpga_chan)){
		printk("get dma chan error\n");

	}
#endif

	/* gpmc phys addr */
	priv->phys_addr = (dma_addr_t)res.start;
	priv->phys_size = resource_size(&res);

	//fpga_info(("fpga phys_addr:0x%x, size:0x%x\n",priv->phys_addr,priv->phys_size ));


	/* ioremap the registers for gpmc virt addr use */
	priv->regs = of_iomap(of_node, 0);
	if (!priv->regs) {
		dev_err(&op->dev, "Unable to ioremap registers\n");
		ret = -ENOMEM;
		goto out_free_priv;
	}



	if (!op->dev.of_node) {

		resdma = platform_get_resource_byname(op, IORESOURCE_DMA, "fpga-rx");
		if (!resdma) {
			dev_err(&op->dev, "Unable to IORESOURCE_DMA registers\n");
			ret = -ENOMEM;
			goto out_free_priv;
		}
		rx_req = resdma->start;
		printk("dma rx_req: %d\n", rx_req);
	}
#if 0


	priv->fpga_rst = of_get_named_gpio(of_node, "fpga-reset", 0);
	if (!gpio_is_valid(priv->fpga_rst)) {
		dev_err(&op->dev, "unadble to get dt reset property\n");
		goto out_unmap_regs;
	}

	ret = devm_gpio_request_one(&op->dev, priv->fpga_rst, GPIOF_OUT_INIT_LOW, "fpga-rest");
	if(ret) {
		dev_err(&op->dev, "failed to request reset gpio %d: %d\n",priv->fpga_rst, ret);
		goto out_unmap_regs;
	}
	
	priv->fpga_rst = of_get_named_gpio(of_node, "fpga-ready", 0);
	if (!gpio_is_valid(priv->fpga_ready)) {
		dev_err(&op->dev, "unadble to get dt ready property\n");
		goto out_unmap_regs;
	}
	
	ret = devm_gpio_request_one(&op->dev, priv->fpga_rst, GPIOF_IN, "fpga-rest");
	if(ret) {
		dev_err(&op->dev, "failed to request ready gpio %d: %d\n",priv->fpga_rst, ret);
		goto out_unmap_regs;
	}
#endif	



	
	/* Drive the GPIO for FPGA IRQ high (no interrupt) */

	/* prevent the FPGAs from generating interrupts */
	
	init_completion(&rbcinfo_wait);
	init_completion(&wbcinfo_wait);

	init_completion(&diffinfo_wait);

	/*  create fpga read data wait queue used in interrupt handle for read data */
	init_waitqueue_head(&priv->fpga_rq);

	/* create fpga workqueue for set serial of  fpga sequence */
	
	//priv->wqfifo = create_singlethread_workqueue("read_fifo");
	priv->wqfifo  = alloc_workqueue("read_fifo",
				WQ_MEM_RECLAIM |WQ_HIGHPRI |WQ_UNBOUND, 1);
	if( priv->wqfifo == NULL) {
		dev_err(priv->dev, "create read_fifo workqueue failed\n");
		goto out_irq_dispose_mapping;
	}
	//set_user_nice(priv->wqfifo->rescuer->task,-10);

	//INIT_DELAYED_WORK();


	/* read fpga data(sysinfo, hgb, wbc...) , in interrupt handle thread*/
	//INIT_WORK(&priv->fifo_sys, fpga_fifo_sys_work);

	/* read fpga data(sysinfo, hgb, wbc...) , in interrupt handle thread*/
	//INIT_WORK(&priv->fifo_wbc, fpga_fifo_wbc_work);
	/* set fpga work sequence time,in read caller */
	INIT_WORK(&priv->fifo_rbc, fpga_fifo_rbc_work);
	INIT_WORK(&priv->fifo_wbc, fpga_fifo_wbc_work);
	INIT_WORK(&priv->fifo_diff, fpga_fifo_diff_work);
	/* set fpga work sequence time,in read caller */
	//INIT_WORK(&priv->sys_float, fpga_sys_float_work);

#if 1
	ret = fpga_dma_init(priv,0);
	if(ret != 0){
		dev_err(priv->dev, "Register DMA  Failed\n");
		goto out_destroy_workqueue;
	}

#endif

	//fpga_dma_memcpy_init(priv,0);
		

	fpga_timer_init(priv);


		
	spin_lock_irq(&priv->lock);
	//priv->enabled = true;
	spin_unlock_irq(&priv->lock);

	/* allow the FPGAs to generate interrupts */


	/* test if can read fpga success */
	fpgaver = fpga_read_reg(priv, GPMC_FPGA_VERSION);
	fpga_info(("Detect fpga version:0x%x\n\n", fpgaver));

	/* Register the miscdevice */
	ret = misc_register(&priv->miscdev);
	if (ret) {
		dev_err(&op->dev, "Unable to register miscdevice\n");
		goto out_destroy_workqueue;
	}

	/* Create the debugfs files */
	ret = gpmc_fpga_debugfs_init(priv);
	if (ret) {
		dev_err(&op->dev, "Unable to create debugfs files\n");
		goto out_misc_deregister;
	}

	/* Create the sysfs files */
	this_device = priv->miscdev.this_device;
	dev_set_drvdata(this_device, priv);
	ret = sysfs_create_group(&this_device->kobj, &fpga_sysfs_attr_group);
	if (ret) {
		dev_err(&op->dev, "Unable to create sysfs files\n");
		goto out_gpmc_fpga_debugfs_exit;
	}

	load_motor_pos_steps();
	msleep(100);
	
	set_motor_init_accspeed();


	//fpga_write_reg(g_fpga_dev, 0x40002, 0xff);
	//fpga_write_reg(g_fpga_dev, 0x40004, 0xff);

	priv->sche_wa_flga=1;	
	//queue_work(priv->wqfifo,&priv->fifo_rbc); //read wq to handle dma data;

	printk("11111\n");

#if 1
		/* Find the correct IRQ number */
	priv->rbc_irq = irq_of_parse_and_map(of_node, 0);
	if (priv->rbc_irq == NO_IRQ) {
		dev_err(&op->dev, "Unable to find fpga irq line\n");
		ret = -ENODEV;
		goto out_unmap_regs;
	}
	
	printk("priv->rbc_irq:%d\n", priv->rbc_irq);
#endif


	irq_set_irq_type(priv->rbc_irq,IRQ_TYPE_EDGE_RISING);
		/* hookup the irq handler */
	ret = request_irq(priv->rbc_irq,  gpmc_fpga_rbc_irq, IRQF_SHARED, "rbc-irqhandler", priv);
	if (ret) {
		dev_err(priv->dev, "unable to request rbc IRQ handler\n");
		goto out_gpmc_fpga_debugfs_exit;
	}

	
	priv->wbc_irq = 68;   //wbc irq number;	
	irq_set_irq_type(priv->wbc_irq,IRQ_TYPE_EDGE_RISING);
		/* hookup the irq handler */
	ret = request_irq(priv->wbc_irq,  gpmc_fpga_wbc_irq, IRQF_SHARED, "wbc-irqhandler", priv);
	if (ret) {
		dev_err(priv->dev, "unable to request wbc IRQ handler\n");
		goto out_gpmc_fpga_debugfs_exit;
	}
	printk("IRQ register success\n");



#if 1

	priv->diff_irq = 101;   //diff irq number;	
	irq_set_irq_type(priv->diff_irq,IRQ_TYPE_EDGE_RISING);
		/* hookup the irq handler */
	ret = request_irq(priv->diff_irq,  gpmc_fpga_diff_irq, IRQF_SHARED, "diff-irqhandler", priv);
	if (ret) {
		dev_err(priv->dev, "unable to request diff IRQ handler\n");
		goto out_gpmc_fpga_debugfs_exit;
	}

#endif

	struct netlink_kernel_cfg cfg = {
		.input	= fpga_netlink_rcv,
		.flags 	= NL_CFG_F_NONROOT_RECV,
	};

	g_fpga_dev->fpga_nlsk = (struct sock *)netlink_kernel_create(&init_net, NETLINK_FPGA, &cfg);
	if (!g_fpga_dev->fpga_nlsk){
		fpga_info(("netlink create failed"));
		goto out_irq_dispose_mapping;
	}
	fpga_info(("Netlink create success\n"));


	fpga_info(("ANT GPMC FPGA  Data Driver Loaded Success"));
	
	return 0;

out_irq_dispose_mapping:
	free_irq(priv->irq, priv);
	irq_dispose_mapping(priv->irq);

out_gpmc_fpga_debugfs_exit:
	gpmc_fpga_debugfs_exit(priv);
out_misc_deregister:
	misc_deregister(&priv->miscdev);
out_destroy_workqueue:
	flush_workqueue(priv->wqfifo);
	if(priv->wqfifo)
		destroy_workqueue(priv->wqfifo);		
out_unmap_regs:
	iounmap(priv->regs);
out_free_priv:
	kref_put(&priv->ref, gpmc_fpga_device_release);
	
out_return:
	return ret;
}

static int gpmc_fpga_of_remove(struct platform_device *op)
{
	struct fpga_dev *priv = platform_get_drvdata(op);
	struct device *this_device = priv->miscdev.this_device;


	netlink_kernel_release(priv->fpga_nlsk);


		/* unhook the irq handler */
	free_irq(priv->rbc_irq, priv);
	irq_dispose_mapping(priv->rbc_irq);
	
	free_irq(priv->wbc_irq, priv);
	irq_dispose_mapping(priv->wbc_irq);

	free_irq(priv->diff_irq, priv);
	irq_dispose_mapping(priv->diff_irq);

	printk("@@@@@@@@@remove@@@@@\n");
	/* remove all sysfs files, now the device cannot be re-enabled */
	sysfs_remove_group(&this_device->kobj, &fpga_sysfs_attr_group);

	/* remove all debugfs files */
	gpmc_fpga_debugfs_exit(priv);

	/* remove the character device to stop new readers from appearing */
	misc_deregister(&priv->miscdev);

	dma_release_channel(priv->fpga_rbc_dma_chan);
	dma_release_channel(priv->fpga_wbc_dma_chan);
	dma_release_channel(priv->fpga_diff_dma_chan);

	/* cleanup everything not needed by readers */


			

	iounmap(priv->regs);
	
	flush_workqueue(priv->wqfifo);
	if(priv->wqfifo)
		destroy_workqueue(priv->wqfifo);		
	printk("@@@@@@@@@111111@@@@@\n");
	
	dma_free_coherent(priv->fpga_rbc_dma_chan->device->dev, FIFO_DATA_DMA_SIZE*sizeof(u16),
			 priv->rbcout_addr , priv->rbcout_dma_addr);
	
	printk("@@@@@@@@@3222222@@@@@\n");
	dma_free_coherent(priv->fpga_wbc_dma_chan->device->dev, FIFO_DATA_DMA_SIZE*sizeof(u16),
			 priv->wbcout_addr , priv->wbcout_dma_addr);

	dma_free_coherent(priv->fpga_diff_dma_chan->device->dev, FIFO_DATA_DMA_SIZE*sizeof(u16),
			 priv->diffout_addr , priv->diffout_dma_addr);

	/* release our reference */
	kref_put(&priv->ref, gpmc_fpga_device_release);
	return 0;
}

static const struct of_device_id gpmc_fpga_of_match[] = {
	{ .compatible = "mayi,gpmc-fpga", },
	{},
};

static struct platform_driver gpmc_fpga_of_driver = {
	.probe		= gpmc_fpga_of_probe,
	.remove		= gpmc_fpga_of_remove,
	.driver		= {
		.name		= drv_name,
		.of_match_table	= gpmc_fpga_of_match,
	},
};


int get_func_ts_id_index(int func_ts_id)
{
	int index = 0;

	for(index = 0; index < g_func_num; index++) {
		//printk("index:%d--> funcid:%d ---func_ts_id:%d\n", index, g_func_tshdr[index].func_id, func_ts_id);
		if(func_ts_id == g_func_tshdr[index].func_id)
			break;
	}


	if(index >= g_func_num) {
		fpga_info(("get_func_ts_id_index: can not found the func_ts_id:%d",func_ts_id));
		return -1;
	}	
	
	return index;
}



static enum hrtimer_restart func_ts_timer_handler(struct hrtimer *hrtimer)
{
	struct func_ts_timer *ft_timer = container_of(hrtimer, struct func_ts_timer, ts_timer);
	
	complete(ft_timer->ts_complete);
	
	return HRTIMER_NORESTART;
}

struct timespec tc_old;
long delay_time;

#if 1
static tscmd_exe_thread(void *p)
{
	struct cmd_contex  *cmdinfo;
	struct list_head *pos2, *n2;
	int j=0;

	struct sched_param param = {  .sched_priority = 1 };
	sched_setscheduler(current, SCHED_RR,  &param);

	
	struct time_info  *tminfo = (struct time_info *)p;	

	list_for_each_safe(pos2,n2, &tminfo->ta_list) {
		
		//if(kthread_should_stop()) {
		//	goto exit_thread;
		//}

		//if(j == 1)
		//	break;
		
		cmdinfo = list_entry(pos2, struct cmd_contex, cmd_list );
	
		fpga_ctrl[cmdinfo->m_dev_type].pfun( cmdinfo->m_dev_num, cmdinfo->m_timeout, cmdinfo->m_action,cmdinfo->m_optional_1, cmdinfo->m_optional_2, cmdinfo->m_optional_3, cmdinfo->m_optional_4,tminfo->pts_mri);

		//fpga_info(("\tcmd:%d --->  para:m_dev_type:%d	m_dev_num:%d , m_action:%d, timout:%d op1:%d, op2:%d, op3:%d, op4:%d",j, cmdinfo->m_dev_type,cmdinfo->m_dev_num,cmdinfo->m_action,cmdinfo->m_timeout,cmdinfo->m_optional_1,cmdinfo->m_optional_2,cmdinfo->m_optional_3,cmdinfo->m_optional_4)); 		
		j++;
		
	}
	

}


long timarry[50]={0};

int fpga_exec_functs_thread(void *p)
{

	struct list_head *pos1, *n1, *pos2, *n2;
	struct time_info  *tminfo;
	struct cmd_contex  *cmdinfo;
	int i = 0,j=0;
	int func_ts_index = 0;
	int tsid ;
	int index;
	int ret = 0;
	int old_tstime=0,new_tstime;
	int k=0;
	struct task_struct ** ts_thread;
	struct timespec tc_new;
	struct timespec ts1, ts2,ts3;
	
	int ts_cnt;
	
	struct sched_param param = {  .sched_priority = 1 };

	//struct task_struct *cmd_thread[30];

	getnstimeofday(&ts1);	


	DECLARE_COMPLETION_ONSTACK(ts_complete);
	
	struct measure_related_info * mri =(struct measure_related_info *)p;

	struct func_ts_timer *ft_timer = (struct func_ts_timer *)kzalloc( sizeof(*ft_timer), GFP_KERNEL);


	if( g_func_tshdr == NULL || g_func_num == 0 ) {
		fpga_info(("Func TS Head List IS Not Initialize, Please Check..."));
		return -1;
	}
	
	func_ts_index = get_func_ts_id_index(mri->m_ts_id);
	if(func_ts_index < 0)
		return -1;


		
	hrtimer_init(&ft_timer->ts_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ft_timer->ts_timer.function = func_ts_timer_handler;
	ft_timer->ts_complete = &ts_complete;


	ts_cnt = g_func_tshdr[func_ts_index].tm_cnt;

	fpga_info(("\nexcute func_ts id:0x%x  total tm block:%d\n",g_func_tshdr[func_ts_index].func_id, ts_cnt));


	ts_thread = (struct task_struct **)kzalloc(sizeof(struct task_struct *) * ts_cnt, GFP_KERNEL);
	if(ts_thread) {
		fpga_info(("Create %d tmseq thread success", ts_cnt));
	}else {
		fpga_info(("Create	tmseq thread failed"));	
		return -ENOMEM;
	}


	getnstimeofday(&ts2);	



	i = 0;

	list_for_each_safe(pos1,n1,&g_func_tshdr[func_ts_index].g_lh) {	
		tminfo = list_entry(pos1, struct time_info, tm_list);	
		tminfo->pts_mri = mri;

		ts_thread[i] = kthread_create(tscmd_exe_thread, tminfo, "tscmd_exe_thread");
		if (IS_ERR(ts_thread)) {
			ret = PTR_ERR(ts_thread);
			return ret;
		}
		
		sched_setscheduler(ts_thread[i], SCHED_RR,  &param);

		i++;
		
	}
	


	encode_tlv_netlink_send_ts_rpt(MSG_TYPE_TS_REPORT, SUBTYPE_TS_REPORT_START, mri, sizeof(struct measure_related_info));

	i=0;
	list_for_each_safe(pos1,n1,&g_func_tshdr[func_ts_index].g_lh) {	

	
		//if(kthread_should_stop()){
		//	goto exit_thread;
		//}
		
		tminfo = list_entry(pos1, struct time_info, tm_list);	


		delay_time =  tminfo->tm - old_tstime;		
		old_tstime = tminfo->tm;
		
		//fpga_info(("tm block:%d ---> m_time:(%d ms) cmd_cnt:0x%x \n",i+1, delay_time,tminfo->action_cnt));
		

		//getnstimeofday(&tc_old);	

		hrtimer_start(&ft_timer->ts_timer, ms_to_ktime(delay_time), HRTIMER_MODE_REL);


		wait_for_completion(&ts_complete);

		wake_up_process(ts_thread[i]);
		
		i++;

	}
	getnstimeofday(&ts3);	

	//printk("@@ (ts3:%lds .%ldus) -(ts2:%ld s.%ld us) -(ts1:%ld s.%ld us)->ts2diff =%ld  ts3diff =%ldus\r",  ts3.tv_sec, ts3.tv_nsec/1000, ts2.tv_sec, ts2.tv_nsec/1000,ts1.tv_sec, ts1.tv_nsec/1000,	((ts2.tv_sec*1000*1000 + ts2.tv_nsec/1000 )	-  (ts1.tv_sec*1000*1000 + ts1.tv_nsec/1000 )),((ts3.tv_sec*1000*1000 + ts3.tv_nsec/1000 )	-  (ts1.tv_sec*1000*1000 + ts1.tv_nsec/1000 )));

		
exit_thread:
	encode_tlv_netlink_send_ts_rpt(MSG_TYPE_TS_REPORT, SUBTYPE_TS_REPORT_END, mri, sizeof(struct measure_related_info));
	//fpga_info(("\tNow Sending SUBTYPE_TS_REPORT_END MSG To Userspace...\n"));
	printk("rbc_irq:%d  rbc_total:%d\n", rbc_irq, rbc_total);
	printk("wbc_irq:%d  wbc_total:%d\n", wbc_irq, wbc_total);
	printk("diff_irq:%d  diff_total:%d\n", diff_irq, diff_total);

#if 0	
	printk("rbctime sch:dmatime\n");

	for(i = 0, j=0; i< intcnt; ) {
		printk("%d : %d-%d  \n",rbctime[i], rbctime[i+1], rbc_xfer_len[j]);
		if((i+1)/2 == 0)
			printk("\n");	
		i +=2;
		j++;
	}

	printk("rbc copy time:\n");
	for(i = 0; i< cpycnt; ) {
		printk("%d : %d \n",rbc_cpy_time[i], rbc_cpy_time[i+1] );
		if((i+1)/2 == 0)
			printk("\n");	
		i +=2;
	}
	printk("\n");		

	//fpga_info(("\tfpga_timseq_thread (pid:%d) end....",current->pid));
#endif

	if(ft_timer)
		kfree(ft_timer);
	if(mri)
		kfree(mri);

	return 0;
}

#else

long timarry[50]={0};


static int fpga_exec_functs_thread(void *p)
{

	struct list_head *pos1, *n1, *pos2, *n2;
	struct time_info  *tminfo;
	struct cmd_contex  *cmdinfo;
	int i = 0,j=0;
	int func_ts_index = 0;
	int tsid ;
	int index;
	int ret = 0;
	int old_tstime=0,new_tstime;
	
	int k=0;
	struct task_struct ** cmd_thread;
	struct timespec tc_new;

	//struct task_struct *cmd_thread[30];
	

	DECLARE_COMPLETION_ONSTACK(ts_complete);
	
	struct measure_related_info * mri =(struct measure_related_info *)p;

	struct func_ts_timer *ft_timer = (struct func_ts_timer *)kzalloc( sizeof(*ft_timer), GFP_KERNEL);


	if( g_func_tshdr == NULL || g_func_num == 0 ) {
		fpga_info(("Func TS Head List IS Not Initialize, Please Check..."));
		return -1;
	}
	
	func_ts_index = get_func_ts_id_index(mri->m_ts_id);
	if(func_ts_index < 0)
		return -1;

	
	hrtimer_init(&ft_timer->ts_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ft_timer->ts_timer.function = func_ts_timer_handler;
	ft_timer->ts_complete = &ts_complete;

	
	//fpga_info(("\nexcute func_ts id:0x%x  total tm block:%d\n",g_func_tshdr[func_ts_index].func_id,g_func_tshdr[func_ts_index].tm_cnt));
	//fpga_info(("Now Sending SUBTYPE_TS_REPORT_START MSG To Userspace...\n"));

	
	fpga_info(("func_ts thread : pid:%d(func ts id:%d) started\n", current->pid, mri->m_ts_id));

	encode_tlv_netlink_send_ts_rpt(MSG_TYPE_TS_REPORT, SUBTYPE_TS_REPORT_START, mri, sizeof(struct measure_related_info));

	list_for_each_safe(pos1,n1,&g_func_tshdr[func_ts_index].g_lh) {
		i++;
		j=0;

		//if(kthread_should_stop()){
		//	goto exit_thread;
		//}
		
		tminfo = list_entry(pos1, struct time_info, tm_list);		

		
		delay_time =  tminfo->tm - old_tstime;		
		old_tstime = tminfo->tm;

		//fpga_info(("\n\n\n\t\t tm block:%d ---> m_time:(%d ms) cmd_cnt:0x%x ",i, delay_time,tminfo->action_cnt));

	
		hrtimer_start(&ft_timer->ts_timer, ms_to_ktime(delay_time), HRTIMER_MODE_REL);
		getnstimeofday(&tc_old);	

		wait_for_completion(&ts_complete);

		list_for_each_safe(pos2,n2, &tminfo->ta_list) {
			
			//fpga_info(("\n\t\tcmd:%d --->  para:m_dev_type:%d   m_dev_num:%d , m_action:%d, timout:%d op1:%d, op2:%d, op3:%d, op4:%d",j, cmdinfo->m_dev_type,cmdinfo->m_dev_num,cmdinfo->m_action,cmdinfo->m_timeout,cmdinfo->m_optional_1,cmdinfo->m_optional_2,cmdinfo->m_optional_3,cmdinfo->m_optional_4));			
			//if(kthread_should_stop()) {
			//	goto exit_thread;
			//}

			cmdinfo = list_entry(pos2, struct cmd_contex, cmd_list );

			cmdinfo->pcmd_mri = mri;

			//timarry[k] =((tc_new.tv_sec*1000*1000 + tc_new.tv_nsec/1000 )	-  (tc_old.tv_sec*1000*1000 + tc_old.tv_nsec/1000 )) - diff_time*1000;
			//k++;


			//getnstimeofday(&tc_new);
			//fpga_info(("@@New time seq :timeseq_delay:%d --> (nowtime:%lds .%ldus) -(pretime:%ld s.%ld us) -->interval_diff =%ldus\r", delay_time, tc_new.tv_sec, tc_new.tv_nsec/1000, tc_old.tv_sec, tc_old.tv_nsec/1000,	((tc_new.tv_sec*1000*1000 + tc_new.tv_nsec/1000 )	-  (tc_old.tv_sec*1000*1000 + tc_old.tv_nsec/1000 )) - delay_time*1000));

			j++;
			fpga_ctrl[cmdinfo->m_dev_type].pfun( cmdinfo->m_dev_num, cmdinfo->m_timeout, cmdinfo->m_action,cmdinfo->m_optional_1, cmdinfo->m_optional_2, cmdinfo->m_optional_3, cmdinfo->m_optional_4,mri);
		}
		
	}
		printk("time diff\n");
		for(i=0;i<50;i++)
			printk("%ldus ", timarry[i]);
		printk("\n");
		
exit_thread:
	encode_tlv_netlink_send_ts_rpt(MSG_TYPE_TS_REPORT, SUBTYPE_TS_REPORT_END, mri, sizeof(struct measure_related_info));
	//fpga_info(("\tNow Sending SUBTYPE_TS_REPORT_END MSG To Userspace...\n"));

	//fpga_info(("\tfpga_timseq_thread (pid:%d) end....",current->pid));
	
	if(ft_timer)
		kfree(ft_timer);
	if(mri)
		kfree(mri);

	return 0;
}


#endif



#if 0

static int fpga_exec_functs_thread(void *p)
{

	struct list_head *pos1, *n1, *pos2, *n2;
	struct time_info  *tminfo;
	struct cmd_contex  *cmdinfo;
	int i = 0,j=0;
	int func_ts_index = 0;
	int tsid ;
	int index;
	int ret = 0;
	int old_tstime=0,new_tstime;
	int k=0;
	struct task_struct ** ts_thread;
	struct timespec tc_new;
	int ts_cnt;
	

	//struct task_struct *cmd_thread[30];
	for(i = 0; i < 50; i++)
		timarry[i]= 0;

	DECLARE_COMPLETION_ONSTACK(ts_complete);
	
	struct measure_related_info * mri =(struct measure_related_info *)p;

	struct func_ts_timer *ft_timer = (struct func_ts_timer *)kzalloc( sizeof(*ft_timer), GFP_KERNEL);


	if( g_func_tshdr == NULL || g_func_num == 0 ) {
		fpga_info(("Func TS Head List IS Not Initialize, Please Check..."));
		return -1;
	}
	
	func_ts_index = get_func_ts_id_index(mri->m_ts_id);
	if(func_ts_index < 0)
		return -1;

	
	hrtimer_init(&ft_timer->ts_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ft_timer->ts_timer.function = func_ts_timer_handler;
	ft_timer->ts_complete = &ts_complete;


	ts_cnt = g_func_tshdr[func_ts_index].tm_cnt;

	fpga_info(("\nexcute func_ts id:0x%x  total tm block:%d\n",g_func_tshdr[func_ts_index].func_id, ts_cnt));


	ts_thread = (struct task_struct **)kzalloc(sizeof(struct task_struct *) * ts_cnt, GFP_KERNEL);
	if(ts_thread) {
		fpga_info(("Create %d tmseq thread success", ts_cnt));
	}else {
		fpga_info(("Create	tmseq thread failed"));	
		return -ENOMEM;
	}

	list_for_each_safe(pos1,n1,&g_func_tshdr[func_ts_index].g_lh) {	
		tminfo = list_entry(pos1, struct time_info, tm_list);	

		ts_thread[i] = kthread_run(tscmd_exe_thread, tminfo, "tscmd_exe_thread");
		if (IS_ERR(ts_thread)) {
			ret = PTR_ERR(ts_thread);
			return ret;
		}
		
	}

	encode_tlv_netlink_send_ts_rpt(MSG_TYPE_TS_REPORT, SUBTYPE_TS_REPORT_START, mri, sizeof(struct measure_related_info));

	
	i=0;
	list_for_each_safe(pos1,n1,&g_func_tshdr[func_ts_index].g_lh) {	

	
		//if(kthread_should_stop()){
		//	goto exit_thread;
		//}
		
		if(i ==2)
			break;

	
		tminfo = list_entry(pos1, struct time_info, tm_list);	

			
		tminfo->pts_mri = mri;

		delay_time =  tminfo->tm - old_tstime;		
		old_tstime = tminfo->tm;
		
		fpga_info(("tm block:%d ---> m_time:(%d ms) cmd_cnt:0x%x \n",i+1, delay_time,tminfo->action_cnt));
		


		hrtimer_start(&ft_timer->ts_timer, ms_to_ktime(delay_time), HRTIMER_MODE_REL);
		getnstimeofday(&tc_old);	

		wait_for_completion(&ts_complete);

		ts_thread[i] = kthread_run(tscmd_exe_thread, tminfo, "tscmd_exe_thread");
		if (IS_ERR(ts_thread)) {
			ret = PTR_ERR(ts_thread);
			return ret;
		}
		i++;

	}

	msleep(10000);

	printk("time diff\n");
	for(i=0;i<50;i++)
		printk("%ldus ", timarry[i]);
	printk("\n");
		
exit_thread:
	encode_tlv_netlink_send_ts_rpt(MSG_TYPE_TS_REPORT, SUBTYPE_TS_REPORT_END, mri, sizeof(struct measure_related_info));
	//fpga_info(("\tNow Sending SUBTYPE_TS_REPORT_END MSG To Userspace...\n"));

	//fpga_info(("\tfpga_timseq_thread (pid:%d) end....",current->pid));
	
	if(ft_timer)
		kfree(ft_timer);
	if(mri)
		kfree(mri);

	return 0;
}


#endif

static int fpga_timseq_thread(void *p)
{
	struct fpga_dev *priv = p;

	struct time_info  *tinfo;
	struct to_dev_ai  *fai;
	struct list_head *pos1, *n1, *pos2, *n2;
	int act_cnt = 0;
	char *msgexit="exit\n";
	
	fpga_info(("fpga_timseq_thread start thread name:%s pid:0x%x....",current->comm,current->pid));
	
	list_for_each_safe(pos1,n1,&g_tehdr.g_lh) {
		tinfo = list_entry(pos1, struct time_info, tm_list);
		//fpga_info(("tm block:%d  tm:0x%x action_cnt:0x%x ",i, tinfo->tm, tinfo->action_cnt));		
		msleep(tinfo->tm*1000);
		list_for_each_safe(pos2,n2, &tinfo->ta_list) {
			fai = list_entry(pos2, struct to_dev_ai, action_list );
			{
				fpga_info(("\n\nAt time:%d action %d: action type:%d devtype:%d param0:%d", tinfo->tm,act_cnt, fai->action_type,fai->dev,fai->action_p1));
				if(fai->action_type == FPGA_CONTROL_PATH){
					fpga_info(("control path action"));
					//if(fai->dev < FPGA_CTRL_DEV_TYPE_MIN || fai->dev > FPGA_CTRL_DEV_TYPE_MAX)
					//	fpga_info(("control path dev type invalid"));
					//fpga_ctrl[fai->dev].pfun( fai->dev_no, fai->timeout, fai->action_p1,fai->action_p2, fai->action_p3, fai->action_p4, fai->action_p5);
					
				}else if(fai->action_type == FPGA_DATA_PATH) {
					fpga_info(("data path action"));
					
				}
				
				act_cnt++;
			}
		}
	}
	msleep(10*1000);
	
exit_thread:	
	fpga_info(("\n\nfpga_timseq_thread end...."));
	//fpga_netlink_send(msgexit, strlen(msgexit), netlink_port);


	return 0;
}

int fpga_netlink_send(char *toubuf, u32 len, u32 port)
{
	struct sk_buff *nl_skb;
	struct nlmsghdr *nlh;
	int ret;

	netlink_port = 100;


	
	nl_skb = nlmsg_new(len, GFP_ATOMIC);
	if(!nl_skb) {
		fpga_error(("netlink alloc failure"));
		return -ENOMEM;
	}
	
	nlh = nlmsg_put(nl_skb, 0, 0, NETLINK_FPGA, len,  0);
	if(!nlh) {
		fpga_error(("nlmsg_put alloc failure"));
		return -ENOMEM;
	}

	memcpy(nlmsg_data(nlh), toubuf, len);
	//printk("len:%d fpga_nlsk=0x%x  nl_skb=0x%x  g_fpga_dev:0x%x\n", len, g_fpga_dev->fpga_nlsk, nl_skb, g_fpga_dev);

	ret = nlmsg_unicast(g_fpga_dev->fpga_nlsk, nl_skb, netlink_port);

	return ret;
	
}



static int __init ant_fpga_init(void)
{

	return platform_driver_register(&gpmc_fpga_of_driver);
}

static void __exit ant_fpga_exit(void)
{
	int i=0;
	
	platform_driver_unregister(&gpmc_fpga_of_driver);
	
	//netlink_kernel_release(fpga_nlsk);
	kfree(g_func_tshdr);
}

module_init(ant_fpga_init);
module_exit(ant_fpga_exit);


MODULE_AUTHOR("wangp");
MODULE_DESCRIPTION("MAYI GPMC-FPGA Access Driver");
MODULE_LICENSE("GPL");
