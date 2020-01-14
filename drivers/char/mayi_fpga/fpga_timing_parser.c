/*
 * Kuke Ant GPMC-FPGA Access Driver-timing parser
 *
 * Copyright (c) 2019 wangping
 *
 * This program is through gpmc interface to access fpga resource;
 * use dma to get sysinfo, wbc, hgb data, and use cpu interface to get 
 * fpga control regs.
 */

/*
 * this is only for timing parser from raw data file
 *
 * 
 */

#include <linux/vmalloc.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/kref.h>
#include <linux/io.h>
#include <linux/sizes.h>
#include <linux/delay.h>
#include <linux/list.h>

#include "gpmc_fpga.h"
#include "fpga_timing_parser.h"

int fpga_msg_level = FPGA_INFO_VAL;

struct timing_exec_hdr g_tehdr;

//used for global list node;
struct list_head g_timing_hd;
extern int netlink_port;

extern struct fpga_dev *g_fpga_dev;


void timing_parser_init(void)
{

	INIT_LIST_HEAD(&g_tehdr.g_lh);

}

//char *tdata_path = "/data/timing_data";
int data_len;
#define DEBUG_FPGA 
int timing_data_read(char *tdf)
{
	struct file *filep = NULL;
	struct kstat stat;
	mm_segment_t fs;
	char *raw_fmts;
	int tdata_size = 0;
	int error;
	struct timing_seq_hdr *hdr;
	unsigned int tm_cnt;
	int *tmp;
	int i;
	
	
	set_fs(KERNEL_DS);
	fs = get_fs();
	filep = filp_open(tdf, O_RDONLY, 0);
	if (IS_ERR(filep)) {
		fpga_error(("Failed to open the timing file:%s  in %s", tdf, __FUNCTION__));
		goto fail;
	}
	error = vfs_stat(tdf, &stat);
	if (error) {
		fpga_error(("Failed in %s to find file stat", __FUNCTION__));
		goto fail;
	}
	tdata_size = (int) stat.size;
	

	g_tehdr.tsi = kzalloc(tdata_size, GFP_KERNEL);
	if (g_tehdr.tsi == NULL) {
		fpga_error(("Failed to allocate raw_fmts memory"));
		goto fail;
	}
	if (vfs_read(filep, g_tehdr.tsi, tdata_size, &filep->f_pos) !=	tdata_size) {
		fpga_error(("Error: raw timing file read failed"));
		goto fail;
	}

	/* Remember header from the timing seq file */
	hdr = (struct timing_seq_hdr *)g_tehdr.tsi;
	
	//tm_cnt = (tdata_size - sizeof(struct timing_data_hdr)) / sizeof(struct timing_seq_info);
	

	if (hdr->tm_magic == TIMING_SEQ_MAGIC) {
		//if(hdr->tm_cnt != tm_cnt )
		//	fpga_warn("warn: raw timing file time seq count abnormal cm_cnt:%d", tm_cnt);
		//fpga_info(("timing seq count :%d", hdr->tm_cnt));	
	}

	g_tehdr.tm_cnt = hdr->tm_cnt;

	tmp =(int *)g_tehdr.tsi;
	
	data_len = tdata_size;
#ifdef DEBUG_FPGA	
	for(i=0;i<tdata_size/sizeof(unsigned int);i++){
		printk("0x%x ",tmp[i]);
	}
	printk("\n");
#endif	

	return 0;
fail:
	if (g_tehdr.tsi) {
		kfree(g_tehdr.tsi);
		g_tehdr.tsi = NULL;
	}
	if (!IS_ERR(filep))
		filp_close(filep, NULL);
	set_fs(fs);
	return -1;

}


 int timing_seq_action_parser(void)
{
	int i, j;
	int ts_cnt = g_tehdr.tm_cnt;
	int ret;
	
#define TM_INFO_LEN sizeof(struct time_info_user)
#define ACTION_DEV_INFO_LEN sizeof(struct timing_action_info)
#define FPGA_DEBUG

	struct time_info  *tmp_ti;
	struct to_dev_ai  *tmp_ai_dev;
	struct to_userspace_ai *tmp_ai_user; 	
	struct to_serial_ai *tmp_ai_serial;
	struct to_net_ai *tmp_ai_net;
	struct timing_action_info  *tmp_tai, *ptai;
	struct time_info_user *pti;
#ifdef FPGA_DEBUG
	struct time_info  *tinfo;
	struct to_dev_ai  *fai;
	struct list_head *pos1, *n1, *pos2, *n2;
	int *ptmp;
	int len;
#endif
	
	
	unsigned char * p_tsi = g_tehdr.tsi+sizeof(struct timing_seq_hdr);

	len = data_len-sizeof(struct timing_seq_hdr);
	ptmp = (int *)g_tehdr.tsi;
	
	
	printk("\n\n\n####################################:\n");
	printk("parser dump data buf:\n");


	//build the tm block link list;
	for ( i = 0; i < ts_cnt; i++ ){	
		tmp_ti = (struct time_info *)kzalloc(sizeof(struct time_info), GFP_KERNEL);
		pti =(struct time_info_user *) p_tsi;  //get the first tm struct ;
		tmp_ti->tm = pti->tm;
		tmp_ti->action_cnt = pti->action_cnt;
		INIT_LIST_HEAD(&tmp_ti->ta_list);
		list_add_tail(&tmp_ti->tm_list, &g_tehdr.g_lh);

		p_tsi += TM_INFO_LEN;
		
		//build the action info block link list;
		for ( j = 0; j < tmp_ti->action_cnt ; j++ ){ 

			ptai = (struct timing_action_info *)p_tsi; //get the first timing action info block struct;
			tmp_ai_dev = (struct to_dev_ai *)kzalloc(sizeof(struct to_dev_ai), GFP_KERNEL);				
			if(ptai->msgtype == MSG_TO_FPGA){
				//fpga_info(("msgtype to fpga"));
			}else if(ptai->msgtype == MSG_TO_SERIAL){	
				//fpga_info(("msgtype to serial"));
			}
			//tmp_ai_dev->msgtype = ptai->msgtype;
			memcpy((char *)tmp_ai_dev+sizeof(struct list_head),(char *)ptai,sizeof(struct timing_action_info));

			list_add_tail(&tmp_ai_dev->action_list, &tmp_ti->ta_list);
			//printk("\n tmp_ai_dev:tmp_ai_dev: msgtype:0x%x, dev:0x%x, dev_no:0x%x, timeout:0x%x\n",tmp_ai_dev->msgtype,tmp_ai_dev->dev,tmp_ai_dev->dev_no,tmp_ai_dev->timeout);
			p_tsi = p_tsi + ACTION_DEV_INFO_LEN;
	
		}

	}

#ifdef DEBUG_FPGA
	fpga_info(("timing seq action dump info:"));
	i=0;
	j=0;
	list_for_each_safe(pos1,n1,&g_tehdr.g_lh) {
		i++;
		tinfo = list_entry(pos1, struct time_info, tm_list);
		fpga_info(("tm block:%d  tm:0x%x action_cnt:0x%x ",i, tinfo->tm, tinfo->action_cnt));
		list_for_each_safe(pos2,n2, &tinfo->ta_list) {
			j++;
			fai = list_entry(pos2, struct to_dev_ai, action_list );
			fpga_info(("action block:%d	fpga msgtype:0x%x fpgadev:0x%x fpgadev_no:0x%x",j, fai->msgtype,fai->dev,fai->dev_no));				

		}
	}
	printk("\n####################################:\n\n");
#endif
		
}





/*********************************timing parser according to the protocol******************************************/


static char *ts_buf;
static int ts_len;
struct timing_exec_hdr *g_func_tshdr;
int g_func_num;


int ts_data_read(char *tdf)
{
	struct file *filep = NULL;
	struct kstat stat;
	mm_segment_t fs;
	char *raw_fmts;
	int tdata_size = 0;
	int error;
	unsigned int tm_cnt;
	int *tmp;
	int i;
	
	
	set_fs(KERNEL_DS);
	fs = get_fs();
	filep = filp_open(tdf, O_RDONLY, 0);
	if (IS_ERR(filep)) {
		fpga_error(("Failed to open the timing file:%s  in %s", tdf, __FUNCTION__));
		goto fail;
	}
	error = vfs_stat(tdf, &stat);
	if (error) {
		fpga_error(("Failed in %s to find file stat", __FUNCTION__));
		goto fail;
	}
	tdata_size = (int) stat.size;
	ts_len = tdata_size;

	printk("tdata_size:%d",ts_len);

	ts_buf = kzalloc(tdata_size, GFP_KERNEL);
	if (ts_buf == NULL) {
		fpga_error(("Failed to allocate raw_fmts memory"));
		goto fail;
	}
	if (vfs_read(filep, ts_buf, tdata_size, &filep->f_pos) !=	tdata_size) {
		fpga_error(("Error: raw timing file read failed"));
		goto fail;
	}


#ifdef DEBUG_FPGA	
//	for(i=0;i<tdata_size/sizeof(unsigned int);i++){	
	for(i=0;i<ts_len;i++){
		printk("0x%x ",ts_buf[i]);
	}
	printk("\n");
#endif	

	return 0;
fail:
	if (ts_buf) {
		kfree(ts_buf);
		ts_buf = NULL;
	}
	if (!IS_ERR(filep))
		filp_close(filep, NULL);
	set_fs(fs);
	return -1;

}




int decode_ver_ctx(struct ts_contex * ts_ctx, int ts_ctx_len)
{
	
	fpga_info(("decode_ver_ctx "));
}

int decode_ctime_ctx(struct ts_contex * ts_ctx, int ts_ctx_len)
{
	 fpga_info(("decode_ctime_ctx"));
}


int deserialize_comm_item(struct comm_item *ts_item, unsigned char *pbuf, int max_size,uint32_t *outlen)
{
	int size = 0;
	int len,i;

	unsigned char *work = pbuf;

    	memcpy(&ts_item->m_item_type, work, sizeof(ts_item->m_item_type));
    	size += sizeof(ts_item->m_item_type);
    	work += sizeof(ts_item->m_item_type);
 
    	memcpy(&ts_item->m_len, work, sizeof(ts_item->m_len));
    	size += sizeof(ts_item->m_len);
    	work += sizeof(ts_item->m_len);

    	if (ts_item->m_len < 0 || ts_item->m_len > 100 * 1024){
		return -1;
    	}

	ts_item->pcomm_value = (unsigned char *)kzalloc(ts_item->m_len,GFP_KERNEL);
	memcpy(ts_item->pcomm_value,work,ts_item->m_len);
	*outlen =ts_item->m_len;
	size += ts_item->m_len;

	return size;

}

int deserialize_func_ctx(struct func_contex *func_ctx, unsigned char *context_str,int in_func_item_len,int *out_func_ctx_len)
{
	int size =0;

	unsigned char *work=context_str;
	int len = in_func_item_len;
    	int value_len;

    	memcpy(&func_ctx->m_func_id,work,sizeof(func_ctx->m_func_id));
    	size += sizeof(func_ctx->m_func_id);
    	work += sizeof(func_ctx->m_func_id);

    	memcpy(&func_ctx->m_func_finish_time,work,sizeof(func_ctx->m_func_finish_time));
    	size += sizeof(func_ctx->m_func_finish_time);
    	work += sizeof(func_ctx->m_func_finish_time);

    	memcpy(&func_ctx->m_num_timenode,work,sizeof(func_ctx->m_num_timenode));
    	size += sizeof(func_ctx->m_num_timenode);
    	work += sizeof(func_ctx->m_num_timenode);

    	value_len = len - size;

    	func_ctx->pfc_value = kzalloc(value_len+1,GFP_KERNEL);
    	memcpy(func_ctx->pfc_value,work,value_len);
    	//func_ctx->pfc_value[value_len]='\0';

    	*out_func_ctx_len = value_len;

   	return len;
}

int deserialize_tm_node_ctx(struct time_node_contex *tm_nd_ctx, unsigned char *context_str,int ts_tm_node_len, int *tm_node_ctx_len)
{
	int size = 0;
    	int value_len;

	unsigned char *work = context_str;
	int len = ts_tm_node_len;

    	memcpy(&tm_nd_ctx->m_time, work, sizeof(tm_nd_ctx->m_time));
    	size += sizeof(tm_nd_ctx->m_time);
    	work += sizeof(tm_nd_ctx->m_time);

    	memcpy(&tm_nd_ctx->m_num_command, work, sizeof(tm_nd_ctx->m_num_command));
    	size += sizeof(tm_nd_ctx->m_num_command);
    	work += sizeof(tm_nd_ctx->m_num_command);

    	if(len < size)
       	 	return -1;

    	value_len = len - size;
    	tm_nd_ctx->tnc_value = kzalloc(value_len+1,GFP_KERNEL);
    	memcpy(tm_nd_ctx->tnc_value,work,value_len);


    	*tm_node_ctx_len = value_len;

    	return len;
}

int decode_ctrl_cmd_ctx(struct ctrl_cmd_contex *ctrl_cmd_ctx,int len,struct time_info *tmp_ti)
{
    	struct cmd_contex *cmd_ctx;

	cmd_ctx=(struct cmd_contex *)kzalloc(sizeof(struct cmd_contex), GFP_KERNEL);				

	INIT_LIST_HEAD(&cmd_ctx->cmd_list);

	memcpy((char *)cmd_ctx+sizeof(struct list_head),(char *)ctrl_cmd_ctx->cmd_value,len);

	list_add_tail(&cmd_ctx->cmd_list, &tmp_ti->ta_list);
	
    	return 0;
}

int decode_ctrl_cmd_item(struct comm_item *ctrl_item,int ctrl_cmd_len,struct time_info *tmp_ti)
{
    	struct ctrl_cmd_contex *ctrl_cmd_ctx=kzalloc(sizeof(struct ctrl_cmd_contex),GFP_KERNEL);

    	int len = ctrl_cmd_len;

    	ctrl_cmd_ctx->cmd_value = kzalloc(ctrl_cmd_len+1,GFP_KERNEL);
   	memcpy( ctrl_cmd_ctx->cmd_value,ctrl_item->pcomm_value,len);

    	decode_ctrl_cmd_ctx(ctrl_cmd_ctx,ctrl_cmd_len,tmp_ti);
}

int decode_tm_node_ctx(struct time_node_contex *tm_nd_ctx,struct time_info *tmp_ti,int tm_nd_ctx_len)
{
	struct comm_item *ctrl_cmd;
	int decode_size = 0,i;
	int ctrl_cmd_len;

	unsigned char *work = tm_nd_ctx->tnc_value;

	int remain_size = tm_nd_ctx_len;
	int cmd_num = tm_nd_ctx->m_num_command;

    	for(i = 0; i < cmd_num; i++) {
        		ctrl_cmd =(struct comm_item *)kzalloc(sizeof(*ctrl_cmd),GFP_KERNEL);
		
        		decode_size = deserialize_comm_item(ctrl_cmd, work, remain_size,&ctrl_cmd_len);
		decode_ctrl_cmd_item(ctrl_cmd,ctrl_cmd_len,tmp_ti);

        		work = work + decode_size;
        		remain_size = remain_size - decode_size;
	}

}

int decode_tm_node_item(struct comm_item *ts_timenode, struct time_info *tmp_ti, int ts_tm_node_len)
{
	struct time_node_contex *tm_nd_ctx= (struct time_node_contex *)kzalloc(sizeof(*tm_nd_ctx),GFP_KERNEL);
    	int tm_nd_ctx_len;

	deserialize_tm_node_ctx(tm_nd_ctx, ts_timenode->pcomm_value,ts_tm_node_len,&tm_nd_ctx_len);

	tmp_ti->tm = tm_nd_ctx->m_time;
	tmp_ti->action_cnt = tm_nd_ctx->m_num_command;

	decode_tm_node_ctx(tm_nd_ctx, tmp_ti,tm_nd_ctx_len);

}

int 	decode_func_item(struct comm_item*func_item, struct timing_exec_hdr *p_func_tsh, int func_item_len)
{
	int i;
	int decode_size = 0;
	int remain_size = 0;
	unsigned char *work;
	int out_func_ctx_len;
	int tm_num;
	int ts_tm_node_len;
	struct func_contex *func_ctx;
	struct comm_item *ts_timenode;
	struct time_info  *tmp_ti;
	struct timing_exec_hdr pfunc_ts;

	INIT_LIST_HEAD(&p_func_tsh->g_lh);

	func_ctx = (struct func_contex *)kzalloc(sizeof(*func_ctx),GFP_KERNEL);

   	deserialize_func_ctx(func_ctx,func_item->pcomm_value,func_item_len,&out_func_ctx_len);

	//every func_ts contain :
	p_func_tsh->tm_cnt = func_ctx->m_num_timenode;
	p_func_tsh->func_id = func_ctx->m_func_id;
	p_func_tsh->m_func_finish_time = func_ctx->m_func_finish_time;

	work = func_ctx->pfc_value;

	remain_size= out_func_ctx_len;

	tm_num = func_ctx->m_num_timenode;

	for(i = 0; i < tm_num; i++){
        		ts_timenode = (struct comm_item *)kzalloc(sizeof(*ts_timenode),GFP_KERNEL);

		tmp_ti = (struct time_info *)kzalloc(sizeof(struct time_info),GFP_KERNEL);
		INIT_LIST_HEAD(&tmp_ti->ta_list);

		list_add_tail(&tmp_ti->tm_list, &p_func_tsh->g_lh);

		decode_size = deserialize_comm_item(ts_timenode,work,remain_size,&ts_tm_node_len);
		decode_tm_node_item(ts_timenode,tmp_ti,ts_tm_node_len);

		work = work + decode_size;
		remain_size = remain_size - decode_size;

	}
	return 0;

}



int decode_func_all_ctx(struct ts_contex * ts_ctx,int ts_ctx_len)
{
	int i = 0;
	unsigned char *work = ts_ctx->pts_value;
	struct comm_item*func_item;

	int remain_size = ts_ctx_len;
	int decode_size = 0;
	int func_num = ts_ctx->m_num;
    	int func_item_len;

	g_func_tshdr = (struct timing_exec_hdr *)kzalloc(sizeof(*g_func_tshdr)*func_num,GFP_KERNEL);
	g_func_num= func_num;


	for(i = 0; i <func_num; i ++) {
		func_item = (struct comm_item *)kzalloc(sizeof(struct comm_item),GFP_KERNEL);
		decode_size = deserialize_comm_item(func_item,work,remain_size,&func_item_len);
		decode_func_item(func_item,&g_func_tshdr[i],func_item_len);

		work = work + decode_size;
		remain_size = remain_size - decode_size;
	}

	return 0;
}


int deserializer_ts_ctx(struct ts_contex *ts_ctx, unsigned char *contex_str, int in_tsitem_len, int *out_ts_ctx_len)
{
	int size = 0;
	int i;
    	int value_len;
	unsigned char *work=contex_str;
		
    	memcpy(&ts_ctx->m_num,work,sizeof(ts_ctx->m_num));
    	work += sizeof(ts_ctx->m_num);
    	size += sizeof(ts_ctx->m_num);

    	value_len = in_tsitem_len - size;

    	ts_ctx->pts_value = kzalloc(value_len+1,GFP_KERNEL);
    	memcpy(ts_ctx->pts_value,contex_str+size,value_len);

    	*out_ts_ctx_len = value_len;

}


void decode_ts_attr(struct comm_item *ts_item,int tsitem_len)
{
	struct ts_contex *ts_ctx =(struct ts_contex *)kzalloc(sizeof(*ts_ctx),GFP_KERNEL);
	int type = ts_item->m_item_type;
    	int ts_ctx_len;
	int i;
	

	deserializer_ts_ctx(ts_ctx, ts_item->pcomm_value,tsitem_len,&ts_ctx_len);
	
	switch(type){
		case TS_TYPE_VERSION:
			decode_ver_ctx(ts_ctx,ts_ctx_len);
			break;

		case TS_TYPE_COMPILE_TIME:
			decode_ctime_ctx(ts_ctx,ts_ctx_len);
			break;
		case TS_TYPE_FUNC:
			decode_func_all_ctx(ts_ctx,ts_ctx_len);
			break;
	}

}

void decode_ts(char *ts_buf, int size)
{
	struct comm_item *ts_item;
	int decode_size = 0;
	unsigned char *work = ts_buf;
	int remain_size = size;
    	int ts_item_len;
	int i;

	ts_item = (struct comm_item *)kzalloc(sizeof(*ts_item),GFP_KERNEL);

	while(remain_size > 0 ) {
		decode_size = 	deserialize_comm_item(ts_item, work,remain_size,&ts_item_len);

		decode_ts_attr(ts_item,ts_item_len);
		work += decode_size;
		remain_size = remain_size - decode_size;
	}
}


//read  ts data from raw data file
void ts_parser_data_from_file(void)
{
	ts_data_read("/home/encode_file");
	decode_ts(ts_buf, ts_len);

}

 //read data  from netlink msg
void ts_parser_data(char *tsbuf, int tslen)
{
	decode_ts(tsbuf, tslen);

}



void dump_func_ts(void)
{
	int i=0,j=0,cnt=0;
	struct list_head *pos1, *n1, *pos2, *n2;
	struct time_info  *tminfo;
	struct cmd_contex  *cmdinfo;
	int diff_time, old_tstime=0 ;

	fpga_info(("#####################dump func ts data, func ts number:%d############################\n",g_func_num));

	for(cnt = 0; cnt < g_func_num; cnt++) {
		i = 0;
		old_tstime = 0;
		
		fpga_info(("\n\n\n       func_ts id:0x%x  total tm block:%d\n", g_func_tshdr[cnt].func_id,g_func_tshdr[cnt].tm_cnt));
		list_for_each_safe(pos1,n1,&g_func_tshdr[cnt].g_lh) {
				i++;
				j=0;
				tminfo = list_entry(pos1, struct time_info, tm_list);

				diff_time =  tminfo->tm - old_tstime;		
				old_tstime = tminfo->tm;
					
				fpga_info(("\n                    tm block:%d ---> m_time:(%d ms ) cmd_cnt:%d ",i, diff_time,tminfo->action_cnt));
				list_for_each_safe(pos2,n2, &tminfo->ta_list) {
					j++;
					cmdinfo = list_entry(pos2, struct cmd_contex, cmd_list );
					fpga_info(("\n                    cmd:%d --->  para:m_dev_type:%d   m_dev_num:%d , m_action:%d op1:%d,  op2:%d,  op3:%d op4:%d ",j, cmdinfo->m_dev_type, cmdinfo->m_dev_num,cmdinfo->m_action, cmdinfo->m_optional_1, cmdinfo->m_optional_2, cmdinfo->m_optional_3, cmdinfo->m_optional_4)); 			
		
				}
		}
	}
	fpga_info(("\n#############################dump func ts data end####################################:\n\n"));

}			



/*
* Netlink event protocol struct;
*/

int deserialize_tlv(struct tlv_data *tlvdata, char *rawdata, int *value_len)
{
	int size;
	char *work = rawdata;

	memcpy(&tlvdata->m_type, work, sizeof(tlvdata->m_type));
	size += sizeof(tlvdata->m_type);
	work += sizeof(tlvdata->m_type);
	memcpy(&tlvdata->m_len, work, sizeof(tlvdata->m_len));
	size += sizeof(tlvdata->m_len);
	work += sizeof(tlvdata->m_len);

	if(tlvdata->m_len < 0 | tlvdata->m_len > 100*1024){
		fpga_error(("deserialize_tlv m_len invalid\n"));
		return -1;
	}

	tlvdata->m_value = work;
		
	*value_len = tlvdata->m_len;
	size += tlvdata->m_len;

	return size;
	
}


//collect_data point to TL_TL_value;
int deserialize_collect_data(struct signal_collect_data *collect_data, char *raw_collect_data, int raw_collect_datalen )
{
	int size = 0;
	int raw_data_len;
	
	char *p_work = NULL;


	size = sizeof(collect_data->m_info);
	memcpy(&collect_data->m_info, raw_collect_data, size);
	raw_data_len = raw_collect_datalen-size;

	collect_data->m_raw_value = kzalloc(raw_data_len, GFP_KERNEL);
	memcpy(&collect_data->m_raw_value, raw_collect_data+size, raw_data_len);
	
	return raw_collect_datalen;

}




/*
*  type: the first tlv's type param;  etc. MSG_TYPE_TS_REPORT, MSG_DATA_REPORT, MSG_TYPE_OTHER_REPORT,
*  subtype: the second type param if have;
*/
#define TLV_HEADER_LEN (sizeof(uint32_t) + sizeof(uint32_t))

//buf: only point to tlv data(measure_related_info )
void encode_tlv_netlink_send_ts_rpt(uint32_t  type, uint32_t subtype, struct measure_related_info * pmir, uint32_t mir_len)
{
	struct tlv_data *tlvdata ;
	unsigned char *tmpbuf = (unsigned char *)kzalloc( 2*TLV_HEADER_LEN +mir_len, GFP_KERNEL);

	struct tlv_data *sub_tlvdata;
	struct measure_related_info *mri = pmir;
	int i;

	tlvdata = (struct tlv_data *)tmpbuf;
	tlvdata->m_type = type;
	tlvdata->m_len = mir_len + TLV_HEADER_LEN;
	tlvdata->m_value = tmpbuf+TLV_HEADER_LEN;
	
	sub_tlvdata = (struct tlv_data *)(tlvdata->m_value);
	sub_tlvdata->m_type = subtype;
	sub_tlvdata->m_len = mir_len;
	

	memcpy((unsigned char *)(tmpbuf+2*TLV_HEADER_LEN), (unsigned char *)mri, mir_len);

	//fpga_info(("tlvdata:(type:%d ,len:%d, subtype:%d, sublen:%d\n",tlvdata->m_type,tlvdata->m_len,sub_tlvdata->m_type,sub_tlvdata->m_len));
	
	fpga_netlink_send((char *)tlvdata, tlvdata->m_len +TLV_HEADER_LEN , netlink_port);	

}

//buf: measure_related_info , len: report data len, etc, hgb data len, wbc data len;
// TL--TL--measure_related_info + fifo_data
void encode_tlv_netlink_send_data_rpt(uint32_t  type, uint32_t subtype, struct measure_related_info * pmir, char *data_buf, uint32_t data_len)
{
	unsigned char *tmpbuf;

	spin_lock_bh(&g_fpga_dev->lock);

	tmpbuf = (unsigned char *)kzalloc( 2*TLV_HEADER_LEN +sizeof(struct measure_related_info) +data_len , GFP_KERNEL);

	struct tlv_data *tlvdata;

	struct tlv_data *sub_tlvdata;
	struct signal_collect_data*scd; 

	tlvdata = (struct tlv_data *)tmpbuf;
	tlvdata->m_type = type;
	tlvdata->m_len = TLV_HEADER_LEN + sizeof(struct measure_related_info) + data_len;
	tlvdata->m_value = tmpbuf+TLV_HEADER_LEN;

	sub_tlvdata = (struct tlv_data *)tlvdata->m_value;
	sub_tlvdata->m_type = subtype;
	sub_tlvdata->m_len = sizeof(struct measure_related_info) + data_len;
	sub_tlvdata->m_value = tmpbuf + 2*TLV_HEADER_LEN;

	
	scd = (struct signal_collect_data*)(sub_tlvdata->m_value);
	memcpy(&scd->m_info, pmir, sizeof(struct measure_related_info));

	scd->m_raw_value = tmpbuf + 2*TLV_HEADER_LEN + sizeof(struct measure_related_info);
	
	memcpy(scd->m_raw_value, data_buf, data_len);
	fpga_netlink_send((char *)tmpbuf, tlvdata->m_len + TLV_HEADER_LEN, netlink_port);	
	spin_unlock_bh(&g_fpga_dev->lock);

}



//send other data :temp , press, motor, ....
void encode_tlv_netlink_send_other_rpt(uint32_t  type, uint32_t subtype,  uint32_t third_type, struct other_data_info *other_di)
{
	struct tlv_data *tlvdata ;
	unsigned char *tmpbuf ;

	spin_lock_bh(&g_fpga_dev->lock);

	tmpbuf = (unsigned char *)kzalloc( 2*TLV_HEADER_LEN + sizeof(struct other_data_info), GFP_KERNEL);

	struct tlv_data *sub_tlvdata;
	struct range_info *pr_info;
	struct motor_info *pm_info;
	struct sensor_info *ps_info;
	int i, data_len;

	pr_info = &other_di->r_info;
	pm_info = &other_di->m_info;
	ps_info = &other_di->s_info;

	
	if(third_type == OTHERREPORT_THIRD_TYPE_RANGE_INFO)
		data_len = sizeof(*pr_info);
	else if(third_type == OTHERREPORT_THIRD_TYPE_MOTOR_INFO)
		data_len = sizeof(*pm_info);
	else if(third_type == OTHERREPORT_THIRD_TYPE_SENSOR_INFO)
		data_len = sizeof(*ps_info);

	tlvdata = (struct tlv_data *)tmpbuf;
	tlvdata->m_type = type;
	tlvdata->m_len = data_len + TLV_HEADER_LEN;
	tlvdata->m_value = tmpbuf+TLV_HEADER_LEN;
	
	sub_tlvdata = (struct tlv_data *)(tlvdata->m_value);
	sub_tlvdata->m_type = subtype;
	sub_tlvdata->m_len = data_len;
	

	memcpy((unsigned char *)(tmpbuf+2*TLV_HEADER_LEN), (unsigned char *)other_di, data_len);

	//fpga_info(("tlvdata:(type:%d ,len:%d, subtype:%d, sublen:%d\n",tlvdata->m_type,tlvdata->m_len,sub_tlvdata->m_type,sub_tlvdata->m_len));
	
	fpga_netlink_send((char *)tlvdata, tlvdata->m_len +TLV_HEADER_LEN , netlink_port);	
	spin_unlock_bh(&g_fpga_dev->lock);

}

/********************************************************************************/
