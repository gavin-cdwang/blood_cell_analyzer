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


#ifndef __DRIVERS_FPGA_TIMINTG_PARSER_H
#define __DRIVERS_FPGA_TIMINTG_PARSER_H
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

#include "fpga_timing_parser.h"

//char *g_raw_fmts =  NULL;


/*  these api is used to parser timeing sequence form raw data file,
*    and orgainze as below: 2 dim link list
*
*   g_node
*         |
*   |time1|--->action1---->action2--->action8--->action12
*         |
*   |time2|--->action4---->action7
*         |
*   |time4|--->action1---->action2--->action18

*   |time11|--->action6
*	 |	
*   |time7|--->action31--->action9
*/



enum msg_type
{
	MSG_TO_FPGA,
	MSG_TO_USERSPACE,		
	MSG_TO_SERIAL,
	MSG_TO_NET,
};

struct time_info
{
	struct list_head tm_list;
	struct list_head ta_list;
	unsigned int tm;    // time to exec action;
	unsigned int action_cnt;
	struct measure_related_info * pts_mri;
};


struct to_userspace_ai
{
	struct list_head user_list;

	unsigned int ts_key;  //indicate the current unique timing seq key;
	unsigned int subtype; //indicate  how to exec the action;
	char *action;    //do the action through to usespace,need discuss detial;
};


/*
*   fpga struct action info desc:
*   common data: dev, no,time_out,  note: exec time store in tm_info struct,time_out=0 means no end_time params;
*   action_type=0(default): control cmd;      action_type=1: INIT or Sample cmd
*
*
*   valve:action(dev,no,time_out, P:enable/disable) : time_out=0 action_p1
*   pump:action(dev,no,time_out, P:enable/disable) :time_out=0 action_p1
*   injector:action(dev,no,time_out, P:speed ,step)   :action_p1, action_p2
*   motor_move:action(dev,no,time_out, P:speed, pos): action_p1, action_p2
*   hgb:action(dev,no,time_out): sample data; no param or action_p1 for sample count data; action_type=1
*   diff:action(dev,no,time_out); sample data; no param or action_p1 for sample count data; action_type=1 
*   wbc:action(dev,no,time_out): sample data;no param; action_type=1 
*   rbc:action(dev,no,time_out): sample data;no param; action_type=1 
*   laser:action(dev,no,time_out, P:enable/disable): time_out=0 action_p1 
*   cnst_control(dev,no,time_out,P: enable/disable):time_out=0  action_p1
*   burn(dev,no,time_out,P: enable/disable): time_out=0 action_p1
*   burn_control(dev,no,time_out,P: enable/disable): time_out=0 action_p1
*   hgb_led(dev,no,time_out,P: enable/disable): time_out=0 action_p1
*   pool_mix(dev,no,time_out,P:away_step,close_step,speed,wait_time,mix_times): action_p1,action_p2,action_p3,action_p4,action_p5
*   set_reg(dev,no,time_out,P:regval):time_out=0 action_p1
*   vac_set(dev,no,time_out,P:mix,max); action_p1,action_p2
*  vac_detc(dev,no,time_out,P:mix, max); time_out=0  action_p1,action_p2 , action_type=1 indicate sample 
*  sampler(dev,no,time_out,P:pos); action_p1
*  press_detc(dev,no,time_out,P:mix, max); time_out=0  action_p1, action_p2, action_type=1 indicate sample ;
*  level_pool(dev,no,time_out,P: expect); time_out=0  action_p1, action_type=1 indicate sample ;
*  air_bubble_detc(dev,no,time_out,P: action); time_out=0 action_p1 , action_type=1  indicate sample ;
*  
*
*/
struct to_dev_ai   //(to fpga , serial, net, common struct )
{
	struct list_head action_list;
	
	enum msg_type msgtype;  		//indicate the action send to (0:fpga, 1: userspace, 2:serial, 3:net ...)
	//unsigned int ts_key;  //indicate the current unique timing seq key;
	//action info param , used for how to exec action;
	int action_type;  //0:control, 1:data sample 2:mon

	//common action info:
	unsigned int dev;	 // which dev(valve, pump, motor,wbc, hgb...)  to exec action;
	unsigned int dev_no; //which number dev to exec ;
	unsigned int timeout;  //exec to timeout; 

	int action_p1;  //how to exec action param1, E.g : valve(0,1 :open or close)  
	int action_p2;  //how to exec action param2 E.g : injector(speed, step)  
	int action_p3;  //how to exec action param3 E.g : 
	int action_p4;  //how to exec action param4 E.g : 

	int action_p5;  //how to exec action param5 E.g : pool_mix(away_step,close_step,speed,wait_time,mix_times)  

};

struct to_serial_ai
{
	struct list_head serial_list;
	enum msg_type msgtype;  		//indicate the action send to (0:fpga, 1: userspace, 2:serial, 3:net ...)

	unsigned int ts_key;  //indicate the current unique timing seq key;
	char *action;    //do the action through to serial ,need discuss detial;


};

struct to_net_ai
{
	struct list_head net_list;

	unsigned int ts_key;  //indicate the current unique timing seq key;
	char *action;    //do the action through to net ,need discuss detial;
};


struct timing_action_info
{
	//unsigned int ts_key;  //indicate the current unique timing seq key;

	//struct time_info tm_info;   //indicate the time to exec the cmd;
	enum msg_type msgtype;  		//indicate the action send to (0:fpga, 1: userspace, 2:serial, 3:net ...)
	int action_type;  //0:control, 1:data sample 2:mon

	//common action info:
	unsigned int dev;	 // which dev(valve, pump, motor,wbc, hgb...)  to exec action;
	unsigned int dev_no; //which number dev to exec ;
	unsigned int timeout;  //exec to timeout; 

	int action_p1;	//how to exec action param1, E.g : valve(0,1 :open or close)  
	int action_p2;	//how to exec action param2 E.g : injector(speed, step)  
	int action_p3;	//how to exec action param3 E.g : 
	int action_p4;	//how to exec action param4 E.g : 

	int action_p5;	//how to exec action param5 E.g : pool_mix(away_step,close_step,speed,wait_time,mix_times)	


#if 0	
	union {
		struct to_fpga_ai  ai_fpga;
		struct to_userspace_ai ai_user;		
		struct to_serial_ai ai_serial;
		struct to_net_ai ai_net;
	};

#endif
};


struct time_info_user
{
	unsigned int tm;    // time to exec action;
	unsigned int action_cnt;
};



//used for time_info link list header 
struct timing_exec_hdr
{
	struct list_head g_lh;
	unsigned int tm_cnt; //time seq count;	
	unsigned int func_id;
	unsigned int m_func_finish_time;
	char *tsi;
};

#define TIMING_SEQ_MAGIC 0x11223344

struct timing_seq_hdr
{
	unsigned int tm_magic;
	unsigned int tm_cnt; //time seq count;
	//unsigned int ts_key;  //indicate the current unique timing seq key
};



enum commonItemtype

{
    TS_TYPE_VERSION         = 1,
    TS_TYPE_COMPILE_TIME    = 2,
    TS_TYPE_FUNC            = 3,
    FUNC_TYPE_CONTEXT       = 10,
    TIME_NODE_TYPE_CONTEXT  = 100,
    CTRL_CMD_TYPE_CONTEXT   = 1000,
};



struct comm_item
{
    uint32_t    m_item_type;          // type
    uint32_t    m_len;                // len
    char *    pcomm_value;              // value，其值为下面的一种,TSContext,FuncContext,TimeNodeContext,CtrlCommandContext
};

struct ts_contex
{
    uint32_t    m_num;                // 有多少个func ts
    char *  pts_value;              // 二进制原始数据
};

struct func_contex
{
    uint32_t    m_func_id;            // 时序id字段
    uint32_t    m_func_finish_time;   // func ts运行总时间，单位:ms
    uint32_t    m_num_timenode;       // 该func ts中包含多少个TimeNode
    char *pfc_value;              // 二进制原始数据
};


struct time_node_contex
{
    uint32_t m_time;                  // 间隔时间，单位：ms
    uint32_t m_num_command;           // 该time node中具体有多少个命令
    char *tnc_value;              // 二进制原始数据
};



struct ctrl_cmd_contex
{
    char *cmd_value;             //command contex :24byte;
};

struct cmd_contex
{
    struct list_head cmd_list;
    unsigned char m_dev_type;
    unsigned char m_dev_num;
    unsigned char m_action;
    unsigned int m_timeout;
    int m_optional_1;
    int m_optional_2; 
    int m_optional_3;
    int m_optional_4;
    struct measure_related_info * pcmd_mri;
};


int timing_data_read(char *tdf);
int timing_seq_action_parser(void);
void timing_parser_init(void);


void ts_parser_data_from_file(void);
void ts_parser_data(char *tsbuf, int tslen);


void dump_func_ts(void);


/*
* Netlink event protocol struct;
*/

//inter message in ICS<->TS_EXE
enum message_type{
	MSG_TYPE_TS_INIT = 1,                   //  ICS->TS_EXE
	MSG_TYPE_TS_CTRL,                       // ICS->TS_EXE
	MSG_TYPE_TS_REPORT,                     // TS_EXE->ICS
	MSG_DATA_REPORT,                        // TS_EXE->ICS
	MSG_TYPE_OTHER_CTRL,                    // ICS->TS_EXE
	MSG_TYPE_OTHER_REPORT,                  //TS_EXE->ICS
};

/*
 * TS ctrl type:
 */
enum message_ts_ctrl_type{
	SUBTYPE_TS_CTRL_MEASURE_START = 100,
	SUBTYPE_TS_CTRL_MEASURE_STOP,
	SUBTYPE_TS_CTRL_MEASURE_FORCE_STOP,
};

/*
 * TS report
 */
enum message_ts_report_type{
	SUBTYPE_TS_REPORT_START = 200,
	SUBTYPE_TS_REPORT_END,
	SUBTYPE_TS_REPORT_TRIGGER,
	SUBTYPE_TS_REPORT_ASP_COMPLETE,
    	SUBTYPE_TS_REPORT_UNLIMITED,
    	SUBTYPE_TS_REPORT_MODE_LIMIT_START,
    	SUBTYPE_TS_REPORT_MODE_LIMIT_STOP,
};

/*
 * TS data report
 */
enum message_data_report_type{
    	SUBTYPE_DATA_REPORT_HGB = 300,
    	SUBTYPE_DATA_REPORT_RBC,
    	SUBTYPE_DATA_REPORT_WBC,
    	SUBTYPE_DATA_REPORT_DIFF,
};

/*
 * other ctrl type ICS->TS_EXE
 */
enum message_other_ctrl_type
{   
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
};

/*
 * other data report TS_EXE->ICS
 */
enum message_other_report_type
{
    SUBTYPE_OTHERREPORT_INFO_SAMPLER = 2000,

    SUBTYPE_OTHERREPORT_INFO_VALVE,
    SUBTYPE_OTHERREPORT_INFO_PUMP,
    SUBTYPE_OTHERREPORT_INFO_TEMP,
    SUBTYPE_OTHERREPORT_INFO_PRESS,
    SUBTYPE_OTHERREPORT_INFO_MOTOR,
    SUBTYPE_OTHERREPORT_INFO_INJECT,
    SUBTYPE_OTHERREPORT_INFO_TS_NAME_LIST,

    SUBTYPE_OTHERREPORT_INFO_SENSOR,
    SUBTYPE_OTHERREPORT_INFO_ALARM,
    SUBTYPE_OTHERREPORT_INFO_TSEXE_LOG,
};



#define OTHERREPORT_THIRD_TYPE_RANGE_INFO 1 //temp, press
#define OTHERREPORT_THIRD_TYPE_MOTOR_INFO 2
#define OTHERREPORT_THIRD_TYPE_SENSOR_INFO 3  //valve, pump...



/*
 * TLV
 */
struct tlv_data
{
    	uint32_t    m_type;
    	uint32_t    m_len;
    	char 	*m_value;
};

/*
 * 
 */
struct measure_related_info{
    	uint32_t m_unique_key;              //key
    	uint32_t m_ts_id;                   // timing id

    	uint32_t m_sample_rack_id;          //sample rack id
    	uint32_t m_sample_postion;          //sample pos
    	uint32_t m_sample_asp_dosage;       // 
    	uint32_t m_r1_postion;              //
    	uint32_t m_r1_asp_dosage;           // 
    	uint32_t m_r2_postion;              // 
    	uint32_t m_r2_asp_dosage;           // 
    	uint32_t m_spectrum_bitmap;         // 
};

/*
 * 
 */
struct signal_collect_data{
    	struct measure_related_info    m_info;
    	char *m_raw_value;
};




struct range_info
{
    uint32_t m_id;                
    int32_t  m_min;                    
    int32_t  m_max;                  
    int32_t  m_target;                 
    int32_t  m_actual;                 
 
};


struct motor_info
{
    uint32_t m_id;                     
    uint32_t m_type;                   
    int32_t  m_speed;                  
    int32_t  m_steps;                  
    uint32_t m_pos_current;             
    uint32_t m_pos_target;              
    uint32_t m_execute_time;           

};


struct sensor_info
{
    uint32_t m_bitmap[4];             
    uint32_t m_mask[4];                
};


struct  other_data_info {
	union  {
		struct range_info r_info;
		struct motor_info m_info;
		struct sensor_info s_info;
	}
};


int deserialize_tlv(struct tlv_data *tlvdata, char *rawdata, int *value_len);
int deserialize_collect_data(struct signal_collect_data *collect_data, char *raw_collect_data, int raw_collect_datalen );
void encode_tlv_netlink_send_ts_rpt(uint32_t  type, uint32_t subtype, struct measure_related_info * pmir, uint32_t mir_len);
void encode_tlv_netlink_send_data_rpt(uint32_t	type, uint32_t subtype, struct measure_related_info * pmir, char *data_buf, uint32_t data_len);
void encode_tlv_netlink_send_other_rpt(uint32_t  type, uint32_t subtype,  uint32_t third_type, struct other_data_info *other_di);
/*****************************************************************************/


#endif
