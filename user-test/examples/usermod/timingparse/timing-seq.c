#include <stdlib.h>
#include <stdio.h>
#include <string.h>

struct timing_seq_hdr
{
	unsigned int tm_magic;
	unsigned int tm_cnt;
}tsh={0x11223344,1};

struct time_info
{
	unsigned int tm;
	unsigned int action_cnt;
};


enum msg_type
{
	MSG_TO_FPGA,
	MSG_TO_USERSPACE,		
	MSG_TO_SERIAL,
	MSG_TO_NET,
};

struct timing_action_info
{
	//unsigned int ts_key;  //indicate the current unique timing seq key;

	//struct time_info tm_info;   //indicate the time to exec the cmd;
	enum msg_type msgtype;  		//indicate the action send to (0:fpga, 1: userspace, 2:serial, 3:net ...)
//action info param , used for how to exec action;
	
	int action_type; //0:control, 1:data sample 2:mon	

	unsigned int devtype;	 // which dev(valve, pump, motor,wbc, hgb...)  to exec action;
	
	unsigned int dev_no; //which number dev to exec ;
	unsigned int timeout;  //exec to timeout; 

	
	int action_p1;	//how to exec action param1, E.g : valve(0,1 :open or close)  
	int action_p2;	//how to exec action param2 E.g : injector(speed, step)  
	int action_p3;	//how to exec action param3 E.g : 
	int action_p4;	//how to exec action param4 E.g : 

	int action_p5;	//how to exec action param5 E.g : pool_mix(away_step,close_step,speed,wait_time,mix_times)	
};


void main()
{
	FILE *fd;
	struct time_info t1={2,10},t2={4,4},t3={3,2};
	struct timing_action_info t1_ai1={0,0,10,3,0,-50000,-42000,0,0,0};
	struct timing_action_info t1_ai2={0,0,3,1,0,1,0,0,0,0};
	struct timing_action_info t1_ai3={0,0,6,2,0,3,0,0,0,0};
	struct timing_action_info t1_ai4={0,0,11,3,0,40,60,0,0,0};
	struct timing_action_info t1_ai5={0,0,14,5,0,1,0,0,0,0};
	struct timing_action_info t1_ai6={0,0,4,1,0,25,40,0,0,0};
	struct timing_action_info t1_ai7={0,0,15,1,4000,7,40,0,0,0};
	struct timing_action_info t1_ai8={0,0,12,9,0,5,40,0,0,0};
	struct timing_action_info t1_ai9={0,0,19,3,7000,5,40,0,0,0};
	struct timing_action_info t1_ai10={0,0,21,1,5,5,40,0,0,0};

	struct timing_action_info t2_ai1={0,3,2,0,0,2,0,0,0,0};
	struct timing_action_info t2_ai2={1,2,5,0,1,3,4,7,0,0};
	struct timing_action_info t2_ai3={0,1,8,5,0,9,0,0,9,0};
	struct timing_action_info t2_ai4={4,2,6,0,0,5,0,4,0,0};


	struct timing_action_info t3_ai1={0,2,5,0,0,4,0,0,0,0};
	struct timing_action_info t3_ai2={0,1,8,0,0,7,3,6,0,0};




	fd = fopen("./timseq.bin","wb+");
	if(fd == NULL)
		printf("open file error\n");

	fwrite(&tsh,sizeof(tsh),1,fd);

	fwrite(&t1,sizeof(t1),1,fd);
	fwrite(&t1_ai1,sizeof(t1_ai1),1,fd);
	fwrite(&t1_ai2,sizeof(t1_ai2),1,fd);
	fwrite(&t1_ai3,sizeof(t1_ai3),1,fd);
	fwrite(&t1_ai4,sizeof(t1_ai4),1,fd);
	fwrite(&t1_ai5,sizeof(t1_ai5),1,fd);
	fwrite(&t1_ai6,sizeof(t1_ai6),1,fd);
	fwrite(&t1_ai7,sizeof(t1_ai7),1,fd);
	fwrite(&t1_ai8,sizeof(t1_ai8),1,fd);
	fwrite(&t1_ai9,sizeof(t1_ai9),1,fd);
	fwrite(&t1_ai10,sizeof(t1_ai10),1,fd);

#if 0
	fwrite(&t2,sizeof(t2),1,fd);
	fwrite(&t2_ai1,sizeof(t2_ai1),1,fd);
	fwrite(&t2_ai2,sizeof(t2_ai2),1,fd);
	fwrite(&t2_ai3,sizeof(t2_ai3),1,fd);
	fwrite(&t2_ai4,sizeof(t2_ai4),1,fd);


	fwrite(&t3,sizeof(t3),1,fd);
	fwrite(&t3_ai1,sizeof(t3_ai1),1,fd);
	fwrite(&t3_ai2,sizeof(t3_ai2),1,fd);
#endif


	fclose(fd);
	


}
