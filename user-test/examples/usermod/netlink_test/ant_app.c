#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <string.h>
#include <linux/netlink.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <sys/time.h>

#define NETLINK_FPGA    30
#define MSG_LEN            125
#define MAX_PLOAD        150*1024

#define FPGA_TIMING 4
#define FPGA_EXIT 3
#define FPGA_EXEC 1
#define FPGA_INIT 2



/* 1、首先发送测试命令
 * 2、测试通过后将发送通信指令或者内核主动上报信息
 */
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
	SUBTYPE_TS_CTRL_MEASURE_START=100,
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

struct test {
	int type;
	int len;
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

#define TLV_HEADER 2*sizeof(uint32_t)



static int setnonblocking( int fd )
{
	int opts;
	if( fd<=0 )
		return -1;
	opts = fcntl( fd, F_GETFL );
	if( opts<0 )
		return -1;
	if( fcntl( fd, F_SETFL, opts|O_NONBLOCK ) < 0 )
		return -1;
	return 0;
}

int main(int argc, char **argv)
{
    int skfd;
    int ret;
     int i;

    struct iovec iov;

    socklen_t len;
		struct msghdr msg;
    struct nlmsghdr *nlh = NULL;
    struct sockaddr_nl saddr, daddr;
    char *umsg = "test1111111111111111111111111111112";
    char *timing_str= "getts";
    char *exitmsg = "byebye";
    int msg_len;
    
    int fd,ts_file_len;
    struct stat buf;
	
    struct tlv_data *tlvdata, *subtlvdata,  *rcv_tlvdata, *rcv_sub_tlvdata;
	unsigned char *tmp, *tmpbuf;
	struct measure_related_info *pmri;








	//tmpbuf[ts_file_len+8] = '\0';

    /* 创建NETLINK socket */
    skfd = socket(AF_NETLINK, SOCK_RAW, NETLINK_FPGA);
    if(skfd == -1)
    {
        perror("create socket error\n");
        return -1;
    }

    memset(&saddr, 0, sizeof(saddr));
    saddr.nl_family = AF_NETLINK; //AF_NETLINK
    saddr.nl_pid = 100;//getpid();  //非常关键 内核netlink_unicast发送端口必须一样 不然接收不到数据 getpid()内核主动发送不行
    saddr.nl_groups = 0;
    if(bind(skfd, (struct sockaddr *)&saddr, sizeof(saddr)) != 0)
    {
        perror("bind() error\n");
        close(skfd);
        return -1;
    }

    memset(&daddr, 0, sizeof(daddr));
    daddr.nl_family = AF_NETLINK;
    daddr.nl_pid = 0; // to kernel 0
    daddr.nl_groups = 0;

    nlh = (struct nlmsghdr *)malloc(NLMSG_SPACE(MAX_PLOAD));
	if(nlh != NULL)
		printf("malloc ok\n");
    memset(nlh, 0, sizeof(struct nlmsghdr));
    nlh->nlmsg_len = NLMSG_SPACE(MAX_PLOAD);
    nlh->nlmsg_flags = 0;
    nlh->nlmsg_type = FPGA_INIT;
    nlh->nlmsg_seq = 0;
    nlh->nlmsg_pid =100; //getpid(); //self port

		memset(&iov, 0, sizeof(iov));
		iov.iov_base = nlh;
		iov.iov_len = nlh->nlmsg_len;	
		memset(&msg, 0, sizeof(msg));
		msg.msg_name = &daddr;
		msg.msg_namelen = sizeof(daddr);
		msg.msg_iov = &iov;
		msg.msg_iovlen = 1;



    while(1){
	     printf("\n\n\nnow ready to recv new message...\n");
	    ret = recvmsg(skfd, (struct msghdr *) &msg, 0);
	    if(!ret)
	    {
		perror("recv form kernel error\n");
		close(skfd);
		exit(-1);
	    }
	msg_len = nlh->nlmsg_len -NLMSG_HDRLEN;
	
 	printf("get from kernel len:%d \n",msg_len);
	tmp = malloc(msg_len);
	memcpy(tmp, (char *)NLMSG_DATA(nlh), msg_len);
	
	printf("20:bufdata:%s\n", NLMSG_DATA(nlh));

	for(i = 0; i<20; i++) {
		printf("0x%x ", ((unsigned short *)((unsigned int)tmp))[i]);
	}
	printf("\n");


	
	rcv_tlvdata = (struct tlv_data *)tmp;
	rcv_sub_tlvdata = (struct tlv_data *)(tmp+8);
	//rcv_sub_tlvdata = (struct tlv_data *)(rcv_tlvdata->m_value);
	printf("type:%d subtype:%d\n",rcv_tlvdata->m_type, rcv_sub_tlvdata->m_type);
	if(rcv_tlvdata->m_type == MSG_TYPE_TS_REPORT && rcv_sub_tlvdata->m_type == SUBTYPE_TS_REPORT_TRIGGER)
	{
		printf("tirgger signal... need send func ts data\n");
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	
		fd = open("./encode_file", O_RDONLY);
		if(fd < 0)
			printf("open file failed\n");

		fstat(fd, &buf);	
		printf("file size:%d\n",buf.st_size);
		ts_file_len = buf.st_size;


		tmpbuf = (char *)malloc(ts_file_len + 8);

		tlvdata = (struct tlv_data *)tmpbuf;
		tlvdata->m_type = MSG_TYPE_TS_INIT;
		tlvdata->m_len = ts_file_len;

		ret = read(fd, tmpbuf+8, ts_file_len);
    		
#if 0
		//tmp =(unsigned char *)tlvdata;
		printf("tlvdat:\n");
		for(i = 0; i< ts_file_len; i++)
			printf("0x%x ",tmpbuf[i]);
		printf("\n");
#endif

    	   	nlh->nlmsg_pid = getpid(); //self port

    		nlh->nlmsg_len = NLMSG_SPACE(ts_file_len+8);
	
		printf("tlvdat:nlmsg_len:%d\n", nlh->nlmsg_len);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		memcpy(NLMSG_DATA(nlh), tmpbuf, ts_file_len);
		 // memcpy(NLMSG_DATA(nlh), timing_str, strlen(timing_str));
		printf("111111111\n");
	

	   	ret = sendmsg(skfd, (struct msghdr *) &msg, MSG_DONTWAIT);
	    	if(!ret)
	    	{
			perror("sendmsg1 error\n");
			close(skfd);
			exit(-1);
	    	}
		printf("222222222222\n");
		//free(tmpbuf);
		//close(fd);
		
	}else if(rcv_tlvdata->m_type == MSG_TYPE_TS_REPORT && rcv_sub_tlvdata->m_type == SUBTYPE_TS_REPORT_START) {

		printf("Got TS EXE Start Signal....\n");

	}else if(rcv_tlvdata->m_type == MSG_TYPE_TS_REPORT && rcv_sub_tlvdata->m_type == SUBTYPE_TS_REPORT_END) {

		printf("Got TS EXE End Signal, Bye....\n");
	}




	if(strncmp("start",NLMSG_DATA(nlh),5) == 0){
 	    printf("start\n");

            msg_len = sizeof(struct measure_related_info) + 16;

            nlh->nlmsg_pid = getpid(); //self port
	    nlh->nlmsg_len = NLMSG_SPACE(msg_len);


	tmpbuf = (unsigned char *)malloc(msg_len);

	tlvdata = (struct tlv_data *)tmpbuf;
	tlvdata->m_type = MSG_TYPE_TS_CTRL;
	tlvdata->m_len = sizeof(struct measure_related_info) +8;

	

	subtlvdata = tmpbuf + 8;

	subtlvdata->m_type = SUBTYPE_TS_CTRL_MEASURE_START;

	subtlvdata->m_len = sizeof(struct measure_related_info);
	
	pmri = tmpbuf + 16;
	pmri->m_unique_key = 11223344;
	pmri->m_ts_id = 1;//0x1870c;//2;



            memcpy(NLMSG_DATA(nlh), tmpbuf, msg_len);
            ret = sendmsg(skfd, (struct msghdr *) &msg, MSG_DONTWAIT);

	    if(!ret)
	    {
		perror("sendmsg1 error\n");
		close(skfd);
		exit(-1);
	    }

#if 0
	sleep(10);
	printf("sleep.....10s , start other thread....\n");

	
	memset(tmpbuf,0,msg_len);

	tlvdata = (struct tlv_data *)tmpbuf;
	tlvdata->m_type = MSG_TYPE_TS_CTRL;
	tlvdata->m_len = sizeof(struct measure_related_info) +8;

	

	subtlvdata = tmpbuf + 8;

	subtlvdata->m_type = SUBTYPE_TS_CTRL_MEASURE_START;

	subtlvdata->m_len = sizeof(struct measure_related_info);
	
	pmri = tmpbuf + 16;
	pmri->m_unique_key = 55668899;
	pmri->m_ts_id = 100021;



            memcpy(NLMSG_DATA(nlh), tmpbuf, msg_len);
            ret = sendmsg(skfd, (struct msghdr *) &msg, 0);

	    if(!ret)
	    {
		perror("sendmsg1 error\n");
		close(skfd);
		exit(-1);
	    }
#endif

	free(tmpbuf);


    }else if(strncmp("stop",NLMSG_DATA(nlh),4) == 0){
 	    printf("stop one thread...\n");


            msg_len = sizeof(struct measure_related_info) + 16;

            nlh->nlmsg_pid = getpid(); //self port
	    nlh->nlmsg_len = NLMSG_SPACE(msg_len);


	tmpbuf = (unsigned char *)malloc(msg_len);

	tlvdata = (struct tlv_data *)tmpbuf;
	tlvdata->m_type = MSG_TYPE_TS_CTRL;
	tlvdata->m_len = sizeof(struct measure_related_info) +8;

	

	subtlvdata = tmpbuf + 8;

	subtlvdata->m_type = SUBTYPE_TS_CTRL_MEASURE_STOP;

	subtlvdata->m_len = sizeof(struct measure_related_info);
	
	pmri = tmpbuf + 16;
	pmri->m_unique_key = 333666999;
	pmri->m_ts_id = 1;



            memcpy(NLMSG_DATA(nlh), tmpbuf, msg_len);
            ret = sendmsg(skfd, (struct msghdr *) &msg, 0);

	    if(!ret)
	    {
		perror("sendmsg1 error\n");
		close(skfd);
		exit(-1);
	    }
	free(tmpbuf);

   }else if(strncmp("forcestop",NLMSG_DATA(nlh),9) == 0){
 	    printf("forcestop all thread...\n");


            msg_len = sizeof(struct measure_related_info) + 16;

            nlh->nlmsg_pid = getpid(); //self port
	    nlh->nlmsg_len = NLMSG_SPACE(msg_len);


	tmpbuf = (unsigned char *)malloc(msg_len);

	tlvdata = (struct tlv_data *)tmpbuf;
	tlvdata->m_type = MSG_TYPE_TS_CTRL;
	tlvdata->m_len = sizeof(struct measure_related_info) +8;

	

	subtlvdata = tmpbuf + 8;

	subtlvdata->m_type = SUBTYPE_TS_CTRL_MEASURE_FORCE_STOP;

	subtlvdata->m_len = sizeof(struct measure_related_info);
	


            memcpy(NLMSG_DATA(nlh), tmpbuf, msg_len);
            ret = sendmsg(skfd, (struct msghdr *) &msg, 0);

	    if(!ret)
	    {
		perror("sendmsg1 error\n");
		close(skfd);
		exit(-1);
	    }

   }else if(strncmp("runtiming",NLMSG_DATA(nlh),9) == 0){
 	    nlh->nlmsg_type = FPGA_EXEC;
            nlh->nlmsg_pid = getpid(); //self port
 	    memcpy(NLMSG_DATA(nlh), umsg, strlen(umsg));
	   ret = sendmsg(skfd, (struct msghdr *) &msg, 0);
	    if(!ret)
	    {
		perror("sendmsg1 error\n");
		close(skfd);
		exit(-1);
	    }
  }else if(strncmp("exit",NLMSG_DATA(nlh),4) == 0){
    	    nlh->nlmsg_type = FPGA_EXIT;
            nlh->nlmsg_pid = getpid(); //self port
 	    memcpy(NLMSG_DATA(nlh), exitmsg, strlen(exitmsg));
	   ret = sendmsg(skfd, (struct msghdr *) &msg, 0);
	    if(!ret)
	    {
		perror("exit,byebye\n");
	    }

  }else if(strncmp("settemp",NLMSG_DATA(nlh),7) == 0){
 	    printf("get temp...\n");
	    struct range_info *pri;

            msg_len = sizeof(struct range_info) + 16;

            nlh->nlmsg_pid = getpid(); //self port
	    nlh->nlmsg_len = NLMSG_SPACE(msg_len);


	tmpbuf = (unsigned char *)malloc(msg_len);

	tlvdata = (struct tlv_data *)tmpbuf;
	tlvdata->m_type = MSG_TYPE_OTHER_CTRL;
	tlvdata->m_len = sizeof(struct range_info) +8;

	

	subtlvdata = tmpbuf + 8;

	subtlvdata->m_type = SUBTYPE_OTHERCTRL_SET_TEMP;

	subtlvdata->m_len = sizeof(struct range_info);
	
	pri = tmpbuf + 16;
	pri->m_id = 2;
	pri->m_min = 20;
        pri->m_max = 60;
	pri->m_actual = 33;

            memcpy(NLMSG_DATA(nlh), tmpbuf, msg_len);
            ret = sendmsg(skfd, (struct msghdr *) &msg, 0);

	    if(!ret)
	    {
		perror("sendmsg1 error\n");
		close(skfd);
		exit(-1);
	    }
	free(tmpbuf);

   	}else if(strncmp("setmotor",NLMSG_DATA(nlh),7) == 0){
 	    printf("set motor...\n");
	    struct motor_info *pmi;

            msg_len = sizeof(struct motor_info) + 16;

            nlh->nlmsg_pid = getpid(); //self port
	    nlh->nlmsg_len = NLMSG_SPACE(msg_len);


	tmpbuf = (unsigned char *)malloc(msg_len);

	tlvdata = (struct tlv_data *)tmpbuf;
	tlvdata->m_type = MSG_TYPE_OTHER_CTRL;
	tlvdata->m_len = sizeof(struct motor_info) +8;

	

	subtlvdata = tmpbuf + 8;

	subtlvdata->m_type = SUBTYPE_OTHERCTRL_SET_INJECT_MOVE;

	subtlvdata->m_len = sizeof(struct motor_info);
	
	pmi = tmpbuf + 16;
	pmi->m_id = 2;
	pmi->m_speed = 120;
        pmi->m_pos_target = 3;
	pmi->m_steps = 330;

            memcpy(NLMSG_DATA(nlh), tmpbuf, msg_len);
            ret = sendmsg(skfd, (struct msghdr *) &msg, 0);

	    if(!ret)
	    {
		perror("sendmsg1 error\n");
		close(skfd);
		exit(-1);
	    }
	free(tmpbuf);

   	}
   
   }

    close(skfd);

   // free((void *)nlh);
    return 0;
}
