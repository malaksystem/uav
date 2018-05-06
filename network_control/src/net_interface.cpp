#include <iostream>
#include <unistd.h>
#include <pthread.h>

#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"


#include "network_control/net_interface.h"
#include "network_control/network_control.h"


#define HEADER_0  0X55
#define HEADER_1  0XAA


enum DAT_STATE
{
  DAT_INIT = 0,
  DAT_HEADER,
  DAT_LEN,
  DAT_ID,
  DAT_DATA,
  DAT_CHACK
};

/*---------------------------------------------------------------------------*/
void * read_thread(void * arg);
void frame_decode(unsigned char ch);
static unsigned char get_check(unsigned char * data_buf);
static void data_decode(port_frame * frame);
int sent_data(unsigned char *data,unsigned int len);

serial_port_com net_com;
uint8_t NET_cmd = 255;

port_frame receive_frame;

unsigned char sent_buf[TX_BUF_LEN];

int open_port(void)
{
	net_com.receive_callback = frame_decode;
	return net_com.serial_open_com(115200);
}

void port_receive(void)
{
	net_com.serial_port_receive();
}

int port_sent_frame(port_frame * frame)
{
	unsigned int frame_len = 0,i,ret;

	memset(sent_buf, 0, TX_BUF_LEN);

	sent_buf[frame_len++] = HEADER_0;
	sent_buf[frame_len++] = HEADER_1;

	sent_buf[frame_len++] = 0X00;			//2 frame length

	sent_buf[frame_len++] = frame->frame_id;			//frame id 

	for (i = 0; i < frame->data_len; ++i)
	{
		sent_buf[frame_len++] = frame->data[i];
	}

	sent_buf[2] = frame_len - 3;

	sent_buf[frame_len++] = get_check(sent_buf+2);
	
	sent_buf[frame_len++] = 0X0a;			//add 0x0a for contiki detect END!!! don't send 0x0a in data, it will be ignored by receiver

	/*for (i = 0; i < frame_len; ++i)
	{
		printf("%02x\n",(unsigned char)sent_buf[i]);
	}*/
	

	ret = net_com.serial_port_write((char *)sent_buf, frame_len);

	return ret;
}

void frame_decode(unsigned char ch)
{
	static DAT_STATE receive_state = DAT_INIT;
	static int data_count = 0;
	static unsigned char data_buf[RX_BUF_LEN];
	//cout << "receive_state:" << receive_state << endl;

	switch (receive_state)
	{
		case DAT_INIT:
		{
			if(ch == HEADER_0)
			{
				receive_state = DAT_HEADER;
			}
			break;
		}

		case DAT_HEADER:
		{

			if(ch == HEADER_1)
			{
				receive_state = DAT_LEN;
			}
			else
			{
				receive_state = DAT_INIT;
			}
			break;
		}
		case DAT_LEN:
		{
			data_buf[data_count++] = ch;
			receive_frame.frame_len = ch;
			receive_frame.data_len = receive_frame.frame_len - 1;
			receive_state = DAT_ID;
			//cout << "frame_len:" << receive_frame.frame_len << endl;
			break;
		}

		case DAT_ID:
		{
			data_buf[data_count++] = ch;
			receive_frame.frame_id = ch;
			receive_state = DAT_DATA;
			break;
		}

		case DAT_DATA:
		{
			data_buf[data_count++] = ch;
			//cout << "data_count" << data_count << endl;

			if(data_count == (receive_frame.frame_len + 1))
			{
				//cout << "frame_len" << frame_len << endl;
				receive_state = DAT_CHACK;
				data_count = 0;
			}
			break;
		}

		case DAT_CHACK:
		{
			unsigned char check_ore;
			check_ore = get_check(data_buf);
			//printf("ch :%02x  get:%02x \n",ch,check_ore);
			if(ch == check_ore)
			{
				receive_frame.data = (data_buf + 2);
				data_decode(&receive_frame);
				//cout  << "frame_id:" << receive_frame.frame_id << "  check successfully!" << endl; 
				//printf("data1 :%02x \n",data_buf[2]);
			}
			else
			{
				cout << "frame_id:" << receive_frame.frame_id  << "  check fail!" << endl;
			}
			memset(data_buf, 0, strlen((char *)data_buf));
			receive_state = DAT_INIT;
			break;
		}

		default :
		{
			receive_state = DAT_INIT;
			break;
		}
	}
}


uint8_t get_gateway_cmd(void)
{
	return NET_cmd;
}

void reset_gateway_cmd(void)
{
	NET_cmd = 255;
}

static void data_decode(port_frame * frame)
{
	
	switch(frame->frame_id)
	{
		case CMD_FRAME:
			NET_cmd = frame->data[0];
			break;
		case REPORT_FRAME:
			network_report net_report;
			net_report = *(network_report *)frame->data;
			cout<< "node " << net_report.node_addr << " get new version!!!" << endl;
			net_data_record(net_report);
		break;
		case RSSI_FRAME:
			rssi_report report;
			report = *(rssi_report *)frame->data;

			if(UAV_is_ready())
			{
				rssi_record(report);
			}
		break;
		default:
			break;
	}
}

static unsigned char get_check(unsigned char * data_buf)
{
	int i,check_len;
	unsigned char ret = data_buf[0];
	check_len = data_buf[0] + 1;
	for(i = 1;i < check_len;i++)
	{
		ret ^= data_buf[i];
		//printf("i :%d  data:  %02x \n",i,data_buf[i]);
	}
	return ret;
}
