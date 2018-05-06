#ifndef __POD_IO_H
#define __POD_IO_H

#include <iostream>
#include <termios.h>   	 /*POSIX terminal contorl*/
#include <fcntl.h>       /*file control define */

#define RX_BUF_LEN 			500
#define TX_BUF_LEN			30

struct port_frame
{
  uint8_t frame_id;
  uint8_t frame_len;
  uint8_t data_len;
  unsigned char *data;
};

enum NET_CMD
{
  START_MISSION = 1,
  STOP_MISSION,
  START_FLY,
  STOP_FLY
};

typedef enum FRRAME_ID
{
  CMD_FRAME = 1,
  REPORT_FRAME,
  RSSI_FRAME,
}FRRAME_ID;


typedef struct network_report
{
  uint16_t node_addr;
}network_report;

typedef struct rssi_report
{
  uint16_t time;
  signed char rssi_buf[3];
}rssi_report;

using namespace std;

class serial_port_com
{
    int net_fd;
    char rx_buffer[RX_BUF_LEN];
    //    int speed;
public:

	   void (*receive_callback) (unsigned char ch);

    int serial_open_com(int speed)
    {
    	struct termios opt; 
        string full_dir("/dev/ttyACM0");

        net_fd = open(full_dir.c_str(), O_RDWR|O_NOCTTY);

        if (net_fd == -1)   
        {           
            cout << "Can't Open Serial Port" << endl;
            return -1;
        }
        else
        {   
            cout << "Open Successfully!" << endl;
            set_opt(net_fd,speed,8,'n',1);

            return net_fd;
        }
    }

    int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop) 
    { 
       struct termios newtio,oldtio; 
  
       if  ( tcgetattr( fd,&oldtio)  !=  0) 
       {  
        perror("SetupSerial 1");
        printf("tcgetattr( fd,&oldtio) -> %d\n",tcgetattr( fd,&oldtio)); 
        return -1; 
       } 
       bzero( &newtio, sizeof( newtio ) ); 
  
       newtio.c_cflag  |=  CLOCAL | CREAD;  
       newtio.c_cflag &= ~CSIZE;  
 
       switch( nBits ) 
       { 
       case 7: 
        newtio.c_cflag |= CS7; 
        break; 
       case 8: 
        newtio.c_cflag |= CS8; 
        break; 
       } 

       switch( nEvent ) 
       { 
       case 'o':
       case 'O': //奇数 
        newtio.c_cflag |= PARENB; 
        newtio.c_cflag |= PARODD; 
        newtio.c_iflag |= (INPCK | ISTRIP); 
        break; 
       case 'e':
       case 'E': //偶数 
        newtio.c_iflag |= (INPCK | ISTRIP); 
        newtio.c_cflag |= PARENB; 
        newtio.c_cflag &= ~PARODD; 
        break;
       case 'n':
       case 'N':  //无奇偶校验位 
        newtio.c_cflag &= ~PARENB; 
        break;
       default:
        break;
       } 
        
      switch( nSpeed ) 
       { 
       case 2400: 
        cfsetispeed(&newtio, B2400); 
        cfsetospeed(&newtio, B2400); 
        break; 
       case 4800: 
        cfsetispeed(&newtio, B4800); 
        cfsetospeed(&newtio, B4800); 
        break; 
       case 9600: 
        cfsetispeed(&newtio, B9600); 
        cfsetospeed(&newtio, B9600); 
        break; 
       case 115200: 
        cfsetispeed(&newtio, B115200); 
        cfsetospeed(&newtio, B115200); 
        break; 
       case 460800: 
        cfsetispeed(&newtio, B460800); 
        cfsetospeed(&newtio, B460800); 
        break; 
       default: 
        cfsetispeed(&newtio, B9600); 
        cfsetospeed(&newtio, B9600); 
       break; 
       } 

       if( nStop == 1 ) 
        newtio.c_cflag &=  ~CSTOPB; 
       else if ( nStop == 2 ) 
        newtio.c_cflag |=  CSTOPB; 
  
       newtio.c_cc[VTIME]  = 0; 
       newtio.c_cc[VMIN] = 0; 
  
       tcflush(fd,TCIFLUSH); 
  
    if((tcsetattr(fd,TCSANOW,&newtio))!=0) 
       { 
        perror("com set error"); 
        return -1; 
       } 
       printf("set done!\n"); 
       return 0; 
    }

    //------------------ receive data--- --------------------------//
     void serial_port_receive(void)
	{
		
	    int nByte,i;

	    if(receive_callback == NULL)
	    {
	    	cout << "need a receive callback function!!!";
	    	return;
	    }

		while((nByte = read(net_fd, rx_buffer, RX_BUF_LEN))>0)
	    {
	        //cout << "received:" << nByte << endl;
	        for(i = 0; i < nByte; i++)
	        {
	            //printf("%02x\n",(unsigned char)rx_buffer[i]);
	            receive_callback(rx_buffer[i]);
                
	        }
          printf("%s\n",rx_buffer);
	        memset(rx_buffer, 0, strlen(rx_buffer));
	        nByte = 0;
	    }
	}

    //------------------ write data--- --------------------------//
    // void *Txthread(void *ptr)
    int serial_port_write(char* Txbuf, int len)
    {
        int WriteNum = write(net_fd,Txbuf,len);
        //cout << "send num:" << WriteNum << endl;
        usleep(20000);
        return WriteNum;
     }
};


int open_port(void);
void port_receive(void);
int port_sent_frame(port_frame * frame);
uint8_t get_gateway_cmd(void);
void reset_gateway_cmd(void);


#endif
