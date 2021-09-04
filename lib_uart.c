#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <poll.h>
#include <stdlib.h>
#include "../common.h"

#define ERROR_NOSUPPORT	-5
#define ERROR_SYS		-4
#define ERROR_INPARA	-3
#define ERROR_TIMEOUT	-2
#define ERROR_FAIL		-1
#define ERROR_OK		 0

static int set_opt(int fd,int nBaud, int nBits, char nEvent, int nStop)
{
    struct termios newtio,oldtio;
    if( tcgetattr( fd,&oldtio)  !=  0) { 
        perror("SetupSerial 1");
        return -1;
    }
    bzero( &newtio, sizeof( newtio ) );
    newtio.c_cflag  |=  CLOCAL | CREAD; 
    newtio.c_cflag &= ~CSIZE; 

    switch( nBits ) {
		case 7:
			newtio.c_cflag |= CS7;
			break;
		case 8:
			newtio.c_cflag |= CS8;
			break;
    }

    switch(nEvent) {
		case 'O':                     //��У��
		case 'o':
			newtio.c_cflag |= PARENB;
			newtio.c_cflag |= PARODD;
			newtio.c_iflag |= (INPCK);
			break;
		case 'E':                     //żУ��
		case 'e':
			newtio.c_iflag |= (INPCK);
			newtio.c_cflag |= PARENB;
			newtio.c_cflag &= ~PARODD;
			break;
		case 'N':                    //��У��
		case 'n':
			newtio.c_cflag &= ~PARENB;
			break;
    }

	switch( nBaud ) {
		case 50: cfsetispeed(&newtio, B50); cfsetospeed(&newtio, B50); break;
		case 75: cfsetispeed(&newtio, B75); cfsetospeed(&newtio, B75); break;
		case 110: cfsetispeed(&newtio, B110); cfsetospeed(&newtio, B110); break;
		case 134: cfsetispeed(&newtio, B134); cfsetospeed(&newtio, B134); break;
		case 150: cfsetispeed(&newtio, B150); cfsetospeed(&newtio, B150); break;
		case 200: cfsetispeed(&newtio, B200); cfsetospeed(&newtio, B200); break;
		case 300: cfsetispeed(&newtio, B300); cfsetospeed(&newtio, B300); break;	
		case 600: cfsetispeed(&newtio, B600); cfsetospeed(&newtio, B600); break;		
		case 1200: cfsetispeed(&newtio, B1200); cfsetospeed(&newtio, B1200); break;
		case 1800: cfsetispeed(&newtio, B1800); cfsetospeed(&newtio, B1800); break;		
		case 2400: cfsetispeed(&newtio, B2400); cfsetospeed(&newtio, B2400); break;
		case 4800: cfsetispeed(&newtio, B4800); cfsetospeed(&newtio, B4800); break;
		case 9600: cfsetispeed(&newtio, B9600); cfsetospeed(&newtio, B9600); break;
		case 19200: cfsetispeed(&newtio, B19200); cfsetospeed(&newtio, B19200); break;       
		case 38400: cfsetispeed(&newtio, B38400); cfsetospeed(&newtio, B38400); break;  
		case 57600: cfsetispeed(&newtio, B57600); cfsetospeed(&newtio, B57600); break;
		case 115200: cfsetispeed(&newtio, B115200); cfsetospeed(&newtio, B115200); break;
		case 230400: cfsetispeed(&newtio, B230400); cfsetospeed(&newtio, B230400); break;
		case 460800: cfsetispeed(&newtio, B460800); cfsetospeed(&newtio, B460800); break;
		case 500000: cfsetispeed(&newtio, B500000); cfsetospeed(&newtio, B500000); break;
		case 576000: cfsetispeed(&newtio, B576000); cfsetospeed(&newtio, B576000); break;
		case 921600: cfsetispeed(&newtio, B921600); cfsetospeed(&newtio, B921600); break;
		case 1000000: cfsetispeed(&newtio, B1000000); cfsetospeed(&newtio, B1000000); break;
		case 1152000: cfsetispeed(&newtio, B1152000); cfsetospeed(&newtio, B1152000); break;
		case 1500000: cfsetispeed(&newtio, B1500000); cfsetospeed(&newtio, B1500000); break;
		case 2000000: cfsetispeed(&newtio, B2000000); cfsetospeed(&newtio, B2000000); break;
		case 2500000: cfsetispeed(&newtio, B2500000); cfsetospeed(&newtio, B2500000); break;
		case 3000000: cfsetispeed(&newtio, B3000000); cfsetospeed(&newtio, B3000000); break;
		case 3500000: cfsetispeed(&newtio, B3500000); cfsetospeed(&newtio, B3500000); break;
		case 4000000: cfsetispeed(&newtio, B4000000); cfsetospeed(&newtio, B4000000); break;
		default:
			printf("sorry! [%d bps] is not support ,Set default 9600bps now!\n",nBaud);
			cfsetispeed(&newtio, B9600);
			cfsetospeed(&newtio, B9600);
			break;
    }
    if(nStop == 1) {
        newtio.c_cflag &=  ~CSTOPB;
    } else if (nStop == 2) {
        newtio.c_cflag |=  CSTOPB;
    }
    newtio.c_cc[VTIME]  = 0;
    newtio.c_cc[VMIN] = 0;
    tcflush(fd,TCIFLUSH);
    if((tcsetattr(fd,TCSANOW,&newtio))!=0) {
        perror("com set error");
        return -1;
    }
    //printf("set done!\n");
    return 0;
}

static int open_port(char *dev)
{
	int fd;
	fd = open(dev, O_RDWR|O_NOCTTY); //open("/dev/tttyS0",O_RDWR | O_NOCTTY);  //O_NDELAY
    if (fd < 0) {
		return fd;
	}
	
    if(fcntl(fd, F_SETFL, 0)<0) {
        printf("fcntl failed!\n");
    }
    if(isatty(STDIN_FILENO)==0) {
        printf("standard input is not a terminal device\n");
    }

    return fd;
}

int uart_init(char* dev, int nBaud, int nBits, char nEvent, int nStop)
{
	int ret;
	int fd = open_port(dev);
	if(fd < 0) {
		return -1;
	}
	ret = set_opt(fd,nBaud,nBits,nEvent,nStop);
	close(fd);
        return ret;
}

int tq_receive_uart(char *dev_name,char *buff,int len, int timeout_ms, int rx_time_ms)
{

	int i,ret;
	int nread; 
	int fd;
	fd = open(dev_name, O_RDONLY|O_NOCTTY);
	if(fd < 0) {
		return ERROR_NOSUPPORT;
	}
	struct pollfd pollfds;
	pollfds.fd  = fd;
	pollfds.events = POLLIN;
	ret = poll(&pollfds,1,timeout_ms);
	
	if(pollfds.revents == POLLIN) {
		usleep(rx_time_ms*1000);
		nread = read(fd,buff,len);
		//printf("buff=%s\n",buff);
		ret = nread;
	} else {
		ret = ERROR_TIMEOUT;
	}
	close(fd);
	return ret;
}

/*int tq_receive_uart(int fd,char *buff,int len, int timeout_ms)
{
	int i,ret;
	int nread; 
	
	struct pollfd pollfds;
	pollfds.fd  = fd;
	pollfds.events = POLLIN;
	ret = poll(&pollfds,1,timeout_ms);
	
	if(pollfds.revents == POLLIN) {
		nread = read(fd,buff,len);
		ret = nread;
	} else {
		ret = ERROR_TIMEOUT;
	}
	close(fd);
	return ret;
}*/

int uart_write_data(int fd,char *buff,int len)
{
	int i,ret;
	if(check_fd_fine(fd) != 0){
		return ERROR_NOSUPPORT;
	}
	ret = write(fd,buff,len);
	if(ret < 0) {
		ret = ERROR_FAIL;
	}
	return ret;
}

int uart_open(char *dev_name)
{
        return open_port(dev_name);
}

int uart_read_data(int fd, char *buff,int len, int timeout_ms)
{
	int i,ret;
	int nread; 
	struct pollfd pollfds;
	if(check_fd_fine(fd) != 0){
		return ERROR_NOSUPPORT;
	}
	pollfds.fd  = fd;
	pollfds.events = POLLIN;
	ret = poll(&pollfds,1,timeout_ms);
	
	if(pollfds.revents == POLLIN) {
		memset(buff, 0x0, len);
		nread = read(fd,buff,len);
		//printf("buff=%s\n",buff);
		ret = nread;
	} else {
		ret = ERROR_TIMEOUT;
	}
	return ret;
}

void uart_close(int fd)
{
	if(check_fd_fine(fd) == 0)
		close(fd);
}

// int tq_uart_test(){
//     int fd;
//     int i = 0;
//     char *dev_name = "/dev/ttySAC3";
//     if(fd = uart_init(dev_name,115200,8,'E',1)<0)
//         return -1;

//     int read_fd = uart_start_receive(dev_name);
//     if(read_fd < 0)
//         return -3;

//     char buff[64] = "123456789" ;
//     int ret = uart_write_data(dev_name,buff,sizeof(buff));		//�������ݵ�dev_name
//     if(ret < 0)
//         return -2;

//     char str[64];
//     memset(str,0,sizeof(str));
//     ret = uart_read_data(read_fd,str,sizeof(str), 100);	//�Ӵ��ڽ�������
//     if(ret < 0)
//         return -4;
// 	else{
// 		for(i =0;i<ret;i++){
// 			if(str[i]!=buff[i])				
// 				return -5;
// 		}
// 	}

//     uart_stop_receive(read_fd);

//     return 0;
// }
