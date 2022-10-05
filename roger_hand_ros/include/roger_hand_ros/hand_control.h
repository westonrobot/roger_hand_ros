#ifndef _HAND_CONTROL_H_
#define _HAND_CONTROL_H_

#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <sys/times.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <iostream>

#define TIMEOUT_SEC(buflen,baud) (buflen*20/baud+2)             //接收超时
#define TIMEOUT_USEC 0

#define CMD_MAX         7
#define TX_MAX          12
#define C_NULL          0x00
#define BAUDRATE     115200


struct PortInfo_t
{
    int         baudrate;                                       //baudrate
    int         databit;                                        //data bits, 5, 6, 7, 8
    int         fctl;                                           //flow control, 0: none, 1: hardware, 2: software
    int         parity;                                         //parity 0: none, 1: odd, 2: even
    int         stopbit;                                        //stop bits, 1, 2
    const int   reserved;                                       //reserved, must be zero
};

typedef PortInfo_t *pPortInfo_t;

/*
 * 打开串口，返回文件描述符
 * dev：设备文件名
*/
int SerialOpen(const char* dev);

/*
 * 设置串口
 * fdcom: 串口文件描述符， pportinfo： 待设置的串口信息
*/
int SerialSet(int fdcom, const pPortInfo_t pportinfo);

/*
 * 关闭串口
 * fdcom：串口文件描述符
*/
void SerialClose(int fdcom);

/*
 * 发送数据
 * fdcom：串口描述符， data：待发送数据， datalen：数据长度
 * 返回实际发送长度
*/
int SerialSend(int fdcom, const unsigned char *data, int datalen);

/*
 * 接收数据
 * fdcom：串口描述符， data：接收缓冲区, datalen.：接收长度， baudrate：波特率
 * 返回实际读入的长度
*/
int SerialRecv(int fdcom, unsigned char *data, int datalen, int baudrate);


/*
 * 波特率转换函数
*/
int ConvBaudRate(unsigned long int baudrate);

/*
 * 发送位置指令
*/
int WritePos(int fd,int index,int pos_dst,int& pos_real,int& temp,int& ampere,int& err);

/*
 * 状态查询
*/
int ReadState(int fd,int index,int& pos_real,int& temp,int& ampere,int& err);
/*
 * 参数读取
*/
int ReadPara(int fd, int index,int& pro_ampere,int& pro_temp, int& work_temp);
/*
 * 参数装订
*/
int SetPara(int fd, int index, int pro_ampere, int pro_temp, int work_temp);

/*
 * 停止
*/
int StopWorking(int fd, int index, int &pos_real, int &temp, int &ampere, int &err);

/*
 * 工作
*/
int StartWorking(int fd, int index, int &pos_real, int &temp, int &ampere, int &err);

/*
 * 消除异常
*/
int ClearErr(int fd, int index, int &pos_real, int &temp, int &ampere, int &err);

/*
 *睡眠
*/
void sleep_time(unsigned int msec);

void enable_hand(int fd);
void unable_hand(int fd);
void clearErr_hand(int fd);

#endif
