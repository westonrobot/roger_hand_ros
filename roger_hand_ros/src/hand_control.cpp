#include "roger_hand_ros/hand_control.h"

/*******************************************
 *  Setup serial attr
 *  fdcom: 串口文件描述符，pportinfo: 待设置的端口信息（请确认）
 *
********************************************/
int SerialSet(int fdcom, const pPortInfo_t pportinfo)
{
    termios termios_old, termios_new;
    int     baudrate, tmp;
    char    databit, stopbit, parity, fctl;

    bzero(&termios_old, sizeof(termios_old));
    bzero(&termios_new, sizeof(termios_new));
    cfmakeraw(&termios_new);
    tcgetattr(fdcom, &termios_old);                                 	//get the serial port attributions
    /*------------设置端口属性----------------*/
    //baudrates
    baudrate = ConvBaudRate(pportinfo -> baudrate);
    cfsetispeed(&termios_new, baudrate);                            	//填入串口输入端的波特率
    cfsetospeed(&termios_new, baudrate);                            	//填入串口输出端的波特率
    termios_new.c_cflag |= CLOCAL;                                  	//控制模式，保证程序不会成为端口的占有者
    termios_new.c_cflag |= CREAD;                                   	//控制模式，使能端口读取输入的数据

    // 控制模式，flow control
    fctl = pportinfo-> fctl;
    switch(fctl){
    case 0:{
        termios_new.c_cflag &= ~CRTSCTS;                        	//no flow control
    }break;
    case 1:{
        termios_new.c_cflag |= CRTSCTS;                         	//hardware flow control
    }break;
    case 2:{
        termios_new.c_iflag |= IXON | IXOFF |IXANY;             	//software flow control
    }break;
    }

    //控制模式，data bits
    termios_new.c_cflag &= ~CSIZE;                                  	//控制模式，屏蔽字符大小位
    databit = pportinfo -> databit;
    switch(databit){
    case 5:
        termios_new.c_cflag |= CS5;
    case 6:
        termios_new.c_cflag |= CS6;
    case 7:
        termios_new.c_cflag |= CS7;
    default:
        termios_new.c_cflag |= CS8;
    }

    //控制模式 parity check
    parity = pportinfo -> parity;
    switch(parity){
    case 0:{
        termios_new.c_cflag &= ~PARENB;                         	//no parity check
    }break;
    case 1:{
        termios_new.c_cflag |= PARENB;                          	//odd check
        termios_new.c_cflag &= ~PARODD;
    }break;
    case 2:{
        termios_new.c_cflag |= PARENB;                          	//even check
        termios_new.c_cflag |= PARODD;
    }break;
    }

    //控制模式，stop bits
    stopbit = pportinfo -> stopbit;
    if(stopbit == 2){
        termios_new.c_cflag |= CSTOPB;                              	//2 stop bits
    }
    else{
        termios_new.c_cflag &= ~CSTOPB;                             	//1 stop bits
    }

    //other attributions default
    termios_new.c_oflag &= ~OPOST;          				            //输出模式，原始数据输出
    termios_new.c_cc[VMIN]  = 1;            				            //控制字符, 所要读取字符的最小数量
    termios_new.c_cc[VTIME] = 1;            				            //控制字符, 读取第一个字符的等待时间	unit: (1/10)second

    tcflush(fdcom, TCIFLUSH);               				            //溢出的数据可以接收，但不读
    tmp = tcsetattr(fdcom, TCSANOW, &termios_new);  			        //设置新属性，TCSANOW：所有改变立即生效	tcgetattr(fdcom, &termios_old);
    return(tmp);
}

/*******************************************
 *  Open serial port
 *  tty: 端口号 ttyS0, ttyS1, ....
 *  返回值为串口文件描述符
********************************************/
int SerialOpen(const char* dev)
{
    return(open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK));
}

/*******************************************
 *  Close serial port
********************************************/
void SerialClose(int fdcom)
{
    close(fdcom);
}

/********************************************
 *  send data
 *  fdcom: 串口描述符，data: 待发送数据，datalen: 数据长度
 *  返回实际发送长度
*********************************************/
int SerialSend(int fdcom, const unsigned char *data, int datalen)
{
    int len = 0;

    len = write(fdcom, data, datalen);
    if(len == datalen){
        return (len);
    }
    else{
        tcflush(fdcom, TCOFLUSH);
        return -1;
    }
}

/*******************************************
 *  receive data
 *  返回实际读入的字节数
 *
********************************************/
int SerialRecv(int fdcom, unsigned char *data, int datalen, int baudrate)
{
    int readlen, fs_sel;
    fd_set  fs_read;
    struct timeval tv_timeout;

    FD_ZERO(&fs_read);
    FD_SET(fdcom, &fs_read);
    tv_timeout.tv_sec = TIMEOUT_SEC(datalen, baudrate);
    tv_timeout.tv_usec = TIMEOUT_USEC;

    fs_sel = select(fdcom+1, &fs_read, NULL, NULL, &tv_timeout);
    if(fs_sel){
        readlen = read(fdcom, data, datalen);
        return(readlen);
    }
    else{
        return(-1);
    }

    return (readlen);
}

int ConvBaudRate(unsigned long int baudrate)
{
    switch(baudrate){
    case 2400:
        return B2400;
    case 4800:
        return B4800;
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    default:
        return B9600;
    }
}

/*
 * 发送位置指令
*/
int WritePos(int fd,int index,int pos_dst,int& pos_real,int& temp,int& ampere,int& err){
    unsigned char TxData[9];
    unsigned char dt0 = (unsigned char)pos_dst;
    unsigned char dt1 = (unsigned char)(pos_dst>>8);
    TxData[0] = 0x55;//帧头
    TxData[1] = 0xAA;//帧头
    TxData[2] = 0x04;//帧长度 N+2
    TxData[3] = (unsigned char)index;//ID
    TxData[4] = 0x021;//指令类型
    TxData[5] = 0x37;//控制表索引地址
    TxData[6] = dt0;//低8位
    TxData[7] = dt1;//高8位
    TxData[8] = TxData[2]+TxData[3]+TxData[4]+TxData[5]+TxData[6]+TxData[7];
    int len = SerialSend(fd,TxData,sizeof( TxData));
    if(len>0)
    {
        int data_len=22;
        unsigned char rcv_buf[data_len];
        len = SerialRecv(fd,rcv_buf,data_len,BAUDRATE);
        pos_real = (int)(rcv_buf[9])+(int)(rcv_buf[10]<<8);
        temp = (int)rcv_buf[11];
        ampere = (int)rcv_buf[12]+(int)(rcv_buf[13]<<8);
        err = (int)rcv_buf[15];
    }
    return len;
}

/*
 * 状态查询
*/
int ReadState(int fd, int index, int& pos_real, int&temp, int&ampere, int&err){
    unsigned char TxData[8];
    TxData[0] = 0x55;//帧头
    TxData[1] = 0xAA;//帧头
    TxData[2] = 0x03;//帧长度
    TxData[3] = (unsigned char)index;//ID
    TxData[4] = 0x04;//指令类型
    TxData[5] = 0x00;//控制表索引地址
    TxData[6] = 0x22;//查询状态信息
    TxData[7] = TxData[2]+TxData[3]+TxData[4]+TxData[5]+TxData[6];
    int len = SerialSend(fd,TxData,sizeof( TxData));
    if(len>0)
    {
        int data_len=22;
        unsigned char rcv_buf[data_len];
        len = SerialRecv(fd,rcv_buf,data_len,BAUDRATE);
        pos_real = (int)(rcv_buf[9])+(int)(rcv_buf[10]<<8);
        temp = (int)rcv_buf[11];
        ampere = (int)rcv_buf[12]+(int)(rcv_buf[13]<<8);
        err = (int)rcv_buf[15];
    }
    return len;
}
int ReadPara(int fd, int index,int& pro_ampere,int& pro_temp,int& work_temp){
    unsigned char TxData[8];
    TxData[0] = 0x55;//帧头
    TxData[1] = 0xAA;//帧头
    TxData[2] = 0x03;//帧长度
    TxData[3] = (unsigned char)index;//ID
    TxData[4] = 0x01;//指令类型
    TxData[5] = 0x20;//控制表索引地址 过流保护
    TxData[6] = 0x02;//数据段字节数
    TxData[7] = TxData[2]+TxData[3]+TxData[4]+TxData[5]+TxData[6];
    int len = SerialSend(fd,TxData,sizeof( TxData));
    if(len>0)
    {
        int data_len=9;
        unsigned char rcv_buf[data_len];
        len = SerialRecv(fd,rcv_buf,data_len,BAUDRATE);
        pro_ampere = (int)rcv_buf[6]+(int)(rcv_buf[7]<<8);
    }
    sleep_time(10);
    TxData[5] = 0x62;//控制表索引地址 过温保护
    TxData[7] = TxData[2]+TxData[3]+TxData[4]+TxData[5]+TxData[6];
    len = SerialSend(fd,TxData,sizeof( TxData));
    if(len>0)
    {
        int data_len=9;
        unsigned char rcv_buf[data_len];
        len = SerialRecv(fd,rcv_buf,data_len,BAUDRATE);
        pro_temp = (int)rcv_buf[6]+(int)(rcv_buf[7]<<8);
    }
    sleep_time(10);
    TxData[5] = 0x64;//控制表索引地址 回温启动
    TxData[7] = TxData[2]+TxData[3]+TxData[4]+TxData[5]+TxData[6];
    len = SerialSend(fd,TxData,sizeof( TxData));
    if(len>0)
    {
        int data_len=9;
        unsigned char rcv_buf[data_len];
        len = SerialRecv(fd,rcv_buf,data_len,BAUDRATE);
        work_temp = (int)rcv_buf[6]+(int)(rcv_buf[7]<<8);
    }
    return len;
}

int SetPara(int fd,int index,int pro_ampere,int pro_temp,int work_temp){
    unsigned char TxData[9];
    TxData[0] = 0x55;//帧头
    TxData[1] = 0xAA;//帧头
    TxData[2] = 0x04;//帧长度
    TxData[3] = (unsigned char)index;//ID
    TxData[4] = 0x03;//指令类型 写指令

    TxData[5] = 0x20;//控制表索引地址 过流保护
    TxData[6] = (unsigned char)pro_ampere;//低位
    TxData[7] = (unsigned char)(pro_ampere>>8);//高位
    TxData[8] = TxData[2]+TxData[3]+TxData[4]+TxData[5]+TxData[6]+TxData[7];
    int len = SerialSend(fd,TxData,sizeof( TxData));

    sleep_time(10);
    TxData[5] = 0x62;//控制表索引地址 过温保护
    TxData[6] = (unsigned char)(pro_temp);//低位
    TxData[7] = (unsigned char)(pro_temp>>8);//高位
    TxData[8] = TxData[2]+TxData[3]+TxData[4]+TxData[5]+TxData[6]+TxData[7];
    len = SerialSend(fd,TxData,sizeof( TxData));

    sleep_time(10);
    TxData[5] = 0x64;//控制表索引地址 回温启动
    TxData[6] = (unsigned char)(work_temp);//低位
    TxData[7] = (unsigned char)(work_temp>>8);//高位
    TxData[8] = TxData[2]+TxData[3]+TxData[4]+TxData[5]+TxData[6]+TxData[7];
    len = SerialSend(fd,TxData,sizeof( TxData));

    //烧录
    sleep_time(10);
    unsigned char TxData2[8];
    TxData2[0] = 0x55;//帧头
    TxData2[1] = 0xAA;//帧头
    TxData2[2] = 0x03;//帧长度
    TxData2[3] = (unsigned char)index;//ID
    TxData2[4] = 0x04;//指令类型
    TxData2[5] = 0x00;//控制表索引地址
    TxData2[6] = 0x20;//参数装订
    TxData2[7] = TxData2[2]+TxData2[3]+TxData2[4]+TxData2[5]+TxData2[6];
    len = SerialSend(fd,TxData2,sizeof( TxData2));
    if(len>0)
    {
        int data_len=22;
        unsigned char rcv_buf[data_len];
        len = SerialRecv(fd,rcv_buf,data_len,BAUDRATE);
    }
    return len;
}

/*
 * 停止
*/
int StopWorking(int fd,int index,int& pos_real,int& temp,int& ampere,int& err){
    unsigned char TxData[8];
    TxData[0] = 0x55;//帧头
    TxData[1] = 0xAA;//帧头
    TxData[2] = 0x03;//帧长度
    TxData[3] = (unsigned char)index;//ID
    TxData[4] = 0x04;//指令类型
    TxData[5] = 0x00;//控制表索引地址
    TxData[6] = 0x23;//急停
    TxData[7] = TxData[2]+TxData[3]+TxData[4]+TxData[5]+TxData[6];
    int len = SerialSend(fd,TxData,sizeof( TxData));
    if(len>0)
    {
        int data_len=22;
        unsigned char rcv_buf[data_len];
        len = SerialRecv(fd,rcv_buf,data_len,BAUDRATE);
        pos_real = (int)(rcv_buf[9])+(int)(rcv_buf[10]<<8);
        temp = (int)rcv_buf[11];
        ampere = (int)rcv_buf[12]+(int)(rcv_buf[13]<<8);
        err = (int)rcv_buf[15];
    }
    return len;
}

/*
 * 工作
*/
int StartWorking(int fd,int index,int& pos_real,int& temp,int& ampere,int& err){
    unsigned char TxData[8];
    TxData[0] = 0x55;//帧头
    TxData[1] = 0xAA;//帧头
    TxData[2] = 0x03;//帧长度
    TxData[3] = (unsigned char)index;//ID
    TxData[4] = 0x04;//指令类型
    TxData[5] = 0x00;//控制表索引地址
    TxData[6] = 0x04;//工作
    TxData[7] = TxData[2]+TxData[3]+TxData[4]+TxData[5]+TxData[6];
    int len = SerialSend(fd,TxData,sizeof( TxData));
    if(len>0){
        int data_len=22;
        unsigned char rcv_buf[data_len];
        len = SerialRecv(fd,rcv_buf,data_len,BAUDRATE);
        pos_real = (int)(rcv_buf[9])+(int)(rcv_buf[10]<<8);
        temp = (int)rcv_buf[11];
        ampere = (int)rcv_buf[12]+(int)(rcv_buf[13]<<8);
        err = (int)rcv_buf[15];

    }
    return len;
}

/*
 * 消除异常
*/
int ClearErr(int fd,int index,int& pos_real,int& temp,int& ampere,int& err){
    unsigned char TxData[8];
    TxData[0] = 0x55;//帧头
    TxData[1] = 0xAA;//帧头
    TxData[2] = 0x03;//帧长度
    TxData[3] = (unsigned char)index;//ID
    TxData[4] = 0x04;//指令类型
    TxData[5] = 0x00;//控制表索引地址
    TxData[6] = 0x1E;//故障清除
    TxData[7] = TxData[2]+TxData[3]+TxData[4]+TxData[5]+TxData[6];
    int len = SerialSend(fd,TxData,sizeof( TxData));
    if(len>0){
        int data_len=22;
        unsigned char rcv_buf[data_len];
        len = SerialRecv(fd,rcv_buf,data_len,BAUDRATE);
        pos_real = (int)(rcv_buf[9])+(int)(rcv_buf[10]<<8);
        temp = (int)rcv_buf[11];
        ampere = (int)rcv_buf[12]+(int)(rcv_buf[13]<<8);
        err = (int)rcv_buf[15];
    }
    return len;
}

/*
 *睡眠
*/
void sleep_time(unsigned int msec){
    msec = 1000*msec;
    do{
        msec = 1000*usleep(msec);
    }while(msec > 0);

}

void enable_hand(int fd){
    sleep_time(1);
    int pos_real1,temp1,ampere1,err1;
    int pos_real2,temp2,ampere2,err2;
    int pos_real3,temp3,ampere3,err3;
    int pos_real4,temp4,ampere4,err4;
    int pos_real5,temp5,ampere5,err5;
    int pos_real6,temp6,ampere6,err6;
    StartWorking(fd,1,pos_real1,temp1,ampere1,err1);
    StartWorking(fd,2,pos_real2,temp2,ampere2,err2);
    StartWorking(fd,3,pos_real3,temp3,ampere3,err3);
    StartWorking(fd,4,pos_real4,temp4,ampere4,err4);
    StartWorking(fd,5,pos_real5,temp5,ampere5,err5);
    StartWorking(fd,6,pos_real6,temp6,ampere6,err6);
}
void unable_hand(int fd){
    sleep_time(1);
    int pos_real1,temp1,ampere1,err1;
    int pos_real2,temp2,ampere2,err2;
    int pos_real3,temp3,ampere3,err3;
    int pos_real4,temp4,ampere4,err4;
    int pos_real5,temp5,ampere5,err5;
    int pos_real6,temp6,ampere6,err6;
    StopWorking(fd,1,pos_real1,temp1,ampere1,err1);
    StopWorking(fd,2,pos_real2,temp2,ampere2,err2);
    StopWorking(fd,3,pos_real3,temp3,ampere3,err3);
    StopWorking(fd,4,pos_real4,temp4,ampere4,err4);
    StopWorking(fd,5,pos_real5,temp5,ampere5,err5);
    StopWorking(fd,6,pos_real6,temp6,ampere6,err6); 
}
void clearErr_hand(int fd){
    sleep_time(1);
    int pos_real1,temp1,ampere1,err1;
    int pos_real2,temp2,ampere2,err2;
    int pos_real3,temp3,ampere3,err3;
    int pos_real4,temp4,ampere4,err4;
    int pos_real5,temp5,ampere5,err5;
    int pos_real6,temp6,ampere6,err6;
    ClearErr(fd,1,pos_real1,temp1,ampere1,err1);
    ClearErr(fd,2,pos_real2,temp2,ampere2,err2);
    ClearErr(fd,3,pos_real3,temp3,ampere3,err3);
    ClearErr(fd,4,pos_real4,temp4,ampere4,err4);
    ClearErr(fd,5,pos_real5,temp5,ampere5,err5);
    ClearErr(fd,6,pos_real6,temp6,ampere6,err6);
    sleep_time(1);
    enable_hand(fd);
}



