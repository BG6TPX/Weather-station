//****************************************************************//
//	       AM系列读单总线使用范例 
//单片机 ：AT89S52 或 STC89C52RC 
// 功能  ：串口发送温湿度数据  波特率 9600 
// 晶振  ：12M (用户系统时钟如不是12M 请更改相关宏定义及注释的延时时间)
// 编译环境:  Keil3
// 公司  ：奥松电子    
//****************************************************************//

#include "reg52.h"
#include <intrins.h>

//用户根据自己的晶振修改相应值
#define FOSC  11059200
#define BAUD  9600 

//读传感器 端口位定义，可修改
sbit Sensor_SDA = P0^0;
sbit Sensor_SCL = P0^1;

// 变量定义
unsigned char Sensor_Data[5]={0x00,0x00,0x00,0x00,0x00};
unsigned char Sensor_Check;		  //校验和

unsigned char Sensor_AnswerFlag;  //收到起始标志位
unsigned char Sensor_ErrorFlag;   //读取传感器错误标志
unsigned int  Sys_CNT;
unsigned int  Tmp;

unsigned char *String;

//字符串定义
#define S_Temp "Temp:"
#define S_RH   "RH:"
#define S_CRCT "Check: True"
#define S_CRCF "Check: Wrong"
#define S_Data "Data: "
#define S_NotS "Sensor Not Connected"

/********************************************\
|* 功能： 延时	晶振为12M时				    *|
|*  t = 1 为 20us  然后成倍增加10us左右		*|
\********************************************/
void Delay_N10us(unsigned char t)
{
    while(t--)
   {
   	 _nop_();
   }
}
/********************************************\
|* 功能： 延时	晶振为12M时					*|
|* 延时大约 1ms			    				*|
\********************************************/ 
void Delay_N1ms(unsigned int t)
{
  unsigned int i;
  unsigned int j;
  for(j=t;j>0;j--)
     for(i=124;i>0;i--);  //延时大约 1ms
}
/********************************************\
|* 功能： 初始化串口					    *|
\********************************************/
void InitUART(void)
  {
    unsigned int iTmpBaud;
    unsigned long lTmpBaud;
    iTmpBaud = 0;
    //首先选定定时器2作为波特率发生器,16位定时器,自动装载
    SCON = 0x50;	//SM0 SM1 SM2 REN TB8 RB8 TI RI		//0   1   0   1   0   0   0  0	
  	PCON = 0x00;	//PCON的地址是87H,这里SMOD =0

  	T2CON = 0x30;	//TF2 EXF2 RCLK TCLK EXEN2 TR2 C(/T2) CP(/RL2) //0 0 1 1 0 0 0 0 
  	T2MOD = 0x00;	// /	/	/	/		/	/	T2OE	DCEN   //0 0 0 0 0 0 0 0

  	lTmpBaud = FOSC/BAUD;
  	lTmpBaud /= 32;						//12T-mode
  	iTmpBaud = lTmpBaud & 0xFFFF;		
  	iTmpBaud = 65536 - iTmpBaud;
  	RCAP2H = (iTmpBaud>>8) & 0x0FF;
  	RCAP2L = iTmpBaud & 0x0FF;

  	RI = 0;			//清除接收中断标志
  	REN = 1;		//允许串行接收
  	ES = 1;			//允许串行中断
  	TR2 = 1;		//启动定时器1

  	EA=1;//开总中断
  }  

/********************************************\
|* 功能： 串口发送函数 			            *|
\********************************************/
void UARTSend(char UCHAR)
  {
    SBUF=UCHAR;
    while(TI==0);
    TI=0;
  }
/********************************************\
|* 功能： 串口中断函数 			            *|
\********************************************/
void UARTRead(void) interrupt 4
  {
    char temp; 
    if(RI)
    {
      RI=0;
      temp = SBUF;
    }
  }
 /********************************************\
|* 功能： 串口发送子函数 			        *|
\********************************************/
void UART_PutString(unsigned char *buf)
  {
	while(*buf)
      UARTSend(*buf++);
  }
void UART_PutStringAndNum(unsigned char *buf ,unsigned int num)
  {
	unsigned char a[3],i;
	a[3] = '0'+num%10;
	a[2] = '.';
	a[1] = '0'+num/10%10;
	a[0] = '0'+num/100%10;
	while(*buf)
      UARTSend(*buf++);
	UARTSend(' ');
	 for(i=0;i<4;i++)
	{
		UARTSend(a[i]);
	} 
  }
void UART_PutStringAnd_Data(unsigned char *buf ,unsigned char *bufdata)
  {
	unsigned char a[2],i,j;
	while(*buf)
      UARTSend(*buf++);
	UARTSend(' ');
	for(i=0;i<5;i++)
	{
		a[0] = bufdata[i]/16; 
		a[1] = bufdata[i]%16;
		for(j=0;j<2;j++)
		{
		  if(a[j]>9)
		  {
		    a[j] = (a[j]-10)+'A';
		  }
		  else
		  {
		    a[j] = a[j]+'0';
		  }
		  UARTSend(a[j]);
		}
		UARTSend(' ');
	} 
  }
/********************************************\
|* 功能： 串口发送传感器数据函数 			*|
\********************************************/
void UARTSend_Nbyte(void)
  {
	   if(Sensor_AnswerFlag == 1)
	   {
	      Sensor_Check = Sensor_Data[0]+Sensor_Data[1]+Sensor_Data[2]+Sensor_Data[3];
		  //校验成功
		  if(Sensor_Check ==Sensor_Data[4])	
		  {
		     String = S_RH;//"RH:";	   
	         Tmp = Sensor_Data[0]*256+Sensor_Data[1];	   
	         UART_PutStringAndNum(String,Tmp); 
	   		 UARTSend(' ');
	         UARTSend(' ');
	         
			 String = S_Temp;// "Temp:";	   
	       	 Tmp = Sensor_Data[2]*256+Sensor_Data[3];	   
	         UART_PutStringAndNum(String,Tmp);
		   	 UARTSend(' ');
	         UARTSend(' ');

		     String = S_CRCT;//"Check: True";
		     UART_PutString(String);
		  }else	//校验失败 送上读到数据
		  {
             String = S_Data;//"Data: ";
		     UART_PutStringAnd_Data(String,Sensor_Data);
			 UARTSend(' ');	
			 UARTSend(' ');
			 String = S_CRCF;//"Check: Wrong";
		     UART_PutString(String); 
		  }
		}// 传感器未连接
		else
		{
		   String = S_NotS; //"Sensor Not Connected";
 		   UART_PutString(String);
		}	    
	    UARTSend(0x0A); 

  }  
void Clear_Data (void)
  {
	int i;
	for(i=0;i<5;i++)
	{
	   Sensor_Data[i] = 0x00;
	 }//接收数据清零
  }


/********************************************\
|* 功能： 读传感器发送的单个字节	        *|
\********************************************/
unsigned char Read_SensorData(void)
  {
	unsigned char i,cnt;
	unsigned char buffer,tmp;
	buffer = 0;
	for(i=0;i<8;i++)
	{
		cnt=0;
		while(!Sensor_SDA)	//检测上次低电平是否结束
		{
		  if(++cnt >= 300)
		   {
			  break;
		   }
		}
		//延时Min=26us Max50us 跳过数据"0" 的高电平
		Delay_N10us(2);	 //延时30us   
		
		//判断传感器发送数据位
		tmp =0;
		if(Sensor_SDA)	 
		{
		  tmp = 1;
		}  
		cnt =0;
		while(Sensor_SDA)		//等待高电平 结束
		{
		   	if(++cnt >= 200)
			{
			  break;
			}
		}
		buffer <<=1;
		buffer |= tmp;	
	}
	return buffer;
  }

/********************************************\
|* 功能： 读传感器              	        *|
\********************************************/
unsigned char Read_Sensor(void)
  {
	unsigned char i;
	//主机拉低(Min=800US Max=20Ms) 
    Sensor_SDA = 0;
	Delay_N1ms(2);  //延时2Ms
	  
	//释放总线 延时(Min=30us Max=50us)
	Sensor_SDA = 1; 	
	Delay_N10us(1);//延时30us
	//主机设为输入 判断传感器响应信号 
	Sensor_SDA = 1; 
     	  
	Sensor_AnswerFlag = 0;  // 传感器响应标志	 

	//判断从机是否有低电平响应信号 如不响应则跳出，响应则向下运行	  
	if(Sensor_SDA ==0)
	{
	   Sensor_AnswerFlag = 1;//收到起始信号
	   Sys_CNT = 0;
	   //判断从机是否发出 80us 的低电平响应信号是否结束	 
	   while((!Sensor_SDA))
	   {
	     if(++Sys_CNT>300) //防止进入死循环
		 {
		   Sensor_ErrorFlag = 1;
		   return 0;
		  } 
	    }
	    Sys_CNT = 0;
	    //判断从机是否发出 80us 的高电平，如发出则进入数据接收状态
	    while((Sensor_SDA))
	    {
	       if(++Sys_CNT>300) //防止进入死循环
		   {
		     Sensor_ErrorFlag = 1;
		     return 0;
		   } 
	    } 		 
	    // 数据接收	传感器共发送40位数据 
	    // 即5个字节 高位先送  5个字节分别为湿度高位 湿度低位 温度高位 温度低位 校验和
	    // 校验和为：湿度高位+湿度低位+温度高位+温度低位
	    for(i=0;i<5;i++)
	    {
	      Sensor_Data[i] = Read_SensorData();
	    }
	  }
	  else
	  {
	    Sensor_AnswerFlag = 0;	  // 未收到传感器响应	
	  }
	  return 1;
  }    

void main(void)
{
  Sensor_SCL = 0;
  InitUART();  //初始串口发送函数
  while(1)
  {
	Clear_Data();         // 清除收到数据
	Read_Sensor();		  // 读取传感器数据
	UARTSend_Nbyte();	  // 串口发送读到传感器数据
    Delay_N1ms(2000);	  // 延时 2S(两次读取间隔至少2S) 
  }
}
