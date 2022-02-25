//****************************************************************//
//	       AM系列读IIC使用范例 
//单片机 ：AT89S52 或 STC89C52RC 
// 功能  ：串口发送温湿度数据  波特率 9600 
// 晶振  ：12M (用户系统时钟如不是12M 请更改相关宏定义及注释的延时时间)
// 编译环境:  Keil3
// 公司  ：奥松电子    
//****************************************************************//

#include "reg52.h"
#include <intrins.h>

#define USE_T2 
#define FOSC  11059200
#define BAUD  9600

//端口位定义，可修改
sbit SDA=P0^0;
sbit SCL=P0^1;

//内部数据定义
#define IIC_Add 0xB8    //器件地址
#define IIC_RX_Length 15

unsigned char IIC_TX_Buffer[]={0x03,0x00,0x04}; //读温湿度命令（无CRC校验）
unsigned char IIC_RX_Buffer[IIC_RX_Length] = {0x00};//读回的温湿度

unsigned char Uart_RX_Buffer[30] = {0x00};
unsigned char *String;
unsigned char WR_Flag;

//字符串定义
#define S_Function  "Function: 03 04"
#define S_Temp "Temp:"
#define S_RH   "RH:"
#define S_CRCT "CRC: True"
#define S_CRCF "CRC: Wrong"
#define S_Data "Data: "
#define S_NotS "Sensor Not Connected"

void Ack(void);
void NoAck(void);
 
void delay10us(void) //这个延时函数 要大于5US以上
{
  _nop_(); _nop_(); _nop_(); 
  _nop_(); _nop_(); _nop_(); 
}
 
void delay1ms(unsigned int t)
{
  unsigned int i;
  unsigned int j;
  for(j=t;j>0;j--)
   for(i=124;i>0;i--);  
}

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

	//fosc = 22.1184M,6T: 144,设置波特率
	//(RCAP2H,RCAP2L) = 65536- fosc/(n*Baud)。n:32(12T-mode),16:(6T-mode)
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

//串口发送
void UARTSend(char UCHAR)
{
   SBUF=UCHAR;
  while(TI==0);
  TI=0;
}

void UARTRead(void) interrupt 4
{
  char temp;
  if(RI)
  {
    RI=0;
    temp = SBUF;
  }
}

//**********************************************
//送起始位 sda=1->0
void I2C_Start()
{
  SDA=1;
  SCL=1;
  delay10us();
  SDA=0;
  delay10us();
  SCL=0; 
}
//************************************************
//送停止位 sda=0->1
void I2C_Stop()
{
   SDA=0;
   delay10us();
   SCL=1;
   delay10us();
   SDA=1;
}
//************************************************
//主应答(包含ack:sda=0和no_ack:sda=0)
void Ack(void)
{  //设置SDA 口为输出
   SDA=0;
   SCL=0;
   delay10us();
   SCL=1;
   delay10us();	
   SCL=0;
   SDA=1;
}

void NoAck(void)
{  //设置SDA 口为输出
   SDA=1;  
   SCL=0;
   delay10us();
   SCL=1;
   delay10us();
   SDA=1;
   SCL=0;
}

// 检测 SDA是否回ACK
bit Test_Ack()
{  //设置SDA 口为输入
   bit ACK_Flag=0;
   SCL=0;
   SDA=1;    
   delay10us();
   SCL=1;
   delay10us();
   if(SDA==0)
     ACK_Flag = 1;
   else 
     ACK_Flag = 0;
   SCL=0;
   return ACK_Flag;
}

//*************************************************
//字节发送程序
//发送c(可以是数据也可是地址)，送完后接收从应答
//不考虑从应答位
void SendData(unsigned char buffer)
{
   unsigned char BitCnt=8;//一字节8位
   //设置SDA 口为输出
   do
   {
 	  SCL=0;
	  delay10us();
      if((buffer&0x80)==0) //判断最高位是0还是1
        SDA=0;
      else
        SDA=1;
      SCL=1;
	  delay10us();
      buffer=buffer<<1;//将buffer中的数据左移一位
      BitCnt--;
   }
   while(BitCnt);
   SCL=0;        
}
//**************************************************
//字节接收程序
//接收器件传来的数据，此程序应配合|主应答函数|i2c_ack_main()使用
//return: uchar型1字节
unsigned char ReceiveData()
{
  unsigned char BitCnt=8,IIC_RX_Data=0;
  unsigned char temp=0;
  SDA=1;           //读入数据 设置SDA 口为输入
  do
  {
     SCL=0;
	 delay10us();  
	 IIC_RX_Data=_crol_(IIC_RX_Data,1);   //数据左移一位
	 BitCnt--;	  
	 SCL=1;
	 delay10us();
     if(SDA==1)
       IIC_RX_Data = IIC_RX_Data|0x01;  //低位置1
     else
       IIC_RX_Data = IIC_RX_Data&0x0fe; //低位清0	    
   }
   while(BitCnt);
   SCL=0;
   return IIC_RX_Data;
}
//***************************************************
bit WriteNByte(unsigned char sla,unsigned char *s,unsigned char n)
{
   unsigned char i;
   
   I2C_Start();  //启动I2C
   SendData(sla);//发送器件地址
   if(!Test_Ack())
   {	
      WR_Flag = 1;
	  return(0);
   }
   for(i=0;i<n;i++)//写入8字节数据
   {
      SendData(*(s+i));
	  if(!Test_Ack())
	  {
	    WR_Flag = 1;
		return(0);
	  }
   }
   I2C_Stop();
   return(1);
}
bit ReadNByte(unsigned char Sal, unsigned char *p,unsigned char n)
{
  unsigned char i;
  I2C_Start();    // 启动I2C
  SendData((Sal)| 0x01); //发送器件地址
  if(!Test_Ack())
  {
  	WR_Flag = 1;
	return(0);
  }
  delay10us();  
  delay10us();
  delay10us(); // 延时时间必须大于30us 只要大于 30us 以上的值都可以 但是最好不要太长 ，测试时，试过25MS都OK！ 
        
  for(i=0;i<n-1;i++)  //读取字节数据
  {
     *(p+i)=ReceiveData(); //读取数据
     Ack(); 
  }
  *(p+n-1)=ReceiveData();        
  NoAck();
  I2C_Stop(); 
  return(1);	 
}
///计算CRC校验码	
unsigned int CRC16(unsigned char *ptr, unsigned char len)
{
   unsigned int crc=0xffff;
   unsigned char i;
   while(len--)
   {
       crc ^=*ptr++;
       for(i=0;i<8;i++)
	   {
	       if(crc & 0x1)
		   {
		      crc>>=1;
			  crc^=0xa001;
		   }
		   else
		   {
		      crc>>=1;
		   }
	   }
   }
   return crc;
}
///检测CRC校验码是否正确
unsigned char CheckCRC(unsigned char *ptr,unsigned char len)
{
  unsigned int crc;
	crc=(unsigned int)CRC16(ptr,len-2);
	if(ptr[len-1]==(crc>>8) && ptr[len-2]==(crc & 0x00ff))
	{
	    return 0xff;
	}
	else
	{
	   return 0x0;
	}
}
void Waken(void)
   {
    I2C_Start();       // 启动I2C
    SendData(IIC_Add); // 发送器件地址
    Test_Ack();	       // 唤醒指令时 传感器不会回ACK 但是第一定要发检测ACK的时钟 否则会出错
    delay1ms(2);       // 至少延时1个Ms	说明书里，有个最大值 ，实际当中 你只要大于1MS
    I2C_Stop();	
   }

void UART_PutString(unsigned char *buf )
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
	for(i=0;i<8;i++)
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

void UARTSend_Nbyte(void)
      {
	   int Tmp; 
	   if(WR_Flag == 0)
	   {
		 if(CheckCRC(IIC_RX_Buffer,8))
		 {
		   String = S_Function;  // "Function: 03 04"
	       UART_PutString(String);
 	       UARTSend(' ');
	       UARTSend(' ');

	       String = S_RH;//"RH:"	   
	       Tmp = IIC_RX_Buffer[2]*256+IIC_RX_Buffer[3];	   
	       UART_PutStringAndNum(String,Tmp); 
	   
	       UARTSend(' ');
	       UARTSend(' ');
	       String = S_Temp; //"Temp:"
		   	          
		   Tmp = IIC_RX_Buffer[4]*256+IIC_RX_Buffer[5];	   
	       UART_PutStringAndNum(String,Tmp);
		   
		   UARTSend(' ');
	       UARTSend(' ');

		   String = S_CRCT;//"CRC: True";
		   UART_PutString(String);

		 }else
		 {
			String = S_Data;//"Data: ";

			UART_PutStringAnd_Data(String,IIC_RX_Buffer);
			  UARTSend(' ');
			  UARTSend(' ');
			String = S_CRCF;//"CRC: Wrong";
		      UART_PutString(String); 
		 }
	    }
		else
		{
		   String = S_NotS;//"Sensor Not Connected";
 		   UART_PutString(String);
		}	    
	   UARTSend(0x0A); 			  

       }  
void Clear_Data (void)
     {
	    int i;
	    for(i=0;i<IIC_RX_Length;i++)
	     {
	     IIC_RX_Buffer[i] = 0x00;
	     }//接收数据清零
	  }

void main(void)
{
  SCL = 1;
  SDA = 1; //上电时保证两总线为高
  InitUART();
  Clear_Data();
  while(1)
  {
	Clear_Data(); // 清除收到数据
	WR_Flag = 0;
	Waken();	  // 唤醒传感器
    //发送读指令
    WriteNByte(IIC_Add,IIC_TX_Buffer,3); 
    //发送读取或写数据命令后，至少等待2MS（给探头返回数据作时间准备）
	delay1ms(2);    
	//读返回数据
    ReadNByte(IIC_Add,IIC_RX_Buffer,8);

	SCL = 1; SDA = 1;	//确认释放总线
  	//通过串口向上发送传感器数据
	UARTSend_Nbyte();

    delay1ms(2000);	  //延时 2S	(两次读取间隔至少2S)
  }
}






