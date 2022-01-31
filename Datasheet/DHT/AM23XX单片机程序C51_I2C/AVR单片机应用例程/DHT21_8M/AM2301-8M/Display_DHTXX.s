	.module Display_DHTXX.c
	.area text(rom, con, rel)
	.dbfile C:\DOCUME~1\Owner\桌面\C\给客户调试板\DHT21_8M\DHT21_8M\Display_DHTXX.c
	.dbfunc e port_init _port_init fV
	.even
_port_init::
	.dbline -1
	.dbline 72
; //ICC-AVR application builder : 2008-4-28 9:22:37
; // Target : M8
; // Crystal: 8.0000Mhz
; /*************************************************************************
; 程序开发软件：ICC-AVR 
; 功能：波特率9600 串口发送温湿度数据  
; 数据格式：湿度高8位 湿度高8位  温度低8位 温度低8位  校验和     
;    
;           校验和= 湿度高8位+湿度低8位+温度高8位+温度低8位
; 		  
; 公司：DHT21
; 芯片：M8
; 晶振：外部 8Mhz
; 创建人： 
; 创建时间：09.07.03
; 修改人：
; 修改时间：
; 修改原因：
; **************************************************** **********************/
; 
; 
; #include <iom8v.h>
; #include <macros.h>
; typedef unsigned char  U8;       /* defined for unsigned 8-bits integer variable 	  无符号8位整型变量  */
; typedef signed   char  S8;       /* defined for signed 8-bits integer variable		  有符号8位整型变量  */
; typedef unsigned int   U16;      /* defined for unsigned 16-bits integer variable 	  无符号16位整型变量 */
; typedef signed   int   S16;      /* defined for signed 16-bits integer variable 	  有符号16位整型变量 */
; typedef unsigned long  U32;      /* defined for unsigned 32-bits integer variable 	  无符号32位整型变量 */
; typedef signed   long  S32;      /* defined for signed 32-bits integer variable 	  有符号32位整型变量 */
; typedef float          F32;      /* single precision floating point variable (32bits) 单精度浮点数（32位长度） */
; typedef double         F64;      /* double precision floating point variable (64bits) 双精度浮点数（64位长度） */
; 
; 
; /**************************************************************************
;                     I/O空间定义
; ***************************************************************************/
; // B0_IN	DDRB &= ~(1 << PB0)		/*设置输入*/
; // B0_OUT	DDRB |= (1 << PB0)		/*设置输出*/
; // B0_CLR	PORTB &= ~(1 << PB0)	/*置低电平*/
; // B0_SET	PORTB |= (1 << PB0)		/*置高电平*/
; // B0_R	    PINB & (1 <<PB0)		/*读了电平*/
; 
;  #define    C4_IN	  DDRC &= ~(1 << PC4)		/*设置输入*/
;  #define    C4_OUT	  DDRC |= (1 << PC4)		/*设置输出*/
;  #define    C4_CLR	  PORTC &= ~(1 << PC4)	    /*置低电平*/
;  #define    C4_SET	  PORTC |= (1 << PC4)		/*置高电平*/
;  #define    C4_R	    PINC & (1 <<PC4)		/*读了电平*/
; 
; /**************************************************************************
;                     通讯I/O空间定义
; ***************************************************************************/
;  #define       COM_IN          C4_IN  
;  #define       COM_OUT         C4_OUT
;  #define       COM_CLR         C4_CLR
;  #define       COM_SET         C4_SET
;  #define       COM_R           C4_R
;  
; /**************************************************************************
;                     全局变量定义区
; ***************************************************************************/
;       U8       i;
;       U8    U8FLAG,k;
;       U8    U8count,U8temp;
;       U8    U8T_data_H,U8T_data_L,U8RH_data_H,U8RH_data_L,U8checkdata;
;       U8    U8T_data_H_temp,U8T_data_L_temp,U8RH_data_H_temp,U8RH_data_L_temp,U8checkdata_temp;
;       U8    U8comdata;
;       U16   ReceiveHighByte;
;       U16   ReceiveLowByte;
; /**************************************************************************
; ***************************************************************************/
; void port_init(void)
; {
	.dbline 73
;  PORTB = 0x00;
	clr R2
	out 0x18,R2
	.dbline 74
;  DDRB  = 0xFF;
	ldi R24,255
	out 0x17,R24
	.dbline 75
;  PORTC = 0x03;                    
	ldi R24,3
	out 0x15,R24
	.dbline 76
;  DDRC  = 0x0C;
	ldi R24,12
	out 0x14,R24
	.dbline 77
;  PORTD = 0x00;
	out 0x12,R2
	.dbline 78
;  DDRD  = 0xF0;
	ldi R24,240
	out 0x11,R24
	.dbline -2
L1:
	.dbline 0 ; func end
	ret
	.dbend
	.dbfunc e watchdog_init _watchdog_init fV
	.even
_watchdog_init::
	.dbline -1
	.dbline 82
; }
; 
; void watchdog_init(void)
; {
	.dbline -2
L2:
	.dbline 0 ; func end
	ret
	.dbend
	.dbfunc e Delay _Delay fV
;              i -> R16,R17
	.even
_Delay::
	.dbline -1
	.dbline 89
;   //WDR();        //this prevents a timout on enabling
;   //WDTCR = 0x0F; //WATCHDOG ENABLED - dont forget to issue WDRs
; }
; 
; 
; void Delay(unsigned int i)
; {
L4:
	.dbline 90
L5:
	.dbline 90
; 	while(i--);
	movw R2,R16
	subi R16,1
	sbci R17,0
	tst R2
	brne L4
	tst R3
	brne L4
X0:
	.dbline -2
L3:
	.dbline 0 ; func end
	ret
	.dbsym r i 16 i
	.dbend
	.dbfunc e DelaymS _DelaymS fV
;              i -> R20,R21
;            dly -> R16,R17
	.even
_DelaymS::
	rcall push_gset1
	.dbline -1
	.dbline 94
; }
; 
; void DelaymS (U16 dly)                   
; {
	.dbline 96
; 	U16 i;
; 	for ( ; dly>0; dly--)
	rjmp L11
L8:
	.dbline 97
; 	{
	.dbline 98
; 	    WDR();
	wdr
	.dbline 99
	clr R20
	clr R21
	rjmp L15
L12:
	.dbline 99
L13:
	.dbline 99
	subi R20,255  ; offset = 1
	sbci R21,255
L15:
	.dbline 99
	cpi R20,95
	ldi R30,4
	cpc R21,R30
	brlo L12
	.dbline 100
L9:
	.dbline 96
	subi R16,1
	sbci R17,0
L11:
	.dbline 96
	cpi R16,0
	cpc R16,R17
	brne L8
X1:
	.dbline -2
L7:
	rcall pop_gset1
	.dbline 0 ; func end
	ret
	.dbsym r i 20 i
	.dbsym r dly 16 i
	.dbend
	.dbfunc e Delay_10us _Delay_10us fV
;              i -> R16
	.even
_Delay_10us::
	.dbline -1
	.dbline 105
; 		for (i=0; i<1119; i++);        
; 	}
; }
; 
; 
;        void  Delay_10us(void)
;       {
	.dbline 107
;         U8 i;                     
;         i++;i++;i++;i++;i++;i++;i++;i++;i++;
	inc R16
	.dbline 107
	inc R16
	.dbline 107
	inc R16
	.dbline 107
	inc R16
	.dbline 107
	inc R16
	.dbline 107
	inc R16
	.dbline 107
	inc R16
	.dbline 107
	inc R16
	.dbline 107
	inc R16
	.dbline 108
; 		i++;i++;i++;i++;i++;i++;i++;i++;i++;
	inc R16
	.dbline 108
	inc R16
	.dbline 108
	inc R16
	.dbline 108
	inc R16
	.dbline 108
	inc R16
	.dbline 108
	inc R16
	.dbline 108
	inc R16
	.dbline 108
	inc R16
	.dbline 108
	inc R16
	.dbline 109
; 		i++;i++;i++;i++;i++;i++;i++;i++;i++;
	inc R16
	.dbline 109
	inc R16
	.dbline 109
	inc R16
	.dbline 109
	inc R16
	.dbline 109
	inc R16
	.dbline 109
	inc R16
	.dbline 109
	inc R16
	.dbline 109
	inc R16
	.dbline 109
	inc R16
	.dbline 110
; 		i++;i++;i++;i++;i++;i++;i++;i++;i++;
	inc R16
	.dbline 110
	inc R16
	.dbline 110
	inc R16
	.dbline 110
	inc R16
	.dbline 110
	inc R16
	.dbline 110
	inc R16
	.dbline 110
	inc R16
	.dbline 110
	inc R16
	.dbline 110
	inc R16
	.dbline 111
; 		i++;i++;i++;i++;i++;i++;i++;i++;i++;
	inc R16
	.dbline 111
	inc R16
	.dbline 111
	inc R16
	.dbline 111
	inc R16
	.dbline 111
	inc R16
	.dbline 111
	inc R16
	.dbline 111
	inc R16
	.dbline 111
	inc R16
	.dbline 111
	inc R16
	.dbline 112
; 		i++;i++;i++;i++;i++;i++;i++;i++;i++;
	inc R16
	.dbline 112
	inc R16
	.dbline 112
	inc R16
	.dbline 112
	inc R16
	.dbline 112
	inc R16
	.dbline 112
	inc R16
	.dbline 112
	inc R16
	.dbline 112
	inc R16
	.dbline 112
	inc R16
	.dbline 113
; 		i++;i++;i++;i++;i++;i++;i++;i++;i++;
	inc R16
	.dbline 113
	inc R16
	.dbline 113
	inc R16
	.dbline 113
	inc R16
	.dbline 113
	inc R16
	.dbline 113
	inc R16
	.dbline 113
	inc R16
	.dbline 113
	inc R16
	.dbline 113
	inc R16
	.dbline 114
; 		i++;i++;i++;i++;i++;i++;i++;
	inc R16
	.dbline 114
	inc R16
	.dbline 114
	inc R16
	.dbline 114
	inc R16
	.dbline 114
	inc R16
	.dbline 114
	inc R16
	.dbline 114
	inc R16
	.dbline -2
L16:
	.dbline 0 ; func end
	ret
	.dbsym r i 16 c
	.dbend
	.dbfunc e COM _COM fV
;              i -> R20
	.even
_COM::
	rcall push_gset1
	.dbline -1
	.dbline 117
;        }
;         void  COM(void)
;       {
	.dbline 120
;      
; 	        U8 i;
;        for(i=0;i<8;i++)	   
	clr R20
	rjmp L21
L18:
	.dbline 121
; 	    {
	.dbline 123
; 		
; 	   	    U8FLAG=2;
	ldi R24,2
	sts _U8FLAG,R24
L22:
	.dbline 124
L23:
	.dbline 124
; 	   	while((!(COM_R))&&U8FLAG++);
	sbic 0x13,4
	rjmp L25
	lds R2,_U8FLAG
	clr R3
	mov R24,R2
	subi R24,255    ; addi 1
	sts _U8FLAG,R24
	tst R2
	brne L22
L25:
	.dbline 125
; 		 if(U8FLAG==1)break;
	lds R24,_U8FLAG
	cpi R24,1
	brne L26
	.dbline 125
	rjmp L20
L26:
	.dbline 126
; 			Delay_10us();
	rcall _Delay_10us
	.dbline 127
; 			Delay_10us();
	rcall _Delay_10us
	.dbline 128
; 		    Delay_10us();
	rcall _Delay_10us
	.dbline 129
; 			Delay_10us();
	rcall _Delay_10us
	.dbline 130
; 	  		U8temp=0;
	clr R2
	sts _U8temp,R2
	.dbline 131
; 	     if(COM_R)U8temp=1;
	sbis 0x13,4
	rjmp L28
	.dbline 131
	ldi R24,1
	sts _U8temp,R24
L28:
	.dbline 132
; 		    U8FLAG=2;
	ldi R24,2
	sts _U8FLAG,R24
L30:
	.dbline 133
L31:
	.dbline 133
; 		   while((COM_R)&&U8FLAG++);
	sbis 0x13,4
	rjmp L33
	lds R2,_U8FLAG
	clr R3
	mov R24,R2
	subi R24,255    ; addi 1
	sts _U8FLAG,R24
	tst R2
	brne L30
L33:
	.dbline 135
; 	   	//超时则跳出for循环		  
; 	   	 if(U8FLAG==1)break;
	lds R24,_U8FLAG
	cpi R24,1
	brne L34
	.dbline 135
	rjmp L20
L34:
	.dbline 140
	lds R2,_U8comdata
	lsl R2
	sts _U8comdata,R2
	.dbline 141
	lds R2,_U8temp
	lds R3,_U8comdata
	or R3,R2
	sts _U8comdata,R3
	.dbline 142
L19:
	.dbline 120
	inc R20
L21:
	.dbline 120
	cpi R20,8
	brsh X2
	rjmp L18
X2:
L20:
	.dbline -2
L17:
	rcall pop_gset1
	.dbline 0 ; func end
	ret
	.dbsym r i 20 c
	.dbend
	.dbfunc e RH _RH fV
	.even
_RH::
	.dbline -1
	.dbline 160
; 	   	//判断数据位是0还是1	 
; 	   	   
; 		// 如果高电平高过预定0高电平值则数据位为 1 
; 	   	 
; 		   U8comdata<<=1;
; 	   	   U8comdata|=U8temp;        //0
; 	     }//rof
; 	   
; 	}
; 
; 	//--------------------------------
; 	//-----湿度读取子程序 ------------
; 	//--------------------------------
; 	//----以下变量均为全局变量--------
; 	//----温度高8位== U8T_data_H------
; 	//----温度低8位== U8T_data_L------
; 	//----湿度高8位== U8RH_data_H-----
; 	//----湿度低8位== U8RH_data_L-----
; 	//----校验 8位 == U8checkdata-----
; 	//----调用相关子程序如下----------
; 	//---- Delay();, Delay_10us();,COM(); 
; 	//--------------------------------
; 
; 	void RH(void)
; 	{
	.dbline 163
; 	 
; 	  //主机拉低18ms 
;        COM_OUT;
	sbi 0x14,4
	.dbline 164
;        COM_CLR;
	cbi 0x15,4
	.dbline 165
; 	   DelaymS(1);
	ldi R16,1
	ldi R17,0
	rcall _DelaymS
	.dbline 166
; 	   COM_IN;
	cbi 0x14,4
	.dbline 167
; 	   COM_SET;
	sbi 0x15,4
	.dbline 170
; 	  
; 	 //总线由上拉电阻拉高 主机延时20us
; 	   Delay_10us();
	rcall _Delay_10us
	.dbline 171
; 	   Delay_10us();
	rcall _Delay_10us
	.dbline 172
; 	   Delay_10us();
	rcall _Delay_10us
	.dbline 173
; 	   Delay_10us();
	rcall _Delay_10us
	.dbline 179
; 	 //主机设为输入 判断从机响应信号 
; 	 //P2_0=1;
; 	 
; 	 //判断从机是否有低电平响应信号 如不响应则跳出，响应则向下运行	  
; 	  // if(!P2_0)		 //T !	 
; 	   if(COM_R)U8FLAG=1;
	sbis 0x13,4
	rjmp L40
	.dbline 179
	ldi R24,1
	sts _U8FLAG,R24
	rjmp L40
L39:
	.dbline 181
; 	   while(!(COM_R))		 
; 	   {
	.dbline 182
; 	   U8FLAG=2;
	ldi R24,2
	sts _U8FLAG,R24
L42:
	.dbline 185
L43:
	.dbline 185
; 	 //判断从机是否发出 80us 的低电平响应信号是否结束	 
; 	   //while((!P2_0)&&U8FLAG++);
; 	   while((!(COM_R))&&U8FLAG++);
	sbic 0x13,4
	rjmp L45
	lds R2,_U8FLAG
	clr R3
	mov R24,R2
	subi R24,255    ; addi 1
	sts _U8FLAG,R24
	tst R2
	brne L42
L45:
	.dbline 186
; 	   if(U8FLAG==1)break;
	lds R24,_U8FLAG
	cpi R24,1
	brne L46
	.dbline 186
	rjmp L41
L46:
	.dbline 187
; 	   U8FLAG=2;
	ldi R24,2
	sts _U8FLAG,R24
L48:
	.dbline 190
L49:
	.dbline 190
; 	 //判断从机是否发出 80us 的高电平，如发出则进入数据接收状态
; 	 //while((P2_0)&&U8FLAG++);
; 	   while((COM_R)&&U8FLAG++);
	sbis 0x13,4
	rjmp L51
	lds R2,_U8FLAG
	clr R3
	mov R24,R2
	subi R24,255    ; addi 1
	sts _U8FLAG,R24
	tst R2
	brne L48
L51:
	.dbline 191
; 	   if(U8FLAG==1)break;
	lds R24,_U8FLAG
	cpi R24,1
	brne L52
	.dbline 191
	rjmp L41
L52:
	.dbline 193
; 	 //数据接收状态		 
; 	   COM();
	rcall _COM
	.dbline 194
; 	   if(U8FLAG==1)break;
	lds R24,_U8FLAG
	cpi R24,1
	brne L54
	.dbline 194
	rjmp L41
L54:
	.dbline 195
; 	   U8T_data_H_temp=U8comdata;
	lds R2,_U8comdata
	sts _U8T_data_H_temp,R2
	.dbline 196
; 	   COM();
	rcall _COM
	.dbline 197
; 	   if(U8FLAG==1)break;
	lds R24,_U8FLAG
	cpi R24,1
	brne L56
	.dbline 197
	rjmp L41
L56:
	.dbline 198
; 	   U8T_data_L_temp=U8comdata; 
	lds R2,_U8comdata
	sts _U8T_data_L_temp,R2
	.dbline 199
; 	   COM();
	rcall _COM
	.dbline 200
; 	   if(U8FLAG==1)break;
	lds R24,_U8FLAG
	cpi R24,1
	brne L58
	.dbline 200
	rjmp L41
L58:
	.dbline 201
; 	   U8RH_data_H_temp=U8comdata;
	lds R2,_U8comdata
	sts _U8RH_data_H_temp,R2
	.dbline 202
; 	   COM();
	rcall _COM
	.dbline 203
; 	   if(U8FLAG==1)break;
	lds R24,_U8FLAG
	cpi R24,1
	brne L60
	.dbline 203
	rjmp L41
L60:
	.dbline 204
; 	   U8RH_data_L_temp=U8comdata;
	lds R2,_U8comdata
	sts _U8RH_data_L_temp,R2
	.dbline 205
; 	   COM();
	rcall _COM
	.dbline 206
; 	   if(U8FLAG==1)break;
	lds R24,_U8FLAG
	cpi R24,1
	brne L62
	.dbline 206
	rjmp L41
L62:
	.dbline 207
	lds R2,_U8comdata
	sts _U8checkdata_temp,R2
	.dbline 208
	rjmp L41
L40:
	.dbline 180
	sbis 0x13,4
	rjmp L39
L41:
	.dbline 211
; 	   U8checkdata_temp=U8comdata;
; 	   break;
; 	    }//while
; 	   //P2_0=1;
; 	   COM_IN;
	cbi 0x14,4
	.dbline 212
; 	   COM_SET;
	sbi 0x15,4
	.dbline 215
; 	 //数据校验 
; 	 
; 	   U8temp=(U8T_data_H_temp+U8T_data_L_temp+U8RH_data_H_temp+U8RH_data_L_temp);
	lds R2,_U8T_data_L_temp
	lds R3,_U8T_data_H_temp
	add R3,R2
	lds R2,_U8RH_data_H_temp
	add R3,R2
	lds R2,_U8RH_data_L_temp
	add R3,R2
	sts _U8temp,R3
	.dbline 216
; 	   if(U8temp==U8checkdata_temp)
	lds R2,_U8checkdata_temp
	cp R3,R2
	brne L64
	.dbline 217
; 	   {
	.dbline 218
; 	   	  U8RH_data_H=U8T_data_H_temp;
	lds R2,_U8T_data_H_temp
	sts _U8RH_data_H,R2
	.dbline 219
; 	   	  U8RH_data_L=U8T_data_L_temp;
	lds R2,_U8T_data_L_temp
	sts _U8RH_data_L,R2
	.dbline 220
; 	   	  U8T_data_H=U8RH_data_H_temp;
	lds R2,_U8RH_data_H_temp
	sts _U8T_data_H,R2
	.dbline 221
; 	   	  U8T_data_L=U8RH_data_L_temp;
	lds R2,_U8RH_data_L_temp
	sts _U8T_data_L,R2
	.dbline 222
; 	   	  U8checkdata=U8checkdata_temp;
	lds R2,_U8checkdata_temp
	sts _U8checkdata,R2
	.dbline 223
; 	   }//fi
L64:
	.dbline 226
; 	  
; 	  
; 	   COM_IN;
	cbi 0x14,4
	.dbline 227
; 	   COM_SET;
	sbi 0x15,4
	.dbline -2
L36:
	.dbline 0 ; func end
	ret
	.dbend
	.dbfunc e Uart_Init _Uart_Init fV
	.even
_Uart_Init::
	.dbline -1
	.dbline 238
; 
; 	}
; 	
; /*
; 串口初始化函数
; 系统频率：2MHz
; 停止位：1
; 波特率：9600
; 数据长度：8
; */
; void Uart_Init(void) {
	.dbline 239
; 	UCSRA = 0x02;		/*2倍速					 */
	ldi R24,2
	out 0xb,R24
	.dbline 240
; 	UCSRB = 0x18;		/*允许接收和发送*/
	ldi R24,24
	out 0xa,R24
	.dbline 241
; 	UCSRC = 0x06;		/*8位数据				 */
	ldi R24,6
	out 0x20,R24
	.dbline 242
; 	UBRRH = 0x00;
	clr R2
	out 0x20,R2
	.dbline 245
; 	//UBRRL = 47;		/*9600		3.6864M      */  
; 	//UBRRL = 95;	    /*9600	    7.3728M      */ 
; 	UBRRL = 103;	    /*9600	    8.0000M      */ 
	ldi R24,103
	out 0x9,R24
	.dbline -2
L66:
	.dbline 0 ; func end
	ret
	.dbend
	.dbfunc e Uart_Transmit _Uart_Transmit fV
;              i -> R16
	.even
_Uart_Transmit::
	.dbline -1
	.dbline 252
; }
; 
; /*
; 串数据发送
; 查询方式发送
; */
; void Uart_Transmit(unsigned char i) {
L68:
	.dbline 255
L69:
	.dbline 255
; 
; 	/* 等待发送缓冲器为空*/
; 	while (!(UCSRA & (1<<UDRE)));
	sbis 0xb,5
	rjmp L68
	.dbline 257
; 	/* 将数据放入缓冲器，发送数据*/
; 	UDR = i;
	out 0xc,R16
	.dbline -2
L67:
	.dbline 0 ; func end
	ret
	.dbsym r i 16 c
	.dbend
	.dbfunc e Uart_Receive _Uart_Receive fc
	.even
_Uart_Receive::
	.dbline -1
	.dbline 264
; }
; 
; /*
; 串数据接收
; 查询方式接收
; */
; unsigned char Uart_Receive( void ) {
L72:
	.dbline 267
L73:
	.dbline 267
; 
; 	/* 等待接收数据*/
; 	while (!(UCSRA & (1<<RXC)));
	sbis 0xb,7
	rjmp L72
	.dbline 270
; 	/* 从缓冲器中获取并返回数据*/
; 	// Uart_Transmit(UDR);
; 	return UDR;
	in R16,0xc
	.dbline -2
L71:
	.dbline 0 ; func end
	ret
	.dbend
	.dbfunc e init_devices _init_devices fV
;              i -> <dead>
	.even
_init_devices::
	.dbline -1
	.dbline 274
; }
; 
; void init_devices(void) 
; {
	.dbline 276
; unsigned char i;
; 		   CLI();                     
	cli
	.dbline 277
;      port_init();
	rcall _port_init
	.dbline 278
;  watchdog_init();
	rcall _watchdog_init
	.dbline 279
;     Uart_Init();
	rcall _Uart_Init
	.dbline 280
;     MCUCR = 0x03;
	ldi R24,3
	out 0x35,R24
	.dbline 281
;     GICR  = 0x00;                //关外部中断0
	clr R2
	out 0x3b,R2
	.dbline 282
;     TIMSK = 0x00;                //关定时器1中断
	out 0x39,R2
	.dbline 283
;            SEI();                //re-enable interrupts
	sei
	.dbline -2
L75:
	.dbline 0 ; func end
	ret
	.dbsym l i 1 c
	.dbend
	.dbfunc e main _main fV
	.even
_main::
	.dbline -1
	.dbline 299
;                            
; }	
; /**************************************************************************
; 
; 
; 
;                             主函数
; 							
; 							
; 							
; 							
; **************************************************************************/
; 
; 
;  void main(void)
; {
	.dbline 301
; 
;  init_devices() ;
	rcall _init_devices
	.dbline 302
;  COM_IN;
	cbi 0x14,4
	.dbline 303
;  COM_SET;
	sbi 0x15,4
	rjmp L78
L77:
	.dbline 306
;  
;  while(1)
;  {
	.dbline 309
;  
;  //读DHT21   
;   RH();
	rcall _RH
	.dbline 312
;   
;   //由串口发送数据 
;   Uart_Transmit(U8RH_data_H); 
	lds R16,_U8RH_data_H
	rcall _Uart_Transmit
	.dbline 313
;   Uart_Transmit(U8RH_data_L);
	lds R16,_U8RH_data_L
	rcall _Uart_Transmit
	.dbline 314
;   Uart_Transmit(U8T_data_H);  
	lds R16,_U8T_data_H
	rcall _Uart_Transmit
	.dbline 315
;   Uart_Transmit(U8T_data_L); 
	lds R16,_U8T_data_L
	rcall _Uart_Transmit
	.dbline 316
;   Uart_Transmit(U8checkdata); 
	lds R16,_U8checkdata
	rcall _Uart_Transmit
	.dbline 319
;   
; 
;   Uart_Transmit(U8checkdata); 
	lds R16,_U8checkdata
	rcall _Uart_Transmit
	.dbline 321
;   //延时2S
;   for(U8temp=0;U8temp<200;U8temp++)
	clr R2
	sts _U8temp,R2
	rjmp L83
L80:
	.dbline 322
	.dbline 323
	ldi R16,10
	ldi R17,0
	rcall _DelaymS
	.dbline 325
L81:
	.dbline 321
	lds R24,_U8temp
	subi R24,255    ; addi 1
	sts _U8temp,R24
L83:
	.dbline 321
	lds R24,_U8temp
	cpi R24,200
	brlo L80
	.dbline 327
L78:
	.dbline 305
	rjmp L77
X3:
	.dbline -2
L76:
	.dbline 0 ; func end
	ret
	.dbend
	.area bss(ram, con, rel)
	.dbfile C:\DOCUME~1\Owner\桌面\C\给客户调试板\DHT21_8M\DHT21_8M\Display_DHTXX.c
_ReceiveLowByte::
	.blkb 2
	.dbsym e ReceiveLowByte _ReceiveLowByte i
_ReceiveHighByte::
	.blkb 2
	.dbsym e ReceiveHighByte _ReceiveHighByte i
_U8comdata::
	.blkb 1
	.dbsym e U8comdata _U8comdata c
_U8checkdata_temp::
	.blkb 1
	.dbsym e U8checkdata_temp _U8checkdata_temp c
_U8RH_data_L_temp::
	.blkb 1
	.dbsym e U8RH_data_L_temp _U8RH_data_L_temp c
_U8RH_data_H_temp::
	.blkb 1
	.dbsym e U8RH_data_H_temp _U8RH_data_H_temp c
_U8T_data_L_temp::
	.blkb 1
	.dbsym e U8T_data_L_temp _U8T_data_L_temp c
_U8T_data_H_temp::
	.blkb 1
	.dbsym e U8T_data_H_temp _U8T_data_H_temp c
_U8checkdata::
	.blkb 1
	.dbsym e U8checkdata _U8checkdata c
_U8RH_data_L::
	.blkb 1
	.dbsym e U8RH_data_L _U8RH_data_L c
_U8RH_data_H::
	.blkb 1
	.dbsym e U8RH_data_H _U8RH_data_H c
_U8T_data_L::
	.blkb 1
	.dbsym e U8T_data_L _U8T_data_L c
_U8T_data_H::
	.blkb 1
	.dbsym e U8T_data_H _U8T_data_H c
_U8temp::
	.blkb 1
	.dbsym e U8temp _U8temp c
_U8count::
	.blkb 1
	.dbsym e U8count _U8count c
_k::
	.blkb 1
	.dbsym e k _k c
_U8FLAG::
	.blkb 1
	.dbsym e U8FLAG _U8FLAG c
_i::
	.blkb 1
	.dbsym e i _i c
