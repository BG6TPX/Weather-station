CC = iccavr
CFLAGS =  -IC:\icc\include\ -e -DATMega8  -l -g -Wf-intenum -Mavr_enhanced_small -Wa-W 
ASFLAGS = $(CFLAGS)  -Wa-g
LFLAGS =  -O -LC:\icc\lib\ -g -Wl-W -bfunc_lit:0x26.0x2000 -dram_end:0x45f -bdata:0x60.0x45f -dhwstk_size:16 -beeprom:1.512 -fihx_coff -S2
FILES = Display_DHTXX.o 

Display_DHTXX:	$(FILES)
	$(CC) -o Display_DHTXX $(LFLAGS) @Display_DHTXX.lk  
Display_DHTXX.o: C:/icc/include/iom8v.h C:/icc/include/macros.h
Display_DHTXX.o:	C:\DOCUME~1\Owner\桌面\C\给客户调试板\DHT21_8M\DHT21_8M\Display_DHTXX.c
	$(CC) -c $(CFLAGS) C:\DOCUME~1\Owner\桌面\C\给客户调试板\DHT21_8M\DHT21_8M\Display_DHTXX.c
