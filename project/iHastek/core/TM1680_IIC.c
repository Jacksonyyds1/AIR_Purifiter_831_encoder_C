#include "ihastek.h"
#include "sc32_conf.h"



/****命令宏定义****/
#define TM1680ID     0xe4
#define TM1680ID1    0xe5

#define SYSDIS 			0x80
#define SYSEN 			0x81
#define LEDOFF 			0x82
#define LEDON   		0x83
#define BLINKOFF 		0x88
#define BLINK2HZ 		0x89
#define BLINK1HZ 		0x8A
#define BLINK0_5HZ 	    0x8B

#define SLAVEMODE	  0x90
#define RCMODE0		  0x98
#define RCMODE1		  0x9A
#define EXTCLK0		  0x9C
#define EXTCLK1		  0x9E

#define COM8NMOS	0xA0
#define COM16NMOS	0xA4
#define COM8PMOS	0xA8
#define COM16PMOS	0xAC

#define PWM01	    0xB0
#define PWM02	    0xB1
#define PWM03	    0xB2
#define PWM04	    0xB3
#define PWM05	    0xB4
#define PWM06	    0xB5
#define PWM07	    0xB6
#define PWM08	    0xB7
#define PWM09	    0xB8
#define PWM10	    0xB9
#define PWM11	    0xBA
#define PWM12	    0xBB
#define PWM13	    0xBC
#define PWM14	    0xBD
#define PWM15	    0xBE
#define PWM16	    0xBF
/******命令宏定义******/


/***端口定义***/
//sbit SDA=P1^4;   
//sbit SCL=P1^5;
//sbit MA1=P1^6;
//sbit MA0=P1^7;


unsigned char TM1680perseg[8]={0x10,0x20,0x40,0x80,0x01,0x02,0x04,0x8};
unsigned char DispA[8]={0x10,0xFE,0x92,0x92,0xFE,0x92,0x10,0x10};

void delayms(unsigned int n)  
{
	unsigned int i;
	while(n--)
	{
		for(i=0;i<550;i++);
	}
}

void delayus(unsigned char n)  //256
{
	while (--n)
	{
		_nop_();
	}
}


/**************************底层函数*****************************/    
void TM1680start(void)
{
		drive_SDA_OUTPUT();					
		drive_SCL_OUTPUT();
		
		drive_SCL_PIN(1);
		delayus(100);
		drive_SDA_PIN(1);
    delayus(100); 
    SDA=0;
    delayus(100);  
    SCL=0;
}

void Ack(void)
{
    SCL = 0;  
    delayus(8);     
    SCL = 1;                   
    delayus(8); 
		while(SDA);
		SCL=0;
		delayus(15);
}

void TM1680SetAck(bit ack)
{
    SCL=0; delayus(5);
    SDA = ack;                  
    SCL = 1;                    
    delayus(5);                 
    SCL = 0;                    
    delayus(5);                 
}

void TM1680stop(void)
{
    SDA=0; SCL=1; 
    delayus(10); 
    SDA=1;      
    delayus(10);
    SCL=1; SDA=1;
}


void TM1680SendByte(unsigned char sbyte)
{
    unsigned char i=0;
    for(i=0; i<8; i++)
    {
        SCL=0;delayus(2); 
        if(sbyte&0x80)
        {
               SDA=1;   
        }else{
                SDA=0;
        }
        delayus(3); 
        SCL=1;
        delayus(5); 
        sbyte<<=1;    
        delayus(2); 
    }
    SCL=0; delayus(3);
    SDA=0; delayus(3);
}

unsigned char  TM1680RecvByte(void)
{
		unsigned char i=0, sbyte;
    
        SDA=1;delayus(6); 
        for(i=0; i<8; i++)
        {
           SCL=0;delayus(6); 
            if(SDA)
            {
                sbyte |= 0x01 ;   
            }else{
                sbyte &= 0xfe;    
            }
            delayus(3); 
            SCL=1;
            delayus(5);  
            sbyte<<=1;
        }
        SCL=0;
        return sbyte;
}
/******************************底层函数结束**************************/


/******************************功能函数**************************/

void TM1680WriteCmd(unsigned char scmd)
{
	TM1680start();

	TM1680SendByte(TM1680ID);     
	Ack();

	TM1680SendByte(scmd);  
	Ack();

	TM1680stop();
}

void TM1680WriteOneByte(unsigned char faddr, unsigned char sdate)
{  
    TM1680start();
    
    TM1680SendByte(TM1680ID);     
    Ack();

    TM1680SendByte(faddr);  
    Ack();
    
    TM1680SendByte(sdate); 
    Ack();
    
    TM1680stop();
}


void TM1680PageWrite(unsigned char faddr, unsigned char *pdate,unsigned char cnt) 
{
    unsigned char i=0;
    TM1680start();
    
    TM1680SendByte(TM1680ID);  
    Ack();

    TM1680SendByte(faddr);  
    Ack();

    for(i=0; i<cnt; i++)
    {
        TM1680SendByte(*pdate);  
        Ack();
        pdate++;
    }

    TM1680stop();
}
	 
void TM1680PageAllWrite(unsigned char faddr, unsigned char sdate,unsigned char cnt) 
{
    unsigned char i=0;
    TM1680start();
    
    TM1680SendByte(TM1680ID);  
    Ack();

    TM1680SendByte(faddr);  
    Ack();

    for(i=0; i<cnt; i++)
    {
        TM1680SendByte(sdate);  
        Ack();
    }

    TM1680stop();
}
void TM1680PageAllWrite1(unsigned char faddr1, unsigned char sdate1,unsigned char cnt1) 
{
    unsigned char i1=0;
    TM1680start();
    
    TM1680SendByte(TM1680ID1);  
    Ack();

    TM1680SendByte(faddr1);  //eeprom 地址
    Ack();

    for(i1=0; i1<cnt1; i1++)
    {
        TM1680SendByte(sdate1);  //写数据
        Ack();
    }

    TM1680stop();
}

void TM1680WriteCmdDate(unsigned char faddr, unsigned char sdate, unsigned char cnt)
{
   unsigned char i=0;
	
	TM1680start();

	TM1680SendByte(TM1680ID); 	
	Ack();
	TM1680SendByte(SYSDIS);  
	Ack();
	TM1680SendByte(COM8PMOS); 
	Ack();
	TM1680SendByte(RCMODE1); 
	Ack();
	TM1680SendByte(SYSEN);  
	Ack();
	TM1680SendByte(LEDON); 
	Ack();
	TM1680SendByte(PWM16);   
	Ack();
	TM1680SendByte(BLINKOFF); 
	Ack();
	
	TM1680SendByte(faddr); 
	Ack();
	 
	for(i=0; i<cnt; i++)
	{
			TM1680SendByte(sdate); 
			Ack();	
	}
	
	TM1680stop();
}	
/******************************************函数功能结束****************************************/

void TM1680Init(void)
{
	TM1680start();

	TM1680SendByte(TM1680ID); 	
	Ack();
	TM1680SendByte(SYSDIS);  
	Ack();
	TM1680SendByte(COM8PMOS);   
	Ack();
	TM1680SendByte(RCMODE1); 
	Ack();
	TM1680SendByte(SYSEN);  
	Ack();
	TM1680SendByte(LEDON); 
	Ack();
//	TM1680SendByte(LEDOFF); 
//	Ack();
	TM1680SendByte(PWM16);   
	Ack();
	TM1680SendByte(BLINKOFF); 
	Ack();
	
	TM1680stop();	
}
void TM1680Init1(void)
{
	TM1680start();

	TM1680SendByte(TM1680ID1); 	
	Ack();
	TM1680SendByte(SYSDIS);  
	Ack();
	TM1680SendByte(COM8PMOS);   
	Ack();
	TM1680SendByte(EXTCLK1); 
	Ack();
	TM1680SendByte(SYSEN);  
	Ack();
	TM1680SendByte(LEDON); 
	Ack();
//	TM1680SendByte(LEDOFF); 
//	Ack();
	TM1680SendByte(PWM16);   
	Ack();
	TM1680SendByte(BLINKOFF); 
	Ack();
	
	TM1680stop();	
}
void PWMTest(unsigned char sTime)
{
	 unsigned char i=0;
	
	 for(i=0;i<16;i++)
	{
			TM1680WriteCmd(PWM01 | i);
			delayms(sTime);
			delayms(sTime);
	}
}

/***TM1680ID判断***/
void TM1680IDChange(void)
{
		switch(TM1680ID)
		{
				case 0xe4: MA1=0; MA0=0; break;
				case 0xe5: MA1=0; MA0=1; break;
				case 0xe6: MA1=1; MA0=0; break;
				case 0xe7: MA1=1; MA0=1; break;
				default:break;
		}
}

void TM1680PerDisp(void)
{
	  unsigned char i=0,j=0;
	  unsigned char faddr=0x00, fdate=TM1680perseg[0];
		for(i=0;i<32; i++)
		{
				for(j=0;j<8;j++)
				{
							fdate=fdate|TM1680perseg[j];
							TM1680WriteOneByte(faddr,fdate);
							delayms(20);
				}
				faddr+=2;
				fdate=TM1680perseg[0];
		}
}


void LED_main(void)
{
	unsigned char i=0,j=0,fdate=0x10;
	TM1680Init();
	TM1680Init1();
	TM1680PageAllWrite(0x00, 0x00, 32);  
	delayms(100);
	TM1680PageAllWrite(0x00, 0xff, 32);  
	TM1680PageAllWrite1(0x00, 0xff, 32);                     
}

