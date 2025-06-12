#include "ihastek.h"
#include "sc32_conf.h"
#if 1//USE_IIC       

static unsigned char	b_ErrorBit; 

//static void delay_ms(void)
//{
//	unsigned short i,j;
//	for(i = 0;i < 120;i++ )
//	{
// 		 for (j = 0;j < 100;j++);
//	}
//}



static void delay_us(void)
{
	unsigned short	B_t;
  for (B_t = 0; B_t < 100; B_t++);
}

void drive_SCL_OUTPUT(void);

void drive_SCL_free(void);

void drive_SDA_INPUT(void);
void drive_SDA_OUTPUT(void);
void drive_SCL_PIN(unsigned char en);	
void drive_SDA_PIN(unsigned char en);
unsigned char drive_get_SDA_pin(void);

/****************************************************************************************
 @brief void Start(void)
 @retval 
****************************************************************************************/
void core_Start(void)
{
	drive_SDA_OUTPUT();					
	drive_SCL_OUTPUT();

	drive_SCL_PIN(1); //SCL_PIN = 1
	delay_us();		
	drive_SDA_PIN(1);	//SDA_PIN = 1 		
	delay_us();	
	drive_SDA_PIN(0); //SDA_PIN = 0 		
	delay_us();	
	drive_SCL_PIN(0); //SCL_PIN = 0;		
	delay_us();

}  
/****************************************************************************************
 @brief void Stop(void)
 @retval  
****************************************************************************************/
void core_Stop(void)
{
	drive_SDA_OUTPUT();					
	drive_SCL_OUTPUT();

	drive_SDA_PIN(0); //SDA_PIN = 0
	delay_us();	
	drive_SCL_PIN(1); //SCL_PIN = 1
	delay_us();	
	drive_SDA_PIN(1);	//SDA_PIN = 1;					
	delay_us();
}
/****************************************************************************************
 @brief void Ack(void)
 @retval 
****************************************************************************************/
void core_Ack(void)
{
	drive_SDA_OUTPUT();					
	drive_SCL_OUTPUT();

	drive_SDA_PIN(0); //SDA_PIN = 0;
	delay_us();	
	drive_SCL_PIN(1); //SCL_PIN = 1;
	delay_us();	
	drive_SCL_PIN(0); //SCL_PIN = 0;
	delay_us();
	drive_SDA_PIN(1);	//SDA_PIN = 1;
	delay_us();
}
/****************************************************************************************
 @brief void NoAck(void)
 @retval 
****************************************************************************************/
void core_NoAck(void)
{
	drive_SDA_OUTPUT();					
	drive_SCL_OUTPUT();

	drive_SDA_PIN(1);	//SDA_PIN = 1;
	delay_us();
	drive_SCL_PIN(1); //SCL_PIN = 1;
	delay_us();
	drive_SCL_PIN(0); //SCL_PIN = 0;
	delay_us();
	drive_SDA_PIN(0); //SDA_PIN = 0;
	delay_us();
}
/****************************************************************************************
 @brief void TestAck(void)
 @retval
****************************************************************************************/
void core_TestAck(void)
{
	unsigned int B_i = 0;
	unsigned char Bit = 0;

	drive_SDA_INPUT();						
	drive_SCL_OUTPUT();

	b_ErrorBit = 0;						

	drive_SCL_PIN(1); //SCL_PIN = 1;
	do{
		Bit = drive_get_SDA_pin();
		B_i++;
	}while( Bit && (B_i < 600));   
	if(B_i >= 600)
	{
		b_ErrorBit = 1; 				
	}
	drive_SCL_PIN(0); //SCL_PIN = 0;
	delay_us();
}
/****************************************************************************************
 @brief Write_I2c_byte(UINT8 date)
 @retval 
****************************************************************************************/
void core_Write_I2c_byte(unsigned char B_date)
{
	unsigned char B_i;

	drive_SDA_OUTPUT();					
	drive_SCL_OUTPUT();

	for(B_i = 0; B_i < 8; B_i++)
	{
		if(B_date & 0x80)
		{
			drive_SDA_PIN(1);
		}
		else
		{
			drive_SDA_PIN(0);
		}
		B_date <<= 1;
		delay_us();
		drive_SCL_PIN(1); //SCL_PIN = 1;		
		delay_us();
		drive_SCL_PIN(0); //SCL_PIN = 0;
		delay_us();
	}
}
/****************************************************************************************
 @brief UINT8 Read_I2c_byte(void)
 @retval 
****************************************************************************************/
unsigned char core_Read_I2c_byte(void)
{
	unsigned char B_i,B_byte = 0;

	drive_SDA_INPUT();						
	drive_SCL_OUTPUT();

	for(B_i = 0; B_i < 8; B_i++)
	{
		drive_SCL_PIN(1); //SCL_PIN = 1			
		delay_us();

		B_byte <<= 1;

		if(drive_get_SDA_pin())
		{
			B_byte |= 0x01;	
		}

		drive_SCL_PIN(0); //SCL_PIN = 0;
		delay_us();
	}
	return (B_byte);
}

unsigned char	core_Write_I2C_data(unsigned char IIC_ADDRESS,unsigned char cammand,unsigned char *buf,unsigned char len)
{

	  int i=0;
		core_Start();
		core_Write_I2c_byte(IIC_ADDRESS);
	
		core_TestAck();
    if(b_ErrorBit)
		{
     //  #ifdef DEBUG_PRINT_LEVEL
			  printf("iic(W) address NACK\r\n");
			// #endif
			
			return 0;
		}			

			
		
		core_Write_I2c_byte((unsigned char)(cammand));
		core_TestAck();	
		
    if(b_ErrorBit)
		{
			printf("iic cmd NACK\r\n");			
			return 0;
		}		

		for(i=0;i<len;i++)
		{		
				core_Write_I2c_byte(buf[i]);
				core_TestAck();	
				
				if(b_ErrorBit)
				{
						printf("iic buff[%d] NACK\r\n",(int)buf[i]);
					return 0;
				}				
		}
		
	  core_Stop();
		drive_SCL_free();
	return 1;
}


unsigned char core_Read_I2C_data(unsigned char *B_date,unsigned char B_bytes,unsigned char IIC_ADDRESS)
{
	unsigned char B_i;

		core_Start();
		core_Write_I2c_byte(IIC_ADDRESS);
		core_TestAck();
	
    if(b_ErrorBit)
		{
			  printf("iic(R) address NACK\r\n");
			
			core_Stop();
			drive_SCL_free();			
			return 0;
		}		
	
	

		for (B_i = 0; B_i < B_bytes; B_i++)	
		{
			*B_date = core_Read_I2c_byte();

			if(B_i == B_bytes-1)
			{
				core_NoAck();				
			}
			else
			{	
				core_Ack();						
			}
			
			B_date++;
		}
			
	core_Stop();
	return 1;
}

#endif

