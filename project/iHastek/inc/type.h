#ifndef __TYPE_H
#define	__TYPE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <string.h>
#include <stdint.h>
//#include "sc32_conf.h"
//#include "Function_Init.h"
#include "stdio.h"

//typedef unsigned char   tek_u8;
//typedef unsigned int    tek_u16;
//typedef unsigned long   tek_u32;
//typedef unsigned char  tek_bool;

#define _PRINTF_EN  1

#define _ON	1
#define _OFF	0
#define _FALSE      0
#define _TRUE      1
#define _DEC        0
#define _ADD        1
#define _YES         1
#define _NO           0
#define _HIGH         1
#define _LOW           0
/***************************************************
λ����
***************************************************/
#define _BIT0              0x01
#define _BIT1              0x02
#define _BIT2              0x04
#define _BIT3              0x08
#define _BIT4              0x10
#define _BIT5              0x20
#define _BIT6              0x40
#define _BIT7              0x80
#define _BIT8              0x0100
#define _BIT9              0x0200
#define _BIT10            0x0400
#define _BIT11            0x0800
#define _BIT12            0x1000
#define _BIT13            0x2000
#define _BIT14            0x4000
#define _BIT15            0x8000
#define _BIT16            0x010000
#define _BIT17            0x020000
#define _BIT18            0x040000
#define _BIT19            0x080000
#define _BIT20            0x100000
#define _BIT21            0x200000
#define _BIT22            0x400000
#define _BIT23            0x800000
#define _BIT24            0x01000000
#define _BIT25            0x02000000
#define _BIT26            0x04000000
#define _BIT27            0x08000000
#define _BIT28            0x10000000
#define _BIT29            0x20000000
#define _BIT30            0x40000000
#define _BIT31            0x80000000
#define _BIT32            0x0100000000
#define _BIT33            0x0200000000
#define _BIT34            0x0400000000
#define _BIT35            0x0800000000
#define _BIT36            0x1000000000
#define _BIT37            0x2000000000
#define _BIT38            0x4000000000
#define _BIT39            0x8000000000
#define _BIT40            0x010000000000
#define _BIT41            0x020000000000
#define _BIT42            0x040000000000
#define _BIT43            0x080000000000
#define _BIT44            0x100000000000
#define _BIT45            0x200000000000
#define _BIT46            0x400000000000
#define _BIT47            0x800000000000
#define _BIT48            0x01000000000000
#define _BIT49            0x02000000000000
#define _BIT50            0x04000000000000
#define _BIT51            0x08000000000000
#define _BIT52            0x10000000000000
#define _BIT53            0x20000000000000
#define _BIT54            0x40000000000000
#define _BIT55            0x80000000000000
#define _BIT56            0x0100000000000000
#define _BIT57            0x0200000000000000
#define _BIT58            0x0400000000000000
#define _BIT59            0x0800000000000000
#define _BIT60            0x1000000000000000
#define _BIT61            0x2000000000000000
#define _BIT62            0x4000000000000000
#define _BIT63            0x8000000000000000

//***************************************************/
#define _RAM_1K       		0x400
#define _RAM_2K       		0x800
#define _RAM_4K       		0x1000
#define _RAM_8K       		0x2000
#define _RAM_16K     		0x4000
#define _RAM_32K     		0x8000
#define _RAM_64K     		0x10000
#define _RAM_128K   		0x20000
#define _RAM_256K   		0x40000
#define _RAM_512K   		0x80000
#define _RAM_1M      		0x100000
#define _RAM_2M      		0x200000
#define _RAM_4M      		0x400000
#define _RAM_8M      		0x800000
#define _RAM_16M    		0x1000000
#define _RAM_32M    		0x2000000
#define _RAM_64M    		0x4000000
#define _RAM_128M    	0x8000000
#define _RAM_256M    	0x10000000
#define _RAM_512M    	0x20000000
#define _RAM_1G    		0x40000000
//***************************************************/

#define flag0_1     Bin(00000001)
#define flag0_0     Bin(11111110)

#define flag1_1     Bin(00000010)
#define flag1_0     Bin(11111101)

#define flag2_1     Bin(00000100)
#define flag2_0     Bin(11111011)

#define flag3_1     Bin(00001000)
#define flag3_0     Bin(11110111)

#define flag4_1     Bin(00010000)
#define flag4_0     Bin(11101111)

#define flag5_1     Bin(00100000)
#define flag5_0     Bin(11011111)

#define flag6_1     Bin(01000000)
#define flag6_0     Bin(10111111)

#define flag7_1     Bin(10000000)
#define flag7_0     Bin(01111111)




#define TK0     0x00000001
#define TK1     0x00000002
#define TK2     0x00000004
#define TK3     0x00000008
#define TK4     0x00000010
#define TK5     0x00000020
#define TK6     0x00000040
#define TK7     0x00000080

#define TK8     0x00000100
#define TK9     0x00000200
#define TK10    0x00000400
#define TK11    0x00000800
#define TK12    0x00001000
#define TK13    0x00002000
#define TK14    0x00004000
#define TK15    0x00008000


#define TK16    0x00010000
#define TK17    0x00020000
#define TK18    0x00040000
#define TK19    0x00080000
#define TK20    0x00100000
#define TK21    0x00200000
#define TK22    0x00400000
#define TK23    0x00800000
#define TK24    0x01000000
#define TK25    0x02000000
#define TK26    0x04000000
#define TK27    0x08000000
#define TK28    0x10000000
#define TK29    0x20000000
#define TK30    0x40000000
#define TK31    0x80000000


//#define u8                        unsigned char
//#define u16                      uint16_t
//#define u32                      uint32_t
//#define u64                      uint64_t
//#define s8                         char
//#define s16                      int16_t
//#define s32                      int32_t

//#define U8                        u8
//#define U16                      u16
//#define U32                      u32
//***************************************************/

#define SEC(x)                          		(1000 * x)
#define Num2ASCII(Num) 	        		(Num+0x30)

#define _ABS(VALUE1,VALUE2)      (VALUE1 > VALUE2 ? (VALUE1) - (VALUE2) : (VALUE2) - (VALUE1))
#define abs1(x) (((x)>=0)?(x):(-(x)))

#ifdef __cplusplus
}
#endif

#endif /* __LED_H */
