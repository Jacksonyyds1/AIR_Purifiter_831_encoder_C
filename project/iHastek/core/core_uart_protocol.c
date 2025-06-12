#include "ihastek.h"
//#include "user_define.h"

#if USE_UART_PROTOCOL

//static unsigned char  Tx_buf[UART_TX_BUF_LEN];
static unsigned char  rx_buf[UART_RX_BUF_LEN];
static TEKUart_t  uart;
static void _core_uart_buff_init(void)
{
    static unsigned char  init;
    if(init == 0)
    {
        init = 1;
        uart.prx = rx_buf;
      //  uart.ptx = Tx_buf;
     //   uart.rx_size = UART_RX_BUF_LEN;
      //  uart.tx_size = UART_TX_BUF_LEN;
        uart.rx_rear = 0;
        uart.rx_top = 0;
        uart.tx_rear = 0;
        uart.tx_top = 0;
    }
}


#if UART_USE_SELF_PROTOCOL==0

void core_uart_protocol_sent(unsigned int len, unsigned char cmd, unsigned char *payload)
{
	unsigned char uartArr[UART_TX_BUF_LEN];
    unsigned int i, checksum;
    unsigned int all_len = len + 7;
    checksum = 0;
    if(all_len > UART_TX_BUF_LEN)
    {
        return;
    }
	uart.ptx=uartArr;

    uart.ptx[0] = 0xc5;
    uart.ptx[1] = all_len & 0xff;
    uart.ptx[2] = (all_len >> 8) & 0xff;
    uart.ptx[3] = 0 & 0xff;
    uart.ptx[4] = (0 >> 8) & 0xff;
    uart.ptx[5] = cmd;
    checksum = 0;
    for(i = 0; i < 6; i++)
    {
        checksum += uart.ptx[i];
    }
    for(i = 0; i < len; i++)
    {
        uart.ptx[i + 6] = payload[i];
        checksum += payload[i];
    }
    uart.ptx[i + 6] = checksum;

    for(i = 0; i < all_len; i++)
    {
        drive_protocol_uart_send(uart.ptx[i]);
    }
}
#else

void platform_uart_protocol_Handler(unsigned char *cmdState, unsigned char *b_rec, unsigned char Data) //���մ������Զ���Э�������޸�����
{
    static unsigned char  checksum, cmd;
    static unsigned char  palyload_index;
    static unsigned int rev_len, rev_addr;
    static unsigned char  rev_payload[30];
    switch(*cmdState)
    {
        case 0:
            if(Data == 0xC5)
            {
                checksum = Data;
                *cmdState = 1;
            }
            break;
        case 1:
            rev_len = Data;
            *cmdState = 2;
            checksum += Data;
            break;
        case 2:
            rev_len |= (unsigned int)Data << 8;
            *cmdState = 3;
            checksum += Data;
            if(rev_len - 7 > 30) { cmdState = 0; }
            break;
        case 3:
            cmd = Data;
            checksum += Data;
            if(rev_len > 5)
            {
                *cmdState = 4;
            }
            else
            {
                *cmdState = 5;
            }
            palyload_index = 0;
            break;
        case 4:
            if(palyload_index >= 30)
            {
                palyload_index = 0;
                *cmdState = 0;
                break;
            }
            rev_payload[palyload_index++] = Data;
            checksum += Data;
            if(palyload_index >= (rev_len - 5))
            {
                *cmdState = 5;
            }
            break;
        case 5:
            *cmdState = 0;
            *b_rec = 0;
            if(checksum == Data)
            {
                platform_uart_protocol(cmd, rev_payload);
            }
            break;
        default:
            break;
    }
}

#endif



static unsigned char _core_isRecvBufferEmpty(void)
{
    if(uart.rx_rear == uart.rx_top) { return 1; }
    return 0;
}

void core_uart_protocol_rec_byte(unsigned char value)
{
    uart.prx[uart.rx_rear++] = value;
    if(uart.rx_rear >= UART_RX_BUF_LEN) { uart.rx_rear = 0; }
}


static unsigned char _core_RecvBufferReadByte(void)
{
    unsigned char Data;
    Data = uart.prx[uart.rx_top];
    uart.rx_top = (uart.rx_top + 1) % UART_RX_BUF_LEN;
    return Data;
}

int decrypt_arrayData_file(unsigned char *input_file_path, unsigned char *output_file_path, const unsigned char *key,unsigned int file_size) ;
void core_uart_protocol_Handler(void)
{
	extern uint8_t usart2_rx_buffer[4*1024];
	extern volatile uint16_t usart2_rx_counter;
	unsigned char i,dataarr[5]={0xc5,1,0,1,0xc7},databuff[2048]={0};
	unsigned int length=0;
	const unsigned char *key="0123456789abcdef";
	static unsigned long addrIndex = OTABIN_START_ADDR;
//    unsigned char Data;
//    static unsigned char cmdState,b_rec;
//    static unsigned long long  rec_tick;
//    #if (UART_USE_SELF_PROTOCOL==0)
//    static unsigned char cmd,checksum,rev_payload[30];
//    static unsigned int palyload_index,rev_len,rev_addr;
//    #endif
//    _core_uart_buff_init();
    //while(!_core_isRecvBufferEmpty())
		if(usart2_rx_counter)
    {
			length = usart2_rx_buffer[2]; 
			length <<= 8;
			length |= usart2_rx_buffer[1];
			if(usart2_rx_buffer[0] == 0xc5 && (usart2_rx_counter-4)==length)
			{
				decrypt_arrayData_file(&usart2_rx_buffer[3],&databuff[0],key,length);
				flash_write_byte(addrIndex,databuff,length);
				addrIndex += length;
				platform_uart_protocol_sent(1,0,dataarr);
			}
			usart2_rx_counter = 0;
			
		
    }
//    if(b_rec && (Get_Sys_Tick() - rec_tick > UART_DELAY_TIME))
//    {
//        b_rec = 0;
//        cmdState = 0;
//    }
}


#endif

