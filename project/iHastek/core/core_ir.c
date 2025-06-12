#include "ihastek.h"
#include "define.h"
#include "user_config.h"
#include "stdint.h"

//IR NEC Protocol handler, running in timer1 interrupt
#if USE_IR
uint32_t ir_tick = 0;
IR_TypeDef ir_state;

#define HAL_IR_TICK_INC()   ir_tick++
#define HAL_IR_TICK_CLR()   ir_tick = 0

//红外协议处理，定时器1中断中运行，周期125us
void core_ir_sig_handle(uint8_t sig)
{
    if(sig)
    {
        ir_state.sig_high = 1;
        HAL_IR_TICK_INC();
    }
    else
    {
        if(ir_state.sig_high)
        {
            ir_state.sig_high = 0;
            if(ir_state.frame_head)
            {
                ir_state.rec_data >>= 1;

                if(ir_tick > 8 && ir_tick < 20) //bit0
                {
                    ir_state.rec_data |= 0x80000000;
                }
                else if(ir_tick >= 20)  //长按信号
                {
                    ir_state.frame_head = 1;
                    ir_state.rec_data = 0;
                    ir_state.bitcnt = 0;
                    return;
                }

                ir_state.bitcnt++;
                if(ir_state.bitcnt >= 32)   //bit1
                {
                    ir_state.bitcnt = 0;
                    ir_state.frame_head = 0;
                    ir_state.rec_ok = 1;
                }
            }
            else
            {
                if(ir_tick >= 28 && ir_tick < 48)
                {
                    ir_state.frame_head = 1;
                    ir_state.rec_data = 0;
                }
                else
                {
                    ir_state.frame_head = 0;
                }
                ir_state.bitcnt = 0;
            }
        }

        HAL_IR_TICK_CLR();
    }
}

void core_ir_key_press(uint8_t key_code)
{
    extern void core_key_release(uint16_t key);
    core_key_release(key_code);
}

void core_ir_handle(void)
{
    uint16_t user_code;
    uint8_t key_code, key_code_reverse;
    if(ir_state.rec_ok)
    {
        ir_state.rec_ok = 0;
        user_code = ir_state.rec_data & 0xFFFF;
        key_code_reverse = (ir_state.rec_data >> 24) & 0xFF;
        key_code = (ir_state.rec_data >> 16) & 0xFF;

        if((user_code == 0xFF00) && (key_code == ~key_code_reverse))
        {
            switch(key_code)
            {
            case 0x45: ir_state.key_last = KEY_POWER;     break;
            default: return;
            }
            core_ir_key_press(ir_state.key_last);
        }
    }
}

#endif
