#include "stdio.h"
#include "TimeEvent.h"
#include "type.h"

/*===================================  Types ================================*/
typedef struct TIMEREVENT_STRUCT
{
    uint16_t Time;				///! The rest time of this event
    void (*Event)(void);		///! The event handler's function pointer of this event
} _TIMEREVENT_STRUCT;

/*================================ Definitions ==============================*/
#define _INACTIVE_TIMER_EVENT   	(0xFFFF)
#define _MAX_EVENT_AMOUNT   		(12)		///! The ammount of all soft timer events.


/*================================== Variables ==============================*/
struct TIMEREVENT_STRUCT TimerEvent[_MAX_EVENT_AMOUNT];


/*============================================================================*/
/**
 * TimerEvent_CancelTimerEvent
 * Cancel a timer event
 *
 * @param <void>		{ void }
 * @return 			{ void }
 *
 */
void fw_timer_event_CancelTimerEvent(void (*Event) (void))
{
    uint8_t  timereventcnt;

    // Disable interrupt in future
    for (timereventcnt = 0;timereventcnt < _MAX_EVENT_AMOUNT; timereventcnt++)
    {
        if (TimerEvent[timereventcnt].Event == Event)
        {
            TimerEvent[timereventcnt].Time = _INACTIVE_TIMER_EVENT;
        }
    }
}
/*============================================================================*/
/**
 * TimerEvent_CancelAllTimerEvent
 * Cancel all timer event in internal table
 *
 * @param <void>		{ void }
 * @return 			{ void }
 *
 */
void fw_timer_event_CancelAllTimerEvent(void)
{
    uint8_t  timereventcnt;

    for (timereventcnt = 0; timereventcnt < _MAX_EVENT_AMOUNT; timereventcnt++)
    {
        TimerEvent[timereventcnt].Time = _INACTIVE_TIMER_EVENT;
        TimerEvent[timereventcnt].Event= 0;
    }
 
}
/*============================================================================*/
/**
 * TimerEvent_ActiveTimerEvent
 * Add new function into timer event
 *
 * @param <usTime>	{ timer count }
 * @param <Event>	{ function pointer to callback }
 *
 */
uint8_t fw_timer_event_ActiveTimerEvent(uint16_t msTime, void (*Event)(void))
{
    uint8_t  timereventcnt;
    uint16_t  stTimeInterval;//=msTime*100


    stTimeInterval =  (msTime);
    if (!stTimeInterval)
        stTimeInterval = 1;


    for (timereventcnt = 0;timereventcnt < _MAX_EVENT_AMOUNT;timereventcnt++)
    {
        if ((TimerEvent[timereventcnt].Time != _INACTIVE_TIMER_EVENT) && 
			(TimerEvent[timereventcnt].Event == Event))
        {
            TimerEvent[timereventcnt].Time = stTimeInterval;
            return _FALSE;
        }
    }

    for (timereventcnt = 0;timereventcnt < _MAX_EVENT_AMOUNT;timereventcnt++)
    {
        if (TimerEvent[timereventcnt].Time == _INACTIVE_TIMER_EVENT)
        {
            TimerEvent[timereventcnt].Time = stTimeInterval;
            TimerEvent[timereventcnt].Event = Event;
            return _TRUE;
        }
    }
    return _FALSE;
}

/*============================================================================*/
/**
 * TimerEvent_Handler
 *
 *
 * @param <void>		{ void }
 * @return 			{ void }
 *
 */
void fw_timer_event_Handler(void)
{
    uint8_t  timereventcnt;
    for (timereventcnt = 0; timereventcnt < _MAX_EVENT_AMOUNT; timereventcnt++)
    {
        if (TimerEvent[timereventcnt].Time == 0)
        {
            TimerEvent[timereventcnt].Time = _INACTIVE_TIMER_EVENT;
//			RTD_Log(LOGGER_INFO, "event = %x\n", (UINT32)TimerEvent[timereventcnt].Event);
            (*TimerEvent[timereventcnt].Event) ();

        }
    }
}

void  fw_timer_event_isr_1ms(void)
{
	uint8_t  timereventcnt;
	for (timereventcnt = 0; timereventcnt < _MAX_EVENT_AMOUNT; timereventcnt++)
	{
	if (TimerEvent[timereventcnt].Time != _INACTIVE_TIMER_EVENT)
		{
    		if (TimerEvent[timereventcnt].Time > 0)
			{
    			TimerEvent[timereventcnt].Time--;
			}
	 	}
	}
}

