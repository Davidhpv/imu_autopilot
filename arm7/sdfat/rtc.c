/*****************************************************************************
 *   rtc.c:  Realtime clock C file for Philips LPC214x Family Microprocessors
 *
 *   Copyright(C) 2006, Philips Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2005.10.01  ver 1.00    Prelimnary version, first Release
 *
*****************************************************************************/

// modified by Martin Thomas
// modified by Holger Klabunde code didn't work on LPC2136

#include "LPC21xx.h"                        /* LPC21xx definitions */
#include "rtc.h"
#include "config.h"

volatile DWORD alarm_on = 0;

/*****************************************************************************
** Function name:		RTCHandler
**
** Descriptions:		RTC interrupt handler, it executes based on the
**				the alarm setting
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
/*
void RTCHandler (void) __irq 
{  
    ILR |= ILR_RTCCIF;		// clear interrupt flag
    IENABLE;			// handles nested interrupt

    alarm_on = 1;

    IDISABLE;
    VICVectAddr = 0;		// Acknowledge Interrupt
}
*/

/*****************************************************************************
** Function name:		RTCInit
**
** Descriptions:		Initialize RTC timer
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void RTCInit( void )
{
    alarm_on = 0;

// mthomas: replaced for 32kHz-XTAL

#if 0
    /*--- Initialize registers ---*/    
    AMR = 0;
    CIIR = 0;
    CCR = 0;
    PREINT = PREINT_RTC;
    PREFRAC = PREFRAC_RTC;
#endif	

    RTCAMR = 0;
    RTCCIIR = 0;
//    CCR = 0;
    CCR |= CCR_CTCRST;
    ILR = ILR_RTCCIF | ILR_RTCALF;
		
	PREINT = PREINT_RTC;
	PREFRAC = PREFRAC_RTC;
	
//	CCR = CCR_CLKEN;  /* disabale CTC-Reset and enable clock with CLKEN */
//	CCR |= CCR_CLKSRC; // LPC213x: switch to 32kHz XTAL:
	CCR = CCR_CLKEN | CCR_CLKSRC;  /* disable CTC-Reset and enable clock with CLKEN */

	// LPC213x: disconnect RTC from System-Clock (=connect to 32kHz)
	PCONP &= ~(1UL<<9);
	
    return;
}

/*****************************************************************************
** Function name:		RTCStart
**
** Descriptions:		Start RTC timer
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void RTCStart( void ) 
{
    /*--- Start RTC counters ---*/
    CCR |= CCR_CLKEN;

    ILR = ILR_RTCCIF | ILR_RTCALF;
    return;
}

/*****************************************************************************
** Function name:		RTCStop
**
** Descriptions:		Stop RTC timer
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void RTCStop( void )
{   
    /*--- Stop RTC counters ---*/
    CCR &= ~CCR_CLKEN;
    return;
} 

/*****************************************************************************
** Function name:		RTC_CTCReset
**
** Descriptions:		Reset RTC clock tick counter
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void RTC_CTCReset( void )
{   
    /*--- Reset CTC ---*/
    CCR |= CCR_CTCRST;
    return;
}

/*****************************************************************************
** Function name:		RTCSetTime
**
** Descriptions:		Setup RTC timer value
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void RTCSetTime( RTCTime Time ) 
{
    SEC = Time.RTC_Sec;
    MIN = Time.RTC_Min;
    HOUR = Time.RTC_Hour;
    DOM = Time.RTC_Mday;
    DOW = Time.RTC_Wday;
    DOY = Time.RTC_Yday;
    MONTH = Time.RTC_Mon;
    YEAR = Time.RTC_Year;    
    return;
}

/*****************************************************************************
** Function name:		RTCSetAlarm
**
** Descriptions:		Initialize RTC timer
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void RTCSetAlarm( RTCTime Alarm ) 
{   
    ALSEC = Alarm.RTC_Sec;
    ALMIN = Alarm.RTC_Min;
    ALHOUR = Alarm.RTC_Hour;
    ALDOM = Alarm.RTC_Mday;
    ALDOW = Alarm.RTC_Wday;
    ALDOY = Alarm.RTC_Yday;
    ALMON = Alarm.RTC_Mon;
    ALYEAR = Alarm.RTC_Year;    
    return;
}

/*****************************************************************************
** Function name:		RTCGetTime
**
** Descriptions:		Get RTC timer value
**
** parameters:			None
** Returned value:		The data structure of the RTC time table
** 
*****************************************************************************/
RTCTime RTCGetTime( void ) 
{
    RTCTime LocalTime;
    
    LocalTime.RTC_Sec = SEC;
    LocalTime.RTC_Min = MIN;
    LocalTime.RTC_Hour = HOUR;
    LocalTime.RTC_Mday = DOM;
    LocalTime.RTC_Wday = DOW;
    LocalTime.RTC_Yday = DOY;
    LocalTime.RTC_Mon = MONTH;
    LocalTime.RTC_Year = YEAR;
    return LocalTime;    
}

/*****************************************************************************
** Function name:		RTCSetAlarmMask
**
** Descriptions:		Set RTC timer alarm mask
**
** parameters:			Alarm mask setting
** Returned value:		None
** 
*****************************************************************************/
void RTCSetAlarmMask( DWORD AlarmMask ) 
{
    /*--- Set alarm mask ---*/    
    AMR = AlarmMask;
    return;
}

/*****************************************************************************
**                            End Of File
******************************************************************************/

