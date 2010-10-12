#include "inttypes.h"
#include "conf.h"
#include "comm.h"

#include "global_data.h"

#include "uart.h"

void comm_init(mavlink_channel_t chan)
{

    if ( chan == MAVLINK_COMM_0 ) {
    	/* initialize uart0 with the values from conf.h*/
    	uart0_init((int)global_data.param[PARAM_UART0_BAUD], COMM_UART_MODE,UART_FIFO_8);
        //comm_channel_used[chan] = 1;
    }
    if ( chan == MAVLINK_COMM_1 ) {
    	/* initialize uart1 with the values from conf.h*/
    	uart1_init((int)global_data.param[PARAM_UART1_BAUD], COMM_UART_MODE,UART_FIFO_8);
        //comm_channel_used[chan] = 1;
    }
}

uint8_t comm_ch_available(mavlink_channel_t chan)
{
    if (chan == MAVLINK_COMM_0) {
        return uart0_char_available();
    }
    if (chan == MAVLINK_COMM_1) {
    	return uart1_char_available();
    }
    else
        return 0;
}

uint8_t comm_get_ch(mavlink_channel_t chan)
{
	if (chan == MAVLINK_COMM_0) {
		return uart0_get_char();
	}
	if (chan == MAVLINK_COMM_1) {
		return uart1_get_char();
	}
    else return '\0';
}

void comm_send_ch(mavlink_channel_t chan, uint8_t ch)
{
	if (chan == MAVLINK_COMM_0 && global_data.state.uart0mode
			== UART_MODE_MAVLINK)
	{
		uart0_transmit(ch);
	}
	if (chan == MAVLINK_COMM_1 && global_data.state.uart1mode
			== UART_MODE_MAVLINK)
	{
		uart1_transmit(ch);
	}
}

uint8_t comm_check_free_space (mavlink_channel_t chan, uint8_t len)
{
    if (chan == MAVLINK_COMM_0)
    {
    	return uart0_check_free_space(len);
    }
    if (chan == MAVLINK_COMM_1)
    {
    	return uart1_check_free_space(len);
    }
    else
        return 0;
}
