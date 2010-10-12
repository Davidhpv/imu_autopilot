#include "max1168.h"
#include "LPC21xx.h"
#include "spi.h"
#include "conf.h"
#include "armVIC.h"
#include "led.h"

static spi_package max1168_read_channel_package;

static void max1168_on_spi_int_channel0(void);
static void max1168_on_spi_int_channel1(void);
static void max1168_on_spi_int_channel2(void);
static void max1168_on_spi_int_channel3(void);
static void max1168_on_spi_int_channel4(void);
static void max1168_on_spi_int_channel5(void);
static void max1168_on_spi_int_channel6(void);
static void max1168_on_spi_int_channel7(void);
static void max1168_unselect(void);
static void max1168_select(void);
static int max1168_current_channel;
static unsigned short max1168_values[MAX1168_NB_CHAN];

void max1168_init(void) {
	max1168_read_channel_package.bit_mode = SPI_16_BIT_MODE;
	max1168_read_channel_package.data[0] = 0;
	max1168_read_channel_package.data[1] = 0;
	max1168_read_channel_package.length = 2;
	max1168_read_channel_package.slave_select = &max1168_select;
	max1168_read_channel_package.slave_unselect = &max1168_unselect;

	/* SS pin is output */
	MAX1168_SS_IODIR |= (1 << MAX1168_SS_PIN);
	/* unselected max1168 */
	max1168_unselect();

	for (int i = 0; i < MAX1168_NB_CHAN; i++)
		max1168_values[i] = 0;
}

void max1168_read_channel(int channel) {
	max1168_current_channel = channel;
	max1168_read_channel_package.data[0] = (channel << 5) << 8;
	switch (channel){
	case 0:
		max1168_read_channel_package.spi_interrupt_handler = &max1168_on_spi_int_channel0;
		break;
	case 1:
		max1168_read_channel_package.spi_interrupt_handler = &max1168_on_spi_int_channel1;
		break;
	case 2:
		max1168_read_channel_package.spi_interrupt_handler = &max1168_on_spi_int_channel2;
		break;
	case 3:
		max1168_read_channel_package.spi_interrupt_handler = &max1168_on_spi_int_channel3;
		break;
	case 4:
		max1168_read_channel_package.spi_interrupt_handler = &max1168_on_spi_int_channel4;
		break;
	case 5:
		max1168_read_channel_package.spi_interrupt_handler = &max1168_on_spi_int_channel5;
		break;
	case 6:
		max1168_read_channel_package.spi_interrupt_handler = &max1168_on_spi_int_channel6;
		break;
	case 7:
		max1168_read_channel_package.spi_interrupt_handler = &max1168_on_spi_int_channel7;
		break;
	}
	spi_transmit(&max1168_read_channel_package);
	return;
}

unsigned short max1168_get_value(int channel) {
	return max1168_values[channel];
}

static void max1168_on_spi_int_channel0(void) {
	//comm_send_string("channel0 spi int\r\n");
	unsigned short foo __attribute__ ((unused)) = SSPDR;
	unsigned short data = SSPDR;
	max1168_values[0]=data;
	return;
}
static void max1168_on_spi_int_channel1(void) {
	//comm_send_string("channel1 spi int\r\n");
	unsigned short foo __attribute__ ((unused)) = SSPDR;
	unsigned short data = SSPDR;
	max1168_values[1]=data;
	return;
}
static void max1168_on_spi_int_channel2(void) {
	//comm_send_string("channel2 spi int\r\n");
	unsigned short foo __attribute__ ((unused)) = SSPDR;
	unsigned short data = SSPDR;
	max1168_values[2]=data;
	return;
}
static void max1168_on_spi_int_channel3(void) {
	unsigned short foo __attribute__ ((unused)) = SSPDR;
	unsigned short data = SSPDR;
	max1168_values[3]=data;
	return;
}
static void max1168_on_spi_int_channel4(void) {
	unsigned short foo __attribute__ ((unused)) = SSPDR;
	unsigned short data = SSPDR;
	max1168_values[4]=data;
	return;
}
static void max1168_on_spi_int_channel5(void) {
	unsigned short foo __attribute__ ((unused)) = SSPDR;
	unsigned short data = SSPDR;
	max1168_values[5]=data;
	return;
}
static void max1168_on_spi_int_channel6(void) {
	unsigned short foo __attribute__ ((unused)) = SSPDR;
	unsigned short data = SSPDR;
	max1168_values[6]=data;
	return;
}
static void max1168_on_spi_int_channel7(void) {
	/* read out dummy frame */
	unsigned short foo __attribute__ ((unused)) = SSPDR;
	unsigned short data = SSPDR;
	max1168_values[7]=data;
	return;
}

static void max1168_unselect(void){
	MAX1168_SS_IOSET |= (1 << MAX1168_SS_PIN);
}

static void max1168_select(void){
	MAX1168_SS_IOCLR |= (1 << MAX1168_SS_PIN);
}
