#ifndef __UDP_H__
#  define __UDP_H__

#  include "mytypes.h"

void udp_isr_C(void);
int udp_init(void);
void udp_disable(void);
void udp_enable(int reset);
void udp_reset(void);
int udp_write(U8* buf, int off, int len);
int udp_read(U8* buf, int off, int len);
int udp_status();
void udp_set_serialno(U8 *serNo, int len);
void udp_set_name(U8 *name, int len);
void udp_rconsole(U8* buf, int len);
void systick_wait_ms(int unit);
void led_turnon();
void led_turnoff();
void usb_activity_on();
void usb_activity_off();

#define USB_TIMEOUT      0x0BB8
#define SUSPEND_INT      ((unsigned int) 0x1 << 8)
#define SUSPEND_RESUME   ((unsigned int) 0x1 << 9)
#define END_OF_BUS_RESET ((unsigned int) 0x1 << 12)
#define WAKEUP           ((unsigned int) 0x1 << 13)

#define BUSPOWERED_NOREMOTEWAKEUP  0x80
#define BUSPOWERED_REMOTEWAKEUP    0xA0
#define SELFPOWERED_NOREMOTEWAKEUP 0xC0
#define SELFPOWERED_REMOTEWAKEUP   0xE0

/* USB standard request codes */

#define STD_GET_STATUS_ZERO           	0x0080
#define STD_GET_STATUS_INTERFACE      	0x0081
#define STD_GET_STATUS_ENDPOINT       	0x0082

#define STD_CLEAR_FEATURE_ZERO        	0x0100
#define STD_CLEAR_FEATURE_INTERFACE   	0x0101
#define STD_CLEAR_FEATURE_ENDPOINT    	0x0102

#define STD_SET_FEATURE_ZERO          	0x0300
#define STD_SET_FEATURE_INTERFACE     	0x0301
#define STD_SET_FEATURE_ENDPOINT      	0x0302

#define STD_SET_ADDRESS               	0x0500
#define STD_GET_DESCRIPTOR            	0x0680
#define STD_SET_DESCRIPTOR            	0x0700
#define STD_GET_CONFIGURATION         	0x0880
#define STD_SET_CONFIGURATION         	0x0900
#define STD_GET_INTERFACE             	0x0A81
#define STD_SET_INTERFACE             	0x0B01
#define STD_SYNCH_FRAME               	0x0C82

#define VENDOR_SET_FEATURE_INTERFACE  	0x0341
#define VENDOR_CLEAR_FEATURE_INTERFACE  0x0141
#define VENDOR_GET_DESCRIPTOR         	0x06c0

#define STD_GET_REPORT_DESCRIPTOR      	0x0681

/* CCID Class Specific Request Code */
#define ABORT_COMMAND 					0x01A1
#define GET_CLOCK_FREQUENCIES_COMMAND 	0x02A1
#define GET_DATA_RATES_COMMAND 			0x03A1

#define PC_RDR_ICC_POWER_ON 			0x62
#define PC_RDR_ICC_POWER_OFF 			0x63
#define PC_RDR_GET_SLOT_STATUS 			0x65
#define PC_RDR_XFR_BLOCK 				0x6F
#define PC_RDR_GET_PARAMETERS 			0x6C
#define PC_RDR_RESET_PARAMETERS 		0x6D
#define PC_RDR_SET_PARAMETERS 			0x61
#define PC_RDR_ESCAPE 					0x6B
#define PC_RDR_ICC_CLOCK 				0x6E
#define PC_RDR_T0APDU 					0x6A
#define PC_RDR_SECURE 					0x69
#define PC_RDR_MECHANICAL 				0x71
#define PC_RDR_ABORT	 				0x72
#define PC_RDR_SETDATARATEANDCLOCK		0x73

#define RDR_TO_PC_DATABLOCK				0x80
#define RDR_TO_PC_SLOTSTATUS			0x81
#define RDR_TO_PC_PARAMETERS			0x82

#define DATA_SIZE 						16
#define DATA_BASE_ADDRESS				0x13FF00	// Start of last page on the flash for the SAM7S256

/* Data types */
/*
typedef unsigned char  uchar; 	// 1 byte
typedef unsigned char  byte; 	// 1 byte
typedef unsigned int   word; 	// 4 bytes
*/
typedef struct DATA_PACKET
{
  	U8 	bMessageType;
  	U8 	dwLength;		// 4 bytes in the 32-bit ARM platform
  	U8	dwLengthUnused1;
  	U8	dwLengthUnused2;
  	U8	dwLengthUnused3;
  	U8 	bSlot;
  	U8	bSeq;
  	U8	respByte1;
  	U8	respByte2;
  	U8	respByte3;
	U8	data[54];
} DATA_PACKET;

#endif
