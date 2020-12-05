/**
 * USB support for leJOS.
 *
 * We use a mixture of interrupt driven and directly driven I/O. Interrupts
 * are used for handling the configuration/enumeration phases, which allows
 * us to respond quickly to events. For actual data transfer, we drive the
 * process directly thus removing the need to have data buffers available at
 * interrupt time.
 *
 * As with other leJOS drivers, we implement only a minimal
 * set of functions in the firmware with as much as possible being done in
 * Java. In the case of USB, there are strict timing requirements so we
 * perform all of the configuration and enumeration here.
 *
 * The leJOS implementation uses the standard Lego identifiers (and so can
 * be used from the PC side applications that work with the standard Lego
 * firmware).
 *
 * This implementation handles the initial sequence of requests/responses to place
 * the firmware in a state of being recognized as a CCID device.
 */

#include "mytypes.h"
#include "udp.h"
#include "interrupts.h"
#include "AT91SAM7.h"

#include "aic.h"
#include <string.h>

#define AT91C_PERIPHERAL_ID_UDP        11

#define AT91C_UDP_CSR0  ((AT91_REG *)   0xFFFB0030)
#define AT91C_UDP_CSR1  ((AT91_REG *)   0xFFFB0034)
#define AT91C_UDP_CSR2  ((AT91_REG *)   0xFFFB0038)
#define AT91C_UDP_CSR3  ((AT91_REG *)   0xFFFB003C)

#define AT91C_UDP_FDR0  ((AT91_REG *)   0xFFFB0050)
#define AT91C_UDP_FDR1  ((AT91_REG *)   0xFFFB0054)
#define AT91C_UDP_FDR2  ((AT91_REG *)   0xFFFB0058)
#define AT91C_UDP_FDR3  ((AT91_REG *)   0xFFFB005C)

// Set or clear flag(s) in a register
#define SET_CSR(register, flags)        ((register) = (register) | (flags))
#define CLEAR_CSR(register, flags)      ((register) &= ~(flags))


// Poll the status of flags in a register
#define ISSET(register, flags)      (((register) & (flags)) == (flags))
#define ISCLEARED(register, flags)  (((register) & (flags)) == 0)

#define UDP_CLEAREPFLAGS(register, dFlags) { \
    while (!ISCLEARED((register), dFlags)) \
        CLEAR_CSR((register), dFlags); \
}

#define UDP_SETEPFLAGS(register, dFlags) { \
    while (ISCLEARED((register), dFlags)) \
        SET_CSR((register), dFlags); \
}

#define MIN(a, b) ((a) < (b) ? (a) : (b))

// USB States
#define USB_READY       0
#define USB_CONFIGURED  1
#define USB_SUSPENDED   2

#define USB_DISABLED    0x8000
#define USB_NEEDRESET   0x4000
#define USB_WRITEABLE   0x100000
#define USB_READABLE    0x200000

//#define LED1			(1<<18)						// PA18 green LED on Olimex regular board
//#define LED1			(1<<8)						// PA8 green LED on Olimex header board
#define LED1			(1<<0)						// DS1 green LED on Atmel board


static U8 currentConfig;
static U32 currentFeatures;
static unsigned currentRxBank;
static int configured = (USB_DISABLED|USB_NEEDRESET);
static int newAddress;
static U8 *outPtr;
static U32 outCnt;
static U8 delayedEnable = 0;

#if REMOTE_CONSOLE
	static U8 rConsole = 0;
#endif

/*
	Card control variables
*/
unsigned int clock_frequency[] = {4000, 4800, 6000, 8000};
unsigned int data_rate[] = {10752, 12903, 21505, 25806, 43010, 86021, 129032, 172053, 215053, 344086};
unsigned char cardInserted[] = {0x50, 0x03};
unsigned char return_data[DATA_SIZE];

// Device descriptor
static const U8 dd[] = {
  0x12,	// size of this descriptor
  0x01,	// type of descriptor (DEVICE)
  0x00,	// USB version
  0x02,	// USB version
  0x00,	// class
  0x00,	// subclass
  0x00,	// protocol
  0x08,	// control endpoint data size
  0xEB, // VENDOORID byte 1
  0x03, // VENDOORID byte 2
  0x34, // PRODUCTID byte 1
  0x12, // PRODUCTID byte 2
  0x00, // RELEASE byte 1
  0x00, // RELEASE byte 2
  0x02, // Index of manufacturer description
  0x03,	// Index of product description
  0x01, // Index of serial number description
  0x01	// One possible configuration
};

// Configuration descriptor
static const U8 cfd[] =
{
  0x09,	// size
  0x02,	// configuration type
  0x5D, // size byte 1
  0x00,	// size byte 2
  0x01, // There is one interface in this configuration
  0x01,	// This is configuration #1
  0x00, // No string descriptor for this configuration
  BUSPOWERED_NOREMOTEWAKEUP, // bmAttributes
  50, // power (50 means 100mA)

// Interface descriptor
  0x09,	// size
  0x04,	// type of descriptor (INTERFACE)
  0x00,	// This is interface #0
  0x00, // This is alternate setting #0
  0x03,	// number of endpoints used (1 out, 1 in, 1 interrupt)
  0x0B, // CLASS
  0x00, // SUBCLASS
  0x00, // PROTOCOL
  0x00, // associated string descriptor

// CCID class specific descriptor
  0x36,					// (bLength*) Size of this description
  0x21,					// (bDescriptorType*) Description type
  0x00, 0x01,			// (bcdCCID*) version 1.00
  0x00, 				// (bMaxSlotIndex*)
  0x01,					// (bVoltageSupport) 01h indicates 5.0 volts
  0x01,0x00,0x00,0x00,	// (dwProtocols*) upper word (PPPP) must be 0. Lower word indicates support for T0 and T1 (bit 1 and 2)
  0x00,0x48,0x00,0x00, 	// dwDefaultClock (18.432Mhz given in Khz), not used in v1.00, fixed for legacy reasons
  0x00,0x48,0x00,0x00, 	// dwMaximumClock (18.432Mhz given in Khz), not used in v1.00, fixed for legacy reasons
  0x04, 				// bNumClockSupported => no manual setting
  0x00,0x2A,0x00,0x00,	// (10752) dwDataRate
  0xE7,0x4C,0x06,0x00, 	// (412903) dwMaxDataRate
  0x0A, 				// bNumDataRatesSupported
  0xFE,0x00,0x00,0x00, 	// dwMaxIFSD
  0x07,0x00,0x00,0x00, 	// dwSynchProtocols
  0x00,0x00,0x00,0x00, 	// dwMechanical
  0xB2,0x07,0x02,0x00, 	// dwFeatures
  0x0F,0x01,0x00,0x00,  // dwMaxCCIDMessageLength (271)
  0xFF, 				// bClassGetResponse
  0xFF, 				// bClassEnvelope
  0x00,0x00,			// wLcdLayout
  0x00,					// bPINSupport
  0x01,					// bMaxCCIDBusySlots

// Endpoint descriptors
  0x07,	// size
  0x05,	// type of configuration (ENDPOINT)
  0x01,	// endpoint number and direction (OUT)
  0x02,	// type of endpoint (BULK)
  64,	// endpoint data size byte 1
  0x00, // endpoint data size byte 2
  0x00, // interval

  0x07,	// size
  0x05,	// type of configuration (ENDPOINT)
  0x82,	// endpoint number and direction (IN)
  0x02,	// type of endpoint (BULK)
  64,	// endpoint data size byte 1
  0x00,	// endpoint data size byte 2
  0x00, // interval

  0x07,	// size
  0x05,	// type of configuration (ENDPOINT)
  0x83,	// endpoint number and direction (IN)
  0x03,	// type of endpoint (INTERRUPT)
  8,	// endpoint data size byte 1
  0x00,	// endpoint data size byte 2
  0x18	// interval
};


// Serial Number Descriptor
static U8 snd[] =
{
      0x1A,           // Descriptor length
      0x03,           // Descriptor type 3 == string
      0x31, 0x00,     // MSD of Lap (Lap[2,3]) in UNICode
      0x32, 0x00,     // Lap[4,5]
      0x33, 0x00,     // Lap[6,7]
      0x34, 0x00,     // Lap[8,9]
      0x35, 0x00,     // Lap[10,11]
      0x36, 0x00,     // Lap[12,13]
      0x37, 0x00,     // Lap[14,15]
      0x38, 0x00,     // LSD of Lap (Lap[16,17]) in UNICode
      0x30, 0x00,     // MSD of Nap (Nap[18,19]) in UNICode
      0x30, 0x00,     // LSD of Nap (Nap[20,21]) in UNICode
      0x39, 0x00,     // MSD of Uap in UNICode
      0x30, 0x00      // LSD of Uap in UNICode
};

// Name descriptor, we allow up to 16 unicode characters
static U8 named[] =
{
      0x08,           // Descriptor length
      0x03,           // Descriptor type 3 == string
      0x6e, 0x00,     // n
      0x78, 0x00,     // x
      0x74, 0x00,     // t
      0x00, 0x00,
      0x00, 0x00,
      0x00, 0x00,
      0x00, 0x00,
      0x00, 0x00,
      0x00, 0x00,
      0x00, 0x00,
      0x00, 0x00,
      0x00, 0x00,
      0x00, 0x00,
      0x00, 0x00,
      0x00, 0x00,
      0x00, 0x00
};

// Manufacturer string descriptor
static U8 manufacturer[] =
{
      0x16,           // Descriptor length
      0x03,           // Descriptor type 3 == string
      'B', 0x00,
      'l', 0x00,
      'u', 0x00,
      'e', 0x00,
      ' ', 0x00,
      'R', 0x00,
      'i', 0x00,
      'v', 0x00,
      'e', 0x00,
      'r', 0x00
};

// Product string descriptor
static U8 product[] =
{
      0x0E,           // Descriptor length
      0x03,           // Descriptor type 3 == string
      'V', 0x00,
      '-', 0x00,
      'C', 0x00,
      'a', 0x00,
      'r', 0x00,
      'd', 0x00
};


static const U8 ld[] = {0x04,0x03,0x09,0x04}; // Language descriptor

extern void udp_isr_entry(void);


void led_configure()
{
	volatile AT91PS_PIO	pPIO = AT91C_BASE_PIOA;			// pointer to PIO data structure
	pPIO->PIO_PER = LED1;				// PIO Enable Register - allow PIO to control pins P0 - P3 and pin 19
	pPIO->PIO_OER = LED1;							// PIO Output Enable Register - sets pins P0 - P3 to outputs
	pPIO->PIO_SODR = LED1;							// PIO Set Output Data Register - turns off the four LEDs
}

void led_turnoff()
{
	volatile AT91PS_PIO	pPIO = AT91C_BASE_PIOA;			// pointer to PIO data structure
	pPIO->PIO_SODR = LED1;							// PIO Set Output Data Register - turns off LED
}

void led_turnon()
{
	volatile AT91PS_PIO	pPIO = AT91C_BASE_PIOA;			// pointer to PIO data structure
	pPIO->PIO_CODR = LED1;							// PIO Set Output Data Register - turns on LED
}

// turns the USB activity ON
void usb_activity_on()
{
	led_turnon();
	systick_wait_ms(20);
}

// turns the USB activity OFF
void usb_activity_off()
{
	led_turnoff();
	systick_wait_ms(20);
}


static
void
reset()
{
  // setup config state.
  currentConfig = 0;
  currentRxBank = AT91C_UDP_RX_DATA_BK0;
  configured = USB_READY;
  currentFeatures = 0;
  newAddress = -1;
  outCnt = 0;
  delayedEnable = 0;
}


int
udp_init(void)
{
  udp_disable();
  configured = (USB_DISABLED|USB_NEEDRESET);

  led_configure();
  return 1;
}

void
udp_reset()
{
  int i_state;

  // We must be enabled
  if (configured & USB_DISABLED)
     return;

  // Take the hardware off line
  *AT91C_PIOA_PER = (1 << 16);
  *AT91C_PIOA_OER = (1 << 16);
  *AT91C_PIOA_SODR = (1 << 16);
  *AT91C_PMC_SCDR = AT91C_PMC_UDP;
  *AT91C_PMC_PCDR = (1 << AT91C_ID_UDP);
  systick_wait_ms(2);

  // now bring it back online
  i_state = interrupts_get_and_disable();

  /* Make sure the USB PLL and clock are set up */
  *AT91C_CKGR_PLLR |= AT91C_CKGR_USBDIV_1;
  *AT91C_PMC_SCER = AT91C_PMC_UDP;
  *AT91C_PMC_PCER = (1 << AT91C_ID_UDP);
  *AT91C_UDP_FADDR = 0;
  *AT91C_UDP_GLBSTATE = 0;

  /* Enable the UDP pull up by outputting a zero on PA.16 */
  *AT91C_PIOA_PER = (1 << 16);
  *AT91C_PIOA_OER = (1 << 16);
  *AT91C_PIOA_CODR = (1 << 16);
  *AT91C_UDP_IDR = ~0;

  /* Set up default state */
  reset();


  *AT91C_UDP_IER = (AT91C_UDP_EPINT0 | AT91C_UDP_RXSUSP | AT91C_UDP_RXRSM);
  if (i_state)
    interrupts_enable();
}

int
udp_read(U8* buf, int off, int len)
{
  // Perform a non-blocking read operation. We use double buffering (ping-pong)
  // operation to provide better throughput.
  //
  int packetSize = 0, i, blockSize = 0, a=0;

  if (configured != USB_CONFIGURED)
     return -1;

  if (len == 0)
     return 0;


  while (1)
  {
    if ( !((*AT91C_UDP_CSR1) & currentRxBank) )
       break;

    packetSize = ((*AT91C_UDP_CSR1) & AT91C_UDP_RXBYTECNT) >> 16;

    if (packetSize + blockSize > len) {
		packetSize = len - blockSize;
	}
    else
    if (packetSize > len)
       packetSize = len;

	// turns the USB activity ON
	led_turnon();
	systick_wait_ms(20);

    for (i=0;i<packetSize;i++) {
       buf[off+a++] = *AT91C_UDP_FDR1;
    }

	// Clear transmission flag and wait for the synchronization
	while (*AT91C_UDP_CSR1 & currentRxBank)
       *AT91C_UDP_CSR1 &= ~(currentRxBank);

    // Flip bank
    currentRxBank = currentRxBank == AT91C_UDP_RX_DATA_BK0 ? AT91C_UDP_RX_DATA_BK1 : AT91C_UDP_RX_DATA_BK0;

    blockSize += packetSize;


	// turns the USB activity OFF
	led_turnoff();
	systick_wait_ms(20);

    if (blockSize == len || packetSize < 64)
       break;

    systick_wait_ms(20);

  }

  return blockSize;
}


int
udp_write(U8* buf, int off, int len)
{
  /* Perform a non-blocking write. Return the number of bytes actually
   * written.
   */
  int i;

  if (configured != USB_CONFIGURED)
     return -1;

  // Can we write ?
  if ((*AT91C_UDP_CSR2 & AT91C_UDP_TXPKTRDY) != 0)
     return 0;

  // Limit to max transfer size
  if (len > 64)
     len = 64;

  for (i=0;i<len;i++)
      *AT91C_UDP_FDR2 = buf[off+i];

  UDP_SETEPFLAGS(*AT91C_UDP_CSR2, AT91C_UDP_TXPKTRDY);
  UDP_CLEAREPFLAGS(*AT91C_UDP_CSR2, AT91C_UDP_TXCOMP);
  return len;
}

 /* Perform a non-blocking write through the interrupt endpoint. Return the number of bytes actually
  * written.
  */

int udp_write_interrupt_in(U8* buf, int len)
{
  int i;

  if (configured != USB_CONFIGURED)
     return -1;

  // Can we write ?
  if ((*AT91C_UDP_CSR3 & AT91C_UDP_TXPKTRDY) != 0)
     return 0;

  // Limit to max transfer size
  if (len > 8)
     len = 8;

  for (i=0;i<len;i++)
      *AT91C_UDP_FDR3 = buf[i];

  UDP_SETEPFLAGS(*AT91C_UDP_CSR3, AT91C_UDP_TXPKTRDY);
  UDP_CLEAREPFLAGS(*AT91C_UDP_CSR3, AT91C_UDP_TXCOMP);
  return len;
}


static
void
udp_send_null()
{
  UDP_SETEPFLAGS(*AT91C_UDP_CSR0, AT91C_UDP_TXPKTRDY);
}

static void udp_send_stall()
{
  UDP_SETEPFLAGS(*AT91C_UDP_CSR0, AT91C_UDP_FORCESTALL);
}

static void udp_send_control(U8* p, int len)
{
  outPtr = p;
  outCnt = len;
  int i;

  // Start sending the first part of the data...
  for (i=0; i<8 && i<outCnt; i++)
      *AT91C_UDP_FDR0 = outPtr[i];

  UDP_SETEPFLAGS(*AT91C_UDP_CSR0, AT91C_UDP_TXPKTRDY);
}

static void udp_enumerate()
{
  U8 bt, br;
  int req, len, ind, val;
  short status;
    //display_goto_xy(8,3);
    //display_string(hex4(*AT91C_UDP_CSR0));
    //display_goto_xy(12,3);
    //display_string("    ");

  // First we deal with any completion states.
  if ((*AT91C_UDP_CSR0) & AT91C_UDP_TXCOMP)
  {
    // Write operation has completed.
    // Send config data if needed. Send a zero length packet to mark the
    // end of the data if an exact multiple of 8.
    if (outCnt >= 8)
    {
      outCnt -= 8;
      outPtr += 8;
      int i;
      // Send next part of the data
      for (i=0;i<8 && i<outCnt;i++)
        *AT91C_UDP_FDR0 = outPtr[i];
      UDP_SETEPFLAGS(*AT91C_UDP_CSR0, AT91C_UDP_TXPKTRDY);
    }
    else
      outCnt = 0;

    // Clear the state
    UDP_CLEAREPFLAGS(*AT91C_UDP_CSR0, AT91C_UDP_TXCOMP);
    if (newAddress >= 0)
    {
      // Set new address
      *AT91C_UDP_FADDR = (AT91C_UDP_FEN | newAddress);
      *AT91C_UDP_GLBSTATE  = (newAddress) ? AT91C_UDP_FADDEN : 0;
      newAddress = -1;
    }
  }

  if ((*AT91C_UDP_CSR0) & (AT91C_UDP_RX_DATA_BK0))
  {
    // Got Transfer complete ack
    // Clear the state
    UDP_CLEAREPFLAGS(*AT91C_UDP_CSR0, AT91C_UDP_RX_DATA_BK0);
  }

  if (*AT91C_UDP_CSR0 & AT91C_UDP_ISOERROR)
  {
    // Clear the state
    UDP_CLEAREPFLAGS(*AT91C_UDP_CSR0, (AT91C_UDP_ISOERROR|AT91C_UDP_FORCESTALL));
  }

  //display_goto_xy(12,3);
  //display_string("E1");

  if (!((*AT91C_UDP_CSR0) & AT91C_UDP_RXSETUP))
     return;

  bt = *AT91C_UDP_FDR0;
  br = *AT91C_UDP_FDR0;
  val = ((*AT91C_UDP_FDR0 & 0xFF) | (*AT91C_UDP_FDR0 << 8));
  ind = ((*AT91C_UDP_FDR0 & 0xFF) | (*AT91C_UDP_FDR0 << 8));
  len = ((*AT91C_UDP_FDR0 & 0xFF) | (*AT91C_UDP_FDR0 << 8));

  if (bt & 0x80)
  {
    UDP_SETEPFLAGS(*AT91C_UDP_CSR0, AT91C_UDP_DIR);
  }

  UDP_CLEAREPFLAGS(*AT91C_UDP_CSR0, AT91C_UDP_RXSETUP);

  req = br << 8 | bt;

  switch(req)
  {
	// Here we treat the class specific requests first.
	// Begin of class specific requests
	case ABORT_COMMAND:
		break;

	case GET_CLOCK_FREQUENCIES_COMMAND:
		// this will send data through the Control endpoint
		udp_send_control((U8*)clock_frequency, 16);
		break;

	case GET_DATA_RATES_COMMAND:
		// this will send data through the Control Endpoint
		udp_send_control((U8*)data_rate, 40);

		// Now inform the host driver, through the interrupt endpoint,
		// that a card has been inserted
		udp_write_interrupt_in((U8*)cardInserted, 2);
		break;
	// End of class specific requests

    case STD_GET_DESCRIPTOR:
      if (val == 0x100) // Get device descriptor
      {
        udp_send_control((U8 *)dd, MIN(sizeof(dd), len));
      }
      else
      if (val == 0x200) // Configuration descriptor
      {
        udp_send_control((U8 *)cfd, MIN(sizeof(cfd), len));
        //if (len > sizeof(cfd)) udp_send_null();
      }
      else
      if ((val & 0xF00) == 0x300)
      {
        switch(val & 0xFF)
        {
          case 0x00:
            udp_send_control((U8 *)ld, MIN(sizeof(ld), len));
            break;
          case 0x01:		// serial number string descriptor
            udp_send_control(snd, MIN(sizeof(snd), len));
            break;
          case 0x02:		// manufacturer string descriptor
            udp_send_control(manufacturer, MIN(sizeof(manufacturer), len));
            break;
          case 0x03:		// product string descriptor
            udp_send_control(product, MIN(sizeof(product), len));
            break;
          default:
            udp_send_stall();
        }
      }
      else
      {
        udp_send_stall();
      }
      break;

    case STD_SET_ADDRESS:
      newAddress = val;
      udp_send_null();
      break;

    case STD_SET_CONFIGURATION:
      configured = (val ? USB_CONFIGURED : USB_READY);
      currentConfig = val;
      udp_send_null();
      *AT91C_UDP_GLBSTATE  = (val) ? AT91C_UDP_CONFG : AT91C_UDP_FADDEN;
      delayedEnable = 0;
      *AT91C_UDP_CSR1 = (val) ? (AT91C_UDP_EPEDS | AT91C_UDP_EPTYPE_BULK_OUT) : 0;
      *AT91C_UDP_CSR2 = (val) ? (AT91C_UDP_EPEDS | AT91C_UDP_EPTYPE_BULK_IN)  : 0;
//      *AT91C_UDP_CSR3 = (val) ? (AT91C_UDP_EPTYPE_INT_IN)   : 0;
      *AT91C_UDP_CSR3 = (val) ? (AT91C_UDP_EPEDS | AT91C_UDP_EPTYPE_INT_IN)   : 0;

      break;

    case STD_SET_FEATURE_ENDPOINT:
      ind &= 0x0F;

      if ((val == 0) && ind && (ind <= 3))
      {
        switch (ind)
        {
          case 1:
            (*AT91C_UDP_CSR1) = 0;
            delayedEnable = 0;
            break;
          case 2:
            (*AT91C_UDP_CSR2) = 0;
            break;
          case 3:
            (*AT91C_UDP_CSR3) = 0;
            break;
        }
        udp_send_null();
      }
      else
        udp_send_stall();
      break;

    case STD_CLEAR_FEATURE_ENDPOINT:
      ind &= 0x0F;

      if ((val == 0) && ind && (ind <= 3))
      {
        // Enable and reset the end point
        if (ind == 1) {
          // We need to take special care for the input end point because
          // we may have data in the hardware buffer. If we do then the reset
          // will cause this to be lost. To prevent this loss we delay the
          // enable until the data has been read.
          if (((*AT91C_UDP_CSR1) & AT91C_UDP_RXBYTECNT) == 0)
          {
            (*AT91C_UDP_CSR1) = (AT91C_UDP_EPEDS | AT91C_UDP_EPTYPE_BULK_OUT);
            (*AT91C_UDP_RSTEP) |= AT91C_UDP_EP1;
            (*AT91C_UDP_RSTEP) &= ~AT91C_UDP_EP1;
            delayedEnable = 0;
          }
          else
          {
            // Use delayed anable. We also force the ep disabled to prevent
            // any I/O using the wrong data toggle.
            (*AT91C_UDP_CSR1) &= ~AT91C_UDP_EPEDS;
            delayedEnable = 1;
          }
        }
        else
        if (ind == 2)
        {
          (*AT91C_UDP_CSR2) = (AT91C_UDP_EPEDS | AT91C_UDP_EPTYPE_BULK_IN);
          (*AT91C_UDP_RSTEP) |= AT91C_UDP_EP2;
          (*AT91C_UDP_RSTEP) &= ~AT91C_UDP_EP2;
        }
        else
        if (ind == 3)
        {
          (*AT91C_UDP_CSR3) = (AT91C_UDP_EPEDS | AT91C_UDP_EPTYPE_INT_IN);
          (*AT91C_UDP_RSTEP) |= AT91C_UDP_EP3;
          (*AT91C_UDP_RSTEP) &= ~AT91C_UDP_EP3;
        }
        udp_send_null();
      }
      else
        udp_send_stall();

      break;

    case STD_GET_CONFIGURATION:
      udp_send_control((U8 *) &(currentConfig), MIN(sizeof(currentConfig), len));
      break;

    case STD_GET_STATUS_ZERO:
      status = 0x01;
      udp_send_control((U8 *) &status, MIN(sizeof(status), len));
      break;

    case STD_GET_STATUS_INTERFACE:
      status = 0;
      udp_send_control((U8 *) &status, MIN(sizeof(status), len));
      break;

    case STD_GET_STATUS_ENDPOINT:
      status = 0;
      ind &= 0x0F;

      if (((*AT91C_UDP_GLBSTATE) & AT91C_UDP_CONFG) && (ind <= 3))
      {
        switch (ind)
        {
          case 1:
            status = ((*AT91C_UDP_CSR1) & AT91C_UDP_EPEDS) ? 0 : 1;
            break;
          case 2:
            status = ((*AT91C_UDP_CSR2) & AT91C_UDP_EPEDS) ? 0 : 1;
            break;
          case 3:
            status = ((*AT91C_UDP_CSR3) & AT91C_UDP_EPEDS) ? 0 : 1;
            break;
        }
        udp_send_control((U8 *) &status, MIN(sizeof(status), len));
      }
      else
      if ( ((*AT91C_UDP_GLBSTATE) & AT91C_UDP_FADDEN) && (ind == 0) )
      {
        status = ((*AT91C_UDP_CSR0) & AT91C_UDP_EPEDS) ? 0 : 1;
        udp_send_control((U8 *) &status, MIN(sizeof(status), len));
      }
      else
        udp_send_stall();                                // Illegal request :-(

      break;

    case VENDOR_SET_FEATURE_INTERFACE:
      ind &= 0xf;
      currentFeatures |= (1 << ind);
      udp_send_null();
      break;

    case VENDOR_CLEAR_FEATURE_INTERFACE:
      ind &= 0xf;
      currentFeatures &= ~(1 << ind);
      udp_send_null();
      break;

    case VENDOR_GET_DESCRIPTOR:
      udp_send_control((U8 *)named, MIN(named[0], len));
      break;

    case STD_SET_FEATURE_INTERFACE:
    case STD_CLEAR_FEATURE_INTERFACE:
      udp_send_null();
      break;

    case STD_SET_INTERFACE:
    case STD_SET_FEATURE_ZERO:
    case STD_CLEAR_FEATURE_ZERO:
    default:
      udp_send_stall();
  }
    //display_goto_xy(14,3);
    //display_string("E2");
}

void
udp_isr_C(void)
{
  /* Process interrupts. We mainly use these during the configuration and
   * enumeration stages.
   */

  /*
   display_goto_xy(0,3);
   display_string(hex4(*AT91C_UDP_ISR));
   display_goto_xy(4,3);
   display_string(hex4(intCnt++));
   */

  // Should never get here if disabled, but just in case!
  if (configured & USB_DISABLED)
     return;

  if (*AT91C_UDP_ISR & END_OF_BUS_RESET)
  {
    //display_goto_xy(0,2);
    //display_string("Bus Reset     ");
    //display_update();
    *AT91C_UDP_ICR = END_OF_BUS_RESET;
    *AT91C_UDP_ICR = SUSPEND_RESUME;
    *AT91C_UDP_ICR = WAKEUP;
    *AT91C_UDP_RSTEP = 0xFFFFFFFF;
    *AT91C_UDP_RSTEP = 0x0;
    *AT91C_UDP_FADDR = AT91C_UDP_FEN;
    reset();
    UDP_SETEPFLAGS(*AT91C_UDP_CSR0,(AT91C_UDP_EPEDS | AT91C_UDP_EPTYPE_CTRL));
    *AT91C_UDP_IER = (AT91C_UDP_EPINT0 | AT91C_UDP_RXSUSP | AT91C_UDP_RXRSM);
    //display_goto_xy(12,2);
    //display_string("IE1");
    return;
  }

  if (*AT91C_UDP_ISR & SUSPEND_INT)
  {
    //display_goto_xy(0,2);
    //display_string("Suspend      ");
    //display_update();
    if (configured == USB_CONFIGURED)
       configured = USB_SUSPENDED;
    else
       configured = USB_READY;
    *AT91C_UDP_ICR = SUSPEND_INT;
    currentRxBank = AT91C_UDP_RX_DATA_BK0;
  }

  if (*AT91C_UDP_ISR & SUSPEND_RESUME)
  {
    //display_goto_xy(0,2);
    //display_string("Resume     ");
    //display_update();
    if (configured == USB_SUSPENDED)
       configured = USB_CONFIGURED;
    else
       configured = USB_READY;
    *AT91C_UDP_ICR = WAKEUP;
    *AT91C_UDP_ICR = SUSPEND_RESUME;
  }

  if (*AT91C_UDP_ISR & AT91C_UDP_EPINT0)
  {
    //display_goto_xy(0,2);
    //display_string("Data       ");
    //display_update();
    *AT91C_UDP_ICR = AT91C_UDP_EPINT0;
    udp_enumerate();
  }

  //display_goto_xy(12,2);
  //display_string("IE2");
}

int
udp_status()
{
  /* Return the current status of the USB connection. This information
   * can be used to determine if the connection can be used. We return
   * the connected state, the currently selected configuration and
   * the currenly active features. This latter item is used by co-operating
   * software on the PC and nxt to indicate the start and end of a stream
   * connection.
   */
  int ret = (configured << 28) | (currentConfig << 24) | (currentFeatures & 0xffff);

  if (configured == USB_CONFIGURED)
  {
    if ((*AT91C_UDP_CSR1) & currentRxBank)
       ret |= USB_READABLE;
    if ((*AT91C_UDP_CSR2 & AT91C_UDP_TXPKTRDY) == 0)
       ret |= USB_WRITEABLE;
  }

  return ret;
}

void
udp_enable(int reset)
{
  /* Enable the processing of USB requests. */
  /* Initialise the interrupt handler. We use a very low priority because
   * some of the USB operations can run for a relatively long time...
   */
  if (reset & 0x2)
  {
	#if REMOTE_CONSOLE
    	rConsole = 1;
    	printf("Firmware output enabled\n");
	#endif
    return;
  }

  int i_state = interrupts_get_and_disable();
  aic_mask_off(AT91C_PERIPHERAL_ID_UDP);
  aic_set_vector(AT91C_PERIPHERAL_ID_UDP, AIC_INT_LEVEL_LOWEST, (U32) udp_isr_entry);
  aic_mask_on(AT91C_PERIPHERAL_ID_UDP);
  *AT91C_UDP_IER = (AT91C_UDP_EPINT0 | AT91C_UDP_RXSUSP | AT91C_UDP_RXRSM);
  reset = reset || (configured & USB_NEEDRESET);
  configured &= ~USB_DISABLED;

  if (i_state)
     interrupts_enable();

  if (reset)
     udp_reset();
}

void
udp_disable()
{
  /* Disable processing of USB requests */
  int i_state = interrupts_get_and_disable();
  aic_mask_off(AT91C_PERIPHERAL_ID_UDP);
  *AT91C_UDP_IDR = (AT91C_UDP_EPINT0 | AT91C_UDP_RXSUSP | AT91C_UDP_RXRSM);
  configured |= USB_DISABLED;
  currentFeatures = 0;

  if (i_state)
    interrupts_enable();

	#if REMOTE_CONSOLE
  		rConsole = 0;
	#endif
}

void
udp_set_serialno(U8 *serNo, int len)
{
  /* Set the USB serial number. serNo should point to a 12 character
   * Unicode string, containing the USB serial number.
   */
  if (len == (sizeof(snd)-2)/2)
    memcpy(snd+2, serNo, len*2);
}

void
udp_set_name(U8 *name, int len)
{
  if (len <= (sizeof(named)-2)/2)
  {
    memcpy(named+2, name, len*2);
    named[0] = len*2 + 2;
  }
}


#if REMOTE_CONSOLE
void
udp_rconsole(U8 *buf, int cnt)
{
  if (!rConsole)
     return;

  while (udp_write(buf, 0, cnt) == 0)
    ;
}
#endif
