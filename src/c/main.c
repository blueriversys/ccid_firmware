
#include "mytypes.h"
#include "interrupts.h"
#include "aic.h"
#include "AT91SAM7.h"
#include "stdio.h"
#include "udp.h"
#include "usb_cmd.h"
#include "flash.h"
#include <string.h>

extern U32 __free_ram_start__;
extern U32 __free_ram_end__;
extern U32 __extra_ram_start__;
extern U32 __extra_ram_end__;

#define USB_STATE_MASK       0xf0000000;
#define USB_STATE_CONNECTED  0x10000000;
#define USB_CONFIG_MASK      0x0f000000;
#define ABDATA_SIZE 271  // This is the value of the CCID's dwMaxCCIDMessageLength

#define TEST_PAGE_NUMBER 0

U8 inMsg[ABDATA_SIZE];
U8 reply[ABDATA_SIZE];
U8 gReplyBuffer[FLASH_PAGE_SIZE];
U8 gFlashBuffer[FLASH_PAGE_SIZE];
char gFilename[32];
int gOutCount;
U8 gReplyLen = 0;
int gBytesSent = 0;
int gBytesReceived = 0;
int gFlashIndex = 0;
int gFlashPage = 0;  // start page for binary files
int gFlashStart = 3;  // start page for binary files
int gStartPage;
int gHandle;
U32 gFileSize;
int gPagesWritten; // holds the count of pages written so far
int gPagesRead; // holds the count of pages read so far
int gReadBlock;
int gBytesToSend;
U8 cardInited = 0;
U8 sessionChecked = 0;

void sendNotInited() {
	reply[0] = RDR_TO_PC_DATABLOCK;	// reply message id
	reply[1] = 2; 	      // Count of bytes in the reply data
	reply[5] = inMsg[5];  // bSlot
	reply[6] = inMsg[6];  // bSeq
	reply[7] = 0x00;	  // resp byte 1
	reply[8] = 0x00;	  // resp byte 2
	reply[9] = 0x00;	  // resp byte 3
	reply[10] = (U8)0x90;
	reply[11] = (U8)0x01; 
	udp_write(reply, 0, 12);
}

void flash_read(unsigned int address, unsigned int length, void *data) {
    unsigned char *source = (unsigned char *) address;
    unsigned char *dest = (unsigned char *) data;
    // Read data
    while (length > 0) {
        *dest = *source;
        dest++;
        source++;
        length--;
    }
}

// Little Endian
U32 calc_file_size_BE(U8 * bytes) {
	U32 myInt = bytes[0] + (bytes[1] << 8) + (bytes[2] << 16) + (bytes[3] << 24);
	return myInt;
}

U32 calc_file_size_LE(U8 * bytes) {
	U32 myInt = (bytes[0] << 24) + (bytes[1] << 16) + (bytes[2] << 8) + bytes[3];
	return myInt;
}

/*
static char x4[5];
static char* hexchars = "0123456789abcdef";

// convert an int var into a hex array
static char * hex4(int i)
{
  x4[0] = hexchars[(i >> 12) & 0xF];
  x4[1] = hexchars[(i >> 8) & 0xF];
  x4[2] = hexchars[(i >> 4) & 0xF];
  x4[3] = hexchars[i & 0xF];
  x4[4] = 0;
  return x4;
}
*/

void int32ToArray(U32 n, U8 * bytes) {
    *bytes = (n >> 24) & 0xFF;
    *(bytes+1) = (n >> 16) & 0xFF;
    *(bytes+2) = (n >> 8) & 0xFF;
    *(bytes+3) = n & 0xFF;
}

// called only once after the USB device is powered and before process_usb_requests() loop
void initCheck() {
	// read the index page
	flash_read(DATA_BASE_ADDRESS, FLASH_PAGE_SIZE, gFlashBuffer);
	U8 blank[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	cardInited = memcmp((U8*)gFlashBuffer+16, blank, 15) == 0 ? 0 : 1;
}

void setPassword(U8 * password, U8 len) {
	// read the index page
	flash_read(DATA_BASE_ADDRESS, FLASH_PAGE_SIZE, gFlashBuffer);
	
	// update the index page section
	memcpy(gFlashBuffer+16, password, len);  // the first 16 bytes is RFU
	
	// rewrite index page
	AT91F_Flash_Write(DATA_BASE_ADDRESS, FLASH_PAGE_SIZE, gFlashBuffer);
}
	
void process_usb_requests() {
    int len = udp_read(inMsg, 0, ABDATA_SIZE);

    if (len < 1)
       return;

	int bMessageType = inMsg[0];
	U8 cla = inMsg[10];
	U8 ins = inMsg[11];

    // C0 is the GET RESPONSE command from the usbccid driver to request the card's data
	if (bMessageType == PC_RDR_XFR_BLOCK && inMsg[11] == (U8)0xC0) {
		int requestSize = inMsg[14];
		reply[0] = RDR_TO_PC_DATABLOCK;	// reply message id
		reply[1] = requestSize+2; 	// Count of bytes in the reply data
		reply[5] = inMsg[5];  	// bSlot
		reply[6] = inMsg[6];	// bSeq
		reply[7] = 0x00;		// resp byte 1
		reply[8] = 0x00;		// resp byte 2
		reply[9] = 0x00;		// resp byte 3
		int i;
		
        memcpy(reply+10, gReplyBuffer, requestSize);
		
		if ( (gBytesSent + requestSize) == gBytesToSend ) {
		   reply[10+requestSize] = 0x90;
		   reply[10+requestSize+1] = 0x00;
	    }
	    else {
		   reply[10+requestSize] = 0x61;
		   reply[10+requestSize+1] = gBytesToSend - requestSize;
		}
		
		gBytesSent += requestSize;
		udp_write(reply, 0, 10+requestSize+2);
	}
	else
	// the file name and file size block is a block of 32 bytes
    // XXXXXXXXXXXXXXXXXXXXXXXXXXXXZZZZ
	// X = file name
	// Z = file size
	if (bMessageType == PC_RDR_XFR_BLOCK && cla == (U8)0x80 && ins == (U8)0xC1) {  // The RECEIVE FILE SIZE + FILE NAME command
	    if (!cardInited) {
			sendNotInited();
			return;
		}
	    int offset = inMsg[13];
	    int reqlen = inMsg[14];   // the size of the file name + file size array
        memcpy(gFlashBuffer+offset, inMsg+15+1, reqlen);  // 15 is where the data starts
		AT91F_Flash_Write(DATA_BASE_ADDRESS, FLASH_PAGE_SIZE, gFlashBuffer);
		gReplyLen = 2;
		reply[0] = RDR_TO_PC_DATABLOCK;	// reply message id
		reply[1] = 0x02; 	// Count of bytes in the reply data
		reply[5] = inMsg[5];  	// bSlot
		reply[6] = inMsg[6];	// bSeq
		reply[7] = 0x00;		// resp byte 1
		reply[8] = 0x00;		// resp byte 2
		reply[9] = 0x00;		// resp byte 3
		reply[10] = (U8)0x90;
		reply[11] = (U8)0x00;
		gPagesWritten = 1;
		udp_write(reply, 0, 12);
    }
	else
	// the file name and file size block is a block of 32 bytes
    // XXXXXXXXXXXXXXXXXXXXXXXXXXXXZZZZ
	// X = file name
	// Z = file size
	if (bMessageType == PC_RDR_XFR_BLOCK && cla == (U8)0x80 && ins == (U8)0xC2) {  // The RECEIVE FILE SIZE + FILE NAME command
	    if (!cardInited) {
			sendNotInited();
			return;
		}
	    int reqlen = inMsg[14];   // the size of the file name + file size array
		U8 pageCount = 0;
		int offset;
	    U8 sizeArray[4];
		
		// read the index page
        flash_read(DATA_BASE_ADDRESS, FLASH_PAGE_SIZE, gFlashBuffer);
		
		// calculate occupied pages. i starts at 1 to skip the 32-byte password sector
		for (int i=1; i<8; i++) {
			offset = 32*i;
		    sizeArray[0] = gFlashBuffer[offset+28];
			sizeArray[1] = gFlashBuffer[offset+29];
			sizeArray[2] = gFlashBuffer[offset+30];
			sizeArray[3] = gFlashBuffer[offset+31];
			U32 size = calc_file_size_LE(sizeArray);
			
			if (size == 0) {
				break;
			}
			
			pageCount += (size/256) + 1;
		}
		
		// update the index page section
        memcpy(gFlashBuffer+offset, inMsg+16, reqlen);  // 16 is where the file info starts
		
		// rewrite index page
		AT91F_Flash_Write(DATA_BASE_ADDRESS, FLASH_PAGE_SIZE, gFlashBuffer);
		
		// get a byte array
		U8 pageArray[4];
		int32ToArray((U32)pageCount, pageArray);
		
		reply[0] = RDR_TO_PC_DATABLOCK;	// reply message id
		reply[1] = 0x06; 	// Count of bytes in the reply data
		reply[5] = inMsg[5];  	// bSlot
		reply[6] = inMsg[6];	// bSeq
		reply[7] = 0x00;		// resp byte 1
		reply[8] = 0x00;		// resp byte 2
		reply[9] = 0x00;		// resp byte 3
		reply[10] = pageArray[0];
		reply[11] = pageArray[1];
		reply[12] = pageArray[2];
		reply[13] = pageArray[3];
		reply[14] = (U8)0x90;
		reply[15] = (U8)0x00;
		gPagesWritten = pageCount + 1;
		udp_write(reply, 0, 16);
    }
	else
	if (bMessageType == PC_RDR_XFR_BLOCK && cla == (U8)0x80 && ins == (U8)0xC3) {  // The DELETE INDEX PAGE command
	    if (!cardInited) {
			sendNotInited();
			return;
		}
		// zero out the index page
        memset(gFlashBuffer+32, 0x00, 256); // skip the 32-byte password section of the index page
		
		// write the index page
		AT91F_Flash_Write(DATA_BASE_ADDRESS, FLASH_PAGE_SIZE, gFlashBuffer);
		
		gReplyLen = 2;
		reply[0] = RDR_TO_PC_DATABLOCK;	// reply message id
		reply[1] = 0x02; 	// Count of bytes in the reply data
		reply[5] = inMsg[5];  	// bSlot
		reply[6] = inMsg[6];	// bSeq
		reply[7] = 0x00;		// resp byte 1
		reply[8] = 0x00;		// resp byte 2
		reply[9] = 0x00;		// resp byte 3
		reply[10] = (U8)0x90;
		reply[11] = (U8)0x00;
		udp_write(reply, 0, 12);
	}
	else
	if (bMessageType == PC_RDR_XFR_BLOCK && cla == (U8)0x80 && ins == (U8)0xC4) {  // CHECK PASSWORD command
	    if (!cardInited) {
			sendNotInited();
			return;
		}
	    int reqlen = inMsg[14];

		// read the index page
        flash_read(DATA_BASE_ADDRESS, FLASH_PAGE_SIZE, gFlashBuffer);
		U8 ret;
		U8 blank[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
		
		if ( memcmp((U8*)gFlashBuffer+16, blank, 15) == 0) {
			ret = 2;   // password not yet set
		}
		else
		if ( memcmp((U8*)(gFlashBuffer+16), (U8*)(inMsg+15), reqlen) == 0) {
			ret = 0;   // password matches
		}
		else {
			ret = 1;   // password doesn't match
		}
		
		reply[0] = RDR_TO_PC_DATABLOCK;	// reply message id
		reply[1] = 0x03; 	 // Count of bytes in the reply data
		reply[5] = inMsg[5]; // bSlot
		reply[6] = inMsg[6]; // bSeq
		reply[7] = 0x00;	 // resp byte 1
		reply[8] = 0x00;	 // resp byte 2
		reply[9] = 0x00;	 // resp byte 3
		reply[10] = ret;
		reply[11] = (U8)0x90;
		reply[12] = (U8)0x00;
		udp_write(reply, 0, 13);
	}
	else
	if (bMessageType == PC_RDR_XFR_BLOCK && cla == (U8)0x80 && ins == (U8)0xC5) {  // SET PASSWORD command
	    if (!cardInited) {
			sendNotInited();
			return;
		}
	    int reqlen = inMsg[14];
		setPassword(inMsg+15, reqlen);
		reply[0] = RDR_TO_PC_DATABLOCK;	// reply message id
		reply[1] = 0x02; 	 // Count of bytes in the reply data
		reply[5] = inMsg[5]; // bSlot
		reply[6] = inMsg[6]; // bSeq
		reply[7] = 0x00;	 // resp byte 1
		reply[8] = 0x00;	 // resp byte 2
		reply[9] = 0x00;	 // resp byte 3
		reply[10] = (U8)0x90;
		reply[11] = (U8)0x00;
		udp_write(reply, 0, 12);
	}
	else
	if (bMessageType == PC_RDR_XFR_BLOCK && cla == (U8)0x80 && ins == (U8)0xC6) {  // INIT CARD command
	    if (cardInited) {
		    reply[10] = (U8)0x90;
		    reply[11] = (U8)0x02; // card already initialized
		}
		else {
			int reqlen = inMsg[14];
			setPassword(inMsg+15, reqlen);
			reply[10] = (U8)0x90;
			reply[11] = (U8)0x00;
			cardInited = 1;
		}
		
		reply[0] = RDR_TO_PC_DATABLOCK;	// reply message id
		reply[1] = 0x02; 	 // Count of bytes in the reply data
		reply[5] = inMsg[5]; // bSlot
		reply[6] = inMsg[6]; // bSeq
		reply[7] = 0x00;	 // resp byte 1
		reply[8] = 0x00;	 // resp byte 2
		reply[9] = 0x00;	 // resp byte 3
		udp_write(reply, 0, 12);
	}
	else 
	if (bMessageType == PC_RDR_XFR_BLOCK && cla == (U8)0x80 && ins == (U8)0xB3) {  // The RECEIVE DATA command
	    if (!cardInited) {
			sendNotInited();
			return;
		}
        // 10 11 12 13 14 15
        // 80 B3 00 00 81 80
		int writeFlag = inMsg[13];
	    int reqlen = inMsg[14] - 1;  // the size of the block of data
	    int offset = inMsg[15];
        memcpy(gFlashBuffer+offset, inMsg+16, reqlen);  // 16 is where the data starts
        
        if (writeFlag == 1) {
			AT91F_Flash_Write(DATA_BASE_ADDRESS+(gPagesWritten*256), FLASH_PAGE_SIZE, gFlashBuffer);
		    gPagesWritten++;
		}

		gReplyLen = 2;
		reply[0] = RDR_TO_PC_DATABLOCK;	// reply message id
		reply[1] = 0x02; 	// Count of bytes in the reply data
		reply[5] = inMsg[5];  	// bSlot
		reply[6] = inMsg[6];	// bSeq
		reply[7] = 0x00;		// resp byte 1
		reply[8] = 0x00;		// resp byte 2
		reply[9] = 0x00;		// resp byte 3
		reply[10] = (U8)0x90;
		reply[11] = (U8)0x00;
		udp_write(reply, 0, 12);
    }
	else
	// locate the find in the index page and, if found, returns the file size back to host	
	if (bMessageType == PC_RDR_XFR_BLOCK && cla == (U8)0x80 && ins == (U8)0xB5) {  // The FIND FILE command
	    if (!cardInited) {
			sendNotInited();
			return;
		}
		// read the index page
        flash_read(DATA_BASE_ADDRESS, FLASH_PAGE_SIZE, gFlashBuffer);
		U8 found = 0;
		U8 sizeArray[4];
		U8 len = inMsg[14];
		int np = 0;
		int offset;
		
		// calculate occupied pages
		for (int i=1; i<8; i++) {
			offset = 32*i;
		    sizeArray[0] = gFlashBuffer[offset+28];
			sizeArray[1] = gFlashBuffer[offset+29];
			sizeArray[2] = gFlashBuffer[offset+30];
			sizeArray[3] = gFlashBuffer[offset+31];
			
			if ( memcmp(gFlashBuffer+offset, inMsg+15, len) == 0) {
				found = 1;   // password matches
		        gFileSize = calc_file_size_LE(sizeArray);
				break;
			}
			
			np += (calc_file_size_LE(sizeArray) / 256) + 1;
		}

        if (!found) {
			memset(sizeArray, 0, 4); // zero out the response to the host
		}
		
		gPagesRead = np + 1;
		gReadBlock = 7; // this indicates we need to read on the first request
		
		reply[0] = RDR_TO_PC_DATABLOCK;	// reply message id
		reply[1] = 0x06; 	    // Count of bytes in the reply data
		reply[5] = inMsg[5];  	// bSlot
		reply[6] = inMsg[6];	// bSeq
		reply[7] = 0x00;		// resp byte 1
		reply[8] = 0x00;		// resp byte 2
		reply[9] = 0x00;		// resp byte 3
		reply[10] = sizeArray[0];
		reply[11] = sizeArray[1];
		reply[12] = sizeArray[2];
		reply[13] = sizeArray[3];
		reply[14] = (U8)0x90;
		reply[15] = (U8)0x00;
		udp_write(reply, 0, 16);
    }
	else
	if (bMessageType == PC_RDR_XFR_BLOCK && cla == (U8)0x80 && ins == (U8)0xB7) {  // The READ PAGE command
	    if (!cardInited) {
			sendNotInited();
			return;
		}
	    int reqlen = inMsg[15];  

        if (gReadBlock == 7) {
            flash_read(DATA_BASE_ADDRESS+(gPagesRead*256), FLASH_PAGE_SIZE, gFlashBuffer);
			gReadBlock = 0;
            gPagesRead++;
		}
		else {
			gReadBlock++;
		}

        memcpy(gReplyBuffer, gFlashBuffer+(gReadBlock*reqlen), reqlen);

        reply[10] = 0x61;
        reply[11] = reqlen; // tell C0 there are reqlen bytes to be sent to the host
        gBytesToSend = reqlen;
			
		reply[0] = RDR_TO_PC_DATABLOCK;	// reply message id
		reply[1] = 0x02; 	    // Count of bytes in the reply data
		reply[5] = inMsg[5];  	// bSlot
		reply[6] = inMsg[6];	// bSeq
		reply[7] = 0x00;		// resp byte 1
		reply[8] = 0x00;		// resp byte 2
		reply[9] = 0x00;		// resp byte 3
		gBytesSent = 0;
		udp_write(reply, 0, 12);
    }
	else
	if (bMessageType == PC_RDR_XFR_BLOCK && cla == (U8)0x80 && ins == (U8)0xB8) {  // The PREPARE INDEX PAGE TO BE READ command
	    if (!cardInited) {
			sendNotInited();
			return;
		}
        // read the file table page
        flash_read(DATA_BASE_ADDRESS, FLASH_PAGE_SIZE, gReplyBuffer);
		
		reply[0] = RDR_TO_PC_DATABLOCK;	// reply message id
		reply[1] = 0x02; 	    // Count of bytes in the reply data
		reply[5] = inMsg[5];  	// bSlot
		reply[6] = inMsg[6];	// bSeq
		reply[7] = 0x00;		// resp byte 1
		reply[8] = 0x00;		// resp byte 2
		reply[9] = 0x00;		// resp byte 3
		reply[10] = (U8)0x90;
		reply[11] = (U8)0x00;
		udp_write(reply, 0, 12);
    }
	else
	if (bMessageType == PC_RDR_XFR_BLOCK && cla == (U8)0x80 && ins == (U8)0xB9) { // The READ INDEX PAGE command 
	    if (!cardInited) {
			sendNotInited();
			return;
		}
        // 10 11 12 13 14 15
        // 80 B3 00 00 81 80
	    int reqlen = inMsg[12];  
	    int offset = inMsg[13];
		
		// copy to reply buffer
        memcpy(reply+10, gReplyBuffer+offset+32, reqlen);

		reply[0] = RDR_TO_PC_DATABLOCK;	// reply message id
		reply[1] = 2 + reqlen; 	// Count of bytes in the reply data
		reply[5] = inMsg[5];  	// bSlot
		reply[6] = inMsg[6];	// bSeq
		reply[7] = 0x00;		// resp byte 1
		reply[8] = 0x00;		// resp byte 2
		reply[9] = 0x00;		// resp byte 3
		reply[10+reqlen] = (U8)0x90;
		reply[11+reqlen] = (U8)0x00;
		udp_write(reply, 0, 12+reqlen);
    }
	else
	if	(bMessageType == PC_RDR_ICC_POWER_ON) {
		int rLen = 0;
//      U8 ATR[16] = {0x3B, 0x8D, 0x00, 0x4A, 0x61, 0x76, 0x73, 0xFE, 0x21, 0x1B, 0x66, 0xD0, 0x01, 0x9F, 0x13, 0x4D};
//      U8 ATR[16] = {0x3B, 0x1D, 0x14, 0x4A, 0x61, 0x76, 0x73, 0xFE, 0x21, 0x1B, 0x66, 0xD0, 0x01, 0x9F, 0x13, 0x4D};
//      U8 ATR[18] = {0x3B, 0x1D, 0x14, 0x14, 0x31, 0xE0, 0x73, 0xFE, 0x21, 0x1B, 0x66, 0xD0, 0x01, 0x9F, 0x13, 0x4D};
//      U8 ATR[16] = {0x3B, 0x1D, 0x14, 0x4A, 0x61, 0x76, 0x61, 0x43, 0x61, 0x72, 0x64, 0x06, 0x01, 0x16, 0x01, 0x05};
        U8 ATR[15] = {0x3B, 0xAA, 0x00, 0x40, 0x20, 0x53, 0x4F, 0x53, 0x53, 0x45, 0x06, 0x01, 0x16, 0x01, 0x05};
		rLen = sizeof(ATR);

		reply[0] = RDR_TO_PC_DATABLOCK;	// reply message id
		reply[1] = rLen;  	  // 15 bytes or 0. count of bytes in the reply data
		reply[5] = inMsg[5];  // bSlot
		reply[6] = inMsg[6];  // bSeq
		reply[9] = 0x00;      // resp byte 3

        int i=0;
        for (i=0; i<sizeof(ATR); i++) {
			reply[10+i] = ATR[i];
		}		
		
		udp_write(reply, 0, 10+rLen);
	}
	else
	if	(bMessageType ==  PC_RDR_ICC_POWER_OFF) {
		reply[0] = RDR_TO_PC_SLOTSTATUS;
		reply[1] = 0x00;		// 0 bytes. count of bytes in the reply data
		reply[5] = inMsg[5];  	// bSlot
		reply[6] = inMsg[6];	// bSeq
		reply[7] = 0x01;		// resp byte 1
		reply[8] = 0x00;		// resp byte 2
		reply[9] = 0x01;		// resp byte 3
		udp_write(reply, 0, 10);
	}
	else
	if (bMessageType == PC_RDR_SET_PARAMETERS) {
		reply[0] = RDR_TO_PC_PARAMETERS;	// reply message id
		reply[1] = 0x05;  		// 5 bytes. count of bytes in the reply data
		reply[5] = inMsg[5];  	// bSlot
		reply[6] = inMsg[6];	// bSeq
		reply[7] = 0x00;		// resp byte 1
		reply[8] = 0x00;		// resp byte 2
		reply[9] = 0x00;		// resp byte 3
		reply[10] = 0x11;
		reply[11] = 0x00;
		reply[12] = 0x00;
		reply[13] = 0x20;
		reply[14] = 0x00;
		udp_write(reply, 0, 15);
	}
	else
	if (bMessageType == PC_RDR_XFR_BLOCK) {
		reply[0] = RDR_TO_PC_DATABLOCK;	// reply message id
		reply[1] = 0x02;  		// 2 bytes. count of bytes in the reply data
		reply[5] = inMsg[5];  	// bSlot
		reply[6] = inMsg[6];	// bSeq
		reply[7] = 0x00;		// resp byte 1
		reply[8] = 0x00;		// resp byte 2
		reply[9] = 0x00;		// resp byte 3
		reply[10] = 0x6E;
		reply[11] = 0x00;
		udp_write(reply, 0, 12);
	}

} // end of process_usb_requests()

int main(void) {
  /* When we get here:
   * PLL and flash have been initialised and
   * interrupts are off, but the AIC has not been initialised.
   */

  aic_initialise();
  interrupts_enable();
  udp_init();


  // First, we need to enable USB
  udp_enable(1);

  // Now, we wait until the enumeration process has finished
  while (1) {
    int status = udp_status();
    if ( (status & 0xf0000000) == 0x10000000 && (status & 0xf000000) != 0 ) // compiler will not take my defines here.
       break;
  }

  // this sets the card initialization flag
  initCheck();
  
  while (1) {
	// here is where we process all types of requests coming from the host,
	// including the request to run an application.
	process_usb_requests();
  }
} 
