/* Name: main.c
 * Project: 1-Key Keyboard
 * Author: Flip van den Berg - www.flipwork.nl
 * Creation Date: 2008-10-06
 * Based on AVR-USB drivers from Objective Developments - http://www.obdev.at/products/avrusb/index.html
 */

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdlib.h>

#include "usbdrv.h"
#include "oddebug.h"

#include "osccal.h"

#define BUTTON_PORT PORTB       /* PORTx - register for button output */
#define BUTTON_PIN PINB         /* PINx - register for button input */
#define BUTTON_BIT PB0          /* bit for button input/output */

/* ------------------------------------------------------------------------- */

static uchar    reportBuffer[2];    /* buffer for HID reports */
static uchar    idleRate;           /* in 4 ms units */

static uchar    buttonState;		/*  stores state of button */
static uchar    buttonStateChanged;     /*  indicates edge detect on button */
static uchar	debounceTimeIsOver;	/* for switch debouncing */

/* ------------------------------------------------------------------------- */

PROGMEM char usbHidReportDescriptor[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] = { /* USB report descriptor */
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                    // USAGE (Keyboard)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
    0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
    0xc0                           // END_COLLECTION
};
/* We use a simplifed keyboard report descriptor which does not support the
 * boot protocol. We don't allow setting status LEDs and we only allow one
 * simultaneous key press (except modifiers). We can therefore use short
 * 2 byte input reports.
 * The report descriptor has been created with usb.org's "HID Descriptor Tool"
 * which can be downloaded from http://www.usb.org/developers/hidpage/.
 * Redundant entries (such as LOGICAL_MINIMUM and USAGE_PAGE) have been omitted
 * for the second INPUT item.
 */

/* ------------------------------------------------------------------------- */

static void usbSendScanCode(unsigned char scancode)
{

	reportBuffer[0] = 0;
	reportBuffer[1] = scancode;

	usbSetInterrupt(reportBuffer, sizeof(reportBuffer));
}

/* ------------------------------------------------------------------------- */

#define SCANQ_LENGTH 8
#define SCANQ_MASK (SCANQ_LENGTH-1)
#define SCANQ_NEXT(x) ((x+1) & SCANQ_MASK)

static unsigned char scanq[8];
static char scanq_head;
static char scanq_tail;
static char scanq_scancode; /* last trasnsmitted scancode */

static void scanqAppend(unsigned char scancode)
{
	char next_tail = SCANQ_NEXT(scanq_tail);

	/* don't allow "no key press" to be queued. scanqPoll() will
	 * automatically insert key up as required and its algorithm
	 * will fail if "no key press" is queued.
	 */
	if (0 == scancode)
		return; /* send no keys */

	if (next_tail == scanq_head) {
		/* TODO: need to call error handler here */
		return;
	}

	scanq[(int) scanq_tail] = scancode;
	scanq_tail = next_tail;

}

static void scanqPoll(void)
{
	if(!usbInterruptIsReady())
		return;

	if (scanq_head == scanq_tail) {
		if (scanq_scancode) {
			scanq_scancode = 0;
			usbSendScanCode(0); /* no keys pressed */
		}
	} else {
		char scancode = scanq[(int) scanq_head];

		if (scancode == scanq_scancode) {
			/* this could loop forever if zero were inserted into the
			 * queue (scanqAppend() prevents this)
			 */
			scanq_scancode = 0;
			usbSendScanCode(0); /* no keys pressed */
		} else {
			scanq_scancode = scancode;
			usbSendScanCode(scancode);
			scanq_head = SCANQ_NEXT(scanq_head);
		}
	}
}

/* ------------------------------------------------------------------------- */

static void timerPoll(void)
{
	static unsigned int timerCnt;

    if(TIFR & (1 << TOV1)){
        TIFR = (1 << TOV1); /* clear overflow */
        if(++timerCnt >= 3){       // 3/63 sec delay for switch debouncing
			timerCnt = 0;
			debounceTimeIsOver = 1; 
        }
    }
}

/* ------------------------------------------------------------------------- */

static void buttonPoll(void) {
	
	uchar tempButtonValue = bit_is_clear(BUTTON_PIN, BUTTON_BIT); //status of switch is stored in tempButtonValue 

	if (tempButtonValue != buttonState && debounceTimeIsOver == 1){ //if status has changed and the debounce-delay is over
		buttonState = tempButtonValue;	// change buttonState to new state
		buttonStateChanged = 1;
		debounceTimeIsOver = 0;	// debounce timer starts
	}
}

/* ------------------------------------------------------------------------- */

static void stateMachinePoll(void)
{
	static char state = 1;
	static uchar timeout;

	switch (state) {
	case 1:
		if (buttonStateChanged && buttonState) {
			scanqAppend(0x2c); /* keyboard spacebar */
			timeout = usbSofCount + 250u;
			state = 2;
		}
		break;
	case 2:
		if (buttonStateChanged && buttonState) {
			/* spotted a double click */
			scanqAppend(0x2c); /* keyboard spacebar */
			scanqAppend(30); /* keypad 1 */
			state = 3;
		}

		if (timeout == usbSofCount) {
			state = 1;
		}
		break;
	case 3:
		if (buttonStateChanged && buttonState) {
			scanqAppend(0x2c); /* keyboard spacebar */
			timeout = usbSofCount + 250u;
			state = 4;
		}
		break;
	case 4:
		if (buttonStateChanged && buttonState) {
			/* spotted a double click */
			scanqAppend(0x2c); /* keyboard spacebar */
			scanqAppend(31); /* keypad 2 */
			state = 1;
		}

		if (timeout == usbSofCount) {
			state = 3;
		}
		break;
	default:
		state = 1;
		break;
	}

	buttonStateChanged = 0;
}

/* ------------------------------------------------------------------------- */

static void timerInit(void)
{
    TCCR1 = 0x0b;           /* select clock: 16.5M/1k -> overflow rate = 16.5M/256k = 62.94 Hz */
}

/* -------------------------------------------------------------------------------- */
/* ------------------------ interface to USB driver ------------------------ */
/* -------------------------------------------------------------------------------- */

uchar	usbFunctionSetup(uchar data[8])
{
usbRequest_t    *rq = (void *)data;

    usbMsgPtr = reportBuffer;
    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* class request type */
        if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
            /* we only have one report type, so don't look at wValue */
            /* buildReport(); */
            return sizeof(reportBuffer);
        }else if(rq->bRequest == USBRQ_HID_GET_IDLE){
            usbMsgPtr = &idleRate;
            return 1;
        }else if(rq->bRequest == USBRQ_HID_SET_IDLE){
            idleRate = rq->wValue.bytes[1];
        }
    }else{
        /* no vendor specific requests implemented */
    }
	return 0;
}

void    hadUsbReset(void)
{
    cli();
    calibrateOscillator();
    sei();

    /* store the calibrated value in EEPROM if it has changed */
    if (eeprom_read_byte(0) != OSCCAL)
        eeprom_write_byte(0, OSCCAL);
}

/* ------------------------------------------------------------------------- */
/* --------------------------------- main ---------------------------------- */
/* ------------------------------------------------------------------------- */

int main(void)
{
uchar   i;
uchar   calibrationValue;

    calibrationValue = eeprom_read_byte(0); /* calibration value from last time */
    if(calibrationValue != 0xff){
        OSCCAL = calibrationValue;
    }
    
	//odDebugInit();
    usbInit();
    usbDeviceDisconnect();  /* enforce re-enumeration, do this while interrupts are disabled! */
    i = 0;
    while(--i){             /* fake USB disconnect for > 250 ms */
        wdt_reset();
        _delay_ms(1);
    }
    usbDeviceConnect();

    wdt_enable(WDTO_1S);

	/* turn on internal pull-up resistor for the switch */
    BUTTON_PORT |= _BV(BUTTON_BIT);

    timerInit();

    sei();

    for(;;){    /* main event loop */
        wdt_reset();
        usbPoll();
	buttonPoll();
	stateMachinePoll();
	scanqPoll();
	timerPoll();
    }

    return 0;
}
