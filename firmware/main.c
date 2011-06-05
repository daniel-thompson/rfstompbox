/*
 * main.c
 *
 * Main functions for RFStompbox, a firmware for a USB guitar pedal based
 * on vusb.
 *
 * Copyright (C) 2011 Daniel Thompson <daniel@redfelineninja.org.uk> 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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

#define lengthof(x) (sizeof(x) / sizeof(x[0]))

/* ------------------------------------------------------------------------- */

#define BUTTON_PORT PORTB       /* PORTx - register for button output */
#define BUTTON_PIN PINB         /* PINx - register for button input */
#define BUTTON_BIT PB0          /* bit for button input/output */

#define LED_BIT PB1
#define LED_ON()   (PORTB &= ~_BV(LED_BIT))
#define LED_OFF()  (PORTB |= _BV(LED_BIT))
#define LED_TOGGLE() (PORTB ^= _BV(LED_BIT))
#define LED_INIT() (LED_OFF(), DDRB |= _BV(LED_BIT))

#define UTIL_BIN4(x)        (uchar)((0##x & 01000)/64 + (0##x & 0100)/16 + (0##x & 010)/4 + (0##x & 1))
#define UTIL_BIN8(hi, lo)   (uchar)(UTIL_BIN4(hi) * 16 + UTIL_BIN4(lo))

/* ------------------------------------------------------------------------- */

static uchar    reportBuffer[2];    /* buffer for HID reports */
static uchar    idleRate;           /* in 4 ms units */

static uchar    buttonState;		/*  stores state of button */
static uchar    buttonStateChanged;     /*  indicates edge detect on button */

/* The following variables store the status of the current data transfer */
static unsigned currentAddress;
static uchar    bytesRemaining;

/* ------------------------------------------------------------------------- */

struct state {
	uchar next_state;
	uchar timeout;
	uchar timeout_state;
	uchar scancode;
};

enum {
	KEYBOARD_NO_EVENT = 0,

	KEYBOARD_1 = 30,
	KEYBOARD_2,
	KEYBOARD_3,
	KEYBOARD_4,
	KEYBOARD_5,
	KEYBOARD_6,
	KEYBOARD_7,
	KEYBOARD_8,
	KEYBOARD_9,
	KEYBOARD_0,

	KEYBOARD_RETURN = 40,

	KEYBOARD_SPACE = 44,
};

static const EEMEM struct {
    uchar storedCalibration;
    uchar diagnosticReport[7];
    struct state stateTable[96]; /* reduce to 62 for ATTiny45 */
} eeprom = {
    .storedCalibration = 0xff,
    .stateTable = {
#define UP(x) (0x80 | (x))
#define DN(x) (x)
	[  0] = { DN(  0),   0,   1, KEYBOARD_NO_EVENT },
	[  1] = { DN(  2),   0,   0, KEYBOARD_SPACE },
	[  2] = { UP(  1), 100,   3, KEYBOARD_NO_EVENT },
	[  3] = { DN(  4),   0,   0, KEYBOARD_RETURN },
	[  4] = { UP(  5), 100,   1, KEYBOARD_NO_EVENT },
	[  5] = { DN(  6),  40,   3, KEYBOARD_RETURN },
	[  6] = { UP(  7), 100,   1, KEYBOARD_1 },
	[  7] = { DN(  8),   0,   0, KEYBOARD_RETURN },
	[  8] = { UP(  9), 100,   1, KEYBOARD_NO_EVENT },
	[  9] = { DN( 10),  40,   7, KEYBOARD_RETURN },
	[ 10] = { UP(  3), 100,   1, KEYBOARD_2 },
#undef UP
#undef DN
    },
};

#define EEPROM_CALIBRATION ((void *) (&eeprom.storedCalibration))
#define EEPROM_DIAGNOSTIC(x) ((unsigned) (&eeprom.diagnosticReport[x]))
#define EEPROM_STATE(x) ((void *) (&eeprom.stateTable[x]))

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
    0xc0,                          // END_COLLECTION

    0x06, 0x00, 0xff,              // USAGE_PAGE (Generic Desktop)
    0x09, 0x01,                    // USAGE (Vendor Usage 1)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x85, 0x01,                    //   REPORT_ID(1)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x80,                    //   REPORT_COUNT (128)
    0x09, 0x00,                    //   USAGE (Undefined)
    0xb2, 0x02, 0x01,              //   FEATURE (Data,Var,Abs,Buf)
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

static void usbSendScanCode(uchar scancode)
{

	reportBuffer[0] = 0;
	reportBuffer[1] = scancode;

	usbSetInterrupt(reportBuffer, sizeof(reportBuffer));
}

/* ------------------------------------------------------------------------- */

#define SCANQ_LENGTH 8
#define SCANQ_MASK (SCANQ_LENGTH-1)
#define SCANQ_NEXT(x) ((x+1) & SCANQ_MASK)

static uchar scanq[8];
static uchar scanq_head;
static uchar scanq_tail;
static uchar scanq_scancode; /* last trasnsmitted scancode */

static void scanqAppend(uchar scancode)
{
	uchar next_tail = SCANQ_NEXT(scanq_tail);

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
		uchar scancode = scanq[(int) scanq_head];

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

#define TICKS_PER_SECOND      ((F_CPU + 1024) / 2048)
#define TICKS_PER_HUNDREDTH   ((TICKS_PER_SECOND + 50) / 100)
#define TICKS_PER_MILLISECOND ((TICKS_PER_SECOND + 500) / 1000)

uchar clockHundredths;
uchar clockMilliseconds;

#define timeAfter(x, y) (((schar) (x - y)) > 0)

static void timerInit(void)
{
    /* first nibble:  free running clock, no PWM
     * second nibble: prescale by 2048 (~8 ticks per millisecond)
     */
    TCCR1 = UTIL_BIN8(0000, 1100);

    /* syncrhronous clocking mode */
    /*PLLCSR &= ~_BV(PCKE);*/ /* clear by default */
}

static void timerPoll(void)
{
    static uchar next_hundredth = TICKS_PER_HUNDREDTH;
    static uchar next_millisecond = TICKS_PER_MILLISECOND;

    while (timeAfter(TCNT1, next_hundredth)) {
	clockHundredths++;
	next_hundredth += TICKS_PER_HUNDREDTH;
    }

    while (timeAfter(TCNT1, next_millisecond)) {
	clockMilliseconds++;
	next_millisecond += TICKS_PER_MILLISECOND;
    }
}

/* ------------------------------------------------------------------------- */

static void buttonPoll(void)
{
    static uchar debounceTimeIsOver;
    static uchar debounceTimeout;

    uchar tempButtonValue = bit_is_clear(BUTTON_PIN, BUTTON_BIT);

    if (!debounceTimeIsOver)
	if (timeAfter(clockHundredths, debounceTimeout))
            debounceTimeIsOver = 1;

    /* trigger a change if status has changed and the debounce-delay is over,
     * this has good debounce rejection and latency but is subject to
     * false trigger on electrical noise
     */
    if (tempButtonValue != buttonState && debounceTimeIsOver == 1) {
	/* change button state */
	buttonState = tempButtonValue;
	buttonStateChanged = 1;

	/* restart debounce timer */
	debounceTimeIsOver = 0;
	debounceTimeout = clockHundredths + 5;
    }
}

/* ------------------------------------------------------------------------- */

static uchar stateWaitForButton;
static uchar stateTimeout;
static struct state state;

static void stateMachineSwitchTo(uchar stateId)
{
	/* automatic reset on state machine overflow */
	if (stateId > lengthof(eeprom.stateTable))
		stateId = 0;

        eeprom_read_block(&state, EEPROM_STATE(stateId), sizeof(state));

	/* handle transient states early */
	while (0 == state.next_state &&
	       0 == state.timeout) {
		scanqAppend(state.scancode);

		eeprom_read_block(&state, EEPROM_STATE(state.timeout_state), sizeof(state));
        }

	/* figure out what button state we are waiting for (and clear the marker bit) */
	stateWaitForButton = !(state.next_state & _BV(7));
	state.next_state &= ~_BV(7);

	/* calculate the timeout action (this may be a nop) */
	stateTimeout = clockHundredths + state.timeout;
}

static void stateMachineInit(void)
{
	stateMachineSwitchTo(0);
}

static void stateMachinePoll(void)
{
	/* check for timeout first */
	if (state.timeout &&
	    timeAfter(clockHundredths, stateTimeout)) {
		if (0 == state.next_state &&
		    0 != state.scancode)
			scanqAppend(state.scancode);

		stateMachineSwitchTo(state.timeout_state);
	}

	/* check for state change due to button */
	if (buttonStateChanged &&
	    0 != state.next_state &&
	    (buttonState == stateWaitForButton)) {
		if (0 != state.scancode)
			scanqAppend(state.scancode);
	
		stateMachineSwitchTo(state.next_state);
	}
	buttonStateChanged = 0;

}

/* ------------------------------------------------------------------------- */

static void testPoll(void)
{
#if 0
    /* show (in humane units) that the tick rate is correctly calculated */
    static uchar ledTimeout;

    if (timeAfter(clockHundredths, ledTimeout)) {
	ledTimeout += 100; /* two second duty cycle */
	LED_TOGGLE();
    }
#endif
}

/* -------------------------------------------------------------------------------- */
/* ------------------------ interface to USB driver ------------------------ */
/* -------------------------------------------------------------------------------- */

static uchar usbGetReportByte(unsigned offset) {
    if (offset == EEPROM_DIAGNOSTIC(0))
	return OSCCAL;

    if (offset == EEPROM_DIAGNOSTIC(1))
	return stateWaitForButton;

    if (offset == EEPROM_DIAGNOSTIC(2))
	return 0; // unused

    if (offset >= EEPROM_DIAGNOSTIC(3) && offset <= EEPROM_DIAGNOSTIC(7))
	return ((uchar *) (&state))[offset - EEPROM_DIAGNOSTIC(3)];

    return eeprom_read_byte((uchar *)0 + offset);
}

/* usbFunctionRead() is called when the host requests a chunk of data from
 * the device. For more information see the documentation in usbdrv/usbdrv.h.
 */
uchar usbFunctionRead(uchar *data, uchar len)
{
    uchar i;

    if(len > bytesRemaining)
        len = bytesRemaining;

    for (i=0; i<len; i++, currentAddress++)
	    data[i] = usbGetReportByte(currentAddress);
    bytesRemaining -= len;
	 
    return len;
}
/* usbFunctionWrite() is called when the host sends a chunk of data to the
 * device. For more information see the documentation in usbdrv/usbdrv.h.
 */
uchar   usbFunctionWrite(uchar *data, uchar len)
{
    if(bytesRemaining == 0)
        return 1; /* end of transfer */

    if(len > bytesRemaining)
        len = bytesRemaining;

    /* handle the nop and RAM based addresses */
    while (len != 0 &&
           currentAddress <= EEPROM_DIAGNOSTIC(7)) {
        /* most addresses in this range are ignored (write is not allowed to those) */

	/* no RAM based addresses at present */

        data++;
	len--;
	currentAddress++;
	bytesRemaining--;    
    }

    /* lazily program the EEPROM based addresses */
    if (len != 0) {
	eeprom_update_block(data, (uchar *)0 + currentAddress, len);

	currentAddress += len;
	bytesRemaining -= len;
    }

    /* return 1 if this was the last chunk */
    return bytesRemaining == 0;
}

uchar	usbFunctionSetup(uchar data[8])
{
usbRequest_t    *rq = (void *)data;

    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* class request type */
        if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
  	    if (rq->wValue.bytes[0] >= 1 && rq->wValue.bytes[0] <= 4) {
		bytesRemaining = 128;
		currentAddress = (rq->wValue.bytes[0] - 1) * 128;
		return USB_NO_MSG; /* use usbFunctionRead() to obtain data */
	    }

	    usbMsgPtr = reportBuffer;
	    return sizeof(reportBuffer);
	} else if (rq->bRequest == USBRQ_HID_SET_REPORT) {
  	    if (rq->wValue.bytes[0] >= 1 && rq->wValue.bytes[0] <= 4) {
		bytesRemaining = 128;
		currentAddress = (rq->wValue.bytes[0] - 1) * 128;
		return USB_NO_MSG; /* use usbFunctionRead() to obtain data */
	    }

	    return 0;
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
    if (eeprom_read_byte(EEPROM_CALIBRATION) != OSCCAL)
        eeprom_write_byte(EEPROM_CALIBRATION, OSCCAL);
}

/* ------------------------------------------------------------------------- */
/* --------------------------------- main ---------------------------------- */
/* ------------------------------------------------------------------------- */

int main(void)
{
uchar   i;
uchar   calibrationValue;

    calibrationValue = eeprom_read_byte(EEPROM_CALIBRATION); /* calibration value from last time */
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

    LED_INIT();
	/* turn on internal pull-up resistor for the switch */
    BUTTON_PORT |= _BV(BUTTON_BIT);

    stateMachineInit();
    timerInit();

    sei();

    for(;;){    /* main event loop */
        wdt_reset();
        usbPoll();
	buttonPoll();
	stateMachinePoll();
	scanqPoll();
	timerPoll();
	testPoll();
    }

    return 0;
}
