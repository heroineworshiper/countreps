/*
 * Servo/IR to USB serial
 * Copyright (C) 2021-2023 Adam Williams <broadcast at earthling dot net>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 * 
 */


// this provides a TTYACM to the jetson
// It receives IR from the remote control & generates servo PWM.
// USB must be hacked to provide enough current to the servos.

// test PWM:
// echo -n -e "\xff\x2d\xd4\xe5\x00\x10\x00\x20" > /dev/ttyACM0

// CONFIG1L
#pragma config CPUDIV = NOCLKDIV// CPU System Clock Selection bits (No CPU System Clock divide)
#pragma config USBDIV = OFF     // USB Clock Selection bit (USB clock comes directly from the OSC1/OSC2 oscillator block; no divide)

// CONFIG1H
#pragma config FOSC = XT        // Oscillator Selection bits 
#pragma config PLLEN = ON       // 4 X PLL Enable bit (Oscillator multiplied by 4)
#pragma config PCLKEN = ON      // Primary Clock Enable bit (Primary clock enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRTEN = OFF     // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 19        // Brown-out Reset Voltage bits (VBOR set to 3.0 V nominal)

// CONFIG2H
#pragma config WDTEN = ON       // Watchdog Timer Enable bit (WDT is always enabled. SWDTEN bit has no effect.)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config HFOFST = ON      // HFINTOSC Fast Start-up bit (HFINTOSC starts clocking the CPU without waiting for the oscillator to stablize.)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RA3 input pin disabled)

// CONFIG4L
#pragma config STVREN = OFF     // Stack Full/Underflow Reset Enable bit (Stack full/underflow will not cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config BBSIZ = OFF      // Boot Block Size Select bit (1kW boot block size)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Table Write Protection bit (Block 0 not write-protected)
#pragma config WRT1 = OFF       // Table Write Protection bit (Block 1 not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block not protected from table reads executed in other blocks)


#include <xc.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <pic18f14k50.h>

#define CLOCKSPEED 48000000
#define SYNC_CODE0 0xff
#define SYNC_CODE1 0x2d
#define SYNC_CODE2 0xd4
#define SYNC_CODE3 0xe5

#define TIMER3_DELAY (CLOCKSPEED / 4 / 8 / 50)

#define DEBUG_LAT LATBbits.LATB6
#define DEBUG_TRIS TRISBbits.TRISB6

#define SERVO0_LAT LATCbits.LATC6
#define SERVO0_TRIS TRISCbits.TRISC6

#define SERVO1_LAT LATCbits.LATC7
#define SERVO1_TRIS TRISCbits.TRISC7

typedef union 
{
	struct
	{
		unsigned interrupt_complete : 1;
        unsigned ir_wrap : 1;
        unsigned got_ir : 1;
	};
	
	unsigned char value;
} flags_t;

typedef union 
{
	struct
	{
		unsigned bit7 : 1;
		unsigned bit6 : 1;
		unsigned bit5 : 1;
		unsigned bit4 : 1;
		unsigned bit3 : 1;
		unsigned bit2 : 1;
		unsigned bit1 : 1;
		unsigned bit0 : 1;
	};
	
	unsigned char value;
} bits_t;

flags_t flags;
uint16_t pwm0 = 0;
uint16_t pwm1 = 0;
void sync_code0(uint8_t c);
void sync_code1(uint8_t c);
void sync_code2(uint8_t c);
void sync_code3(uint8_t c);
void (*input_state)(uint8_t c) = sync_code0;
uint8_t counter;
uint8_t buffer[8];
uint8_t tick;
#define HZ 200
uint8_t ir_code;
#define IR_LOW 0
#define IR_HIGH 1
#define IR_REPEAT 2
#define IR_TIMEOUT 3
#define IR_LOW_TIME 1100
#define IR_REPEAT_TIME 15000

// UART ------------------------------------------------------------------------

#define UART_OUT_SIZE 256
static uint16_t serial_out_count = 0;
static uint16_t serial_out_ptr = 0;
static uint16_t serial_out_ptr2 = 0;
static uint8_t serial_out_buffer[UART_OUT_SIZE];

// data to send from IR to host
#define IR_SIZE 32
static uint16_t ir_count = 0;
static uint16_t ir_ptr = 0;
static uint16_t ir_ptr2 = 0;
static uint8_t ir_buffer[IR_SIZE];

void init_uart()
{
    RCSTA = 0b10010000;
    TXSTA = 0b00100100;
    BAUDCTL = 0b00001000;
#define BAUDCODE (CLOCKSPEED / 4 / 115200 - 1)
    SPBRGH = BAUDCODE >> 8;
    SPBRG = BAUDCODE & 0xff;
    PIR1bits.RCIF = 0;
    PIE1bits.RCIE = 1;
}

void handle_uart_rx()
{
    flags.interrupt_complete = 0;
// clear interrupt
    uint8_t c = RCREG;
}

// must be in the bottom half to access ir_count
void handle_ir(uint8_t c)
{
    if(ir_count < IR_SIZE)
    {
        ir_buffer[ir_ptr++] = c;
        ir_count++;
        if(ir_ptr >= IR_SIZE)
        {
            ir_ptr = 0;
        }
    }
}

// 48Mhz crystal
void delayMicroseconds(uint16_t x)
{
    uint16_t i;
    x -= 3;
    for(i = 0; i < x; i++)
    {
        asm("nop");
    }
}

void handle_uart()
{
// clear the overflow bit
    if(RCSTAbits.OERR)
    {
        RCSTAbits.OERR = 0;
        RCSTAbits.CREN = 0;
        RCSTAbits.CREN = 1;
    }

    if(PIR1bits.TXIF)
    {
        if(serial_out_count > 0)
        {
            TXREG = serial_out_buffer[serial_out_ptr2++];
            if(serial_out_ptr2 >= UART_OUT_SIZE)
            {
                serial_out_ptr2 = 0;
            }
            serial_out_count--;
        }
    }
}

// flush the UART
void flush_uart()
{
   while(serial_out_count)
   {
       handle_uart();
   }
}


void print_byte(uint8_t c)
{
	if(serial_out_count < UART_OUT_SIZE)
	{
		serial_out_buffer[serial_out_ptr++] = c;
		serial_out_count++;
		if(serial_out_ptr >= UART_OUT_SIZE)
		{
			serial_out_ptr = 0;
		}
	}
}

void print_text(const uint8_t *s)
{
	while(*s != 0)
	{
		print_byte(*s);
		s++;
	}
}

void print_number_nospace(uint16_t number)
{
	if(number >= 10000) print_byte('0' + (number / 10000));
	if(number >= 1000) print_byte('0' + ((number / 1000) % 10));
	if(number >= 100) print_byte('0' + ((number / 100) % 10));
	if(number >= 10) print_byte('0' + ((number / 10) % 10));
	print_byte('0' + (number % 10));
}

void print_number(uint16_t number)
{
    print_number_nospace(number);
   	print_byte(' ');
}

const uint8_t hex_table[] = { 
    '0', '1', '2', '3', '4', '5', '6', '7', 
    '8', '9', 'a', 'b', 'c', 'd', 'e','f'
};

void print_hex2(uint8_t number)
{
    print_byte(hex_table[number >> 4]);
    print_byte(hex_table[number & 0xf]);
   	print_byte(' ');
}

void print_bin(uint8_t number)
{
	print_byte((number & 0x80) ? '1' : '0');
	print_byte((number & 0x40) ? '1' : '0');
	print_byte((number & 0x20) ? '1' : '0');
	print_byte((number & 0x10) ? '1' : '0');
	print_byte((number & 0x8) ? '1' : '0');
	print_byte((number & 0x4) ? '1' : '0');
	print_byte((number & 0x2) ? '1' : '0');
	print_byte((number & 0x1) ? '1' : '0');
}



// USB HID ---------------------------------------------------------------------

#define VENDOR_ID 0x04d8
#define PRODUCT_ID 0x000b
#define USB_WORD(x) (uint8_t)((x) & 0xff), (uint8_t)((x) >> 8)
#define EP0_SIZE 8
#define DESCRIPTOR_DEV     0x01
#define DESCRIPTOR_CFG     0x02
#define DESCRIPTOR_STR     0x03
#define DESCRIPTOR_INTF    0x04
#define DESCRIPTOR_EP      0x05
#define DESCRIPTOR_QUAL    0x06
/* Class Descriptor Types */
#define DSC_HID         0x21
#define DSC_RPT         0x22
#define DSC_PHY         0x23
/* HID Interface Class Code */
#define HID_INTF                    0x03
/* HID Interface Class SubClass Codes */
#define BOOT_INTF_SUBCLASS          0x01
/* HID Interface Class Protocol Codes */
#define HID_PROTOCOL_NONE           0x00
#define HID_PROTOCOL_KEYBOARD       0x01
#define HID_PROTOCOL_MOUSE          0x02
#define HID_NUM_OF_DSC 1
#define _EP_IN 0x80
#define _EP_OUT 0x00
#define CDC_EP  0x01
#define DATA_EP 0x02
#define EP_TYPE_INTERRUPT 0x03
#define EP_TYPE_BULK 0x2 
#define SETUP_TOKEN 0b00001101
#define EP1_SIZE 0x10
#define EP2_SIZE 0x40


const uint8_t usb_descriptor[] = 
{
    0x12,                   // Size of this descriptor in bytes
    DESCRIPTOR_DEV,         // DEVICE descriptor type
    USB_WORD(0x0110),       // USB Spec Release Number in BCD format
    239,                   // Class Code
    0x02,                   // Subclass code
    0x01,                   // Protocol code
    EP0_SIZE,               // Max packet size for EP0, see usb_config.h
    USB_WORD(VENDOR_ID),    // Vendor ID
    USB_WORD(PRODUCT_ID),   // Product ID: Mouse in a circle fw demo
    USB_WORD(0x0003),       // Device release number in BCD format
    0x01,                   // Manufacturer string index 
    0x02,                   // Product string index
    0x00,                   // Device serial number string index
    0x01,                   // Number of possible configurations
};

const uint8_t qual_descriptor[] = 
{
    0x0a,                   // Size of this descriptor in bytes
    DESCRIPTOR_QUAL,        // DEVICE descriptor type
    USB_WORD(0x0200),       // USB Spec Release Number in BCD format
    0x00,                   // Class Code
    0x00,                   // Subclass code
    0x00,                   // Protocol code
    EP0_SIZE,               // Max packet size for EP0, see usb_config.h
    0x01,                   // Number of possible configurations
    0x0                     // reserved
};

const uint8_t usb_config1[] =
{
    0x09,                   // Size of this descriptor in bytes
    DESCRIPTOR_CFG,         // CONFIGURATION descriptor type
    USB_WORD(70),           // Total length of data for this cfg
    2,                      // Number of interfaces in this cfg
    1,                      // Index value of this configuration
    0,                      // Configuration string index
    0x80,                   // Attributes: bus powered
    50,                    // Max power consumption (2X mA)


    /* interface association */
    0x08, // length
    0x0b, // type
    0x00, // 1st interface
    0x02, // interface count
    0x02, // class
    0x02, // subclass
    0x01, // protocol
    0x00, // function


    /* Interface Descriptor */
    0x09,                   // Size of this descriptor in bytes
    DESCRIPTOR_INTF,        // INTERFACE descriptor type
    0,                      // Interface Number
    0,                      // Alternate Setting Number
    1,                      // Number of endpoints in this intf
    0x2 ,                   // Class code: Communications
    0x2,                    // Subclass code: Abstract
    0x1,                    // Protocol code: AT commands
    0,                      // Interface string index

    /* CDC header */
    0x5,  // size
    0x24, // type
    0x0, // subtype
    0x10, 0x01, // CDC version in BCD

    /* CDC ACM */
    0x4, // size
    0x24, // type
    0x02, // subtype
    //0x06,  // capabilities: break, line coding.  Causes "failed to set dtr/rts'
    0x00,  // capabilities: none

    /* CDC Union */
    0x5, // size
    0x24, // type
    0x06, // subtype
    0x00, 0x01, // master, slave interface

    /* user endpoint descriptors */
    0x07,                       // Size of this descriptor in bytes
    DESCRIPTOR_EP,              // Endpoint Descriptor
    CDC_EP | _EP_IN,           // EndpointAddress
    EP_TYPE_INTERRUPT,          // Attributes
    USB_WORD(EP1_SIZE),         // size
    0x20,                        // Interval

    /* Interface Descriptor */
    0x09,                   // Size of this descriptor in bytes
    DESCRIPTOR_INTF,        // INTERFACE descriptor type
    1,                      // Interface Number
    0,                      // Alternate Setting Number
    2,                      // Number of endpoints in this intf
    0x0a,                   // Class code: CDC data
    0x0,                    // Subclass code: None
    0x0,                    // Protocol code: None
    0,                      // Interface string index

    /* user endpoint descriptors */
    0x07,                       // Size of this descriptor in bytes
    DESCRIPTOR_EP,              // Endpoint Descriptor
    DATA_EP | _EP_IN,            // EndpointAddress
    EP_TYPE_BULK,               // Attributes
    USB_WORD(EP2_SIZE),         // size
    0x0,                         // Interval

    /* user endpoint descriptors */
    0x07,                       // Size of this descriptor in bytes
    DESCRIPTOR_EP,              // Endpoint Descriptor
    DATA_EP | _EP_OUT,           // EndpointAddress
    EP_TYPE_BULK,               // Attributes
    USB_WORD(EP2_SIZE),         // size
    0x0                         // Interval
};

// Language code string descriptor
const uint8_t sd000[] = 
{
    0x04,      // length
    DESCRIPTOR_STR,    // descriptor type
    USB_WORD(0x0409)   // text
};

// Manufacturer string descriptor
const uint8_t sd001[] = 
{
    22,       // length
    DESCRIPTOR_STR,    // descriptor type
    'M', 0, 'c', 0, 'L', 0, 'i', 0, 
    'o', 0, 'n', 0, 'h', 0, 'e', 0, 
    'a', 0, 'd', 0,
};


// Product string descriptor
const uint8_t sd002[] = 
{
    18,
    DESCRIPTOR_STR,
    'R', 0, 'O', 0, 'B', 0, 'O', 0, 'T', 0, 'I', 0, 'C', 0, 'S', 0
};


static uint8_t usb_state;
#define USB_DETACHED_STATE 0
#define USB_ATTACHED_STATE 1
#define USB_POWERED_STATE 2
#define USB_DEFAULT_STATE 3
#define USB_ADR_PENDING_STATE 4
#define USB_ADDRESS_STATE 5
#define USB_CONFIGURED_STATE 6

static uint8_t usb_config;

static uint8_t ctrl_trf_state;
// ctrl_trf_state
#define USB_WAIT_SETUP 0
// CTRL_TRF_TX from dev to host
#define USB_WAIT_IN 1
// CTRL_TRF_RX from host to dev
#define USB_WAIT_OUT 2

static uint8_t ctrl_trf_session_owner;
// MUID = Microchip USB Class ID
// Used to identify which of the USB classes owns the current
// session of control transfer over EP0
#define MUID_NULL  0
#define MUID_USB9  1

uint16_t data_count;
const uint8_t *data_ptr;

// Buffer Descriptor Status Register Initialization Parameters
#define _BSTALL     0x04  // Buffer Stall enable
#define _DTSEN      0x08  // Data Toggle Synch enable
#define _INCDIS     0x10  // Address increment disable
#define _KEN        0x20  // SIE keeps buff descriptors enable
#define _DAT0       0x00  // DATA0 packet expected next
#define _DAT1       0x40  // DATA1 packet expected next
#define _DTSMASK    0x40  // DTS Mask
#define _USIE       0x80  // SIE owns buffer
#define _UCPU       0x00  // CPU owns buffer

#define EP0_SIZE 8

// must use dual port RAM for endpoint memory
// 18F2450
//#define USB_BANK 0x400
// 18F14K50
#define USB_BANK 0x200

volatile uint8_t *EP0_OUT = (uint8_t *)(USB_BANK);
volatile uint8_t *EP0_IN = (uint8_t *)(USB_BANK + 4);
// user endpoints
volatile uint8_t *EP1_IN = (uint8_t *)(USB_BANK + 12);
volatile uint8_t *EP2_OUT = (uint8_t *)(USB_BANK + 16);
volatile uint8_t *EP2_IN = (uint8_t *)(USB_BANK + 20);
volatile uint8_t *setup_out_packet = (uint8_t *)(USB_BANK + 24);
volatile uint8_t *setup_in_packet = (uint8_t *)(USB_BANK + 32);
volatile uint8_t *cdc_in_packet = (uint8_t *)(USB_BANK + 40);
volatile uint8_t *data_in_packet = (uint8_t *)(USB_BANK + 56);
volatile uint8_t *data_out_packet = (uint8_t *)(USB_BANK + 120);

#define USTAT_EP00_OUT    (0x00 << 3) | (0 << 2)
#define USTAT_EP00_IN     (0x00 << 3) | (1 << 2)


// buffer descriptor utilities

#define SET_EP(PTR, STAT, CNT, ADR) \
	*(PTR + 1) = (CNT); \
	*(PTR + 2) = ((uint16_t)(ADR) & 0xff); \
	*(PTR + 3) = ((uint16_t)(ADR) >> 8); \
/* STAT must be updated last */ \
	*(PTR) = (STAT);


#define SET_EP_STAT(PTR, VALUE) \
	*(PTR) = (VALUE);


#define SET_EP_CNT(PTR, VALUE) \
	*(PTR + 1) = (VALUE);

#define SET_EP_ADR(PTR, VALUE) \
	*(PTR + 2) = ((uint16_t)(VALUE) & 0xff); \
	*(PTR + 3) = ((uint16_t)(VALUE) >> 8);


#define GET_EP_CNT(PTR) \
	*(PTR + 1)

#define GET_EP_STAT(PTR) \
	*(PTR)


// if STAT.DTS is 1
#define IF_EP_DTS(PTR) \
	if(*(PTR) & (1 << 6))

// if endpoint is owned by the CPU
#define IF_EP_READY(PTR) \
	if(!(*(PTR) & (1 << 7)))

// if endpoint is not owned by the CPU
#define IF_EP_NOTREADY(PTR) \
	if((*(PTR) & (1 << 7)))



void init_usb()
{
    uint8_t i;
    for(i = 0; i < 8; i++)
    {
        EP0_OUT[i] = 0;
    }

// full speed requires a 48Mhz oscillator (page 134)
    UCFG = 0b00010100;
// slow speed is possible with a 6Mhz oscillator
    //UCFG = 0b00010000;
    UCON = 0b00001000;
    
// After enabling the USB module, it takes some time for the voltage
// on the D+ or D- line to rise high enough to get out of the SE0 condition.
// The USB Reset interrupt should not be unmasked until the SE0 condition is
// cleared. This helps prevent the firmware from misinterpreting this
// unique event as a USB bus reset from the USB host.
    while(1)
    {
        ClrWdt();
        if(!UCONbits.SE0)
        {
            break;
        }
    }

// Clear all USB interrupts
    UIR = 0;
    UIE = 0;
    PIR2bits.USBIF = 0;
    UIEbits.URSTIE = 1;
    UIEbits.STALLIE = 1;
    UIEbits.TRNIE = 1;
    PIE2bits.USBIE = 1;
    usb_state = USB_POWERED_STATE;
}


// forces EP0 OUT to be ready for a new Setup
// transaction, and forces EP0 IN to be owned by CPU.
// USBPrepareForNextSetupTrf
void usb_prepare_setup()
{
    ctrl_trf_state = USB_WAIT_SETUP;
// this is where data from the host goes
//	SET_EP EP0_OUT, _USIE | _DAT0 | _DTSEN, EP0_SIZE, setup_out_packet
	SET_EP(EP0_OUT, _USIE | _DAT0 | _DTSEN | _BSTALL, EP0_SIZE, setup_out_packet)
// EP0 IN buffer initialization
// this is where data to the host goes
    SET_EP_STAT(EP0_IN, _UCPU)
}


void handle_usb_reset()
{
//print_text("handle_usb_reset\n");
//flush_uart();
// Clears all USB interrupts
    UIR = 0;
// Reset to default address
    UADDR = 0;
// Init EP0 as a Ctrl EP
    UEP0 = 0;
    UEP0bits.EPOUTEN = 1;
    UEP0bits.EPINEN = 1;
    UEP0bits.EPHSHK = 1;
// Flush any pending transactions
    while(UIRbits.TRNIF)
    {
        UIRbits.TRNIF = 0;
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
    }
// Make sure packet processing is enabled
    UCONbits.PKTDIS = 0;
    usb_prepare_setup();
// Clear active configuration
    usb_config = 0;
    usb_state = USB_DEFAULT_STATE;
}

void handle_usb_stall()
{
    if(UEP0bits.EPSTALL)
    {
        usb_prepare_setup();
        UEP0bits.EPSTALL = 0;
    }
    
    UIRbits.STALLIF = 0;
}



void user_endpoint_init()
{
// enable the endpoint
    UEP1 = 0;
    UEP1bits.EPCONDIS = 1;
    UEP1bits.EPOUTEN = 1;
    UEP1bits.EPINEN = 1;
    UEP1bits.EPHSHK = 1;
// enable the endpoint
    UEP2 = 0;
    UEP2bits.EPCONDIS = 1;
    UEP2bits.EPOUTEN = 1;
    UEP2bits.EPINEN = 1;
    UEP2bits.EPHSHK = 1;

    SET_EP(EP1_IN, _USIE | _DAT1,           EP1_SIZE, cdc_in_packet)
//    SET_EP(EP1_OUT, _USIE | _DAT0,          EP1_SIZE, cdc_out_packet)
    SET_EP(EP2_IN,  _USIE | _DAT1,          EP2_SIZE, data_in_packet)
    SET_EP(EP2_OUT, _USIE | _DAT0 | _DTSEN, EP2_SIZE, data_out_packet)
}

void usb_check_std_request()
{
// handle setup data packet
// USBCheckStdRequest
// CTRL_TRF_SETUP.bRequest
	uint8_t request = setup_out_packet[1];


#define REQUEST_SET_ADR  0x05
#define REQUEST_GET_DSC  0x06
#define REQUEST_SET_CFG  0x09
#define REQUEST_GET_CFG  0x08
	if(request == REQUEST_SET_ADR)
    {
		ctrl_trf_session_owner = MUID_USB9;
		usb_state = USB_ADR_PENDING_STATE;
		return;
    }
    else
    if(request == REQUEST_GET_DSC)
	{
		ctrl_trf_session_owner = MUID_USB9;
// CTRL_TRF_SETUP.bDscType
		uint8_t type = setup_out_packet[3];

		if(type == DESCRIPTOR_DEV)
		{
			data_ptr = usb_descriptor;
			data_count = sizeof(usb_descriptor);
			return;
        }
        else
	    if(type == DESCRIPTOR_CFG)
        {
			data_ptr = usb_config1;
			data_count = sizeof(usb_config1);
			return;
        }
        else
        if(type == DESCRIPTOR_STR)
		{

// CTRL_TRF_SETUP.bDscIndex
			uint8_t index = setup_out_packet[2];

//print_text("C ");
//print_number(index);
//print_text("\n");
			if(index == 2)
			{
// string 2
				data_ptr = sd002;
				data_count = sizeof(sd002);
				return;
            }
            else
			if(index == 1)
			{
// string 1
				data_ptr = sd001;
				data_count = sizeof(sd001);
				return;
            }
            else
            {
// default string
			    data_ptr = sd000;
			    data_count = sizeof(sd000);
			    return;
            }
        }
        else
        if(type == DESCRIPTOR_QUAL)
        {
			data_ptr = qual_descriptor;
			data_count = sizeof(qual_descriptor);
			return;
        }
        else
        {
// This is required to stall the DEVICE_QUALIFIER request
print_text("unknown descriptor type: ");
print_number(type);
print_text("\n");
		    ctrl_trf_session_owner = MUID_NULL;
		    return;
        }
    }
    else
    if(request == REQUEST_SET_CFG)
    {
// USBStdSetCfgHandler
// This routine first disables all endpoints by clearing
// UEP registers. It then configures (initializes) endpoints
// specified in the modifiable section.
		ctrl_trf_session_owner = MUID_USB9;
// CTRL_TRF_SETUP.bCfgValue
        usb_config = setup_out_packet[2];
		if(usb_config == 0)
        {
			usb_state = USB_ADDRESS_STATE;
			return;
        }
        else
        {
			usb_state = USB_CONFIGURED_STATE;

// user configures endpoints here
			user_endpoint_init();

// done configuring.  Yay!
			return;
        }
    }
    else
    if(request == REQUEST_GET_CFG)
    {
		ctrl_trf_session_owner = MUID_USB9;
		data_ptr = &usb_config;
		data_count = 1;
		return;
    }
    else
    {
        print_text("unknown request ");
        print_number(request);
        print_text("\n");
        flush_uart();
    }
}


// USBCtrlTrfTxService
// send data to host
void handle_usb_ctrl_in()
{
// print_text("B ");
// print_number(data_count);
// print_text("\n");
// flush_uart();
// clamp fragment
	uint16_t fragment = data_count;
	if(fragment > EP0_SIZE)
    {
        fragment = EP0_SIZE;
    }

	SET_EP_CNT(EP0_IN, fragment)

// copy fragment to USB RAM
    uint8_t i;
    for(i = 0; i < fragment; i++)
    {
        setup_in_packet[i] = data_ptr[i];
    }
    data_ptr += fragment;
	data_count -= fragment;
}

// USBCtrlTrfSetupHandler
void handle_usb_ctrl_setup()
{
	ctrl_trf_state = USB_WAIT_SETUP;
	ctrl_trf_session_owner = MUID_NULL;
	data_count = 0;

// print_text("handle_usb_ctrl_setup ");
// print_hex2(setup_out_packet[0]);
// print_hex2(setup_out_packet[1]);
// print_hex2(setup_out_packet[2]);
// print_hex2(setup_out_packet[3]);
// print_hex2(setup_out_packet[4]);
// print_hex2(setup_out_packet[5]);
// print_hex2(setup_out_packet[6]);
// print_hex2(setup_out_packet[7]);
// print_text("\n");

// scan the data received
// USBCheckStdRequest
// CTRL_TRF_SETUP.RequestType
    uint8_t type = setup_out_packet[0];
	type &= 0b01100000;
// CTRL_TRF_SETUP.RequestType
// STM32CubeF7/Middlewares/ST/STM32_USB_Device_Library/Core/Inc/usbd_def.h
#define USB_REQ_TYPE_STANDARD 0x00
#define USB_REQ_TYPE_CLASS 0x20
#define USB_REQ_TYPE_VENDOR 0x40
	if(type == USB_REQ_TYPE_STANDARD)
    {
    	usb_check_std_request();
    }
    else
    if(type == USB_REQ_TYPE_CLASS)
    {
// STM32CubeF7/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c: USBD_CDC_Setup
        uint16_t length = setup_out_packet[6] |
            (setup_out_packet[7] << 8);
        if(length > 0)
        {
//print_text("USB_REQ_TYPE_CLASS\n");
            
        }
        else
        {
            uint8_t command = setup_out_packet[1];
// STM32CubeF7/Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc/usbh_cdc.h
#define CDC_SET_LINE_CODING                                     0x20U
#define CDC_GET_LINE_CODING                                     0x21U
#define CDC_SET_CONTROL_LINE_STATE                              0x22U
#define CDC_SEND_BREAK                                          0x23U
// STM32CubeF7/Projects/STM32F769I_EVAL/Applications/USB_Device/CDC_Standalone/Src/usbd_cdc_interface.c: CDC_Itf_Control
            switch(command)
            {
                case CDC_SET_LINE_CODING:
                    //print_text("CDC_SET_LINE_CODING\n");
                    break;
                case CDC_GET_LINE_CODING:
                    //print_text("CDC_GET_LINE_CODING\n");
                    break;
                case CDC_SET_CONTROL_LINE_STATE:
                    //print_text("CDC_SET_CONTROL_LINE_STATE\n");
                    break;
                case CDC_SEND_BREAK:
                    //print_text("CDC_SEND_BREAK\n");
                    break;
            }
// print_hex2(setup_out_packet[0]);
// print_hex2(setup_out_packet[1]);
// print_hex2(setup_out_packet[2]);
// print_hex2(setup_out_packet[3]);
// print_hex2(setup_out_packet[4]);
// print_hex2(setup_out_packet[5]);
// print_hex2(setup_out_packet[6]);
// print_hex2(setup_out_packet[7]);
// print_text("\n");
        }
    }

// USBCtrlEPServiceComplete
	UCONbits.PKTDIS = 0;

	if(ctrl_trf_session_owner == MUID_NULL)
    {

// If no one knows how to service this request then stall.
// Must also prepare EP0 to receive the next SETUP transaction.
		SET_EP(EP0_OUT, _USIE | _BSTALL, EP0_SIZE, setup_out_packet);
		SET_EP_STAT(EP0_IN,  _USIE | _BSTALL);
		return;
    }
    else
    {
// A module has claimed ownership of the control transfer session.
// CTRL_TRF_SETUP.DataDir
	    if(setup_out_packet[0] & (1 << 7))
        {
// SetupPkt.DataDir == DEV_TO_HOST
// clamp packet size to size in request descriptor
// CTRL_TRF_SETUP.wLength
            uint16_t length = setup_out_packet[6] |
                (((uint16_t)setup_out_packet[7]) << 8);
		    if(length < data_count)
		    {
			    data_count = length;
            }

		    handle_usb_ctrl_in();
		    ctrl_trf_state = USB_WAIT_IN;
// If something goes wrong, prepare for setup packet
		    SET_EP(EP0_OUT, _USIE, EP0_SIZE, setup_out_packet);
// Prepare for input packet
		    SET_EP_ADR(EP0_IN, setup_in_packet);
		    SET_EP_STAT(EP0_IN, _USIE | _DAT1 | _DTSEN);
		    return;


        }
        else
        {
// SetupPkt.DataDir == HOST_TO_DEV
	        ctrl_trf_state = USB_WAIT_OUT;
	        SET_EP_CNT(EP0_IN, 0);
	        SET_EP_STAT(EP0_IN, _USIE | _DAT1 | _DTSEN);

	        SET_EP(EP0_OUT, _USIE | _DAT1 | _DTSEN, EP0_SIZE, setup_in_packet);
	        return;
        }
    }
}

void handle_usb_ctrl_out()
{
// USBCtrlTrfRxService
// read data from host
    uint8_t temp;
	temp = GET_EP_CNT(EP0_OUT);
	
	data_count += temp;
	
// data is now in setup_out_packet but ignored
print_text("ctrl_out ");
print_hex2(setup_out_packet[0]);
print_hex2(setup_out_packet[1]);
print_hex2(setup_out_packet[2]);
print_hex2(setup_out_packet[3]);
print_hex2(setup_out_packet[4]);
print_hex2(setup_out_packet[5]);
print_hex2(setup_out_packet[6]);
print_hex2(setup_out_packet[7]);
print_text("\n");
}

// USTAT == EP0_OUT
void handle_usb_ctrl_output()
{
    uint8_t temp;
	temp = GET_EP_STAT(EP0_OUT);
    temp &= 0b00111100;

//print_text("ctrl_output 1\n");
//print_number(GET_EP_CNT(EP0_OUT));
//print_text("\n");
// print_hex2(EP0_OUT[0]);
// print_hex2(EP0_OUT[1]);
// print_hex2(EP0_OUT[2]);
// print_hex2(EP0_OUT[3]);
// print_text("\n");
	if(temp == (SETUP_TOKEN << 2))
    {
//print_text("ctrl_output 2\n");
    	handle_usb_ctrl_setup();
    }
    else
    {

// USBCtrlTrfOutHandler
		if(ctrl_trf_state == USB_WAIT_OUT)
		{
print_text("ctrl_output 3\n");
			handle_usb_ctrl_out();
			
			IF_EP_DTS(EP0_OUT)
			{
// STAT.DTS == 1
				SET_EP_STAT(EP0_OUT, _USIE | _DAT0 | _DTSEN)
            }
// STAT.DTS == 0
            {
			    SET_EP_STAT(EP0_OUT, _USIE | _DAT1 | _DTSEN)
			}
        }
        else
        {
//print_text("ctrl_output 4\n");
	        usb_prepare_setup();
        }
    }
}





// USBCtrlTrfInHandler
void handle_usb_ctrl_input()
{
// check if in ADR_PENDING_STATE
// mUSBCheckAdrPendingState
	if(usb_state == USB_ADR_PENDING_STATE)
    {
// SetupPkt.bDevADR
		UADDR = *(setup_out_packet + 2);

		if(!UADDR)
		{
            usb_state = USB_DEFAULT_STATE;
        }
        else
        {
			usb_state = USB_ADDRESS_STATE;
        }
    }


    if(ctrl_trf_state == USB_WAIT_IN)
	{
		handle_usb_ctrl_in();

// endpoint 0 descriptor STAT
		IF_EP_DTS(EP0_IN)
		{
// STAT.DTS == 1
			SET_EP_STAT(EP0_IN, _USIE | _DAT0 | _DTSEN)
        }
        else
        {
// STAT.DTS == 0
    		SET_EP_STAT(EP0_IN, _USIE | _DAT1 | _DTSEN)
		}
    }
    else
    {
    	usb_prepare_setup();
    }
}



void handle_usb_transaction()
{
// USBCtrlEPService
// Must do only 1 per interrupt
	if(USTAT == USTAT_EP00_OUT)
	{
    	handle_usb_ctrl_output();
	}
    else
    if(USTAT == USTAT_EP00_IN)
    {
    	handle_usb_ctrl_input();
    }

//print_bin(USTAT);
//print_text("\n");

// must process TRNIF interrupt before clearing it
	UIRbits.TRNIF = 0;
}





// MANE ------------------------------------------------------------------------




void get_pwm(uint8_t c)
{
    buffer[counter++] = c;
    if(counter >= 4)
    {
        input_state = sync_code0;
// disable interrupts
	    INTCONbits.GIE = 0;
        pwm0 = buffer[0] | (((uint16_t)buffer[1]) << 8);
        pwm1 = buffer[2] | (((uint16_t)buffer[3]) << 8);
// enable interrupts
	    INTCONbits.GIE = 1;
// print_text("get_pwm ");
// print_number(pwm0);
// print_number(pwm1);
// print_text("\n");
    }
}

void sync_code3(uint8_t c)
{
    if(c == SYNC_CODE3)
    {
//print_text("sync_code3\n");
        input_state = get_pwm;
        counter = 0;
    }
    else
    if(c == SYNC_CODE0)
    {
        input_state = sync_code1;
    }
    else
    {
        input_state = sync_code0;
    }
}

void sync_code2(uint8_t c)
{
    if(c == SYNC_CODE2)
    {
//print_text("sync_code2\n");
        input_state = sync_code3;
    }
    else
    if(c == SYNC_CODE0)
    {
        input_state = sync_code1;
    }
    else
    {
        input_state = sync_code0;
    }
}

void sync_code1(uint8_t c)
{
    if(c == SYNC_CODE1)
    {
//print_text("sync_code1\n");
        input_state = sync_code2;
    }
    else
    if(c == SYNC_CODE0)
    {
        input_state = sync_code1;
    }
    else
    {
        input_state = sync_code0;
    }
}

void sync_code0(uint8_t c)
{
    if(c == SYNC_CODE0)
    {
//print_text("sync_code0\n");
        input_state = sync_code1;
    }
}



int main(int argc, char** argv) 
{
// digital mode
    ANSEL = 0b00000000;
    ANSELH = 0b00000000;

// debug UART
    init_uart();
    delayMicroseconds(10000);
    

// print to debug UART
    print_byte(0xff);
    print_text("\n\n\n\nWelcome to servo/IR controller\n");
    flush_uart();

    SERVO0_LAT = 0;
    SERVO1_LAT = 0;
    SERVO0_TRIS = 0;
    SERVO1_TRIS = 0;
    

// servo 0 timer
    T3CON = 0b10000001;
    TMR3IF = 0;
    TMR3IE = 1;

// servo 1 timer
    T1CON = 0b10000001;
    TMR1IF = 0;
    TMR1IE = 1;
    

// IR timer.  1:16 prescale
    T0CON = 0b10000011;
    TMR0IF = 0;
    TMR0IE = 1;
// IR interrupt
    INT0IE = 1;
// falling edge
    INTEDG0 = 0;

// HZ timer
    T2CON = 0b01111111;
    PR2 = (CLOCKSPEED / 4 / 16 / 16 / HZ) - 1;
    TMR2IF = 0;
    TMR2IE = 1;

    init_usb();

    INTCONbits.PEIE = 1;
	INTCONbits.GIE = 1;

    while(1)
    {
        ClrWdt();
        handle_uart();

//DEBUG_UART_LAT = PORTBbits.RB5;

// Get data from the host
        IF_EP_READY(EP2_OUT)
        {
// print_text("OUT ");
// print_number(GET_EP_CNT(EP2_OUT));
// print_text("\n");

            uint8_t i;
            for(i = 0; i < GET_EP_CNT(EP2_OUT); i++)
            {
                input_state(data_out_packet[i]);
            }


//             for(i = 0; i < GET_EP_CNT(EP2_OUT); i++)
//             {
//                 print_hex2(data_out_packet[i]);
//             }
//             print_text("\n");


// prepare for next receive
            IF_EP_DTS(EP2_OUT)
            {
                SET_EP(EP2_OUT, _USIE | _DAT0 | _DTSEN, EP2_SIZE, data_out_packet)
            }
            else
            {
                SET_EP(EP2_OUT, _USIE | _DAT1 | _DTSEN, EP2_SIZE, data_out_packet)
            }
        }


        if(flags.got_ir)
        {
            flags.got_ir = 0;
            handle_ir(ir_code);
        }


// pass IR codes to USB
        if(ir_count > 0)
        {
            IF_EP_READY(EP2_IN)
            {
// disable interrupt
                PIE1bits.RCIE = 0;
                uint8_t i = 0;
                uint8_t fragment = ir_count;

//print_text("IN ");
//print_number(fragment);
//print_text("\n");
                if(fragment > EP2_SIZE)
                {
                    fragment = EP2_SIZE;
                }
                for(i = 0; i < fragment; i++)
                {
                    data_in_packet[i] = ir_buffer[ir_ptr2++];
                    if(ir_ptr2 >= IR_SIZE)
                    {
                        ir_ptr2 = 0;
                    }
                }
                ir_count -= fragment;
// enable interrupt
                PIE1bits.RCIE = 1;

                IF_EP_DTS(EP2_IN)
                {
                    SET_EP(EP2_IN, _USIE | _DAT0 | _DTSEN, fragment, data_in_packet)
                }
                else
                {
                    SET_EP(EP2_IN, _USIE | _DAT1 | _DTSEN, fragment, data_in_packet)
                }
            }
        }
    }

    return (EXIT_SUCCESS);
}



void __interrupt(low_priority) isr1()
{
}

void __interrupt(high_priority) isr()
{
    flags.interrupt_complete = 0;
	while(!flags.interrupt_complete)
	{
		flags.interrupt_complete = 1;

// have to do USB in the interrupt handler
        if(PIR2bits.USBIF)
        {
            flags.interrupt_complete = 0;
            PIR2bits.USBIF = 0;
            if(UIRbits.URSTIF)
            {
                handle_usb_reset();
            }

            if(UIRbits.STALLIF)
            {
                handle_usb_stall(); 
            } 

            if(UIRbits.TRNIF) 
            {
                handle_usb_transaction(); 
            }
        }


        if(TMR3IF)
        {
            TMR3IF = 0;
            flags.interrupt_complete = 0;
            if(SERVO0_LAT)
            {
                SERVO0_LAT = 0;
            }
        }

        if(TMR1IF)
        {
            TMR1IF = 0;
            flags.interrupt_complete = 0;
            if(SERVO1_LAT)
            {
                SERVO1_LAT = 0;
            }
        }

        if(TMR2IF)
        {
            TMR2IF = 0;
            flags.interrupt_complete = 0;
            tick++;
            if(tick >= HZ / 50)
            {
                tick = 0;

                if(pwm0 > 0)
                {
                    SERVO0_LAT = 1;
                    TMR3 = -pwm0;
                    TMR3IF = 0;
                }

                if(pwm1 > 0)
                {
                    SERVO1_LAT = 1;
                    TMR1 = -pwm1;
                    TMR1IF = 0;
                }
            }
        }


        if(TMR0IF)
        {
            TMR0IF = 0;
            flags.interrupt_complete = 0;
            if(!flags.ir_wrap)
            {
                flags.ir_wrap = 1;
                flags.got_ir = 1;
                ir_code = IR_TIMEOUT;
                print_text("IR TIMEOUT\n");
            }
        }

// IR code
        if(INT0IF)
        {
            INT0IF = 0;
// get the time
            uint16_t value = TMR0;
            TMR0 = 0;

// ignore if the timer wrapped
            if(flags.ir_wrap)
            {
                flags.ir_wrap = 0;
            }
            else
            {
                flags.got_ir = 1;
                if(value >= IR_REPEAT_TIME)
                    ir_code = IR_REPEAT;
                else
                if(value >= IR_LOW_TIME) 
                    ir_code = IR_HIGH;
                else
                    ir_code = IR_LOW;
                print_text("IR: ");
                print_number(ir_code);
                print_text("\n");
            }
        }

        if(PIR1bits.RCIF)
        {
//DEBUG_LAT = !DEBUG_LAT;
            handle_uart_rx();
        }
    }
}















