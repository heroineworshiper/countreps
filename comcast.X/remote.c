/*
 * Unlock a Comca$t XR16 remote control
 *
 * Copyright (C) 2023 Adam Williams <broadcast at earthling dot net>
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



// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection Bits (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = ON        // Internal/External Switchover Mode (Internal/External Switchover Mode is enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

#include <pic16f1508.h>
#include <stdint.h>
#include <string.h>

#define CLOCKSPEED 16000000

#define UART_BUFSIZE 64
uint8_t uart_buffer[UART_BUFSIZE];
uint8_t uart_size = 0;
uint8_t uart_position1 = 0;
uint8_t uart_position2 = 0;

#define LED_LAT LATA2
#define LED_TRIS TRISA2
#define PWM_TRIS TRISC1
// button matrix
// rows
#define TP20 PORTCbits.RC3
#define TP21 PORTBbits.RB6
#define TP22 PORTCbits.RC5
#define TP23 PORTCbits.RC4

// columns
#define TP27 PORTCbits.RC7
#define TP28 PORTCbits.RC2
#define TP31 PORTBbits.RB4
#define TP32 PORTCbits.RC6

// stop & restart the timer to set it.  page 156
#define SET_TIMER1(x) \
    TMR1ON = 0; \
    TMR1H = (x) >> 8; \
    TMR1L = (x) & 0xff; \
    TMR1ON = 1;

// timer1 values with 1:8 prescale
// .1ms at 16Mhz
#define BUTTON_TIMEOUT -50000
// 500us at 16Mhz
#define IR_PULSE -250
#define IR_LOW -250
// 1500us at 16Mhz
#define IR_HIGH -750
// 5ms at 16Mhz
#define BUTTON_REPEAT -2500

typedef union
{
    struct
    {
        unsigned tp27 : 1;
        unsigned tp28 : 1;
        unsigned tp31 : 1;
        unsigned tp32 : 1;
    };
    unsigned char value;
} columns_t;

columns_t current_columns;
columns_t prev_columns;

// the starting code
const uint8_t CODE[] = { 0xac, 0xf4, 0x56, 0x6a };

// the pressed button
uint8_t button;
#define NO_BUTTON 0
#define STAR 1
#define I_BUTTON 2
#define UP 3
#define BACK 4
#define MIC 5
#define HOME 6
#define LEFT 7
#define RIGHT 8
#define SELECT 9
#define DOWN 10
#define TV_INPUT 11
#define POWER 12
#define PLUS 13
#define MUTE 14
#define MINUS 15

const uint8_t *button_strings[] = 
{
    "NO_BUTTON",
    "STAR",
    "I_BUTTON",
    "UP",
    "BACK",
    "MIC",
    "HOME",
    "LEFT",
    "RIGHT",
    "SELECT",
    "DOWN",
    "TV_INPUT",
    "POWER",
    "PLUS",
    "MUTE",
    "MINUS",
};

void send_uart(uint8_t c)
{
	if(uart_size < UART_BUFSIZE)
	{
		uart_buffer[uart_position1++] = c;
		uart_size++;
		if(uart_position1 >= UART_BUFSIZE)
		{
			uart_position1 = 0;
		}
	}
}

static uint16_t number;
static int force;
void print_digit(uint16_t place)
{
	if(number >= place || force)
	{ 
		force = 1; 
		send_uart('0' + number / place); 
		number %= place; 
	}
}

void print_number(uint16_t number_arg)
{
	number = number_arg;
	force = 0;
	print_digit(10000000);
	print_digit(1000000);
	print_digit(100000);
	print_digit(10000);
	print_digit(1000);
	print_digit(100);
	print_digit(10);
	send_uart('0' + (number % 10)); 
	send_uart(' ');
}

void print_text(const uint8_t *s)
{
	while(*s != 0)
	{
		send_uart(*s);
		s++;
	}
}

void delay(uint16_t x)
{
    x += 16;
    SET_TIMER1(x);
    TMR1IF = 0;
    while(!TMR1IF) 
    {
    }
}

void send_code(uint8_t x)
{
    uint8_t i;
    for(i = 0; i < 8; i++)
    {
        if((x & 0x80)) 
            delay(IR_HIGH);
        else
            delay(IR_LOW);
        x <<= 1;

        PWM_TRIS = 0;
        delay(IR_PULSE);
        PWM_TRIS = 1;
    }
}

void main()
{
// 2 Mhz internal clock
//    OSCCON = 0b01100000;
// 16 Mhz
    OSCCON = 0b01111000;
// serial port
    TXSTA = 0b00100100;
// disable receive
    RCSTA = 0b10000000;
    BAUDCON = 0b00001000;
// 100kbaud
    SPBRG = CLOCKSPEED / 4 / 100000;
// digital mode
    ANSELA = 0;
    ANSELB = 0;
    ANSELC = 0;

// button timeout & IR modulation
// 1:8 prescaler
// page 162
    T1CON = 0b00110001;

// disable IR
    PWM_TRIS = 1;

    LED_LAT = 0;
    LED_TRIS = 0;

// enable PWM4/C1
    T2CON = 0b00000100;
// 38khz at 2Mhz
//    PR2 = 12;
// 38khz at 16Mhz
    PR2 = 104;
    PWM4DCH = PR2 / 2;
    PWM4CON = 0b11000000;


    print_text("\n\n\n\nWelcome to Comca$t XR16\n");
    current_columns.value = 0;
    prev_columns.value = 0;
    button = NO_BUTTON;

// DEBUG
// while(1)
// {
// delay(BUTTON_REPEAT);
// PWM_TRIS = !PWM_TRIS;
// }

    while(1)
    {
// send a UART char
// must use TRMT instead of TXIF on the 16F1508
        if(uart_size > 0 && TRMT)
        {
            TXREG = uart_buffer[uart_position2++];
		    uart_size--;
		    if(uart_position2 >= UART_BUFSIZE)
		    {
			    uart_position2 = 0;
		    }
        }

// probe columns
        prev_columns = current_columns;
// probe columns 1st since rows are always GND after the column edge
        current_columns.tp27 = TP27;
        current_columns.tp28 = TP28;
        current_columns.tp31 = TP31;
        current_columns.tp32 = TP32;

        uint8_t button2 = NO_BUTTON;
// Check column edge
        if(prev_columns.tp27 && !current_columns.tp27)
        {
//            print_text("TP27\n");
// Check row DC
            if(!TP20) button2 = STAR;
            if(!TP21) button2 = MINUS;
            if(!TP22) button2 = I_BUTTON;
            if(!TP23) button2 = UP;
        }
        
        if(prev_columns.tp28 && !current_columns.tp28)
        {
//            print_text("TP28\n");
// Check row DC
            if(!TP20) button2 = BACK;
            if(!TP21) button2 = MIC;
            if(!TP22) button2 = HOME;
        }
        
        if(prev_columns.tp31 && !current_columns.tp31)
        {
//            print_text("TP31\n");
// Check row DC
            if(!TP20) button2 = LEFT;
            if(!TP21) button2 = SELECT;
            if(!TP22) button2 = RIGHT;
            if(!TP23) button2 = DOWN;
        }
        
        if(prev_columns.tp32 && !current_columns.tp32)
        {
//            print_text("TP32\n");
// Check row DC
            if(!TP20) button2 = TV_INPUT;
            if(!TP21) button2 = POWER;
            if(!TP22) button2 = PLUS;
            if(!TP23) button2 = MUTE;
        }
        
        if(button2 != NO_BUTTON)
        {
            LED_LAT = 1;

// send start code
            if(button2 != button)
            {
                print_text(button_strings[button2]);
                print_text("\n");
                button = button2;


// sometimes reboots when it starts
// start bit
                PWM_TRIS = 0;
                delay(IR_PULSE);
                PWM_TRIS = 1;
                send_code(CODE[0] ^ button);
                send_code(CODE[1] ^ button);
                send_code(CODE[2] ^ button);
                send_code(CODE[3] ^ button);
            }
            else
// send repeat code
            {
                PWM_TRIS = 0;
                delay(BUTTON_REPEAT);
                PWM_TRIS = 1;
                print_text("repeat\n");
            }

// reset timeout
            SET_TIMER1(BUTTON_TIMEOUT)
            TMR1IF = 0;
        }

// button timed out
        if(TMR1IF && button != NO_BUTTON)
        {
            print_text("released\n");
            button = NO_BUTTON;
            TMR1IF = 0;
            LED_LAT = 0;
        }
    }
}









