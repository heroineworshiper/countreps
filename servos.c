/*
 * tracking camera
 * Copyright (C) 2019 Adam Williams <broadcast at earthling dot net>
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

// this runs on an atmega to drive the servos with PWM

// reads packets starting with the sync code.  No interactive mode

// compile with make servos
// program fuses: make servos_fuse
// program with make servos_isp






#include "avr_debug.h"
#include "servos.h"
#include <string.h> // memcpy
#include <stdint.h>


// pins
#define PAN_PIN 0
#define TILT_PIN 1
#define PAN_PORT PORTC
#define PAN_DDR DDRC
#define TILT_PORT PORTC
#define TILT_DDR DDRC

#define SYNC_CODE0 0xff
#define SYNC_CODE1 0x2d
#define SYNC_CODE2 0xd4
#define SYNC_CODE3 0xe5

#define DEBUG_PIN 2
#define DEBUG_PORT PORTD
#define DEBUG_DDR DDRD

#define MAX_PWM 32768
#define MIN_PWM 16384
#define MID_PWM ((MAX_PWM + MIN_PWM) / 2)

void (*input_state)();
uint8_t buffer[4];
uint8_t counter;
uint8_t pwm_started = 0;

// with DSLR
uint16_t pan = 24976;
uint16_t tilt = 16976;

// with webcam for testing
//uint16_t pan = 24976;
//uint16_t tilt = 25776;

#define TABLE_SIZE 8
pwm_table_t pwm_table[TABLE_SIZE];
pwm_table_t new_table[TABLE_SIZE];
volatile uint8_t pwm_table_size = 0;
volatile uint8_t new_table_size = 0;
volatile uint8_t have_new_table = 0;


uint8_t isr_table_offset = 0;
ISR(TIMER1_OVF_vect)
{
//    bitToggle(PAN_PORT, PAN_PIN);
//    bitToggle(TILT_PORT, TILT_PIN);
    if(isr_table_offset == 0 && have_new_table)
    {
        memcpy(pwm_table, new_table, sizeof(pwm_table_t) * new_table_size);
        pwm_table_size = new_table_size;
    }
    
    TCNT1 = pwm_table[isr_table_offset].time;
    PAN_PORT = pwm_table[isr_table_offset].value;
    isr_table_offset++;
    if(isr_table_offset >= pwm_table_size)
    {
        isr_table_offset = 0;
    }
}


void make_pwm_table()
{
// 50Hz downtime
    uint8_t i;
    for(i = 0; i < 4; i++)
    {
        new_table[i].time = 0;
        new_table[i].value = 0;
    }
//     
//     new_table[i].time = 32767;
//     new_table[i].value = 0;
//     i++;

    uint16_t min_pwm = MIN(pan, tilt);
    uint16_t max_pwm = MAX(pan, tilt);
    uint16_t on_time = 65535 - min_pwm;
// pulse on time
    new_table[i].time = on_time;
    new_table[i].value = 0x3;
    i++;
    
// pulse off time
    uint16_t off_time = max_pwm - min_pwm;
    if(off_time == 0)
    {
        off_time = max_pwm;
    }

    new_table[i].time = 65535 - off_time;
    if(pan == tilt)
    {
        new_table[i].value = 0;
    }
    else
    if(pan < tilt)
    {
        new_table[i].value = 0x3 ^ (1 << PAN_PIN);
    }
    else
    {
        new_table[i].value = 0x3 ^ (1 << TILT_PIN);
    }
    i++;

    if(pan != tilt)
    {
        off_time = MAX(pan, tilt);
        new_table[i].time = 65535 - off_time;
        new_table[i].value = 0;
        i++;
    }
    
    new_table_size = i;
    have_new_table = 1;
    
    
//     for(i = 0; i < new_table_size; i++)
//     {
//         print_text("i=");
//         print_number(i);
//         print_text("TCNT1=");
//         print_number(new_table[i].time);
//         print_text("PORT=");
//         print_number(new_table[i].value);
//         print_byte('\n');
//     }
}

void sync_code0();
void sync_code1();


void get_pwm()
{
    buffer[counter++] = uart_in;
    if(counter >= 4)
    {
        input_state = sync_code0;
        pan = buffer[0] | (((uint16_t)buffer[1]) << 8);
        tilt = buffer[2] | (((uint16_t)buffer[3]) << 8);
        
//print_text("get_pwm\n");
        if(!pwm_started)
        {
//print_text("starting PWM\n");
            pwm_started = 1;
    // enable the timer
            bitSet(TIMSK1, TOIE1);
        }

        make_pwm_table();
    }
}

void sync_code3()
{
    if(uart_in == SYNC_CODE3)
    {
//print_text("sync_code3\n");
        input_state = get_pwm;
        counter = 0;
    }
    else
    if(uart_in == SYNC_CODE0)
    {
        input_state = sync_code1;
    }
    else
    {
        input_state = sync_code0;
    }
}

void sync_code2()
{
    if(uart_in == SYNC_CODE2)
    {
//print_text("sync_code2\n");
        input_state = sync_code3;
    }
    else
    if(uart_in == SYNC_CODE0)
    {
        input_state = sync_code1;
    }
    else
    {
        input_state = sync_code0;
    }
}

void sync_code1()
{
    if(uart_in == SYNC_CODE1)
    {
//print_text("sync_code1\n");
        input_state = sync_code2;
    }
    else
    if(uart_in == SYNC_CODE0)
    {
        input_state = sync_code1;
    }
    else
    {
        input_state = sync_code0;
    }
}

void sync_code0()
{
    if(uart_in == SYNC_CODE0)
    {
//print_text("sync_code0\n");
        input_state = sync_code1;
    }
}


int main()
{
	WDTCSR = 0;


// verify the prescaler is 0
	CLKPR = 0x80;
	CLKPR = 0x00;

	init_serial();
	print_text("Servo driver\n");
    make_pwm_table();
    
// set pin to enable output
//	bitSet(DEBUG_DDR, DEBUG_PIN);
//	bitClear(DEBUG_PORT, DEBUG_PIN);

// PWM pins
	bitClear(PAN_PORT, PAN_PIN);
	bitClear(TILT_PORT, TILT_PIN);

// set pin to enable output
//	bitClear(PAN_DDR, PAN_PIN);
//	bitClear(TILT_DDR, TILT_PIN);

    
// enable the timer
	TCCR1B = (1 << CS10);
//    bitSet(TIMSK1, TOIE1);
    bitClear(TIMSK1, TOIE1);
    
// enable interrupts
	sei();
    
    input_state = sync_code0;

	while(1)
	{
		handle_serial();
		if(have_uart_in)
		{
			have_uart_in = 0;
			input_state();
		}
	}
}












