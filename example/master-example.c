/*
 *  Created: 23.03.2019
 *  Author: Max Brueggemann
 */ 

/*
*	An example project implementing a modbus master device using an
*	ATmega88PA running at 20MHz.
*	Baudrate: 38400, 8 data bits, 1 stop bit, no parity
*	This code is going to:
*	1. read holding registers 0 to 3 from client device 1
*	2. wait 1 second
*	3. write the value x to register 0 at client device 1
*	4. increment x
*	5. wait 1 second
*	and then start again at 1.
*/

/*
 *	** A word on the busmaster capabilities of yaMBSiavr **
 *	yaMBSiavr has been solely written to be a modbus server library. 
 *	While it is possible to use it as a modbus master, it cannot be
 *	done without going a little deeper into the modbus protocol.
 *	This example shall serve as a guideline for those who want to do
 *	it anyway :-)
 *
*/

#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#define F_CPU 20000000
#include "yaMBSiavr.h"

uint16_t holdingRegisters[4];

void timer0100us_start(void) {
	TCCR0B|=(1<<CS01); //prescaler 8
	TIMSK0|=(1<<TOIE0);
}

ISR(TIMER0_OVF_vect) { //this ISR is called 9765.625 times per second
	modbusTickTimer();
}

void modbusGet(void) {
	if (modbusGetBusState() & (1<<ReceiveCompleted))
	{
		modbusSendException(0x01);
	}
}


void readReg(uint8_t slaveid, uint16_t address, uint8_t amount) {
        _delay_ms(2);
        rxbuffer[0]=slaveid;
	modbusSetAddress(slaveid);
        rxbuffer[1]=0x03;
        rxbuffer[2]=(address>>8)&0xFF;
        rxbuffer[3]=address&0xFF;
        rxbuffer[4]=0x00;
        rxbuffer[5]=amount;
        modbusSendMessage(5);
}

void writeReg(uint8_t slaveid, uint16_t address, uint16_t value) {
        _delay_ms(2);
        rxbuffer[0]=slaveid;
	modbusSetAddress(slaveid);
        rxbuffer[1]=0x06;
        rxbuffer[2]=(address>>8)&0xFF;
        rxbuffer[3]=address&0xFF;
        rxbuffer[4]=(value>>8)&0xFF;;
        rxbuffer[5]=value&0xFF;
        modbusSendMessage(5);
}

#define receiveOkay (modbusGetBusState() & (1<<ReceiveCompleted))


int main(void)
{
	sei();
	modbusSetAddress(1); //better set this to sth.
	modbusInit();
	timer0100us_start();

    while(1)
    {
	wdt_reset();
	_delay_ms(1000);

	/*********************/
	/*  Read 4 registers */
	readReg(1,0,4);

	uint8_t breaker = 100;
	while(!receiveOkay && breaker) { //wait for client response, time out after 1s
		breaker--;
		_delay_ms(10);	
	}

	if(receiveOkay) { //if this fails, there was either no response or a crc error
		if(rxbuffer[1]&0x80) { //client responded with an error code
			//handle the error
		}
		else {
			for(uint8_t x=0;x<4;x++) { //rxbuffer[2] should be 8 (4 registers => 8 bytes). You might want to check this at this point.
				holdingRegisters[x]=(rxbuffer[3+x*2]<<8)+rxbuffer[4+x*2]; //do sth with the acquired data.
			}
		}

	}


	_delay_ms(1000);
	/*********************/
	/*  Write 1 register */
	static uint16_t incr = 0;
	incr++;
	writeReg(1,0,incr);
	
	breaker = 100;
	while(!receiveOkay && breaker) { //wait for client response, time out after 1s
		breaker--;
		_delay_ms(10);	
	}

	if(receiveOkay) { //if this fails, there was either no response or a crc error
		if(rxbuffer[1]&0x80) { //client responded with an error code
			//handle the error
		}
	}
	/*******************/
	
	
    }
}
