/*
 * Common_AVR.cpp
 *
 * Created: 11-05-2017 01:13:51
 *  Author: Harshit
 */ 
#include "Common_AVR.h"

void Com_Init(void)
{
	sei();
	USART_Init(51,0);
	_delay_ms(100);
	init_HMC5883L();
	_delay_ms(100);				// time to let compass sensor load
	
	DDRE = (1<<PINE2) | (1<<PINE3) | (1<<PINE4) | (1<<PINE5);
	
	//interrupt , any logical change
	EICRA = (1<<ISC20) | (1<<ISC21)| (1<<ISC30) | (1<<ISC31);
	EIMSK = (1<<INT2) | (1<<INT3);
	
	//PWM_timer , Fast_PWM_mode, Top = 0x03FF(in Hex) or 1023(in Decimal)
	TCCR3B = (1<<CS30) | (1<<WGM32);
	TCCR3A = (1<<COM3A1) | (1<<COM3B1) | (1 << WGM31) | (1<< WGM30);
	
}

void Timer_Init(void)
{
	TCCR0 = (1<<CS02) | (1<<CS01) | (1<<CS00);
	TIMSK = (1<<TOIE0);
}

void ActuateLeftWheel(bool dir , uint16_t PWM)
{
	if(dir)
	{
		PORTE|=(1<<PINE2);
	}
	else
	{
		PORTE&=~(1<<PINE2);
	}
	
	leftPWM = PWM;
}

void ActuateRightWheel(bool dir , uint16_t PWM)
{
	if(dir)
	{
		PORTE|=(1<<PINE5);
	}
	else
	{
		PORTE&=~(1<<PINE5);
	}
	
	rightPWM = PWM;	
}