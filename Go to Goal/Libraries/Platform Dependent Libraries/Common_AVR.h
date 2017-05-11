/*
 * Common_AVR.h
 *
 * Created: 11-05-2017 01:12:09
 *  Author: Harshit
 */ 


#ifndef COMMON_AVR_H_
#define COMMON_AVR_H_

#define F_CPU 8000000UL
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "compass_sensor.h"
#include "USART_128.h"

#define leftPWM		OCR3A
#define rightPWM	OCR3B

void Com_Init(void);
void Timer_Init(void);
void ActuateLeftWheel(bool dir,uint16_t PWM);
void ActuateRightWheel(bool dir,uint16_t PWM);



#endif /* COMMON_AVR_H_ */