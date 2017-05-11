/*
 * Go2Goal.cpp
 *
 * Created: 15-04-2017 23:02:31
 * Author : Harshit
 */ 


#define F_CPU 8000000UL
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <math.h>
#include "compass_sensor.h"
#include "USART_128.h"
#include "PID_MovingArray.h"
#include "GoToGoal.h"
#include "Common_AVR.h"

struct Go2Goal::differentialState Go2Goal::curDiffState;		//Static Declaration
MovingArray Go2Goal::LeftWheelSpeed;
MovingArray Go2Goal::RightWheelSpeed;
long Go2Goal::ticks[2];


uint8_t unitTimeCount=0,timekeeper=0;
uint16_t i;

Go2Goal MainGoal;


int main(void)
{
    Com_Init();
    
    MainGoal.LeftWheel.Init(0.26,0.0,1.68);   //Set Kp,Ki and Kd for PIDs
    MainGoal.RightWheel.Init(0.26,0.0,1.65);
    MainGoal.CurPhi.Init(0.01745*5,0.0,0.0);	//No PID at 0.01745
    
    Go2Goal::LeftWheelSpeed.Init(10);
    Go2Goal::RightWheelSpeed.Init(10);
	
	MainGoal.desiredBotPosition.x = 80;
	MainGoal.desiredBotPosition.y = -80;
	
	MainGoal.Phi_Refernce = getHeading();
	_delay_ms(100);
	
	do 
	{
		MainGoal.calculatePos(getHeading());
		_delay_ms(100);
	}
	while((MainGoal.curBotPosition.phi < 85 && MainGoal.curBotPosition.phi > 95));
	
	_delay_ms(1000);
	MainGoal.curBotPosition.x = 0;
	MainGoal.curBotPosition.y = 0;
	
	Timer_Init();
	
    while (1) 
    {
			
		
    }
}

ISR(TIMER0_OVF_vect) 
{	
	unitTimeCount++;
	timekeeper++;
	
	if(timekeeper == 2)
	{
		Go2Goal::calculateDiffState();
		timekeeper = 0;
	}	
	
	MainGoal.calculatePos(getHeading());
	
	if(unitTimeCount == PIDLoop_mainLoop_ratio)
	{
		MainGoal.desiredDiffState = MainGoal.transformUniToDiff(MainGoal.getDesiredUnicycleState(MainGoal.curBotPosition,MainGoal.desiredBotPosition));
		unitTimeCount = 0;
	}
	
	MainGoal.getWheelOutputs(Go2Goal::curDiffState,MainGoal.desiredDiffState);
	
	ActuateLeftWheel(MainGoal.LeftWheelPWM.Dir,lround(MainGoal.LeftWheelPWM.PWM));
	ActuateRightWheel(MainGoal.RightWheelPWM.Dir,lround(MainGoal.RightWheelPWM.PWM));
	
	
	USART_TransmitNumber(MainGoal.curBotPosition.x,0);
	USART_Transmitchar(0x0A,0);
	
	USART_TransmitNumber(MainGoal.curBotPosition.y,0);
	USART_Transmitchar(0x0A,0);
	
	USART_TransmitNumber(MainGoal.curBotPosition.phi,0);
	USART_Transmitchar(0x0D,0);
	
	/*USART_TransmitNumber(Go2Goal::curDiffState.leftRPM,0);
	USART_Transmitchar(0x0A,0);
	
	USART_TransmitNumber(Go2Goal::curDiffState.rightRPM,0);
	USART_Transmitchar(0x0A,0);
	
	USART_TransmitNumber(MainGoal.desiredDiffState.leftRPM,0);
	USART_Transmitchar(0x0A,0);
	
	USART_TransmitNumber(MainGoal.desiredDiffState.rightRPM,0);
	USART_Transmitchar(0x0D,0);*/
	
	/*USART_TransmitNumber(leftPWM,0);
	USART_Transmitchar(0x0A,0);
	
	USART_TransmitNumber(rightPWM,0);
	USART_Transmitchar(0x0D,0);*/
	
	
}

 ISR(INT2_vect) 
 {
	 if(bit_is_clear(PIND,4))
	 Go2Goal::ticks[MainGoal.left]++;
	 else if(bit_is_set(PIND,4))
	 Go2Goal::ticks[MainGoal.left]--;
 }


 ISR(INT3_vect) 
 {
	 if(bit_is_clear(PIND,7))
	 Go2Goal::ticks[MainGoal.right]++; 
	 else if(bit_is_set(PIND,7))
	 Go2Goal::ticks[MainGoal.right]--;
 }
 

