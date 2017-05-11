/*
 * GoToGoal.h
 *
 * Created: 15-04-2017 23:04:23
 *  Author: Harshit
 */ 


#ifndef GOTOGOAL_H_
#define GOTOGOAL_H_


#include <math.h>
#include "PID_MovingArray.h"


 #define PIDLoop_mainLoop_ratio		1
 #define pi							3.1416
 #define timeInterval				0.032768			// 1024 * 255 / F_CPU // in sec
 #define TimeConstant				0.91911764
		
 #define ticksPerRotation			1000
 #define r							5.0
 #define L							22.2
 #define circumference				31.4				//2 * pi * r
 #define vmax						70
 #define MaxPWM						1000



class Go2Goal
{
		
	public:
	
		struct differentialState {float leftRPM; float rightRPM;};
		struct position {float x; float y; int phi;};
		struct unicycleState {float v; float w;};
		struct wheelSpeed {bool Dir; float PWM; };

		enum {left, right};					//wheel
		enum {lRPM, rRPM, angularVel};		// movingArray, PID
			
		PID LeftWheel;
		PID RightWheel;
		PID CurPhi;
		
		int16_t Phi_Refernce;
			
		struct position curBotPosition;
		struct position desiredBotPosition;
		struct differentialState desiredDiffState;
		
		struct unicycleState getDesiredUnicycleState(struct position curBotPosition, struct position desiredBotPosition);
		struct differentialState transformUniToDiff(struct unicycleState uniState);
			
		void calculatePos(int16_t Headings);  
		void getWheelOutputs(struct differentialState curState, struct differentialState desiredState);
		
		static struct differentialState curDiffState;
		
		static MovingArray LeftWheelSpeed;
		static MovingArray RightWheelSpeed;
		
		static long ticks[2];
		
		static void calculateDiffState();
		
		struct wheelSpeed LeftWheelPWM,RightWheelPWM;
	
};








#endif /* GOTOGOAL_H_ */
