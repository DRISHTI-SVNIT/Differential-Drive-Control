#include "GoToGoal.h"


void Go2Goal::calculateDiffState(void)
{
	curDiffState.leftRPM = LeftWheelSpeed.ImplementMovingArray(ticks[left]*TimeConstant);
	curDiffState.rightRPM = RightWheelSpeed.ImplementMovingArray(ticks[right]*TimeConstant);
	ticks[left] = 0;
	ticks[right] = 0;
}



struct Go2Goal::unicycleState Go2Goal::getDesiredUnicycleState(struct position curBotPosition, struct position desiredBotPosition) 
{
	 struct unicycleState desiredState;
	 
	 int errDist; 
	 errDist = sqrt((desiredBotPosition.x - curBotPosition.x) * (desiredBotPosition.x -curBotPosition.x) + (desiredBotPosition.y - curBotPosition.y)*(desiredBotPosition.y - curBotPosition.y));
	 int desiredPhi = 0;
	 
	 if((curBotPosition.x - desiredBotPosition.x) == 0) 
	 {
		 if(desiredBotPosition.y > curBotPosition.y) 
		 {
			 desiredPhi = 90;
		 } 
		 else 
		 {
			 desiredPhi = -90;
		 }
	 } 
	 else if((curBotPosition.y - desiredBotPosition.y) == 0) 
	 {
		 if(desiredBotPosition.x > curBotPosition.x) 
		 {
			 desiredPhi = 0;
		 } 
		 else 
		 {
			 desiredPhi = 180;
		 }
	 } 
	 else 
	 {
		desiredPhi = radToDegree(atan2((desiredBotPosition.y - curBotPosition.y) , (desiredBotPosition.x - curBotPosition.x)));
	 }
	 
	 desiredState.v =  vmax * sigmoid(errDist,45.0);
	 desiredState.w = CurPhi.ImplementPID(normalizeAngle(desiredPhi - curBotPosition.phi));
	 
	 return desiredState;
}

struct Go2Goal::differentialState Go2Goal::transformUniToDiff(struct unicycleState uniState)
{
	 struct differentialState diffState;
	 
	 uniState.w *= sigmoid(uniState.v,10.0);
	 
	 float vleft = (2*uniState.v -L*uniState.w) / (2 * r);
	 float vright = (2*uniState.v + L*uniState.w)/(2 * r);
	 diffState.rightRPM = vright / circumference * 60;
	 diffState.leftRPM = vleft / circumference * 60;
	 return diffState;
}





void Go2Goal::calculatePos(int16_t Headings) 
{
	 curBotPosition.phi = normalizeAngle(90 - Headings + Phi_Refernce);
	 float leftDist = curDiffState.leftRPM * timeInterval / 60.0 * circumference;
	 float rightDist = curDiffState.rightRPM * timeInterval / 60.0 * circumference;
	 float dist = (leftDist + rightDist) / 2;
	  
	 curBotPosition.x += dist * cos(degreeToRad(curBotPosition.phi));
	 curBotPosition.y += dist * sin(degreeToRad(curBotPosition.phi));
}


void Go2Goal::getWheelOutputs(struct differentialState curState, struct differentialState desiredState) 
{
	
	if(desiredState.leftRPM<0)
	{
		desiredState.leftRPM=(-1)*desiredState.leftRPM;
		LeftWheelPWM.Dir = 1;
	}
	else
	{
		LeftWheelPWM.Dir = 0;
	}
	
	
	if(desiredState.rightRPM<0)
	{
		desiredState.rightRPM=(-1)*desiredState.rightRPM;
		RightWheelPWM.Dir = 1;
	}
	else
	{
		RightWheelPWM.Dir = 0;
	}
	
	LeftWheelPWM.PWM += LeftWheel.ImplementPID(desiredState.leftRPM - curState.leftRPM);
	RightWheelPWM.PWM += RightWheel.ImplementPID(desiredState.rightRPM - curState.rightRPM);
	
	
	if(LeftWheelPWM.PWM > MaxPWM) 
	{
		LeftWheelPWM.PWM = MaxPWM;
	} 
	else if(LeftWheelPWM.PWM < 0)
	{
		LeftWheelPWM.PWM = 0;
	}
	
	
	if(RightWheelPWM.PWM > MaxPWM) 
	{
		RightWheelPWM.PWM = MaxPWM;
	} 
	else if(RightWheelPWM.PWM < 0)
	{
		RightWheelPWM.PWM = 0;
	}
}
