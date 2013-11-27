/*
 * File: CandyCane.cpp
 * Description: cpp for the CandyCane class
 * Last Changed: 2-1-13
 * Author: Mitchell S
 */

//***********************************************************
//              INCLUDES
//***********************************************************
#include "WPILib.h"
#include "CandyCane.h"
//***********************************************************
//              DEFINES
//***********************************************************
#define MANUAL 1

#define MOTOR_FORWARDS   1.0
#define MOTOR_BACKWARDS -1.0
#define MOTOR_STOPPED    0

// Limit switch states
#define ACTIVATED 0
#define DE_ACTIVATED 1

#define CC_RETRACT_LIMIT 18
#define CC_EXTEND_LIMIT  CC_FULL_SAFETY

// Servo Hook positions
#define HOOK_POSITION 130  //90 degrees
#define HOOK_NEUTRAL  30 // 0 degrees

#define GRAB_RELEASE_OUT_DISTANCE  22500//The distance the CC will need to go out before coming back in

//***********************************************************
//             CLASS METHOD DECLARATIONS
//***********************************************************
CandyCane_Class::CandyCane_Class(
		UINT32 moduleNumber,
		UINT32 extensionArmMotorPort,    // port that the extension arm motor(lead screw) is connected to
		UINT32 twistServoPort,           // pwm port that the twist servo is connected to
		//TODO REMOVE			//UINT32 hookLimitPort,          // port that the hook limit switch is connected to
		UINT32 armMinLimitPort,          // port that the fully extended switch is connected to
		UINT32 armMaxLimitPort,          // port that the fully retracted limit switch is connected to
		UINT32 clawGrabSwitchPort,		 // port that the claw's limit switch is on
		//UINT32 lightSensorPort,			 // port that gets the ferous sensor that detcecets the horizontal
		UINT32 railPositionPort1,		 // port that gets the rail position when it's in between the switches
		UINT32 railPositionPort2
)
{
	extensionArmMotor          	= new Talon(moduleNumber, extensionArmMotorPort);//sidecar#,port#
	twistServo                 	= new Servo(moduleNumber, twistServoPort);
	ArmUpLimit	 				= new DigitalInput(moduleNumber, armMaxLimitPort);
	ArmDownLimit 				= new DigitalInput(moduleNumber, armMinLimitPort);
	homeSwitch					= new DigitalInput(moduleNumber, clawGrabSwitchPort);//TODO
	//lightSensor					= new AnalogChannel(moduleNumber, lightSensorPort);
	railPosition				= new Encoder(moduleNumber,railPositionPort1,moduleNumber,railPositionPort2);
	railPosition->Reset();
	railPosition->Start();

	hookTurnTimer = new Timer();
	hookTurnTimer->Reset();

	bringingInTower = false;
	bringingInCC = false;
	pushingOut = false;

	candyCaneState = CC_IDLE;
	
	twistServo->SetAngle(HOOK_NEUTRAL);
}

CandyCane_Class::~CandyCane_Class()
{
	delete extensionArmMotor;
	delete twistServo;
	//TODO delete hookLimitSwitch;
	delete ArmUpLimit;
	delete ArmDownLimit;
	delete homeSwitch;
	//delete lightSensor;
	delete railPosition;
	//delete railPositionPort2;
}

void CandyCane_Class::HomeReset()
{
	if(homeSwitch->Get() == PRESSED)
	{
		extensionArmMotor->Set(0);
		railPosition->Reset();
	}
}

void CandyCane_Class::StartGrab(double grabbingDistance)
{
	//Don't do anything unless we're not already doing anything
	if(candyCaneState == CC_IDLE)
	{
		candyCaneState = CC_GRAB_PUSH_OUT;
		grabDistance = grabbingDistance;
	}
}
//**************************************************************************************************
//
//  Method  Grab()
//  Parameters: None
//  Returns: candyCaneState
//  Description:  This method is the state machine process for grabbing the vertical leg of the pyramid
//                It extends a hook to vertical bar, rotates the hook into position and then secures the
//                robot to the vertical bar.
//
//*************************************************************************************************
int CandyCane_Class::Grab()
{
	switch(candyCaneState)
	{
	case CC_GRAB_PUSH_OUT:
		//put the servo in position
		ServoNeutral();
		extensionArmMotor->Set(MOTOR_FORWARDS);
		candyCaneState = CC_GRAB_ROTATE_SERVO;
		break;
	case CC_GRAB_ROTATE_SERVO:
		//Put the servo in position and stop the motor
		if(railPosition->GetDistance() >= GRAB_RELEASE_OUT_DISTANCE)
		{
			extensionArmMotor->Set(MOTOR_STOPPED);
			ServoPosition();
			candyCaneState = CC_GRAB_BRING_IN;
			hookTurnTimer->Reset();
			hookTurnTimer->Start();
		}
		break;
	case CC_GRAB_BRING_IN:
		//bring in the cc until it gets to the grab distance
		if(hookTurnTimer->Get() >= 0.25)
		{
			extensionArmMotor->Set(MOTOR_BACKWARDS);
			candyCaneState = CC_GRAB_END;
			hookTurnTimer->Stop();
		}
		break;
	case CC_GRAB_END:
		//Turn everything off
		if((railPosition->GetDistance() <= grabDistance) || (homeSwitch->Get() == PRESSED))
		{
			if(homeSwitch->Get() == PRESSED)
			{
				ResetEncoder();
			}
			extensionArmMotor->Set(MOTOR_STOPPED);
			candyCaneState = CC_IDLE;
		}
		break;
	default:
		break;
	}

	return candyCaneState;
}

void CandyCane_Class::StartRelease()
{
	if(candyCaneState == CC_IDLE)
	{
		candyCaneState = CC_RELEASE_PUSH_OUT;
	}
}

//************************************************************************************
//
//	Method:      Release()
//	Parameters:  None
//	Returns:     candyCaneState
//	Description: This function is the state machine for releasing a candy cane
//				 from a tower. It extends the candy cane, turns the servo, and brings in 
//               the candy cane.
//**************************************************************************************
int CandyCane_Class::Release()
{
	switch(candyCaneState)
	{
	//Pushes the CC to its outer limit
	case CC_RELEASE_PUSH_OUT:
		extensionArmMotor->Set(MOTOR_FORWARDS);
		candyCaneState = CC_RELEASE_ROTATE_SERVO;
		break;
		//Put the servo into the neutral position
	case CC_RELEASE_ROTATE_SERVO:
		if(railPosition->GetDistance() >= GRAB_RELEASE_OUT_DISTANCE)
		{
			ServoNeutral();
			hookTurnTimer->Reset();
			hookTurnTimer->Start();
			candyCaneState = CC_RELEASE_BRING_IN;
		}
		break;
		//Brings the CC into home
	case CC_RELEASE_BRING_IN:
		if(hookTurnTimer->Get() >= 0.25)
		{
			extensionArmMotor->Set(MOTOR_BACKWARDS);
			hookTurnTimer->Stop();
			candyCaneState = CC_RELEASE_END;
		}
		break;
		//Turns everything off
	case CC_RELEASE_END:
		if((railPosition->GetDistance() <= CC_RETRACT_LIMIT) || (homeSwitch->Get() == PRESSED))
		{
			if(homeSwitch->Get() == PRESSED)
			{
				ResetEncoder();
			}
			extensionArmMotor->Set(MOTOR_STOPPED);
			candyCaneState = CC_IDLE;
		}
		break;
	default:
		break;
	}
	return candyCaneState;
}

void CandyCane_Class::StartSafety(double distance)
{
	if(candyCaneState == CC_IDLE)
	{
		candyCaneState = CC_SAFETY_PUSH_OUT;
		safetyDistance = distance;
	}
}

//***********************************************************************************
//
//  Method:      Safety()
//  Parameters:  None
//  Returns:     candyCaneState
//  Description: This function is a state machine for pushing the cc out to it's outer
//  limit, and then turns the servo into position
//***********************************************************************************
int CandyCane_Class::Safety()
{
	switch(candyCaneState)
	{
	//Brings the motor out to its limit
	case CC_SAFETY_PUSH_OUT:
		extensionArmMotor->Set(MOTOR_FORWARDS);
		candyCaneState = CC_SAFETY_ROTATE_SERVO;
		break;
	case CC_SAFETY_ROTATE_SERVO:
		if(railPosition->GetDistance() >= safetyDistance)
		{
			extensionArmMotor->Set(MOTOR_STOPPED);
			ServoPosition();
			candyCaneState = CC_IDLE;
		}
		break;
	default:
		break;
	}
	return candyCaneState;
}

void CandyCane_Class::StartLeanIn(double leaningDistance)
{
	if(candyCaneState == CC_IDLE)
	{
		candyCaneState = CC_LEAN_IN_LEAN;
		leanInDistance = leaningDistance;
	}
}

//************************************************************************************
//
//	Method:      LeanIn()
//	Parameters:  None
//	Returns:     candyCaneState
//	Description: This function brings in the cc in a more after an initial grab or 
//  			 Possibly safety
//**************************************************************************************
int CandyCane_Class::LeanIn()
{
	switch(candyCaneState)
	{
	case CC_LEAN_IN_LEAN:
		extensionArmMotor->Set(MOTOR_BACKWARDS);
		ServoPosition();
		candyCaneState = CC_LEAN_IN_END;
		break;
	case CC_LEAN_IN_END:
		if((railPosition->GetDistance() <= leanInDistance) 
				|| (homeSwitch->Get() == PRESSED))
		{
			if(homeSwitch->Get() == PRESSED)
			{
				ResetEncoder();
			}
			extensionArmMotor->Set(MOTOR_STOPPED);
			candyCaneState = CC_IDLE;
		}
		break;
	default:
		break;
	}
	return candyCaneState;
}

void CandyCane_Class::BringInTower(void)
{
	//if(candyCaneDevice->)
#if MANUAL
	//twistServo->SetAngle(HOOK_POSITION);
	/*if(hookTurnTimer->Get() >= 1)
	{*/
	if(railPosition->GetDistance() >= CC_RETRACT_LIMIT )
	{
		extensionArmMotor->Set(MOTOR_BACKWARDS);
	}
	else if(homeSwitch->Get() == PRESSED)
	{
		ResetEncoder();
		extensionArmMotor->Set(0);
	}
	else
	{
		extensionArmMotor->Set(0);
	}
	//}
#else
	if(bringingInTower)
	{
		twistServo->SetAngle(HOOK_POSITION);
		if(hookTurnTimer->Get() >= 1)
		{
			extensionArmMotor->Set(MOTOR_BACKWARDS);
		}
		if(railPosition->GetDistance() <= CC_RETRACT_LIMIT)
		{
			extensionArmMotor->Set(MOTOR_STOPPED);
			bringingInTower = false;
		}
	}
#endif
	/*else
	{
		extensionArmMotor->Set(0);
	}*/
}

void CandyCane_Class::BringInCC(void)
{
#if MANUAL
	if((railPosition->GetDistance() >= CC_RETRACT_LIMIT))
	{
		extensionArmMotor->Set(MOTOR_BACKWARDS);
	}
	else if(homeSwitch->Get() == PRESSED)
	{
		ResetEncoder();
		extensionArmMotor->Set(0);
	}
	else
	{
		extensionArmMotor->Set(0);
	}
	//twistServo->SetAngle(HOOK_NEUTRAL);
#else
	if(bringingInCC)
	{
		extensionArmMotor->Set(MOTOR_BACKWARDS);
		twistServo->SetAngle(HOOK_NEUTRAL);
		if(railPosition->GetDistance() <= CC_RETRACT_LIMIT)
		{
			extensionArmMotor->Set(MOTOR_STOPPED);
			bringingInCC = false;
		}
	}
#endif
}

bool CandyCane_Class::PushOut(void)
{
	//if(ArmMaxLimit->Get() == 1)
#if MANUAL
	if(railPosition->GetDistance() <= CC_EXTEND_LIMIT)
	{
		extensionArmMotor->Set(MOTOR_FORWARDS);
		return true;
	}
	else
	{
		extensionArmMotor->Set(0);
		return false;
	}
	//twistServo->SetAngle(HOOK_NEUTRAL);
#else
	if(pushingOut)
	{
		extensionArmMotor->Set(MOTOR_FORWARDS);
		twistServo->SetAngle(HOOK_NEUTRAL);
		if(railPosition->GetDistance() >= CC_EXTEND_LIMIT)
		{
			extensionArmMotor->Set(MOTOR_STOPPED);
			pushingOut = false;
		}
	}
#endif
	/*else
	{
		extensionArmMotor->Set(0);
	}*/
}

void CandyCane_Class::ServoPosition()
{
	twistServo->SetAngle(HOOK_POSITION);
}

void CandyCane_Class::ServoNeutral()
{
	twistServo->SetAngle(HOOK_NEUTRAL);
}

void CandyCane_Class::StartBringInTower()
{
	if(!bringingInTower && !pushingOut && !bringingInCC)
	{
		bringingInTower = true;
		hookTurnTimer->Reset();
		hookTurnTimer->Start();
	}
}

void CandyCane_Class::StartBringInCC()
{
	if(!bringingInTower && !pushingOut && !bringingInCC)
	{
		bringingInCC = true;
	}
}

void CandyCane_Class::StartPushOut()
{
	if(!bringingInTower && !pushingOut && !bringingInCC)
	{
		pushingOut = true;
	}
}

bool CandyCane_Class::BringInOverride()
{
	/*if(bringInOverride)
	{
		if(homeSwitch->Get() == RELEASED)
		{
			extensionArmMotor->Set(-0.25);
			candyCaneState = CC_IDLE;
		}
		else
		{
			extensionArmMotor->Set(0);
			bringInOverride = false;
		}
	}*/
	//If the homeswitch is released, bring in the CC
	if(homeSwitch->Get() == RELEASED)
	{
		extensionArmMotor->Set(MOTOR_BACKWARDS);
		candyCaneState = CC_IDLE;
		return true;
	}
	//Otherwise stop the CC and reset the encoder values
	else
	{
		extensionArmMotor->Set(0);
		ResetEncoder();
		return false;
	}
}

void CandyCane_Class::PushOutOverride()
{
	/*if(pushOutOverride)
	{
		if(homeSwitch->Get() == RELEASED)
		{
			extensionArmMotor->Set(0.25);
			candyCaneState = CC_IDLE;
		}
		else
		{
			extensionArmMotor->Set(0);
			pushOutOverride = false;
		}
	}*/
	/*if(homeSwitch->Get() == RELEASED)
	{
		extensionArmMotor->Set(0.25);
		candyCaneState = CC_IDLE;
	}
	else
	{
		extensionArmMotor->Set(0);
	}*/
	extensionArmMotor->Set(MOTOR_FORWARDS);
	candyCaneState = CC_IDLE;
}

void CandyCane_Class::Stop(void)
{
	extensionArmMotor->Set(MOTOR_STOPPED);
	candyCaneState = CC_IDLE;
	bringingInTower = false;
	bringingInCC = false;
	pushingOut = false;
}
/*void CandyCane_Class::ClawUp(void)
{
	twistServo->Set(0.2);//TODO Depreciate ClawUp(), and ClawDown()
}
void CandyCane_Class::ClawDown(void)
{
	twistServo->Set(0.5);
}*/

void CandyCane_Class::ResetEncoder()
{
	railPosition->Reset();
}

/*void CandyCane_Class::Grab(void)
{
 *****************************************************************************************
int CandyCane_Class::getCandyCaneState(void)
{
	return (candyCaneState);
}

int CandyCane_Class::getBarSensor()
{
	return (barSensor->GetAverageValue());
}*/

/*int CandyCane_Class::getCaneOutSensor(void)
{
	return (ArmMaxLimit->Get());
}	
int CandyCane_Class::getCaneInSensor(void)
{
	return (ArmMinLimit->Get());
}	
int CandyCane_Class::getLightSensor(void)
{
	return (lightSensor->GetVoltage());
}*/	
int CandyCane_Class::getClawTop(void)
{
	return ((int)ArmUpLimit->Get());
}	
int CandyCane_Class::getClawBottom(void)
{
	return ((int)ArmDownLimit->Get());
}
/*int CandyCane_Class::getClawPos(void)
{
	return (twistServo->Get());
}*/
float CandyCane_Class::getServoPos(void)
{
	return (twistServo->Get());
}
double CandyCane_Class::getRailPos(void)
{
	return (railPosition->GetDistance());
}
/*int CandyCane_Class::getHomeSwitch(void)
{
	return (homeSwitch->Get());
}*/




