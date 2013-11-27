/*
 *File: Dumper.cpp
 * Description: cpp file for the dumper class. If button is pressed or dumping state
 * 	if not equal to STATE_NEUTRAL, call Dumper::Dump()
 * Last Changed: 1-23-13
 * Author: Bijan T marked1-29
 */
#include "Dumper.h"

//Timer defines
#define GATE_OPEN_TIME 3 //The amount of time the dumper gate will be open until it closes
#define TIME_BETWEEN_LIMIT_PULSES 0.03 //Half the amount of time between 1/6th of a lead screw revolution

//Motor speeds
#define MOTOR_FORWARDS   1
#define MOTOR_BACKWARDS -1

//Servo movement defines, the values are opposite because the servo are mirrored
#define SERVO_ONE_NEUTRAL 5
#define SERVO_ONE_TILTED  130//120
#define SERVO_TWO_NEUTRAL 165
#define SERVO_TWO_TILTED  40//50

Dumper_Class::Dumper_Class(
		UINT32 moduleNumber,
		UINT32 motorPort,
		UINT32 gatePort1,
		UINT32 gatePort2,
		UINT32 counterPort,
		UINT32 topLimitPort,
		UINT32 bottomLimitPort,
		int maxTurns)
{
	//Initialize the objects in the class
	motor = new Victor(moduleNumber, motorPort);
	gate1 = new Servo(moduleNumber, gatePort1);
	gate1->SetAngle(SERVO_ONE_NEUTRAL);
	gate2 = new Servo(moduleNumber, gatePort2);
	gate2->SetAngle(SERVO_TWO_NEUTRAL);
	gateOpenTime = new Timer();
	timeBetweenPulse = new Timer();
	turnCountLimit = new DigitalInput(moduleNumber, counterPort);
	topLimitSwitch = new DigitalInput(moduleNumber, topLimitPort);
	bottomLimitSwitch = new DigitalInput(moduleNumber, bottomLimitPort);
	pressed = false;
	turnCount = 0;
	turnsToLimit = maxTurns;
	dumpingState = STATE_IDLE;
	reseting = false;
	extending = false;
	raisingToHeight = false;
}

Dumper_Class::~Dumper_Class()
{
	delete motor;
	delete gate1;
	delete gate2;
	delete gateOpenTime;
	delete timeBetweenPulse;
	delete turnCountLimit;
	delete topLimitSwitch;
	delete bottomLimitSwitch;
	//delete turnCounter;
}

//Dump the contents of the dumper
void Dumper_Class::Dump(void)
{	
	//Move the dumper forwards until the dumper can not go
	//any farther, then move backwards
	/*if((!topLimitSwitch->Get()) && (motor->Get() != MOTOR_BACKWARDS))
	{
		motor->Set(MOTOR_FORWARDS);
	}
	else
	{
		motor->Set(0);
		Wait(1.0);
		motor->Set(MOTOR_BACKWARDS);
	}

	//If the dumper is going backwards and on the bottom,
	//stop the motor
	if((motor->Get() == MOTOR_BACKWARDS) && (bottomLimitSwitch->Get()))
	{
		motor->Set(0);
		//We're done dumping, so set dumping to false
		dumping = false;
	}*/
	//SmartDashboard::PutNumber("Debug", 1);
	switch(dumpingState)//if the dump function is called
	{
	case STATE_BEGIN:
		turnCount = 0;//Resets the turn count
		motor->Set(MOTOR_FORWARDS);//push the bucket up
		dumpingState = STATE_RAISE;
		//SmartDashboard::PutNumber("Debug", 2);
		break;
	case STATE_RAISE:
		//SmartDashboard::PutNumber("Debug", 3);
		//The second part is so we can get into the if statement when it was time for the gate to close
		if(((turnCountLimit->Get() == PRESSED)&& !pressed) 
				|| (gateOpenTime->Get() >= GATE_OPEN_TIME)
				|| (topLimitSwitch->Get() == PRESSED))
		{						//until above a certain height and the gate open time has
			turnCount++;		//expired update and proced to next event
			pressed = true;
			timeBetweenPulse->Start();
			if((turnCount >= turnsToLimit) || (topLimitSwitch->Get() == PRESSED))//if above the tilt height
			{
				motor->Set(0);//turn the motor off
				//start tilting
				gate1->SetAngle(SERVO_ONE_TILTED);
				gate2->SetAngle(SERVO_TWO_TILTED);
				gateOpenTime->Start();//start the timer
				if(gateOpenTime->Get() >= GATE_OPEN_TIME)//after 4 seconds of tilting
				{
					//Reset the timers
					gateOpenTime->Stop();
					gateOpenTime->Reset();
					//set bucket flat
					gate1->SetAngle(SERVO_ONE_NEUTRAL);
					gate2->SetAngle(SERVO_TWO_NEUTRAL);
					motor->Set(MOTOR_BACKWARDS);//start lowering and continue
					dumpingState = STATE_LOWER;
					turnCount = 0;//Reset the turn count
				}
			}
		}
		break;
	case STATE_LOWER:
		//if bucket is all the way down
		if((bottomLimitSwitch->Get() == PRESSED))
		{
			motor->Set(0);//turn all motors off
			dumpingState = STATE_IDLE;//Set the dumper to idle because we're done
			turnCount = 0;//reset encoder to 0
		}
		break;
	case STATE_IDLE:
		break;
	default:
		break;
	}
	//SmartDashboard::PutNumber("Debug", 4);
	//Turns pressed to false if the limit switch isn't pressed and it is passed the time
	//Between limit pulses to make sure to get rid of the bounce
	if((turnCountLimit->Get() == RELEASED) &&
			timeBetweenPulse->Get() >= TIME_BETWEEN_LIMIT_PULSES)
	{
		//SmartDashboard::PutNumber("Debug", 5);
		pressed = false;
		timeBetweenPulse->Stop();
		timeBetweenPulse->Reset();
	}

	/*SmartDashboard::PutNumber("Debug", 6);
	SmartDashboard::PutNumber("Turn Count", turnCount);
	SmartDashboard::PutBoolean("Pressed", pressed);*/
	/*SmartDashboard::PutNumber("Count Switch", turnCountLimit->Get());
	SmartDashboard::PutNumber("Top Limit", topLimitSwitch->Get());
	SmartDashboard::PutNumber("Bottom Limit", bottomLimitSwitch->Get());
	SmartDashboard::PutNumber("Dump Gate 1",gate1->GetAngle());
	SmartDashboard::PutNumber("Dump Gate 2", gate2->GetAngle());*/

	DriverStationLCD *dsLCD = DriverStationLCD::GetInstance();
	dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "Dump %i", turnCount);
	/*dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Bottom Limit %i", topLimitSwitch->Get());
	dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "Bottom Limit %i", turnCountLimit->Get());
	dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "Bottom Limit %i", turnCount);*/
	dsLCD->UpdateLCD();
}

void Dumper_Class::StartDump(void)
{
	//Make sure we're not dmping so we don't start over in the middle of dumping
	if(GetState() == STATE_IDLE)
		dumpingState = STATE_BEGIN;
}

void Dumper_Class::OpenGate()
{
	gate1->SetAngle(SERVO_ONE_TILTED);
	gate2->SetAngle(SERVO_TWO_TILTED);
}

void Dumper_Class::CloseGate()
{
	gate1->SetAngle(SERVO_ONE_NEUTRAL);
	gate2->SetAngle(SERVO_TWO_NEUTRAL);
}

void Dumper_Class::StartReset()
{
	reseting = true;
	extending = false;
	raisingToHeight = false;
}

bool Dumper_Class::Reset()
{
	//Reset the dumper, until it hits the bottom limit switch
	/*if(bottomLimitSwitch->Get() == RELEASED)
	{
		motor->Set(MOTOR_BACKWARDS);
		gate->Set(SERVO_NEUTRAL);
		gateOpenTime->Stop();
		gateOpenTime->Reset();
		timeBetweenPulse->Stop();
		timeBetweenPulse->Reset();
		dumpingState = STATE_IDLE;
		return false;
	}
	else
	{
		motor->Set(0);
		return true;
	}*/
	if(reseting)
	{
		if(bottomLimitSwitch->Get() == RELEASED)
		{
			motor->Set(MOTOR_BACKWARDS);
			gate1->SetAngle(SERVO_ONE_NEUTRAL);
			gate2->SetAngle(SERVO_TWO_NEUTRAL);
			dumpingState = STATE_IDLE;
		}
		else
		{
			motor->Set(0);
			reseting = false;
		}
	}

	return true;
}

void Dumper_Class::StartExtend()
{
	reseting = false;
	extending = true;
	raisingToHeight = false;
}

void Dumper_Class::Extend()
{
	if(extending)
	{
		if(topLimitSwitch->Get() == RELEASED)
		{
			motor->Set(MOTOR_FORWARDS);
			dumpingState = STATE_IDLE;
		}
		else
		{
			motor->Set(0);
			extending = false;
		}
	}
}

void Dumper_Class::StartRaiseToHeight()
{
	raisingToHeight = true;
	reseting = false;
	extending = false;
	turnCount = 0;
}

void Dumper_Class::RaiseToHeight()
{
	//Raise the dumper until it either hits the top limit or goes the height we want it to
	if(raisingToHeight)
	{
		motor->Set(MOTOR_FORWARDS);
		if((turnCountLimit->Get() == PRESSED && !pressed) 
				|| (topLimitSwitch->Get() == PRESSED))
		{
			pressed = true;
			turnCount++;
			if(turnCount >= turnsToLimit || topLimitSwitch->Get() == PRESSED)
			{
				motor->Set(0);
				raisingToHeight = false;
				turnCount = 0;
			}
		}
	}
	
	//Reset the pressed boolean when neccessary
	if((turnCountLimit->Get() == RELEASED) 
			&& (timeBetweenPulse->Get() >= TIME_BETWEEN_LIMIT_PULSES))
	{
		pressed = false;
		timeBetweenPulse->Stop();
		timeBetweenPulse->Reset();
	}
}

void Dumper_Class::Stop()
{
	motor->Set(0);
//	gate1->SetAngle(SERVO_ONE_NEUTRAL);
//	gate2->SetAngle(SERVO_TWO_NEUTRAL);
	dumpingState = STATE_IDLE;
	reseting = false;
	extending = false;
	raisingToHeight = false;
}

//Check to see if the dumper is dumping
int Dumper_Class::GetState(void)
{
	return dumpingState;
}
