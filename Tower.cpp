
/*****************************************
 * File: Tower.cpp
 * Created: 1-25-13
 * Author: Matthew W marked1-29
 * Desc: The cpp file for the tower class
 *****************************************/

/***********
 * INCLUDES:
 ***********/
#include "Tower.h"

/*******************
 * PUBLIC FUNCTIONS:
 *******************/

/************************************************************************************************
 * Function Name: Tower_Class(); (Constructor)
 * Parameters:
 * - moduleNumber: The UINT32 that will determine the module of all ports used below.
 * - liftMotorPort: The UINT32 used to determine the port for the lift motor.
 * - liftEncoderPort1&2: The UINT32 used to determine the ports for the lift encoder.
 * - railReachLimitPort: The UINT32 used to determine the port for the safety reach limit switch.
 * - railClimbLimitPOrt: The UINT32 used to determine the port for the safety climb limit switch.
 * Description: The constructor of the Tower_Class that will assign all the ports given to the
 * correct instances of the objects used in the Tower_Class.
 ************************************************************************************************/
Tower_Class::Tower_Class(
		UINT32 moduleNumber,
		UINT32 liftMotorPort, 
		UINT32 liftEncoderPort1, 
		UINT32 liftEncoderPort2, 
		UINT32 railReachLimitPort,
		UINT32 railClimbLimitPort)
{
	//Initialize the objects used for this class
	liftMotor = new Talon(moduleNumber, liftMotorPort);
	liftEncoder = new Encoder(moduleNumber, liftEncoderPort1, moduleNumber, liftEncoderPort2);
	railReachLimit = new DigitalInput(moduleNumber, railReachLimitPort);
	railClimbLimit = new DigitalInput(moduleNumber, railClimbLimitPort);
	
	//Initialize the encoders
	liftEncoder->Start();
	liftEncoder->Reset();
	liftEncoder->SetDistancePerPulse(MOTOR_DISTANCE_PER_PULSE);
	
	//Initialize the variables used in this class
	reaching = false;
	reachDistancing = false;
	climbing = false;
	reachState = TOWER_IDLE_STATE;
	climbState = TOWER_IDLE_STATE;
}

Tower_Class::~Tower_Class()
{
	delete liftMotor;
	delete liftEncoder;
	delete railReachLimit;
	delete railClimbLimit;
	delete &reaching;
	delete &reachDistancing;
	delete &climbing;
	delete &reachState;
	delete &climbState;
}

/********************************************************************************
 * Function Name: Reach();
 * Parameters: None
 * Returns: Integer value based on what state the reach sequence is currently on.
 * - 2: if the reach sequence is still reaching.
 * - 0: if the reach sequence is done.
 * Description: This function will move the non-flippy hook claw up until
 * the safety limit switch for reaching is hit.
 * 
 * Steps for reaching: (Used for planning)S
 * 1) Start the reach motor to go up
 * 2) Keep reaching until railReachLimit is tripped
 * 3) Reverse the lift motor
 * 4) Keep climbing until the reachClawLimit is tripped
 ********************************************************************************/
int Tower_Class::Reach()
{
	//Start the state block
	switch(reachState)
	{
	case TOWER_REACH_STATE:
		if((railReachLimit->Get() == PRESSED))
		{
			reachState = TOWER_IDLE_STATE;
		}
		else
			liftMotor->Set(MOTOR_REACH_VELOCITY);
		reaching = true;
		break;
	default:
		liftMotor->Set(0);
		reaching = false;
		break;
	}
	
	return (reachState);
}

/********************************************************************************
 * Function Name: ReachDistance();
 * Parameters: None
 * Returns: Integer value based on what state the reach sequence is currently on.
 * - 2: if the reach sequence is still going.
 * - 0: if the reach sequence is done.
 * Description: This function will move the non-flippy hook up until the preset
 * encoder value for how high the hook will need to travel is surpassed.
 ********************************************************************************/
int Tower_Class::ReachDistance(double encoderStopValue)
{
	switch(reachDistanceState)
	{
	case TOWER_DISTANCE_REACH_STATE:
		if((liftEncoder->GetDistance() > encoderStopValue) || 
				railReachLimit->Get() == PRESSED)
			reachDistanceState = TOWER_DISTANCE_IDLE_STATE;
		else
			liftMotor->Set(MOTOR_REACH_VELOCITY);
		reachDistancing = true;
		break;
	default:
		liftMotor->Set(0);
		reachDistancing = false;
		break;
	}
	return (reachDistanceState);
}

/***********************************************************************************
 * Function Name: ResetReach();
 * Parameters: None
 * Returns: Void
 * Description: This will reset the reach state so that the claws will be moveable
 * again inside the functions Reach() and ReachDistance() when they are set to idle.
 ***********************************************************************************/
void Tower_Class::ResetReach()
{
	reachState = TOWER_REACH_STATE;
	reachDistanceState = TOWER_DISTANCE_REACH_STATE;
}

/*************************************************************************************
 * Function Name: Climb();
 * Parameters: None
 * Returns: Integer value based on what part of the climb sequence the function is on.
 * - 1: if the climb sequence is still going.
 * - 0: if the climb sequence is done.
 * Description: This function will move the non-flippy hook down until the safety
 * climb limit switch is hit.
 * 
 * Tower::Climb()
 * Steps for climbing:
 * 1) Set motor to climb
 * 2) Keep climbing until the railClimbLimit is tripped
 * 3) Reverse the motor
 * 4) Keep reaching until both claws have contact with the pole 
 *************************************************************************************/
int Tower_Class::Climb()
{
	switch(climbState){
	case TOWER_CLIMB_STATE:
		if((railClimbLimit->Get() == PRESSED))
		{
			climbState = TOWER_IDLE_STATE;
			ResetEncoder();
		}
		else
			liftMotor->Set(MOTOR_CLIMB_VELOCITY);
		climbing = true;
		break;
	default:
		liftMotor->Set(0);
		climbing = false;
		break;
	}
	return (climbState);
}

/******************************************************************************
 * Function Name: ClimbDistance();
 * Parameters: None
 * Returns: Integer value based on what part of the climb sequence it is on.
 * - 1: if the climb sequence is still going.
 * - 0: if the climb sequence is done.
 * Description: This function will move the non-flippy hook down until the 
 * lift encoder reaches a distance below or equal to 0, which will be where the
 * hook originally started.
 ******************************************************************************/
int Tower_Class::ClimbDistance(double encoderStopValue)
{
	switch(climbDistanceState)
	{
	case TOWER_DISTANCE_CLIMB_STATE:
		if((liftEncoder->GetDistance() < encoderStopValue) || 
				railClimbLimit->Get() == PRESSED)
			climbDistanceState = TOWER_DISTANCE_IDLE_STATE;
		else
			liftMotor->Set(MOTOR_CLIMB_VELOCITY);
		climbDistancing = true;
		break;
	default:
		liftMotor->Set(0);
		climbDistancing = false;
		break;
	}
	return (climbDistanceState);
}

/*
 * Function Name: ResetClimb();
 * Parameters: None
 * Returns: Void
 * Description: This will reset the climb state so that the claws will be moveable
 * again inside the functions Climb() and ClimbDistance() when they are set to idle.
 */
void Tower_Class::ResetClimb()
{
	climbState = TOWER_CLIMB_STATE;
	climbDistanceState = TOWER_DISTANCE_CLIMB_STATE;;
}

/********************************************************
 * Function Name: StopMoving();
 * Parameters: None
 * Returns: Void
 * Description: This will make the lift motor stop moving.
 ********************************************************/
void Tower_Class::StopMoving()
{
	liftMotor->Set(0);
}

/**************************************************************************
 * Function Name: Kill();
 * Parameters: None
 * Returns: Void
 * Description: This will reset all climbing sequences to idle and set the
 * then call StopMoving().
 **************************************************************************/
void Tower_Class::Kill()
{
	reachState = TOWER_IDLE_STATE;
	climbState = TOWER_IDLE_STATE;
	StopMoving();
}

/******************************************************************************
 * Function Name: Reset();
 * Parameters: None
 * Returns: Boolean value based on if reseting is done.
 * - true: if the function is done reseting the tower.
 * - false: if the function is not done reseting the tower.
 * Descritpion: This function will move the tower back down to where it started
 * when the tower instance was created.
 ******************************************************************************/
bool Tower_Class::Reset()
{
	if(railClimbLimit->Get() == RELEASED)
	{
		ResetReach();
		ResetClimb();
		liftMotor->Set(MOTOR_CLIMB_VELOCITY);
		return false;
	}
	else
	{
		StopMoving();
		liftEncoder->Reset();
		return true;
	}
}

void Tower_Class::ResetEncoder()
{
	liftEncoder->Reset();
}
//
//double Tower_Class::GetVelocity()
//{
//	return liftEncoder->GetRate();
//}
//
double Tower_Class::GetDistance()
{
	return liftEncoder->GetDistance();
}
//
//int Tower_Class::GetUpLimit()
//{
//	return railReachLimit->Get();
//}
//
//int Tower_Class::GetDownLimit()
//{
//	return railClimbLimit->Get();
//}
