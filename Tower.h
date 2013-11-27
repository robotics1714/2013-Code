
/********************************************************
 * File: Tower.h
 * Created: 1-19-13
 * Last Updated: 2-15-13
 * Author: Matthew W
 * Desc: The header file for the tower class marked 1-29
 ********************************************************/
#ifndef TOWER_H
#define TOWER_H

/***********
 * INCLUDES:
 ***********/
#include <Talon.h>
#include <Encoder.h>
#include <DigitalInput.h>
#include <DriverStationLCD.h>
#include "GlobalDefines.h"

/**********
 * DEFINES:
 **********/

/*****************************************************
 * Defines for use inside of the tower reach sequence,
 * namely the Reach() and Climb() function.
 *****************************************************/
#define TOWER_REACH_STATE 2
#define TOWER_CLIMB_STATE 1
#define TOWER_IDLE_STATE 0

/***************************************************************
 * Defines for use inside of the tower reach sequences that will
 * utilize the encoders on the towers to stop before the safety.
 * These defines will be used in the ReachDistance() and 
 * ClimbDistance() functions.
 ***************************************************************/
#define TOWER_DISTANCE_REACH_STATE 2
#define TOWER_DISTANCE_CLIMB_STATE 1
#define TOWER_DISTANCE_IDLE_STATE 0
#define TOWER_REACH_ENCODER_DISTANCE 100

/***********************************************************************
 * Defines used for the movement of the tower motor. This will determine
 * how fast the motors on the tower will move based on a number between
 * -1 and 1.
 ***********************************************************************/
#define MOTOR_REACH_VELOCITY 1.0
#define MOTOR_CLIMB_VELOCITY -1.0
#define MOTOR_DISTANCE_PER_PULSE 1

class Tower_Class
{
private:
	Talon* liftMotor;
	Encoder* liftEncoder;
	DigitalInput* railReachLimit;
	DigitalInput* railClimbLimit;
	
	bool reaching;
	bool reachDistancing;
	bool climbing;
	bool climbDistancing;
	int reachState;
	int reachDistanceState;
	int climbState;
	int climbDistanceState;
	
public:
	Tower_Class(UINT32 moduleNumber,
			UINT32 liftMotorPort, 
			UINT32 liftEncoderPort1, 
			UINT32 liftEncoderPort2, 
			UINT32 railReachLimitPort,
			UINT32 railClimbLimitPort);
	~Tower_Class();
	
	/*******************************************************************
	 * These functions are the functions that will be called when using
	 * the tower. A full explanaition for each funcion can be found
	 * in the Tower.cpp file.
	 *******************************************************************/
	int Reach();
	int ReachDistance(double encoderStopValue);
	void ResetReach();
	int Climb();
	int ClimbDistance(double encoderStopValue);
	void ResetClimb();
	void StopMoving();
	void Kill();
	bool Reset();
	
	/*************************************************************************
	 * Testing functions used for testing feedback on encoders / limit switches.
	 * These will be commented out when they are not needed.
	 *************************************************************************/
	void ResetEncoder();
	double GetVelocity();
	double GetDistance();
//	int GetUpLimit();
//	int GetDownLimit();
};

#endif
