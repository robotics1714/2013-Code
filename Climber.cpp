
/**********************************************
 * File: Climber.cpp
 * Created: 2-1-13
 * Author: Sam D,Mitchell S marked1-29
 * Desc: The cpp file for the Climber class
 **********************************************/

#include "Climber.h"

//TODO: Set the correct ports
/*
 * Left tower port defines
 */
#define MODULE_NUMBER_ONE 1

#define LEFT_MODULE_NUMBER 1
#define LEFT_LIFT_MOTOR_PORT_PARAM 3
#define LEFT_LIFT_ENCODER_PORT1_PARAM 5
#define LEFT_LIFT_ENCODER_PORT2_PARAM 6
#define LEFT_RAIL_REACH_LIMIT_PORT_PARAM 9
#define LEFT_RAIL_CLIMB_LIMIT_PORT_PARAM 10

#define RIGHT_MODULE_NUMBER 1
#define RIGHT_LIFT_MOTOR_PORT_PARAM 4
#define RIGHT_LIFT_ENCODER_PORT1_PARAM 7
#define RIGHT_LIFT_ENCODER_PORT2_PARAM 8
#define RIGHT_RAIL_REACH_LIMIT_PORT_PARAM 11
#define RIGHT_RAIL_CLIMB_LIMIT_PORT_PARAM 12

/*
 * Candy cane port defines
 */
#define MODULE_NUMBER_TWO 2
#define EXTENSION_ARM_MOTOR_PORT_PARAM 1
#define TWIST_SERVO_PORT_PARAM 2
#define ARM_MIN_LIMIT_PORT_PARAM 1
#define ARM_MAX_LIMIT_PORT_PARAM 2
#define HOME_SWITCH_PARAM 5
//#define CLAW_GRAB_SENSOR_PORT_PARAM 11
//#define BAR_SENSOR_PORT_PARAM 7
#define RAIL_POSITION_PORT1_PARAM 3
#define RAIL_POSITION_PORT2_PARAM 4

/*********************
 * Auto Climb Defines:
 *********************/
//TODO Change encoder values to correct values
#define AUTO_CLIMB_START_STATE 0
#define AUTO_CLIMB_STATE_1 1
#define AUTO_CLIMB_STATE_2 2
#define AUTO_CLIMB_STATE_3 3
#define AUTO_CLIMB_STATE_4 4
#define AUTO_CLIMB_STATE_5 5
#define AUTO_CLIMB_STATE_6 6
#define AUTO_CLIMB_STATE_7 7
#define AUTO_CLIMB_STATE_8 8
#define AUTO_CLIMB_STATE_9 9
#define AUTO_CLIMB_STATE_10 10
#define AUTO_CLIMB_STATE_11 11
#define AUTO_CLIMB_STATE_12 12
#define AUTO_CLIMB_STATE_13 13
#define AUTO_CLIMB_STATE_14 14
#define AUTO_CLIMB_STATE_15 15
#define AUTO_CLIMB_STATE_16 16
#define AUTO_CLIMB_STATE_17 17
#define AUTO_CLIMB_STATE_18 18
#define AUTO_CLIMB_STATE_19 19
#define AUTO_CLIMB_STATE_20 20
#define AUTO_CLIMB_STATE_21 21
#define AUTO_CLIMB_STATE_22 22
#define AUTO_CLIMB_STATE_23 23
#define AUTO_CLIMB_STATE_24 24
#define AUTO_CLIMB_STATE_25 25
#define AUTO_CLIMB_STATE_26 26
#define AUTO_CLIMB_STATE_27 27
//#define AUTO_CLIMB_STATE_25 25
//#define AUTO_CLIMB_ENCODER_STATE_25 141
//#define AUTO_CLIMB_STATE_26 26

#define LEVEL_1A_REACH_POS  490
#define LEVEL_1_GRAB_POS    8750//5895
#define LEVEL_1A_CLIMB_POS  301
#define LEVEL_2A_REACH_POS  385
#define LEVEL_2A_SAFETY_POS 41000//32800//20779 //go back 1500
#define LEVEL_2A_SAFETY_LEAN_POS 38500
#define LEVEL_2B_REACH_POS  570
#define LEVEL_2A_LEAN_POS   24500//14347//12348
#define LEVEL_2B_LEAN_POS   15582
#define LEVEL_2A_CLIMB_POS  527
#define LEVEL_2B_SAFETY     32152//10636
#define LEVEL_3A_REACH_POS  385
#define LEVEL_3A_SAFETY_POS 41000//11992
#define LEVEL_3A_SAFETY_LEAN_POS 38500
#define LEVEL_3B_REACH_POS  580//539
#define LEVEL_3A_LEAN_POS   21000//14347//5984
#define LEVEL_3B_LEAN_POS   17161//2814
#define LEVEL_3A_CLIMB_POS  535//525
#define LEVEL_3B_SAFETY_POS 33730//10600//12000
#define LEVEL_3B_CLIMB_POS  110

#define DISTANCE_FOR_EXTENDEING_CC 365//325

/*
 * Candy Cane Bar Sensor Defines
 * TODO Change BAR_SENSOR_SENSED to correct value
 */
#define BAR_SENSOR_SENSED 10

Climber_Class::Climber_Class()
{
	leftTower = new Tower_Class(
			MODULE_NUMBER_ONE,
			LEFT_LIFT_MOTOR_PORT_PARAM,
			LEFT_LIFT_ENCODER_PORT1_PARAM,
			LEFT_LIFT_ENCODER_PORT2_PARAM,
			//LEFT_REACH_CLAW_PORT_PARAM,
			//LEFT_TRAIL_CLAW_PORT_PARAM,
			LEFT_RAIL_REACH_LIMIT_PORT_PARAM,
			LEFT_RAIL_CLIMB_LIMIT_PORT_PARAM
	);
	rightTower = new Tower_Class(
			MODULE_NUMBER_ONE,
			RIGHT_LIFT_MOTOR_PORT_PARAM,
			RIGHT_LIFT_ENCODER_PORT1_PARAM, 
			RIGHT_LIFT_ENCODER_PORT2_PARAM, 
			//RIGHT_REACH_CLAW_PORT_PARAM,
			//RIGHT_TRAIL_CLAW_PORT_PARAM,
			RIGHT_RAIL_REACH_LIMIT_PORT_PARAM,
			RIGHT_RAIL_CLIMB_LIMIT_PORT_PARAM); 
	candyCane = new CandyCane_Class(
			MODULE_NUMBER_TWO,
			EXTENSION_ARM_MOTOR_PORT_PARAM,
			TWIST_SERVO_PORT_PARAM,
			ARM_MIN_LIMIT_PORT_PARAM,
			ARM_MAX_LIMIT_PORT_PARAM,
			HOME_SWITCH_PARAM,
			RAIL_POSITION_PORT1_PARAM,
			RAIL_POSITION_PORT2_PARAM
	); 

	climbState = IDLE_STATE;
	startCC = false;
	inState = false;
}

Climber_Class::~Climber_Class()
{
	delete leftTower;
	delete rightTower;
	delete candyCane;
}

/*
 * Move();
 * Steps for making the robot climb:
 * 1) Grab pole w/ candy cane(CC)
 * 2) Reach w/ towers until distance reached
 * 3) If claws tripped release CC
 * 4) if claws tripped then climb
 * 5) Match speeds
 * 6) At grapplelimit, grap pole w/ cc
 * 
 * This method will command the towers and candy cane to move the robot up the pyramid.
 * 
 */

bool Climber_Class::Reach()
//{if(rightTower->GetRailEncoder() >= 144) &&//and we hit the top limit switches
//(leftTower->GetRailEncoder() >= 144))//or enco limits of the towers
{
	//Reach returns 0(false) when it's done and 2(true) when it's going
	bool rightReach = (bool)rightTower->Reach();
	bool leftReach = (bool)leftTower->Reach();
	
	//returns true if the towers are still reaching and false if they're stopped
	return !(!rightReach && !leftReach);
}
bool Climber_Class::Climb()
{
	//Climb returns 0(false) when it's done and 1(true) when it's going
	bool rightClimb = rightTower->Climb();
	bool leftClimb = leftTower->Climb();
	
	//returns true if the towers are still climbing and false when they're stopped
	return !(!rightClimb && !leftClimb);
}

bool Climber_Class::ReachDistance(double encoderValue)
{
	//ReachDistance returns 0(false) when it's done and 2(true) when it's going
	bool leftReach = leftTower->ReachDistance(encoderValue);
	bool rightReach = rightTower->ReachDistance(encoderValue);
	
	//returns true if the towers are still reaching and false when they're stopped
	return !(!rightReach && !leftReach);
}

bool Climber_Class::ClimbDistance(double encoderValue)
{
	//ClimbDistance returns 0(false) when it's done and 2(true) when it's going
	bool leftClimb = leftTower->ClimbDistance(encoderValue);
	bool rightClimb = rightTower->ClimbDistance(encoderValue);
	
	//returns true if the towers are still climbing and false when they're stopped
	return !(!rightClimb && !leftClimb);
}

void Climber_Class::TowersStop()
{
	leftTower->StopMoving();
	rightTower->StopMoving();
}

void Climber_Class::ResetTowers()
{
	leftTower->ResetReach();
	leftTower->ResetClimb();
	rightTower->ResetReach();
	rightTower->ResetClimb();
}

void Climber_Class::AutoClimb(int state)
{	
	bool climb;
	bool reach;
	bool pushOut;
	bool safety;
	bool release;
	
	switch(state)
	{
	case AUTO_CLIMB_STATE_1:
		//brings the towers stationary hooks all the way up
		//The towers need to be reset, but only once
		/*if(firstReach)
		{
			ResetTowers();
			firstReach = false;
		}*/
		if(startCC)
		{
			ResetTowers();
			startCC = false;
		}
		inState = ReachDistance(LEVEL_1A_REACH_POS);
		break;
	case AUTO_CLIMB_STATE_2:
		//pushes the candy cane out
		inState = (bool)candyCane->PushOut();
		//The startCC boolean only allows the CC functions to be called once per state
		startCC = true;
		break;
	case AUTO_CLIMB_STATE_3:
		//Reset the towers so we know we can move them in the next state
		ResetTowers();
		TowersStop();
		//Grab the pole
		if(startCC)
		{
			candyCane->StartGrab(LEVEL_1_GRAB_POS);
			//This line makes the CC only grab once
			startCC = false;
		}
		//If we're done grabbing, inState will be false and we can go to the next state otherwise it will be true
		inState = (bool)candyCane->Grab();
		break;
	case AUTO_CLIMB_STATE_4:
		//climb until arms grab the horizontal bar
		
		//If we're still climbing inState is set to true, otherwise it is set to false
		inState = (bool)ClimbDistance(LEVEL_1A_CLIMB_POS);
		//Tell the CC it can move in the next state
		startCC = true;
		break;
	case AUTO_CLIMB_STATE_5:
		//Release the CC
		if(startCC)
		{
			//Reset the towers so we know we can move them in the next state
			ResetTowers();
			TowersStop();
			candyCane->StartRelease();
			startCC = false;
			release = true;
			climb = true;
		}
		//If we're done releasing, inState will be set to false so we can go to the next state, otherwise it will be true
		release = (bool)candyCane->Release();
		climb = (bool)Climb();
		
		if(!climb && !release)
		{
			inState = false;
		}
		break;
	/*case AUTO_CLIMB_STATE_6:	
		//Climb until the hooks transition
		//Check to see if we're still climbing
		inState = (bool)Climb();
		break;*/
	case AUTO_CLIMB_STATE_6:
		//Reach until the CC is clear
		if(startCC)
		{
			reach = true;
			pushOut = true;
			startCC = false;
			bringOutCC = true;
		}
		//Check to see if we're still reaching
		reach = (bool)ReachDistance(LEVEL_2A_REACH_POS);

		//Once the towers reached a certain distance, push out the CC
		if((leftTower->GetDistance() >= DISTANCE_FOR_EXTENDEING_CC) && bringOutCC)
		{
			candyCane->StartSafety(LEVEL_2A_SAFETY_POS);
			bringOutCC = false;
		}
		pushOut = candyCane->Safety();
		//If we're not reaching or pushing out, we're not in the state anymore
		if(!reach && !pushOut)
		{
			inState = false;
		}
		/*//Check to see if we're still reaching
		inState = (bool)ReachDistance(LEVEL_2A_REACH_POS);
		//Tell the CC it can move in the next state
		startCC = true;*/
		break;
	//Level 2
	/*case AUTO_CLIMB_STATE_7:
		//CC safety position
		if(startCC)
		{
			candyCane->StartSafety(LEVEL_2A_SAFETY_POS);
			//This line makes the CC go into the safety position only once
			startCC = false;
		}
		inState = (bool)candyCane->Safety();
		break;*/
	case AUTO_CLIMB_STATE_7:
		//Reset the towers so we know we can move them in the next state
		ResetTowers();
		TowersStop();
		//Bring in the CC so the servo turns
		if(startCC)
		{
			candyCane->StartLeanIn(LEVEL_2A_SAFETY_LEAN_POS);
			startCC = false;
		}
		inState = (bool)candyCane->LeanIn();
		break;
	case AUTO_CLIMB_STATE_8:
		//Reach until the CC grabs the vertical bar
		//Check to see if we're still reaching
		inState = (bool)ReachDistance(LEVEL_2B_REACH_POS);
		//Tell the CC it can move in the next state
		startCC = true;
		break;
	case AUTO_CLIMB_STATE_9:
		//Reset the towers so we know we can move them in the next state
		ResetTowers();
		TowersStop();
		//Lean a little bit
		if(startCC)
		{
			candyCane->StartLeanIn(LEVEL_2A_LEAN_POS);
			startCC = false;
		}
		//Check to see if we're still leaning in
		inState = (bool)candyCane->LeanIn();
		break;
	case AUTO_CLIMB_STATE_10:
		//Reach all the way
		//Check to see if we're still reaching
		inState = (bool)Reach();
		startCC = true;
		break;
	case AUTO_CLIMB_STATE_11:
		//Reset the towers so we know we can move them in the next state
		ResetTowers();
		TowersStop();
		//Lean in until the arms touch the horizontal bars
		if(startCC)
		{
			candyCane->StartLeanIn(LEVEL_2B_LEAN_POS);
			startCC = false;
		}
		//Check to see if we're still leaning in
		inState = (bool)candyCane->LeanIn();
		break;
	case AUTO_CLIMB_STATE_12:
		//climb until the arms grab the horizontal bars
		//Check to see if we're still climbing
		inState = (bool)ClimbDistance(LEVEL_2A_CLIMB_POS);
		startCC = true;
		break;
	/*case AUTO_CLIMB_STATE_14:
		//Reset the towers so we know we can move them in the next state
		ResetTowers();
		TowersStop();
		//CC safety
		if(startCC)
		{
			candyCane->StartSafety(LEVEL_2B_SAFETY);
			startCC = false;
		}
		//Check to see if we're still safetying
		inState = (bool)candyCane->Safety();
		break;*/
	case AUTO_CLIMB_STATE_13:
		//Release the CC
		if(startCC)
		{
			//Reset the towers so we know we can move them in the next state
			ResetTowers();
			TowersStop();
			candyCane->StartRelease();
			startCC = false;
			climb = true;
			release = true;
		}
		//Climb until the hooks transition
		//Check to see if we're still climbing
		climb = (bool)Climb();
		release = (bool)candyCane->Release();
		
		if(!climb && !release)
		{
			inState = false;
		}
		break;
	/*case AUTO_CLIMB_STATE_16:
		//Reset the towers so we know we can move them in the next state
		ResetTowers();
		TowersStop();
		//Release the CC
		if(startCC)
		{
			candyCane->StartRelease();
			startCC = false;
		}
		//Check to see if we're still releasing
		inState = (bool)candyCane->Release();
		break;*/
	//Level 3
	case AUTO_CLIMB_STATE_14:
		//Reach until the CC is clear
		if(startCC)
		{
			reach = true;
			pushOut = true;
			bringOutCC = true;
			startCC = false;
		}
		//Check to see if we're still reaching
		reach = (bool)ReachDistance(LEVEL_3A_REACH_POS);

		//Once the towers reached a certain distance, push out the CC
		if((leftTower->GetDistance() >= DISTANCE_FOR_EXTENDEING_CC) && bringOutCC)
		{
			candyCane->StartSafety(LEVEL_3A_SAFETY_POS);
			bringOutCC = false;
		}
		pushOut = candyCane->Safety();
		
		//If we're not reaching or pushing out, we're not in the state anymore
		if(!reach && !pushOut)
		{
			inState = false;
		}
		/*//Check to see if we're still reaching
		inState = (bool)ReachDistance(LEVEL_3A_REACH_POS);
		startCC = true;*/
		break;
	/*case AUTO_CLIMB_STATE_16:
		//CC safety
		if(startCC)
		{
			candyCane->StartSafety(LEVEL_3A_SAFETY_POS);
			startCC = false;
		}
		//Check to see if we're still safetying
		inState = (bool)candyCane->Safety();
		break;*/
	case AUTO_CLIMB_STATE_15:
		//Reset the towers so we know we can move them in the next state
		ResetTowers();
		TowersStop();
		//Bring in the CC so the servo turns
		if(startCC)
		{
			candyCane->StartLeanIn(LEVEL_3A_SAFETY_LEAN_POS);
			startCC = false;
		}
		inState = (bool)candyCane->LeanIn();
		break;
	case AUTO_CLIMB_STATE_16:
		//Reach until the CC grabs the verticle bar
		//Check to see if we're still reaching
		inState = (bool)ReachDistance(LEVEL_3B_REACH_POS);
		startCC = true;
		break;
	case AUTO_CLIMB_STATE_17:
		//Reset the towers so we know we can move them in the next state
		ResetTowers();
		TowersStop();
		//Lean in
		if(startCC)
		{
			candyCane->StartLeanIn(LEVEL_3A_LEAN_POS);
			startCC = false;
		}
		//Check to see if we're still leaning in
		inState  = (bool)candyCane->LeanIn();
		break;
	case AUTO_CLIMB_STATE_18:
		//Reach all the way
		//Check to see if we're still reaching
		inState = (bool)Reach();
		startCC = true;
		break;
	case AUTO_CLIMB_STATE_19:
		//Reset the towers so we know we can move them in the next state
		ResetTowers();
		TowersStop();
		//Lean in until the arms hit the horizontal bar
		if(startCC)
		{
			candyCane->StartLeanIn(LEVEL_3B_LEAN_POS);
			startCC = false;
		}
		//Check to see if we're still leaning in
		inState = (bool)candyCane->LeanIn();
		break;
	case AUTO_CLIMB_STATE_20:
		//climb until the arms grab the horizontal bars
		//Check to see if we're still climbing
		inState = (bool)ClimbDistance(LEVEL_3A_CLIMB_POS);
		startCC = true;
		break;
	case AUTO_CLIMB_STATE_21:
		//CC safety
		if(startCC)
		{
			//Reset the towers so we know we can move them in the next state
			ResetTowers();
			TowersStop();
			candyCane->StartSafety(LEVEL_3B_SAFETY_POS);
			startCC = false;
			safety = true;
			climb = true;
		}
		//Check to see if we're still safetying
		safety = (bool)candyCane->Safety();
		climb = (bool)ClimbDistance(LEVEL_3B_CLIMB_POS);
		
		if(!safety && !climb)
		{
			inState = false;
		}
		break;
	/*case AUTO_CLIMB_STATE_27:
		//Climb until we want to stop and win
		////Check to see if we're still climbing
		inState = (bool)ClimbDistance(LEVEL_3B_CLIMB_POS);
		break;*/
	}
	/*grabbing = (bool)candyCane->Grab();
	safetying = (bool)candyCane->Safety();
	releasing = (bool)candyCane->Release();
	leaningIn = (bool)candyCane->LeanIn();*/
}

Tower_Class* Climber_Class::GetLeftTower()
{
	return leftTower;
}

Tower_Class* Climber_Class::GetRightTower()
{
	return rightTower;
}

CandyCane_Class* Climber_Class::GetCandyCaneClass()
{
	return candyCane;
}

bool Climber_Class::InState()
{
	return inState;
}

//Sets inState to true when we begin a new state
void Climber_Class::NewState()
{
	inState = true;
	startCC = true;
}
