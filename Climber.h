/*
 * File: Climber.h
 * Description: Declares the climber class
 * Last Changed: 1-19-13
 * Author: Bijan T,Mitchell S.,Marked1-29
 */

#ifndef CLIMBER_H
#define CLIMBER_H

#include "Tower.h"
#include "CandyCane.h"
#include <DriverStationEnhancedIO.h>

//states of climbing
#define FIRST_GRAB_STATE         1
#define REACHING_STATE           2
#define CLIMBING_STATE           3
#define CANDYCANE_RELEASE_STATE  4
#define CANDYCANE_GRAB_STATE     5
#define IDLE_STATE               6
#define KILL_STATE               7

#define END_GAME 10

class Climber_Class
{
private:
	Tower_Class* leftTower;				//brings info from left tower class
	Tower_Class* rightTower;			//brings info from right tower class
	CandyCane_Class* candyCane;	//brings info from candy cane class
	
	int climbState;
	bool isClimbing;
	bool startCC;
	bool inState; //True if we are in the middle of the state, false if the state is done
	bool bringOutCC;
	
public:
	Climber_Class();
	~Climber_Class();
	//void Move();			//Call this to make both the towers and candy cane start the process of climbing the tower (this needs to be called repeditively)
	void KillClimb();		//Kill the robot if anything goes wrong
	bool Reach();			//in manual bring towers down
	bool Climb();			//in manual bring hooks down
	bool ReachDistance(double encoderValue);
	bool ClimbDistance(double encoderValue);
	void TowersStop();
	void ResetTowers();
	void AutoClimb(int state);
	void Halt();			//in manual bring stop towers form moving
	bool IsClimbing();		//Checks to see if the robot is climbing
	Tower_Class* GetLeftTower();
	Tower_Class* GetRightTower();
	CandyCane_Class* GetCandyCaneClass();
	
	void StartLevelOneClimb();
	void StartNextLevelClimb();
	
	bool InState();
	void NewState(); //Sets inState to true when we begin a new state
};

#endif
