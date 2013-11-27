/*************************************************************
 * File: CandyCane.h
 * Description: Declares the CandyCane class
 * Last Changed: 2-1-13
 * Author: Mitchell S
 *************************************************************/
#ifndef CANDYCANE_H
#define CANDYCANE_H
//************************************************************
//         Includes
//************************************************************
#include <Victor.h>
#include <Servo.h>
#include <DigitalInput.h>
#include <AnalogChannel.h>
#include <Encoder.h>
#include <Timer.h>
#include "GlobalDefines.h"
//************************************************************
//         Defines
//************************************************************
//   CandyCane States
#define CC_IDLE 0

#define CC_GRAB_PUSH_OUT        1
#define CC_GRAB_ROTATE_SERVO    2
#define CC_GRAB_BRING_IN        3
#define CC_GRAB_END             4

#define CC_RELEASE_PUSH_OUT     5
#define CC_RELEASE_ROTATE_SERVO 6
#define CC_RELEASE_BRING_IN     7
#define CC_RELEASE_END          8

#define CC_SAFETY_PUSH_OUT      9
#define CC_SAFETY_ROTATE_SERVO  10

#define CC_LEAN_IN_LEAN         11
#define CC_LEAN_IN_END          12

#define CC_FULL_SAFETY          41400//24800

/*#define CC_RETRACTED 0
#define CC_EXTENDING 1
#define CC_HOOKING   2
#define CC_HOLDING   3 
#define CC_UNHOOKING 4
#define CC_RETRACTING 5*/



//************************************************************
//         Class Definitions
//************************************************************

class CandyCane_Class
{

private:
	Talon        *extensionArmMotor;         // Motor that drives cc to extend hook
	Servo         *twistServo;                // Servo that positions hook to grab bar
	DigitalInput  *ArmUpLimit; 	 		  // signal that cc is fully up
	DigitalInput  *ArmDownLimit; 	  		  // signals that cc is fully down
	DigitalInput  *homeSwitch;			  // signals cc is all in
	//AnalogChannel *lightSensor;		       // tells us when horizontal bar is coming
	Encoder 	  *railPosition;			  // tells us where the rail precisly is
	int            candyCaneState;            // state of the candycane machine
	double         grabDistance;              //The distance the cc will tilt in while pulling in
	double         safetyDistance;            //The distance the cc will go back in the safety
	double         leanInDistance;            //The distance the cc will lean in
	
	Timer         *hookTurnTimer;
	
	bool           bringingInTower;
	bool           bringingInCC;
	bool           pushingOut;
public:
	CandyCane_Class(
		UINT32 moduleNumber,	
		UINT32 extensionArmMotorPort,
		UINT32 twistServoPort,
		UINT32 armMinLimitPort,
		UINT32 armMaxLimitPort,
		UINT32 homeSwitchPort,
		//UINT32 lightSensorPort,
		UINT32 railPositionPort1,
		UINT32 railPositionPort2
		
		);
	
	~CandyCane_Class();
	
	//Semi-auto CC commands
	void StartGrab(double grabbingDistance);
	int  Grab();
	void StartRelease();
	int  Release();
	void StartSafety(double distance);
	int  Safety();
	void StartLeanIn(double leaningDistance);
	int  LeanIn();
	
	//Manual CC commands
	void StartBringInTower();
	void StartBringInCC();
	void StartPushOut();
	void BringInTower(void);//turns motor to push cc out
	void BringInCC(void);
	bool PushOut(void);//turns motor to bring cc in
	
	void ServoPosition();
	void ServoNeutral();
	
	void Stop(void);//stops in and out movements of the cc
	//void ClawUp(void);//sets servo up
	//void ClawDown(void);//sets servo down
	void ResetEncoder(void);
	//void ClawUp(void);
	//void ClawDown(void);
	int getCandyCaneState(void);   //returns the state of the CandyCane machine
	int getLightSensor(void);
	//int getCaneOutSensor(void);
	//int getCaneInSensor(void);
	//int getClawGrabSensor(void);
	int getClawPos(void);//gets location of servo placement by the servo
	double getRailPos(void);//gets encoder
	int getClawTop(void);//checks claw rotate up limit switch
	int getClawBottom(void);//checks claw rotate down limit switch
	int getHomeSwitch(void);//gets limit switch when cc is all in
	float getServoPos(void);
	
	void StartBringInOverride();
	void StartPushOutOverride();
	bool BringInOverride();
	void PushOutOverride();
	
	void HomeReset();		//when home switch is hit, reset cc encoder
	
	int  GetState(){return candyCaneState;}
};
//************************************************************
//         Globals
//************************************************************

#endif
