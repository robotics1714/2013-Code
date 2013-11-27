/*
 *File: Dumper.h
 *Description: Declares the dumping class 
 * Last Changed: 1-23-13
 * Author: Bijan T marked1-29
*/

#ifndef DUMPER_H
#define DUMPER_H

//State defines
#define STATE_IDLE  0
#define STATE_BEGIN 1
#define STATE_RAISE 2
#define STATE_LOWER 3

#include <DigitalInput.h>
#include <Servo.h>
#include <Victor.h>
#include <Timer.h>
//#include "SmartDashboard/SmartDashboard.h"
#include "GlobalDefines.h"
#include "DriverStationLCD.h"

class Dumper_Class
{
private:
	Victor* motor;						//motor that lifts bucket
	Servo* gate1;						//the first servo that tilts bucket
	Servo* gate2;                       //The second servo that tilts the bucket
	Timer *gateOpenTime;                //Measures the amount of time between when the gate opens and closes
	Timer* timeBetweenPulse;            //Tests to make sure that more than one pulse is counted on one hit on a limit switch
	DigitalInput* turnCountLimit;		//Counts how many times the lead screw is turned
	DigitalInput* topLimitSwitch;       //Limit switch on the bottom of the dumper
	DigitalInput* bottomLimitSwitch;    //Limit switch on the top limit of the dumper
	bool pressed;                       //Sees if the limit switch is pressed
	int turnCount;                      //Counts the amount of times the lead screw turns
	int turnsToLimit;                   //The amount times the lead screw turns until you reach the top or bottom of the dumper
	int dumpingState;
	bool reseting;
	bool extending;
	bool raisingToHeight;
public:
	Dumper_Class(
			UINT32 moduleNumber,
			UINT32 motorPort,
			UINT32 gatePort1,
			UINT32 gatePort2,
			UINT32 counterPort,
			UINT32 topLimitPort,
			UINT32 bottomLimitPort,
			int maxTurns);
	~Dumper_Class();
	
	void Dump(void);//Dump the contents of the dumper
	void StartDump(void);//Starts dumping
	void StartReset(void);
	bool Reset(void);//Reset the dumper, returns true if it's done reseting
	void StartExtend(void);
	void Extend(void);
	void Stop();
	void StartRaiseToHeight();
	void RaiseToHeight();
	int GetState(void);//Check the state of the dumper
	
	void OpenGate();
	void CloseGate();
};

#endif
