
/*
 * File: XboxController.h
 * Description: h file for the XboxController
 * Last Changed: 1-19-13 borderline 1-20-13 :D
 * Author: Matthew W
 */

#ifndef XBOXCONTROLLER_H
#define XBOXCONTROLLER_H

#include <Joystick.h>

class XboxController
{

private:
	Joystick* xboxController;
	
public:
	XboxController(Joystick* parXboxController);
	~XboxController();
	
	/*
	 * Use these methods for easier methods on reading input
	 * from an xbox controller
	 */
	
	bool IsAPressed();
	bool IsBPressed();
	bool IsXPressed();
	bool IsYPressed();
	bool IsLeftBumperPressed();
	bool IsRightBumperPressed();
	bool IsBackButtonPressed();
	bool IsStartButtonPressed();
	bool IsLeftStickPressed();
	bool IsRightStickPressed();
	
	/*
	 * LeftXAxis: Finds the x axis of the left joysick on the xbox controller
	 * LeftYAxis: Finds the y axis of the left joystick
	 * TriggerAxis: Finds the axis of both the triggers combined, with the left trigger making the value more negative and the right trigger making the value more positive
	 * RightXAxis: Finds the x axis of the right joystick
	 * RightYAxis: Finds the y axis of the right joystick
	 */
	
	double GetLeftXAxis();
	double GetLeftYAxis();
	double GetTriggerAxis();
	double GetRightXAxis();
	double GetRightYAxis();

};

#endif
