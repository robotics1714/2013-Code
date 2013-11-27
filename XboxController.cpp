
/*
 * File: XboxController.cpp
 * Description: cpp file for the XboxController, making it easier to instantiate an xbox controller
 * Last Changed: 1-19-13 borderline 1-20-13 :D
 * Author: Matthew W
 */

#include "XboxController.h"
 
#define LEFT_X 1
#define LEFT_Y 2
#define TRIGGERS 3
#define RIGHT_X 4
#define RIGHT_Y 5

#define BUTTON_A 1
#define BUTTON_B 2
#define BUTTON_X 3
#define BUTTON_Y 4
#define LEFT_BUMPER 5
#define RIGHT_BUMPER 6
#define BACK_BUTTON 7
#define START_BUTTON 8
#define LEFT_STICK_CLICK 9
#define RIGHT_STICK_CLICK 10

XboxController::XboxController(Joystick* parXboxController)
{
	xboxController = parXboxController;
}

XboxController::~XboxController()
{
	delete xboxController;
}

bool XboxController::IsAPressed()
{
	return xboxController->GetRawButton(BUTTON_A);
}

bool XboxController::IsBPressed()
{
	return xboxController->GetRawButton(BUTTON_B);
}

bool XboxController::IsXPressed()
{
	return xboxController->GetRawButton(BUTTON_X);
}

bool XboxController::IsYPressed()
{
	return xboxController->GetRawButton(BUTTON_Y);
}

bool XboxController::IsLeftBumperPressed()
{
	return xboxController->GetRawButton(LEFT_BUMPER);
}

bool XboxController::IsRightBumperPressed()
{
	return xboxController->GetRawButton(RIGHT_BUMPER);
}

bool XboxController::IsLeftStickPressed()
{
	return xboxController->GetRawButton(LEFT_STICK_CLICK);
}

bool XboxController::IsRightStickPressed()
{
	return xboxController->GetRawButton(RIGHT_STICK_CLICK);
}

double XboxController::GetLeftXAxis()
{
	return xboxController->GetRawAxis(LEFT_X);
}

double XboxController::GetLeftYAxis()
{
	return xboxController->GetRawAxis(LEFT_Y);
}

double XboxController::GetTriggerAxis()
{
	return xboxController->GetRawAxis(TRIGGERS);
}

double XboxController::GetRightXAxis()
{
	return xboxController->GetRawAxis(RIGHT_X);
}

double XboxController::GetRightYAxis()
{
	return xboxController->GetRawAxis(RIGHT_Y);
}
