#include "WPILib.h"
#include "Tower.h"
#include "CandyCane.h"
#include "Dumper.h"
#include "Climber.h"
#include "XboxController.h"
#include "Math.h"

//competition #s here
/*#define LEFT_TOWER_MODULE_NUMBER 1				//1
#define LEFT_TOWER_LIFT_MOTOR_PORT 3			//3
#define LEFT_TOWER_ENCODER_PORT_1 5				//5
#define LEFT_TOWER_ENCODER_PORT_2 6				//6
#define LEFT_TOWER_RAIL_REACH_LIMIT_PORT 9		//9
#define LEFT_TOWER_RAIL_CLIMB_LIMIT_PORT 10		//10

#define RIGHT_TOWER_MODULE_NUMBER 1				//1
#define RIGHT_TOWER_LIFT_MOTOR_PORT 4			//4
#define RIGHT_TOWER_ENCODER_PORT_1 7			//7
#define RIGHT_TOWER_ENCODER_PORT_2 8			//8
#define RIGHT_TOWER_RAIL_REACH_LIMIT_PORT 11	//11
#define RIGHT_TOWER_RAIL_CLIMB_LIMIT_PORT 12	//12*/

#define DUMPER_MODULE 2							//2
#define DUMPER_MOTOR_PORT 3						//3
#define DUMPER_LGATE_PORT 9						//9
#define DUMPER_RGATE_PORT 10					//10
#define DUMPER_COUNTER_PORT 6					//6
#define DUMPER_TOP_LIMIT_PORT 7					//7
#define DUMPER_BOTTOM_LIMIT_PORT 8				//8
#define DUMPER_MAX_TURNS 37

//Autonomous defines
#define DISTANCE_UNTIL_START_DUMP 1
#define DUMPING_TIMER             2.5

class RobotDemo : public SimpleRobot
{
	RobotDrive *robot;
	Joystick *leftStick;
	Joystick *rightStick;
	XboxController *xboxController;
	DriverStationLCD *lcd;
	Climber_Class *climber;
	Dumper_Class *dumperDevice;

	AnalogChannel *autoWait; 
	AnalogChannel *autoPos;
	Timer *autoTimer;

	Encoder *rightEnco;
	Encoder *leftEnco;

	Gyro* gyro;

	int distNeed;
	float turnNeed;
	bool doneWait;

public:
	RobotDemo(void)
	{
		AxisCamera &camera = AxisCamera::GetInstance("10.17.14.11");
		robot = new RobotDrive(1,2);
		leftStick = new Joystick(1);
		rightStick = new Joystick(2);
		xboxController = new XboxController(new Joystick(3));
		lcd = DriverStationLCD::GetInstance();
		climber = new Climber_Class();

		autoWait = new AnalogChannel(4);
		autoPos = new AnalogChannel(3);
		autoTimer = new Timer();

		//encoder values for the drive wheels
		leftEnco = new Encoder(1,2);
		rightEnco = new Encoder(3,4);
		rightEnco->SetDistancePerPulse(0.05235);//this number should be the circumference of the
		leftEnco->SetDistancePerPulse(0.05235);//wheel divided by 360
		leftEnco->Start();
		rightEnco->Start();

		gyro = new Gyro(1);

		dumperDevice = new Dumper_Class(
				DUMPER_MODULE,
				DUMPER_MOTOR_PORT,
				DUMPER_LGATE_PORT,
				DUMPER_RGATE_PORT,
				DUMPER_COUNTER_PORT,
				DUMPER_TOP_LIMIT_PORT,
				DUMPER_BOTTOM_LIMIT_PORT,
				DUMPER_MAX_TURNS);

		//Reset the reach on the towers so we can move the towers up in the auto climb right away
		climber->GetLeftTower()->ResetReach();
		climber->GetRightTower()->ResetReach();

		GetWatchdog().SetExpiration(1.0);
	}

	//robot turns to desired position with a deadband of 2 degrees in each direction
	bool GyroTurn (float desiredTurnAngle, float turnSpeed)
	{
		bool turning = true;
		float myAngle = gyro->GetAngle();
		//normalizes angle from gyro to [-180,180) with zero as straight ahead
		while(myAngle >= 180)
		{
			GetWatchdog().Feed();
			myAngle = myAngle - 360;
		}
		while(myAngle < -180)
		{
			GetWatchdog().Feed();
			myAngle = myAngle + 360;
		}
		if(myAngle < (desiredTurnAngle - 2))// if robot is too far left, turn right a bit
		{
			robot->Drive(turnSpeed, -turnSpeed); //(right,left)
		}
		if(myAngle > (desiredTurnAngle + 2))// if robot is too far right, turn left a bit
		{
			robot->Drive(-turnSpeed, turnSpeed); //(right,left)
		}
		else
		{
			robot->Drive(0, 0);
			turning = false;
		}

		return turning;
	}

	bool GyroDrive(float desiredDriveAngle, float speed, int desiredDistance)//todo:fix for teleop
	{
		bool driving = true;
		double encoderInchesTraveled = fabs(leftEnco->GetDistance());//absolute value distance
		float myAngle = gyro->GetAngle();
		//normalizes angle from gyro to [-180,180) with zero as straight ahead
		while(myAngle >= 180)
		{
			GetWatchdog().Feed();
			myAngle = myAngle - 360;
		}
		while(myAngle < -180)
		{
			GetWatchdog().Feed();
			myAngle = myAngle + 360;
		}

		float my_speed = 0.0;
		float turn = 0.0;

		if(speed > 0)
			//30.0 is the number you have to change to adjust properly
			turn = -((myAngle + desiredDriveAngle) / 30.0); //proportionally adjust turn. As the robot gets more off 0, the greater the turn will be
		else
			turn = (myAngle + desiredDriveAngle) / 30.0; //proportionally adjust turn. As the robot gets more off 0, the greater the turn will be

		if (encoderInchesTraveled < desiredDistance)
			my_speed = speed;
		else
		{
			my_speed = 0.0;
			driving = false;
		}

		robot->Drive(my_speed, turn);

		return driving;
	}

	void UseAutoWait(float autoDelay)
	{
		while(autoDelay > 0.3)//if all switches off skip the waiting section
		{
			if((autoDelay  > 0) && (autoDelay  < 0.9))//if wait b 1 is pressed wait 1 sec
			{
				if(autoTimer->Get() > 1)
				{
					doneWait = true;
				}
			}
			else if((autoDelay  > 0.9) && (autoDelay  < 1.5))//if wait b 2 is pressed wait 2 sec
			{
				if(autoTimer->Get() > 2)
				{
					doneWait = true;
				}
			}
			else if((autoDelay  > 1.5) && (autoDelay  < 1.9))//if wait b 3 is pressed wait 3 sec
			{
				if(autoTimer->Get() > 3)
				{
					doneWait = true;
				}
			}
			else if((autoDelay  > 1.9) && (autoDelay  < 2.5))//if wait b 4 is pressed wait 4 sec
			{
				if(autoTimer->Get() > 4)
				{
					doneWait = true;
				}
			}
			else if((autoDelay  > 2.5) && (autoDelay  < 3.1))//if wait b 5 is pressed wait 5 sec
			{
				if(autoTimer->Get() > 5)
				{
					doneWait = true;
				}
			}
			else if((autoDelay  > 3.1) && (autoDelay  < 3.8))//if wait b 6 is pressed wait 6 sec
			{
				if(autoTimer->Get() > 6)
				{
					doneWait = true;
				}
			}
			else if((autoDelay  > 3.8) && (autoDelay  < 4.5))//if wait b 7 is pressed wait 7 sec
			{
				if(autoTimer->Get() > 7)
				{
					doneWait = true;
				}
			}
			else if((autoDelay  > 4.5) && (autoDelay  < 5.5))//if wait b 8 is pressed wait 8 sec
			{
				if(autoTimer->Get() > 8)
				{
					doneWait = true;
				}
			}
		}
	}

	void UsePosSense(float autoPos)
	{
		if(autoPos < 1)
		{
			distNeed = 20;
			turnNeed = 0;
		}
		//Second auto position: Start at left of the frontright corner of pyramid
		else if((autoPos > 1) && (autoPos < 1.45))
		{
			distNeed = 10;
			turnNeed = 0;
		}
		//Third auto position: Start at right of the front right corner of pyramid
		else if((autoPos > 1.5) && (autoPos < 2.1))
		{

			distNeed = 10;
			turnNeed = 0;
		}
		//Fourth auto position: start back right corner
		else if((autoPos > 2.1) && (autoPos < 2.45))
		{
			distNeed = 20;
			turnNeed = 0;
		}
		else
		{
			distNeed = 0;
			turnNeed = 0;
		}
	}
	
	//The autonomous code in right now is for demos
	void Autonomous(void)
	{
		GetWatchdog().SetEnabled(false);
		bool lArmStart = true;
		bool rArmStart = true;
		bool lArmUp = false;
		bool rArmUp = false;
		bool CCIn   = true;
		
		//Reset the towers
		climber->GetLeftTower()->ResetReach();
		climber->GetLeftTower()->ResetClimb();
		climber->GetRightTower()->ResetReach();
		climber->GetRightTower()->ResetClimb();
		/*gyro->Reset();
		bool continueAuto = true;

		autoTimer->Reset();
		autoTimer->Start();

		//Calculate the autonomous delay
		UsePosSense(autoPos->GetAverageVoltage());

		bool drivingTo = true;//Driving to the goals
		bool raiseDumper = false;
		bool lowerDumper = false;
		bool openGate = false;
		bool drivingFrom = true;
		bool secondState = false;
		Timer* dumperTimer = new Timer();

		bool retractingCC = true;
		bool doneSafetyCC = false;
		bool resetingTowers = true;
		climber->ResetTowers();

		dumperTimer->Reset();
		dumperTimer->Start();*/
		while(IsAutonomous())
		{
			//Bring the arms all the way down to reset the encoders
			if(lArmStart)
			{
				lArmStart = (bool)climber->GetLeftTower()->Climb();
				//When the climb hits the limit switch start the up and down movement
				if(!lArmStart)
				{
					lArmUp = true;
				}
			}
			if(rArmStart)
			{
				rArmStart = (bool)climber->GetRightTower()->Climb();
				//When the climb hits the limit switch start the up and down movement
				if(!rArmStart)
				{
					rArmUp = true;
				}
			}
			
			//figure out if the left tower should go up or down
			if(lArmUp)//Left Arm Up
			{
				lArmUp = (bool)climber->GetLeftTower()->ReachDistance(300);
			}
			else if(!lArmUp && !lArmStart)//Left arm down
			{
				lArmUp = !((bool)climber->GetLeftTower()->ClimbDistance(50));
				//We need to reset the climber before going through the cycle again
				if(lArmUp)
				{
					climber->GetLeftTower()->ResetReach();
					climber->GetLeftTower()->ResetClimb();
				}
			}
			
			if(rArmUp)//Right Arm Up
			{
				rArmUp = (bool)climber->GetRightTower()->ReachDistance(300);
			}
			else if(!rArmUp && !rArmStart)
			{
				rArmUp = !((bool)climber->GetRightTower()->ClimbDistance(50));
				//We need to reset the climber before going through the cycle again
				if(rArmUp)
				{
					climber->GetRightTower()->ResetReach();
					climber->GetRightTower()->ResetClimb();
				}
			}
			
			//have the dumper keep on dumping
			dumperDevice->StartDump();
			dumperDevice->Dump();
			
			/*if(continueAuto && doneWait)
			{
				if(drivingTo)
				{
					drivingTo = GyroDrive(0, 0.2, 5);
					//Start raising the dumper right away, but only at the beginning
					if(!raisedDumper)
					{
						dumperDevice->StartRaiseToHeight();
						raisedDumper = true;
					}
				}
				//when the robot is close to the goal, open the gate
				if((rightEnco->GetDistance() >= (distNeed - DISTANCE_UNTIL_START_DUMP)) 
						&& !finishedDump && drivingTo)
				{
					dumperDevice->OpenGate();
					dumpTimer->Reset();
					dumpTimer->Start();
				}

				//After a cerain amount of timer, go backwards
				if(dumpTimer->Get() >= DUMPING_TIMER)
				{
					dumpTimer->Stop();
					finishedDump = true;
				}

				if(finishedDump)
				{
					continueAuto = GyroDrive(0, -0.2, 5);
					//Bring the dumper down and close the gate
					dumperDevice->StartReset();
					dumperDevice->CloseGate();
				}

				dumperDevice->RaiseToHeight();
				dumperDevice->Reset();
			}
			else 
			{
				UseAutoWait(autoWait->GetAverageVoltage());
			}*/

			//Bring the CC to home to reset the encoder then bring it back out so it's in the frame perimeter
			/*if(retractingCC)
			{
				retractingCC = climber->GetCandyCaneClass()->BringInOverride();
			}
			else if(!retractingCC && !doneSafetyCC)
			{
				climber->GetCandyCaneClass()->StartSafety(15500);
				doneSafetyCC = true;
			}*/

			/*climber->GetCandyCaneClass()->Safety();
			//Since Safety sets the servo to position, manualy put the servo to neutral
			climber->GetCandyCaneClass()->ServoNeutral();*/

			/*//Bring the towers all the way down to reset the encoder
			climber->GetLeftTower()->Climb();
			climber->GetRightTower()->Climb();*/

			//Bring the towers all the way down and then back up
			/*if(resetingTowers)
			{
				bool left = climber->GetLeftTower()->Climb();
				bool right = climber->GetRightTower()->Climb();
				if(!left && !right)
				{
					resetingTowers = false;

				}
			}
			if(!resetingTowers)
			{
				climber->GetLeftTower()->ReachDistance(100);
				climber->GetRightTower()->ReachDistance(100);
			}*/

			/*if(!secondState)
			{
				//Drive forward
				if(drivingTo)
				{
					drivingTo = GyroDrive(0, 0.5, 120);
				}

				if(!raiseDumper)
				{
					dumperDevice->StartRaiseToHeight();
					raiseDumper = true;
				}

				if((dumperTimer->Get() <= 4))
				{
					dumperDevice->RaiseToHeight();
				}
				else
				{
					dumperDevice->Stop();
					openGate = true;
				}

				if(openGate)
				{
					dumperDevice->OpenGate();
					dumperTimer->Stop();
					dumperTimer->Reset();
					dumperTimer->Start();
					drivingFrom = true;
					secondState = true;
					openGate = false;
				}
			}*/
			
			//Drive backwards
			/*else
			{
				lcd->Printf(DriverStationLCD::kUser_Line6, 1, "HELLO! %s", (secondState)?"True":"False");
				if(drivingFrom && (dumperTimer->Get() >= 2.5))
				{
					drivingFrom = GyroDrive(45, -0.5, 90);
					lowerDumper = true;
				}
				dumperDevice->Reset();
			}*/


			/*if(drivingTo)
			{
				drivingTo = GyroDrive(7, -0.5, 40);
			}*/

			lcd->Printf(DriverStationLCD::kUser_Line1, 1, "CC %f", climber->GetCandyCaneClass()->getRailPos());
			lcd->Printf(DriverStationLCD::kUser_Line2, 1, "L Arm %d", climber->GetLeftTower()->GetDistance());
			lcd->Printf(DriverStationLCD::kUser_Line3, 1, "R Arm %d", climber->GetRightTower()->GetDistance());
			lcd->Printf(DriverStationLCD::kUser_Line4, 1, "LArm %s", (lArmUp)?"true":"false");
			lcd->UpdateLCD();
		}
		//Stop the motors when autonomous is done
		dumperDevice->Stop();
		climber->GetCandyCaneClass()->Stop();
		climber->GetLeftTower()->StopMoving();
		climber->GetRightTower()->StopMoving();
	}

	void OperatorControl(void)
	{
		GetWatchdog().Feed();
		//candyCaneDevice->ServoNeutral();
		int climberState = 0;
		bool lastLeftTriggerPress = false;
		bool lastButton3TriggerPress = false;
		bool autoClimb = true;

		bool input = false;
		bool lastRJoyStop = false;
		bool lastYButtonPress = false;
		bool lastAButtonPress = false;
		bool isStopping = false;
		//SmartDashboard::PutString("hi", "hi");
		while (IsOperatorControl())
		{
			GetWatchdog().Feed();
			//SmartDashboard::PutString("Reach reached", "...");
			//SmartDashboard::PutString("Stop reached", "...");

			//Stop everything and switch from auto climb to manual climb
			if(leftStick->GetRawButton(3) && rightStick->GetRawButton(7) && !lastButton3TriggerPress)
			{
				climber->GetLeftTower()->StopMoving();
				climber->GetRightTower()->StopMoving();
				climber->GetCandyCaneClass()->Stop();
				autoClimb = !autoClimb;
			}
			lastButton3TriggerPress = leftStick->GetRawButton(3);

			if(autoClimb)
			{
				//Go to the next state
				if(leftStick->GetRawButton(2) && rightStick->GetRawButton(7) && !lastLeftTriggerPress && !climber->InState())
				{
					climberState += 1;
					climber->NewState();
				}
				//Gives the second driver the power to bring the arms up
				else if(xboxController->IsLeftBumperPressed() && 
						xboxController->IsRightBumperPressed() && climberState == 0
						&& !climber->InState())
				{
					climberState += 1;
					climber->NewState();
				}
				//Give the second driver the power to push the CC out, but on a different button
				else if(xboxController->IsLeftBumperPressed() &&
						xboxController->IsXPressed() && climberState == 1 &&
						!climber->InState())
				{
					climberState += 1;
					climber->NewState();
				}
				
				lastLeftTriggerPress = leftStick->GetRawButton(2);
				climber->AutoClimb(climberState);
			}

			//SmartDashboard::PutNumber("Climber State", climberState);
			if(!autoClimb)
			{
				if(leftStick->GetRawButton(4))
					climber->GetLeftTower()->Reach();
				else if(leftStick->GetRawButton(5))
					climber->GetLeftTower()->Climb();
				/*else if(!leftStick->GetRawButton(4) && !leftStick->GetRawButton(5)
					&& !climber->InState())
				climber->GetLeftTower()->StopMoving();*/

				if(rightStick->GetRawButton(4))
					climber->GetRightTower()->Reach();
				else if(rightStick->GetRawButton(5))
					climber->GetRightTower()->Climb();
				/*else if(!rightStick->GetRawButton(4) && !rightStick->GetRawButton(5) 
					&& !climber->InState())
				climber->GetRightTower()->StopMoving();*/

				if(rightStick->GetRawButton(3))
				{
					climber->GetLeftTower()->Reach();
					climber->GetRightTower()->Reach();
				}
				else if(rightStick->GetRawButton(2))
				{
					climber->GetLeftTower()->Climb();
					climber->GetRightTower()->Climb();
				}

				if(!rightStick->GetRawButton(3) && !rightStick->GetRawButton(2)
						&& !rightStick->GetRawButton(4) && !rightStick->GetRawButton(5)
						&& !leftStick->GetRawButton(4) && !leftStick->GetRawButton(5)
						&& !climber->InState())
				{
					climber->GetLeftTower()->StopMoving();
					climber->GetRightTower()->StopMoving();
					climber->GetLeftTower()->ResetReach();
					climber->GetLeftTower()->ResetClimb();
					climber->GetRightTower()->ResetReach();
					climber->GetRightTower()->ResetClimb();
				}

				if(leftStick->GetTrigger())
				{
					climber->GetLeftTower()->ResetReach();
					climber->GetLeftTower()->ResetClimb();
				}
				if(rightStick->GetTrigger())
				{
					climber->GetRightTower()->ResetReach();
					climber->GetRightTower()->ResetClimb();
				}

				/*if(leftStick->GetRawButton(3))
			{
				climber->GetLeftTower()->ResetEncoder();
				climber->GetRightTower()->ResetEncoder();
				climber->GetCandyCaneClass()->ResetEncoder();
			}*/
				//Reset the encoders
				if(rightStick->GetRawButton(6))
				{
					climber->GetLeftTower()->ResetEncoder();
					climber->GetRightTower()->ResetEncoder();
					climber->GetCandyCaneClass()->ResetEncoder();
				}

				if(rightStick->GetRawButton(8))
				{
					climber->GetCandyCaneClass()->ServoPosition();
				}
				else if(rightStick->GetRawButton(9))
				{
					climber->GetCandyCaneClass()->ServoNeutral();
				}

				if(rightStick->GetRawButton(10))
				{
					climber->GetCandyCaneClass()->BringInOverride();
				}
				else if(rightStick->GetRawButton(11))
				{
					climber->GetCandyCaneClass()->PushOut();
				}
				//If the cc is not doing a function and none of the buttons are pressed, stop the cc
				else if(!rightStick->GetRawButton(10) && !rightStick->GetRawButton(11)
						&& climber->GetCandyCaneClass()->GetState() == CC_IDLE)
				{
					climber->GetCandyCaneClass()->Stop();
				}

				//CC functions
				if(leftStick->GetRawButton(6))
				{
					climber->GetCandyCaneClass()->StartGrab(6000);
				}
				else if(leftStick->GetRawButton(7))
				{
					climber->GetCandyCaneClass()->StartRelease();
				}
				else if(leftStick->GetRawButton(8))
				{
					climber->GetCandyCaneClass()->StartSafety(CC_FULL_SAFETY);
				}
				else if(leftStick->GetRawButton(9))
				{
					climber->GetCandyCaneClass()->StartLeanIn(6000);
				}
				else if(leftStick->GetRawButton(10))
				{
					climber->GetCandyCaneClass()->Stop();
				}
				climber->GetCandyCaneClass()->Grab();
				climber->GetCandyCaneClass()->Release();
				climber->GetCandyCaneClass()->LeanIn();
				climber->GetCandyCaneClass()->Safety();
			}

			if(xboxController->GetRightYAxis() < -0.5)
			{
				dumperDevice->StartExtend();
				lastRJoyStop = true;
				input = true;
			}
			else if(xboxController->GetRightYAxis() > 0.5)
			{
				dumperDevice->StartReset();
				lastRJoyStop = true;
				input = true;
			}
			else
			{
				if(lastRJoyStop)
				{
					lastRJoyStop = false;
					dumperDevice->Stop();
				}
			}

			if(xboxController->GetTriggerAxis() < -0.5)
			{
				dumperDevice->OpenGate();
				input = true;
			}
			else if(xboxController->GetTriggerAxis() > 0.5)
			{
				dumperDevice->CloseGate();
				input = true;
			}

			if(xboxController->IsYPressed() && !lastYButtonPress)
			{
				dumperDevice->StartReset();
				dumperDevice->CloseGate();
				input = true;
			}
			lastYButtonPress = xboxController->IsYPressed();

			if(xboxController->IsAPressed() && !lastAButtonPress)
			{
				dumperDevice->StartExtend();
				input = true;
			}
			lastAButtonPress = xboxController->IsAPressed();

			if(isStopping)
				isStopping = input;

			if(xboxController->IsBPressed() || isStopping)
			{
				dumperDevice->Stop();
				isStopping = true;
			}

			//If the CC latched on to the tower with little time left, the second driver can extend the CC so the drive team is able to get the robot off the tower
			if(xboxController->IsLeftStickPressed() && xboxController->IsRightStickPressed())
			{
				climber->GetCandyCaneClass()->StartSafety(10000);
			}
			climber->GetCandyCaneClass()->Safety();

			input = false;

			dumperDevice->Reset();
			dumperDevice->Extend();

			lcd->Printf(DriverStationLCD::kUser_Line1, 1, "CC %f", climber->GetCandyCaneClass()->getRailPos());
			lcd->Printf(DriverStationLCD::kUser_Line2, 1, "left %f", climber->GetLeftTower()->GetDistance());
			lcd->Printf(DriverStationLCD::kUser_Line3, 1, "Right %f", climber->GetRightTower()->GetDistance());
			lcd->Printf(DriverStationLCD::kUser_Line4, 1, "%i", climberState);
			lcd->Printf(DriverStationLCD::kUser_Line5, 1, "inState %s", (climber->InState())?"true":"false");
			lcd->Printf(DriverStationLCD::kUser_Line6, 1, "auto %s", (autoClimb)?"true":"false");
			lcd->UpdateLCD();

			robot->TankDrive(leftStick->GetY() * -1, rightStick->GetY() * -1);

			Wait(0.005);
		}
	}

	void Test() {
		while(IsTest())
		{
			GetWatchdog().Feed();
		}
	}
};

START_ROBOT_CLASS(RobotDemo);
