#include "main.h"

void initialize() 
{
	pros::lcd::initialize();
	//muiltithreaded because we need to run them with the drive code, there are blocking while loops
//	pros::Task MoveCascadeTask(moveCascade);
	//pros::Task RingFlipTask(actuateRingFlip);

}

void disabled() {}

void competition_initialize() {}
extern void runAuton();

void autonomous() 
{
	runAuton();
}
extern bool runningAuton;

void opcontrol() 
{
	cascade.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	frontGoalLift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	rearGoalLift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	topTwist.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);


	runningAuton = false;
	
	while(true)
	{
		int Ch1 = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		int Ch3 = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int Ch4 = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

		leftFront.move(Ch3 + Ch1 + Ch4);
		leftBack.move(Ch3 + Ch1 - Ch4);
		rightFront.move(Ch3 - Ch1 - Ch4);
		rightBack.move(Ch3 - Ch1 + Ch4);
		
		//setCascade();
		moveGoalLift();
		controlLoader();
		//actuateRingFlip();
		//actuatetopTwist();

		pros::delay(10);
	}
}
