#include "main.h"

void initialize() 
{
	pros::lcd::initialize();
}


void disabled() {}

void competition_initialize() {}


void autonomous() {}


extern bool runningAuton;

void opcontrol() 
{
	fourBar.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	goalLift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	runningAuton = false;
	
	while(true)
	{
		leftFront.move(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
		leftBack.move(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
		rightFront.move(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
		rightBack.move(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));


		moveCascade();
		controlLoader();
		moveGoalLift();

		pros::delay(5);
	}
}
