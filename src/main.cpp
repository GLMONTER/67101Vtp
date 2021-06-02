#include "main.h"
extern void trackPosition();

void initialize() 
{
	pros::lcd::initialize();
	pros::Task TRACKPOS(trackPosition);
}


void disabled() {}

void competition_initialize() {}


void autonomous() {}


extern bool runningAuton;
extern void moveToPoint(const float x, const float y, const float angle);
void opcontrol() 
{
	runningAuton = false;
	while(true)
	{
		int32_t ch1 = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		int32_t ch2 = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
		int32_t ch3 = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int32_t ch4 = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

		leftFront.move(ch3 + ch1 + ch4);
		rightFront.move(ch3 - ch1 - ch4);
		leftBack.move(ch3 + ch1 - ch4);
		rightBack.move(ch3 - ch1 + ch4);

		pros::delay(5);
	}
}
