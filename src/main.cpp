#include "main.h"


extern void trackPosition();
void initialize() 
{
    /*
    gyro.reset();
    while (gyro.is_calibrating())
    {
        pros::delay(10);
    }
    */
    pros::lcd::initialize();
	//pros::Task trackingTask(trackPosition);
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
    frontGoalLift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    rearGoalLiftRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    rearGoalLiftLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    runningAuton = false;
   
    while(true)
    {
        leftFront.move(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
        leftBack.move(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));

        rightFront.move(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
        rightBack.move(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));

        
        moveGoalLift();
        controlLoader();
        pros::delay(10);
    }
}

