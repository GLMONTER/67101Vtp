#include "main.h"

extern void trackPosition();
extern void threadMacro();

void initialize() 
{
    pros::lcd::initialize();

	pros::Task trackingTask(trackPosition);
    pros::Task macroTask(threadMacro);
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
    claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    clawLift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    runningAuton = false;
   
    while(true)
    {
        int Ch1 = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int Ch3 = -controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int Ch4 = -controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

        leftFront.move(Ch3 + Ch1 + Ch4);
        leftBack.move(Ch3 + Ch1 - Ch4);
        rightFront.move(Ch3 - Ch1 - Ch4);
        rightBack.move(Ch3 - Ch1 + Ch4);
        
        intake.move(127);

        moveGoalLift();
        pros::delay(10);
    }
}

