#include "main.h"

extern void trackPosition();
extern void threadMacro();
pros::Optical intakeDetect(6);
int flag = true;

void intakeSense()
{
    while(true)
    {
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT))
            break;
        pros::delay(10);
    }
    flag = false;
    intake.move_velocity(140);
    intakeDetect.set_led_pwm(100);
    while(true)
    {
        pros::lcd::print(6, "%f", intake.get_position());
        if(intakeDetect.get_hue() > 80)
        {
            static int temp  = intake.get_position();
            intake.move_relative(700, 75);
            while(std::abs(intake.get_position() - temp) < 300)
            {
                pros::delay(10);
            }
            temp = intake.get_position();
            intake.move_relative(-500, 150);
            while(std::abs(intake.get_position() - temp) < 150)
            {
                pros::delay(10);
            }
            intake.move_velocity(200);
            pros::delay(1000);
            intake.move_velocity(0);
            break;
        }
        pros::delay(10);
    }
    flag = true;
}
void initialize() 
{
    pros::lcd::initialize();

	pros::Task trackingTask(trackPosition);
    pros::Task macroTask(threadMacro);
   // pros::Task intakeTask(intakeSense);
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
    intake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

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
        /*
        if(flag)
        {
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_B))
            intake.move_velocity(150);
        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A))
            intake.move_velocity(-150);
        else
            intake.move(0);
        }
        */
       intake.move(127);
        moveGoalLift();
        pros::delay(10);
    }
}

