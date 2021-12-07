#include"control_sys.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::Motor rightBack(19, pros::E_MOTOR_GEARSET_18, true);
pros::Motor leftBack(8, pros::E_MOTOR_GEARSET_18, false);
pros::Motor rightFront(11, pros::E_MOTOR_GEARSET_18, false);
pros::Motor leftFront(20, pros::E_MOTOR_GEARSET_18, true);

pros::Motor intake(1, pros::E_MOTOR_GEARSET_18, false);

pros::Motor frontGoalLift(13, pros::E_MOTOR_GEARSET_36, false);

pros::Motor claw(3, pros::E_MOTOR_GEARSET_36, true);
pros::Motor clawLift(2, pros::E_MOTOR_GEARSET_36, false);

pros::ADIDigitalIn buttonLimit('H');
//allow macro systems to be overrided
bool overrideFlag = false;
//to check if we are running the auton, for use in the multithreaded task
extern bool runningAuton;

void threadMacro()
{
    int state = 0;

    int upPressed = 0;
    int upToggle = 0;

    int downPressed = 0;
    int downToggle = 0;
    
    while(true)
    {
        if(overrideFlag || runningAuton)
        {
            pros::delay(10);
            continue;
        }
        //up toggle
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
        {
            if(!upPressed)
            {
                upToggle = 1 - upToggle;

                upPressed = 1;
            }
        }
        else
            upPressed = 0;

        //down toggle
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))
        {
            if(!downPressed)
            {
                downToggle = 1 - downToggle;

                downPressed = 1;
            }
        }
        else
            downPressed = 0;

        //actually execute
        if(upToggle)
        {
            if(state != 3)
                state++;
            upToggle = false;
        }
        if(downToggle)
        {
            if(state != 0)
                state--;
            downToggle = false;
        }
        //go to macro
        if(state == 0)
        {
            clawLift.move_absolute(0, 200);
        }
        else if(state == 1)
        {
            clawLift.move_absolute(-1200, 200);
        }
        else if(state == 2)
        {
            clawLift.move_absolute(-3200, 200);
        }
        else if(state == 3)
        {
            clawLift.move_absolute(-4000, 200);
        }
        pros::delay(10);
    }
}

void moveGoalLift()
{
    //enables or disables the override flag to give the drive different controls.
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_B) && controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
        overrideFlag = true;
    if(overrideFlag && controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP) && controller.get_digital(pros::E_CONTROLLER_DIGITAL_X))
        overrideFlag = false;

    //check to make sure the front goal lift stays above its range
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && frontGoalLift.get_position() < 0)
    {
        frontGoalLift.move_velocity(127);
    }
    else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
    {
        if(!buttonLimit.get_value())
            frontGoalLift.move_velocity(-127);
    }
    else
    {
        frontGoalLift.move_velocity(0);
    }
    static bool clampFlag = false;
    if(clampFlag)
    {
        if(claw.get_torque() < 2)
        {
            claw.move(-127);
        }
        
    }
    pros::lcd::print(5, "%f", claw.get_position());
    //move the claw
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
    {
        clampFlag = true;
    }
    else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
    {
        clampFlag = false;
        claw.move_absolute(0, 200);
    }
    else if((claw.get_position() > -50 && claw.get_position() < 50) && !clampFlag)
    {
        claw.move_velocity(0);
    }
    
    //if override is enabled, manually control claw lift
    if(overrideFlag)
    {
        //claw and claw lift
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT))
        {
            clawLift.move(-127);
        }
        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
        {
            clawLift.move(127);
        }
        else
        {
            clawLift.move_velocity(0);
        }
    }
}