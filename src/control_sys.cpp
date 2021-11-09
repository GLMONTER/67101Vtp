#include"control_sys.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::Motor rightBack(19, pros::E_MOTOR_GEARSET_18, true);
pros::Motor leftBack(8, pros::E_MOTOR_GEARSET_18, false);
pros::Motor rightFront(11, pros::E_MOTOR_GEARSET_18, false);
pros::Motor leftFront(20, pros::E_MOTOR_GEARSET_18, true);

pros::Motor intake(1, pros::E_MOTOR_GEARSET_18, false);

pros::Motor frontGoalLift(13, pros::E_MOTOR_GEARSET_36, false);

pros::Motor rearGoalLiftRight(3, pros::E_MOTOR_GEARSET_36, false);
pros::Motor rearGoalLiftLeft(2, pros::E_MOTOR_GEARSET_36, false);

pros::ADIDigitalIn buttonLimit('H');
//when enabled, systems like the goal lifts will check internal encoder position to make sure there is no 
//mechanical damage from rotating the mechanism too far
#define ENABLE_CHECKS false

enum CheckStates 
{
    larger = 0,
    smaller = 1,
    correct = 2
};

void setDrive(const int32_t leftPower, const int32_t rightPower)
{
    rightBack.move(rightPower);
    leftBack.move(leftPower);
}
//generic function to verify positions of mechanims that can break when over-rotated.
int checkPosition(int32_t topValue, int32_t botValue, int32_t currentValue)
{
    if(currentValue > topValue)
        return CheckStates::larger;
    if(currentValue < botValue)
        return CheckStates::smaller;
    else
        return CheckStates::correct;
}

void moveGoalLift()
{
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
    {
        frontGoalLift.move_velocity(127);
    }
    else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
    {
        if(!buttonLimit.get_value())
            frontGoalLift.move_velocity(-127);
    }
    else
    {
        frontGoalLift.move_velocity(0);
    }
    pros::lcd::print(0, "%f", frontGoalLift.get_position());
 //lLift.get_position());
    //rear goal lift
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
    {
        rearGoalLiftLeft.move(-127);
        rearGoalLiftRight.move(127);

        /*
        rearGoalLiftLeft.move_absolute(0, 200);
        rearGoalLiftRight.move_absolute(0, 200);
        */
    }
    else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
    {
        /*
        rearGoalLiftLeft.move_absolute(-2000, 200);
        rearGoalLiftRight.move_absolute(2000, 200);
        */
        rearGoalLiftLeft.move(127);
        rearGoalLiftRight.move(-127);
    }
    else
    {
        rearGoalLiftLeft.move_velocity(0);
        rearGoalLiftRight.move_velocity(0);

    }
    //std::cout<<goalLift.get_position()<<std::endl;
}

void setLoader(loaderSetting setting)
{
    if(setting == loaderSetting::Disabled)
        intake.move(0);
    else if(setting == loaderSetting::Intake)
        intake.move_velocity(170);
    else if(setting == loaderSetting::Outake)
        intake.move_velocity(-170);
    pros::lcd::print(1, "%f", intake.get_temperature());
}

void controlLoader()
{   
    static int reverseFlag = 0;
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_X))
    {
        setLoader(loaderSetting::Intake);
    }
    else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A))
    {
        setLoader(loaderSetting::Outake);
    }
    else
    {
        setLoader(loaderSetting::Disabled);
    }
    
}
// static float error;
static float integral;
static float derivative;
static float previousError;
//generic PID function
float getNewPIDCONTROL(const float error)
{
    const float Ki = 0.001;
    const float Kd = 0.005f;
    const float Kp = 0.001f;
    //subject to change heading for yaw
    
    integral = integral + error;
    
    if(abs(error) < 5.0f)
    {
        integral = 0.0f;
    }

    derivative = error - previousError;
    previousError = error;
   // std::cout<<(integral*Ki) + (derivative*Kd) + (error*Kp)<<std::endl;
    
    return (integral*Ki) + (derivative*Kd) + (error*Kp);
}