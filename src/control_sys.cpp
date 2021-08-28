#include"control_sys.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::Motor rightFront(10, pros::E_MOTOR_GEARSET_18, true);
pros::Motor leftFront(3, pros::E_MOTOR_GEARSET_18, false);
pros::Motor rightBack(1, pros::E_MOTOR_GEARSET_18, true);
pros::Motor leftBack(2, pros::E_MOTOR_GEARSET_18, false);

pros::Motor cascade(11, pros::E_MOTOR_GEARSET_18, false);
pros::Motor intake(12, pros::E_MOTOR_GEARSET_18, true);
pros::Motor goalLift(15, pros::E_MOTOR_GEARSET_36, false);

//both pneumatics
pros::ADIDigitalOut pneumaticPrimary('A', LOW);
pros::ADIDigitalOut pneumaticSecondary('B', LOW);

//when enabled, systems like the goal lifts will check internal encoder position to make sure there is no 
//mechanical damage from rotating the mechanism too far
#define ENABLE_CHECKS false

const float rightCascadeRatio = 30.0f/36.0f;

enum CheckStates 
{
    larger = 0,
    smaller = 1,
    correct = 2
};

enum cascadeStates
{
    bottom = 0,
    middle = 1,
    top = 2,


    bottomPosition = 0,
    middlePosition = 500,
    topPosition = 1000
};
int cascadeCurrentState = cascadeStates::bottom;

void extendPneumatics()
{
    pneumaticPrimary.set_value(HIGH);
    pneumaticSecondary.set_value(HIGH);
}

void retractPneumatics()
{
    pneumaticPrimary.set_value(LOW);
    pneumaticSecondary.set_value(LOW);
}

void setDrive(const int32_t leftPower, const int32_t rightPower)
{
    rightFront.move(rightPower);
    rightBack.move(rightPower);
    leftFront.move(leftPower);
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
        if(!ENABLE_CHECKS)
        {
            goalLift.move(127);
        }
        else
        //multiply by negative 1 to get correct top and bottom value.
        if(checkPosition(-1500, 0, goalLift.get_position() * -1) == CheckStates::correct || 
        checkPosition(-1500, 0, goalLift.get_position() * -1) == CheckStates::smaller)
        {
            goalLift.move(127);
        }
    }
    else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
    {
        if(!ENABLE_CHECKS)
        {
            goalLift.move(-127);
        }
        if(checkPosition(-1500, 0, goalLift.get_position() * -1) == CheckStates::correct || 
        checkPosition(-1500, 0, goalLift.get_position() * -1) == CheckStates::larger)
        {
            goalLift.move(-127);
        }
    }
    else
    {
        goalLift.move_velocity(0);
    }
    std::cout<<goalLift.get_position()<<std::endl;
}
void setLoader(loaderSetting setting)
{
    if(setting == loaderSetting::Disabled)
        intake.move(0);
    else if(setting == loaderSetting::Intake)
        intake.move(127);
    else if(setting == loaderSetting::Outake)
        intake.move(-127);
}
void controlLoader()
{
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
    {
        setLoader(loaderSetting::Intake);
    }
    else
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
    {
        setLoader(loaderSetting::Outake);
    }
    else
    {
        setLoader(loaderSetting::Disabled);
    }
}
//generic PID function
float getNewPIDCONTROL(const float error)
{
   // static float error;
    static float integral;
    static float derivative;
    static float previousError;
    static float driveValue;

    const float Ki = 0.005;
    const float Kd = 0.01f;
    const float Kp = 0.04f;
    //subject to change heading for yaw
    
    integral = integral + error;
    
    if(abs(error) < 0.25f)
    {
        integral = 0.0f;
    }

    derivative = error - previousError;
    previousError = error;
    std::cout<<(integral*Ki) + (derivative*Kd) + (error*Kp)<<std::endl;
    
    return (integral*Ki) + (derivative*Kd) + (error*Kp);
}
void moveCascade()
{
    static int setPosition;
    if(cascadeCurrentState == cascadeStates::bottom)
        setPosition = cascadeStates::bottomPosition;
    else if(cascadeCurrentState == cascadeStates::middle)
        setPosition = cascadeStates::middlePosition;
    else if(cascadeCurrentState == cascadeStates::top)
        setPosition = cascadeStates::topPosition;
    if(std::abs(cascade.get_position() - setPosition) > 50)
    {
         while(std::abs(cascade.get_position() - setPosition) > 50)
        {
            cascade.move(-getNewPIDCONTROL(cascade.get_position() - setPosition));
            intake.move((-getNewPIDCONTROL(cascade.get_position() - setPosition)) * rightCascadeRatio);
            pros::delay(5);
        }
        cascade.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        cascade.move_velocity(0);
    }
    else
    {
        cascade.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        cascade.move_velocity(0);
    }
}
void setCascade()
{
    pros::lcd::print(0, "%f" ,cascade.get_position());
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT))
    {
        if(cascadeCurrentState != cascadeStates::bottom)
            cascadeCurrentState -= 1;
    }
    else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))
    {
        if(cascadeCurrentState != cascadeStates::top)
            cascadeCurrentState += 1;
    }
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_X))
    {
        cascade.move(-127);
        intake.move((int)(-127.0f * rightCascadeRatio));
    }
    else
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_B))
    {
        cascade.move(127);
        intake.move((int)(127.0f * rightCascadeRatio));
    }
    else
    {
        cascade.move_velocity(0);
        intake.move(0);
    }
}
