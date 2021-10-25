#include"control_sys.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::Motor rightBack(1, pros::E_MOTOR_GEARSET_18, true);
pros::Motor leftBack(8, pros::E_MOTOR_GEARSET_18, false);
pros::Motor rightFront(11, pros::E_MOTOR_GEARSET_18, true);
pros::Motor leftFront(20, pros::E_MOTOR_GEARSET_18, false);

pros::Motor topTwist(2, pros::E_MOTOR_GEARSET_18, false);


pros::Motor cascade(5, pros::E_MOTOR_GEARSET_18, false);
pros::Motor ringFlip(4, pros::E_MOTOR_GEARSET_18, false);
pros::Motor intake(6, pros::E_MOTOR_GEARSET_18, false);

pros::Motor frontGoalLift(10, pros::E_MOTOR_GEARSET_36, false);
pros::Motor rearGoalLift(3, pros::E_MOTOR_GEARSET_36, false);

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
void actuateRingFlip()
{
    std::cout<<"test";
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))
    {
        ringFlip.move(20);
    }
    else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT))
    {
        ringFlip.move(-20);
    }
    else
    {
        ringFlip.move_velocity(0);
    }
    //pros::delay(10);
}
void actuatetopTwist()
{
    std::cout<<"test";
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
    {
        topTwist.move(100);
    }
    else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_B))
    {
        topTwist.move(-100);
    }
    else
    {
        topTwist.move_velocity(0);
    }
    //pros::delay(10);
}

void moveGoalLift()
{
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
    {
        frontGoalLift.move_absolute(0, 200);
    }
    else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
    {
        frontGoalLift.move_absolute(-3700, 200);
    }
    else
    {
        frontGoalLift.move_velocity(0);
    }
    //rear goal lift
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
    {
        rearGoalLift.move_absolute(0, 200);
    }
    else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
    {
        rearGoalLift.move_absolute(-2000, 200);
    }
    else
    {
        rearGoalLift.move_velocity(0);
    }
    //std::cout<<goalLift.get_position()<<std::endl;
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

    if(std::abs(cascade.get_position() - setPosition) > 20)
    {
        previousError = 0;
        integral = 0;
        derivative = 0;
        
         while(std::abs(cascade.get_position() - setPosition) > 20)
        {
            cascade.move(-getNewPIDCONTROL(cascade.get_position() - setPosition));
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
   // pros::delay(10);
}
void setCascade()
{
    pros::lcd::print(0, "%f" ,cascade.get_position());
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP))
    {
        cascade.move(-127);
    }
    else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
    {
        cascade.move(127);
    }
    else
    {
        cascade.move_velocity(0);
    }
}
