#include"control_sys.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::Motor rightFront(9, pros::E_MOTOR_GEARSET_18, true);
pros::Motor leftFront(10, pros::E_MOTOR_GEARSET_18, false);
pros::Motor rightBack(1, pros::E_MOTOR_GEARSET_18, true);
pros::Motor leftBack(2, pros::E_MOTOR_GEARSET_18, false);

pros::Motor fourBar(11, pros::E_MOTOR_GEARSET_18, false);
pros::Motor intake(12, pros::E_MOTOR_GEARSET_18, false);

//both pneumatics
pros::ADIDigitalOut pneumaticPrimary('A', LOW);
pros::ADIDigitalOut pneumaticSecondary('B', LOW);

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
    pros::lcd::print(0, "%f" ,fourBar.get_position());
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT))
    {
        while(std::abs(fourBar.get_position() - 1300) > 50)
        {
            fourBar.move(-getNewPIDCONTROL(fourBar.get_position() - 1300));
            pros::delay(5);
        }
        fourBar.move_velocity(0);
        while (true)
        {
            pros::delay(5);
        }
        
    }
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
    {
        fourBar.move(127);
    }
    else
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
    {
        fourBar.move(-127);
    }
    else
    {
        fourBar.move_velocity(0);
    }
}
