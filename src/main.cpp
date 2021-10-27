#include "main.h"

pros::IMU gyro(5);
extern void trackPosition();
void initialize() 
{
    gyro.reset();
    while (gyro.is_calibrating())
    {
        pros::delay(10);
    }
    
    pros::lcd::initialize();
	//pros::Task trackingTask(trackPosition);
}
pros::Distance rightDistanceSensor(5);
pros::Distance leftDistanceSensor(19); 

void gyroTurn(float deg)
{
    leftFront.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rightFront.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    leftBack.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rightBack.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    float error = 10.0;
    float integral = 0.0;
    float derivative = 0.0;
    float perror = 0.0;
    float value = 0.0;

    float target = deg;
    float Ki = -0.0005;
    float Kd = -1.0;
    float Kp = -1.0;

    while (abs(error) > 0.1 || leftFront.get_actual_velocity() > 0.1)
    {
        pros::lcd::print(0, "val: %f\n", gyro.get_yaw());
        error =  target - gyro.get_yaw();
        printf("%f \n", error);
        integral = integral + error;
        if (abs(error) < 2)
        {
            integral = 0.0;
        }
        derivative = error - perror;
        perror = error;
        value = (integral*Ki) + (derivative*Kd) + (error*Kp);

        leftBack.move(-value);
        rightBack.move(value);
        leftFront.move(-value);
        rightFront.move(value);

        pros::delay(5);
    }
}

enum directionToStrafe
{
    left = 0, right = 1
};

void strafe(int strafeDirection, int speed)
{
    if(strafeDirection == directionToStrafe::right)
    {
        leftFront.move(speed);
        rightFront.move(speed);
        leftBack.move(speed);
        rightFront.move(speed);
    }
}
void maintainDistanceFromWall()
{
    //if we are too close to a wall
    if(std::abs(rightDistanceSensor.get() - leftDistanceSensor.get()))
    {
        if(rightDistanceSensor.get() > leftDistanceSensor.get())
        {
            strafe(directionToStrafe::right, 40);
        }
    }
}
void maintainRotation()
{
    pros::lcd::print(2, "left :  %f\n", gyro.get_yaw());
}
void climb()
{
    gyro.reset();
    pros::delay(2000);


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
    cascade.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    frontGoalLift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    rearGoalLift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    topTwist.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);


    runningAuton = false;

    
    while(true)
    {
        
        maintainRotation();
        int Ch1 = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int Ch3 = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int Ch4 = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

        leftFront.move(Ch3 + Ch1 + Ch4);
        leftBack.move(Ch3 + Ch1 - Ch4);
        rightFront.move(Ch3 - Ch1 - Ch4);
        rightBack.move(Ch3 - Ch1 + Ch4);
        
        //setCascade();
        moveGoalLift();
        //controlLoader();
        //actuateRingFlip();
        //actuatetopTwist();

        pros::delay(10);
    }
}

