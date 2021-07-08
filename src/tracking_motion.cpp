

#include"main.h"
//for trig functions
#include<cmath>

//tracking wheel diameter in inches
#define WHEEL_DIAM 3.25
//calculate how far the wheel will travel in one rotation
float SPIN_TO_IN_LR  = (WHEEL_DIAM * M_PI / 360.0);
//distance from the left tracking wheel to tracking center
#define L_DISTANCE_IN 7.5
//distance from the right tracking wheel to tracking center
#define R_DISTANCE_IN 7.5
//distance from the rear tracking wheel to tracking center
#define S_DISTANCE_IN 4.75
#define PID_ENABLE true
pros::ADIEncoder leftEncoder(3, 4, false);
pros::ADIEncoder rightEncoder(5, 6, false);
pros::ADIEncoder middleEncoder(1, 2, false);

struct Position
{
    float a;
    float x;
    float y;
    int32_t leftLst;
    int32_t rightLst;
    int32_t backLst;
};

Position position;
void lockDrive()
{
    leftFront.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    leftBack.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rightFront.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rightBack.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    leftFront.move_velocity(0);
    leftBack.move_velocity(0);
    rightFront.move_velocity(0);
    rightBack.move_velocity(0);
}
void setDriveSpec(const int32_t leftFrontV,const int32_t leftBackV, const int32_t rightFrontV, const int32_t rightBackV)
{
    leftFront.move(leftFrontV);
    leftBack.move(leftBackV);
    rightFront.move(rightFrontV);
    rightBack.move(rightBackV);
}

void trackPosition()
{    
    while(true)
    {
    int32_t left = leftEncoder.get_value();
    int32_t right = rightEncoder.get_value();
    int32_t back = middleEncoder.get_value();

    float L = (left - position.leftLst) * SPIN_TO_IN_LR; // The amount the left side of the robot moved
	float R = (right - position.rightLst) * SPIN_TO_IN_LR; // The amount the right side of the robot moved
	float S = (back - position.backLst) * SPIN_TO_IN_LR; // The amount the back side of the robot moved

	// Update the last values
	position.leftLst = left;
	position.rightLst = right;
	position.backLst = back;

	float h; // The hypotenuse of the triangle formed by the middle of the robot on the starting position and ending position and the middle of the circle it travels around
	float i; // Half on the angle that I've traveled
	float h2; // The same as h but using the back instead of the side wheels
	float a = (L - R) / (L_DISTANCE_IN + R_DISTANCE_IN); // The angle that I've traveled
	if (a)
	{
		float r = R / a; // The radius of the circle the robot travel's around with the right side of the robot
		i = a / 2.0;
		float sinI = sin(i);
		h = ((r + R_DISTANCE_IN) * sinI) * 2.0;

		float r2 = S / a; // The radius of the circle the robot travel's around with the back of the robot
		h2 = ((r2 + S_DISTANCE_IN) * sinI) * 2.0;
	}
	else
	{
		h = R;
		i = 0;

		h2 = S;
	}
	float p = i + position.a; // The global ending angle of the robot
	float cosP = cos(p);
	float sinP = sin(p);

	// Update the global position
	position.y += h * cosP;
	position.x += h * sinP;

	position.y += h2 * -sinP;
	position.x += h2 * cosP; 

	position.a += a;
  
    //pros::lcd::print(0, "x :  %f\n", position.x);
    pros::lcd::print(1, "y :  %f\n", position.y);

    pros::lcd::print(2, "left :  %d\n", leftEncoder.get_value());
    pros::lcd::print(3, "right :  %d\n", rightEncoder.get_value());
    pros::lcd::print(4, "middle :  %d\n", middleEncoder.get_value());
    pros::lcd::print(5, "rotation :  %f\n", position.a);

    pros::delay(5);
    }
}


float getNewPID(const float error)
{
   // static float error;
    static float integral;
    static float derivative;
    static float previousError;
    static float driveValue;

    const float Ki = 0.1;
    const float Kd = 3.0f;
    const float Kp = 1.5f;
    //subject to change heading for yaw
    
    integral = integral + error;
    /*
    if(abs(error) < 0.25f)
    {
        integral = 0.0f;
    }
*/
    derivative = error - previousError;
    previousError = error;
    return (integral*Ki) + (derivative*Kd) + (error*Kp);
}
//calculating error between requested position and current position using pid and running the drive train
void moveToPoint(const float x, const float y, const float angle, 
const bool goThroughFlag, const uint32_t maxVelocity = 127, const uint32_t timeout = 0)
{
    static uint32_t timer;
    timer = 0;
    leftFront.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    leftBack.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    rightFront.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    rightBack.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    float posTamper;
    float angleTamper;
    if(goThroughFlag)
    {
        posTamper = 2.0f;
        angleTamper = 0.1;
    }
    else
    {
        posTamper = 1.0f;
        angleTamper = 0.04;
    }
    float averageVelocity = 0;
    while((std::abs(position.x - x) > posTamper) || (std::abs(position.y - y) > posTamper) || (std::abs(position.a - angle) > angleTamper)
    || averageVelocity > 35)
    {
        averageVelocity = (leftFront.get_actual_velocity() + leftBack.get_actual_velocity() + 
        rightFront.get_actual_velocity() + rightBack.get_actual_velocity()) / 4;
        #if PID_ENABLE
        float Speed = 1.0;
        #else
        float Speed = 0.4;
        #endif
        //What direction to drive in
        float T = std::atan2((y - position.y), (x - position.x)) + position.a;

        //refer to movement document
        float Psub1 = -(cos(T + (M_PI / 4)) / cos(M_PI / 4));
        float Psub2 = -(cos(T + (3 * M_PI) / 4)) / (cos(M_PI / 4));

        float SubS = std::max(std::abs(Psub1), std::abs(Psub2)) / Speed;
        //difference in rotation from current rotation to target rotation
        float differenceOfAngle = (angle - position.a);

        float scaleValue = std::abs(std::sqrt(std::abs(position.x - x) + std::abs(position.y - y))) + (3 * differenceOfAngle);

        //calculate motor values from -1 to 1
        float m_FrontLeft = (Psub2/SubS) * (1 - std::abs(differenceOfAngle)) + differenceOfAngle;
        float m_FrontRight = (Psub1/SubS) * (1 - std::abs(differenceOfAngle)) - differenceOfAngle;
        float m_BackLeft = (Psub1/SubS) * (1 - std::abs(differenceOfAngle)) + differenceOfAngle;
        float m_BackRight = (Psub2/SubS) * (1 - std::abs(differenceOfAngle)) - differenceOfAngle;
        float scaledPID = getNewPID(scaleValue);
        std::cout<<"Scaled : "<<scaledPID<<std::endl;
        if(scaledPID > maxVelocity)
            scaledPID = maxVelocity;
        #if PID_ENABLE
        setDriveSpec(m_FrontLeft * scaledPID, m_BackLeft * scaledPID, m_FrontRight * scaledPID, m_BackRight * scaledPID);

        #else
        //actually set drive values and scale to 127
        setDriveSpec(m_FrontLeft * 127, m_BackLeft * 127, m_FrontRight * 127, m_BackRight * 127);
        #endif


        pros::delay(4);
        timer += 5;
        if((timeout != 0) && (timeout < timer))
        {
            break;
        }
    }
    //turn off drive when done.
    if(!goThroughFlag)
        lockDrive();
    else
        setDrive(0,0);
    pros::delay(50);
}
