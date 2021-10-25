#include"main.h"

bool runningAuton = false;
/*
auto chassis = ChassisControllerBuilder()
    .withMotors({8, 20}, {1, 11}) // Left motor is 1, right motor is 2 (reversed)
    // Green gearset, 4 inch wheel diameter, 11.5 inch wheel track
    .withDimensions(AbstractMotor::gearset::green, {{4_in, 11.5_in}, imev5GreenTPR})
     // Use the same scales as the chassis (above)
    .build();
  */

//tracking wheel diameter in inches
#define WHEEL_DIAM 2.783
//calculate how far the wheel will travel in one rotation
float SPIN_TO_IN_LR  = (WHEEL_DIAM * PI / 360.0);
//distance from the left tracking wheel to tracking center
#define L_DISTANCE_IN 7.5
//distance from the right tracking wheel to tracking center
#define R_DISTANCE_IN 7.5
//distance from the rear tracking wheel to tracking center
#define S_DISTANCE_IN 4.75

typedef struct _pos
{
	float a;
	float y;
	float x;
	int leftLst;
	int rightLst;
	int backLst;
} sPos; // Position of the robot

sPos gPosition;

void winPoint()
{
    
}

void skills()
{

}

pros::ADIEncoder leftEncoder(3, 4, false);
pros::ADIEncoder rightEncoder(5, 6, false);
pros::ADIEncoder middleEncoder(9, 9, false);

void trackPosition()
{    
    while(true)
    {
    int32_t left = leftEncoder.get_value();
    int32_t right = rightEncoder.get_value();
    int32_t back = middleEncoder.get_value();

    float L = (left - gPosition.leftLst) * SPIN_TO_IN_LR; // The amount the left side of the robot moved
	float R = (right - gPosition.rightLst) * SPIN_TO_IN_LR; // The amount the right side of the robot moved
	float S = (back - gPosition.backLst) * SPIN_TO_IN_LR; // The amount the back side of the robot moved

	// Update the last values
	gPosition.leftLst = left;
	gPosition.rightLst = right;
	gPosition.backLst = back;

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
	float p = i + gPosition.a; // The global ending angle of the robot
	float cosP = cos(p);
	float sinP = sin(p);

	// Update the global position
	gPosition.y += h * cosP;
	gPosition.x += h * sinP;

	gPosition.y += h2 * -sinP;
	gPosition.x += h2 * cosP; 

	gPosition.a += a;
  
    pros::lcd::print(0, "x :  %f\n", gPosition.x);
    pros::lcd::print(1, "y :  %f\n", gPosition.y);

    pros::lcd::print(2, "left :  %d\n", leftEncoder.get_value());
    pros::lcd::print(3, "right :  %d\n", rightEncoder.get_value());
    pros::lcd::print(4, "middle :  %d\n", middleEncoder.get_value());
    pros::lcd::print(5, "rotation :  %f\n", gPosition.a);

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

void moveToPoint(const float x, const float y, const float angle, bool goThroughFlag, const uint32_t maxVelocity = 127, uint32_t timeout = 0)
{
	float positionTamper = 1.0f;
	float angleTamper = 1.0f;
	while(std::abs(gPosition.x - x) > positionTamper || std::abs(gPosition.y - y) > positionTamper || std::abs(gPosition.a - x) > angleTamper)
	{
 		//difference in rotation from current rotation to target rotation
        float differenceOfAngle = (angle - gPosition.a);		 

		//the distance between the current position and the desired point plus the scaled angle
		float scaleValue = std::abs(std::sqrt(std::abs(gPosition.x - x) + std::abs(gPosition.y - y))) + (3 * differenceOfAngle);

		//What direction to drive in
        float T = std::atan2((y - gPosition.y), (x - gPosition.x)) + gPosition.a;

		//use pid to move elegantly to the target
		float scaledPID = getNewPID(scaleValue);
		
		leftFront.move((sin(T + 1/4*PI) + differenceOfAngle) * scaledPID);
		rightBack.move((sin(T + 1/4*PI) - differenceOfAngle) * scaledPID);

		rightFront.move((sin(T - 1/4*PI) - differenceOfAngle) * scaledPID);
		leftBack.move((sin(T - 1/4*PI) + differenceOfAngle) * scaledPID);

		pros::delay(5);
	}
}
void init()
{
    
}
void rightSide()
{

}
//actually running the auton
void runAuton()
{
    runningAuton = true;
    init();
    rightSide();
    runningAuton = false;
}
