#include"main.h"

bool runningAuton = false;

//tracking wheel diameter in inches
#define WHEEL_DIAM 2.783
//calculate how far the wheel will travel in one rotation
float SPIN_TO_IN_LR  = (WHEEL_DIAM * PI / 360.0);
//distance from the left tracking wheel to tracking center
#define L_DISTANCE_IN 4.025
//distance from the right tracking wheel to tracking center
#define R_DISTANCE_IN 4.025
//distance from the rear tracking wheel to tracking center
#define S_DISTANCE_IN 2.25

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


pros::Rotation leftEncoder(17);
pros::Rotation rightEncoder(4);
pros::Rotation middleEncoder(18);

void trackPosition()
{    
    middleEncoder.reverse();
    //reset position because position is a set state
    leftEncoder.reset_position();
    rightEncoder.reset_position();
    middleEncoder.reset_position();

    //update encoders every 5 miliseconds
    leftEncoder.set_data_rate(5);
    rightEncoder.set_data_rate(5);
    middleEncoder.set_data_rate(5);

    while(true)
    {
    //get encoder position based on a 360 tick rotation
    float left = leftEncoder.get_position() / 100;
    float right = rightEncoder.get_position() / 100;
    float back = middleEncoder.get_position() / 100;

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
    
    pros::lcd::print(2, "left :  %d\n", leftEncoder.get_position() / 100);
    pros::lcd::print(3, "right :  %d\n", rightEncoder.get_position() / 100);
    pros::lcd::print(4, "middle :  %d\n", middleEncoder.get_position() / 100);
    
    pros::lcd::print(2, "rotation :  %f\n", gPosition.a);
    
    
    pros::delay(5);
    }
}

float getNewPID(const float error, bool resetFlag)
{
   // static float error;
    static float integral;
    static float derivative;
    static float previousError;
    if(resetFlag)
    {
        integral = 0;
        derivative = 0;
        previousError = 0;
    }
    const float Ki = 0.125;
    const float Kd = 0.1f;
    const float Kp = 1.5f;
    //subject to change heading for yaw
    
    integral = integral + error;
    
    if(abs(error) < 0.5f)
    {
        integral = 0.0f;
    }

    derivative = error - previousError;
    previousError = error;
    return (integral*Ki) + (derivative*Kd) + (error*Kp);
}

void init()
{
    
}
void moveToPoint(const float x, const float y, const float angle, bool goThroughFlag, const uint32_t maxVelocity = 127, uint32_t timeout = 0)
{
    leftFront.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    leftBack.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rightFront.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rightBack.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    
	float positionTamper = 1.0f;
	float angleTamper = 0.05f;
    bool resFlag = false;
    int i = 0;
	while(std::abs(gPosition.x - x) > positionTamper || std::abs(gPosition.y - y) > positionTamper || std::abs(gPosition.a - angle) > angleTamper)
	{
        i += 10;
        if(i > timeout && timeout != 0)
            break;
 		//difference in rotation from current rotation to target rotation
        float differenceOfAngle = (angle - gPosition.a);		 
            
		//the distance between the current position and the desired point plus the scaled angle
		float scaleValue = std::abs(std::sqrt(std::pow((gPosition.x - x), 2) + std::pow((gPosition.y - y), 2))) + (3 * differenceOfAngle);
		//What direction to drive in
        float T = std::atan2((y - gPosition.y), (x - gPosition.x)) + gPosition.a;
       float scaledPID;
		//use pid to move elegantly to the target
        if(!resFlag)
		    scaledPID = getNewPID(scaleValue, true);
        else
            scaledPID = getNewPID(scaleValue, false);
        resFlag = true;
		if(std::abs((sin(T + 0.25*PI) + differenceOfAngle * 3) * scaledPID) > maxVelocity)
        {
            if((sin(T + 0.25*PI) + differenceOfAngle * 3) * scaledPID > 0)
                leftFront.move(maxVelocity);
            else
                leftFront.move(-maxVelocity);
        }
        else
        {
            leftFront.move((sin(T + 0.25*PI) + differenceOfAngle * 3) * scaledPID);
        }
        //right Back
        if(std::abs((sin(T + 0.25*PI) - differenceOfAngle * 3) * scaledPID) > maxVelocity)
        {
            if(((sin(T + 0.25*PI) - differenceOfAngle * 3) * scaledPID) > 0)
                rightBack.move(maxVelocity);
            else
                rightBack.move(-maxVelocity);
        }
        else
        {
            rightBack.move((sin(T + 0.25*PI) - differenceOfAngle * 3) * scaledPID);
        }
        //right Front

        if(std::abs((sin(T - 0.25*PI) - differenceOfAngle * 3) * scaledPID) > maxVelocity)
        {
            if((sin(T - 0.25*PI) - differenceOfAngle * 3) * scaledPID > 0)
                rightFront.move(maxVelocity);
            else
                rightFront.move(-maxVelocity);
        }
        else
        {
            rightFront.move((sin(T - 0.25*PI) - differenceOfAngle * 3) * scaledPID);
        }
        //left Back
        if(std::abs((sin(T - 0.25*PI) + differenceOfAngle * 3) * scaledPID) > maxVelocity)
        {
            if((sin(T - 0.25*PI) + differenceOfAngle * 3) * scaledPID > 0)
                leftBack.move(maxVelocity);
            else
                leftBack.move(-maxVelocity);
        }
        else
        {
            leftBack.move((sin(T - 0.25*PI) + differenceOfAngle * 3) * scaledPID);
        }

		pros::delay(10);
	}
   if(!goThroughFlag)
   {
        //lock the drive after the movement
        leftFront.move_velocity(0);
        leftBack.move_velocity(0);
        rightFront.move_velocity(0);
        rightBack.move_velocity(0);
   }
   else
   {
        leftFront.move(0);
        leftBack.move(0);
        rightFront.move(0);
        rightBack.move(0); 
   }
}
void winPoint()
{
    clawLift.move_relative(-700, 200);
    //move up to goal
    moveToPoint(0, -3, 0, true, 200);
    claw.move_relative(700, 200);
    pros::delay(350);

    //move to side to get ready for straight away
    moveToPoint(16, 0, 0, false, 200);
    //go all of the way to the other side
    moveToPoint(16, -60, 0, false, 200);
    //align with goal for outake
    moveToPoint(9, -85, 2.25, true, 200);
    intake.move(127);
    pros::delay(500);
    intake.move(0);
    //go in front of goal
    moveToPoint(20, -75, 3.14, false, 200);
    //go beside goal
    moveToPoint(30, -88, 3.14, false, 200);
    //push goal
    moveToPoint(0, -88, 3.14, true, 200);
}
void rightQuali()
{
    clawLift.move_relative(-700, 200);
    //move up to goal
    moveToPoint(0, -4, 0, true, 200);
    claw.move_relative(1000, 200);
    pros::delay(500);

    moveToPoint(12, 0, 0, true, 200);
    moveToPoint(12, -30, 0, true, 90);
    moveToPoint(-2, -30, 0, true, 90);
    moveToPoint(-1, -18, 0, true, 90);
    moveToPoint(18, -33, 0, true, 127);
    
    clawLift.move_relative(-300, 100);
    moveToPoint(18, -40, 0, true, 75);
    //clamp
    claw.move_relative(-1300, 200);
    pros::delay(750);
    clawLift.move_relative(-300, 100);

    moveToPoint(22, 0, 0, true, 200);
}
void leftQuali()
{
    clawLift.move_relative(-700, 200);
    //move up to goal
  
    moveToPoint(0, -4, 0, true, 127);
    claw.move_relative(1000, 200);
    pros::delay(600);
    moveToPoint(29.5, -16, -1.45, true, 200);
    clawLift.move_relative(-300, 100);
    moveToPoint(40.5, -16.5, -1.45, true, 75);
    //clamp
    claw.move_relative(-1200, 200);
    pros::delay(750);
    clawLift.move_relative(-400, 100);
    moveToPoint(4, -3, -1.28, true, 100);

}

void skills()
{
    frontGoalLift.move_absolute(-3700, 200);
    pros::delay(1500);
    moveToPoint(0, -6.8, 0, true, 80, 5000);
    moveToPoint(-18.5, 3, 1.57, true, 65, 5000);
    clawLift.move_absolute(-1200, 200);
    claw.move_absolute(1300, 200);
    moveToPoint(-42, 3, 1.57, false, 40, 5000);
    claw.move_absolute(-100, 200);
    pros::delay(700);
    clawLift.move_absolute(-4000, 200);
    pros::delay(1500);
    moveToPoint(-27, 1.6, 1.57, true, 60, 5000);
    moveToPoint(-28, 35, 1.57, true, 70, 5000);
    moveToPoint(-30, 35, 4.57, true, 60, 5000);
    moveToPoint(-19, 35, 4.57, true, 60);
    clawLift.move_absolute(-3200, 200);
    pros::delay(500);
    claw.move_absolute(1300, 200);
    pros::delay(1000);
    moveToPoint(-36, 32, 3.14, true, 70, 5000);
    moveToPoint(-56, 31, 3.14, true, 70, 5000);
    moveToPoint(-65, 31, 3.14, true, 70, 5000);
    clawLift.move_absolute(-1200, 200);
    claw.move_absolute(1300, 200);
    //go get last neutral
    moveToPoint(-50.5, 47, 3.14, true, 70, 5000);
    moveToPoint(-50.5, 57, 3.14, false, 65, 5000);
    claw.move_absolute(-100, 200);
    pros::delay(700);
    clawLift.move_absolute(-4000, 200);
    pros::delay(1500);
    moveToPoint(-30, 30, 4.57, true, 65, 5000);
    moveToPoint(-19, 30, 4.57, true, 65, 5000);
    //lay last neut down
    clawLift.move_absolute(-3200, 200);
    pros::delay(500);
    claw.move_absolute(1300, 200);
    pros::delay(1000);

    moveToPoint(-24, 67.6, 3.14, true, 70, 5000);

    clawLift.move_absolute(-1200, 200);
    claw.move_absolute(1300, 200);

    moveToPoint(-24, 77.7, 3.14, false, 65, 5000);

     claw.move_absolute(-100, 200);

    pros::delay(700);
    moveToPoint(-24, 73, 3.14, false, 65, 5000);
    clawLift.move_absolute(-4000, 200);

    pros::delay(1500);
    moveToPoint(-24, 73, 4.57, false, 70, 2000);
    moveToPoint(-84.5, 77, 4.57, false, 90);
    
}
void leftElim()
{
    moveToPoint(6, 34.5, 0, true, 127);
    moveToPoint(6, 38, 0, true, 70);
    frontGoalLift.move_absolute(-3000, 200);
    
    pros::delay(1500);
    moveToPoint(27.3, 42, -1.45, true, 127);
    moveToPoint(27, 42, -1.45, true, 80);
    clawLift.move_absolute(-1100, 200);
    claw.move_relative(1200, 200);
    //clamp
    claw.move_relative(-1200, 200);
    pros::delay(750);
    clawLift.move_relative(-400, 100);
    moveToPoint(21, 16, -1.45, true, 127);
    
}
//actually running the auton
void runAuton()
{
    runningAuton = true;
    init();
    skills();

    runningAuton = false;
}
