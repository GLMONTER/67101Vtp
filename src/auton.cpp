#include"main.h"

bool runningAuton = false;
#define Win true
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
    #if Win
    const float Ki = 0.125;
    const float Kd = 0.1f;
    const float Kp = 1.5f;
    #else
    const float Ki = 0.3;
    const float Kd = 0.1f;
    const float Kp = 2.0f;
    #endif
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
		if(std::abs((sin(T + 0.25*PI) + differenceOfAngle * 4) * scaledPID) > maxVelocity)
        {
            if((sin(T + 0.25*PI) + differenceOfAngle * 4) * scaledPID > 0)
                leftFront.move(maxVelocity);
            else
                leftFront.move(-maxVelocity);
        }
        else
        {
            leftFront.move((sin(T + 0.25*PI) + differenceOfAngle * 4) * scaledPID);
        }
        //right Back
        if(std::abs((sin(T + 0.25*PI) - differenceOfAngle * 4) * scaledPID) > maxVelocity)
        {
            if(((sin(T + 0.25*PI) - differenceOfAngle * 4) * scaledPID) > 0)
                rightBack.move(maxVelocity);
            else
                rightBack.move(-maxVelocity);
        }
        else
        {
            rightBack.move((sin(T + 0.25*PI) - differenceOfAngle * 4) * scaledPID);
        }
        //right Front

        if(std::abs((sin(T - 0.25*PI) - differenceOfAngle * 4) * scaledPID) > maxVelocity)
        {
            if((sin(T - 0.25*PI) - differenceOfAngle * 4) * scaledPID > 0)
                rightFront.move(maxVelocity);
            else
                rightFront.move(-maxVelocity);
        }
        else
        {
            rightFront.move((sin(T - 0.25*PI) - differenceOfAngle * 4) * scaledPID);
        }
        //left Back
        if(std::abs((sin(T - 0.25*PI) + differenceOfAngle * 4) * scaledPID) > maxVelocity)
        {
            if((sin(T - 0.25*PI) + differenceOfAngle * 4) * scaledPID > 0)
                leftBack.move(maxVelocity);
            else
                leftBack.move(-maxVelocity);
        }
        else
        {
            leftBack.move((sin(T - 0.25*PI) + differenceOfAngle * 4) * scaledPID);
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
void winPointold()
{
     frontGoalLift.move_absolute(-3700, 200);
    pros::delay(1500);
    intake.move(127);
    moveToPoint(0, -6.8, 0, true, 80, 3000);
    intake.move(0);
  //  moveToPoint(-24, -6.8, 0, true, 100, 3000);
    moveToPoint(-20, -6.8, 3.14, true, 80, 3000);
    clawLift.move_absolute(-1200, 200);
    moveToPoint(-20, 72, 3.14, true, 90, 5000);
    claw.move_absolute(1300, 200);
    pros::delay(500);
    moveToPoint(-38, 72, 3.14, true, 80, 3000);
    moveToPoint(-38, 80, 3.14, true, 80, 3000);
    moveToPoint(-14, 80, 3.14, true, 80, 3000);
}
void winPoint()
{
    frontGoalLift.move_absolute(-3800, 200);
    pros::delay(800);
    intake.move(127);
    moveToPoint(0, -4.8, 0, true, 80, 3000);
    intake.move(0);
  //  moveToPoint(-24, -6.8, 0, true, 100, 3000);
    moveToPoint(-20, -4.8, 3.14, true, 80, 3000);
    clawLift.move_absolute(-1200, 200);
    moveToPoint(-16.5, 74.5, 3.14, true, 85, 4000);
    claw.move_absolute(1300, 200);
    pros::delay(500);
    moveToPoint(-33, 66, 1.57, true, 100, 2000);
    moveToPoint(-40, 69, 1.57, true, 90, 1500);
    claw.move_absolute(-300, 200);
    pros::delay(500);
    clawLift.move_absolute(-3000, 200);
    //moveToPoint(-40, 66, 0.82, true, 100, 3000);
    moveToPoint(-8, 87, 1, true, 127, 3000);
    frontGoalLift.move_absolute(-3000, 200);
}
void rightQuali()
{
    
}
void rightElim()
{
    frontGoalLift.move_relative(-3200, 200);
    moveToPoint(0, 38, 0, true, 80, 2000);
    frontGoalLift.move_relative(1000, 200);
    pros::delay(800);
    moveToPoint(0, 18, 0, true, 110, 2000);
    moveToPoint(0, 18, 2, true, 110, 2000);
    frontGoalLift.move_relative(-1000, 200);
    pros::delay(750);
    intake.move(127);
    moveToPoint(-28, 23.5, 3.14, true, 110, 2000);
    moveToPoint(-28, 23.5, 6.28, true, 110, 2000);
    moveToPoint(-30.5, 38, 6.28, true, 110, 2000);
    frontGoalLift.move_relative(1000, 200);
    pros::delay(750);
    moveToPoint(-30.5, 18, 6.28, true, 110, 2000);
    moveToPoint(-30.5, 18, 3.14, true, 110, 2000);
}
void leftQuali()
{
    frontGoalLift.move_absolute(-3700, 200);
    pros::delay(1200);
    intake.move(127);
    moveToPoint(0, -4.8, 0, true, 90, 5000);
    moveToPoint(-18.5, 3, 1.57, true, 80, 5000);
    clawLift.move_absolute(-1200, 200);
    claw.move_absolute(1300, 200);
    moveToPoint(-42, 5, 1.57, false, 90, 5000);
    claw.move_absolute(-300, 200);
    pros::delay(700);
    clawLift.move_absolute(-1800, 200);
    moveToPoint(-30, 5, 4, false, 90, 5000);
     clawLift.move_absolute(-1200, 200);
    claw.move_absolute(1300, 200);
    pros::delay(400);
    moveToPoint(-36, 2, 3.14, false, 90, 5000);
    clawLift.move_absolute(-1200, 200);
    claw.move_absolute(1300, 200);
    moveToPoint(-32.8, 36.5, 1.57, true, 100, 3000);
    //get mid
    moveToPoint(-42, 35.5, 1.57, true, 100, 3000);
    claw.move_absolute(-300, 200);
    pros::delay(700);
    clawLift.move_absolute(-1800, 200);
    moveToPoint(-8, 42, 3.14, true, 127, 2000);
    
    
   
}

void skills()
{
    frontGoalLift.move_absolute(-3700, 200);
    pros::delay(1500);
    moveToPoint(0, -4.8, 0, true, 80, 5000);
    moveToPoint(-18.5, 5, 1.57, true, 65, 5000);
    clawLift.move_absolute(-1200, 200);
    claw.move_absolute(1300, 200);
    moveToPoint(-42, 5, 1.57, false, 40, 5000);
    claw.move_absolute(-100, 200);
    pros::delay(700);
    clawLift.move_absolute(-4000, 200);
    pros::delay(1500);
    moveToPoint(-27, 3.6, 1.57, true, 60, 5000);
    moveToPoint(-28, 37, 1.57, true, 70, 5000);
    moveToPoint(-30, 37, 4.57, true, 60, 5000);
    moveToPoint(-19, 37, 4.57, true, 60, 2500);

    clawLift.move_absolute(-3200, 200);
    pros::delay(500);
    claw.move_absolute(1300, 200);
    pros::delay(1000);

    moveToPoint(-36, 34, 3.14, true, 80, 5000);
    //moveToPoint(-56, 31, 3.14, true, 70, 5000);
    moveToPoint(-75, 33, 3.14, true, 80, 7000);
    moveToPoint(-25, 33, 3.14, true, 80, 7000);
/*
    clawLift.move_absolute(-1200, 200);
    claw.move_absolute(1300, 200);
    //go get last neutral
    moveToPoint(-50.5, 55, 3.14, true, 70, 5000);
    moveToPoint(-50.5, 59, 3.14, false, 65, 5000);

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

    //moveToPoint(-24, 67.6, 3.14, true, 70, 5000);
    moveToPoint(-24, 65, 3.14, true, 70, 5000);
    moveToPoint(-28, 70, 2.5, true, 70, 5000);
    moveToPoint(-24, 65, 3.14, true, 70, 5000);

    clawLift.move_absolute(-1200, 200);
    claw.move_absolute(1300, 200);

    moveToPoint(-24, 77.7, 3.14, false, 65, 5000);

     claw.move_absolute(-100, 200);

    pros::delay(700);
    moveToPoint(-24, 73, 3.14, false, 65, 5000);
    clawLift.move_absolute(-4000, 200);

    pros::delay(1500);
    moveToPoint(-24, 73, 4.57, false, 70, 2000);
    moveToPoint(-90, 77, 4.57, false, 90);
    */
}
void leftElim()
{
   frontGoalLift.move_relative(-3200, 200);
   moveToPoint(0, 12, 0, true, 80, 2000);
    moveToPoint(6, 38, 0, true, 70, 2000);
    frontGoalLift.move_relative(1000, 200);
    pros::delay(800);
    moveToPoint(6, 18, 0, true, 110, 2000);
    moveToPoint(6, 18, 2, true, 110, 2000);
    frontGoalLift.move_relative(-1000, 200);
    pros::delay(750);
    intake.move(127);
    moveToPoint(34, 23.5, 3.14, true, 110, 2000);
    moveToPoint(34, 23.5, 6.28, true, 110, 2000);
    moveToPoint(36.5, 37, 6.28, true, 110, 2000);
    frontGoalLift.move_relative(1000, 200);
    pros::delay(750);
    moveToPoint(36.5, 18, 6.28, true, 110, 2000);
    moveToPoint(36.5, 18, 3.14, true, 110, 2000);





}
//actually running the auton
void runAuton()
{
    runningAuton = true;
    init();
    rightElim();

    runningAuton = false;
}
