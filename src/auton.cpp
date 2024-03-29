#include"../include/main.h"

bool runningAuton = false;
extern bool clawOpened;
extern bool liftUp;
extern bool actuatePNflag;
//related to distance sensor
bool grabFlag = false;

#define Win true
//tracking wheel diameter in inches
#define WHEEL_DIAM 3.25

//calculate how far the wheel will travel in one rotation
float SPIN_TO_IN_LR  = (WHEEL_DIAM * PI / 36000);

//distance from the left tracking wheel to tracking center
#define L_DISTANCE_IN 5.25
//distance from the right tracking wheel to tracking center
#define R_DISTANCE_IN 4.15
//distance from the rear tracking wheel to tracking center
#define S_DISTANCE_IN 2

extern pros::ADIDigitalIn buttonLimit;

pros::Rotation leftEncoder(17);
pros::Rotation rightEncoder(4);

pros::Rotation middleEncoder(18);

pros::Distance frontDistance(15);


void distanceGrab()
{
    while(true)
    {
        pros::lcd::print(6, "%d", grabFlag);

        while(grabFlag)
        {
            if(frontDistance.get() < 200 && frontDistance.get() != 0)
            {
                clawOpened = false;
                grabFlag = false;
            }
            pros::delay(10);
            pros::lcd::print(5, "%f", frontDistance.get());
        }
        pros::delay(10);
    }
}


typedef struct _pos
{
	float a = 0;
	float y = 0;
	float x = 0;
	int leftLst = 0;
	int rightLst = 0;
	int backLst = 0;
} sPos; // Position of the robot

sPos gPosition;

void setDrive(int32_t left, int32_t right)
{
    rightFront.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rightBack.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    leftFront.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    leftBack.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    rightFront.move_velocity(right);
    rightBack.move_velocity(right);

    leftFront.move_velocity(left);
    leftBack.move_velocity(left);
}

void trackPosition()
{    
    int counter = 0;
    middleEncoder.reverse();
    //reset position because position is a set state
    leftEncoder.reset_position();
    rightEncoder.reset_position();
    middleEncoder.reset_position();

    //update encoders every 5 milliseconds
    leftEncoder.set_data_rate(5);
    rightEncoder.set_data_rate(5);
    middleEncoder.set_data_rate(5);

    while(true)
    {
        counter ++;

        //get encoder position based on a 360 tick rotation
        float left = leftEncoder.get_position();
        float right = rightEncoder.get_position();
        float back = middleEncoder.get_position();

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

        pros::lcd::print(0, "X : %f", gPosition.x);
        pros::lcd::print(1, "Y : %f", gPosition.y);
        pros::lcd::print(2, "R : %f", gPosition.a);

        //write to terminal
        if(counter > 500)
        {
            std::cout<<"X : "<<gPosition.x<<"   ";
            std::cout<<"Y : "<<gPosition.y<<"   ";
            std::cout<<"R : "<<gPosition.a<<"   "<<std::endl<<std::endl<<std::endl<<std::endl<<std::endl<<std::endl;

            counter = 0;
        }
        pros::delay(5);
    }
}
float getNewPID(const float error, bool resetFlag, const float Pgain = 0.0f, const float iGain = 0.0f, const float dGain = 0.0f)
{
    // static float error;
    static float integral;
    static float derivative;
    static float previousError;

    //pid gains
    float Ki;
    float Kd;
    float Kp;

    if(resetFlag)
    {
        integral = 0;
        derivative = 0;
        previousError = 0;
    }
    //if gains are default, use default gains
    if (Pgain == 0.0f && iGain == 0.0f && dGain == 0.0f)
    {
        #if Win
        Ki = 0.1;
        Kd = 0.11f;
        Kp = 1.5f;
        #else
        Ki = 0.14;
        Kd = 0.15f;
        Kp = 1.6f;
        #endif
    }
    //if not use custom gains
    else
    {
        Ki = iGain;
        Kd = dGain;
        Kp = Pgain;
    }
       
    integral = integral + error;
    
    //if the error is small delete integral to prevent overshoot
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

void moveToPoint(const float x, const float y, const float angle, bool goThroughFlag, const uint32_t maxVelocity = 127, uint32_t timeout = 0
, const float Pgain = 0.0f, const float iGain = 0.0f, const float dGain = 0.0f)
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

        //left front
		if(std::abs((sin(T + 0.25*PI) + differenceOfAngle * 4) * std::abs(scaledPID)) > maxVelocity)
        {
            if((sin(T + 0.25*PI) + differenceOfAngle * 4) * std::abs(scaledPID) > 0)
                leftFront.move(maxVelocity);
            else
                leftFront.move(-maxVelocity);
        }
        else
        {
            leftFront.move((sin(T + 0.25*PI) + differenceOfAngle * 4) * std::abs(scaledPID));
        }
        //right Back
        if(std::abs((sin(T + 0.25*PI) - differenceOfAngle * 4) * std::abs(scaledPID)) > maxVelocity)
        {
            if(((sin(T + 0.25*PI) - differenceOfAngle * 4) * std::abs(scaledPID)) > 0)
                rightBack.move(maxVelocity);
            else
                rightBack.move(-maxVelocity);
        }
        else
        {
            rightBack.move((sin(T + 0.25*PI) - differenceOfAngle * 4) * std::abs(scaledPID));
        }
        //right Front

        if(std::abs((sin(T - 0.25*PI) - differenceOfAngle * 4) * std::abs(scaledPID)) > maxVelocity)
        {
            if((sin(T - 0.25*PI) - differenceOfAngle * 4) * std::abs(scaledPID) > 0)
                rightFront.move(maxVelocity);
            else
                rightFront.move(-maxVelocity);
        }
        else
        {
            rightFront.move((sin(T - 0.25*PI) - differenceOfAngle * 4) * std::abs(scaledPID));
        }
        //left Back
        if(std::abs((sin(T - 0.25*PI) + differenceOfAngle * 4) * std::abs(scaledPID)) > maxVelocity)
        {
            if((sin(T - 0.25*PI) + differenceOfAngle * 4) * std::abs(scaledPID) > 0)
                leftBack.move(maxVelocity);
            else
                leftBack.move(-maxVelocity);
        }
        else
        {
            leftBack.move((sin(T - 0.25*PI) + differenceOfAngle * 4) * std::abs(scaledPID));
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
    intake.move(-127);
    pros::delay(250);
    intake.move(0);
    moveToPoint(0, 24, 0, true, 80, 3000);
    moveToPoint(0, 24, 1.57, false, 100, 3000);
    moveToPoint(75, 24, 1.57, true, 80, 4000);

    moveToPoint(94, 29, 2.68, true, 80, 4000);

    intake.move(127);
    pros::delay(2500);
    intake.move(0);
    setDrive(150, 150);
    pros::delay(4000);
    setDrive(0,0);
}
void rightElim()
{
    frontGoalLift.move_relative(-3200, 200);
    moveToPoint(0, 38, 0, true, 90, 2000);
    frontGoalLift.move_relative(1000, 200);
    pros::delay(800);
    moveToPoint(0, 18, 0, true, 110, 2000);
    moveToPoint(0, 18, 2, true, 110, 2000);
    frontGoalLift.move_relative(-1000, 200);
    pros::delay(750);
    intake.move(127);
    moveToPoint(-28, 23.5, 3.14, true, 95, 2000);
    moveToPoint(-28, 23.5, 6.28, true, 95, 2000);
    moveToPoint(-28, 38, 6.28, true, 95, 2000);
    frontGoalLift.move_relative(1000, 200);
    pros::delay(750);
    moveToPoint(-28, 20, 6.28, true, 95, 2000);
    moveToPoint(-28, 20, 4.71, true, 95, 2000);
}
void rightQuali()
{

    frontGoalLift.move_relative(-3200, 200);

    while(frontDistance.get() < 1350)
    {
        setDrive(200, 200);
    }

    //lock drive
    leftFront.move_velocity(0);
    leftBack.move_velocity(0);
    rightFront.move_velocity(0);
    rightBack.move_velocity(0);

    frontGoalLift.move_relative(1000, 200);
    pros::delay(400);
    moveToPoint(0, 23, 0, true, 100);
    moveToPoint(0, 23, -1.57, true, 127, 1300);
    frontGoalLift.move_relative(-1000, 200);
    moveToPoint(22, 38, -1.57, true, 100, 2000);
    moveToPoint(22, 38, 1.57, true, 100, 2000);
    moveToPoint(22, 25, 1.57, true, 127, 2000);
    intake.move(-127);
    pros::delay(250);
    intake.move(0);
    moveToPoint(22, 10, 1.57, true, 127, 2000);
        moveToPoint(22, 10, 3.14, true, 127, 2000);
    frontGoalLift.move_relative(3200, 200);



}
void leftQuali()
{
    intake.move(-127);
    pros::delay(500);
    intake.move(0);
    frontGoalLift.move_relative(-3200, 200);

    while(frontDistance.get() < 1350)
    {
        setDrive(200, 200);
    }

    //lock drive
    leftFront.move_velocity(0);
    leftBack.move_velocity(0);
    rightFront.move_velocity(0);
    rightBack.move_velocity(0);

    frontGoalLift.move_relative(1000, 200);
    pros::delay(400);
    moveToPoint(0,0,0, true);



}
void skillsPark()
{
    //lift up counter weight
    liftUp = true;
    while(!buttonLimit.get_value())
    {
        pros::delay(5);
    }
    intake.move(127);
    //drive away from platform
    moveToPoint(0, -4.8, 0, true, 90, 4000);
    //turn towards first goal
     clawLift.move_absolute(-1200, 200);
    moveToPoint(-18.5, 4, 1.57, true, 90, 4000);
    intake.move(0);

    moveToPoint(-48, 4, 1.57, false, 90, 4000);
    clawOpened = false;
    pros::delay(750);
    clawLift.move_absolute(-5000, 200);
    pros::delay(1500);
    moveToPoint(-29, 3.6, 1.57, true, 90, 4000);
    moveToPoint(-29, 38, 1.57, true, 100, 3000);
    moveToPoint(-30, 38, 4.57, true, 90, 2500);
    moveToPoint(-20, 34, 4.57, true, 80, 2500);

    clawLift.move_absolute(-3400, 200);
    pros::delay(500);
    clawOpened = true;
    pros::delay(500);
    clawLift.move_absolute(-4500, 200);

    moveToPoint(-25, 38, 4.57, true, 90, 2500);

    moveToPoint(-35, 39, 4.57, true, 127, 5000);
    clawLift.move_absolute(-1200, 200);
    //push against middle goal
    moveToPoint(-70, 39, 4.57, true, 105, 5000);

    clawLift.move_absolute(-1200, 200);
    clawOpened = true;
    //go get last neutral
    moveToPoint(-57.5, 60, 3.14, true, 90, 5000);
    moveToPoint(-57.5, 64, 3.14, false, 90, 5000);

    clawOpened = false;
    pros::delay(1200);
    clawLift.move_absolute(-5200, 200);
    pros::delay(1000);
    moveToPoint(-34, 44, 4.57, true, 100, 4000);
    moveToPoint(-21.5, 44, 4.57, true, 100, 3000);
    //lay last neut down
    clawLift.move_absolute(-3400, 200);
    pros::delay(500);
    clawOpened = true;
    pros::delay(250);
    clawLift.move_absolute(-4500, 200);



    moveToPoint(-24, 70, 3.14, true, 90, 5000);

    clawLift.move_absolute(-1000, 200);
    clawOpened = true;

    //move towards blue alliance
    moveToPoint(-24, 92, 3.14, false, 80, 4000);
    //clamp
    clawOpened = false;

    pros::delay(700);
    clawLift.move_absolute(-5600, 200);
    pros::delay(250);
    moveToPoint(-24, 75, 3.14, false, 80, 4000);

    pros::delay(1600);
    //turn towards platform
    moveToPoint(-25.5, 75, 0.8, true, 80, 1200);
    //go to platform
    moveToPoint(-126, -5.25, 0.8, true, 100, 5000);
    moveToPoint(-126, -5.25, 3.14, true, 100, 2000);


    clawLift.move_absolute(-6500, 200);
    pros::delay(2000);

    setDrive(-50, -50);
    pros::delay(1500);
    setDrive(50, 50);
    pros::delay(800);
    clawLift.move_absolute(-1400, 200);
    pros::delay(1000);
    setDrive(-100, -100);
    pros::delay(750);
    clawLift.move_absolute(-2200, 200);

    while(gyro.get_pitch() > 19)
    {
        setDrive(-150, -150);
        pros::delay(10);
    }
    pros::delay(400);
    while(true)
    {
        if(gyro.get_pitch() < 45 && gyro.get_pitch() > 32)
        {
            setDrive(0,0);
            actuatePNflag = false;
            continue;
        }
        if(gyro.get_pitch() < 32)
        {
            setDrive(-150,-150);
            actuatePNflag = true;

        }
        if(gyro.get_pitch() > 42)
        {
            setDrive(21,21);
            actuatePNflag = false;
        }
        pros::delay(10);
    }
}
void skills()
{
   //lift up counter weight
    liftUp = true;
    while(!buttonLimit.get_value())
    {
        pros::delay(5);
    }
    intake.move(127);
    //drive away from platform
    moveToPoint(0, -4.8, 0, true, 90, 4000);
    //turn towards first goal
     clawLift.move_absolute(-1200, 200);
    moveToPoint(-18.5, 4, 1.57, true, 90, 4000);
    intake.move(0);

    moveToPoint(-48, 4, 1.57, false, 90, 4000);
    clawOpened = false;
    pros::delay(750);
    clawLift.move_absolute(-5000, 200);
    pros::delay(1500);
    moveToPoint(-29, 3.6, 1.57, true, 90, 4000);
    moveToPoint(-29, 38, 1.57, true, 100, 3000);
    moveToPoint(-30, 38, 4.57, true, 90, 2500);
    moveToPoint(-20, 34, 4.57, true, 80, 2500);

    clawLift.move_absolute(-3400, 200);
    pros::delay(500);
    clawOpened = true;
    pros::delay(500);
    clawLift.move_absolute(-4500, 200);

    moveToPoint(-25, 38, 4.57, true, 90, 2500);

    moveToPoint(-35, 39, 4.57, true, 127, 5000);
    clawLift.move_absolute(-1200, 200);
    //push against middle goal
    moveToPoint(-70, 39, 4.57, true, 105, 5000);

    clawLift.move_absolute(-1200, 200);
    clawOpened = true;
    //go get last neutral
    moveToPoint(-57.5, 60, 3.14, true, 90, 5000);
    moveToPoint(-57.5, 64, 3.14, false, 90, 5000);

    clawOpened = false;
    pros::delay(1200);
    clawLift.move_absolute(-5200, 200);
    pros::delay(1000);
    moveToPoint(-34, 44, 4.57, true, 100, 4000);
    moveToPoint(-21.5, 44, 4.57, true, 100, 3000);
    //lay last neut down
    clawLift.move_absolute(-3400, 200);
    pros::delay(500);
    clawOpened = true;
    pros::delay(250);
    clawLift.move_absolute(-4500, 200);



    moveToPoint(-24, 70, 3.14, true, 90, 5000);

    clawLift.move_absolute(-1000, 200);
    clawOpened = true;

    //move towards blue alliance
    moveToPoint(-24, 92, 3.14, false, 80, 4000);
    //clamp
    clawOpened = false;

    pros::delay(700);
    moveToPoint(-25.5, 75, 3.14, false, 80, 4000);
    clawLift.move_absolute(-5400, 200);
    pros::delay(1250);
    //turn towards platform
    moveToPoint(-25, 75, 1.08, true, 80, 1200);
    //go to platform

    moveToPoint(-107, 29, 1.57, true, 90, 7000, 0.2, 0.2, 1.0);
    clawLift.move_absolute(-3000, 200);
    pros::delay(1000);
    clawOpened = true;
    pros::delay(500);
    clawLift.move_absolute(-5500, 200);
    frontGoalLift.move_absolute(0, 200);
    pros::delay(1000);
    clawOpened = false;
    moveToPoint(-101, 27, 0.0, true, 90, 3000, 0.2, 0.2, 1.0); 
}
void leftElim()
{
   frontGoalLift.move_relative(-3200, 200);
   moveToPoint(0, 12, 0, true, 100, 2000);
    moveToPoint(6, 38, 0, true, 90, 2000);
    frontGoalLift.move_relative(1000, 200);
    pros::delay(800);
    moveToPoint(6, 18, 0, true, 100, 2000);
    moveToPoint(6, 18, 2, true, 100, 2000);
    frontGoalLift.move_relative(-1000, 200);
    pros::delay(750);
   // intake.move(127);
    moveToPoint(34, 23.5, 3.14, true, 100, 2000);
    moveToPoint(34, 23.5, 6.28, true, 100, 2000);
    moveToPoint(38, 37, 6.28, true, 100, 2000);
    frontGoalLift.move_relative(1000, 200);
    pros::delay(750);
    moveToPoint(38, 18, 6.28, true, 100, 2000);
    moveToPoint(38, 18, 4.71, true, 100, 2000);
}
void fastElim()
{
    grabFlag = true;
    clawLift.move_absolute(-1200, 200);
    claw.move_absolute(-400, 200);

    setDrive(-200, -200);

    while(grabFlag)
    {
        if(frontDistance.get() < 300 && frontDistance.get() != 0)
        {
            clawOpened = false;
            grabFlag = false;

        }
        pros::delay(10);
        pros::lcd::print(5, "%f", frontDistance.get());
    }

        clawLift.move_absolute(-1400, 200);

    moveToPoint(0,0,0, false);
    setDrive(0,0);
}
void fastElimNew()
{
    grabFlag = true;
    clawLift.move_absolute(-1200, 200);
    claw.move_absolute(-400, 200);

    setDrive(-200, -200);

    while(grabFlag)
    {
        if(frontDistance.get() < 300 && frontDistance.get() != 0)
        {
            clawOpened = false;
            grabFlag = false;

        }
        pros::delay(10);
        pros::lcd::print(5, "%f", frontDistance.get());
    }
    pros::delay(200);
        clawLift.move_absolute(-1400, 200);

    moveToPoint(0,-56,1.57, false);
  //  setDrive(0,0);
}
//actually running the auton
void runAuton()
{
    runningAuton = true;
    init();

    leftQuali();


    runningAuton = false;
}
