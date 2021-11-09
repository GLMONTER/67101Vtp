#include"main.h"

bool runningAuton = false;

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


struct PathPoint {
    float x;
    float y;
    float w;
  	float curvature;
    float distance;
};

sPos gPosition;


pros::Rotation leftEncoder(1);
pros::Rotation rightEncoder(2);
pros::Rotation middleEncoder(3);

pros::IMU gyro(5);

void trackPosition()
{    
    while(true)
    {
    int32_t left = leftEncoder.get_position();
    int32_t right = rightEncoder.get_position();
    int32_t back = middleEncoder.get_position();

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

    pros::lcd::print(2, "left :  %d\n", leftEncoder.get_position());
    pros::lcd::print(3, "right :  %d\n", rightEncoder.get_position());
    pros::lcd::print(4, "middle :  %d\n", middleEncoder.get_position());
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
void gyroDrive(float targetPitch)
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

    float target = targetPitch;
    float Ki = -0.15;
    float Kd = -0.75;
    float Kp = -3.5;

    while (abs(error) > 0.1 || leftFront.get_actual_velocity() > 0.1)
    {
        pros::lcd::print(0, "val: %f\n", gyro.get_pitch());
        error =  target - gyro.get_pitch();
       // printf("%f \n", error);
        integral = integral + error;
        if (abs(error) < 2)
        {
            integral = 0.0;
        }
        derivative = error - perror;
        perror = error;
        value = (integral*Ki) + (derivative*Kd) + (error*Kp);

        leftBack.move(value);
        rightBack.move(value);
        leftFront.move(value);
        rightFront.move(value);

        pros::delay(10);
    }
}

void gyroDriveTop(float targetPitch, int toLock)
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

    float target = targetPitch;
    float Ki = -0.0125;
    float Kd = -1;
    float Kp = -0.5;

    while (abs(error) > 0.1 || leftFront.get_actual_velocity() > 0.1)
    {
        pros::lcd::print(0, "val: %f\n", gyro.get_pitch());
        error =  target - gyro.get_pitch();
      //  printf("%f \n", error);
        integral = integral + error;
        if (abs(error) < 20)
        {
            integral = 0.0;
        }
        derivative = error - perror;
        perror = error;
        value = (integral*Ki) + (derivative*Kd) + (error*Kp);
        if(std::abs(value) > 40)
        {
            leftBack.move_velocity(40);
            rightBack.move_velocity(40);
            leftFront.move_velocity(40);
            rightFront.move_velocity(40);
        }
        else
        {
            leftBack.move(-value);
            rightBack.move(-value);
            leftFront.move(-value);
            rightFront.move(-value);
        }
         
            
        pros::delay(10);
    }
    leftBack.move_velocity(0);
    rightBack.move_velocity(0);
    leftFront.move_velocity(0);
    rightFront.move_velocity(0);
}


void init()
{
    
}

//actually running the auton
void runAuton()
{
    runningAuton = true;
    init();
    runningAuton = false;
}
