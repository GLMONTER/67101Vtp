#include"main.h"

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

/* Defines */
//#define TAU (PI * 2)
#define NULL_STR (*(string *)0)

// Limit a variable to a value
#define LIM_TO_VAL(input, val) (abs(input) > (val) ? (val) * sgn(input) : (input))

// Limit a variable to a value and set that variable to the result
#define LIM_TO_VAL_SET(input, val) input = LIM_TO_VAL(input, val)

// The length of an array
#define ARR_LEN(array) (sizeof(array) / sizeof(array[0]))

// Swaps any integral type
#define SWAP(x, y) { x = x ^ y; y = x ^ y; x = x ^ y; }

// Checks the bit in a bitmap
#define CHK_BIT(bit, map) ((map & bit) == bit)

//#define MIN(x, y) ((x) < (y) ? (x) : (y))

//#define MAX(x, y) ((x) > (y) ? (x) : (y))

#define NORMAL_RAD(a) (fmod(a + PI, PI * 2) - PI)

#define REINTERPRET(var, type) (*(type *)&var)

#define STOP_TASK_NOT_CUR(t) if (t != nCurrentTask) stopTask(t)
/* Enumerations */
typedef enum _turnDir
{
	cw,
	ccw,
	ch
} tTurnDir;

/* Structures */
typedef struct _pos
{
	float a;
	float y;
	float x;
	int leftLst;
	int rightLst;
	int backLst;
} sPos; // Position of the robot

typedef struct _vel
{
	float a;
	float y;
	float x;
	unsigned long lstChecked;
	float lstPosA;
	float lstPosY;
	float lstPosX;
} sVel; // Velocity of the robot

typedef struct _vector
{
	float y;
	float x;
} sVector; // 2D cartesian vector

typedef struct _polar
{
	float magnitude;
	float angle;
} sPolar; // 2D polar vector

typedef struct _line
{
	sVector p1;
	sVector p2;
} sLine;
void init()
{
    
}

bool runningAuton = false;

typedef enum _stopType
{
	stopNone =		0b00000000,
	stopSoft =		0b00000001,
	stopHarsh =		0b00000010
} tStopType;


typedef enum _mttMode
{
	mttSimple,
	mttProportional,
	mttCascading
} tMttMode;

sVector gTargetLast;
sPos gPosition;
sVel gVelocity;

void winPoint()
{
    
}

void skills()
{

}
int sgn(double v) {
  if (v < 0) return -1;
  if (v > 0) return 1;
  return 0;
}
pros::ADIEncoder leftEncoder(3, 4, false);
pros::ADIEncoder rightEncoder(5, 6, false);
pros::ADIEncoder middleEncoder(1, 2, false);
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
void vectorToPolar(sVector& vector, sPolar& polar)
{
	if (vector.x || vector.y)
	{
		polar.magnitude = sqrt(vector.x * vector.x + vector.y * vector.y);
		polar.angle = atan2(vector.y, vector.x);
	}
	else
		polar.magnitude = polar.angle = 0;
}

void polarToVector(sPolar& polar, sVector& vector)
{
	if (polar.magnitude)
	{
		vector.x = polar.magnitude * cos(polar.angle);
		vector.y = polar.magnitude * sin(polar.angle);
	}
	else
		vector.x = vector.y = 0;
}

float getAngleOfLine(sLine line)
{
	return atan2(line.p2.x - line.p1.x, line.p2.y - line.p1.y);
}

float getLengthOfLine(sLine line)
{
	float x = line.p2.x - line.p1.x;
	float y = line.p2.y - line.p1.y;
	return sqrt(x * x + y * y);
}



float degToRad(float degrees)
{
	return degrees * PI / 180;
}
void applyHarshStop()
{
	sVector vel;
	vel.x = gVelocity.x;
	vel.y = gVelocity.y;
	sPolar polarVel;
	vectorToPolar(vel, polarVel);
	polarVel.angle += gPosition.a;
	polarToVector(polarVel, vel);
	float yPow = vel.y, aPow = gVelocity.a;

	//writeDebugStreamLine("Vel y | a: %f | %f", yPow, aPow);

	yPow *= -0.7;
	aPow *= -6.3;

	int left = yPow + aPow;
	int right = yPow - aPow;

	left = sgn(left) * MAX(fabs(left), 7);
	right = sgn(right) * MAX(fabs(right), 7);

	LIM_TO_VAL_SET(left, 30);
	LIM_TO_VAL_SET(right, 30);

	//writeDebugStreamLine("Applying harsh stop: %d %d", left, right);
	setDrive(left, right);
//	updateMotors();
	pros::Task::delay(150);
	setDrive(0, 0);
	//updateMotors();
}
float radToDeg(float radians)
{
	return radians * 180 / PI;
}

float nearAngle(float angle, float reference)
{
	return round((reference - angle) / (2 * PI)) * (2 * PI) + angle;
}
void moveToTargetSimple(float y, float x, float ys, float xs, int power, int startPower, float maxErrX, float decelEarly, int decelPower, float dropEarly, tStopType stopType, tMttMode mode, bool velSafety)
{
	int velSafetyCounter = 0;
	//if (LOGS) writeDebugStreamLine("Moving to %f %f from %f %f at %d", y, x, ys, xs, power);

	gTargetLast.y = y;
	gTargetLast.x = x;

	// Create the line to follow
	sLine followLine;

	// Start points
	followLine.p1.y = ys;
	followLine.p1.x = xs;

	// End points
	followLine.p2.y = y;
	followLine.p2.x = x;

	float lineLength = getLengthOfLine(followLine);
	//if (LOGS) writeDebugStreamLine("Line length: %.2f", lineLength);
	float lineAngle = getAngleOfLine(followLine); // Get the angle of the line that we're following relative to the vertical
	float pidAngle = nearAngle(lineAngle - (power < 0 ? PI : 0), gPosition.a);
	//if (LOGS) writeDebugStreamLine("Line | Pid angle: %f | %f", radToDeg(lineAngle), radToDeg(pidAngle));

	// Current position relative to the ending point
	sVector currentPosVector;
	sPolar currentPosPolar;
    
	//sCycleData cycle;
	//initCycle(cycle, 10, "moveToTarget");

	float vel;
	float _sin = sin(lineAngle);
	float _cos = cos(lineAngle);

	int last = startPower;
	float correction = 0;

	if (mode == mttSimple)
		setDrive(power, power);

	int finalPower = power;

//	unsigned long timeStart = nPgmTime;
	do
	{
		currentPosVector.x = gPosition.x - x;
		currentPosVector.y = gPosition.y - y;
		vectorToPolar(currentPosVector, currentPosPolar);
		currentPosPolar.angle += lineAngle;
		polarToVector(currentPosPolar, currentPosVector);
		
		if (maxErrX)
		{
			float errA = gPosition.a - pidAngle;
			float errX = currentPosVector.x + currentPosVector.y * sin(errA) / cos(errA);
			float correctA = atan2(x - gPosition.x, y - gPosition.y);
			if (power < 0)
				correctA += PI;
			correction = fabs(errX) > maxErrX ? 8.0 * (nearAngle(correctA, gPosition.a) - gPosition.a) * sgn(power) : 0;
		}

		if (mode != mttSimple)
		{
			switch (mode)
			{
			case mttProportional:
				finalPower = round(-127.0 / 40.0 * currentPosVector.y) * sgn(power);
				break;
			case mttCascading:
#if SKILLS_ROUTE == 0
				const float kB = 2.8;
				const float kP = 2.0;
#else
				float kB, kP;
				if (nPgmTime - gAutoTime > 40000)
				{
					kB = 5.0;
					kP = 3.2;
				}
				else
				{
					kB = 4.5;
					kP = 2.5;
				}
#endif
				float vTarget = 45 * (1 - exp(0.07 * (currentPosVector.y + dropEarly)));
				finalPower = round(kB * vTarget + kP * (vTarget - vel)) * sgn(power);
				break;
			}
			LIM_TO_VAL_SET(finalPower, abs(power));
			if (finalPower * sgn(power) < 30)
				finalPower = 30 * sgn(power);
			int delta = finalPower - last;
			LIM_TO_VAL_SET(delta, 5);
			finalPower = last += delta;
		}

		switch (sgn(correction))
		{
		case 0:
			setDrive(finalPower, finalPower);
			break;
		case 1:
			setDrive(finalPower, finalPower * exp(-correction));
			break;
		case -1:
			setDrive(finalPower * exp(correction), finalPower);
			break;
		}

		vel = _sin * gVelocity.x + _cos * gVelocity.y;

		//endCycle(cycle);
	} while (currentPosVector.y < -dropEarly - MAX((vel * ((stopType & stopSoft) ? 0.175 : 0.098)), decelEarly));// && (velSafety? NOT_SAFETY(power, moveToTargetSimple) : 1 ) );

//if (LOGS) writeDebugStreamLine("%f %f", currentPosVector.y, vel);

	setDrive(decelPower, decelPower);

	do
	{
		currentPosVector.x = gPosition.x - x;
		currentPosVector.y = gPosition.y - y;
		vectorToPolar(currentPosVector, currentPosPolar);
		currentPosPolar.angle += lineAngle;
		polarToVector(currentPosPolar, currentPosVector);

		vel = _sin * gVelocity.x + _cos * gVelocity.y;

		//endCycle(cycle);
	} while (currentPosVector.y < -dropEarly - (vel * ((stopType & stopSoft) ? 0.175 : 0.098)));

	if (stopType & stopSoft)
	{
		setDrive(-6 * sgn(power), -6 * sgn(power));
		do
		{
			currentPosVector.x = gPosition.x - x;
			currentPosVector.y = gPosition.y - y;
			vectorToPolar(currentPosVector, currentPosPolar);
			currentPosPolar.angle += lineAngle;
			polarToVector(currentPosPolar, currentPosVector);

			vel = _sin * gVelocity.x + _cos * gVelocity.y;

			//endCycle(cycle);
		} while (vel > 7 && currentPosVector.y < 0);
	}

	if (stopType & stopHarsh)
		applyHarshStop();
	else
		setDrive(0, 0);

//if (LOGS) writeDebugStreamLine("Moved to %f %f from %f %f | %f %f %f", y, x, ys, xs, gPosition.y, gPosition.x, radToDeg(gPosition.a));
}
//actually running the auton
void runAuton()
{
    runningAuton = true;
    init();
    //call auton function
    runningAuton = false;
}
