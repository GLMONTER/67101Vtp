#include"main.h"

auto chassis = ChassisControllerBuilder()
    .withMotors(1, -2) // left motor is 1, right motor is 2 (reversed)
    .withGains(
        {0.001, 0, 0.0001}, // distance controller gains
        {0.001, 0, 0.0001}, // turn controller gains
        {0.001, 0, 0.0001}  // angle controller gains (helps drive straight)
    )
    .withSensors(
        ADIEncoder{'A', 'B'}, // left encoder in ADI ports A & B
        ADIEncoder{'C', 'D', true},  // right encoder in ADI ports C & D (reversed)
        ADIEncoder{'E', 'F'}  // middle encoder in ADI ports E & F
    )
    // green gearset, tracking wheel diameter (2.75 in), track (7 in), and TPR (360)
    // 1 inch middle encoder distance, and 2.75 inch middle wheel diameter
    .withDimensions(AbstractMotor::gearset::green, {{2.75_in, 7_in, 1_in, 2.75_in}, quadEncoderTPR})
    .withOdometry() // use the same scales as the chassis (above)
    .buildOdometry(); // build an odometry chassis

void init()
{
    
}

bool runningAuton = false;


void winPoint()
{
    
}

void skills()
{

}

//actually running the auton
void runAuton()
{
    runningAuton = true;
    init();
    //call auton function
    runningAuton = false;
}
