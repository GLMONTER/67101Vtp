#include"main.h"

extern void moveToPoint(const float x, const float y, const float angle, bool goThroughFlag, const uint32_t maxVelocity = 127, uint32_t timeout = 0);

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
