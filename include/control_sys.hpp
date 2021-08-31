#pragma once

#include"main.h"
void setDrive(int32_t leftPower, int32_t rightPower);
enum loaderSetting
{
    Disabled = 0,
    Intake = 1,
    Outake = 2
};
void setLoader(loaderSetting setting);
void controlLoader();
void moveCascade();
void setCascade();
void moveGoalLift();
void actuatePneumatics();
extern pros::Controller controller;

extern pros::Motor rightFront;
extern pros::Motor leftFront;
extern pros::Motor rightBack;
extern pros::Motor leftBack;

extern pros::Motor cascade;
extern pros::Motor intake;
extern pros::Motor goalLift;

