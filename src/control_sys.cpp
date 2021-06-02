#include"control_sys.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::Motor rightFront(6, pros::E_MOTOR_GEARSET_18, true);
pros::Motor leftFront(20, pros::E_MOTOR_GEARSET_18, false);
pros::Motor rightBack(18, pros::E_MOTOR_GEARSET_18, true);
pros::Motor leftBack(10, pros::E_MOTOR_GEARSET_18, false);


void setDrive(const int32_t leftPower, const int32_t rightPower)
{
    rightFront.move(rightPower);
    rightBack.move(rightPower);
    leftFront.move(leftPower);
    leftBack.move(leftPower);
}
