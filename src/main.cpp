#include "../include/main.h"
//for std::setpercision
#include <iomanip>
//#define TESTING

extern void trackPosition();
extern void threadMacro();

extern pros::Rotation leftEncoder;
extern pros::Rotation middleEncoder;
extern pros::Rotation rightEncoder;

typedef struct _pos
{
	float a = 0;
	float y = 0;
	float x = 0;
	int leftLst = 0;
	int rightLst = 0;
	int backLst = 0;
} sPos; // Position of the robot

//for screen write
extern sPos gPosition;


extern bool runningAuton;

//a structure that is filled with string stream
struct outputPos
{
    std::string x, y, a;
};
outputPos writerPos;
#ifndef TESTING
//task to draw the ui on screen
void drawUI()
{
    extern const lv_img_dsc_t smallRedBox;
    extern const lv_img_dsc_t bigRedBox;
    extern const lv_img_dsc_t bigGreenBox;
    extern const lv_img_dsc_t smallGreenBox;

    extern const lv_img_dsc_t driveRender;
    extern const lv_img_dsc_t vaquitaImage;

    //create the left box
    lv_obj_t * leftBox;
    leftBox = lv_obj_create(lv_scr_act(), NULL);
    lv_obj_align(leftBox, nullptr, LV_ALIGN_IN_TOP_LEFT, 0, 0);    
    lv_obj_set_style(leftBox, &lv_style_btn_tgl_rel);                   
    lv_obj_set_height(leftBox, 272);
    lv_obj_set_width(leftBox, 240);

    //left tracking label objects
    lv_obj_t * trackingDataLabel;
    lv_obj_t * XYRLabel;
    lv_obj_t * trackingInfoLabel;

    //vaquita image :).
    lv_obj_t * img = lv_img_create(leftBox, NULL);
    lv_img_set_src(img, &vaquitaImage);
    lv_obj_set_pos(img, 0, 0);
    lv_obj_set_drag(img, true);
    lv_obj_align(img, NULL, LV_ALIGN_IN_TOP_MID, 0, 145);

    //using a slider as a seperator, kinda jank
    lv_obj_t * slider = lv_slider_create(leftBox, NULL);      
    lv_obj_set_size(slider, 300, 20);   
    lv_obj_align(slider, NULL, LV_ALIGN_IN_TOP_MID, 0, 120); 
    lv_slider_set_knob_in(slider, false);               
    lv_slider_set_value(slider, 100);  

    //write all of the tracking data to the left box
    trackingDataLabel = lv_label_create(leftBox, NULL);
    lv_label_set_text(trackingDataLabel, "Tracking Data");
    lv_obj_align(trackingDataLabel, NULL, LV_ALIGN_IN_TOP_MID, 0, 10);

    XYRLabel = lv_label_create(leftBox, NULL);
    lv_label_set_text(XYRLabel, " X        Y        R  ");
    lv_obj_align(XYRLabel, NULL, LV_ALIGN_IN_TOP_MID, 0, 45);

    //write all of the tracking data to the left box
    trackingInfoLabel = lv_label_create(leftBox, NULL);
    lv_label_set_text(trackingInfoLabel, "Tracking Data");
    lv_obj_align(trackingInfoLabel, NULL, LV_ALIGN_IN_TOP_MID, 0, 10);

    //create the right box and the label for reuse
    lv_obj_t * rightBox;
    rightBox = lv_obj_create(lv_scr_act(), NULL);
    lv_obj_align(rightBox, leftBox, LV_ALIGN_OUT_RIGHT_TOP, 0, 0);    
    lv_obj_set_style(rightBox, &lv_style_btn_tgl_rel);                  
    lv_obj_set_height(rightBox, 272);
    lv_obj_set_width(rightBox, 240);

    //drivetrain image :).
    lv_obj_t * driveImage = lv_img_create(rightBox, NULL);
    lv_img_set_src(driveImage, &driveRender);
    lv_obj_set_pos(driveImage, 0, 0);
    lv_obj_set_drag(driveImage, true);
    lv_obj_align(driveImage, NULL, LV_ALIGN_IN_TOP_MID, 0, 35);

    //battery percentage label
    lv_obj_t * batteryLabel;
    batteryLabel = lv_label_create(rightBox, NULL);

    //subsystem label objects
    lv_obj_t* clawLabel;
    lv_obj_t* clawLiftLabel;
    lv_obj_t* intakeLabel;
    lv_obj_t* goalLiftLabel;

    //create subsystem labels
    clawLabel = lv_label_create(rightBox, NULL);
    clawLiftLabel = lv_label_create(rightBox, NULL);
    intakeLabel = lv_label_create(rightBox, NULL);
    goalLiftLabel = lv_label_create(rightBox, NULL);

    //draw all of the motor text
    lv_label_set_text(clawLabel, "     Claw:");
    lv_obj_align(clawLabel, NULL, LV_ALIGN_IN_TOP_MID, -60, 195);
    
    lv_label_set_text(clawLiftLabel, "Claw Lift:");
    lv_obj_align(clawLiftLabel, NULL, LV_ALIGN_IN_TOP_MID, -60, 220);
    
    lv_label_set_text(intakeLabel, "   Intake:");
    lv_obj_align(intakeLabel, NULL, LV_ALIGN_IN_TOP_MID, 50, 195);

    lv_label_set_text(goalLiftLabel, "Goal Lift:");
    lv_obj_align(goalLiftLabel, NULL, LV_ALIGN_IN_TOP_MID, 50, 220);

    //drivetrain boxes
    lv_obj_t* leftFrontRedBox;
    lv_obj_t* rightFrontRedBox;
    lv_obj_t* leftBackRedBox;
    lv_obj_t* rightBackRedBox;

    lv_obj_t* leftFrontGreenBox;
    lv_obj_t* rightFrontGreenBox;
    lv_obj_t* leftBackGreenBox;
    lv_obj_t* rightBackGreenBox;

    //subsystem boxes
    lv_obj_t* clawGreenBox;
    lv_obj_t* clawLiftGreenBox;
    lv_obj_t* goalLiftGreenBox;
    lv_obj_t* intakeGreenBox;

    lv_obj_t* clawRedBox;
    lv_obj_t* clawLiftRedBox;
    lv_obj_t* goalLiftRedBox;
    lv_obj_t* intakeRedBox;

    //drivetrain

    //reds
    leftFrontRedBox = lv_img_create(rightBox, NULL);
    lv_img_set_src(leftFrontRedBox, &bigRedBox);

    rightFrontRedBox = lv_img_create(rightBox, NULL);
    lv_img_set_src(rightFrontRedBox, &bigRedBox);

    leftBackRedBox = lv_img_create(rightBox, NULL);
    lv_img_set_src(leftBackRedBox, &bigRedBox);

    rightBackRedBox = lv_img_create(rightBox, NULL);
    lv_img_set_src(rightBackRedBox, &bigRedBox);

    //greens
    leftFrontGreenBox = lv_img_create(rightBox, NULL);
    lv_img_set_src(leftFrontGreenBox, &bigGreenBox);

    rightFrontGreenBox = lv_img_create(rightBox, NULL);
    lv_img_set_src(rightFrontGreenBox, &bigGreenBox);

    leftBackGreenBox = lv_img_create(rightBox, NULL);
    lv_img_set_src(leftBackGreenBox, &bigGreenBox);

    rightBackGreenBox = lv_img_create(rightBox, NULL);
    lv_img_set_src(rightBackGreenBox, &bigGreenBox);

    //subsystems

    //greens
    clawGreenBox = lv_img_create(rightBox, NULL);
    lv_img_set_src(clawGreenBox, &smallGreenBox);

    clawLiftGreenBox = lv_img_create(rightBox, NULL);
    lv_img_set_src(clawLiftGreenBox, &smallGreenBox);

    goalLiftGreenBox = lv_img_create(rightBox, NULL);
    lv_img_set_src(goalLiftGreenBox, &smallGreenBox);

    intakeGreenBox = lv_img_create(rightBox, NULL);
    lv_img_set_src(intakeGreenBox, &smallGreenBox);

    //reds
    clawRedBox = lv_img_create(rightBox, NULL);
    lv_img_set_src(clawRedBox, &smallRedBox);

    clawLiftRedBox = lv_img_create(rightBox, NULL);
    lv_img_set_src(clawLiftRedBox, &smallRedBox);

    goalLiftRedBox = lv_img_create(rightBox, NULL);
    lv_img_set_src(goalLiftRedBox, &smallRedBox);

    intakeRedBox = lv_img_create(rightBox, NULL);
    lv_img_set_src(intakeRedBox, &smallRedBox);

    //so they dont draw before they are needed
    lv_obj_set_hidden(intakeRedBox, true);
    lv_obj_set_hidden(intakeGreenBox, true);
    lv_obj_set_hidden(goalLiftGreenBox, true);
    lv_obj_set_hidden(goalLiftRedBox, true);
    lv_obj_set_hidden(clawGreenBox, true);
    lv_obj_set_hidden(clawRedBox, true);
    lv_obj_set_hidden(clawLiftGreenBox, true);
    lv_obj_set_hidden(clawLiftRedBox, true);
    lv_obj_set_hidden(leftBackGreenBox, true);
    lv_obj_set_hidden(leftBackRedBox, true);
    lv_obj_set_hidden(leftFrontGreenBox, true);
    lv_obj_set_hidden(leftFrontRedBox, true);
    lv_obj_set_hidden(rightBackRedBox, true);
    lv_obj_set_hidden(rightBackGreenBox, true);
    lv_obj_set_hidden(rightFrontGreenBox, true);
    lv_obj_set_hidden(rightFrontRedBox, true);

    while(true)
    {
        /* This string stream code fills a struct with the tracking data and then truncates it to two decimal places*/
         // Create an output string stream
        std::ostringstream streamOBJ;
        // Set Fixed -Point Notation
        streamOBJ << std::fixed;
        // Set precision to 2 digits
        streamOBJ << std::setprecision(2);
        //Add double to stream
        streamOBJ << gPosition.x;
        // Get string from output string stream
        writerPos.x = streamOBJ.str();
        streamOBJ.str("");
        //Add double to stream
        streamOBJ << gPosition.y;
        // Get string from output string stream
        writerPos.y = streamOBJ.str();
        streamOBJ.str("");
        //Add double to stream
        streamOBJ << gPosition.a;
        // Get string from output string stream
        writerPos.a = streamOBJ.str();
        //clear the string stream
        streamOBJ.str("");
       
        std::string values =  writerPos.x + " in" + "  " + writerPos.y + " in" + "  "  + writerPos.a + " rad";
        lv_label_set_text(trackingInfoLabel, values.c_str());
        lv_obj_align(trackingInfoLabel, NULL, LV_ALIGN_IN_TOP_MID, 0, 75);

        //print the battery percentage to the brain
        lv_obj_align(batteryLabel, NULL, LV_ALIGN_IN_TOP_MID, 0, 10);
        lv_label_set_text(batteryLabel, std::string("Battery Percentage: " + std::to_string((int)pros::battery::get_capacity()) + "%").c_str());

        //left front drive status box
        if(leftFront.get_temperature() >= 55)
        {
            lv_obj_set_hidden(leftFrontGreenBox, true);
            lv_obj_set_hidden(leftFrontRedBox, false);
            lv_obj_align(leftFrontRedBox, NULL, LV_ALIGN_IN_TOP_MID, -55, 25);
        }
        else if(leftFront.get_temperature() < 55)
        {
            lv_obj_set_hidden(leftFrontRedBox, true);
            lv_obj_set_hidden(leftFrontGreenBox, false);
            lv_obj_align(leftFrontGreenBox, NULL, LV_ALIGN_IN_TOP_MID, -55, 25);
        }
        //right front drive status box
        if(rightFront.get_temperature() >= 55)
        {
            lv_obj_set_hidden(rightFrontGreenBox, true);
            lv_obj_set_hidden(rightFrontRedBox, false);
            lv_obj_align(rightFrontRedBox, NULL, LV_ALIGN_IN_TOP_MID, 55, 25);
        }
        else if(rightFront.get_temperature() < 55)
        {
            lv_obj_set_hidden(rightFrontRedBox, true);
            lv_obj_set_hidden(rightFrontGreenBox, false);
            lv_obj_align(rightFrontGreenBox, NULL, LV_ALIGN_IN_TOP_MID, 55, 25);
        }
        //right back drive status box
        if(rightBack.get_temperature() >= 55)
        {
            lv_obj_set_hidden(rightBackGreenBox, true);
            lv_obj_set_hidden(rightBackRedBox, false);
            lv_obj_align(rightBackRedBox, NULL, LV_ALIGN_IN_TOP_MID, 55, 120);
        }
        else if(rightBack.get_temperature() < 55)
        {
            lv_obj_set_hidden(leftBackRedBox, true);
            lv_obj_set_hidden(rightBackGreenBox, false);
            lv_obj_align(rightBackGreenBox, NULL, LV_ALIGN_IN_TOP_MID, 55, 120);
        }
        //left back drive status box
        if(leftBack.get_temperature() >= 55)
        {
            lv_obj_set_hidden(leftBackGreenBox, true);
            lv_obj_set_hidden(leftBackRedBox, false);
            lv_obj_align(leftBackRedBox, NULL, LV_ALIGN_IN_TOP_MID, -55, 120);
        }
        else if(leftBack.get_temperature() < 55)
        {
            lv_obj_set_hidden(leftFrontRedBox, true);
            lv_obj_set_hidden(leftBackGreenBox, false);
            lv_obj_align(leftBackGreenBox, NULL, LV_ALIGN_IN_TOP_MID, -55, 120);
        }
        //claw status box
        if(claw.get_temperature() >= 55)
        {
            lv_obj_set_hidden(clawGreenBox, true);
            lv_obj_set_hidden(clawRedBox, false);
            lv_obj_align(clawRedBox, NULL, LV_ALIGN_IN_TOP_MID, -5, 200);
        }
        else if(claw.get_temperature() <=55)
        {
            lv_obj_set_hidden(clawRedBox, true);
            lv_obj_set_hidden(clawGreenBox, false);
            lv_obj_align(clawGreenBox, NULL, LV_ALIGN_IN_TOP_MID, -5, 200);
        }
        //claw lift status box
        if(clawLift.get_temperature() >= 55)
        {
            lv_obj_set_hidden(clawLiftGreenBox, true);
            lv_obj_set_hidden(clawLiftRedBox, false);
            lv_obj_align(clawLiftRedBox, NULL, LV_ALIGN_IN_TOP_MID, -5, 225);
        }
        else if(clawLift.get_temperature() < 55)
        {
            lv_obj_set_hidden(clawLiftRedBox, true);
            lv_obj_set_hidden(clawLiftGreenBox, false);
            lv_obj_align(clawLiftGreenBox, NULL, LV_ALIGN_IN_TOP_MID, -5, 225);
        }
        //intake status box
        if(intake.get_temperature() >= 55)
        {
            lv_obj_set_hidden(intakeGreenBox, true);
            lv_obj_set_hidden(intakeRedBox, false);
            lv_obj_align(intakeRedBox, NULL, LV_ALIGN_IN_TOP_MID, 105, 200);
        }
        else if(intake.get_temperature() < 55)
        {
            lv_obj_set_hidden(intakeRedBox, true);
            lv_obj_set_hidden(intakeGreenBox, false);
            lv_obj_align(intakeGreenBox, NULL, LV_ALIGN_IN_TOP_MID, 105, 200);
        }
        //front goal lift status box
        if(frontGoalLift.get_temperature() >= 55)
        {
            lv_obj_set_hidden(goalLiftGreenBox, true);
            lv_obj_set_hidden(goalLiftRedBox, false);
            lv_obj_align(goalLiftRedBox, NULL, LV_ALIGN_IN_TOP_MID, 105, 225);
        }
        else if(frontGoalLift.get_temperature() < 55)
        {
            lv_obj_set_hidden(goalLiftRedBox, true);
            lv_obj_set_hidden(goalLiftGreenBox, false);
            lv_obj_align(goalLiftGreenBox, NULL, LV_ALIGN_IN_TOP_MID, 105, 225);
        }

        //delay to prevent the program from locking up
        pros::delay(100);
    }
}
#endif
pros::ADIDigitalOut aLight('A');
pros::ADIDigitalOut bLight('B');
pros::ADIDigitalOut cLight('C');
pros::ADIDigitalOut dLight('D');
pros::ADIDigitalOut eLight('E');
pros::ADIDigitalOut fLight('F');
pros::ADIDigitalOut gLight('G');

void christmas()
{
while(true)
{
//turn all off
aLight.set_value(HIGH);
bLight.set_value(HIGH);
cLight.set_value(HIGH);
dLight.set_value(HIGH);
eLight.set_value(HIGH);
fLight.set_value(HIGH);
gLight.set_value(HIGH);

//start sequence
aLight.set_value(LOW);

pros::delay(100);
bLight.set_value(LOW);
aLight.set_value(HIGH);

pros::delay(100);
cLight.set_value(LOW);
bLight.set_value(HIGH);

pros::delay(100);
dLight.set_value(LOW);
cLight.set_value(HIGH);

pros::delay(100);
eLight.set_value(LOW);
dLight.set_value(HIGH);

pros::delay(100);
fLight.set_value(LOW);
eLight.set_value(HIGH);

pros::delay(100);
gLight.set_value(LOW);
fLight.set_value(HIGH);

pros::delay(100);
}
}

void initialize()
{
	pros::Task trackingTask(trackPosition);
    pros::Task macroTask(threadMacro);
    #ifndef TESTING
    pros::Task draw(drawUI);
    #else
    pros::lcd::initialize();
    #endif

    pros::Task christmasLights(christmas);
}

void disabled() {}

void competition_initialize() {}

extern void runAuton();

void autonomous() 
{
    runAuton();
}

void opcontrol() 
{
    frontGoalLift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    clawLift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    intake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    runningAuton = false;

    while(true)
    {
        int Ch1 = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int Ch3 = -controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int Ch4 = -controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

        leftFront.move(Ch3 + Ch1 + Ch4);
        leftBack.move(Ch3 + Ch1 - Ch4);
        rightFront.move(Ch3 - Ch1 - Ch4);
        rightBack.move(Ch3 - Ch1 + Ch4);
        
        //slave controller code to control intake
        if(slaveController.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
            intake.move(127);
        else if(slaveController.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
            intake.move(-127);
        else
            intake.move(0);
        
        moveGoalLift();
        pros::delay(10);
    }
}

