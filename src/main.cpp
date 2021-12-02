#include "main.h"
//for std::setpercision
#include <iomanip>

extern void trackPosition();
extern void threadMacro();

extern pros::Rotation leftEncoder;
extern pros::Rotation middleEncoder;
extern pros::Rotation rightEncoder;

int flag = true;

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

//task to draw the ui on screen
void drawUI()
{
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
    
        //create the left box
        lv_obj_t * leftBox;
        leftBox = lv_obj_create(lv_scr_act(), NULL);
        lv_obj_align(leftBox, nullptr, LV_ALIGN_IN_TOP_LEFT, 0, 0);    
        lv_obj_set_style(leftBox, &lv_style_btn_tgl_rel);                   
        lv_obj_set_height(leftBox, 272);
        lv_obj_set_width(leftBox, 240);
        lv_obj_t * label;
        
        //write all of the tracking data to the left box
        label = lv_label_create(leftBox, NULL);
        lv_label_set_text(label, "Tracking Data");
        lv_obj_align(label, NULL, LV_ALIGN_IN_TOP_MID, 0, 15);

        label = lv_label_create(leftBox, NULL);
        lv_label_set_text(label, " X       Y       R  ");
        lv_obj_align(label, NULL, LV_ALIGN_IN_TOP_MID, 0, 45);

        label = lv_label_create(leftBox, NULL);
        std::string values =  writerPos.x + "     " + writerPos.y + "     "  + writerPos.a;
        lv_label_set_text(label, values.c_str());
        lv_obj_align(label, NULL, LV_ALIGN_IN_TOP_MID, 0, 75);

        //using a slider as a seperator, kinda jank
        lv_obj_t * slider = lv_slider_create(leftBox, NULL);      
        lv_obj_set_size(slider, 300, 20);   
        lv_obj_align(slider, NULL, LV_ALIGN_IN_TOP_MID, 0, 120); 
        lv_slider_set_knob_in(slider, false);               
        lv_slider_set_value(slider, 100);  

        //vaquita image :).
        extern const lv_img_dsc_t vaqr;
        lv_obj_t * img = lv_img_create(leftBox, NULL);
        lv_img_set_src(img, &vaqr);
        lv_obj_set_pos(img, 0, 0);
        lv_obj_set_drag(img, true);
        lv_obj_align(img, NULL, LV_ALIGN_IN_TOP_MID, 0, 145);
        
        //create the right box and the label for reuse
        lv_obj_t * rightBox;
        rightBox = lv_obj_create(lv_scr_act(), NULL);
        lv_obj_align(rightBox, leftBox, LV_ALIGN_OUT_RIGHT_TOP, 0, 0);    
        lv_obj_set_style(rightBox, &lv_style_btn_tgl_rel);                  
        lv_obj_set_height(rightBox, 272);
        lv_obj_set_width(rightBox, 240);
        lv_obj_t * label2;

        //print the battery percentage to the brain
        label2 = lv_label_create(rightBox, NULL);
        lv_label_set_text(label2, std::string("Battery Percentage: " + std::to_string((int)pros::battery::get_capacity()) + "%").c_str());
        lv_obj_align(label2, NULL, LV_ALIGN_IN_TOP_MID, 0, 10);

        //drivetrain image :).
        extern const lv_img_dsc_t render;
        lv_obj_t * im = lv_img_create(rightBox, NULL);
        lv_img_set_src(im, &render);
        lv_obj_set_pos(im, 0, 0);
        lv_obj_set_drag(im, true);
        lv_obj_align(im, NULL, LV_ALIGN_IN_TOP_MID, 0, 35);

        //left front drive status box
        if(leftFront.get_temperature() >= 55)
        {
            extern const lv_img_dsc_t sm;
            lv_obj_t * imger = lv_img_create(rightBox, NULL);
            lv_img_set_src(imger, &sm);
            lv_obj_set_pos(imger, 0, 0);
            lv_obj_set_drag(imger, true);
            lv_obj_align(imger, NULL, LV_ALIGN_IN_TOP_MID, -55, 25);
        }
        else
        {
            extern const lv_img_dsc_t png;
            lv_obj_t * imger = lv_img_create(rightBox, NULL);
            lv_img_set_src(imger, &png);
            lv_obj_set_pos(imger, 0, 0);
            lv_obj_set_drag(imger, true);
            lv_obj_align(imger, NULL, LV_ALIGN_IN_TOP_MID, -55, 25);
        }
        //right front drive status box
        if(rightFront.get_temperature() >= 55)
        {
            extern const lv_img_dsc_t sm;
            lv_obj_t * imger = lv_img_create(rightBox, NULL);
            lv_img_set_src(imger, &sm);
            lv_obj_set_pos(imger, 0, 0);
            lv_obj_set_drag(imger, true);
            lv_obj_align(imger, NULL, LV_ALIGN_IN_TOP_MID, 55, 25);
        }
        else
        {
            extern const lv_img_dsc_t png;
            lv_obj_t * imger = lv_img_create(rightBox, NULL);
            lv_img_set_src(imger, &png);
            lv_obj_set_pos(imger, 0, 0);
            lv_obj_set_drag(imger, true);
            lv_obj_align(imger, NULL, LV_ALIGN_IN_TOP_MID, 55, 25);
        }
        //right back drive status box
        if(rightBack.get_temperature() >= 55)
        {
            extern const lv_img_dsc_t sm;
            lv_obj_t * imger = lv_img_create(rightBox, NULL);
            lv_img_set_src(imger, &sm);
            lv_obj_set_pos(imger, 0, 0);
            lv_obj_set_drag(imger, true);
            lv_obj_align(imger, NULL, LV_ALIGN_IN_TOP_MID, 55, 120);
        }
        else
        {
            extern const lv_img_dsc_t png;
            lv_obj_t * imger = lv_img_create(rightBox, NULL);
            lv_img_set_src(imger, &png);
            lv_obj_set_pos(imger, 0, 0);
            lv_obj_set_drag(imger, true);
            lv_obj_align(imger, NULL, LV_ALIGN_IN_TOP_MID, 55, 120);
        }
        //left back drive status box
        if(leftBack.get_temperature() >= 55)
        {
            extern const lv_img_dsc_t sm;
            lv_obj_t * imger = lv_img_create(rightBox, NULL);
            lv_img_set_src(imger, &sm);
            lv_obj_set_pos(imger, 0, 0);
            lv_obj_set_drag(imger, true);
            lv_obj_align(imger, NULL, LV_ALIGN_IN_TOP_MID, -55, 120);
        }
        else
        {
            extern const lv_img_dsc_t png;
            lv_obj_t * imger = lv_img_create(rightBox, NULL);
            lv_img_set_src(imger, &png);
            lv_obj_set_pos(imger, 0, 0);
            lv_obj_set_drag(imger, true);
            lv_obj_align(imger, NULL, LV_ALIGN_IN_TOP_MID, -55, 120);
        }
        //claw status box
        if(claw.get_temperature() >= 55)
        {
            extern const lv_img_dsc_t smrev;
            lv_obj_t * imger = lv_img_create(rightBox, NULL);
            lv_img_set_src(imger, &smrev);
            lv_obj_set_pos(imger, 0, 0);
            lv_obj_set_drag(imger, true);
            lv_obj_align(imger, NULL, LV_ALIGN_IN_TOP_MID, -5, 200);
        }
        else
        {
            extern const lv_img_dsc_t pngrev;
            lv_obj_t * imger = lv_img_create(rightBox, NULL);
            lv_img_set_src(imger, &pngrev);
            lv_obj_set_pos(imger, 0, 0);
            lv_obj_set_drag(imger, true);
            lv_obj_align(imger, NULL, LV_ALIGN_IN_TOP_MID, -5, 200);
        }
        //claw lift status box
        if(clawLift.get_temperature() >= 55)
        {
            extern const lv_img_dsc_t smrev;
            lv_obj_t * imger = lv_img_create(rightBox, NULL);
            lv_img_set_src(imger, &smrev);
            lv_obj_set_pos(imger, 0, 0);
            lv_obj_set_drag(imger, true);
            lv_obj_align(imger, NULL, LV_ALIGN_IN_TOP_MID, -5, 225);
        }
        else
        {
            extern const lv_img_dsc_t pngrev;
            lv_obj_t * imger = lv_img_create(rightBox, NULL);
            lv_img_set_src(imger, &pngrev);
            lv_obj_set_pos(imger, 0, 0);
            lv_obj_set_drag(imger, true);
            lv_obj_align(imger, NULL, LV_ALIGN_IN_TOP_MID, -5, 225);
        }
        //intake status box
        if(intake.get_temperature() >= 55)
        {
            extern const lv_img_dsc_t smrev;
            lv_obj_t * imger = lv_img_create(rightBox, NULL);
            lv_img_set_src(imger, &smrev);
            lv_obj_set_pos(imger, 0, 0);
            lv_obj_set_drag(imger, true);
            lv_obj_align(imger, NULL, LV_ALIGN_IN_TOP_MID, 105, 200);
        }
        else
        {
            extern const lv_img_dsc_t pngrev;
            lv_obj_t * imger = lv_img_create(rightBox, NULL);
            lv_img_set_src(imger, &pngrev);
            lv_obj_set_pos(imger, 0, 0);
            lv_obj_set_drag(imger, true);
            lv_obj_align(imger, NULL, LV_ALIGN_IN_TOP_MID, 105, 200);
        }
        //front goal lift status box
        if(frontGoalLift.get_temperature() >= 55)
        {
            extern const lv_img_dsc_t smrev;
            lv_obj_t * imger = lv_img_create(rightBox, NULL);
            lv_img_set_src(imger, &smrev);
            lv_obj_set_pos(imger, 0, 0);
            lv_obj_set_drag(imger, true);
            lv_obj_align(imger, NULL, LV_ALIGN_IN_TOP_MID, 105, 225);
        }
        else
        {
            extern const lv_img_dsc_t pngrev;
            lv_obj_t * imger = lv_img_create(rightBox, NULL);
            lv_img_set_src(imger, &pngrev);
            lv_obj_set_pos(imger, 0, 0);
            lv_obj_set_drag(imger, true);
            lv_obj_align(imger, NULL, LV_ALIGN_IN_TOP_MID, 105, 225);
        }
        //draw all of the motor text
        label2 = lv_label_create(rightBox, NULL);
        lv_label_set_text(label2, "     Claw:");
        lv_obj_align(label2, NULL, LV_ALIGN_IN_TOP_MID, -60, 195);

        label2 = lv_label_create(rightBox, NULL);
        lv_label_set_text(label2, "Claw Lift:");
        lv_obj_align(label2, NULL, LV_ALIGN_IN_TOP_MID, -60, 220);

        label2 = lv_label_create(rightBox, NULL);
        lv_label_set_text(label2, "   Intake:");
        lv_obj_align(label2, NULL, LV_ALIGN_IN_TOP_MID, 50, 195);

        label2 = lv_label_create(rightBox, NULL);
        lv_label_set_text(label2, "Goal Lift:");
        lv_obj_align(label2, NULL, LV_ALIGN_IN_TOP_MID, 50, 220);

        //delay to prevent the program from locking up
        pros::delay(150);
    }
}

void initialize() 
{
	pros::Task trackingTask(trackPosition);
    pros::Task macroTask(threadMacro);
    pros::Task draw(drawUI);
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
        /*
        if(flag)
        {
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_B))
            intake.move_velocity(150);
        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A))
            intake.move_velocity(-150);
        else
            intake.move(0);
        }
        */
        intake.move(127);
        moveGoalLift();
        pros::delay(10);
    }
}

