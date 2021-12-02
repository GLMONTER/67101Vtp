#include "main.h"
#include <iomanip>

extern void trackPosition();
extern void threadMacro();

extern pros::Rotation leftEncoder;
extern pros::Rotation middleEncoder;
extern pros::Rotation rightEncoder;

pros::Optical intakeDetect(6);
int flag = true;

void intakeSense()
{
    while(true)
    {
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT))
            break;
        pros::delay(10);
    }
    flag = false;
    intake.move_velocity(140);
    intakeDetect.set_led_pwm(100);
    while(true)
    {
        if(intakeDetect.get_hue() > 80)
        {
            static int temp  = intake.get_position();
            intake.move_relative(700, 75);
            while(std::abs(intake.get_position() - temp) < 300)
            {
                pros::delay(10);
            }
            temp = intake.get_position();
            intake.move_relative(-500, 150);
            while(std::abs(intake.get_position() - temp) < 150)
            {
                pros::delay(10);
            }
            intake.move_velocity(200);
            pros::delay(1000);
            intake.move_velocity(0);
            break;
        }
        pros::delay(10);
    }
    flag = true;
}
void initialize() 
{

	//pros::Task trackingTask(trackPosition);
    pros::Task macroTask(threadMacro);
   // pros::Task intakeTask(intakeSense);
}

void disabled() {}

void competition_initialize() {}

extern void runAuton();

void autonomous() 
{
    runAuton();
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
extern bool runningAuton;
//for screen write
extern sPos gPosition;

struct outputPos
{
    std::string x, y, a;
};
outputPos writerPos;

void opcontrol() 
{
    frontGoalLift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    clawLift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    intake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    runningAuton = false;
       

   



    while(true)
    {
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
        streamOBJ.str("");
    
    
    lv_obj_t * obj2;
    obj2 = lv_obj_create(lv_scr_act(), NULL);
    lv_obj_align(obj2, nullptr, LV_ALIGN_IN_TOP_LEFT, 0, 0);    
    lv_obj_set_style(obj2, &lv_style_btn_tgl_rel);                   
    lv_obj_set_height(obj2, 272);
    lv_obj_set_width(obj2, 240);
    lv_obj_t * label;
 
    label = lv_label_create(obj2, NULL);
    lv_label_set_text(label, "Tracking Data");
    lv_obj_align(label, NULL, LV_ALIGN_IN_TOP_MID, 0, 15);

    label = lv_label_create(obj2, NULL);
    lv_label_set_text(label, " X       Y       R  ");
    lv_obj_align(label, NULL, LV_ALIGN_IN_TOP_MID, 0, 45);

    label = lv_label_create(obj2, NULL);
    std::string values =  writerPos.x + "     " + writerPos.y + "     "  + writerPos.a;
    lv_label_set_text(label, values.c_str());
    lv_obj_align(label, NULL, LV_ALIGN_IN_TOP_MID, 0, 75);


    lv_obj_t * slider = lv_slider_create(obj2, NULL);      
    lv_obj_set_size(slider, 300, 20);   
    lv_obj_align(slider, NULL, LV_ALIGN_IN_TOP_MID, 0, 120); 
    lv_slider_set_knob_in(slider, false);               
    lv_slider_set_value(slider, 100);  

      extern const lv_img_dsc_t vaqr;
	lv_obj_t * img = lv_img_create(obj2, NULL);
	lv_img_set_src(img, &vaqr);
	lv_obj_set_pos(img, 0, 0);
	lv_obj_set_drag(img, true);
    lv_obj_align(img, NULL, LV_ALIGN_IN_TOP_MID, 0, 145);


    

    lv_obj_t * obj3;
    obj3 = lv_obj_create(lv_scr_act(), NULL);
    lv_obj_align(obj3, obj2, LV_ALIGN_OUT_RIGHT_TOP, 0, 0);    
    lv_obj_set_style(obj3, &lv_style_btn_tgl_rel);                  
    lv_obj_set_height(obj3, 272);
    lv_obj_set_width(obj3, 240);
    lv_obj_t * label2;



     label2 = lv_label_create(obj3, NULL);
    lv_label_set_text(label2, std::string("Battery Percentage: " + std::to_string((int)pros::battery::get_capacity()) + "%").c_str());
    lv_obj_align(label2, NULL, LV_ALIGN_IN_TOP_MID, 0, 15);

    extern const lv_img_dsc_t ree;
	lv_obj_t * im = lv_img_create(obj3, NULL);
	lv_img_set_src(im, &ree);
	lv_obj_set_pos(im, 0, 0);
	lv_obj_set_drag(im, true);
    lv_obj_align(im, NULL, LV_ALIGN_IN_TOP_MID, 0, 35);

    label2 = lv_label_create(obj3, NULL);
    lv_label_set_text(label2, "     Claw:");
    lv_obj_align(label2, NULL, LV_ALIGN_IN_TOP_MID, -60, 195);

     label2 = lv_label_create(obj3, NULL);
    lv_label_set_text(label2, "Claw Lift:");
    lv_obj_align(label2, NULL, LV_ALIGN_IN_TOP_MID, -60, 220);

    label2 = lv_label_create(obj3, NULL);
    lv_label_set_text(label2, "   Intake:");
    lv_obj_align(label2, NULL, LV_ALIGN_IN_TOP_MID, 50, 195);

     label2 = lv_label_create(obj3, NULL);
    lv_label_set_text(label2, "Goal Lift:");
    lv_obj_align(label2, NULL, LV_ALIGN_IN_TOP_MID, 50, 220);

    
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

