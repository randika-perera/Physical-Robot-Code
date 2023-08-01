///     robot Design Compettition



#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>
#include <vector>
#include <webots/TouchSensor.hpp>
#include <webots/PositionSensor.hpp>

#define TIME_STEP 16
#define MAX_SPEED 10


using namespace webots;
using namespace std;

Robot *robot = new  Robot();


// -------------------Wheel Motors-----------------------------------------------------------

Motor *Wheel_motor_0 = robot->getMotor("wheel_motor_0");        //Left front
Motor *Wheel_motor_1 = robot->getMotor("wheel_motor_1");        //Left back
Motor *Wheel_motor_2 = robot->getMotor("wheel_motor_2");        // right back
Motor *Wheel_motor_3 = robot->getMotor("wheel_motor_3");        // right fron

// -------------------Top Camera Motors-----------------------------------------------------------

Motor *Top_camera_motor = robot->getMotor("top_camera_motor");     

PositionSensor *Top_camera_pos = robot->getPositionSensor("top_camera_pos");



// -------------------Arm Motors-----------------------------------------------------------

Motor *Arm_verticle_motor = robot->getMotor("arm_verticle_motor");          
Motor *Arm_horizontal_motor_0 = robot->getMotor("arm_horizontal_motor_0");  // left
Motor *Arm_horizontal_motor_1 = robot->getMotor("arm_horizontal_motor_1");    // right

PositionSensor *Arm_verticle_pos = robot->getPositionSensor("arm_verticle_pos");
PositionSensor *Arm_horizontal_pos_0 = robot->getPositionSensor("arm_horizontal_pos_0"); // left
PositionSensor *Arm_horizontal_pos_1 = robot->getPositionSensor("arm_horizontal_pos_1");  // right


// -------------------Antenna  Motors-----------------------------------------------------------

Motor *Antenna_motor_0 = robot->getMotor("antenna_motor_0");          
Motor *Antenna_motor_1 = robot->getMotor("antenna_motor_1"); 

PositionSensor *Antenna_pos_0 = robot->getPositionSensor("antenna_pos_0");
PositionSensor *Antenna_pos_1 = robot->getPositionSensor("antenna_pos_1");


//-------------------------------IR sensors-------------------------------------------------------


DistanceSensor *Ir_0 = robot->getDistanceSensor("ir_0");
DistanceSensor *Ir_1 = robot->getDistanceSensor("ir_1");
DistanceSensor *Ir_2 = robot->getDistanceSensor("ir_2");
DistanceSensor *Ir_3 = robot->getDistanceSensor("ir_3");
DistanceSensor *Ir_4 = robot->getDistanceSensor("ir_4");

//-----------------------------sonar sensors----------------------------------------------------------


DistanceSensor *sonar_0 = robot->getDistanceSensor("sonar_0");  // left front
DistanceSensor *sonar_1 = robot->getDistanceSensor("sonar_1");  // left back
DistanceSensor *sonar_2 = robot->getDistanceSensor("sonar_2");  // right back
DistanceSensor *sonar_3 = robot->getDistanceSensor("sonar_3");  // right front



//-------------------------------Cameras-------------------------------------------------------
Camera *Front_down_camera = robot->getCamera("front_down_camera");
//Camera *Top_camera = robot ->getCamera("top_camera");


//-------------------------------Laser Sensors-------------------------------------------------------

DistanceSensor *laser_sensor_0 = robot->getDistanceSensor("laser_sensor_0");
DistanceSensor *laser_sensor_1 = robot->getDistanceSensor("laser_sensor_1");
DistanceSensor *laser_sensor_2 = robot->getDistanceSensor("laser_sensor_2");

//-----------------------------touch sensors----------------------------------------------------------


TouchSensor *touch_sensor_0 = robot->getTouchSensor("left_touch_sensor");  // left front
TouchSensor *touch_sensor_1 = robot->getTouchSensor("right_touch_sensor");






// --------------------------------Given Color------------------------------------------------------

string dotted_line_color = "red" ;    ///    use either "red"  or "blue"
bool is_red_path_left = true;





int dark_square_color[3] = {  243 , 154 , 154 };
int lite_square_color[3] =  { 253 ,240  ,230  };
/*

<modify this RGB values  testing the real arena colors>

*/

// --------------------------------------------------------------------------------------

//chess arena variables





/*----------------------------line following variables------------------------------------------------------------*/
float kp_white_line = 0.04 ;
float kd_white_line  =0;
float ki_white_line  =0 ;

float total_error_white = 0;
float previous_error_white =0;

float base_speed_white = MAX_SPEED * 0.4 ;  
int    threshold_white = 700;

/*---------------------------------------------*/

float kp_blue_line = 0.04 ;
float kd_blue_line  =0;
float ki_blue_line  =0 ;

float total_error_blue =0;
float previous_error_blue =0;

float base_speed_blue = MAX_SPEED * 0.4 ;
int    threshold_blue = 997;
/*  -------------------------------------------------*/
float kp_red_line = 0.04 ;
float kd_red_line  =0;
float ki_red_line  =0 ;

float total_error_red =0;
float previous_error_red =0;
float base_speed_red = MAX_SPEED * 0.6 ;
int   threshold_red = 600;

/*--------------------------------------------------------------------------------------------------------------------*/




//---------------------------Segmented wall following Variables--------------------------------------------------------------------

    float previousError_segmented_wall = 0;
    float totalError_segmented_wall = 0;
    float Kp_segmented_wall = 0.007;
    float Ki_segmented_wall = 0.0;
    float Kd_segmented_wall = 0.0;
    float refDistance_segmented_wall = 150;    



/*--------------------------------------------------------------------------------------------------------------------*/




/// --------------------------------variaous colors in the board-----------------------------------------

    int floor_black[3] = {32,32,32};
    int path_white[3] = {197,197,197};
    int path_red[3] = {197,32,32};
    int path_blue[3] = {57,32,168};
    int  red_carpet_chess[3] = {113, 32, 32};
    int chess_lite_square[3] = {202 ,193 ,187};
    int chess_dark_square [3] = {168,129,108};

    int downcam_offset = 10;
/// -------------------------------------------------------------------------



