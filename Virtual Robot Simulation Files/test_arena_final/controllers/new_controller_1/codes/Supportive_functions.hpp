#include "Initializing.hpp"

void enable_laser_sensors(){
    //laser_sensor_0->enable(TIME_STEP);
    laser_sensor_1->enable(TIME_STEP);
    //laser_sensor_2->enable(TIME_STEP);
}
void enable_touch_sensors(){
    touch_sensor_0->enable(TIME_STEP);
    touch_sensor_1->enable(TIME_STEP);
}
void enable_Position_sensors(){
    Top_camera_pos->enable(TIME_STEP);
    Arm_verticle_pos->enable(TIME_STEP);
    Arm_horizontal_pos_0->enable(TIME_STEP);
    Arm_horizontal_pos_1->enable(TIME_STEP);
    Antenna_pos_0->enable(TIME_STEP);
    Antenna_pos_1->enable(TIME_STEP);
}
void enable_ir_sensors(){
    Ir_0->enable(TIME_STEP);
    Ir_1->enable(TIME_STEP);
    Ir_2->enable(TIME_STEP);
    Ir_3->enable(TIME_STEP);
    Ir_4->enable(TIME_STEP);

}

void enable_sonar_sensors(){
    sonar_0->enable(TIME_STEP);
    sonar_1->enable(TIME_STEP);
    sonar_2->enable(TIME_STEP);
    sonar_3->enable(TIME_STEP);
}



void motor_initializing(){
    Wheel_motor_0->setPosition(INFINITY);    Wheel_motor_1->setPosition(INFINITY);    Wheel_motor_2->setPosition(INFINITY);    Wheel_motor_3->setPosition(INFINITY);
    Antenna_motor_0->setPosition(INFINITY);  Antenna_motor_1->setPosition(INFINITY);
    Arm_verticle_motor->setPosition(INFINITY);
    Arm_horizontal_motor_0->setPosition(INFINITY);   Arm_horizontal_motor_1->setPosition(INFINITY);
    


    Wheel_motor_0->setVelocity(0.0);    Wheel_motor_1->setVelocity(0.0);    Wheel_motor_2->setVelocity(0.0);    Wheel_motor_3->setVelocity(0.0);
    Antenna_motor_0->setVelocity(0.0);  Antenna_motor_1->setVelocity(0.0);
    Arm_verticle_motor->setVelocity(0.0);
    Arm_horizontal_motor_0->setVelocity(0.0);   Arm_horizontal_motor_1->setVelocity(0.0);
    
    // set postion
    // set velocity

    // wheel motors -4
    //  antenna morors -1
    // arm motors - 2

}




void white_line_following(){
    float ds0 = Ir_0->getValue();
    float ds1 = Ir_1->getValue();
    float ds2 = Ir_2->getValue();
    float ds3 = Ir_3->getValue();
    float ds4 = Ir_4->getValue();

    //cout << ds0 <<" "<< ds1 <<" "<<ds2 <<" "<< ds3 <<" "<<ds4 << endl;

    if (ds0<threshold_white){ ds0 = 1; } else {ds0 = 0;}
    if (ds1<threshold_white){ ds1 = 1; } else {ds1 = 0;}
    if (ds2<threshold_white){ ds2 = 1; } else {ds2 = 0;}
    if (ds3<threshold_white){ ds3 = 1; } else {ds3 = 0;}
    if (ds4<threshold_white){ ds4 = 1; } else {ds4 = 0;}

    float error = ds0*(-500) +   ds1*(-100) +  ds2*(0) +  ds3*(100)  +  ds4*(500) ;

    float control_signal = error * kp_white_line + (error - previous_error_white)*kd_white_line + total_error_white*ki_white_line ; 

    if(control_signal>MAX_SPEED*0.5){control_signal = MAX_SPEED*0.5;  }
  

    float left_motor_velocicity = base_speed_white + control_signal ;
    float right_motor_velocicity = base_speed_white - control_signal ;

   

    Wheel_motor_1->setVelocity(left_motor_velocicity);
    Wheel_motor_2->setVelocity(right_motor_velocicity);
    Wheel_motor_0->setVelocity(left_motor_velocicity);
    Wheel_motor_3->setVelocity(right_motor_velocicity);
}
    


void red_line_following(){
    float ds0 = Ir_0->getValue();
    float ds1 = Ir_1->getValue();
    float ds2 = Ir_2->getValue();
    float ds3 = Ir_3->getValue();
    float ds4 = Ir_4->getValue();

    cout << ds0 <<" "<< ds1 <<" "<<ds2 <<" "<< ds3 <<" "<<ds4 << endl;

    if (ds0<threshold_red){ ds0 = 1; } else {ds0 = 0;}
    if (ds1<threshold_red){ ds1 = 1; } else {ds1 = 0;}
    if (ds2<threshold_red){ ds2 = 1; } else {ds2 = 0;}
    if (ds3<threshold_red){ ds3 = 1; } else {ds3 = 0;}
    if (ds4<threshold_red){ ds4 = 1; } else {ds4 = 0;}

    float error = ds0*(-500) +   ds1*(-100) +  ds2*(0) +  ds3*(100)  +  ds4*(500) ;

    float control_signal = error * kp_red_line + (error - previous_error_red)*kd_red_line + total_error_red*ki_red_line ; 

    if(control_signal>MAX_SPEED*0.5){control_signal = MAX_SPEED*0.5;  }


    float left_motor_velocicity = base_speed_blue + control_signal ;
    float right_motor_velocicity = base_speed_blue - control_signal ;



    Wheel_motor_1->setVelocity(left_motor_velocicity);
    Wheel_motor_2->setVelocity(right_motor_velocicity);
    Wheel_motor_0->setVelocity(left_motor_velocicity);
    Wheel_motor_3->setVelocity(right_motor_velocicity); 

}


void blue_line_following(){
    float ds0 = Ir_0->getValue();
    float ds1 = Ir_1->getValue();
    float ds2 = Ir_2->getValue();
    float ds3 = Ir_3->getValue();
    float ds4 = Ir_4->getValue();

    //cout << ds0 <<" "<< ds1 <<" "<<ds2 <<" "<< ds3 <<" "<<ds4 << endl;

    if (ds0<threshold_blue){ ds0 = 1; } else {ds0 = 0;}
    if (ds1<threshold_blue){ ds1 = 1; } else {ds1 = 0;}
    if (ds2<threshold_blue){ ds2 = 1; } else {ds2 = 0;}
    if (ds3<threshold_blue){ ds3 = 1; } else {ds3 = 0;}
    if (ds4<threshold_blue){ ds4 = 1; } else {ds4 = 0;}

    float error = ds0*(-500) +   ds1*(-100) +  ds2*(0) +  ds3*(100)  +  ds4*(500) ;

    float control_signal = error * kp_blue_line + (error - previous_error_blue)*kd_blue_line + total_error_blue*ki_blue_line ; 

    if(control_signal>MAX_SPEED*0.5){control_signal = MAX_SPEED*0.5;  }


    float left_motor_velocicity = base_speed_blue + control_signal ;
    float right_motor_velocicity = base_speed_blue - control_signal ;



    Wheel_motor_1->setVelocity(left_motor_velocicity);
    Wheel_motor_2->setVelocity(right_motor_velocicity);
    Wheel_motor_0->setVelocity(left_motor_velocicity);
    Wheel_motor_3->setVelocity(right_motor_velocicity);

}


void seg_wall_following(){
    
    // initiate PID controller
    float error=0;
    float CTRSignal_value;
    float readValue;

    // Main loop:
    // - perform simulation steps until Webots is stopping the controller
    float dis_sensor_val_1 = sonar_0->getValue();
    float dis_sensor_val_2 = sonar_1->getValue();
    float dis_sensor_val_3 = sonar_2->getValue();
    float dis_sensor_val_4 = sonar_3->getValue();   


    if ((dis_sensor_val_1 == 1000) && (dis_sensor_val_2<1000)){
    readValue = dis_sensor_val_2;
    }
    else if ((dis_sensor_val_1 < 1000) && (dis_sensor_val_2 ==1000)){
    readValue = dis_sensor_val_1;
    }
    else if ((dis_sensor_val_1 < 1000) && (dis_sensor_val_2<1000)){
    readValue = dis_sensor_val_1;
    }
    else{
    readValue = refDistance_segmented_wall;
    }
    error = refDistance_segmented_wall - readValue;
    CTRSignal_value = Kp_segmented_wall*error + Ki_segmented_wall*totalError_segmented_wall + Kd_segmented_wall*(error-previousError_segmented_wall);

    if (CTRSignal_value > 0.5)
    {
    CTRSignal_value = 0.5;
    } 
    else if (CTRSignal_value < -0.085){
    CTRSignal_value = -0.85;
    }
    Wheel_motor_0->setVelocity((0.5 + CTRSignal_value)*MAX_SPEED);
    Wheel_motor_3->setVelocity((0.5 - CTRSignal_value)*MAX_SPEED);
    Wheel_motor_1->setVelocity((0.5 + CTRSignal_value)*MAX_SPEED);
    Wheel_motor_2->setVelocity((0.5 - CTRSignal_value)*MAX_SPEED);

   // cout<<dis_sensor_val_1<<endl;

    totalError_segmented_wall += error; 
    previousError_segmented_wall = error;
  
}

 

void go_straigh_n_time_steps_with_speed(int n_steps, float speed){
    
    
        Wheel_motor_0->setVelocity(speed);
        Wheel_motor_1->setVelocity(speed);
        Wheel_motor_2->setVelocity(speed);
        Wheel_motor_3->setVelocity(speed);


    for(int i=0;i<n_steps;i++){
        //cout<<i<<endl;
        robot->step(TIME_STEP);
       
    }

        Wheel_motor_0->setVelocity(0.0);
        Wheel_motor_1->setVelocity(0.0);
        Wheel_motor_2->setVelocity(0.0);
        Wheel_motor_3->setVelocity(0.0);
        robot->step(TIME_STEP);
       
    
}




bool is_sonar_detected(){

///   check whether at leat one sonar in the front has detected the wall
    int s0 = sonar_0->getValue();
    int s1 = sonar_0->getValue();
    int s2 = sonar_0->getValue();
    int s3 = sonar_0->getValue();
    //cout<< s0<<" "<<s1 <<" "<<s2 <<" "<<s3<<endl;

    if (s0<250 ||s1<250 || s2<250 || s3<250){
        return true;
    }
    

    else { return false; }

}


bool is_at_least_1_IR_detected(){

    // here isto check if we have sucessessfully ended the segmented wall follwoing
    int a0 = Ir_0->getValue();
    int a1 = Ir_1->getValue();
    int a2 = Ir_2->getValue();
    int a3 = Ir_3->getValue();

    if(a0<1000 || a1<1000 || a2<1000 || a3< 1000){
        return true;
    }

    //cout <<a0 <<" "  <<  a1<<"  "  <<a2<<" "<< a3<<endl;
    return false;
}


void stop_all_wheel_motors(){
     // this can be used intermediately to stop the motors
     // in order to stop the motors in intermediate stages

    Wheel_motor_0->setVelocity(0);
    Wheel_motor_1->setVelocity(0);
    Wheel_motor_2->setVelocity(0);
    Wheel_motor_3->setVelocity(0);
    robot->step(TIME_STEP);

}


string down_cam_right_side_color(){

    const unsigned char *image = Front_down_camera->getImage();
    
    int r = Front_down_camera->imageGetRed(image , 640 , 520,100);
    int g = Front_down_camera->imageGetGreen(image , 640 , 520,100);
    int b = Front_down_camera->imageGetBlue(image , 640 , 520,100);

    //cout << r<<" "<<g<<" "<<b<<endl;


    int offset = downcam_offset;


    bool floor_black_r = (floor_black[0]-offset) < r && r<(floor_black[0]+offset);
    bool floor_black_g = (floor_black[1]-offset) < g && g<(floor_black[1]+offset);
    bool floor_black_b = (floor_black[2]-offset) < b && b<(floor_black[2]+offset);

    bool path_white_r = (path_white[0]-offset) < r && r<(path_white[0]+offset);
    bool path_white_g = (path_white[1]-offset) < g && g<(path_white[1]+offset);
    bool path_white_b = (path_white[2]-offset) < b && b<(path_white[2]+offset);

    bool path_red_r = (path_red[0]-offset) < r && r<(path_red[0]+offset);
    bool path_red_g = (path_red[1]-offset) < g && g<(path_red[1]+offset);
    bool path_red_b = (path_red[2]-offset) < b && b<(path_red[2]+offset);


    bool path_blue_r = (path_blue[0]-offset) < r && r<(path_blue[0]+offset);
    bool path_blue_g = (path_blue[1]-offset) < g && g<(path_blue[1]+offset);
    bool path_blue_b = (path_blue[2]-offset) < b && b<(path_blue[2]+offset);


    bool red_carpet_chess_r = (red_carpet_chess[0]-offset) < r && r<(red_carpet_chess[0]+offset);
    bool red_carpet_chess_g = (red_carpet_chess[1]-offset) < g && g<(red_carpet_chess[1]+offset);
    bool red_carpet_chess_b = (red_carpet_chess[2]-offset) < b && b<(red_carpet_chess[2]+offset);


    bool chess_lite_square_r = (chess_lite_square[0]-offset) < r && r<(chess_lite_square[0]+offset);
    bool chess_lite_square_g = (chess_lite_square[1]-offset) < g && g<(chess_lite_square[1]+offset);
    bool chess_lite_square_b = (chess_lite_square[2]-offset) < b && b<(chess_lite_square[2]+offset);


    bool chess_dark_square_r = (chess_dark_square[0]-offset) < r && r<(chess_dark_square[0]+offset);
    bool chess_dark_square_g = (chess_dark_square[1]-offset) < g && g<(chess_dark_square[1]+offset);
    bool chess_dark_square_b = (chess_dark_square[2]-offset) < b && b<(chess_dark_square[2]+offset);


    if(floor_black_r && floor_black_g && floor_black_b){
        return "floor_black";
    }
    else if (path_white_r && path_white_g && path_white_b){
        return "path_white";
    }
    else if (path_red_r && path_red_g && path_red_b){
        return "path_red";
    }
    else if (path_blue_r && path_blue_g && path_blue_b){
        return "path_blue";
    }
    else if (red_carpet_chess_r && red_carpet_chess_g && red_carpet_chess_b){
        return "red_carpet_chess";
    }
    else if (chess_dark_square_r && chess_dark_square_g && chess_dark_square_b){
        return "chess_dark_square";
    }
    else if (chess_lite_square_r && chess_lite_square_g && chess_lite_square_b){
            return "chess_lite_square";
        }
    return "other";
}


string down_cam_left_side_color(){
  
    const unsigned char *image = Front_down_camera->getImage();
    
    int r = Front_down_camera->imageGetRed(image , 640 , 48,100);
    int g = Front_down_camera->imageGetGreen(image , 640 , 48,100);
    int b = Front_down_camera->imageGetBlue(image , 640 , 48,100);

    //cout << r<<" "<<g<<" "<<b<<endl;


    int offset = downcam_offset;




    bool floor_black_r = (floor_black[0]-offset) < r && r<(floor_black[0]+offset);
    bool floor_black_g = (floor_black[1]-offset) < g && g<(floor_black[1]+offset);
    bool floor_black_b = (floor_black[2]-offset) < b && b<(floor_black[2]+offset);

    bool path_white_r = (path_white[0]-offset) < r && r<(path_white[0]+offset);
    bool path_white_g = (path_white[1]-offset) < g && g<(path_white[1]+offset);
    bool path_white_b = (path_white[2]-offset) < b && b<(path_white[2]+offset);

    bool path_red_r = (path_red[0]-offset) < r && r<(path_red[0]+offset);
    bool path_red_g = (path_red[1]-offset) < g && g<(path_red[1]+offset);
    bool path_red_b = (path_red[2]-offset) < b && b<(path_red[2]+offset);


    bool path_blue_r = (path_blue[0]-offset) < r && r<(path_blue[0]+offset);
    bool path_blue_g = (path_blue[1]-offset) < g && g<(path_blue[1]+offset);
    bool path_blue_b = (path_blue[2]-offset) < b && b<(path_blue[2]+offset);


    bool red_carpet_chess_r = (red_carpet_chess[0]-offset) < r && r<(red_carpet_chess[0]+offset);
    bool red_carpet_chess_g = (red_carpet_chess[1]-offset) < g && g<(red_carpet_chess[1]+offset);
    bool red_carpet_chess_b = (red_carpet_chess[2]-offset) < b && b<(red_carpet_chess[2]+offset);


    bool chess_lite_square_r = (chess_lite_square[0]-offset) < r && r<(chess_lite_square[0]+offset);
    bool chess_lite_square_g = (chess_lite_square[1]-offset) < g && g<(chess_lite_square[1]+offset);
    bool chess_lite_square_b = (chess_lite_square[2]-offset) < b && b<(chess_lite_square[2]+offset);


    bool chess_dark_square_r = (chess_dark_square[0]-offset) < r && r<(chess_dark_square[0]+offset);
    bool chess_dark_square_g = (chess_dark_square[1]-offset) < g && g<(chess_dark_square[1]+offset);
    bool chess_dark_square_b = (chess_dark_square[2]-offset) < b && b<(chess_dark_square[2]+offset);


    if(floor_black_r && floor_black_g && floor_black_b){
        return "floor_black";
    }
    else if (path_white_r && path_white_g && path_white_b){
        return "path_white";
    }
    else if (path_red_r && path_red_g && path_red_b){
        return "path_red";
    }
    else if (path_blue_r && path_blue_g && path_blue_b){
        return "path_blue";
    }
    else if (red_carpet_chess_r && red_carpet_chess_g && red_carpet_chess_b){
        return "red_carpet_chess";
    }
    else if (chess_dark_square_r && chess_dark_square_g && chess_dark_square_b){
        return "chess_dark_square";
    }
    else if (chess_lite_square_r && chess_lite_square_g && chess_lite_square_b){
            return "chess_lite_square";
        }
    return "other";
    }



void turn_right_n_time_step_with_speed( int n , float speed){

    Wheel_motor_0->setVelocity(speed);
    Wheel_motor_1->setVelocity(speed);
    Wheel_motor_2->setVelocity(-speed);
    Wheel_motor_3->setVelocity(-speed);


    for(int i=0;i<n;i++){
        robot->step(TIME_STEP);
        cout<<i<<endl;
    }

    Wheel_motor_0->setVelocity(0);
    Wheel_motor_1->setVelocity(0);
    Wheel_motor_2->setVelocity(0);
    Wheel_motor_3->setVelocity(0);
    robot->step(TIME_STEP);


    // after turning right no need to stop motors
}

void turn_left_n_time_step_with_speed( int n , float speed){

    Wheel_motor_0->setVelocity(-speed);
    Wheel_motor_1->setVelocity(-speed);
    Wheel_motor_2->setVelocity(speed);
    Wheel_motor_3->setVelocity(speed);


    for(int i=0;i<n;i++){
        robot->step(TIME_STEP);
        cout<<i<<endl;
    }

    Wheel_motor_0->setVelocity(0);
    Wheel_motor_1->setVelocity(0);
    Wheel_motor_2->setVelocity(0);
    Wheel_motor_3->setVelocity(0);
    robot->step(TIME_STEP);

}

void antenna_hoist(){
    if (Antenna_pos_1->getValue() <= 0.2){
      Antenna_motor_1->setVelocity(0.01*MAX_SPEED);
    }
    else{
      Antenna_motor_1->setVelocity(0.0);
       if (Antenna_pos_0->getValue() <= 0.2){
        Antenna_motor_0->setVelocity(0.01*MAX_SPEED);
      }
      else{
        Antenna_motor_0->setVelocity(0.0);
        cout<<Top_camera_pos->getValue()<<endl;
        if (Top_camera_pos->getValue() <= 1.57/2){
          cout<<"hai"<<endl;
          Top_camera_motor->setVelocity(0.5*MAX_SPEED);
        }
        else{
         Top_camera_motor->setVelocity(0.0);
        }}}
}

void lift_box(){
    if ((touch_sensor_0->getValue() < 1) && (touch_sensor_1->getValue() < 1)){
       Arm_horizontal_motor_0->setVelocity(0.005*MAX_SPEED);
       Arm_horizontal_motor_1->setVelocity(0.005*MAX_SPEED);
    }
    else{
        Arm_horizontal_motor_0->setVelocity(0.0);
        Arm_horizontal_motor_1->setVelocity(0.0);
        if (Arm_verticle_pos->getValue() < 0.025){
            Arm_verticle_motor->setVelocity(0.005*MAX_SPEED);
        }
        else{
            Arm_verticle_motor->setVelocity(0.0);
}
}

}





void strat_line_seg_wall_line_upto_T(){
   //-------------------------------------stage 1 - line floowing-----------------------------------------
    while (true)
    {

    cout<<"white line following"<<endl;
    robot->step(TIME_STEP);
    white_line_following();
    
    if (is_sonar_detected()){
        break;
    }

    }
     
//---------------------stage 2 -  sgemnted wall foloowing---------------------------------------------------------

    while (true)
    {
        cout<<" segmented wall following"<<endl;
        robot->step(TIME_STEP);
        seg_wall_following();

        if(is_at_least_1_IR_detected()){
            break;
        }

    }

    stop_all_wheel_motors();

  
//---------------------stage 3 -  white line and colored line (dotted) following--------------------------------------------------------
    
    cout<< "white square"<<endl;
    go_straigh_n_time_steps_with_speed(80 , MAX_SPEED);
    stop_all_wheel_motors();

//----------------------------------------------------------------------------------------------------
}




void from_white_line_to_upto_colored_junction(){

    

  Front_down_camera->enable(TIME_STEP);

   while (true)
    {
      cout<< "white line following "<<endl;
        robot->step(TIME_STEP);
        white_line_following();         

        if ((down_cam_left_side_color() == "path_red") || (down_cam_right_side_color() == "path_blue" )){
          stop_all_wheel_motors();
          break;
        }


    }


 
//----------------------------------------------------------------------------------------------------

cout<<"Turnig decition"<<endl;

}

void turning_according_to_decistion(){
    stop_all_wheel_motors();
    if (is_red_path_left && dotted_line_color=="red"){
            turn_left_n_time_step_with_speed(30,MAX_SPEED);
            stop_all_wheel_motors();
    }
    else if (is_red_path_left && dotted_line_color=="blue"){
            turn_right_n_time_step_with_speed(30,MAX_SPEED);
            stop_all_wheel_motors();
    }
    else if ((!is_red_path_left) && dotted_line_color=="red"){
            turn_right_n_time_step_with_speed(30,MAX_SPEED);
            stop_all_wheel_motors();
    }
    else if (!is_red_path_left && dotted_line_color=="blue"){
            turn_left_n_time_step_with_speed(30,MAX_SPEED);
            stop_all_wheel_motors();
    }
}




void dotted_T_upto_chess(){
    while (true)
    {
        robot->step(TIME_STEP);
        red_line_following();
        if (Ir_0->getValue()<700){

        stop_all_wheel_motors();
        turn_left_n_time_step_with_speed(30,MAX_SPEED);
        go_straigh_n_time_steps_with_speed(20,MAX_SPEED*0.5);
        break;

        cout<<"T"<<endl;
        }

    }


        while (true)
        {
        robot->step(TIME_STEP);
        white_line_following();
        if(is_sonar_detected()){
            stop_all_wheel_motors();
            break;
        }
        } 


}

void rook_detection_and_go_closer(){

  while (true)
  {
    robot->step(TIME_STEP);
    float laser_sensor_1_val = laser_sensor_1->getValue();

    go_straigh_n_time_steps_with_speed(50,MAX_SPEED*0.5);
    if (laser_sensor_1_val<1000){
      stop_all_wheel_motors();
        break;
    }
    cout << laser_sensor_1_val<<endl;
  }
}




/*
void down_cam_get_color_arr(){

    // enable the camera in this scope   
    //add the junc code for the camera to be started
    Front_down_camera->enable(TIME_STEP);
    int width = Front_down_camera->getWidth();
    int height = Front_down_camera->getHeight();

    int arr[8][3];

    const unsigned char *image = Front_down_camera->getImage();


    int r = Front_down_camera->imageGetRed(image , width , 100,60);
    int g = Front_down_camera->imageGetGreen(image , width , 100,60);
    int b = Front_down_camera->imageGetBlue(image , width , 100,60);

    
    int r = Front_down_camera->imageGetRed(image , width , 200,60);
    int g = Front_down_camera->imageGetGreen(image , width , 200,60);
    int b = Front_down_camera->imageGetBlue(image , width , 200,60);

    int r = Front_down_camera->imageGetRed(image , width , 400,60);
    int g = Front_down_camera->imageGetGreen(image , width , 400,60);
    int b = Front_down_camera->imageGetBlue(image , width , 400,60);

    int r = Front_down_camera->imageGetRed(image , width , 500,60);
    int g = Front_down_camera->imageGetGreen(image , width , 500,60);
    int b = Front_down_camera->imageGetBlue(image , width , 500,60);
//-----------------------------

    int r = Front_down_camera->imageGetRed(image , width , 100,70);
    int g = Front_down_camera->imageGetGreen(image , width , 100,70);
    int b = Front_down_camera->imageGetBlue(image , width , 100,70);

    int r = Front_down_camera->imageGetRed(image , width , 200,70);
    int g = Front_down_camera->imageGetGreen(image , width , 200,70);
    int b = Front_down_camera->imageGetBlue(image , width , 200,70);

    int r = Front_down_camera->imageGetRed(image , width , 400,70);
    int g = Front_down_camera->imageGetGreen(image , width , 400,70);
    int b = Front_down_camera->imageGetBlue(image , width , 400,70);

    int r = Front_down_camera->imageGetRed(image , width , 500,70);
    int g = Front_down_camera->imageGetGreen(image , width , 500,70);
    int b = Front_down_camera->imageGetBlue(image , width , 500,70);
// --------------------------




    /////  set the camera resolution to be 
     
   
   // 640 x 480 



}

 */

/*
string cam_right_side_color(){

    
    
    
    
    
    //add some procesing code for this section to identify the color 
    
    
    
    




    //return "red";
    return "blue";
}

 bool is_colored_junction_arrived(){
    string color = cam_right_side_color();
    if(color=="red" || color =="blue" ){
        return 1;
    }
    return 0;
 }






bool is_left_side_downcam_color_white(){

    return 1;

}

bool is_right_side_downcam_color_white(){

    return 1;

}


bool is_red_color_detected(){

    return true;
}


void chase_the_box(){

    // in this function we need to 
    /*
     detect the box  and 
     we need to go closer to the box simettryically
    
    
}

*/


/*
void lift_the_box(){


}

void lift_the_slider(){


    //in this function we need to lift two sliders
    
   // the hegiths have to be measure using position sensors

    //calclulate the relative heights and fing how many loops to be executed 
    

}
*/


/* 
void rotate_the_camera_solid_CW_90(){


}




void is_obstacle_piece_detected(){

    // use a suitable sensor to detect a ppies ahead of the robot arm


}




bool king_cross_detected(){


    return 1;
}


  */


