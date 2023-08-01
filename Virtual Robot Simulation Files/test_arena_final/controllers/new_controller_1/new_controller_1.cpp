#include "Supportive_functions.hpp"



int main(){
  motor_initializing();
  enable_ir_sensors();
  enable_sonar_sensors();
  enable_laser_sensors();


  go_straigh_n_time_steps_with_speed(40,MAX_SPEED);

 cout<< "first Square"<<endl;
  
  



  strat_line_seg_wall_line_upto_T();
  from_white_line_to_upto_colored_junction();
  turning_according_to_decistion();
  dotted_T_upto_chess();
  rook_detection_and_go_closer();



 

  return 0;
}






















/* int main(){

  //enable_sonar_sensors();
  //enable_ir_sensors();
  //motor_initializing();
  
    
/* 
   while (true)
   {

    Front_down_camera->enable(TIME_STEP);
    //int width = Front_down_camera->getWidth();
    //int height = Front_down_camera->getHeight();
    robot->step(TIME_STEP);


    const unsigned char *image = Front_down_camera->getImage();
    

    int r = Front_down_camera->imageGetRed(image , 640 , 100,60);
    int g = Front_down_camera->imageGetGreen(image , 640 , 100,60);
    int b = Front_down_camera->imageGetBlue(image , 640 , 100,60);

    cout << r<<" "<<g<<" "<<b<<endl;

    //cout << width<<endl;
    
   }   */
   

/*   Front_down_camera->enable(TIME_STEP);
  while (true)
   {
    robot->step(TIME_STEP);
    
    cout<<down_cam_left_side_color()<<"        "<<down_cam_right_side_color()<<endl;

    //cout<< down_cam_left_side_color()<<endl;




   }
    

    
  return 0;
}
  */