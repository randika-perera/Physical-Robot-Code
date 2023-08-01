/*
Only maze exploration and storing the path taken is implemented in this code. 
The shortest path algorithm is yet to be implemented.
*/

float newKp=0.066;  
float newKd=0.045;
float newBasespeed=57;  
int previouserror;
int IR_sensor0_pin = 0; 
int IR_sensor1_pin = 1; 
int IR_sensor2_pin = 2; 
int IR_sensor3_pin = 3; 
int IR_sensor4_pin = 4; 
int threshold = 500; //240 night 500 day
int turn_delay=280;
int u_turn_delay=440;
int u_turn_back_delay=50;
int move_forward_little_delay=80;
int move_forward_very_little_delay=10;
int move_forward_big_delay=180;
bool allwhitestatus;
int i=0;


int listx[50];
int list_index=0;

void setup() 
{
  Serial.begin(115200);
  delay(2000);
  pinMode(IR_sensor0_pin,INPUT);
  pinMode(IR_sensor1_pin,INPUT);
  pinMode(IR_sensor2_pin,INPUT);
  pinMode(IR_sensor3_pin,INPUT);
  pinMode(IR_sensor4_pin,INPUT);
  small_push();
  move_forward_big();
  move_forward_big();
}

void loop() 
{

    solve_maze();
    
    //Serial.println(String(analogRead(IR_sensor0_pin))+String("  ")+String(analogRead(IR_sensor1_pin))+String("  ")+String(analogRead(IR_sensor2_pin))+String("  ")+String(analogRead(IR_sensor3_pin))+String("  ")+String(analogRead(IR_sensor4_pin)));
}

void solve_maze()
{
  allwhitestatus=false;
  //Serial.println(String(digital_read(IR_sensor0_pin))+String(digital_read(IR_sensor1_pin))+String(digital_read(IR_sensor2_pin))+String(digital_read(IR_sensor3_pin))+String(digital_read(IR_sensor4_pin)));
  if ((digital_read(IR_sensor0_pin)==1) && (digital_read(IR_sensor1_pin)==1) && (digital_read(IR_sensor2_pin)==1) && (digital_read(IR_sensor3_pin)==0) && (digital_read(IR_sensor4_pin)==0))
  {
    stop_robot();
    move_forward_little();
    move_forward_little();
    //Serial.println("LEFT TURN");
    if ((digital_read(IR_sensor0_pin)==0) && (digital_read(IR_sensor1_pin)==0) && (digital_read(IR_sensor2_pin)==0) && (digital_read(IR_sensor3_pin)==0) && (digital_read(IR_sensor4_pin)==0))
    {
      //do nothing
    }
    else
    {
      listx[list_index]=1;  //1=left
      list_index=list_index+1;
    }
    move_back_little();
    left_turn();
    delay(250);
    small_push();
  }
  else if ((digital_read(IR_sensor0_pin)==1) && (digital_read(IR_sensor1_pin)==1) && (digital_read(IR_sensor2_pin)==1) && (digital_read(IR_sensor3_pin)==1) && (digital_read(IR_sensor4_pin)==1))
  {
    allwhitestatus=true;
    move_back_little();
    move_forward_little();
    stop_robot();
    delay(200);
    if ((digital_read(IR_sensor0_pin)==0) && (digital_read(IR_sensor1_pin)==0) && (digital_read(IR_sensor2_pin)==0) && (digital_read(IR_sensor3_pin)==0) && (digital_read(IR_sensor4_pin)==0))
    {
      Serial.println("LEFT TURN");
      listx[list_index]=1; //1=left
      list_index=list_index+1;
      left_turn();
      delay(250);
      small_push();
    }
    else if ((digital_read(IR_sensor0_pin)==1) && (digital_read(IR_sensor1_pin)==1) && (digital_read(IR_sensor2_pin)==1) && (digital_read(IR_sensor3_pin)==1) && (digital_read(IR_sensor4_pin)==1))
    {
      Serial.println("STOP ROBOT");
      stop_robot();
      //printing the array      
      for (int q = 0; q < list_index+1; q++) 
      {
      Serial.print("Element ");
      Serial.print(q);
      Serial.print(" = ");
      Serial.println(listx[q]);
      }
      //use shortest path function and pass that result to motors
      //shortest_path();
      for (int q = 0; q < list_index+1; q++) 
      {
      Serial.print("Element ");
      Serial.print(q);
      Serial.print(" = ");
      Serial.println(listx[q]);
      }
      left_turn();
      left_turn();

      //shortest_path();
      move_shortest();
      exit(0);
      //delay(10000);
    }
    else 
    {
      Serial.println("LEFT TURN");
      listx[list_index]=1; //1=left
      list_index=list_index+1;
      left_turn();
      delay(250);
      small_push();
    }
  }
  else if (((digital_read(IR_sensor0_pin)==0) && (digital_read(IR_sensor1_pin)==0) && (digital_read(IR_sensor2_pin)==1) && (digital_read(IR_sensor3_pin)==1) && (digital_read(IR_sensor4_pin)==1)))
  {
    move_forward_little();
    delay(500);
    if ((digital_read(IR_sensor0_pin)==0) && (digital_read(IR_sensor1_pin)==0) && (digital_read(IR_sensor2_pin)==0) && (digital_read(IR_sensor3_pin)==0) && (digital_read(IR_sensor4_pin)==0))
    {
      Serial.println("RIGHT TURN");
      right_turn();   
      delay(250);
      small_push();   
    }
    else
    {
      Serial.println("FORWARD");
      listx[list_index]=2; //2=forward
      list_index=list_index+1;
      //move_forward_big();
      line_following_forward();
    }
  }
  else if ((digital_read(IR_sensor0_pin)==0) && (digital_read(IR_sensor1_pin)==0) && (digital_read(IR_sensor2_pin)==0) && (digital_read(IR_sensor3_pin)==0) && (digital_read(IR_sensor4_pin)==0))
  {
    if (allwhitestatus==false)
    {
      Serial.println("U TURN");
      listx[list_index]=4; //4=u_turn
      list_index=list_index+1;
      u_turn();
      delay(500);
    }
  }
  else
  {
    //Serial.println("LINE FOLLOWING");
    line_following_forward();
  }
}


 
void move_shortest()
{
  for (int r=0;r++;)
  {
    if (listx[r]==1)
    {
      left_turn();
    }
    else if (listx[r]==2)
    {
      line_following_forward();
    }
    else if (listx[r]==3)
    {
      right_turn();
    }
    else
    {
      stop_robot();
    }
  }
}

void stop_robot()
{
  motor_control(1,0);
  motor_control(2,0);
}

void move_forward_little()
{
  motor_control(1,120);
  motor_control(2,120);
  delay(move_forward_little_delay);
  motor_control(1,0);
  motor_control(2,0);
}

void move_forward_big()
{
  motor_control(1,120);
  motor_control(2,120);
  delay(move_forward_big_delay);
  motor_control(1,0);
  motor_control(2,0);
}

void left_turn()
{
  motor_control(1,-100);
  motor_control(2,100);
  delay(turn_delay);
  stop_robot();
}

void right_turn()
{
  motor_control(1,100);
  motor_control(2,-100);
  delay(turn_delay);
  stop_robot();
}


void u_turn()
{
  motor_control(1,-80);
  motor_control(2,80);
  while (true)
  {
    if (digital_read(IR_sensor0_pin)==1) 
    break;
  }
  stop_robot();
}

void small_push()
{
  motor_control(1,160);
  motor_control(2,160);
  delay(70);
  stop_robot();
}

void motor_control(int M_name , int value)
{
  int forward_pin_A =50 ;int backward_pin_A =51 ;int enA =6 ;
  int forward_pin_B =53 ;int backward_pin_B =52 ;int enB =7 ;
  pinMode(forward_pin_A,OUTPUT);pinMode(backward_pin_A,OUTPUT);pinMode(enA,OUTPUT);
  pinMode(forward_pin_B,OUTPUT);pinMode(backward_pin_B,OUTPUT);pinMode(enB,OUTPUT);
  if (M_name == 1)
  {
    if (value >=0){digitalWrite(backward_pin_A , LOW);digitalWrite(forward_pin_A , HIGH); analogWrite(enA , value);}
    else{digitalWrite(backward_pin_A , HIGH);digitalWrite(forward_pin_A , LOW);analogWrite(enA , -value);}
  }
  else if (M_name == 2)
  {
    if (value >=0){digitalWrite(backward_pin_B , LOW);digitalWrite(forward_pin_B , HIGH);analogWrite(enB , value);}
    else{digitalWrite(backward_pin_B , HIGH);digitalWrite(forward_pin_B , LOW);analogWrite(enB , -value);}
  }  
}


void line_following_forward()
{
  float kp = newKp;
  float kd = newKd;
  int currenterror=error_v();
  int delta=currenterror-previouserror;
  float CT = currenterror*kp + delta*kd;
  previouserror=currenterror;
  int control_signal_2=  newBasespeed+round(constrain(CT,-255,255));
  int control_signal_1=  newBasespeed-round(constrain(CT,-255,255));
  motor_control(1,control_signal_1);motor_control(2,control_signal_2);
  delay(4);
}

int error_v()
{     
  int error_v = (digital_read(IR_sensor0_pin) * 500 + digital_read(IR_sensor1_pin) * 200 +digital_read(IR_sensor2_pin)* 0 + digital_read(IR_sensor3_pin) * -200 + digital_read(IR_sensor4_pin) * -500) ;
  return error_v;
}


int digital_read(int pin)
{
  int IR_analog_value=analogRead(pin);
  if (IR_analog_value<=threshold)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

void move_back_little()
{
  motor_control(1,-100);
  motor_control(2,-100);
  delay(70);
  stop_robot();
}
