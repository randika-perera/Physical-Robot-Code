float newKp=0.066;  
float newKd=0.045;
float newBasespeed=55;
int previouserror;
int IR_sensor0_pin = 0; 
int IR_sensor1_pin = 1; 
int IR_sensor2_pin = 2; 
int IR_sensor3_pin = 3; 
int IR_sensor4_pin = 4; 
int IR0;
int IR1;
int IR2;
int IR3;
int IR4;
int threshold = 500;

void setup() 
{
  delay(500);
  pinMode(IR_sensor0_pin,INPUT);
  pinMode(IR_sensor1_pin,INPUT);
  pinMode(IR_sensor2_pin,INPUT);
  pinMode(IR_sensor3_pin,INPUT);
  pinMode(IR_sensor4_pin,INPUT);
  shock();
  while (!(((digital_read(IR_sensor0_pin)==1)&&(digital_read(IR_sensor1_pin)==1)&&(digital_read(IR_sensor2_pin)==1)&&(digital_read(IR_sensor3_pin)==1)&&(digital_read(IR_sensor4_pin)==1))||((digital_read(IR_sensor0_pin)==0)&&(digital_read(IR_sensor1_pin)==1)&&(digital_read(IR_sensor2_pin)==1)&&(digital_read(IR_sensor3_pin)==1)&&(digital_read(IR_sensor4_pin)==1))||((digital_read(IR_sensor0_pin)==1)&&(digital_read(IR_sensor1_pin)==1)&&(digital_read(IR_sensor2_pin)==1)&&(digital_read(IR_sensor3_pin)==1)&&(digital_read(IR_sensor4_pin)==0))))
  {
    pid_line_following();
  }
  motor_control(1,0);
  motor_control(2,0);
}

void loop()
{
  //do nothing
}

void shock()
{
  motor_control(1,80);
  motor_control(2,80);
  delay(20);
  motor_control(1,0);
  motor_control(2,0);
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

void pid_line_following()
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
  IR0=digital_read(IR_sensor0_pin);
  IR1=digital_read(IR_sensor1_pin);
  IR2=digital_read(IR_sensor2_pin);
  IR3=digital_read(IR_sensor3_pin);
  IR4=digital_read(IR_sensor4_pin);        
  int error_v = (IR0 * 500 + IR1 * 200 +IR2 * 0 + IR3 * -200 + IR4 * -500) ;
  return error_v;
}


int digital_read(int pin)
{
  int IR_analog_value=analogRead(pin);
  if (IR_analog_value<threshold)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}
