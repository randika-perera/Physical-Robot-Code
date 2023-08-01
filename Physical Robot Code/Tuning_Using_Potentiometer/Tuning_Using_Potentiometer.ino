float newKp;
int potentiometerpin=A8;

void setup() 
{
  Serial.begin(9600);

  pinMode(potentiometerpin,INPUT);
}

void loop() 
{
    
  int potentiometerinput=analogRead(potentiometerpin);
  //int potentiometerinput=1023;
  float Kp=map(potentiometerinput,0,1023,0,4000);
  newKp=Kp/10000;
  
  Serial.print("Kp value is ");
  Serial.println(newKp,4);
  delay(100);
}
