// IR Sensors
int sensor1 = 24;      // Left most sensor
int sensor2 = 26;
int sensor3 = 28;
int sensor4 = 30;      // Right most sensor
int sensor5 = 32;
int sensor6 = 34; 

// Initial Values of Sensors
int sensor[6] = {0, 0, 0, 0, 0, 0};

// Motor Variables
const int RPWM1 =10;
const int LPWM1 =9;
const int R_EN_L_EN_1 =8;

const int RPWM2 =7;
const int LPWM2 =6;
const int R_EN_L_EN_2 =5;

const int dongco1 =44;
const int dongco2 =46;

#define DISABLE LOW
#define ENABLE HIGH

//Initial Speed of Motor
int toc_do_ban_dau = 120;

// PID Constants
//float Kp = 60;
//float Ki = 0;
//float Kd = 0;

float Kp = 27;
float Ki = 0;
float Kd = 15;

float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;
int PID_trai, PID_phai;

//int flag = 0;
int nho =0;
int dem =0;


void setup()
{
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);
  pinMode(sensor5, INPUT);
  pinMode(sensor6, INPUT);

  pinMode(LPWM1, OUTPUT);
  pinMode(R_EN_L_EN_1, OUTPUT);
  pinMode(RPWM1, OUTPUT);
  pinMode(dongco1, OUTPUT);
  pinMode(dongco2, OUTPUT);

  pinMode(LPWM2, OUTPUT);
  pinMode(R_EN_L_EN_2, OUTPUT);
  pinMode(RPWM2, OUTPUT);

  Serial.begin(9600);                     //setting serial monitor at a default baund rate of 91100
  delay(500);
  Serial.println("Started !!");
  delay(1000);
}

void stop_bot(){
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(R_EN_L_EN_1,DISABLE);
  digitalWrite(R_EN_L_EN_2,DISABLE);
}

void read_sensor_values()
{
  sensor[0] = digitalRead(sensor1);
  sensor[1] = digitalRead(sensor2);
  sensor[2] = digitalRead(sensor3);
  sensor[3] = digitalRead(sensor4);
  sensor[4] = digitalRead(sensor5);
  sensor[5] = digitalRead(sensor6);
  if ((sensor[5] == 0) && (sensor[4] == 0) && (sensor[3] == 0) && (sensor[2] == 1) && (sensor[1] == 1) && (sensor[0] == 1)) // 000111
   {
      error = 5;
      nho =1;
   }
  else if ((sensor[5] == 0) && (sensor[4] == 0) && (sensor[3] == 0) && (sensor[2] == 0) && (sensor[1] == 0) && (sensor[0] == 1)) //000001
    {   
      error = 4;
      nho =1;
    }
  else if ((sensor[5] == 0) && (sensor[4] == 0) && (sensor[3] == 0) && (sensor[2] == 0) && (sensor[1] == 1) && (sensor[0] == 1)) //000011
    {
      error = 3;
      nho =1;
    }
  else if ((sensor[5] == 0) && (sensor[4] == 0) && (sensor[3] == 0) && (sensor[2] == 1) && (sensor[1] == 1) && (sensor[0] == 0)) //000110
    {
      error = 2;
    }
  else if ((sensor[5] == 0) && (sensor[4] == 0) && (sensor[3] == 1) && (sensor[2] == 1) && (sensor[1] == 1) && (sensor[0] == 0)) //001110
    {
      error = 1;
    }
  else if ((sensor[5] == 0) && (sensor[4] == 0) && (sensor[3] == 1) && (sensor[2] == 1) && (sensor[1] == 0) && (sensor[0] == 0)) //001100
    {
      error = 0;
    }
  else if ((sensor[5] == 0) && (sensor[4] == 1) && (sensor[3] == 1) && (sensor[2] == 1) && (sensor[1] == 0) && (sensor[0] == 0)) //011100
    {
      error = -1;
    }
  else if ((sensor[5] == 0) && (sensor[4] == 1) && (sensor[3] == 1) && (sensor[2] == 0) && (sensor[1] == 0) && (sensor[0] == 0)) //011000
    {
      error = -2;
    }
  else if ((sensor[5] == 1) && (sensor[4] == 1) && (sensor[3] == 0) && (sensor[2] == 0) && (sensor[1] == 0) && (sensor[0] == 0)) // 110000
    {
      error = -3;
      nho = 2;
    }
  else if ((sensor[5] == 1) && (sensor[4] == 0) && (sensor[3] == 0) && (sensor[2] == 0) && (sensor[1] == 0) && (sensor[0] == 0)) // 100000
    {
    //  error = 100;
      error =-4;
      nho = 2;
    }
  else if ((sensor[5] == 1) && (sensor[4] == 1) && (sensor[3] == 1) && (sensor[2] == 0) && (sensor[1] == 0) && (sensor[0] == 0)) // 111000
    {
      error = -5;
      nho =2;
    }
  else if ((sensor[5] == 1) && (sensor[4] == 1) && (sensor[3] == 1) && (sensor[2] == 1) && (sensor[1] == 1) && (sensor[0] == 1)) // 111111
    {
       error = 6;
       
    }

  else if ((sensor[5] == 0) && (sensor[4] == 0) && (sensor[3] == 0) && (sensor[2] == 0) && (sensor[1] == 0) && (sensor[0] == 0)) // 000000
    {
         error = -6;
    }
}

void calculate_pid()
{
  P = error;
  I = I + previous_I; // sai số trước đó cộng sai số hiện tại
  D = error - previous_error; // sai số hiện tại trừ sai số trước đó

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);

  previous_I = I;
  previous_error = error;
}

void motor_control()
{
    forward();
  // Calculating the effective motor speed:
  PID_phai = toc_do_ban_dau - PID_value;
  PID_trai = toc_do_ban_dau + PID_value;

  // The motor speed should not exceed the max PWM value
  PID_phai = constrain(PID_phai, 0, 130);
  PID_trai = constrain(PID_trai, 0, 130);

    analogWrite(RPWM1, PID_phai);
    analogWrite(RPWM2, PID_trai );   
  //following lines of code are to make the bot move forward
    //forward();
}

void forward()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(R_EN_L_EN_1,ENABLE);
  digitalWrite(LPWM1,LOW);
  digitalWrite(R_EN_L_EN_2,ENABLE);
  digitalWrite(LPWM2,LOW);

}
void reverse()
{
  /*The pin numbers and high, low values might be different depending on your connections */

  digitalWrite(R_EN_L_EN_1,ENABLE);
  digitalWrite(LPWM1,HIGH);
  digitalWrite(R_EN_L_EN_2,ENABLE);
  digitalWrite(LPWM2,HIGH);
}
void right()
{
  /*The pin numbers and high, low values might be different depending on your connections */

  digitalWrite(R_EN_L_EN_1,ENABLE);
  digitalWrite(LPWM1,LOW);
  digitalWrite(R_EN_L_EN_2,DISABLE);
}
void left()
{
  /*The pin numbers and high, low values might be different depending on your connections */

  digitalWrite(R_EN_L_EN_1,DISABLE);
  digitalWrite(R_EN_L_EN_2,ENABLE);
  digitalWrite(LPWM2,LOW);
}
void sharpRightTurn() {
  /*The pin numbers and high, low values might be different depending on your connections */

  digitalWrite(R_EN_L_EN_1,ENABLE);
  digitalWrite(LPWM1,LOW);
  digitalWrite(R_EN_L_EN_2,ENABLE);
  digitalWrite(LPWM2,HIGH);
}
void sharpLeftTurn() {

  digitalWrite(R_EN_L_EN_1,ENABLE);
  digitalWrite(LPWM1,HIGH);
  digitalWrite(R_EN_L_EN_2,ENABLE);
  digitalWrite(LPWM2,LOW);
}

void ban_1(){   
   dem=0;
   do{
        read_sensor_values(); 
        if(error==6){
           dem++; 
           stop_bot();
           delay(500);      
           analogWrite(RPWM1, 110);
           analogWrite(RPWM2, 110);  
           forward();
           delay(900);
           read_sensor_values();  
        }
        read_sensor_values(); 
        calculate_pid();
        motor_control();
    }
    while(  dem < 1 );       
 }

void ve_1(){
  dem=1;
   do{
        read_sensor_values(); 
        if(error==6){
           dem++; 
           stop_bot();
           delay(500);      
           analogWrite(RPWM1, 110);
           analogWrite(RPWM2, 110);  
           right();
           delay(3800);
           forward();
           delay(200);
           read_sensor_values();  
        }
        read_sensor_values(); 
        calculate_pid();
        motor_control();
  }
  while(  dem <= 4 );

  stop_bot();
  delay(5000);

}


void loop()
{

    ban_1(); 
    ve_1();

 
}
