//Arduino code
#include "SoftwareSerial.h"
SoftwareSerial s(51,53);

int sensor1 = 24;      // Left most sensor
int sensor2 = 26;
int sensor3 = 28;
int sensor4 = 30;      // Right most sensor
int sensor5 = 32;
int sensor6 = 34; 

int sensor[6] = {0, 0, 0, 0, 0, 0};

const int RPWM1 =10;
const int LPWM1 =9;
const int R_EN_L_EN_1 =8;

const int RPWM2 =7;
const int LPWM2 =6;
const int R_EN_L_EN_2 =5;

const int cambien = 44;

const int dongco1 =50;
const int dongco2 =52;

const int congtac1 =40;
const int congtac2 =42;

#define DISABLE LOW
#define ENABLE HIGH

int toc_do_ban_dau = 120;
int dulieu=0;

float Kp = 27;
float Ki = 0;
float Kd = 15;

float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;
int PID_trai, PID_phai;

//int flag = 0;
int nho =0;
int dem =0;
int dung =0;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  s.begin(9600);
  
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);
  pinMode(sensor5, INPUT);
  pinMode(sensor6, INPUT);

  pinMode(LPWM1, OUTPUT);
  pinMode(R_EN_L_EN_1, OUTPUT);
  pinMode(RPWM1, OUTPUT);
 

  pinMode(LPWM2, OUTPUT);
  pinMode(R_EN_L_EN_2, OUTPUT);
  pinMode(RPWM2, OUTPUT);

  pinMode(congtac1, INPUT_PULLUP);
  pinMode(congtac2, INPUT_PULLUP);
  pinMode(cambien, INPUT);

  pinMode(dongco1, OUTPUT);
  pinMode(dongco2, OUTPUT); 
  
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

void ve_1(){
   dem=1;
   analogWrite(RPWM1, 110);
   analogWrite(RPWM2, 110);  
   forward();
   delay(900);
   do{
        read_sensor_values(); 
        if(error==6){
           dem++; 
           stop_bot();
           delay(500);      
           analogWrite(RPWM1, 110);
           analogWrite(RPWM2, 110);  
           right();
           delay(2800);
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

void ban_3(){
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
           delay(3000);
           forward();
           delay(200);
           read_sensor_values();  
        }
        read_sensor_values(); 
        calculate_pid();
        motor_control();
    }
    while(  dem < 2 );
    dem =2;
    do{
        read_sensor_values(); 
        if(error==6){
           dem++; 
           stop_bot();
           delay(500);       
           congtac();
           read_sensor_values();  
        }
        read_sensor_values(); 
        calculate_pid();
        motor_control();
      
    }
    while(  dem < 3 );
 
   
}
void ve_3(){
   dem=3;
  analogWrite(RPWM1, 110);
  analogWrite(RPWM2, 110);  
  right();
  delay(2800);
  forward();
  delay(200);
   do{
        read_sensor_values(); 
        if(error==6){
           dem++; 
           stop_bot();
           delay(500);      
           analogWrite(RPWM1, 110);
           analogWrite(RPWM2, 110);  
           right();
           delay(2800);
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

void ban_2(){
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

    dem =1;
    do{
        read_sensor_values(); 
        if(error==6){
           dem++; 
           stop_bot();
           delay(500);       
           congtac();
           read_sensor_values();  
        }
        read_sensor_values(); 
        calculate_pid();
        motor_control();
      
    }
    while(  dem < 2 );
 
   
}
void ve_2(){
   dem=2;
  analogWrite(RPWM1, 110);
  analogWrite(RPWM2, 110);  
  right();
  delay(2800);
  forward();
  delay(200);
   do{
        read_sensor_values(); 
        if(error==6){
           dem++; 
           stop_bot();
           delay(500);      
           analogWrite(RPWM1, 110);
           analogWrite(RPWM2, 110);  
           right();
           delay(2800);
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
void ban_1(){   
   dem=0;
   do{
        read_sensor_values(); 
        if(error==6){
           dem++; 
           stop_bot();
           delay(500);            
           congtac();
           read_sensor_values();  
        }
        read_sensor_values(); 
        calculate_pid();
        motor_control();
    }
    while(  dem < 1 );       
 }


void congtac(){
  if(digitalRead(congtac2)==0){
    if(digitalRead(cambien)==0){
      digitalWrite(dongco1,LOW);
      digitalWrite(dongco2,HIGH);
    }
    else if( digitalRead(cambien)==1){
      digitalWrite(dongco1,LOW);
      digitalWrite(dongco2,LOW);
    }
  }

  if(digitalRead(congtac1)==0){
    if(digitalRead(cambien)==0){
      digitalWrite(dongco1,LOW);
      digitalWrite(dongco2,LOW); 
    }
    else if(digitalRead(cambien)==1){
      digitalWrite(dongco1,HIGH);
      digitalWrite(dongco2,LOW);
    }
  }
}
void ban_4(){
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
           delay(2800);
           forward();
           delay(200);
           read_sensor_values();  
        }
        read_sensor_values(); 
        calculate_pid();
        motor_control();
    }
    while(  dem < 3 );
    dem = 3 ; 
    do{
        read_sensor_values(); 
        if(error==6){
           dem++; 
           stop_bot();
           delay(500);       
           congtac();
           read_sensor_values();  
        }
        read_sensor_values(); 
        calculate_pid();
        motor_control();
      
    }
    while(  dem < 4 ); 
}
void ve_4(){
   dem=4;
   do{
        read_sensor_values(); 
        if(error==6){
           dem++; 
           stop_bot();
           delay(500);      
           analogWrite(RPWM1, 110);
           analogWrite(RPWM2, 110);  
           right();
           delay(2800);
           forward();
           delay(200);
           read_sensor_values();  
        }
        read_sensor_values(); 
        calculate_pid();
        motor_control();
  }
  while(  dem <= 5 );
  stop_bot();
  delay(5000);
}

void loop() {
     if(Serial1.available()>0)
     {
        dulieu = Serial1.read();
        Serial.print(dulieu);        
        Serial.print("\n");
        dung =0;
        if (dulieu == 1){
          forward();
        }
        if (dulieu == 2){
           left();
        }
         if (dulieu == 3){
           stop_bot();    
        }
        if (dulieu == 4){
    
           right();
        }
         if (dulieu == 5){
           reverse();    
        }        
        if (dulieu == 6){
          while(1){
             if(digitalRead(cambien)==0 && digitalRead(congtac2)==0){ 
                ban_1();
                stop_bot();
                delay(5000); 
            } 
            congtac();
              
            if(digitalRead(cambien)==1 && digitalRead(congtac2)==0){
                  digitalWrite(dongco1,LOW);
                  digitalWrite(dongco2,LOW);
                  delay(2000);
                  ve_1();
                  dung++;
              }
              if(dung ==1){
              break;
            }
          }
        }
        if (dulieu == 7){
          while(1){
              if(digitalRead(cambien)==0 && digitalRead(congtac2)==0){ 
                  ban_2();
                  stop_bot();
                  delay(5000); 
              } 
              congtac();      
              if(digitalRead(cambien)==1 && digitalRead(congtac2)==0){
                    digitalWrite(dongco1,LOW);
                    digitalWrite(dongco2,LOW);
                    delay(2000);
                    ve_2();
                    dung++;            
              }
              if(dung == 1){
                break; 
              }
          }
        }
        if (dulieu == 8){
          while(1){
             if(digitalRead(cambien)==0 && digitalRead(congtac2)==0){ 
                ban_3();
                stop_bot();
                delay(5000); 
            } 
            congtac();
              
            if(digitalRead(cambien)==1 && digitalRead(congtac2)==0){
                  digitalWrite(dongco1,LOW);
                  digitalWrite(dongco2,LOW);
                  delay(2000);
                  ve_3();
                  dung++;
            }
            if(dung == 1){
              break;
            }
          }
        }
        if (dulieu == 9){
          while(1){
            if(digitalRead(cambien)==0 && digitalRead(congtac2)==0){ 
                ban_4();
                stop_bot();
                delay(5000); 
            } 
            congtac();        
            if(digitalRead(cambien)==1 && digitalRead(congtac2)==0){
                  digitalWrite(dongco1,LOW);
                  digitalWrite(dongco2,LOW);
                  delay(2000);
                  ve_4();
                  dung++;
            }
            if(dung==1){
              break; 
            }
          }
        }
    }  
    s.write("s");
  if (s.available()>0)
  {
    dulieu=s.read();
      delay(1000);
    Serial.println(dulieu);  
    dung =0;
    if (dulieu == 1){
      forward();
    }
    if (dulieu == 2){
       left();
    }
     if (dulieu == 3){
       stop_bot();    
    }
    if (dulieu == 4){

       right();
    }
     if (dulieu == 5){
       reverse();    
    }
    if (dulieu == 6){
      while(1){
         if(digitalRead(cambien)==0 && digitalRead(congtac2)==0){ 
            ban_1();
            stop_bot();
            delay(5000); 
        } 
        congtac();
          
        if(digitalRead(cambien)==1 && digitalRead(congtac2)==0){
              digitalWrite(dongco1,LOW);
              digitalWrite(dongco2,LOW);
              delay(2000);
              ve_1();
              dung++;
          }
          if(dung ==1){
          break;
        }
      }
    }
    if (dulieu == 7){

      while(1){
          if(digitalRead(cambien)==0 && digitalRead(congtac2)==0){ 
              ban_2();
              stop_bot();
              delay(5000); 
          } 
          congtac();      
          if(digitalRead(cambien)==1 && digitalRead(congtac2)==0){
                digitalWrite(dongco1,LOW);
                digitalWrite(dongco2,LOW);
                delay(2000);
                ve_2();
                dung++;            
          }
          if(dung == 1){
            break; 
          }
      }
    }
    if (dulieu == 8){
      while(1){
         if(digitalRead(cambien)==0 && digitalRead(congtac2)==0){ 
            ban_3();
            stop_bot();
            delay(5000); 
        } 
        congtac();
          
        if(digitalRead(cambien)==1 && digitalRead(congtac2)==0){
              digitalWrite(dongco1,LOW);
              digitalWrite(dongco2,LOW);
              delay(2000);
              ve_3();
              dung++;
        }
        if(dung == 1){
          break;
        }
      }
    }
    if (dulieu == 9){
      while(1){
        if(digitalRead(cambien)==0 && digitalRead(congtac2)==0){ 
            ban_4();
            stop_bot();
            delay(5000); 
        } 
        congtac();        
        if(digitalRead(cambien)==1 && digitalRead(congtac2)==0){
              digitalWrite(dongco1,LOW);
              digitalWrite(dongco2,LOW);
              delay(2000);
              ve_4();
              dung++;
        }
        if(dung==1){
          break; 
        }
      }
    } 
  }
}
