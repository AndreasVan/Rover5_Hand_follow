
// Author: AndreasVan 2015-02-16
// Control of a Rover5 robot
// this code is public domain, enjoy!



#include <PID_marco.h>
#include <PID_v1.h>
#include <NewPing.h>
#include <stdio.h>
#include <string.h>

#define LEFT_MOTOR_PWM 11
#define LEFT_MOTOR_DIRECTION 12
#define LEFT_ENCODER_INTERRUPT 1 // PIN 2!
#define LEFT_ENCODER_CHA 3
#define LEFT_ENCODER_CHB 5

#define RIGHT_MOTOR_PWM 6
#define RIGHT_MOTOR_DIRECTION 7
#define RIGHT_ENCODER_INTERRUPT 0 // PIN 3!
#define RIGHT_ENCODER_CHA 2
#define RIGHT_ENCODER_CHB 4

#define TRIGGER_PIN 9
#define ECHO_PIN 8


const int QEM [16] = {0,-1,1,2,1,0,2,-1,-1,2,0,1,2,1,-1,0}; // Quadrature Encoder Matrix

// left encoder variables
volatile unsigned char left_old, left_new; //Old0, New0;
volatile long left_position = 0; //Position0 = 0;

//right encoder variables
volatile unsigned char right_old, right_new; //Old0, New0;
volatile long right_position = 0; //Position0 = 0;

NewPing sonar(TRIGGER_PIN, ECHO_PIN, 100);

double distanceSP = 20;
double distanceVAL, distancePWM = 0;


//double Kp =   38;
//double Ki = 19;
//double Kd = 0;

double Kp =   22.6;
double Ki = 6.7;
double Kd = 1;

//double Kd = 1;
//double Kd = 0.7;
int sampleTime = 15;
PID motorPID(&distanceVAL, &distancePWM, &distanceSP,Kp,Ki,Kd, REVERSE);
char buf[50];

void setup()
{
  pinMode(LEFT_MOTOR_DIRECTION,OUTPUT);
  pinMode(LEFT_MOTOR_PWM,OUTPUT);
  pinMode(LEFT_ENCODER_CHA, INPUT);
  pinMode(LEFT_ENCODER_CHB, INPUT);

  
  pinMode(RIGHT_MOTOR_DIRECTION,OUTPUT);
  pinMode(RIGHT_MOTOR_PWM,OUTPUT);
  
  digitalWrite(LEFT_MOTOR_DIRECTION,LOW);
  analogWrite(LEFT_MOTOR_PWM,0);
  digitalWrite(RIGHT_MOTOR_DIRECTION,LOW);
  analogWrite(RIGHT_MOTOR_PWM,0);
  
//  attachInterrupt(LEFT_ENCODER_INTERRUPT, left_encoder_interrupt, CHANGE); // interrupt on pin 2;
//  attachInterrupt(RIGHT_ENCODER_INTERRUPT, right_encoder_interrupt, CHANGE); // interrupt on pin 2;
  
  motorPID.SetMode(AUTOMATIC);
  motorPID.SetSampleTime(sampleTime);
  motorPID.SetOutputLimits(-150,150);
  Serial.begin(115200);
  //Serial.begin(9600);
}

/* Loop and print the time every second */
unsigned long timeNow;
void loop()
{
  int uS = sonar.ping();  
  distanceVAL = ((double)uS)/US_ROUNDTRIP_CM;
  
  if (distanceVAL != 0 && distanceVAL < 100) {
    if (motorPID.GetMode() == MANUAL) {
        motorPID.SetMode(AUTOMATIC);
    }
    motorPID.Compute();
    if (distancePWM >= 0) {
        digitalWrite(LEFT_MOTOR_DIRECTION,HIGH);
        analogWrite(LEFT_MOTOR_PWM,distancePWM);
        digitalWrite(RIGHT_MOTOR_DIRECTION,HIGH);
        analogWrite(RIGHT_MOTOR_PWM,distancePWM);
    }
    else {
        digitalWrite(LEFT_MOTOR_DIRECTION,LOW);
        analogWrite(LEFT_MOTOR_PWM,-distancePWM);
        digitalWrite(RIGHT_MOTOR_DIRECTION,LOW);
        analogWrite(RIGHT_MOTOR_PWM,-distancePWM);
    }
  }
  else {
    if(motorPID.GetMode() == AUTOMATIC) {
        motorPID.SetMode(MANUAL);
    }
    digitalWrite(LEFT_MOTOR_DIRECTION,LOW);
    analogWrite(LEFT_MOTOR_PWM,0);
    digitalWrite(RIGHT_MOTOR_DIRECTION,LOW);
    analogWrite(RIGHT_MOTOR_PWM,0);
  }
    timeNow = millis();

    sprintf(buf, "%lu, %ld, %ld, %d \n", timeNow,  (long int)(1000*distanceSP), (long int)(1000*distanceVAL), (int)distancePWM);
    Serial.print(buf);  

  //delay(2);
}
      

// 

void left_encoder_interrupt(){
    left_old = left_new;
    left_new =  digitalRead (LEFT_ENCODER_CHA) * 2 + digitalRead (LEFT_ENCODER_CHB);           // Convert binary input to decimal value
    left_position += QEM [left_old * 4 + left_new];
}

void right_encoder_interrupt(){
    right_old = right_new;
    right_new =  digitalRead (RIGHT_ENCODER_CHA) * 2 + digitalRead (RIGHT_ENCODER_CHB);           // Convert binary input to decimal value
    right_position += QEM [right_old * 4 + right_new];
}

