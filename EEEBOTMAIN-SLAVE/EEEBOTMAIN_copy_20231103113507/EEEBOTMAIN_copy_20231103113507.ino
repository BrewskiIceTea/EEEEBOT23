#include <HTTP_Method.h>
#include <Uri.h>
#include <WebServer.h>

#include <AsyncUDP.h>

#include <ESP32Encoder.h>
#include <stdio.h>
#include <Ticker.h>
//********************************************************//
//*  University of Nottingham                            *//
//*  Department of Electrical and Electronic Engineering *//
//*  Alex Ottway & Nat Dacombe                           *//
//*  UoN EEEBot 2023                                     *//
//*  Motor & Servo Basic Test Code                       *//
//*                                                      *//
//*  ASSUMPTION: Channel A is LEFT, Channel B is RIGHT   *//
//********************************************************//
// ASSUMPTION: Channel A is LEFT, Channel B is RIGHT

// use this code to correctly assign the four pins to move the car forwards and backwards
// you first need to change the pin numbers for the four motor input 'IN' pins and two enable 'en' pins below and then 
// decide which go HIGH and LOW in each of the movements, stopMotors has been done for you
// ** marks where you need to insert the pin number or state

// feel free to modify this code to test existing or new functions

#define enA 33  //EnableA command line - should be a PWM pin
#define enB 25   //EnableB command line - should be a PWM pin

//name the motor control pins - replace the CHANGEME with your pin number, digital pins do not need the 'D' prefix whereas analogue pins need the 'A' prefix
#define INa 26  //Channel A direction 
#define INb 27  //Channel A direction 
#define INc 14  //Channel B direction 
#define INd 12  //Channel B direction 

byte speedSetting = 255;   //initial speed = 0
byte speedRampFlag = 1;  //define a direction controller for the loop
byte changeDirection = 0;

// setting PWM properties
const int freq = 2000;
const int servoFrequency = 50;
const int ledChannela = 0;
const int ledChannelb = 1;
const int servoChannel = 2;
const int resolution = 8;
const int servoResolution = 12;

//Servo Value recordings
//Middle = 310
//Max right = 520
//Max Left = 100
int servoPin = 13;
float steeringAngle;  // variable to store the servo position

ESP32Encoder encoder1;
ESP32Encoder encoder2;

// timer and flag for example, not needed for encoders
unsigned long encoder2lastToggled;
bool encoder2Paused = false;

//Encoder position
long newPosition;

//timer
Ticker timer1();


void setup() {
  // put your setup code here, to run once:

  pinMode(INa, OUTPUT);
  pinMode(INb, OUTPUT);
  pinMode(INc, OUTPUT);
  pinMode(INd, OUTPUT);
  // pinMode(enA, OUTPUT);
  // pinMode(enB, OUTPUT);  //if defining some pins as PWM, DON'T set them as OUTPUT!

  // configure LED PWM functionalitites
  ledcSetup(ledChannela, freq, resolution);
  ledcSetup(ledChannelb, freq, resolution);
  ledcSetup(servoChannel, servoFrequency, servoResolution); //servo setup on PWM2, 50Hz, 12-bit (0-4096)

  //attach the channel to the GPIO to be controlled
 
  ledcAttachPin(enA, ledChannela);
  ledcAttachPin(enB, ledChannelb);
  ledcAttachPin(servoPin, servoChannel);

  //initialise Serial Communication
  Serial.begin(9600);
  Serial.println("ESP32 Running");  //sanity Check

  encoder1.attachHalfQuad ( 36, 39 );
  encoder1.setCount ( 0 );
  encoder2.attachHalfQuad ( 34, 35 );
  encoder2.setCount ( 0 );
  //setup the steering angle
  wiggle();
//  timer1.start();

}


void loop() {
//  timer1.update();
  
  //inputSteeringAngle();
  changeSteeringAngle(123.8);
  motors(speedSetting, speedSetting);  //make a call to the 'motors' function and provide it with a value for each of the 2 motors - can be different for each motor - using same value here for expedience
  goForwards();
  // Serial.print("Motor speeds: ");
  // Serial.println(speedSetting);
  checkEncoders();
  if (newPosition>1265){
    stopMotors();
    delay(1000);
    wiggle(); //victory wiggle :)
    
    delay(10000);
  }

}

void motors(int leftSpeed, int rightSpeed) {
  //set individual motor speed
  //direction is set separately
  ledcWrite(ledChannela, leftSpeed);
  ledcWrite(ledChannelb, rightSpeed);
  delay(25);
}

void goForwards() {
  digitalWrite(INa, HIGH);
  digitalWrite(INb, LOW);
  digitalWrite(INc, HIGH);
  digitalWrite(INd, LOW);
}

void goBackwards() {
  digitalWrite(INa, LOW);
  digitalWrite(INb, HIGH);
  digitalWrite(INc, LOW);
  digitalWrite(INd, HIGH);
}

void goClockwise() {
  digitalWrite(INa, HIGH);
  digitalWrite(INb, HIGH);
  digitalWrite(INc, LOW);
  digitalWrite(INd, LOW);
}

void goAntiClockwise() {
  digitalWrite(INa, LOW);
  digitalWrite(INb, LOW);
  digitalWrite(INc, HIGH);
  digitalWrite(INd, HIGH);
}

void stopMotors() {
  digitalWrite(INa, LOW);
  digitalWrite(INb, LOW);
  digitalWrite(INc, LOW);
  digitalWrite(INd, LOW);
}

//encodes 0-255 to 100-520 range that was found for the servo.
void changeSteeringAngle(float angle){
  steeringAngle = (angle * 1.65) + 100;
  ledcWrite(servoChannel, steeringAngle);
    Serial.print("Steering Angle: ");
    Serial.println(abs(steeringAngle));
}
//input steering angle manually through the serial monitor.
void inputSteeringAngle(){
  steeringAngle = Serial.parseInt();
  if (steeringAngle >1){
    changeSteeringAngle(steeringAngle);
  }
}

void checkEncoders(){
  //ENCODER CODE
  newPosition = ((encoder1.getCount() / 2)+(encoder2.getCount() / 2))/2 ;
  Serial.println(newPosition);
}
void wiggle(){
  changeSteeringAngle(0);
  delay(100);
  changeSteeringAngle(255);
  delay(100);
  changeSteeringAngle(0);
  delay(100);
  changeSteeringAngle(255);
  delay(100);
  changeSteeringAngle(0);
  delay(100);
  changeSteeringAngle(255);
  delay(100);
}