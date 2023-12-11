#include <ESP32Encoder.h>
#include <stdio.h>  
#include <Wire.h>

#define enA 33  //EnableA command line - should be a PWM pin
#define enB 25   //EnableB command line - should be a PWM pin

//name the motor control pins - replace the CHANGEME with your pin number, digital pins do not need the 'D' prefix whereas analogue pins need the 'A' prefix
#define INa 26  //Channel A direction 
#define INb 27  //Channel A direction 
#define INc 14  //Channel B direction 
#define INd 12  //Channel B direction 

byte speedSetting = 170;   //initial speed = 0

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
float newPosition;
//what step of parking is completed
int parked = 0;
//I2C Message
char message;
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

  //Setup I2C with other ESP32
  Wire.begin(0x08);             // join i2c bus with address 8
  Wire.onReceive(receiveEvent); // create a receive event
  
  //initialise Serial Communication
  Serial.begin(9600);
  Serial.println("ESP32 Running");  //sanity Check

  encoder1.attachHalfQuad ( 36, 39 );
  encoder1.setCount ( 0 );
  encoder2.attachHalfQuad ( 34, 35 );
  encoder2.setCount ( 0 );
  goBackwards();
  //setup the steering angle
  straight();
  
  delay(3000);
  motors(speedSetting, speedSetting);  //make a call to the 'motors' function and provide it with a value for each of the 2 motors - can be different for each motor - using same value here for expedience
  Serial.print("Starting");

}


void loop() {
  // wiggle(3);
  //inputSteeringAngle();
  edifferential(0);

  edifferential(0.5);
  // goForwards();
  // eventAct();
  // Serial.print("Motor speeds: ");
  // Serial.println(speedSetting);
  checkEncoders();
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

// input steering angle manually through the serial monitor.
void inputSteeringAngle(){
  steeringAngle = Serial.parseInt();
  if (steeringAngle >1){
    changeSteeringAngle(steeringAngle);
  }
}
//
void eventAct(){
  switch(message)
  {
    case 'S':
      if (parked == 0){
      park();
      }
      else {
        stopMotors();
        changeSteeringAngle(127);
        while (true){

        }
      }
      break;
  }

}
void receiveEvent(int howMany){
  Serial.println("\nMessage Recieved");
  message = ' ';
  while (Wire.available()){  // loop whilst bus is busy
    message = Wire.read();     // receive data byte by byte 
  }
  Serial.println(message);    // write string to serial monitor
  delay(100);
} 

void checkEncoders(){
  //ENCODER CODE
  newPosition = ((encoder1.getCount() / 2)+(encoder2.getCount() / 2))/2 ;
  Serial.println(newPosition);
}

int wiggle(int wiggleno){
  for (int i=0; i < wiggleno; i++){
    changeSteeringAngle(0);
    delay(100);
    changeSteeringAngle(255);
    delay(100);
    Serial.println(i);
    Serial.print("Wiggleno: ");
    Serial.println(wiggleno);
  }


}
void straight(){
  changeSteeringAngle(126);
}

void park(){
  delay(400);
  goForwards();
  goDistance(20);
  stopMotors();
  delay(200);
  changeSteeringAngle(0);
  edifferential(0);
  goBackwards();
  goDistance(100);
  edifferential(0.5);
  changeSteeringAngle(127);
  message = 'g';
  parked = 1;
}

void goDistance(float distance){
  checkEncoders();
  float targetPosition = newPosition +distance;
  while (newPosition < targetPosition){
    checkEncoders();
    // Serial.print("Target Position: ");
    // Serial.print(targetPosition);
    // Serial.print("Current Position: %f");
    // Serial.print(newPosition);
    float targetPosition = newPosition + distance;
    delay(50);
  }
}

void edifferential(float angle){ // takes value between 0-1 and translates that to ratio of left motor speed to right motor speed.
  float leftspeed, rightspeed;
  leftspeed = angle * speedSetting;
  rightspeed = (1 - angle)*speedSetting ;
  motors(leftspeed, rightspeed);

}