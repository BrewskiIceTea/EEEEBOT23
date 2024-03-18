#include <ESP32Encoder.h>
#include <stdio.h> 
#include <stdlib.h> 
#include <Wire.h>

#define enA 33  //EnableA command line - should be a PWM pin
#define enB 25   //EnableB command line - should be a PWM pin

//name the motor control pins - replace the CHANGEME with your pin number, digital pins do not need the 'D' prefix whereas analogue pins need the 'A' prefix
#define INa 26  //Channel A direction 
#define INb 27  //Channel A direction 
#define INc 14  //Channel B direction 
#define INd 12  //Channel B direction 

byte speedSetting = 200;   //initial speed = 0

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
//Total encoder count cahce
float enc1, enc2, oldenc1, oldenc2, encdiff;
//what step of parking is completed
int parked = 0;
//I2C Message
String message;
char encoderBuffer[7];

//encodes 0-255 to 100-520 range that was found for the servo.
void changeSteeringAngle(float angle){
  steeringAngle = (angle * 1.65) + 100;
  float constrainted = constrain(steeringAngle, 200, 400);
  ledcWrite(servoChannel, constrainted);
    // Serial.print("Steering Angle: ");
    // Serial.println(abs(steeringAngle));
}

void wiggle(int wiggleno){
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


void checkEncoders(){
  //ENCODER CODE
  newPosition = ((encoder1.getCount() / 2)+(encoder2.getCount() / 2))/2 ;
  // Serial.println(newPosition);
}

void realtimeenc(){
  //ENCODER CODE
  enc1 = (encoder1.getCount()/2);
  enc2 = (encoder2.getCount()/2);
}

void cachecurrentenc(){
  //ENCODER CODE
  realtimeenc();
  oldenc1 = enc1;
  oldenc2 = enc2;

}
void checkEncoder1(){
  //ENCODER CODE
  newPosition = (encoder1.getCount() / 2);
  Serial.println(newPosition);
}

void checkEncoder2(){
  //ENCODER CODE
  newPosition = (encoder2.getCount() / 2);
  Serial.println(newPosition);
}

int checkEncoderdevi(char direction){
  //ENCODER CODE
  realtimeenc();
  //making sure difference is calculated from most recent reading
  encdiff = ((enc1 - oldenc1) - (enc2 - oldenc2));
  //ensure difference is positive
  if (encdiff < 0){
    encdiff = encdiff * (-1);
  }
  Serial.print("encdiff");
  Serial.println(encdiff);
  //difference between two encoders turn 90 degrees
  if (encdiff < 14 && encdiff >= 0 && direction == 'L'){
    return 1;
  }
  else if(encdiff<22 && encdiff>=0 && direction =='R'){
    return 1;
  }
  else {
    return 0;
  }
  realtimeenc();
}


// Remember that the checkEncoders is an average between the two, so distance can be different give a non centred eDifferential.
void goDistance(float distance){ 
  checkEncoders();
  float tempPosition = newPosition;
  while (abs(newPosition - tempPosition) < distance){
    checkEncoders();
    // Serial.print("\nTarget Position: ");
    // Serial.println(tempPosition);
    // Serial.print("\nCurrent Position: ");
    // Serial.println(newPosition);
    // Serial.print("Difference: ");
    // Serial.println(abs(newPosition - tempPosition));
    // Serial.println(distance);
    delay(50);
  }
}

void edifferential(float angle){ // Takes value between 0-1 and translates that to ratio of left motor speed to right motor speed.
  float leftspeed, rightspeed;
  if (angle < 0.5){
    leftspeed = angle * speedSetting * 2;
    rightspeed = speedSetting;
  } else {
    rightspeed = (1 - angle) * speedSetting * 2;
    leftspeed = speedSetting; 
  }
  motors(leftspeed, rightspeed*0.95);
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

void stopMotors() {
  digitalWrite(INa, LOW);
  digitalWrite(INb, LOW);
  digitalWrite(INc, LOW);
  digitalWrite(INd, LOW);
}

void goClockwise() {
  digitalWrite(INa, HIGH);
  digitalWrite(INb, LOW);
  digitalWrite(INc, LOW);
  digitalWrite(INd, HIGH);
}

void goCounterClockwise() {
  digitalWrite(INa, LOW);
  digitalWrite(INb, HIGH);
  digitalWrite(INc, HIGH);
  digitalWrite(INd, LOW);
}

// input steering angle manually through the serial monitor.
void inputSteeringAngle(){
  steeringAngle = Serial.parseInt();
  if (steeringAngle >1){
    changeSteeringAngle(steeringAngle);
  }
}

// events to be made upon different requests of the Master ESP32
void eventAct(char function){
  Serial.println(" ");
  Serial.println(function);
  realtimeenc();
  
  
  switch(function)
  { 
    case 'w':
      realtimeenc();
      Serial.print("WIGGLING");
      delay(50);
      wiggle(1);
      delay(50);
      message = ' ';
      break;
    case 'f':
      realtimeenc();
      goForwards();
      goDistance(60);
      break;
    case 'b':
      realtimeenc();
      goBackwards();
      goDistance(60);
      break;
    case 'r':
      realtimeenc();
      changeSteeringAngle(230);
      delay(50);
      cachecurrentenc();      
      while (checkEncoderdevi('R') != 0){
        realtimeenc();
        delay(50);
        Serial.println("delaying ");
        Serial.println(enc1);
        Serial.println(enc2);
        realtimeenc();
      }
      straight();
      break;
    case 'l':
      realtimeenc();
      //Serial.println(encdiff);
      changeSteeringAngle(50);
      delay(50);
      cachecurrentenc();      
      while (checkEncoderdevi('L') != 0){
        realtimeenc();
        delay(50);
        Serial.println("delaying ");
        Serial.println(enc1);
        Serial.println(enc2);
        realtimeenc();
      }
      straight();
      break;

    default:
      stopMotors();
      changeSteeringAngle(127);
      break;
    }
}

void receiveEvent(int numBytes){
  Serial.println("\nMessage Recieved");
  message = ' ';
  while (Wire.available()) {
    char c = Wire.read();
    message += c;
  } 
  Serial.println(message);    // write string to serial monitor
  delay(100);
  execute(message);
}

void execute(String commands){
  for (char c : commands){
    eventAct(c);
  }
  stopMotors();
  changeSteeringAngle(127);
}

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

  //setup the steering angle
  straight();
  delay(100);
  wiggle(3);
  delay(1500);
  motors(speedSetting, speedSetting-20);  //make a call to the 'motors' function and provide it with a value for each of the 2 motors - can be different for each motor - using same value here for expedience
  Serial.println("Starting...");
  straight();
}



void loop() {
  realtimeenc();
}
