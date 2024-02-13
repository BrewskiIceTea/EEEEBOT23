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
char encoderBuffer[7];

int targetAngle;

//encodes 0-180 to 100-520 range that was found for the servo.
void changeSteeringAngle(float angle){
  steeringAngle = (angle * 2.33) + 100;
  ledcWrite(servoChannel, steeringAngle);
    // Serial.print("Steering Angle: ");
    // Serial.println(abs(steeringAngle));
}

void wiggle(int wiggleno){
  for (int i=0; i < wiggleno; i++){
    changeSteeringAngle(45);
    delay(300);
    changeSteeringAngle(135);
    delay(300);
  }
}

void straight(){
  changeSteeringAngle(90);
}


void checkEncoders(){
  //ENCODER CODE
  newPosition = ((encoder1.getCount() / 2)+(encoder2.getCount() / 2))/2 ;
  // Serial.println(newPosition);
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



// Remember that the checkEncoders is an average between the two, so distance can be different give a non centred eDifferential.
void goDistance(float distance){ 
  checkEncoders();
  float tempPosition = newPosition;
  while (abs(newPosition - tempPosition) < distance){
    checkEncoders();
    Serial.print("\nTarget Position: ");
    Serial.print(tempPosition);
    Serial.print("\nCurrent Position: ");
    Serial.print(newPosition);
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
  motors(leftspeed, rightspeed);
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

// input steering angle manually through the serial monitor.
void inputSteeringAngle(){
  steeringAngle = Serial.parseInt();
  if (steeringAngle >1){
    changeSteeringAngle(steeringAngle);
  }
}

void receiveEvent(){
  int16_t a = 0;
  int16_t b = 0;
  uint8_t bytesReceived = Wire.requestFrom(0x08, 4);  // 4 indicates the number of bytes that are expected
  uint8_t a16_9 = Wire.read();  // receive bits 16 to 9 of a (one byte)
  uint8_t a8_1 = Wire.read();   // receive bits 8 to 1 of a (one byte)
  targetAngle = (a16_9 << 8) | a8_1; // combine the two bytes into a 16 bit number
  Serial.println(targetAngle);
} 

void requestEvent() { // Writing Encoder values to the master ESP32.
  Serial.println("request evented");
  checkEncoders();
  dtostrf(newPosition, 7, 2, encoderBuffer);
  Wire.write(encoderBuffer);
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
  // Wire.onReceive(receiveEvent); // create a receive event
  // Wire.onRequest(requestEvent); // create a request event

  //initialise Serial Communication
  Serial.begin(9600);
  Serial.println("ESP32 Running");  //sanity Check
  encoder1.attachHalfQuad ( 36, 39 );
  encoder1.setCount ( 0 );
  encoder2.attachHalfQuad ( 34, 35 );
  encoder2.setCount ( 0 );
  goForwards();
  //setup the steering angle
  delay(100);
  wiggle(3);
  delay(1500);
  motors(speedSetting, speedSetting);  //make a call to the 'motors' function and provide it with a value for each of the 2 motors - can be different for each motor - using same value here for expedience
  Serial.println("Starting...");
  straight();
  delay(1000);
}

/*void bomb(){
  speedSetting = 20;
  for (int i = 500; i)
}*/

void loop() {
  checkEncoders();
  changeSteeringAngle(targetAngle);
  delay(100);
}





