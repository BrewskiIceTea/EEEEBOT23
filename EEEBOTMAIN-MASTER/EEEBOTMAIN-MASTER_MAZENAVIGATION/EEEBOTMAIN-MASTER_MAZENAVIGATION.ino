#include <Wire.h>
#include <stdlib.h>
#include <stdio.h>
#include <LiquidCrystal.h>
#include <Keypad.h>

#define ROW_NUM     4 // four rows
#define COLUMN_NUM  3 // three columns

float duration, distance;  

String message;



char keys[ROW_NUM][COLUMN_NUM] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};

byte arrowUp[8] = {
  0b00000,
  0b00100,
  0b01110,
  0b10101,
  0b00100,
  0b00100,
  0b00100,
  0b00000
};

byte arrowDown[8] = {
  0b00000,
  0b00100,
  0b00100,
  0b00100,
  0b10101,
  0b01110,
  0b00100,
  0b00000
};

byte arrowLeft[8] = {
  0b00000,
  0b00100,
  0b01000,
  0b11111,
  0b01000,
  0b00100,
  0b00000,
  0b00000
};

byte arrowRight[8] = {
  0b00000,
  0b00100,
  0b00010,
  0b11111,
  0b00010,
  0b00100,
  0b00000,
  0b00000
};

byte pin_rows[ROW_NUM] = {15, 2, 0, 4};

//C4, C0, C2
byte pin_column[COLUMN_NUM] = {16, 17, 5};

LiquidCrystal LCD(13, 12, 14, 27, 26, 25);

  
Keypad keypad = Keypad( makeKeymap(keys), pin_rows, pin_column, ROW_NUM, COLUMN_NUM );

const int ledPin = 4;

void setup() {
  LCD.createChar(10, arrowUp);
  LCD.createChar(11, arrowDown);
  LCD.createChar(12, arrowLeft);
  LCD.createChar(13, arrowRight);

  
	Serial.begin(9600);
  Serial.print("STARTING");  

  Wire.begin();
  delay(2000);  
  LCD.begin(16, 2);
  LCD.clear();
  LCD.print("Loading");
  delay(400);
  LCD.print(" .");
  delay(400);
  LCD.print(" .");
  delay(400);
  LCD.print(" .");
  delay(400);
  LCD.clear();
}


void loop() {
  char key = keypad.getKey();

  if (key){
    keyPressed(key);
  }
  
}

void keyPressed(char key){
  Serial.println(key);
  if (key == '*'){
    message = ' ';
    LCD.clear();
  } else if (key == '#') {
    sendMessage();
    LCD.setCursor(0, 1);
    LCD.write("Sending instructions");
    delay(400);
    LCD.print(" .");
    delay(400);
    LCD.print(" .");
    delay(400);
    LCD.print(" .");
    delay(400);
  } else {
    switch(key){
      case '2': // up arrow (forwards)
        message += 'f';
        LCD.write((uint8_t) 10);
        break;
      case '4': // left arrow (turn left)
        message += 'l';
        LCD.write((uint8_t) 12);
        break;
      case '6': //right arrow (TURN RIGHT)
        message += 'r';
        LCD.write((uint8_t) 13);
        break;
      case '8': //down arrow (backwards)
        message += 'b';
        LCD.write((uint8_t) 11);
        break;
    }
  }

}

void sendMessage(){
  Wire.beginTransmission(0x08);
  Wire.write(message.c_str());
  Wire.write("c");
  Wire.endTransmission();
}