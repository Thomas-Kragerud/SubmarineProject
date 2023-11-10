#include <Arduino.h>
#line 1 "/Users/thomas/ArduinoProjects/sub/main/main.ino"
#include "CytronMotorDriver.h"

#define EIN_A 26 // YELLOW
#define EIN_B 25 // WHITE

#define MPIN_1 13 // white
#define MPIN_2 12 // yellow

#define POT 34 //yellow 

#define GEAR_RATIO 30
#define COUNTS_PER_REVOLUTION 16
int POT_MAX = 4095;

// Speed of Accelerometer up to 6.66 kHz
// Buffer up to 9kB data between reads w bult in FIFO
// Embedded temperatur sensor 16-bit
// 2 Qwiic conncetors 
// 2 interrupt pins 

float ENC_TO_ROT = 1.0/(GEAR_RATIO*COUNTS_PER_REVOLUTION);
volatile int position = 0;
volatile int velocity = 0;
volatile int prev_position = 0;

float n_rotations = 0;
float rpm = 0;


enum states{
  STOP,
  MOVE
};
enum states state = STOP;

// Configure the motor driver.
CytronMD motor(PWM_DIR, MPIN_1, MPIN_2);


// Interrupt variables 
volatile bool deltaT = false;
hw_timer_t * timer0 = NULL;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;

#line 54 "/Users/thomas/ArduinoProjects/sub/main/main.ino"
void readEncoder();
#line 69 "/Users/thomas/ArduinoProjects/sub/main/main.ino"
void setup();
#line 82 "/Users/thomas/ArduinoProjects/sub/main/main.ino"
void loop();
#line 114 "/Users/thomas/ArduinoProjects/sub/main/main.ino"
bool CheckForFullEncoderRotation();
#line 125 "/Users/thomas/ArduinoProjects/sub/main/main.ino"
void updateSpeedAndPos();
#line 147 "/Users/thomas/ArduinoProjects/sub/main/main.ino"
void motor_on();
#line 151 "/Users/thomas/ArduinoProjects/sub/main/main.ino"
void motor_off();
#line 46 "/Users/thomas/ArduinoProjects/sub/main/main.ino"
void IRAM_ATTR onTime0() {
  portENTER_CRITICAL_ISR(&timerMux0);
  velocity = prev_position - position;
  prev_position = position;
  portEXIT_CRITICAL_ISR(&timerMux0);
}


void readEncoder(){
  portENTER_CRITICAL_ISR(&encoderMux);
  deltaT = true;
  int b = digitalRead(EIN_B);
  if(b > 0){
    position++;
  }
  else{
    position--;
  }
  portEXIT_CRITICAL_ISR(&encoderMux);
}



void setup() {
  Serial.begin(9600);
  pinMode(EIN_A, INPUT_PULLUP);
  pinMode(EIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EIN_A),readEncoder,RISING);
  //motor.setSpeed(256);

  timer0 = timerBegin(0, 80, true); 
  timerAttachInterrupt(timer0, &onTime0, true); 
  timerAlarmWrite(timer0, 1000000, true); 
  timerAlarmEnable(timer0);
}

void loop() {
  int pos = 0;
  if(deltaT == true) {
    updateSpeedAndPos();
  }
  switch(state){
    case STOP:
      if (CheckForFullEncoderRotation() == true) {
        motor_on();
      }
      // if full encoder rotation 
      // start motor 

      // if start from phone 
      // start motor 
      break;
    case MOVE:
      // if temp too high 
      // stop motor 
      
      // if acceleration to high
      // stop motor 

      // if stop from phone 
      // stop motor 
      break;

  }

}


bool CheckForFullEncoderRotation() {
  if(n_rotations >= 1){
    return true;
  } else {
    return false;
  }
}




void updateSpeedAndPos() {
  int temp = 0;
  portENTER_CRITICAL(&encoderMux);
  deltaT = false;
  temp = position;
  portEXIT_CRITICAL(&encoderMux);
  n_rotations = temp*ENC_TO_ROT;
  rpm = velocity*ENC_TO_ROT*60;

  int potValue = analogRead(POT);
  int pwmValue = map(potValue, 0, POT_MAX, 0, 255);

  motor.setSpeed(pwmValue);
  Serial.print("Pos: ");
  Serial.print(temp);
  Serial.print(", Rotations: ");
  Serial.print(n_rotations);
  Serial.print(" RPM: ");
  Serial.println(rpm);
}


void motor_on() {

}

void motor_off() {

}
