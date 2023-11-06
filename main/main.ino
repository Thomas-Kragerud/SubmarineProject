#include "CytronMotorDriver.h"

#define EIN_A 26 // YELLOW
#define EIN_B 25 // WHITE

#define MPIN_1 13 // white
#define MPIN_2 12 // yellow

#define POT 34 //yellow 

#define GEAR_RATIO 30
#define COUNTS_PER_REVOLUTION 16
int POT_MAX = 4095;


float enc_to_rot = 1.0/(GEAR_RATIO*COUNTS_PER_REVOLUTION);
volatile int posi = 0;
volatile int veli = 0;
volatile int prev = 0;

// Configure the motor driver.
CytronMD motor(PWM_DIR, MPIN_1, MPIN_2);


// Interrupt variables 
volatile bool deltaT = false;
hw_timer_t * timer0 = NULL;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTime0() {
  portENTER_CRITICAL_ISR(&timerMux0);
  //portENTER_CRITICAL(&encoderMux);
  veli = prev - posi;
  prev = posi;
  //portEXIT_CRITICAL(&encoderMux);
  portEXIT_CRITICAL_ISR(&timerMux0);
}


void readEncoder(){
  portENTER_CRITICAL_ISR(&encoderMux);
  deltaT = true;
  int b = digitalRead(EIN_B);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
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
  timerAlarmWrite(timer0, 1000000, true); // 1000000 * 1 us = 5 s,
  timerAlarmEnable(timer0);
}

void loop() {
  int pos = 0;
  if(deltaT) {
    portENTER_CRITICAL(&encoderMux);
    deltaT = false;
    pos = posi;
    portEXIT_CRITICAL(&encoderMux);
    float rotations = pos*enc_to_rot;
    float rpm = veli*enc_to_rot*60;

    int potValue = analogRead(POT);
    int pwmValue = map(potValue, 0, POT_MAX, 0, 255);
    motor.setSpeed(pwmValue);

    Serial.print("Pos: ");
    Serial.print(pos);
    Serial.print(", Rotations: ");
    Serial.print(rotations);
    Serial.print(" RPM: ");
    Serial.println(rpm);
  }
}



