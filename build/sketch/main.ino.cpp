#include <Arduino.h>
#line 1 "/Users/thomas/ArduinoProjects/sub/main/main.ino"
#define BLYNK_TEMPLATE_ID "TMPL2CNVjjOoK"
#define BLYNK_TEMPLATE_NAME "LED ESP32"
#define BLYNK_FIRMWARE_VERSION        "0.1.0"



#include "CytronMotorDriver.h"
#include "SparkFunLSM6DSO.h"
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

#include "BlynkEdgent.h"
// Authentication for the Blynk app 



#define BLYNK_PRINT Serial
#define APP_DEBUG
#define EIN_A 26 // YELLOW  Encoder
#define EIN_B 25 // WHITE   Encoder 

#define MPIN_1 13 // white  Motor
#define MPIN_2 12 // yellow Motor

#define I2C_SDA 21
#define I2C_SCL 22

//#define APP_MOTOR_SWITCH 13

#define POT 34 //yellow potentiometer 




#define GEAR_RATIO 30
#define COUNTS_PER_REVOLUTION 16
int POT_MAX = 4095;
float ENC_TO_ROT = 1.0/(GEAR_RATIO*COUNTS_PER_REVOLUTION);
int MOTOR_SPEED = 128;
float CUT_OFF_TEMPERATURE = 40;


// ************
// Setup for IMU 
// Speed of Accelerometer up to 6.66 kHz
// Buffer up to 9kB data between reads w bult in FIFO
// Embedded temperatur sensor 16-bit
// 2 Qwiic conncetors 
// 2 interrupt pins 
LSM6DSO myIMU; 
int imu_data;


// ************
// Setup for Encoder
volatile int position = 0;
volatile int velocity = 0;
volatile int prev_position = 0;
float n_rotations = 0;
float rpm = 0;
// Configure the motor driver.
CytronMD motor(PWM_DIR, MPIN_1, MPIN_2);

enum states{
  STOP,
  MOVE
};
enum states state = STOP;


bool APP_MOTOR_SWITCH = false;

// Interrupt variables 
volatile bool deltaT = false;
hw_timer_t * timer0 = NULL;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;

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


// V0 is a datastream used to transfer and store LED switch state.
// Evey time you use the LED switch in the app, this function
// will listen and update the state on device
BLYNK_WRITE(V0)
{
  // Local variable `value` stores the incoming Motor switch state (1 or 0)
  // Based on this value, the physical Motor on the board will be on or off
  int value = param.asInt();

  if (value == 1) {
    APP_MOTOR_SWITCH = true;
    //digitalWrite(APP_MOTOR_SWITCH, HIGH);
    //Serial.print("value =");
    //Serial.println(value);
  } else {
    APP_MOTOR_SWITCH = false;
    //digitalWrite(APP_MOTOR_SWITCH, LOW);
    //Serial.print("value = ");
    //Serial.println(value);
  }
}


void setup() {
  Serial.begin(9600);
  delay(500);

  pinMode(EIN_A, INPUT_PULLUP);
  pinMode(EIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EIN_A),readEncoder,RISING);

  timer0 = timerBegin(0, 80, true); 
  timerAttachInterrupt(timer0, &onTime0, true); 
  timerAlarmWrite(timer0, 1000000, true); 
  timerAlarmEnable(timer0);


  // Setup for IMU
  Wire.begin(I2C_SDA, I2C_SCL);
  if(myIMU.begin()){
    Serial.println("IMU Ready");
  } else {
    Serial.println("Could not connect to IMU.");
  }
  if( myIMU.initialize(SOFT_INT_SETTINGS)){
    Serial.println("Loaded Settings.");
  }

  BlynkEdgent.begin();
}



void loop() {
  delay(10);
  Serial.println("Hey");
  BlynkEdgent.run();
  int pos = 0;
  if(deltaT == true) {
    updateSpeedAndPos();
  }
  switch(state){
    case STOP:
      // if full encoder rotation 
      if (CheckForFullEncoderRotation() == true) {
        motor_on();
      }

      // if start from phone
      if (CheckForMotorOnOffApp() == true) {
        motor_on();
      }
      break;

    case MOVE:
      // if temp too high 
      if (CheckForCriticalTemperature() == true) {
        motor_off();
        //Possibly turn off everything??
      }
      
      // if acceleration to high
      // stop motor
      
      // if stop from phone 
      // stop motor 
      if (CheckForMotorOnOffApp() == false) {
        motor_off();
      }
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

bool CheckForCriticalTemperature() {
  float temperature = 0;
  imu_data = myIMU.listenDataReady();
  if(imu_data == TEMP_DATA_READY) {
    temperature = myIMU.readTempC();
    if(temperature >= CUT_OFF_TEMPERATURE){
      return true;
    }
  } else {
    return false;
  }
}

bool CheckForMotorOnOffApp() {
  if(APP_MOTOR_SWITCH == true) {
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

  //motor.setSpeed(pwmValue);
  Serial.print("Pos: ");
  Serial.print(temp);
  Serial.print(", Rotations: ");
  Serial.print(n_rotations);
  Serial.print(" RPM: ");
  Serial.println(rpm);
}


void motor_on() {
  motor.setSpeed(MOTOR_SPEED);
}

void motor_off() {
  motor.setSpeed(0);
}
