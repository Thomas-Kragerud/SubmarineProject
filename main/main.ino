#include "CytronMotorDriver.h"  // Library for the Cytron motor driver card 
#include "SparkFunLSM6DSO.h"    // Library for the LSM6DS0 IMU 
#include <Wire.h>               // Wire library for I2C communicaition with the IMU 
#include <Ticker.h>             // Ticker library for non-blocking countdown timer

// Pin Definitions 
#define EIN_A 26 // YELLOW Encoder A pin 
#define EIN_B 25 // WHITE Encoder B pin 

#define MPIN_1 13 // WHITE Motor Controll pin 1
#define MPIN_2 12 // YELLOW Motor Controll pin 2

#define I2C_SDA 23 // I2C SDA for IMU 
#define I2C_SCL 22 // I2C SCL for IMU 

// LED Charlieplexing Control Pins
#define L1 15 
#define L2 32 
#define L3 14

// Potentiometer Pin for Motor Speed Control
#define POT 34 // YELLOW potentiometer

// Constants Definitions
#define GEAR_RATIO 30             // Gear ratio of the motor
#define COUNTS_PER_REVOLUTION 16  // Encoder counts per revolution 

float ENC_TO_ROT = 1.0 / (GEAR_RATIO * COUNTS_PER_REVOLUTION); // Encoder count to rotation conversion
float CUT_OFF_TEMPERATURE = 30.0; // Temperature threshold in degrees Celsius
float CUT_OFF_GYRO = 300;         // Gyro threshold for movement detection

// Sensor and Control Variables
float temperature = 0.0;
float GyroX = 0.0, GyroY = 0.0, GyroZ = 0.0;
int POT_MAX = 4095; // Maximum value for 12-bit ADC
int potValue = 0;
int pwmValue = 0;
int pos = 0;        // Non-critical section position

// IMU Setup
LSM6DSO myIMU; // IMU object 
int imu_data;  // Variable to store IMU data 

// Encoder Setup
volatile int position = 0;
volatile int velocity = 0;
volatile int prev_position = 0;
float n_rotations = 0;
float rpm = 0;

CytronMD motor(PWM_DIR, MPIN_1, MPIN_2); // Motor driver object

// State Machine Enums
enum states { STOP, MOVE, OVERHEATED };
enum states state = STOP;

enum LEDS { YELLOW, RED1, GREEN, BLUE, WHITE, RED2 };

// Interrupt Management Variables
volatile bool encoderUpdated = false;
hw_timer_t *timer0 = NULL;
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;
Ticker resetPositionTimer; // ticker object for reseting the position 


// Interrupt Service Routine for Timer
// Every second we update the velocity
void IRAM_ATTR onTime0(){
  portENTER_CRITICAL_ISR(&encoderMux);
  velocity = prev_position - position;
  prev_position = position;
  portEXIT_CRITICAL_ISR(&encoderMux);
}

// Encoder Reading Function
// Works by observing changes to the magnetic field created by a magnet attached to the motor shaft
// If A is high before B, we are moving clockwise
// If B is high before A, we are moving counter-clockwise
// The function is called by an edge-triggered interrupt on A (when A is high), then by checking B we can determine the direction
void readEncoder() {
  portENTER_CRITICAL_ISR(&encoderMux);
  encoderUpdated = true;
  int b = digitalRead(EIN_B);
  if (b == HIGH) {
    posi++;  // If B is HIGH when A rises, increment position (clockwise rotation)
    } else {
      posi--;  // If B is LOW when A rises, decrement position (counterclockwise rotation)
    }
  portEXIT_CRITICAL_ISR(&encoderMux);
}


// Setup Function
void setup(){
  Serial.begin(115200);
  set_LED(BLUE); // Set LED to blue, indicating that we are booting up correctly

  // Initialize Encoder Pins and Interrupt
  pinMode(EIN_A, INPUT_PULLUP);
  pinMode(EIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EIN_A), readEncoder, RISING);

  // Timer Setup for Velocity Calculation
  // Set the 0th HW timer to count upwards
  timer0 = timerBegin(0, 80, true);              // Divide by prescaler 80 to get 1MHz tick frequency
  timerAttachInterrupt(timer0, &onTime0, true);  // Attach onTime0 function to timer
  timerAlarmWrite(timer0, 1000000, true);        // Alarm triggers once every second, resets after being triggered
  timerAlarmEnable(timer0);                      // Enable alarm

  // Setup for IMU
  Wire.begin(I2C_SDA, I2C_SCL);
  if (myIMU.begin()){
    Serial.println("IMU Ready");
  } else {
    Serial.println("Could not connect to IMU.");
  }
  if (myIMU.initialize(SOFT_INT_SETTINGS)) {
    Serial.println("Loaded Settings.");
  }
}

// ************************ Main Loop ************************
void loop() {
  delay(10); // Preventing rapid state changes
  if (encoderUpdated == true) {
    updateSpeedAndPos();
  }

  switch (state) {
  case STOP:
    if (CheckForCriticalTemperature() == true) {                                       // Event Checker
      motor_off();                                                                     // Service Function
      Serial.println("Maximum temperature exceeded. Switching to state = OVERHEATED"); // Print for debugging
      set_LED(RED1); 
      state = OVERHEATED;
    }
    if (CheckForEncoderRotation() == true) {                              // Event Checker
      motor_on();                                                         // Service Function
      Serial.println("Manual start detected. Switching to state = MOVE"); // Print for debugging
      set_LED(GREEN); 
      state = MOVE;
    }
    
    break;

  case MOVE:
    if (CheckForCriticalTemperature() == true) {                                 // Event Checker
      motor_off();                                                               // Service Function
      Serial.println("Maximum temperature exceeded. Switching to state = OVERHEATED"); //Print for debugging
      set_LED(RED1); 
      state = OVERHEATED;
    }
    if (CheckForCriticalGyro() == true) {                                 // Event Checker
      motor_off();                                                        // Service Function
      Serial.println("Maximum gyro exceeded. Switching to state = STOP"); // Print for debugging
      set_LED(YELLOW); 
      state = STOP;
    }
    break;

  case OVERHEATED:
    if (CheckForCriticalTemperature() == false) {                              // Event Checker
      Serial.println("Temperature back to normal. Switching to state = STOP"); //Print for debugging
      set_LED(YELLOW); // Yellow
      state = STOP;
    }
    break;
  }
}

// ************************ Event Checkers (Functions) ************************
bool CheckForEncoderRotation() {
  return abs(position) > 70;
}

bool CheckForCriticalTemperature() {
  temperature = myIMU.readTempC();
  return temperature >= CUT_OFF_TEMPERATURE;
}

bool CheckForCriticalGyro() {
  GyroX = myIMU.readFloatGyroX();
  GyroY = myIMU.readFloatGyroY();
  GyroZ = myIMU.readFloatGyroZ();
  return (abs(GyroX) >= CUT_OFF_GYRO) || (abs(GyroY) >= CUT_OFF_GYRO) || (abs(GyroZ) >= CUT_OFF_GYRO);
}

// ************************ Service Functions ************************
void motor_on() {
  potValue = analogRead(POT);
  pwmValue = map(potValue, 0, POT_MAX, 0, 255);
  motor.setSpeed(pwmValue);
  Serial.print("Motor ON. Desired Motor PWM: ");
  Serial.println(pwmValue);
}

void motor_off() {
  motor.setSpeed(0);
  resetPositionTimer.once(1, reset_position); // calls the reset position function after 1 sec 
  Serial.println("Motor OFF");
}

void reset_position() {
  position = 0;
  prev_position = 0;
  velocity = 0;
  rpm = 0;
}


// Update rpm, pos and rotation count 
void updateSpeedAndPos() {
  int localPosition;
  int localVelocity;

  portENTER_CRITICAL(&encoderMux);
  encoderUpdated = false;
  localPosition = position;
  localVelocity = velocity;
  portEXIT_CRITICAL(&encoderMux);

  pos = localPosition;
  n_rotations = pos * ENC_TO_ROT;
  rpm = localVelocity * ENC_TO_ROT * 60;
}


// ************************ Functions for LEDs (control charlieplexing) ************************
// Implementing charlieplexing, a technique uest to control multiple LEDs with fewer I/O pins 
// exploiting the tristate capabilities of the microcontroller, this means that pins can be both on off and "disconnected"

// Set LED pin to high (sourcing current)
void set_H(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
}

// Set LED pin to low (sinking current)
void set_L(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

// Set LED pin to Z (high-impedans)
void set_Z(int pin) {
  pinMode(pin, INPUT);    // Put pin in high-impedans state, neither souces nor sinks current 
  digitalWrite(pin, LOW); // Enable internal pull-down resistor, ensure pin pulled to low voltage 
}

void set_LED(LEDS color) {
  switch (color)
  {
    case YELLOW:
      set_L(L1); set_H(L2); set_Z(L3);
      break;

    case RED1:
      set_H(L1); set_L(L2); set_Z(L3);
      break;
    
    case GREEN:
      set_Z(L1); set_L(L2); set_H(L3);
      break;

    case BLUE:
      set_Z(L1); set_H(L2); set_L(L3);
      break;

    case WHITE:
      set_L(L1); set_Z(L2); set_H(L3);
      break;
    case RED2:
      set_H(L1); set_Z(L2); set_L(L3);
      break;
  }  
}


