# 1 "/Users/thomas/ArduinoProjects/sub/main/main.ino"
# 2 "/Users/thomas/ArduinoProjects/sub/main/main.ino" 2
# 3 "/Users/thomas/ArduinoProjects/sub/main/main.ino" 2
# 4 "/Users/thomas/ArduinoProjects/sub/main/main.ino" 2
# 5 "/Users/thomas/ArduinoProjects/sub/main/main.ino" 2

// Pin Definitions 
# 16 "/Users/thomas/ArduinoProjects/sub/main/main.ino"
// LED Charlieplexing Control Pins




// Potentiometer Pin for Motor Speed Control


// Constants Definitions



float ENC_TO_ROT = 1.0 / (30 /* Gear ratio of the motor*/ * 16 /* Encoder counts per revolution */); // Encoder count to rotation conversion
float CUT_OFF_TEMPERATURE = 30.0; // Temperature threshold in degrees Celsius
float CUT_OFF_GYRO = 300; // Gyro threshold for movement detection

// Sensor and Control Variables
float temperature = 0.0;
float GyroX = 0.0, GyroY = 0.0, GyroZ = 0.0;
int POT_MAX = 4095; // Maximum value for 12-bit ADC
int potValue = 0;
int pwmValue = 0;
int pos = 0; // Non-critical section position

// IMU Setup
LSM6DSO myIMU; // IMU object 
int imu_data; // Variable to store IMU data 

// Encoder Setup
volatile int position = 0;
volatile int velocity = 0;
volatile int prev_position = 0;
float n_rotations = 0;
float rpm = 0;
bool enableUpdatePosition = true;

CytronMD motor(PWM_DIR, 13 /* WHITE Motor Controll pin 1*/, 12 /* YELLOW Motor Controll pin 2*/); // Motor driver object

// State Machine Enums
enum states { STOP, MOVE_CW, MOVE_CCW, OVERHEATED };
enum states state = STOP;

enum LEDS { YELLOW, RED1, GREEN, BLUE, WHITE, RED2 };

// Interrupt Management Variables
volatile bool encoderUpdated = false;
hw_timer_t *timer0 = 
# 62 "/Users/thomas/ArduinoProjects/sub/main/main.ino" 3 4
                    __null
# 62 "/Users/thomas/ArduinoProjects/sub/main/main.ino"
                        ;
portMUX_TYPE encoderMux = {.owner = 0xB33FFFFF,.count = 0} /**< Spinlock initializer */;
Ticker resetPositionTimer; // ticker object for reseting the position 


// Interrupt Service Routine for Timer
// Every second we update the velocity
void __attribute__((section(".iram1" "." "28"))) onTime0(){
  vPortEnterCritical(&encoderMux);
  velocity = prev_position - position;
  prev_position = position;
  vPortExitCritical(&encoderMux);
}

// Encoder Reading Function
// Works by observing changes to the magnetic field created by a magnet attached to the motor shaft
// If A is high before B, we are moving clockwise
// If B is high before A, we are moving counter-clockwise
// The function is called by an edge-triggered interrupt on A (when A is high), then by checking B we can determine the direction
void readEncoder() {
  vPortEnterCritical(&encoderMux); // Entering and exiting two times the ISR allows for less time spent in them, and execute more complex ISR
  encoderUpdated = true;
  int b = digitalRead(25 /* WHITE Encoder B pin */);
  vPortExitCritical(&encoderMux);

  if (enableUpdatePosition) {
    vPortEnterCritical(&encoderMux);
    if (b == 0x1) {
      position++; // If B is HIGH when A rises, increment position (clockwise rotation)
    } else {
      position--; // If B is LOW when A rises, decrement position (counterclockwise rotation)
    }
    vPortExitCritical(&encoderMux);
  }
}


// Setup Function
void setup(){
  Serial.begin(115200);
  set_LED(BLUE); // Set LED to blue, indicating that we are booting up correctly

  // Initialize Encoder Pins and Interrupt
  pinMode(26 /* YELLOW Encoder A pin */, 0x05);
  pinMode(25 /* WHITE Encoder B pin */, 0x05);
  attachInterrupt((((26 /* YELLOW Encoder A pin */)<40)?(26 /* YELLOW Encoder A pin */):-1), readEncoder, 0x01);

  // Timer Setup for Velocity Calculation
  // Set the 0th HW timer to count upwards
  timer0 = timerBegin(0, 80, true); // Divide by prescaler 80 to get 1MHz tick frequency
  timerAttachInterrupt(timer0, &onTime0, true); // Attach onTime0 function to timer
  timerAlarmWrite(timer0, 1000000, true); // Alarm triggers once every second, resets after being triggered
  timerAlarmEnable(timer0); // Enable alarm

  // Setup for IMU
  Wire.begin(23 /* I2C SDA for IMU */, 22 /* I2C SCL for IMU */);
  if (myIMU.begin()){
    Serial.println("IMU Ready");
  } else {
    Serial.println("Could not connect to IMU.");
  }
  if (myIMU.initialize(0x01)) {
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
      if (CheckForCriticalTemperature() == true) { // Event Checker
        motor_off(); // Service Function
        Serial.println("Maximum temperature exceeded. Switching to state = OVERHEATED"); // Print for debugging
        set_LED(RED1); // Service Function
        state = OVERHEATED;
      }
      if (CheckForEncoderRotationCW() == true) { // Event Checker
        motor_on_CW(); // Service Function
        Serial.println("CW Manual start detected. Switching to state = MOVE_CW"); // Print for debugging
        set_LED(GREEN); // Service Function
        state = MOVE_CW;
      }
      if (CheckForEncoderRotationCCW() == true) { // Event Checker
        motor_on_CCW(); // Service Function
        Serial.println("CCW Manual start detected. Switching to state = MOVE_CCW"); // Print for debugging
        set_LED(WHITE); // Service Function
        state = MOVE_CCW;
      }
      break;

  case MOVE_CW:
    if (CheckForCriticalTemperature() == true) { // Event Checker
      motor_off(); // Service Function
      Serial.println("Maximum temperature exceeded. Switching to state = OVERHEATED"); //Print for debugging
      set_LED(RED1); // Service Function
      state = OVERHEATED;
    }
    if (CheckForCriticalGyro() == true) { // Event Checker
      motor_off(); // Service Function
      Serial.println("Maximum gyro exceeded. Switching to state = STOP"); // Print for debugging
      set_LED(YELLOW); // Service Function
      state = STOP;
    }
    break;

  case MOVE_CCW:
    if (CheckForCriticalTemperature() == true) { // Event Checker
      motor_off(); // Service Function
      Serial.println("Maximum temperature exceeded. Switching to state = OVERHEATED"); //Print for debugging
      set_LED(RED1); // Service Function
      state = OVERHEATED;
    }
    if (CheckForCriticalGyro() == true) { // Event Checker
      motor_off(); // Service Function
      Serial.println("Maximum gyro exceeded. Switching to state = STOP"); // Print for debugging
      set_LED(YELLOW); // Service Function
      state = STOP;
    }
    break;

  case OVERHEATED:
    if (CheckForCriticalTemperature() == false) { // Event Checker
      Serial.println("Temperature back to normal. Switching to state = STOP"); //Print for debugging
      set_LED(YELLOW); // Service Function
      reset_position(); // (since we didn't turn motor off here as it is already off, we need to explicitly reset the position of the encoder)
      state = STOP;
    }
    break;
  }
}

// ************************ Event Checkers (Functions) ************************
bool CheckForEncoderRotationCW() { // Check for clockwise rotation
  return position < -70;
}

bool CheckForEncoderRotationCCW() { // Check for clockwise rotation
  return position > 70;
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
void motor_on_CW() {
  potValue = analogRead(34 /* YELLOW potentiometer*/);
  pwmValue = map(potValue, 0, POT_MAX, 50, 128);
  motor.setSpeed(pwmValue);
  Serial.print("Motor ON (CW). Desired Motor PWM: ");
  Serial.println(pwmValue);
}

void motor_on_CCW() {
  potValue = analogRead(34 /* YELLOW potentiometer*/);
  pwmValue = map(potValue, 0, POT_MAX, 50, 128);
  motor.setSpeed(-pwmValue);
  Serial.print("Motor ON (CCW). Desired Motor PWM: ");
  Serial.println(-pwmValue);
}

void motor_off() {
  motor.setSpeed(0);
  position = 0;
  enableUpdatePosition = false;
  resetPositionTimer.once(1, reset_position); // calls the reset position function after 1 sec 
  Serial.println("Motor OFF");
}

void reset_position() {
  position = 0;
  prev_position = 0;
  velocity = 0;
  rpm = 0;
  enableUpdatePosition = true;
  //resetPositionTimer.detach();
}


// Update rpm, pos and rotation count 
void updateSpeedAndPos() {
  int localPosition;
  int localVelocity;

  vPortEnterCritical(&encoderMux);
  encoderUpdated = false;
  localPosition = position;
  localVelocity = velocity;
  vPortExitCritical(&encoderMux);

  pos = localPosition;
  n_rotations = pos * ENC_TO_ROT;
  rpm = localVelocity * ENC_TO_ROT * 60;
}


// ************************ Functions for LEDs (control charlieplexing) ************************
// Implementing charlieplexing, a technique uest to control multiple LEDs with fewer I/O pins 
// exploiting the tristate capabilities of the microcontroller, this means that pins can be both on off and "disconnected"

// Set LED pin to high (sourcing current)
void set_H(int pin) {
  pinMode(pin, 0x03);
  digitalWrite(pin, 0x1);
}

// Set LED pin to low (sinking current)
void set_L(int pin) {
  pinMode(pin, 0x03);
  digitalWrite(pin, 0x0);
}

// Set LED pin to Z (high-impedans)
void set_Z(int pin) {
  pinMode(pin, 0x01); // Put pin in high-impedans state, neither souces nor sinks current 
  digitalWrite(pin, 0x0); // Enable internal pull-down resistor, ensure pin pulled to low voltage 
}

void set_LED(LEDS color) {
  switch (color)
  {
    case YELLOW:
      set_L(15); set_H(32); set_Z(14);
      break;

    case RED1:
      set_H(15); set_L(32); set_Z(14);
      break;

    case GREEN:
      set_Z(15); set_L(32); set_H(14);
      break;

    case BLUE:
      set_Z(15); set_H(32); set_L(14);
      break;

    case WHITE:
      set_L(15); set_Z(32); set_H(14);
      break;
    case RED2:
      set_H(15); set_Z(32); set_L(14);
      break;
  }
}
