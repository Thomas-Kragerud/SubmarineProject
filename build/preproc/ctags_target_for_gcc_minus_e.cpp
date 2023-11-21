# 1 "/Users/thomas/ArduinoProjects/sub/main/main.ino"
# 2 "/Users/thomas/ArduinoProjects/sub/main/main.ino" 2
# 3 "/Users/thomas/ArduinoProjects/sub/main/main.ino" 2
# 4 "/Users/thomas/ArduinoProjects/sub/main/main.ino" 2
# 19 "/Users/thomas/ArduinoProjects/sub/main/main.ino"
float ENC_TO_ROT = 1.0 / (30 * 16);
float CUT_OFF_TEMPERATURE = 25.0;
float CUT_OFF_GYRO = 300;

float temperature = 0.0;



float GyroX = 0.0;
float GyroY = 0.0;
float GyroZ = 0.0;

int POT_MAX = 4095;
int potValue = 0;
int pwmValue = 0;
int pos = 0;

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
CytronMD motor(PWM_DIR, 13 /* white Motor*/, 12 /* yellow Motor*/);

enum states
{
  STOP,
  MOVE
};
enum states state = STOP;

// Interrupt variables
volatile bool deltaT = false;
hw_timer_t *timer0 = 
# 66 "/Users/thomas/ArduinoProjects/sub/main/main.ino" 3 4
                    __null
# 66 "/Users/thomas/ArduinoProjects/sub/main/main.ino"
                        ;
portMUX_TYPE timerMux0 = {.owner = 0xB33FFFFF,.count = 0} /**< Spinlock initializer */;
portMUX_TYPE encoderMux = {.owner = 0xB33FFFFF,.count = 0} /**< Spinlock initializer */;

void __attribute__((section(".iram1" "." "28"))) onTime0(){
  vPortEnterCritical(&timerMux0);
  velocity = prev_position - position;
  prev_position = position;
  vPortExitCritical(&timerMux0);
}

void readEncoder() {
  vPortEnterCritical(&encoderMux);
  deltaT = true;
  int b = digitalRead(25 /* WHITE Encoder*/);
  if (b < 0) {
    position++;
  } else {
    position--;
  }
  vPortExitCritical(&encoderMux);
}

void setup(){
  Serial.begin(115200);
  // delay(500);

  pinMode(26 /* YELLOW Encoder*/, 0x05);
  pinMode(25 /* WHITE Encoder*/, 0x05);
  attachInterrupt((((26 /* YELLOW Encoder*/)<40)?(26 /* YELLOW Encoder*/):-1), readEncoder, 0x01);

  timer0 = timerBegin(0, 80, true);
  timerAttachInterrupt(timer0, &onTime0, true);

  timerAlarmWrite(timer0, 1000000, true);
  timerAlarmEnable(timer0);

  // Setup for IMU
  Wire.begin(23, 22);
  if (myIMU.begin()){
    Serial.println("IMU Ready");
  } else {
    Serial.println("Could not connect to IMU.");
  }
  if (myIMU.initialize(0x01)) {
    Serial.println("Loaded Settings.");
  }
}

void loop() {
  delay(10);
  if (deltaT == true) {
    updateSpeedAndPos();
  }

  switch (state) {

  case STOP:
    if (CheckForEncoderRotation() == true) { // Event Checker
      motor_on(); // Service Function
      Serial.println("Manual start detected. Switching to state = MOVE"); // Print
      // for debugging
      state = MOVE;
    }
    break;

  case MOVE:
    if (CheckForCriticalTemperature() == true) { // Event Checker
      motor_off(); // Service Function
      Serial.println("Maximum temperature exceeded. Switching to state = STOP"); //
      // Print for debugging
      state = STOP;
    }
    if (CheckForCriticalGyro() == true) { // Event Checker
      motor_off(); // Service Function
      Serial.println("Maximum gyro exceeded. Switching to state = STOP"); // Print
      // for debugging
      state = STOP;
    }
    break;
  }
}

// ************************ Event Checkers (Functions) ************************
bool CheckForEncoderRotation() {
  if (position != 0) {
    return true;
  }
  else {
    return false;
  }
}

bool CheckForCriticalTemperature() {
  temperature = myIMU.readTempC();
  // Serial.println(temperature);
  if (temperature >= CUT_OFF_TEMPERATURE) {
    return true;
  }
  else {
    return false;
  }
}

bool CheckForCriticalGyro() {
  GyroX = myIMU.readFloatGyroX();
  GyroY = myIMU.readFloatGyroY();

  GyroZ = myIMU.readFloatGyroZ();
  // Serial.println(temperature);
  if ((abs(GyroX) >= CUT_OFF_GYRO) || (abs(GyroY) >= CUT_OFF_GYRO) || (abs(GyroZ) >= CUT_OFF_GYRO)){
    return true;
  } else {
    return false;
  }
}

// ************************ Service Functions ************************
void motor_on() {
  potValue = analogRead(34 /* yellow potentiometer*/);
  pwmValue = map(potValue, 0, POT_MAX, 0, 128);
  motor.setSpeed(pwmValue);
  Serial.print("Motor ON. Desired Motor PWM: ");
  Serial.println(pwmValue);
}

void motor_off() {
  motor.setSpeed(0);
  delay(1000); // Need to wait for motor to stop rotating (body with high interia)
  position = 0;

  Serial.println("Motor OFF");
  // Serial.println(position);
}

// ************************ Other Functions ************************

void updateSpeedAndPos() {
  vPortEnterCritical(&encoderMux);
  deltaT = false;
  pos = -1 * position;
  vPortExitCritical(&encoderMux);
  n_rotations = pos * ENC_TO_ROT;
  rpm = velocity * ENC_TO_ROT * 60;
}
