# 1 "/Users/thomas/ArduinoProjects/sub/main/main.ino"
# 2 "/Users/thomas/ArduinoProjects/sub/main/main.ino" 2
# 13 "/Users/thomas/ArduinoProjects/sub/main/main.ino"
int POT_MAX = 4095;

// Speed of Accelerometer up to 6.66 kHz
// Buffer up to 9kB data between reads w bult in FIFO
// Embedded temperatur sensor 16-bit
// 2 Qwiic conncetors 
// 2 interrupt pins 

float ENC_TO_ROT = 1.0/(30*16);
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
CytronMD motor(PWM_DIR, 13 /* white*/, 12 /* yellow*/);


// Interrupt variables 
volatile bool deltaT = false;
hw_timer_t * timer0 = 
# 42 "/Users/thomas/ArduinoProjects/sub/main/main.ino" 3 4
                     __null
# 42 "/Users/thomas/ArduinoProjects/sub/main/main.ino"
                         ;
portMUX_TYPE timerMux0 = {.owner = 0xB33FFFFF,.count = 0} /**< Spinlock initializer */;
portMUX_TYPE encoderMux = {.owner = 0xB33FFFFF,.count = 0} /**< Spinlock initializer */;

void __attribute__((section(".iram1" "." "28"))) onTime0() {
  vPortEnterCritical(&timerMux0);
  velocity = prev_position - position;
  prev_position = position;
  vPortExitCritical(&timerMux0);
}


void readEncoder(){
  vPortEnterCritical(&encoderMux);
  deltaT = true;
  int b = digitalRead(25 /* WHITE*/);
  if(b > 0){
    position++;
  }
  else{
    position--;
  }
  vPortExitCritical(&encoderMux);
}



void setup() {
  Serial.begin(9600);
  pinMode(26 /* YELLOW*/, 0x05);
  pinMode(25 /* WHITE*/, 0x05);
  attachInterrupt((((26 /* YELLOW*/)<40)?(26 /* YELLOW*/):-1),readEncoder,0x01);
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
  vPortEnterCritical(&encoderMux);
  deltaT = false;
  temp = position;
  vPortExitCritical(&encoderMux);
  n_rotations = temp*ENC_TO_ROT;
  rpm = velocity*ENC_TO_ROT*60;

  int potValue = analogRead(34 /*yellow */);
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
