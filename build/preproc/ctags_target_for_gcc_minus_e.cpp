# 1 "/Users/thomas/ArduinoProjects/sub/main/main.ino"
# 2 "/Users/thomas/ArduinoProjects/sub/main/main.ino" 2
# 13 "/Users/thomas/ArduinoProjects/sub/main/main.ino"
int POT_MAX = 4095;


float enc_to_rot = 1.0/(30*16);
volatile int posi = 0;
volatile int veli = 0;
volatile int prev = 0;

// Configure the motor driver.
CytronMD motor(PWM_DIR, 13 /* white*/, 12 /* yellow*/);


// Interrupt variables 
volatile bool deltaT = false;
hw_timer_t * timer0 = 
# 27 "/Users/thomas/ArduinoProjects/sub/main/main.ino" 3 4
                     __null
# 27 "/Users/thomas/ArduinoProjects/sub/main/main.ino"
                         ;
portMUX_TYPE timerMux0 = {.owner = 0xB33FFFFF,.count = 0} /**< Spinlock initializer */;
portMUX_TYPE encoderMux = {.owner = 0xB33FFFFF,.count = 0} /**< Spinlock initializer */;

void __attribute__((section(".iram1" "." "28"))) onTime0() {
  vPortEnterCritical(&timerMux0);
  //portENTER_CRITICAL(&encoderMux);
  veli = prev - posi;
  prev = posi;
  //portEXIT_CRITICAL(&encoderMux);
  vPortExitCritical(&timerMux0);
}


void readEncoder(){
  vPortEnterCritical(&encoderMux);
  deltaT = true;
  int b = digitalRead(25 /* WHITE*/);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
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
  timerAlarmWrite(timer0, 1000000, true); // 1000000 * 1 us = 5 s,
  timerAlarmEnable(timer0);
}

void loop() {
  int pos = 0;
  if(deltaT) {
    vPortEnterCritical(&encoderMux);
    deltaT = false;
    pos = posi;
    vPortExitCritical(&encoderMux);
    float rotations = pos*enc_to_rot;
    float rpm = veli*enc_to_rot*60;

    int potValue = analogRead(34 /*yellow */);
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
