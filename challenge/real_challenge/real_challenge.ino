#define PWMA 27
#define DIRA 14
#define PWMB 12
#define DIRB 13
#define IR1 32
#define IR2 33
#define IR3 34
#define IR4 36
#define IR5 39


struct Tick {
  const uint8_t PIN;
  uint32_t numberTicks;
  bool tickOn;
};

Tick encoder1 = {16, 0, false};
Tick encoder2 = {17, 0, false};

void IRAM_ATTR isr() {
  encoder1.numberTicks += 1;
  encoder1.tickOn = true;
}
void IRAM_ATTR isr2() {
  encoder2.numberTicks += 1;
  encoder2.tickOn = true;
}

int speed1 = 200;

#include <PID_v1.h>

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp = 2, Ki = 5, Kd = 1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


void setup() {
  // put your setup code here, to run once:
  //  pinMode( PWMB, OUTPUT);
  //  pinMode( DIRB, OUTPUT);
  //  pinMode( DIRA, OUTPUT);
  //  pinMode( PWMA, OUTPUT);
  pinMode(23, OUTPUT);
  //  ledcAttachPin(PWMA, 1);
  //  ledcAttachPin(PWMB, 2);
  //  ledcSetup(1, 12000, 8); // 12 kHz PWM, 8-bit resolution
  //  ledcSetup(2, 12000, 8);
  analogSetWidth(12);
  analogSetPinAttenuation(IR1, ADC_11db);
  analogSetPinAttenuation(IR2, ADC_11db);
  analogSetPinAttenuation(IR3, ADC_11db);
  analogSetPinAttenuation(IR4, ADC_11db);
  analogSetPinAttenuation(IR5, ADC_11db);


  Serial.begin(9600);

  pinMode( PWMA, OUTPUT);
  pinMode( DIRA, OUTPUT);
  pinMode( PWMB, OUTPUT);
  pinMode( DIRB, OUTPUT);

  ledcAttachPin(PWMA, 1); // assign RGB led pins to channels
  ledcAttachPin(PWMB, 2);
  // Initialize channels
  // channels 0-15, resolution 1-16 bits, freq limits depend on resolution
  // ledcSetup(uint8_t channel, uint32_t freq, uint8_t resolution_bits);
  ledcSetup(1, 12000, 8); // 12 kHz PWM, 8-bit resolution
  ledcSetup(2, 12000, 8);


  pinMode(encoder1.PIN, INPUT_PULLUP);
  pinMode(encoder2.PIN, INPUT_PULLUP);
  attachInterrupt(encoder1.PIN, isr, FALLING);
  attachInterrupt(encoder2.PIN, isr2, FALLING);

  digitalWrite(23, HIGH);
  delay(10);

  myPID.SetOutputLimits(0, 255);

  //initialize the variables we're linked to
  Input = analogRead(IR3);
  Setpoint = 175;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void loop() {


  int reading1 = analogRead(IR1);
  int reading2 = analogRead(IR2);
  int reading3 = analogRead(IR3);
  int reading4 = analogRead(IR4);
  int reading5 = analogRead(IR5);
  Serial.print("Reading1: ");
  Serial.print(reading1);
  Serial.print("\t Reading2: ");
  Serial.print(reading2);
  Serial.print("\t Reading3: ");
  Serial.print(reading3);
  Serial.print("\t Reading4: ");
  Serial.print(reading4);
  Serial.print("\t Reading5: ");
  Serial.print(reading5);
  Serial.println();


  //  digitalWrite(23, LOW);
  //  delay(100);

  int dir = detect_obstacle(reading1, reading2, reading3, reading4, reading5);

  switch (dir) {
    case 0: // defalt: move forward
      motorA(speed1, 1);
      motorB(speed1, 1);
      break;
    case 1: // obstacle is on the left
      motorA(speed1, 1);
      motorB(0, 1);
      break;
    case 2: // obstacle is in front of the car

      motorA(speed1, 1);
      motorB(speed1, 1);

      break;
    case 3: // obstacle is on the right
      motorA(0, 1);
      motorB(speed1, 1);
      break;
  }


  //Serial.println("BOTH MOTORS...");
  //  if ( encoder1.numberTicks > encoder2.numberTicks) {
  //    motorA (speed1, 1);
  //    motorB (0, 1);
  //  } else {
  //    motorA (0, 1);
  //    motorB (speed1, 1);
  //  }
  //
  //    Input = reading3;
  //    myPID.Compute();
  //    if (Output > 0) {
  //      motorA (Output, 1);
  //    } else {
  //      motorA (-Output, 0);
  //    }
}


void motorA ( int speed, int direction) {
  if (direction) {
    digitalWrite(DIRA, HIGH);
    ledcWrite(1, 255 - speed);

    // sigmaDeltaWrite(0, 255 - speed);
  }
  else {
    digitalWrite(DIRA, LOW);
    //sigmaDeltaWrite(0, speed);
    ledcWrite(1, speed);
  }
}

void motorB ( int speed, int direction) {
  if (direction) {
    digitalWrite(DIRB, HIGH);
    //sigmaDeltaWrite(3, 255 - speed);
    ledcWrite(2, 255 - speed);
  }
  else {
    digitalWrite(DIRB, LOW);
    //sigmaDeltaWrite(3, speed);
    ledcWrite(2, speed);
  }
}

int detect_obstacle(int reading1, int reading2, int reading3, int reading4, int reading5) {
  int max_left = max(reading1, reading2);
  int max_right = max(reading4, reading5);
  int max_one = max(reading3, max_left);
  int max_value = max(max_one, max_right);
  if (max_value <= 200) {
    return 0;
  } else {
    if (max_value == reading1 || max_value == reading2) {
      return 1;
    } else if (max_value == reading3) {
      return 2;
    } else if (max_value == reading4 || max_value == reading5) {
      return 3;
    }
  }
}
