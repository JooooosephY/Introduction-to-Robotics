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



void setup() {
  pinMode(23, OUTPUT);

  analogSetWidth(12);
  analogSetPinAttenuation(IR1, ADC_11db);
  analogSetPinAttenuation(IR2, ADC_11db);
  analogSetPinAttenuation(IR3, ADC_11db);
  analogSetPinAttenuation(IR4, ADC_11db);
  analogSetPinAttenuation(IR5, ADC_11db);


  Serial.begin(9600);

  // pinMode( PWMA, OUTPUT);
  pinMode( DIRA, OUTPUT);
  // pinMode( PWMB, OUTPUT);
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

}

void loop() {


  int reading1 = analogRead(IR1);
  int reading2 = analogRead(IR2);
  int reading3 = analogRead(IR3);
  int reading4 = analogRead(IR4);
  int reading5 = analogRead(IR5);
//  Serial.print("Reading1: ");
//  Serial.print(reading1);
//  Serial.print("\t Reading2: ");
//  Serial.print(reading2);
//  Serial.print("\t Reading3: ");
//  Serial.print(reading3);
//  Serial.print("\t Reading4: ");
//  Serial.print(reading4);
//  Serial.print("\t Reading5: ");
//  Serial.print(reading5);
//  Serial.println();

  int max_left = max(reading1, reading2);
  int max_right = max(reading4, reading5);
  int max_one = max(reading3, max_left);
  int max_value = max(max_one, max_right);
  if (max_value == reading1 || max_value == reading2) {
    Serial.println("The obstacle is on the left.");
  } else if (max_value == reading3) {
    Serial.println("The obstacle is in front of the car. ");
  } else if (max_value == reading4 || max_value == reading5) {
    Serial.println("The obstacle is on the right. ");
  }


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
