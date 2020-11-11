/*Program to control LED (ON/OFF) from ESP32 using Serial Bluetooth
   Thanks to Neil Kolbans for his efoorts in adding the support to Arduino IDE
   Turotial on: www.circuitdigest.com
*/

#include "BluetoothSerial.h" //Header File for Serial Bluetooth, will be added by default into Arduino

#define PWMA 27
#define DIRA 14
#define PWMB 12
#define DIRB 13

BluetoothSerial ESP_BT; //Object for Bluetooth

int incoming;
int speed1 = 200;
int LED_BUILTIN = 2;

void setup() {
  Serial.begin(9600); //Start Serial monitor in 9600
  ESP_BT.begin("ESP32_YZ"); //Name of your Bluetooth Signal
  Serial.println("Bluetooth Device is Ready to Pair");


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

  //  pinMode (LED_BUILTIN, OUTPUT);//Specify that LED pin is output
}

void loop() {

  if (ESP_BT.available()) //Check if we receive anything from Bluetooth
  {
    incoming = ESP_BT.read(); //Read what we recevive
    Serial.print("Received:");
    Serial.println(incoming);

    if (incoming == 50)
    {
      motorA(speed1, 1);
      motorB(speed1, 1);
      ESP_BT.println("Move forward.");
    }

    if (incoming == 56)
    {
      motorA(speed1, 0);
      motorB(speed1, 0);
      ESP_BT.println("Go back.");
    }

    if (incoming == 52)
    {
      motorA(speed1, 1);
      motorB(speed1, 0);
      ESP_BT.println("Turn left.");
    }

    if (incoming == 54)
    {
      motorA(speed1, 0);
      motorB(speed1, 1);
      ESP_BT.println("Turn right.");
    }

    if (incoming == 48)
    {
      motorA(0, 1);
      motorB(0, 1);
      ESP_BT.println("Stop.");
    }
  }
  delay(20);
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
