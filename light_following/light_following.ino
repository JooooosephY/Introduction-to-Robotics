int left_spd = 5;
int right_spd = 6;
int en_left = 4;
int en_right = 7;
int state = 0;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(en_left, OUTPUT);
  pinMode(left_spd, OUTPUT);
  pinMode(right_spd, OUTPUT);
  pinMode(en_right, OUTPUT);

  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

// the loop function runs over and over again forever
void loop() {
  // read the sensor value of left sensor
  int lsp = analogRead(A0);
  // read the sensor value of head sensor
  int hsp = analogRead(A1);
  // read the sensor value of right sensor
  int rsp = analogRead(A2);
  // print out the value you read:
  Serial.println(lsp);
  Serial.println(hsp);
  Serial.println(rsp);
  Serial.println("\n");
  delay(100);        // delay in between reads for stability


  go_straight(70);

  if (rsp < 150 || lsp < 150) {
    if (abs(lsp - rsp) < 50) {
      go_straight(70);
    } else if (rsp < lsp) {
      turn_right();
    } else if (lsp < rsp) {
      turn_left();
    }
  }

}

void turn_left() {
  // turn left
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(en_left, HIGH);  // enable left
  analogWrite(left_spd, 150); // left side speed
  analogWrite(right_spd, 160); // right side speed
  digitalWrite(en_right, HIGH); // enable right
  //  delay(1250);
}

void turn_right() {
  // turn right
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(en_left, LOW);  // enable left
  analogWrite(left_spd, 155); // left side speed
  analogWrite(right_spd, 150); // right side speed
  digitalWrite(en_right, LOW); // enable right
  //  delay(1170);
}

void go_straight(int spd) {
  // go straight
  digitalWrite(en_left, LOW);  // enable left
  analogWrite(left_spd, spd); // left side speed
  analogWrite(right_spd, spd); // right side speed
  digitalWrite(en_right, HIGH); // enable right
  //  delay(4000);
}

void stops() {
  // stop
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  digitalWrite(en_left, LOW);
  analogWrite(left_spd, 0);
  analogWrite(right_spd, 0);
  digitalWrite(en_right, LOW);
  delay(2000);                       // wait for a second
}

/*
Change: We did the light following exercise in last class. And we changed the orientation of the left sensor and right sensor so that they all face forward. 
Result: The platform would automatically go straight and it would turn to the direction of being illuminated when we used the light source to illuminate it.   
  Since we used the flashlight on our phone as the light source, which is not bright as a normal flashlight, we had to put our phone really close to the sensors to make it turn. 
  However, when we put the light source directly in front of the platform, the platform would be trapped in place and kept turning left and right. 
  I think it's because our light was so scattered that one of the light sensors would detect the light was from that direction first and made the platform turn to that direction. Then when the 
  platform turned a little bit, the sensor from the other side would detect more light than the previous sensor so that the platform would turn back. In this case, the previous situation happened again 
  and the process was looped. And the platform would kept turning eventually. We tried to solve this problem by setting a threshhold to the sensor. But it didn't work so well. 
*/
