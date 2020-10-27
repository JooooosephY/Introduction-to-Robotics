#include <FastLED.h>

// How many leds in your strip?
#define NUM_LEDS 2

#define DATA_PIN 25

CRGB leds[NUM_LEDS];

void setup() {
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
}

int n = 0;

void loop() {
  // Turn the LED on, then pause
  leds[0] = CRGB::Red;
  leds[1] = CRGB::Black;
  FastLED.show();
  delay(100);
  leds[0] = CRGB::Black;
  leds[1] = CRGB::Blue;
  FastLED.show();
  delay(100);
//  // Now turn the LED off, then pause
//  leds[0] = CRGB::Black;
//  FastLED.show();
//  n = n + 1;
//  if (n > 2) {
//    n = 0;
//  }

}
