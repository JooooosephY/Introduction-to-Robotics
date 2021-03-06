//#include <Servo.h>

#include <FastLED.h>

FASTLED_USING_NAMESPACE

#if defined(FASTLED_VERSION) && (FASTLED_VERSION < 3001000)
#warning "Requires FastLED 3.1 or later; check github for latest code."
#endif

#define DATA_PIN    25
//#define CLK_PIN   4
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
#define NUM_LEDS    64
CRGB leds[NUM_LEDS];

#define BRIGHTNESS          96
#define FRAMES_PER_SECOND  120


//pins for motors
#define PWMA 27
#define DIRA 14
#define PWMB 12
#define DIRB 13

// constants won't change :
const long TIME = 500;           // interval at which to blink (milliseconds)
int speeds = 50;
int acc = 1;


void setup()
{
  // tell FastLED about the LED strip configuration
  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  //FastLED.addLeds<LED_TYPE,DATA_PIN,CLK_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);

  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);
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
  Serial.println("starting...");


  //we should use PWM instead of sigmadelta!!!
  //because we don't want an analog signal but a PWM!

  //setup channel 0 with frequency 312500 Hz
  //  sigmaDeltaSetup(0, 312500);
  //  // sigmaDeltaSetup(1, 312500);
  //  // sigmaDeltaSetup(2, 312500);
  //  sigmaDeltaSetup(3, 312500);
  //
  //  sigmaDeltaAttachPin(12, 0);
  //  // sigmaDeltaAttachPin(13,1);
  //  // sigmaDeltaAttachPin(14,2);
  //  sigmaDeltaAttachPin(27, 3);
  //  //initialize channel 0 to off
  //  sigmaDeltaWrite(0, 0);
  //  sigmaDeltaWrite(3, 0);
}


// List of patterns to cycle through.  Each is defined as a separate function below.
typedef void (*SimplePatternList[])();
SimplePatternList gPatterns = { rainbow, rainbowWithGlitter, confetti, sinelon, juggle, bpm };

uint8_t gCurrentPatternNumber = 0; // Index number of which pattern is current
uint8_t gHue = 0; // rotating "base color" used by many of the patterns


void loop()
{
  // Call the current pattern function once, updating the 'leds' array
  gPatterns[gCurrentPatternNumber]();

  // send the 'leds' array out to the actual LED strip
  FastLED.show();

  //  int speed0 = 200;
  //  int speed1 = 200;
  //  Serial.println("MOTOR 0...");
  //  motorA (speed0, 0);
  //  delay (TIME);
  //  motorA (speed0, 1);
  //  delay (TIME);
  //  motorA (0, 0);
  //
  //  Serial.println("MOTOR 1...");
  //  motorB (speed1, 0);
  //  delay (TIME);
  //  motorB (speed1, 1);
  //  delay (TIME);
  //  motorB (0, 0);


  Serial.println("BOTH MOTORS...");

  if (speeds >= 254) {
    acc = -1;
  } else if (speeds <= 50) {
    acc = 1;
  }

  speeds += acc;

  //  motorA (speed1, 0);
  //  motorB (speed1, 0);
  //    delay (TIME);
  motorA (speeds, 1);
  motorB (speeds, 1);
  //  delay (TIME);
  //  motorA (0, 0);
  //  motorB (0, 0);
  //  delay (TIME);
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




#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

void nextPattern()
{
  // add one to the current pattern number, and wrap around at the end
  gCurrentPatternNumber = (gCurrentPatternNumber + 1) % ARRAY_SIZE( gPatterns);
}

void rainbow()
{
  // FastLED's built-in rainbow generator
  fill_rainbow( leds, NUM_LEDS, gHue, 7);
}

void rainbowWithGlitter()
{
  // built-in FastLED rainbow, plus some random sparkly glitter
  rainbow();
  addGlitter(80);
}

void addGlitter( fract8 chanceOfGlitter)
{
  if ( random8() < chanceOfGlitter) {
    leds[ random16(NUM_LEDS) ] += CRGB::White;
  }
}

void confetti()
{
  // random colored speckles that blink in and fade smoothly
  fadeToBlackBy( leds, NUM_LEDS, 10);
  int pos = random16(NUM_LEDS);
  leds[pos] += CHSV( gHue + random8(64), 200, 255);
}

void sinelon()
{
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy( leds, NUM_LEDS, 20);
  int pos = beatsin16( 13, 0, NUM_LEDS - 1 );
  leds[pos] += CHSV( gHue, 255, 192);
}

void bpm()
{
  // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
  uint8_t BeatsPerMinute = 62;
  CRGBPalette16 palette = PartyColors_p;
  uint8_t beat = beatsin8( BeatsPerMinute, 64, 255);
  for ( int i = 0; i < NUM_LEDS; i++) { //9948
    leds[i] = ColorFromPalette(palette, gHue + (i * 2), beat - gHue + (i * 10));
  }
}

void juggle() {
  // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy( leds, NUM_LEDS, 20);
  byte dothue = 0;
  for ( int i = 0; i < 8; i++) {
    leds[beatsin16( i + 7, 0, NUM_LEDS - 1 )] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
}
