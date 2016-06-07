#include <Adafruit_NeoPixel.h>
#include "pitches.h"
#define PIN 6
Adafruit_NeoPixel strip = Adafruit_NeoPixel(24, PIN, NEO_GRB + NEO_KHZ800);
int randa =0;
int triggered =0;
int delaycount=0;
int LDR_Pin = A0; //analog pin 0
int LDRReading_x =0;
int melody[] = {  
  NOTE_A3, NOTE_C4, NOTE_F3, NOTE_D3};
int noteDurations[] = {  
  2, 2, 1, 1, 0};

// MAPLIN (not so)SMART PUMPKIN 2014

// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and minimize distance between Arduino and first pixel.  Avoid connecting
// on a live circuit...if you must, connect GND first.

void setup() {
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  Serial.begin(9600);
  int LDRReading = analogRead(LDR_Pin); 
}

void loop() {
  // checks the LDR and runs one of two routines depending on level 
  LDRReading_x = analogRead(LDR_Pin); 
  if(LDRReading_x<400){
    delaycount=1; 
  } //change the 400 here to best suit the placement of your pumpkin
  if(triggered==1){
    delaycount=10;//number of steps to be green - increase to make a longer green period
  }

  if(delaycount>0){
    delaycount--;
    colorWipe(strip.Color(0, 255, 0), 50); // Green
    strip.show();
    makenoise();
    delay(1000);
  }
  else{
    colorWipe(strip.Color(255, 0, 0), 50); // Red
    colorWipe(strip.Color(220, 105, 0), 50); // Red/yellow
    colorWipe(strip.Color(215, 35, 0), 50); // Red/orange
  }
}

// Variation of the Adafruit colorwipe function with a small random parameter (0-20)
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    randa=20-random(20);
    strip.setPixelColor(i, c+(randa*2));
    strip.show();
    delay(wait);
  }
}

//simple noise function with the tone library - actual tones and lengths are defined at top of code
void makenoise(void)
{
  for (int thisNote = 0; thisNote < 4; thisNote++) {
    int noteDuration = 1000/noteDurations[thisNote];
    tone(2, melody[thisNote],noteDuration);
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    noTone(2);
    digitalWrite(2, LOW);    
  }
}
