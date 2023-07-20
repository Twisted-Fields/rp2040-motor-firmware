#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "./hw_setup.h"
#include "./led_signals.h"


Adafruit_NeoPixel pixels(LED_WS2812_NUM, LED_WS2812_PIN, NEO_GRB + NEO_KHZ800);

void LEDSignals::init(int brightness){
  pixels.begin();
  pixels.setBrightness(brightness);
  pixels.clear();
  pixels.show();
};



void LEDSignals::signalInitState(int state){
    if (state>3)
        state=3;
    pixels.clear();
    for (int i=0;i<state;i++){
      pixels.setPixelColor(i, pixels.Color(0,255,0));
    }
    pixels.show();
};



void LEDSignals::signalErrorState(){
    pixels.setPixelColor(0, pixels.Color(255,0,0));
    pixels.setPixelColor(1, pixels.Color(255,0,0));
    pixels.setPixelColor(2, pixels.Color(255,0,0));
    pixels.show();
};



void LEDSignals::setPixel(int num, int r, int g, int b){
    pixels.setPixelColor(num, pixels.Color(r, g, b));
    pixels.show();
};

