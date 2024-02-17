

#include "hardware/gpio.h"

#include "CRC.h"

#include <mcp_can.h>
#include <mcp_can_dfs.h>
#include <SPI.h>
#include <iso-tp.h>
#include <PicoOTA.h>
#include <LittleFS.h>
#include <hardware/exception.h>
#include <Adafruit_NeoPixel.h>


#define LED_WS2812_PIN 5
#define VOUT_ENABLE 7 
#define VIN_ENABLE 8 
#define BLEED_FET 6 
#define SWITCH_1 10
#define SWITCH_2 9
#define SWITCH_3 11

#define VDC_CS 4
#define INPUT_THERMISTOR_CS 1
#define OUTPUT_THERMISTOR_CS 3

#define SPI_CLK 2
#define SPI_DATA 0

#define UART_DEBUG Serial2
#define SERIAL_SPEED 921600


#define CURRENT_MEASURE_IN_PIN A0
#define CURRENT_MEASURE_OUT_PIN A1
#define VINPUT_MEASURE_PIN A2
#define VOUTPUT_MEASURE_PIN A3


Adafruit_NeoPixel pixels(3, LED_WS2812_PIN, NEO_GRB + NEO_KHZ800);


// This function modified from Adafruit example code.
double thermistor_calc(uint16_t adc_value, uint16_t adc_full_scale)
{
  #define TEMPERATURENOMINAL 25 
  #define BCOEFFICIENT  3380
  #define SERIES_RESISTOR_VALUE 10000
  #define THERMISTOR_NOMINAL 10000
  float reading = ((adc_full_scale * SERIES_RESISTOR_VALUE) / adc_value);
  reading -= SERIES_RESISTOR_VALUE;

  double steinhart;
  steinhart = reading / THERMISTOR_NOMINAL;          // (R/Ro)
  steinhart = log(steinhart);                       // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                        // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                      // Invert
  steinhart -= 273.15;                              // convert to C

  return steinhart;
}

float read_spi_thermistor(uint16_t cs_pin){
    digitalWrite(cs_pin, LOW);
    delayMicroseconds(1);
    uint16_t spi_th1 = SPI.transfer16(0x0)>>4;
    float temp = thermistor_calc(spi_th1, 256);
    digitalWrite(cs_pin, HIGH);
    return temp;
}




/// @brief Read the 12 bit voltage adc value
/// @return The 12 bit integer value read from the adc
uint16_t spiRead() {
    uint16_t out = 0x00;
    // Assert CS to select device (active low)
    digitalWrite(VDC_CS, 0);
    for (uint32_t i = 0; i < 2; i++) {
      delayMicroseconds(1);
      digitalWrite(SPI_CLK, HIGH);
      delayMicroseconds(1);
      digitalWrite(SPI_CLK, LOW);
    }
    for (uint32_t i = 0; i < 12; i++) {
        // Assert the clock
        digitalWrite(SPI_CLK, HIGH);
        delayMicroseconds(1);
        // Read the MISO line for slave data
        if (digitalRead(SPI_DATA))
            out |= 0x01;
        out <<= 1;
        digitalWrite(SPI_CLK, LOW);
        delayMicroseconds(1);
    }
    // Negate CS
    digitalWrite(VDC_CS, 1);
    return out;
}

float read_system_voltage()
{
  uint16_t adc_data = spiRead();
  float voltage = (adc_data * 3.3f/4095.0f)/ (5.1/(100+5.1))*(50.0f/49.85f);
  return voltage;
}


void setPixel(int num, int r, int g, int b){
    pixels.setPixelColor(num, pixels.Color(r, g, b));
    pixels.show();
};

void disable_output()
{
    setPixel(0, 255, 0, 0);
    // setPixel(1, 255, 0, 0);
    // setPixel(2, 255, 0, 0);
    // digitalWrite(VOUT_ENABLE, LOW);
    analogWrite(VOUT_ENABLE, 0);
}

void enable_output()
{
    setPixel(0, 0, 255, 0);
    // setPixel(1, 0, 255, 0);
    // setPixel(2, 0, 255, 0);
    // digitalWrite(VOUT_ENABLE, HIGH);
    analogWrite(VOUT_ENABLE, 10);
}

void setup()
{

  UART_DEBUG.setRX(25);
  UART_DEBUG.setTX(24);
  UART_DEBUG.setFIFOSize(128);
  UART_DEBUG.begin(SERIAL_SPEED);

  pixels.begin();
  pixels.setBrightness(10);
  pixels.clear();
  pixels.show();
  setPixel(0, 255, 255, 0);
  setPixel(1, 0, 255, 255);
  setPixel(2, 255, 0, 255);

  pinMode(CURRENT_MEASURE_IN_PIN, INPUT);
  pinMode(CURRENT_MEASURE_OUT_PIN, INPUT);
  pinMode(VINPUT_MEASURE_PIN, INPUT);
  pinMode(VOUTPUT_MEASURE_PIN, INPUT);

  pinMode(BLEED_FET, OUTPUT);
  pinMode(VDC_CS, OUTPUT);
  pinMode(INPUT_THERMISTOR_CS, OUTPUT);
  pinMode(OUTPUT_THERMISTOR_CS, OUTPUT);
  pinMode(SPI_CLK, OUTPUT);
  pinMode(SPI_DATA, INPUT);

  // initialize switch pins
  pinMode(SWITCH_1, INPUT_PULLUP);
  pinMode(SWITCH_2, INPUT_PULLUP);
  pinMode(SWITCH_3, INPUT_PULLUP);

  pinMode(VIN_ENABLE, OUTPUT);
  pinMode(VOUT_ENABLE, OUTPUT);
  digitalWrite(VIN_ENABLE, HIGH);
  digitalWrite(VOUT_ENABLE, LOW);
  // digitalWrite(BLEED_FET, HIGH);
  analogWriteFreq(100000);
  analogWriteRange(100);
  analogWrite(VOUT_ENABLE, 0);
  disable_output();
  // digitalWrite(VOUT_ENABLE, HIGH);
  delay(500);
  // setPixel(0, 0, 0, 0);
  setPixel(1, 0, 0, 0);
  setPixel(2, 0, 0, 0);

}

void debug_print(String text){
  if(UART_DEBUG)
  { 
    UART_DEBUG.println(text);
  }
}




bool output_state = LOW;
bool state_change = false;
unsigned long ts = 0;
unsigned long activation_time = 0;
void loop()
{

  float input_thermistor = read_spi_thermistor(INPUT_THERMISTOR_CS);
  float output_thermistor = read_spi_thermistor(OUTPUT_THERMISTOR_CS);
  float capacitor_voltage = read_system_voltage();

  float vinput = (analogRead(VINPUT_MEASURE_PIN) * 3.3f/1024.0f)/ (5.1/(100+5.1));
  float voutput = (analogRead(VOUTPUT_MEASURE_PIN) * 3.3f/1024.0f)/ (5.1/(100+5.1));
  float input_current = (((analogRead(CURRENT_MEASURE_IN_PIN)-4) * 3.3f/1024.0f) / 50.0f) / 0.0030f; 
  float output_current = (((analogRead(CURRENT_MEASURE_OUT_PIN)-4)* 3.3f/1025.0f) / 50.0f) / 0.0015f;

  // v = i r
  // v/r = i

  // uint16_t input_current = analogRead(CURRENT_MEASURE_IN_PIN)-4.0f;
  // uint16_t output_current = analogRead(CURRENT_MEASURE_OUT_PIN)-4.0f;

  float print_outputs = false;
  if(output_current > 15.0f and millis() - activation_time > 1000)
  {
    print_outputs = true;
    disable_output();
    output_state = LOW;
  }

  if(output_current < 1.0f)
  {
    setPixel(2, 0, 0, 0);
  }else
  if(output_current > 1.0f and output_current < 3.0f)
  {
    setPixel(2, 0, 255, 0);
  }else
  if(output_current > 3.0f and output_current < 5.0f)
  {
    setPixel(2, 255, 255, 0);
  }else
  if(output_current > 5.0f and output_current < 7.0f)
  {
    setPixel(2, 255, 0, 0);
  }else
  if(output_current > 7.0f and output_current < 9.0f)
  {
    setPixel(2, 255, 255, 255);
  } else
  if(output_current > 9.0f)
  {
    setPixel(2, 0, 0, 255);
  }


    if (ts + 10uL < millis() || print_outputs)
    {
      ts = millis();
      debug_print("capacitor_voltage: " + String(capacitor_voltage) + ", " + "vinput: " + String(vinput) + ", " + "voutput: " + String(voutput) + ", " + "input_current: " + String(input_current) + ", " + "output_current: " + String(output_current));
    }

  while(digitalRead(SWITCH_1) == LOW)
  {
    delay(5);
    if(digitalRead(SWITCH_1) == HIGH)
    {
      state_change = true;
    }
  }

  while(digitalRead(SWITCH_2) == LOW)
  {
    delay(5);
    if(digitalRead(SWITCH_2) == HIGH)
    {
      state_change = true;
    }
  }

  while(digitalRead(SWITCH_3) == LOW)
  {
    delay(5);
    if(digitalRead(SWITCH_3) == HIGH)
    {
      state_change = true;
    }
  }

  if(state_change)
  {
    if(output_state==LOW)
    {
      output_state = HIGH;
      enable_output();
      activation_time = millis();
    } else
    {
      output_state = LOW;
      disable_output();
    }
    state_change = false;
  }


}
