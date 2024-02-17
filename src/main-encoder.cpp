

#include "hardware/gpio.h"

#include "CRC.h"

#include <mcp_can.h>
#include <mcp_can_dfs.h>
#include <SPI.h>
#include <iso-tp.h>
#include <PicoOTA.h>
#include <LittleFS.h>
#include <hardware/exception.h>
#include <Wire.h>

#include <Adafruit_ADS1X15.h>
#include "./acorn-common.h"


Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */


int sinMin = 0, sinMax = 0;
int cosMin = 0, cosMax = 0;

bool print_logging = false;
#define CALIBRATE_SENSOR false


uint32_t serial_speed = 57600;


void save_calibration()
// Saves the calibration to SPI Flash using LittleFS
{
  LittleFS.begin();
  File file = LittleFS.open("encoder_calibration.txt", "w");
  if (!file) {
    log_string("Failed to open file for writing.");
    log_string("Calibration not saved!");
    return;
  }
  file.print(sinMin);
  file.print(",");
  file.print(sinMax);
  file.print(",");
  file.print(cosMin);
  file.print(",");
  file.print(cosMax);
  file.close();
  LittleFS.end();
  log_string("Saved calibration values:");
  log_string("sinMin: " + String(sinMin));
  log_string("sinMax: " + String(sinMax));
  log_string("cosMin: " + String(cosMin));
  log_string("cosMax: " + String(cosMax));
  // log_string("Direction: " + String((Direction)direction));
}

void init_sensor_from_saved_calibration()
// Initializes motor using calibration from SPI Flash with LittleFS
{
  LittleFS.begin();
  File file = LittleFS.open("encoder_calibration.txt", "r");
  if (!file) {
    log_string("Failed to open file for reading");
    return;
  }
  String line = file.readStringUntil('\n');
  file.close();
  LittleFS.end();

  log_string("FULL CALIBRATION STRING:");
  log_string(line);

  int comma = line.indexOf(',');
  sinMin = line.substring(0, comma).toInt();
  int next_start = comma + 1;
  comma = line.indexOf(',', next_start);
  sinMax = line.substring(next_start, comma).toInt();
  next_start = comma + 1;
  comma = line.indexOf(',', next_start);
  cosMin = line.substring(next_start, comma).toInt();
  next_start = comma + 1;
  comma = line.indexOf(',', next_start);
  cosMax = line.substring(next_start, comma).toInt();
  log_string("Initializing motor from saved calibration...");
  log_string("Loaded calibration values:");
  log_string("sinMin: " + String(sinMin));
  log_string("sinMax: " + String(sinMax));
  log_string("cosMin: " + String(cosMin));
  log_string("cosMax: " + String(cosMax));
  // log_string("Direction: " + String((Direction)direction));

}

#define ADC_1 A0
#define ADC_2 A1
#define ADC_3 A2

bool sensor_calibrated = false;
int angle_phase = 0;

float angle_offset = 0;

int phase0_analog1 = 0;
int phase0_analog2 = 0;
int phase0_analog3 = 0;
int phase1_analog1 = 0;
int phase1_analog2 = 0;
int phase1_analog3 = 0;
bool phase_detected = false;

void phase_detect()
{
    int analog1 = 512 - analogRead(ADC_1);
    int analog2 = 512 - analogRead(ADC_2);
    int analog3 = 512 - analogRead(ADC_3);

    analog1 = (analog1 > 100) ? 1 : (analog1 < -100) ? -1 : 0;
    analog2 = (analog2 > 100) ? 1 : (analog2 < -100) ? -1 : 0;
    analog3 = (analog3 > 100) ? 1 : (analog3 < -100) ? -1 : 0;

    if(angle_phase == 0)
    {
      if(analog1!=0) phase0_analog1 = analog1;
      if(analog2!=0) phase0_analog2 = analog2;
      if(analog3!=0) phase0_analog3 = analog3;
    } else {
      if(analog1!=0) phase1_analog1 = analog1;
      if(analog2!=0) phase1_analog2 = analog2;
      if(analog3!=0) phase1_analog3 = analog3;
    }

    if(analog1 == 1 || analog2 == -1 || analog3 == 1)
    {
      phase_detected = true;
      if(angle_phase != 0)
      {
        angle_offset += 360;
        angle_phase = 0;
      }
      log_string("Phase 0 detected");
      
    } else
    if(analog1 == -1 || analog2 == 1 || analog3 == -1)
    {
      phase_detected = true;
      if(angle_phase != 1)
      {
        angle_offset += 360;
        angle_phase = 1;
      }
      log_string("Phase 1 detected");
    }

  // in one debug_print command, print all six values
    // debug_print("phase0_analog1: " + String(phase0_analog1) + " | phase0_analog2: " + String(phase0_analog2) + " | phase0_analog3: " + String(phase0_analog3) + " | phase1_analog1: " + String(phase1_analog1) + " | phase1_analog2: " + String(phase1_analog2) + " | phase1_analog3: " + String(phase1_analog3));
  }




void setup(void)
{


  init_debug();


  ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV

  pinMode(ADC_1, INPUT);
  pinMode(ADC_2, INPUT);
  pinMode(ADC_3, INPUT);

  Wire.setSDA(0);
  Wire.setSCL(1);
  Wire.begin();

  if (!ads.begin()) {
    log_string("Failed to initialize ADS.");
    while (1);
  }

  if(!CALIBRATE_SENSOR)
  {
    init_sensor_from_saved_calibration();
    sensor_calibrated = true;
    // delay(5000);
  }
}


float last_angle = 0;
long ts = millis();  


void loop(void)
{

  if(CALIBRATE_SENSOR && millis() > 10000 && !sensor_calibrated)
  {
    save_calibration();
    sensor_calibrated = true;
    delay(5000);
  }

  int16_t result1;
  int16_t result2;

  /* Be sure to update this value based on the IC and the gain settings! */
  float   multiplier = 3.0F;    /* ADS1015 @ +/- 6.144V gain (12-bit results) */
  //float multiplier = 0.1875F; /* ADS1115  @ +/- 6.144V gain (16-bit results) */

  result1 = ads.readADC_Differential_0_1();
  result2 = ads.readADC_Differential_2_3();
   

  int sinValue = result1;
  int cosValue = result2;

  sinMin = min(sinMin, sinValue);
  sinMax = max(sinMax, sinValue);
  cosMin = min(cosMin, cosValue);
  cosMax = max(cosMax, cosValue);

  // Calculate normalized readings (0.0 to 1.0)
  float sinNorm = (sinValue - sinMin) / float(sinMax - sinMin);
  float cosNorm = (cosValue - cosMin) / float(cosMax - cosMin);

  sinNorm = sinNorm * 2.0 - 1.0;
  cosNorm = cosNorm * 2.0 - 1.0;

  float angleRad = atan2(sinNorm, cosNorm);

  // Convert radians to degrees (optional)
  float angleDeg = angleRad * 180.0 / PI  + 180.0;

  if (abs(last_angle - angleDeg) > 90)
  {
    log_string("Angle jump!");
    if(last_angle > angleDeg)
    {
      angle_offset += 360;
      angle_phase = angle_phase==0 ? 1 : 0;
    } else {
      angle_offset -= 360;
      angle_phase = angle_phase==0 ? 1 : 0;

    }
  }
  if(!phase_detected)
  {
    phase_detect();
  } 
  last_angle = angleDeg;
  float corrected_angle = (angleDeg + angle_offset)/2.0;
  int16_t angle_int = (int16_t)(corrected_angle*20);

  if(phase_detected)
  {
    angle_int |=  0x1;
  } else {
    angle_int &= 0xFFFE;
  }
  bool phase_d = angle_int & 0x01;
  float new_angle = (angle_int)/20.0f;
  if(print_logging)
  {
    if (ts + 100uL <= millis()) {
    ts = millis();
    debug_print("Angle (Deg): " + String(new_angle) + " | Phase: " + String(phase_d) + " | Angle (Normal): " + String(corrected_angle));
  }
  }else if(UART_DEBUG.available())
  {
    while (UART_DEBUG.available()) UART_DEBUG.read();
    uint32_t value_representation = reinterpret_cast<uint32_t&>(angle_int);
    UART_DEBUG.write(byte(value_representation & 0xFF));
    UART_DEBUG.write(byte(value_representation >> 8 & 0xFF));
  }

}



   

