
// #define SIMPLEFOC_DISABLE_DEBUG

#include <Arduino.h>
#include "hardware/adc.h"
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
// #include "mbed.h"
// #include "MbedCRC.h"
#include "./hw_setup.h"
#include "./led_signals.h"
#include "./motor_utils.h"
#include "hardware/gpio.h"
#include <PicoOTA.h>
#include <LittleFS.h>

#include "CRC.h"

#include "encoders/mt6701/MagneticSensorMT6701SSI.h"

/*
 IMPORTANT: Twisted fields controller uses active-low polarity for low-side switches! 
 Be sure to set -DSIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH=false in the build.
 If you don't do this, the motor will not turn.
*/
#if not(defined(SIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH)) || (SIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH == true)
#error "Please set -DSIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH=false in the build."
#endif


#define PRE_INITIALIZED_STARTUP true

#define ESTOP_DURATION_MS 100

#define SERIAL_SPEED 921600
#define FIRMWARE_VERSION "0.1"

#define DEFAULT_VOLTAGE_POWER_SUPPLY 50.0f
#define DEFAULT_VOLTAGE_LIMIT 50.0f
#define ALIGNMENT_VOLTAGE_LIMIT 4.0f

#define MOTOR_PP 8
#define MOTOR_KV NOT_SET
//#define MOTOR_PHASE_RESISTANCE 0.2f
#define MOTOR_PHASE_RESISTANCE NOT_SET

#define VDC_CS 2
#define VDC_CLK 3
#define VDC_DATA 23

const byte ACK[] = {0xAC, 0x12};


#define SEND_COMPLETE_SETTINGS 1
#define REQUEST_COMPLETE_SETTINGS 2
#define SEND_BASIC_UPDATE 3
#define REQUEST_SENSORS 4
#define REQUEST_HOMING 5
#define SIMPLEFOC_PASS_THROUGH 6
#define FIRMWARE_UPDATE 7
#define FIRMWARE_UPDATE_CPU2 8
#define SIMPLE_PING 9
#define LOG_REQUEST 10
#define RAW_BRIDGE_COMMAND 11

byte  motor1_setpoint_mode = 0;
float motor1_setpoint = 0.0;
float motor1_max_velocity = 0.0;
float motor1_max_torque = 0.0;
float motor1_acceleration = 0.0;

byte  motor2_setpoint_mode = 0;
float motor2_setpoint = 0.0;
float motor2_max_velocity = 0.0;
float motor2_max_torque = 0.0;
float motor2_acceleration = 0.0;


#include "Arduino.h"
#include "pins_arduino.h"


#define LOGGING_DIVIDER '#'
#define LOGGER_LENGTH 2000
#define LOGGER_FULL "LOGGER FULL" + LOGGING_DIVIDER
#define LOGGER_FULL_LEN LOGGER_LENGTH - strlen(LOGGER_FULL)
char logger_string[LOGGER_LENGTH] = "";
uint32_t logger_index = 0;
bool logger_filled = false;


#define PRINT_LOGGING true


BLDCDriver6PWM driver0 = BLDCDriver6PWM(M0_INUH_PIN, M0_INUL_PIN, M0_INVH_PIN, M0_INVL_PIN, M0_INWH_PIN, M0_INWL_PIN);
BLDCDriver6PWM driver1 = BLDCDriver6PWM(M1_INUH_PIN, M1_INUL_PIN, M1_INVH_PIN, M1_INVL_PIN, M1_INWH_PIN, M1_INWL_PIN);
#define CURRENT_VpA 50.0f
#define GAIN 1.0/(CURRENT_VpA/(3.3/2))
InlineCurrentSense current0 = InlineCurrentSense(1.0f, GAIN, M0_AOUTU_PIN, _NC, M0_AOUTW_PIN);
InlineCurrentSense current1 = InlineCurrentSense(1.0f, GAIN, M1_AOUTU_PIN, _NC, M1_AOUTW_PIN);



#define UART_DEBUG Serial2
#define UART_INTERCHIP Serial1


// main loop speed tracking
int count = 0;
unsigned long ts = 0;

float position_target = 0;
int  tick = 0;

float last_sensor_angle = 0.0;
float max_angle_diff = 0.0;

int lasthalls = 0;
float last_position = 0;

float base_motor1_zero_electric_angle = 0.0f;

int last_estop_val = 0;
unsigned long estop_last_millis = 0;
bool motion_allowed = true;
#define ESTOP_SQUARE_INPUT_MOTOR 19



void debug_print(String text){
  if(UART_DEBUG)
  { 
    UART_DEBUG.println(text);
  }
}

void log_string(const char* text);
void log_string(String text);

void log_string(String text)
{
  char char_array[text.length()+1];
  text.toCharArray(char_array, text.length()+1);
  log_string(char_array);
}

void log_string(const char* text)
{

  uint32_t text_len = strlen(text);
  uint32_t updated_index = logger_index + text_len + 1;
  // if(logger_filled == false && updated_index > LOGGER_FULL_LEN)
  // {
  //   memcpy(logger_string + logger_index, LOGGER_FULL, strlen(LOGGER_FULL));
  //   logger_filled = true;
  // }
  // if(!logger_filled)
  // {
  //   memcpy(logger_string + logger_index, text, text_len);
  //   logger_index = updated_index;
  //   logger_string[logger_index - 1] = LOGGING_DIVIDER;
  // }
  if(PRINT_LOGGING)
  {
    if(logger_filled)
    {
      debug_print(LOGGER_FULL);
    }
    debug_print(text);
  }

}


/// @brief Read the 12 bit voltage adc value
/// @return The 12 bit integer value read from the adc
uint16_t spiRead() {
    uint16_t out = 0x00;
    // Assert CS to select device (active low)
    digitalWrite(VDC_CS, 0);
    for (uint32_t i = 0; i < 2; i++) {
      delayMicroseconds(1);
      digitalWrite(VDC_CLK, HIGH);
      delayMicroseconds(1);
      digitalWrite(VDC_CLK, LOW);
    }
    for (uint32_t i = 0; i < 12; i++) {
        // Assert the clock
        digitalWrite(VDC_CLK, HIGH);
        delayMicroseconds(1);
        // Read the MISO line for slave data
        if (digitalRead(VDC_DATA))
            out |= 0x01;
        out <<= 1;
        digitalWrite(VDC_CLK, LOW);
        delayMicroseconds(1);
    }
    // Negate CS
    digitalWrite(VDC_CS, 1);
    return out;
}


void setup() {

  log_string("Initializing...");

  UART_DEBUG.setRX(25);
  UART_DEBUG.setTX(24);
  UART_DEBUG.setFIFOSize(128);
  UART_DEBUG.begin(SERIAL_SPEED);

  UART_INTERCHIP.setRX(1);
  UART_INTERCHIP.setTX(0);
  UART_INTERCHIP.setFIFOSize(128);
  UART_INTERCHIP.begin(SERIAL_SPEED);

  pinMode(VDC_CS, OUTPUT);
  pinMode(VDC_CLK, OUTPUT);
  pinMode(VDC_DATA, INPUT);
  pinMode(ESTOP_SQUARE_INPUT_MOTOR, INPUT);

  SimpleFOCDebug::enable(&UART_DEBUG);
  SimpleFOCDebug::print("Welcome to Twisted Fields RP2040 firmware, v");
  SimpleFOCDebug::println(FIRMWARE_VERSION);

  // configure motor drivers, 6-PWM
  driver0.voltage_power_supply = DEFAULT_VOLTAGE_POWER_SUPPLY;
  driver0.voltage_limit = DEFAULT_VOLTAGE_LIMIT;
  SimpleFOCDebug::println("Initializing driver 0...");
  if (!driver0.init())
    SimpleFOCDebug::println("Driver 0 init failed!");

  driver1.voltage_power_supply = DEFAULT_VOLTAGE_POWER_SUPPLY;
  driver1.voltage_limit = DEFAULT_VOLTAGE_LIMIT;
  SimpleFOCDebug::println("Initializing driver 1...");
  if (!driver1.init())
    SimpleFOCDebug::println("Driver 1 init failed!"); 

  if (driver0.initialized) {
  current0.linkDriver(&driver0);
  SimpleFOCDebug::println("Initializing current sense 0...");
  if (current0.init()!=1)
    SimpleFOCDebug::println("Current sense 0 init failed!");
  }

  if (driver1.initialized) {
  current1.linkDriver(&driver1);
  SimpleFOCDebug::println("Initializing current sense 1...");
  if (current1.init()!=1)
    SimpleFOCDebug::println("Current sense 1 init failed!");
  }

  start_adc_engine();

  log_string("Startup complete.");

  ts = millis();  

  current1.gain_a *= -1;
  current1.gain_c *= -1;
  current1.skip_align = true;
  current0.gain_a *= -1;
  current0.gain_c *= -1;
  current0.skip_align = true;

  current0.calibrateOffsets();
  current1.calibrateOffsets();

  driver0.phase_state[0] = PHASE_ON;
  driver0.phase_state[1] = PHASE_ON;
  driver0.phase_state[2] = PHASE_ON;
  driver0.setPwm(0, 0, 0);

  driver1.phase_state[0] = PHASE_ON;
  driver1.phase_state[1] = PHASE_ON;
  driver1.phase_state[2] = PHASE_ON;
  driver1.setPwm(0, 0, 0);

}

float read_system_voltage()
{

    float adc_data = spiRead();
    adc_data = (adc_data * 3.3f/4095.0f)/ (5.1/(100+5.1))*(50.0f/49.85f);
    return adc_data;

}

uint32_t max_estop_duration = 0;

unsigned long last_bootsel_millis = 0;
bool bootsel_pressed = false;


void loop(){


  //   if (ts + 500uL < millis()) {
  //   ts = millis();


  // }

  if(UART_INTERCHIP.available())
  {
    String msg = UART_INTERCHIP.readStringUntil('\n');
    PhaseCurrent_s current_val0 = current0.getPhaseCurrents();
    PhaseCurrent_s current_val1 = current1.getPhaseCurrents();
    log_string("Current 0: " + String(current_val0.a) + ", " +String(current_val0.c) + ", Current 1: " + String(current_val1.a) + ", " +String(current_val1.c) + ", " + String(read_system_voltage()) + ", " + msg);
    driver0.setPwm(0, 0, DEFAULT_VOLTAGE_POWER_SUPPLY-0.1);
    driver1.setPwm(0, 0, DEFAULT_VOLTAGE_POWER_SUPPLY-0.1);
    // driver0.setPwm(DEFAULT_VOLTAGE_POWER_SUPPLY-0.1, 0, 0);
    // driver1.setPwm(DEFAULT_VOLTAGE_POWER_SUPPLY-0.1, 0, 0);
  }

  // if(UART_INTERCHIP.available())
  if(false)
  {

    uint32_t msglen = UART_INTERCHIP.read();

    if(msglen!=0xFF)
    { 
      uint8_t msgbuffer[msglen+2];
      UART_INTERCHIP.readBytes(msgbuffer, msglen+2);

      uint32_t crc;
      crc =  crc16(msgbuffer, msglen);
      uint32_t crcbuffer = msgbuffer[msglen] | msgbuffer[msglen+1] << 8;


      if (msgbuffer[0] == RAW_BRIDGE_COMMAND && crcbuffer==crc)
      {
        float bridge1 = msgbuffer[1] / 255.0 * 48.0;
        float bridge2 = msgbuffer[2] / 255.0 * 48.0;
        float bridge3 = msgbuffer[3] / 255.0 * 48.0;
        float bridge4 = msgbuffer[4] / 255.0 * 48.0;
        float bridge5 = msgbuffer[5] / 255.0 * 48.0;
        float bridge6 = msgbuffer[6] / 255.0 * 48.0;

        debug_print("Raw bridge command: " + String(bridge1) + " " + String(bridge2) + " " + String(bridge3) + " " + String(bridge4) + " " + String(bridge5) + " " + String(bridge6));
        driver0.setPwm(bridge1, bridge2, bridge3);
        driver1.setPwm(bridge4, bridge5, bridge6);
      }
    }else
    {

      int val1 = UART_INTERCHIP.read();
      int val2 = UART_INTERCHIP.read();
      int val3 = UART_INTERCHIP.read();

      msglen = val1 | val2<<8 | val3<<16;

      uint8_t msgbuffer[msglen];
      UART_INTERCHIP.readBytes(msgbuffer, msglen);
      log_string("Writing OTA image of length " + String(msglen));
      LittleFS.begin();
      File f = LittleFS.open("ota.bin", "w");
      if (msglen != f.write(msgbuffer, msglen)) {
        log_string("Unable to write OTA binary.  Is the filesystem size set?");
        return;
      }
      f.close();
      debug_print("done\n\n");
      debug_print("Programming OTA commands...");
      picoOTA.begin();
      picoOTA.addFile("ota.bin");
      picoOTA.commit();
      LittleFS.end();
      debug_print("Done!");
      debug_print("Rebooting. Should launch with new app...");
      delay(2000);
      rp2040.reboot();
    }
  }
 

  // if (ts + 1000uL < millis()) {
  //   ts = millis();

  //   log_string(String(max_estop_duration));
  //   max_estop_duration = 0;
  // }



}
