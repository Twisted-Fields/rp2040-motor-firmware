
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


#define PRE_INITIALIZED_STARTUP false


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


// motor 0
// Encoder sensor0 = Encoder(ENCODER0_A_PIN, ENCODER0_B_PIN, 1024, ENCODER0_Z_PIN);


volatile int encoder_pulse_counter = 0;
volatile bool a_active = false;
volatile bool b_active = false;


void handleA0() { 
bool A = digitalRead(ENCODER0_A_PIN);
if(A!=a_active){
  encoder_pulse_counter += (a_active == b_active) ? 1 : -1;
  a_active = A;
};
};

void handleB0() { 
bool B = digitalRead(ENCODER0_B_PIN);
if(B!=b_active){
  encoder_pulse_counter += (b_active != a_active) ? 1 : -1;
  b_active = B;
};
};

void setup_encoder()
{
pinMode(ENCODER0_A_PIN, INPUT);
pinMode(ENCODER0_B_PIN, INPUT);
attachInterrupt(digitalPinToInterrupt(ENCODER0_A_PIN), handleA0, CHANGE);
attachInterrupt(digitalPinToInterrupt(ENCODER0_B_PIN), handleB0, CHANGE);
}


BLDCDriver6PWM driver1 = BLDCDriver6PWM(M1_INUH_PIN, M1_INUL_PIN, M1_INVH_PIN, M1_INVL_PIN, M1_INWH_PIN, M1_INWL_PIN);


// motor 1
HallSensor sensor1 = HallSensor(ENCODER1_A_PIN, ENCODER1_B_PIN, ENCODER1_Z_PIN, MOTOR_PP);

void handleA1() { sensor1.handleA(); };
void handleB1() { sensor1.handleB(); };
void handleC1() { sensor1.handleC(); };



BLDCDriver6PWM driver0 = BLDCDriver6PWM(M0_INUH_PIN, M0_INUL_PIN, M0_INVH_PIN, M0_INVL_PIN, M0_INWH_PIN, M0_INWL_PIN);
#define CURRENT_VpA 50.0f
#define GAIN 1.0/(CURRENT_VpA/(3.3/2))
InlineCurrentSense current1 = InlineCurrentSense(1.0f, GAIN, M1_AOUTU_PIN, _NC, M1_AOUTW_PIN);



BLDCMotor motor1 = BLDCMotor(MOTOR_PP, MOTOR_PHASE_RESISTANCE, MOTOR_KV);


#define UART_DEBUG Serial2
#define UART_INTERCHIP Serial1

// Commander
Commander commander = Commander(UART_DEBUG, '\n', false);
void onMotor0(char* cmd){commander.motor(&motor1, cmd);}
// void onUtil(char* cmd){dispatch_util_single(cmd, &motor1);}


// main loop speed tracking
int count = 0;
unsigned long ts = 0;

float position_target = 0;
int  tick = 0;

float last_sensor_angle = 0.0;
float max_angle_diff = 0.0;

int lasthalls = 0;
float last_position = 0;

// #define ARRAY_SIZE 2000

// float phase_currents[ARRAY_SIZE][9];
// int phase_count = 0;

void debug_print(String text){
  if(UART_DEBUG)
  {
    UART_DEBUG.println(text);
  }
}

void update_motor_setpoint(char* setpoint)
{
    position_target = atof(strtok (setpoint, " "));
    SimpleFOCDebug::print("Position Target: ");
    SimpleFOCDebug::println(position_target);
}

void save_calibration(float offset, int direction)
// Saves the calibration to SPI Flash using LittleFS
{
  LittleFS.begin();
  File file = LittleFS.open("calibration.txt", "w");
  if (!file) {
    debug_print("Failed to open file for writing.");
    debug_print("Calibration not saved!");
    return;
  }
  file.print(offset);
  file.print(",");
  file.print(direction);
  file.close();
  LittleFS.end();
  debug_print("Saved calibration values:");
  debug_print("Offset: " + String(offset));
  debug_print("Direction: " + String((Direction)direction));
}

void init_motor_from_saved_calibration(BLDCMotor *motor)
// Initializes motor using calibration from SPI Flash with LittleFS
{
  LittleFS.begin();
  File file = LittleFS.open("calibration.txt", "r");
  if (!file) {
    debug_print("Failed to open file for reading");
    return;
  }
  String line = file.readStringUntil('\n');
  file.close();
  LittleFS.end();
  int comma = line.indexOf(',');
  float offset = line.substring(0, comma).toFloat();
  int direction = line.substring(comma + 1).toInt();
  debug_print("Initializing motor from saved calibration...");
  debug_print("Loaded calibration values:");
  debug_print("Offset: " + String(offset));
  debug_print("Direction: " + String((Direction)direction));
  motor->initFOC(offset, (Direction) direction);
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


// #define DEBUGPIN 18

void setup() {
  // UART_DEBUG.begin(SERIAL_SPEED);
  // Serial.begin(SERIAL_SPEED);
  // UART_INTERCHIP.begin(SERIAL_SPEED);
  // pinMode(DEBUGPIN, OUTPUT);
  // digitalWrite(DEBUGPIN, LOW);


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


  commander.verbose = VerboseMode::user_friendly;

  // SimpleFOCDebug::enable(&Serial);
  SimpleFOCDebug::enable(&UART_DEBUG);
  // delay(1000);
  // while (!Serial) ; // wait for serial port to connect - remove this later
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

  motor1.linkDriver(&driver1);

  setup_encoder();
  // sensor0.init();
  // sensor0.enableInterrupts(handleA0, handleB0, handleZ0);
  sensor1.init();
  sensor1.enableInterrupts(handleA1, handleB1, handleC1);

  motor1.voltage_limit = DEFAULT_VOLTAGE_LIMIT;
  motor1.voltage_sensor_align = ALIGNMENT_VOLTAGE_LIMIT;
  motor1.velocity_limit = 200.0f; // 200rad/s is pretty fast
  motor1.PID_velocity.P = 0.5f;
  motor1.PID_velocity.I = 0.0f;
  motor1.PID_velocity.D = 0.00f;
  motor1.PID_velocity.output_ramp = 100.0f;
  motor1.PID_velocity.limit = 200.0f;
  // motor1.P_angle.P = 0.0f;
  motor1.LPF_velocity.Tf = 0.01f;
  motor1.foc_modulation = FOCModulationType::SinePWM;
  motor1.controller = MotionControlType::velocity;
  motor1.torque_controller = TorqueControlType::foc_current;
  motor1.motion_downsample = 5;
  motor1.modulation_centered = 0;
  motor1.target = 0.0f;

  // foc current control parameters
  motor1.PID_current_q.P = 1;
  motor1.PID_current_q.I = 0;
  motor1.PID_current_d.P = 1;
  motor1.PID_current_d.I = 0;
  motor1.LPF_current_q.Tf = 0.01; 
  motor1.LPF_current_d.Tf = 0.01; 

  if (driver1.initialized) {
    current1.linkDriver(&driver1);
    motor1.init();
    SimpleFOCDebug::println("Initializing current sense 0...");
    if (current1.init()!=1)
      SimpleFOCDebug::println("Current sense 0 init failed!");
    else
      motor1.linkCurrentSense(&current1);
    if (motor1.motor_status==FOCMotorStatus::motor_uncalibrated) {
      motor1.linkSensor(&sensor1);
      SimpleFOCDebug::println("Initializing FOC motor 1...");
 

      if(PRE_INITIALIZED_STARTUP)
      {
        current1.gain_a *= -1;
        current1.gain_c *= -1;
        // skip alignment procedure
        current1.skip_align = true;
        init_motor_from_saved_calibration(&motor1);
        // motor1.initFOC(5.24, Direction::CW);

      } else
      {
        motor1.initFOC();
        save_calibration(motor1.zero_electric_angle, motor1.sensor_direction);
        SimpleFOCDebug::println(motor1.zero_electric_angle);
        SimpleFOCDebug::println(motor1.sensor_direction);
        SimpleFOCDebug::println(current1.gain_a);
        SimpleFOCDebug::println(current1.gain_b);
        SimpleFOCDebug::println(current1.gain_c);
        SimpleFOCDebug::println("PRINTED VALUES ABOVE");
        // delay(20000);
        // rp2040.reboot();

      }
      
      SimpleFOCDebug::println("Motor 1 intialized.");
    }
    else
      SimpleFOCDebug::println("Motor 1 init failed!");
  }
  else
    SimpleFOCDebug::println("Motor 1 not initialized.");



  commander.add('M', onMotor0, "Drive Motor");
  commander.add('N', update_motor_setpoint, "Brushed Motor");
  // commander.add('U', onUtil, "Motor utilities");


  SimpleFOCDebug::println("Startup complete.");
  ts = millis();  

  driver0.phase_state[0] = PHASE_ON;
  driver0.phase_state[1] = PHASE_ON;
  driver0.setPwm(0, 0, 0);
  sensor1.velocity_max = 10000.0f;

}

float read_system_voltage()
{

    float adc_data = spiRead();
    adc_data = (adc_data * 3.3f/4095.0f)/ (5.1/(100+5.1))*(50.0f/49.85f);
    return adc_data;
    // debug_print("adc_data: " + String(adc_data));

}

void loop(){

  if (motor1.motor_status==FOCMotorStatus::motor_ready)
  {
    motor1.move();
    motor1.loopFOC();
  }else
  {
    sensor1.update();
  }
  // sensor0.update();

    

//     if (ts + 50uL < millis()) {

//     ts = millis();
//     float adc_data = spiRead();
//     adc_data = (adc_data * 3.3f/4095.0f)/ (5.1/(100+5.1))*(50.0f/49.85f);
//     debug_print("adc_data: " + String(adc_data));

// }


  // TODO: Add ramping.
  if (encoder_pulse_counter/1024.0f < position_target - 0.005)
  {
    driver0.setPwm(0, 24, 0);
    //SimpleFOCDebug::println("Move Forward");
  }
  else if (encoder_pulse_counter/1024.0f > position_target + 0.005)
  {
    driver0.setPwm(24, 0, 24);
    //SimpleFOCDebug::println("Move Backward");
  }else
  {
    driver0.setPwm(0, 0, 0);
  }


  if(UART_INTERCHIP.available())
  {
    uint32_t msglen = UART_INTERCHIP.read();
    if(msglen!=0xFF)
    { 
      uint8_t msgbuffer[msglen+2];
      UART_INTERCHIP.readBytes(msgbuffer, msglen+2);
      // for(int i = 0; i<msglen; i++)
      // {
      //   UART_DEBUG.print(msgbuffer[i]);
      //   UART_DEBUG.print(" ");
      // }
      //   UART_DEBUG.println();

      uint32_t crc;
      crc =  crc16(msgbuffer, msglen);
      uint32_t crcbuffer = msgbuffer[msglen] | msgbuffer[msglen+1] << 8;

      // UART_DEBUG.print(crcbuffer);
      // UART_DEBUG.println(crc);
      if (msgbuffer[0] == SEND_COMPLETE_SETTINGS && crcbuffer==crc)
      {
        motor1_setpoint_mode = msgbuffer[1];
        memcpy (&motor1_setpoint, msgbuffer + 2, 4);
        memcpy (&motor1_max_velocity, msgbuffer + 6, 4);
        memcpy (&motor1_max_torque, msgbuffer + 10, 4);
        memcpy (&motor1_acceleration, msgbuffer + 14, 4);
        motor2_setpoint_mode = msgbuffer[18];
        memcpy (&motor2_setpoint, msgbuffer + 19, 4);
        memcpy (&motor2_max_velocity, msgbuffer + 23, 4);
        memcpy (&motor2_max_torque, msgbuffer + 27, 4);
        memcpy (&motor2_acceleration, msgbuffer + 31, 4);

        position_target = motor1_setpoint;
        motor1.target = motor2_setpoint;
        // UART_DEBUG.print(motor1_setpoint);
        // UART_DEBUG.print(" | ");
        // UART_DEBUG.println(crc);
      }
      if (msgbuffer[0] == REQUEST_SENSORS && crcbuffer==crc)
      {
          float system_voltage = read_system_voltage();
          uint32_t voltage_representation = reinterpret_cast<uint32_t&>(system_voltage);
          // debug_print("voltage: " + String(system_voltage) + "[" + String(reinterpret_cast<unsigned char *>(&system_voltage)[0]) + "]"+ "[" + String(reinterpret_cast<unsigned char *>(&system_voltage)[1]) + "]");
          UART_INTERCHIP.write(byte(voltage_representation & 0xFF));
          UART_INTERCHIP.write(byte(voltage_representation >> 8 & 0xFF));
          UART_INTERCHIP.write(byte(voltage_representation >> 16 & 0xFF));
          UART_INTERCHIP.write(byte(voltage_representation >> 24 & 0xFF));

      }
      if ((msgbuffer[0] & 0x7F) == SEND_BASIC_UPDATE && crcbuffer==crc)
      {
        memcpy (&motor1_setpoint, msgbuffer + 1, 4);
        memcpy (&motor2_setpoint, msgbuffer + 5, 4);
        if (msgbuffer[0] & 0x40)
        {
          float system_voltage = read_system_voltage();
          // unsigned char*voltage_bytes = reinterpret_cast<unsigned char *>(&system_voltage)[0];
          uint32_t voltage_representation = reinterpret_cast<uint32_t&>(system_voltage);
          UART_INTERCHIP.write(byte(voltage_representation & 0xFF));
          UART_INTERCHIP.write(byte(voltage_representation >> 8 & 0xFF));
          UART_INTERCHIP.write(byte(voltage_representation >> 16 & 0xFF));
          UART_INTERCHIP.write(byte(voltage_representation >> 24 & 0xFF));
          //debug_print("voltage: " + String(system_voltage) + "[" + String(byte(voltage_representation & 0xFF)) + "]"+ "[" + String(byte(voltage_representation >> 8 & 0xFF)) + "]"+ "[" + String(byte(voltage_representation >> 16 & 0xFF)) + "]"+ "[" + String(byte(voltage_representation >> 24 & 0xFF)) + "]");
        }
      }
      if (msgbuffer[0] == SIMPLE_PING && crcbuffer==crc)
      {
        UART_INTERCHIP.write(ACK, 2);
      }
    }else
    {

      int val1 = UART_INTERCHIP.read();
      int val2 = UART_INTERCHIP.read();
      int val3 = UART_INTERCHIP.read();


      debug_print("val debug: " + String(val1) + " " + String(val2) + " " + String(val3));

      msglen = val1 | val2<<8 | val3<<16;

      uint8_t msgbuffer[msglen];
      UART_INTERCHIP.readBytes(msgbuffer, msglen);
      debug_print("Writing OTA image of length " + String(msglen));
      LittleFS.begin();
      File f = LittleFS.open("ota.bin", "w");
      if (msglen != f.write(msgbuffer, msglen)) {
        debug_print("Unable to write OTA binary.  Is the filesystem size set?");
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
  motor1.PID_velocity.limit = 200.0f;



//     count++;
//     if (ts + 25uL < millis()) {
//       DQCurrent_s current = current1.getFOCCurrents(motor1.electrical_angle);
//       PhaseCurrent_s phase_currents = current1.getPhaseCurrents();
//       // if(halls!=lasthalls) {
//       // lasthalls = halls;
//       // float position = sensor1.getAngle();
//       // float velocity = 10 *(last_position - position);
//       // last_position = position;
//       ts = millis();
//       Serial.println(encoder_pulse_counter/1024.0f);
//       // Serial.print(phase_currents.a);
//       // Serial.print(" ");
//       // Serial.print(phase_currents.b);
//       // Serial.print(" ");
//       // Serial.print(phase_currents.c);
//       // Serial.print(" ");
//       // Serial.print(current.d);
//       // Serial.print(" ");
//       // Serial.println(current.q);
// }






  // count++;
  // if (ts + 1000uL < millis()) {
  //   // if(halls!=lasthalls) {
  //     // lasthalls = halls;

  //   float position = sensor1.getAngle();

  //   float velocity = 10 *(last_position - position);
  //   last_position = position;

  //   ts = millis();
  //   SimpleFOCDebug::print("loop/s: ");
  //   SimpleFOCDebug::print(int(count));
  //   SimpleFOCDebug::print(" angle0: ");
  //   SimpleFOCDebug::print(sensor0.getAngle());
  //   SimpleFOCDebug::print(" angle1: ");
  //   SimpleFOCDebug::print(sensor1.getAngle());
  //   SimpleFOCDebug::print(" velocity1: ");
  //   SimpleFOCDebug::print(sensor1.getVelocity());
  //   // SimpleFOCDebug::print(" avg velocity: ");
  //   // SimpleFOCDebug::print(velocity);
  //   // SimpleFOCDebug::print(" A2: ");
  //   // SimpleFOCDebug::print(analogRead(A2));
  //   // SimpleFOCDebug::print(" A3: ");
  //   // SimpleFOCDebug::print(analogRead(A3));
  //   // // SimpleFOCDebug::print(" angle_diff: ");
  //   // // SimpleFOCDebug::print(angle_diff);
  //   // SimpleFOCDebug::print(" max_angle_diff: ");
  //   // SimpleFOCDebug::print(max_angle_diff);
  //   // SimpleFOCDebug::print(" |  dc_a: ");
  //   // SimpleFOCDebug::print(driver1.dc_a);
  //   // SimpleFOCDebug::print(" |  dc_b: ");
  //   // SimpleFOCDebug::print(driver1.dc_b);
  //   // SimpleFOCDebug::print(" |  dc_c: ");
  //   // SimpleFOCDebug::print(driver1.dc_c);
  //   // SimpleFOCDebug::print(" |  motor1.voltage.d: ");
  //   // SimpleFOCDebug::print(motor1.voltage.d);
  //   // SimpleFOCDebug::print(" |  motor1.voltage.q: ");
  //   // SimpleFOCDebug::print(motor1.voltage.q);
  //   // SimpleFOCDebug::print(" | HALLS: ");
  //   // SimpleFOCDebug::print(pinA);
  //   // SimpleFOCDebug::print("-");
  //   // SimpleFOCDebug::print(pinB);
  //   // SimpleFOCDebug::print("-");
  //   // SimpleFOCDebug::print(pinC);
  //   // SimpleFOCDebug::print(" | ");
  //   // SimpleFOCDebug::print(halls);
  //   // SimpleFOCDebug::print(" | ");
  //   // for(int i=0;i<halls;i++)
  //   // {
  //   //   SimpleFOCDebug::print("<=============>");
  //   // }
  //   // SimpleFOCDebug::println();
  //   SimpleFOCDebug::print(", sensor1.electric_sector: ");
  //   SimpleFOCDebug::println(int(sensor1.electric_sector));

    
  //   count = 0;
  // }



  commander.run();
}