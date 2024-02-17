
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
#define DISABLE_ESTOP true
#define MINIMUM_ALLOWED_MOTOR_VOLTAGE 10.0f
#define RESUME_MOTOR_VOLTAGE 12.0f

#define ESTOP_DURATION_MS 200
#define MOTOR_COMMAND_DURATION_MS 500

#define SERIAL_SPEED 921600
#define FIRMWARE_VERSION 100

#define DEFAULT_VOLTAGE_POWER_SUPPLY 40.0f
#define DEFAULT_VOLTAGE_LIMIT 40.0f
#define ALIGNMENT_VOLTAGE_LIMIT 2.0f

#define MOTOR_PP 8
#define MOTOR_KV NOT_SET
//#define MOTOR_PHASE_RESISTANCE 0.2f
#define MOTOR_PHASE_RESISTANCE NOT_SET

#define VDC_CS 2
#define VDC_CLK 3
#define VDC_DATA 23

#define STEERING_COUNTS_TO_RADIANS 1.0f/(1024.0f*0.43f)
#define DEGREES_TO_RADIANS 0.01745329251f 

const byte ACK[] = {0xAC, 0x12};

#define MAXIMUM_ABSOLUTE_DRIVE_VELOCITY 200.0f


#define ERROR_CODE_MOTOR1_OVERSPEED 0x01
#define ERROR_CODE_INVALID_SPEED_COMMAND 0x02
#define ERROR_CODE_INCONSISTENT_COMMS 0x04

#define OVERSPEED_DURATION_ALLOWANCE_MS 1000
#define OVERSPEED_VEL_ALLOWANCE 5.0f



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
#define FIRMWARE_STATUS 12
#define SET_STEERING_ZERO 13
#define CLEAR_ERRORS 14

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

#include <SoftwareSerial.h>


#define LOGGING_DIVIDER '#'
#define LOGGER_LENGTH 2000
#define LOGGER_FULL "LOGGER FULL" + LOGGING_DIVIDER
#define LOGGER_FULL_LEN LOGGER_LENGTH - strlen(LOGGER_FULL)
char logger_string[LOGGER_LENGTH] = "";
uint32_t logger_index = 0;
bool logger_filled = false;


#define PRINT_LOGGING true

#define INDUCTIVE_ENCODER false


// motor 0
// Encoder sensor0 = Encoder(ENCODER0_A_PIN, ENCODER0_B_PIN, 1024, ENCODER0_Z_PIN);

BLDCDriver6PWM driver1 = BLDCDriver6PWM(M1_INUH_PIN, M1_INUL_PIN, M1_INVH_PIN, M1_INVL_PIN, M1_INWH_PIN, M1_INWL_PIN);
HallSensor sensor1 = HallSensor(ENCODER1_A_PIN, ENCODER1_B_PIN, ENCODER1_Z_PIN, MOTOR_PP);
volatile int encoder_pulse_counter = 0;

#ifdef INDUCTIVE_ENCODER
#define ENCODER_RX 21
#define ENCODER_TX 20
SoftwareSerial encoder_software_serial(ENCODER_RX, ENCODER_TX); // RX, TX
uint32_t encoder_ts = 0; // time tick for encoder read in main loop
float encoder_angle = 0.0f;
#else
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
#endif

void handleA1() { sensor1.handleA(); };
void handleB1() { sensor1.handleB(); };
void handleC1() { sensor1.handleC(); };


BLDCDriver6PWM driver0 = BLDCDriver6PWM(M0_INUH_PIN, M0_INUL_PIN, M0_INVH_PIN, M0_INVL_PIN, M0_INWH_PIN, M0_INWL_PIN);

#define SHUNT_RESISTOR_OHMS 1.5/1000.0f // 1.5mohm shunt
#define GAIN 20.0 // 20 volts per volt gain
InlineCurrentSense current1 = InlineCurrentSense(SHUNT_RESISTOR_OHMS, GAIN, M1_AOUTU_PIN, _NC, M1_AOUTW_PIN);



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

float base_motor1_zero_electric_angle = 0.0f;

int last_estop_val = 0;
unsigned long estop_last_millis = 0;
bool motion_allowed = false;
#define ESTOP_SQUARE_INPUT_MOTOR 19

uint8_t error_codes = 0;

// #define ARRAY_SIZE 2000

// float phase_currents[ARRAY_SIZE][9];
// int phase_count = 0;

void debug_print(String text){
  if(UART_DEBUG)
  { 
    // UART_DEBUG.println("Length: ");
    // UART_DEBUG.println(text.length());
    UART_DEBUG.println(text);
    // char char_array[text.length()+1];
    // text.toCharArray(char_array, text.length()+1);
    // UART_DEBUG.println("Char array: ");
    // UART_DEBUG.println(char_array);
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
  if(logger_filled == false && updated_index > LOGGER_FULL_LEN)
  {
    memcpy(logger_string + logger_index, LOGGER_FULL, strlen(LOGGER_FULL));
    logger_filled = true;
  }
  if(!logger_filled)
  {
    memcpy(logger_string + logger_index, text, text_len);
    logger_index = updated_index;
    logger_string[logger_index - 1] = LOGGING_DIVIDER;
  }
  if(PRINT_LOGGING)
  {
    if(logger_filled)
    {
      debug_print(LOGGER_FULL);
    }
    debug_print(text);
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
    log_string("Failed to open file for writing.");
    log_string("Calibration not saved!");
    return;
  }
  file.print(offset);
  file.print(",");
  file.print(direction);
  file.close();
  LittleFS.end();
  log_string("Saved calibration values:");
  log_string("Offset: " + String(offset));
  log_string("Direction: " + String((Direction)direction));
}

void init_motor_from_saved_calibration(BLDCMotor *motor)
// Initializes motor using calibration from SPI Flash with LittleFS
{
  LittleFS.begin();
  File file = LittleFS.open("calibration.txt", "r");
  if (!file) {
    log_string("Failed to open file for reading");
    return;
  }
  String line = file.readStringUntil('\n');
  file.close();
  LittleFS.end();
  int comma = line.indexOf(',');
  float offset = line.substring(0, comma).toFloat();
  int direction = line.substring(comma + 1).toInt();
  log_string("Initializing motor from saved calibration...");
  log_string("Loaded calibration values:");
  log_string("Offset: " + String(offset));
  log_string("Direction: " + String((Direction)direction));
  motor->initFOC(offset, (Direction) direction);
  base_motor1_zero_electric_angle = motor1.zero_electric_angle;
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

void calibrate_motor(char* value)
{
  motor1.sensor_direction = NOT_SET;
  motor1.zero_electric_angle = NOT_SET;
  motor1.initFOC();
  save_calibration(motor1.zero_electric_angle, motor1.sensor_direction);
  base_motor1_zero_electric_angle = motor1.zero_electric_angle;
  SimpleFOCDebug::println(motor1.zero_electric_angle);
  SimpleFOCDebug::println(motor1.sensor_direction);
  SimpleFOCDebug::println(current1.gain_a);
  SimpleFOCDebug::println(current1.gain_b);
  SimpleFOCDebug::println(current1.gain_c);
  SimpleFOCDebug::println("PRINTED VALUES ABOVE");
  // delay(20000);
  // rp2040.reboot();
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

  log_string("Initializing...");

  pinMode(VDC_CS, OUTPUT);
  pinMode(VDC_CLK, OUTPUT);
  pinMode(VDC_DATA, INPUT);
  pinMode(ESTOP_SQUARE_INPUT_MOTOR, INPUT);


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

#ifdef INDUCTIVE_ENCODER
// pinMode(ENCODER_RX, INPUT);
// pinMode(ENCODER_TX, OUTPUT);
encoder_software_serial.begin(57600);
#else
  setup_encoder();
  // sensor0.init();
  // sensor0.enableInterrupts(handleA0, handleB0, handleZ0);
#endif
  sensor1.init();
  sensor1.enableInterrupts(handleA1, handleB1, handleC1);


  #define DRIVE_VEL_LIMIT 500;

  motor1.voltage_limit = DEFAULT_VOLTAGE_LIMIT;
  motor1.voltage_sensor_align = ALIGNMENT_VOLTAGE_LIMIT;
  motor1.velocity_limit = DRIVE_VEL_LIMIT; // 200rad/s is pretty fast
  motor1.PID_velocity.P = 0.3f;
  motor1.PID_velocity.I = 0.0f;
  motor1.PID_velocity.D = 0.01f;
  motor1.PID_velocity.output_ramp = 200.0f;
  motor1.PID_velocity.limit = DRIVE_VEL_LIMIT;
  // motor1.P_angle.P = 0.0f;
  motor1.LPF_velocity.Tf = 0.01f;
  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
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
    // The following two lines are used with our modified SimpleFOC library
    // to allow for two current sensors to be initialized if desired.
    start_adc_engine();
    current1.calibrateOffsets();
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
        calibrate_motor("");
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
  commander.add('C', calibrate_motor, "Calibrate");
  // commander.add('U', onUtil, "Motor utilities");


  log_string("Startup complete.");

  ts = millis();  

  driver0.phase_state[0] = PHASE_ON;
  driver0.phase_state[1] = PHASE_ON;
  driver0.phase_state[2] = PHASE_ON;
  driver0.setPwm(0, 0, 0);
  sensor1.velocity_max = 1000.0f;

}

float read_system_voltage()
{
  float adc_data = spiRead();
  adc_data = (adc_data * 3.3f/4095.0f)/ (5.1/(100+5.1))*(50.0f/49.85f);
  return adc_data;
  // debug_print("adc_data: " + String(adc_data));
}

void send_float_interchip(float value)
{
    uint32_t value_representation = reinterpret_cast<uint32_t&>(value);
    UART_INTERCHIP.write(byte(value_representation & 0xFF));
    UART_INTERCHIP.write(byte(value_representation >> 8 & 0xFF));
    UART_INTERCHIP.write(byte(value_representation >> 16 & 0xFF));
    UART_INTERCHIP.write(byte(value_representation >> 24 & 0xFF));
}

void send_uint_interchip(uint32_t value)
{
    UART_INTERCHIP.write(byte(value & 0xFF));
    UART_INTERCHIP.write(byte(value >> 8 & 0xFF));
    UART_INTERCHIP.write(byte(value >> 16 & 0xFF));
    UART_INTERCHIP.write(byte(value >> 24 & 0xFF));
}

void send_int_interchip(int32_t value)
{
    uint32_t value_representation = reinterpret_cast<uint32_t&>(value);
    send_uint_interchip(value_representation);
}

void send_firmware_status()
{
  //TODO must clearly constrain the bytes involved. eg firmware should be 2 bytes
  // UART_INTERCHIP.write(FIRMWARE_VERSION);
  // UART_INTERCHIP.write(int(DISABLE_ESTOP));
}


uint32_t max_estop_duration = 0;

unsigned long last_bootsel_millis = 0;
bool bootsel_pressed = false;
float system_voltage = 0.0;
uint32_t power_calc_tick = 0;
float motor1_current_avg = 0.0f;
float motor1_voltage_avg = 0.0f;
float motor1_velocity_avg = 0.0f;
float motor1_currents[20];
float motor1_voltages[20];
float motor1_velocities[20];
bool set_motors_enabled = false;
bool set_motors_disabled = false;

unsigned long last_motor_command_millis = 0;

uint32_t loop_count_tick = 0;

unsigned long vel_avg_timer = 0;
unsigned long overspeed_monitor_timer = 0;



void loop(){

  if (BOOTSEL)
  {
    if (!bootsel_pressed)
    {
      bootsel_pressed = true;
      last_bootsel_millis = millis();
    }
    
  } else
  {
    if (bootsel_pressed)
    {
      bootsel_pressed = false;
      if (millis() - last_bootsel_millis > 5000)
      {
        log_string("Bootsel pressed for 5 seconds.  Recalibrating motor.");

        // Make sure motor is enabled.
        driver0.phase_state[0] = PHASE_ON;
        driver0.phase_state[1] = PHASE_ON;
        driver1.enable();

        calibrate_motor("");

        // Disable motor to be safe. Will be enabled by estop toggling if present.
        driver0.setPwm(0, 0, 0);
        driver0.phase_state[0] = PHASE_OFF;
        driver0.phase_state[1] = PHASE_OFF;
        driver1.disable();
        motor1.target = 0.0f;
        motion_allowed = false;
      }
    }

  }

  int estop_val = digitalRead(ESTOP_SQUARE_INPUT_MOTOR);
  if (estop_val != last_estop_val)
  {
    last_estop_val = estop_val;
    estop_last_millis = millis();
  }
  uint32_t estop_duration = millis() - estop_last_millis;
  uint32_t last_motor_command_duration = millis() - last_motor_command_millis;
  if(estop_duration > max_estop_duration)
  {
    max_estop_duration = estop_duration;
  }
  bool motion_tmp;
  if(DISABLE_ESTOP)
  {
    motion_tmp = true;
  } else
  {
    motion_tmp = (estop_duration < ESTOP_DURATION_MS) and (last_motor_command_duration < MOTOR_COMMAND_DURATION_MS);
    if(last_motor_command_duration > MOTOR_COMMAND_DURATION_MS)
    {
      error_codes |= ERROR_CODE_INCONSISTENT_COMMS;
    }
  }
  

  if(motion_tmp!=motion_allowed or set_motors_disabled or set_motors_enabled)
  {
    if((motion_tmp or set_motors_enabled) and not error_codes)
    {
      driver0.phase_state[0] = PHASE_ON;
      driver0.phase_state[1] = PHASE_ON;
      driver0.phase_state[2] = PHASE_ON;
      driver1.enable();
      set_motors_enabled = false;
      motion_allowed = motion_tmp;
    }
    if(!motion_tmp or set_motors_disabled)
    {
      driver0.setPwm(0, 0, 0);
      driver0.phase_state[0] = PHASE_OFF;
      driver0.phase_state[1] = PHASE_OFF;
      driver0.phase_state[2] = PHASE_OFF;
      driver1.disable();
      motor1.target = 0.0f;
      set_motors_disabled = false;
      motion_allowed = motion_tmp;
    }
  }


  power_calc_tick++;
  if(power_calc_tick>10)
  {
    power_calc_tick = 0;
    system_voltage = read_system_voltage();
    driver1.voltage_power_supply = system_voltage;
    if(system_voltage < MINIMUM_ALLOWED_MOTOR_VOLTAGE)
    {
      set_motors_disabled = true;
    }
    if(system_voltage > RESUME_MOTOR_VOLTAGE)
    {
      set_motors_enabled = true;
    }
    // loop through motor1 currents array, shifting each value down one and adding the new value to the end. then calculate the average.
    for(int i = 0; i<19; i++)
    {
      motor1_currents[i] = motor1_currents[i+1];
      motor1_voltages[i] = motor1_voltages[i+1];
    }
    motor1_currents[19] = motor1.current.d;
    motor1_voltages[19] = abs(motor1.voltage.q);
    motor1_current_avg = 0.0f;
    motor1_voltage_avg = 0.0f;
    for(int i = 0; i<20; i++)
    {
      motor1_current_avg += motor1_currents[i];
      motor1_voltage_avg += motor1_voltages[i];
    }
    motor1_current_avg /= 20.0f;
    motor1_voltage_avg /= 20.0f;

  }



  if (motor1.motor_status==FOCMotorStatus::motor_ready)
  {
    // TODO: utilize system voltage in FOC calculations.
    motor1.move();
    motor1.loopFOC();
  }else
  {
    sensor1.update();
  }


#ifdef INDUCTIVE_ENCODER


if (encoder_ts + 100uL <= millis()) {
  // debug_print("SENDING ENCODER REQUEST");
  encoder_ts = millis();
  encoder_software_serial.write(0x55);
  if(encoder_software_serial.available()) 
  {
    uint8_t buffer[2];
    encoder_software_serial.readBytes(buffer, 2);
    int16_t encoder_value = reinterpret_cast<int16_t &>(buffer);
    // int16_t encoder_value = buffer[0] | buffer[1] << 8;
    debug_print("GOT ENCODER VALUE: " + String(encoder_value));
    bool phase_d = encoder_value & 0x01;
    encoder_angle = (encoder_value)/20.0f;
    debug_print("Angle (Deg): " + String(encoder_angle) + " | Phase: " + String(phase_d));



    if (phase_d==0 or encoder_angle * DEGREES_TO_RADIANS > position_target + 0.1)
    {
      driver0.setPwm(0, 24, 24);
      debug_print("Move Forward " + String(encoder_angle * DEGREES_TO_RADIANS) + " " + String(position_target));
    }
    else if (encoder_angle * DEGREES_TO_RADIANS < position_target - 0.1)
    {
      driver0.setPwm(24, 0, 0);
      debug_print("Move Backward " + String(encoder_angle * DEGREES_TO_RADIANS) + " " + String(position_target));
    }else
    {
      driver0.setPwm(0, 0, 0);
    }
    }
}
#else
  // TODO: Add ramping.
  if (encoder_pulse_counter * STEERING_COUNTS_TO_RADIANS > position_target + 0.005)
  {
    driver0.setPwm(0, 24, 0);
    debug_print("Move Forward " + String(encoder_pulse_counter * STEERING_COUNTS_TO_RADIANS) + " " + String(position_target));
  }
  else if (encoder_pulse_counter * STEERING_COUNTS_TO_RADIANS < position_target - 0.005)
  {
    driver0.setPwm(24, 0, 24);
    debug_print("Move Backward " + String(encoder_pulse_counter * STEERING_COUNTS_TO_RADIANS) + " " + String(position_target));
  }else
  {
    driver0.setPwm(0, 0, 0);
  }
#endif


  if(UART_INTERCHIP.available())
  {
    // debug_print("Got Interchip Message");
    uint32_t msglen = UART_INTERCHIP.read();
    // log_string("Got Interchip Message2");
    // log_string("Got Interchip Message of length: " + String(msglen));
    if(msglen!=0xFF)
    { 
      uint8_t msgbuffer[msglen+2];
      UART_INTERCHIP.readBytes(msgbuffer, msglen+2);
      // log_string("Got request of " + String(msglen) + " legnth.");
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
        if(abs(motor2_setpoint) < MAXIMUM_ABSOLUTE_DRIVE_VELOCITY)
        {
          motor1.target = motor2_setpoint;
        } else
        {
          error_codes |= ERROR_CODE_INVALID_SPEED_COMMAND;
        }
        last_motor_command_millis = millis();
        // UART_DEBUG.print(motor1_setpoint);
        // UART_DEBUG.print(" | ");
        // UART_DEBUG.println(crc);
      }
      else if (msgbuffer[0] == REQUEST_SENSORS && crcbuffer==crc)
      {
        send_float_interchip(system_voltage);
        send_float_interchip(motor1_current_avg);
        send_float_interchip(motor1_voltage_avg);
        int32_t drive_motor_position = sensor1.electric_rotations * 6 + sensor1.electric_sector;
        send_int_interchip(drive_motor_position); // Wheel ticks
        send_int_interchip(encoder_pulse_counter); // Steering ticks
        send_float_interchip(motor1_velocity_avg); // Wheel velocity
        UART_INTERCHIP.write(byte(error_codes));
        last_motor_command_millis = millis();

      }
      else if ((msgbuffer[0] & 0x7F) == SEND_BASIC_UPDATE && crcbuffer==crc)
      {
        memcpy (&motor1_setpoint, msgbuffer + 1, 4);
        memcpy (&motor2_setpoint, msgbuffer + 5, 4);
        position_target = motor1_setpoint;
        if(abs(motor2_setpoint) < MAXIMUM_ABSOLUTE_DRIVE_VELOCITY)
        {
          motor1.target = motor2_setpoint;
        } else
        {
          error_codes |= ERROR_CODE_INVALID_SPEED_COMMAND;
        }
        last_motor_command_millis = millis();
        if (msgbuffer[0] & 0x40)
        {
          send_float_interchip(system_voltage);
          send_float_interchip(motor1_current_avg);
          send_float_interchip(motor1_voltage_avg);
          int32_t drive_motor_position = sensor1.electric_rotations * 6 + sensor1.electric_sector;
          send_int_interchip(drive_motor_position); // Wheel ticks
          send_int_interchip(encoder_pulse_counter); // Steering ticks
          send_float_interchip(motor1_velocity_avg); // Wheel velocity
          UART_INTERCHIP.write(byte(error_codes));
        }
      }
      else if (msgbuffer[0] == SIMPLE_PING && crcbuffer==crc)
      {
        UART_INTERCHIP.write(ACK, 2);
        // debug_print("GOT PING REQUEST");
      }
      else if (msgbuffer[0] == CLEAR_ERRORS && crcbuffer==crc)
      {
        debug_print("CLEAR_ERRORS");
        error_codes = 0;
        motor1.target = 0.0f;
      }
      else if (msgbuffer[0] == LOG_REQUEST && crcbuffer==crc)
      {
        UART_INTERCHIP.write(byte(logger_index & 0xFF));
        UART_INTERCHIP.write(byte(logger_index >> 8 & 0xFF));
        UART_INTERCHIP.write(logger_string, logger_index);
        // debug_print("GOT LOG REQUEST");
        // debug_print("Sent " + String(logger_index) + " bytes: " + String(byte(logger_index & 0xFF)) + " " + String(byte(logger_index >> 8 & 0xFF)));
        logger_index = 0;
        logger_filled = false;
      }
      else if (msgbuffer[0] == FIRMWARE_STATUS && crcbuffer==crc)
      {
        send_firmware_status();
      }
      else if (msgbuffer[0] == SET_STEERING_ZERO && crcbuffer==crc)
      {
        encoder_pulse_counter = 0;
        position_target = 0.0f;
        UART_INTERCHIP.write(ACK, 2);
      }

    }else
    {

      int val1 = UART_INTERCHIP.read();
      int val2 = UART_INTERCHIP.read();
      int val3 = UART_INTERCHIP.read();


      // debug_print("val debug: " + String(val1) + " " + String(val2) + " " + String(val3));

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
  motor1.PID_velocity.limit = DRIVE_VEL_LIMIT;

  // Drive motor velocity averaging
  if (vel_avg_timer + 10uL < millis()) {
    vel_avg_timer = millis();
    for(int i = 0; i<19; i++)
    {
      motor1_velocities[i] = motor1_velocities[i+1];
    }
    motor1_velocities[19] = sensor1.getVelocity();
    motor1_velocity_avg = 0.0f;
    for(int i = 0; i<20; i++)
    {
      motor1_velocity_avg += motor1_velocities[i];
    }
    motor1_velocity_avg /= 20.0f;
    // debug_print(String(motor1_velocity_avg));
}

  // Overspeed monitoring
  if(abs(motor1_velocity_avg) - OVERSPEED_VEL_ALLOWANCE > abs(motor1.target))
  {
    // debug_print("Overspeed detected!");
    if(overspeed_monitor_timer == 0)
    {
      overspeed_monitor_timer = millis();
    } else if (millis() - overspeed_monitor_timer > OVERSPEED_DURATION_ALLOWANCE_MS)
    {
      set_motors_disabled = true;
      error_codes |= ERROR_CODE_MOTOR1_OVERSPEED;
      log_string("Motor 1 overspeed error!");
    }
  } else
  {
    if(overspeed_monitor_timer != 0)
    {
      debug_print("Overspeed cleared! " + String(millis() - overspeed_monitor_timer) + " ms");
    }
    overspeed_monitor_timer = 0;
  }




  // if (ts + 1000uL <= millis()) {
  //   ts = millis();
  //   // String myStr;    /*New string is defined*/
  //   // myStr = String(max_estop_duration);   /*Convert Int to String*/
  //   log_string(String(max_estop_duration));
  //   max_estop_duration = 0;
  // }



//     // count++;
    loop_count_tick++;
    if (ts + 10uL < millis()) {
      // DQCurrent_s current = current1.getFOCCurrents(motor1.electrical_angle);
      // PhaseCurrent_s phase_currents = current1.getPhaseCurrents();
      // // // if(halls!=lasthalls) {
      // // // lasthalls = halls;
      // int32_t drive_motor_position = sensor1.electric_rotations * 6 + sensor1.electric_sector;
      // UART_DEBUG.println(String(sensor1.getVelocity()));
      // // // float velocity = 10 *(last_position - position);
      // // // last_position = position;
      ts = millis();
      // // Serial.println(encoder_pulse_counter/1024.0f);
      // UART_DEBUG.print(phase_currents.a);
      // UART_DEBUG.print(" ");
      // UART_DEBUG.print(phase_currents.b);
      // UART_DEBUG.print(" ");
      // UART_DEBUG.print(phase_currents.c);
      // UART_DEBUG.print(" ");
      // UART_DEBUG.print(current.d);
      // UART_DEBUG.print(".");
      // UART_DEBUG.println(current.q);
      // // UART_DEBUG.println(String(motor1.current.d * motor1_voltage_avg) + " | " + String(motor1.current.q + system_voltage));
      // // UART_DEBUG.println(String(motor1_voltage_avg * motor1_current_avg) + " watts, " + String(loop_count_tick * 5) + " Hz");
      // loop_count_tick = 0;
      // PhaseCurrent_s phase_currents = current1.getPhaseCurrents();
      // float wattage = motor1.current.d * system_voltage;
      // UART_DEBUG.print("Volts: " + String(system_voltage) + ", Current: " + String(motor1.current.d) + ", Watts: " + String(wattage) + ";\r\n");
      // UART_DEBUG.print(String(phase_currents.a) + ", " + String(phase_currents.b) + ", " + String(phase_currents.c) + ";\r\n");
}








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
