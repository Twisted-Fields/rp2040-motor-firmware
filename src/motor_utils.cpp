#include "SimpleFOC.h"

#define SEARCH_ANGLE (6*_PI)




// adapted from SimpleFOC pole count example
int estimate_pole_count(BLDCMotor* motor) {
  int orig_pp = motor->pole_pairs;
  float orig_voltage_limit = motor->voltage_limit;
  MotionControlType orig_controller = motor->controller;

  // move motor to the electrical angle 0
  motor->pole_pairs = 1;
  motor->controller = MotionControlType::angle_openloop;
  motor->voltage_limit = motor->voltage_sensor_align;
  motor->enable();
  motor->move(0);
  _delay(1000);

  // read the sensor angle
  motor->sensor->update();
  float angle_begin = motor->sensor->getAngle();

  // move the motor slowly to the electrical angle pp_search_angle
  float motor_angle = 0;
  while(motor_angle <= SEARCH_ANGLE){
    motor_angle += 0.01f;
    motor->sensor->update();
    motor->move(motor_angle);
    _delay(5);
  }
  _delay(1000);

  // read the sensor value for 180
  motor->sensor->update();
  float angle_end = motor->sensor->getAngle();

  // restore previous state
  motor->disable();
  motor->pole_pairs = orig_pp;
  motor->controller = orig_controller;
  motor->voltage_limit = orig_voltage_limit;


  // calculate the pole pair number
  int pp = round((SEARCH_ANGLE)/(angle_end-angle_begin));

  SimpleFOCDebug::println(F("Estimated PP : "), pp);
  SimpleFOCDebug::println(F("PP = Electrical angle / Encoder angle "));
  SimpleFOCDebug::print(SEARCH_ANGLE*180.0f/_PI);
  SimpleFOCDebug::print(F("/"));
  SimpleFOCDebug::print((angle_end-angle_begin)*180.0f/_PI);
  SimpleFOCDebug::print(F(" = "));
  SimpleFOCDebug::println(SEARCH_ANGLE/(angle_end-angle_begin));

  return pp;
}



void dispatch_util(char* cmd, BLDCMotor* motor0, BLDCMotor* motor1) {
  if (cmd[0] == 'P'){
    if (cmd[1] == '0'){
      estimate_pole_count(motor0);
    }
    else if (cmd[1] == '1'){
      estimate_pole_count(motor1);
    }
  }
}

void dispatch_util_single(char* cmd, BLDCMotor* motor0) {
  if (cmd[0] == 'P'){
    if (cmd[1] == '0'){
      estimate_pole_count(motor0);
    }
    
  }
}
