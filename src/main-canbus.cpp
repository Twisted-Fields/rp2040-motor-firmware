

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

#define MCP_CS 9
#define MCP_INT 4
#define SPI1_MISO 8
#define SPI1_MOSI 11
#define SPI1_SCLK 10


MCP_CAN CAN0(&SPI1, MCP_CS);
IsoTp isotp(&CAN0, MCP_INT);


#define THERM_ADC_CLK 2
#define THERM_ADC_DATA 0
#define THERM_ADC_CS1 1
#define THERM_ADC_CS2 3


struct Message_t txMsg, rxMsg;

#define CAN_BASE_ADDRESS 0x5

uint32_t tx_can_id = 0x1;
uint32_t rx_can_id = 0x0;

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

#define SEND_COMPLETE_SETTINGS 1
#define REQUEST_COMPLETE_SETTINGS 2
#define SEND_BASIC_UPDATE 3
#define REQUEST_SENSORS 4
#define REQUEST_HOMING 5
#define SIMPLEFOC_PASS_THROUGH 6
#define FIRMWARE_UPDATE 7
#define FIRMWARE_UPDATE_CPU2 8
#define SIMPLE_PING 9

const byte ACK[] = {0xAC, 0x12};

#define CAN_ID_1 18
#define CAN_ID_2 19
#define CAN_ID_3 23
#define ESTOP_SWITCH_STATUS 16
#define ESTOP_SQUARE_INPUT 14
#define AUX_1_GPIO 6
#define AUX_2_GPIO 7
#define ADC_AUX_1 A0
#define ADC_AUX_2 A1
#define ADC_MOTOR_TH1 A2
#define ADC_MOTOR_TH2 A3
#define LED_PIN 15
#define LED_COUNT 3

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

int sendcount = 0;
int readcount = 0;


#define SERIAL_SPEED 921600


// #define USE_USBSERIAL
#define WAIT_USBSERIAL false

#ifdef USE_USBSERIAL
#define UART_DEBUG Serial
#else
#define UART_DEBUG Serial2
#endif

#define UART_INTERCHIP Serial1

void debug_print(String text){
  if(UART_DEBUG)
  {
    UART_DEBUG.println(text);
  }
}

void exception_handler()
{
  rp2040.reboot();
}

void setRingRed(bool off) {
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(off ? 0 : 255, 0, 0));
  }
  strip.show();
}

void setLEDs(uint32_t color) {
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
}

void setup()
{

  // CAN0.mcp2515_reset();

  strip.begin();
  strip.show();
  strip.setBrightness(50);

  setLEDs(strip.Color(255,0,0));
  delay(100);
  setLEDs(strip.Color(0,255,0));
  delay(100);
  setLEDs(strip.Color(0,0,255));
  delay(100);

  exception_set_exclusive_handler(HARDFAULT_EXCEPTION, exception_handler);

  #ifdef USE_USBSERIAL
    UART_DEBUG.begin(SERIAL_SPEED);

  
  if(WAIT_USBSERIAL)
  {
    while(!UART_DEBUG);
  }
  #endif
  debug_print("Initializing...");

#ifndef USE_USBSERIAL
  UART_DEBUG.setRX(25);
  UART_DEBUG.setTX(24);
  UART_DEBUG.setFIFOSize(128);
  UART_DEBUG.begin(SERIAL_SPEED);
#endif

  UART_INTERCHIP.setRX(13);
  UART_INTERCHIP.setTX(12);
  UART_INTERCHIP.setFIFOSize(128);
  UART_INTERCHIP.begin(SERIAL_SPEED);

  SPI1.setRX(SPI1_MISO);
  SPI1.setSCK(SPI1_SCLK);
  SPI1.setTX(SPI1_MOSI);

  pinMode(MCP_INT, INPUT);

  pinMode(AUX_1_GPIO, INPUT);
  pinMode(AUX_2_GPIO, INPUT);
  pinMode(ESTOP_SQUARE_INPUT, INPUT);
  pinMode(ESTOP_SWITCH_STATUS, INPUT);
  pinMode(ADC_AUX_1, INPUT);
  pinMode(ADC_AUX_2, INPUT);
  pinMode(ADC_MOTOR_TH1, INPUT);
  pinMode(ADC_MOTOR_TH2, INPUT);

  pinMode(CAN_ID_1, INPUT);
  pinMode(CAN_ID_2, INPUT);
  pinMode(CAN_ID_3, INPUT);
  
  rx_can_id = CAN_BASE_ADDRESS + (digitalRead(CAN_ID_1)<<2 | digitalRead(CAN_ID_2)<<1 | digitalRead(CAN_ID_3));

  debug_print("Setting CAN ID " + String(rx_can_id));

  CAN0.begin(MCP_STDEXT, CAN_500KBPS, MCP_16MHZ);
  CAN0.setMode(MCP_NORMAL);

  // CAN0.init_Mask(0,0,0x07FF0000);                // Init first mask...
  // CAN0.init_Filt(0,0,rx_can_id<<16);                // Init first filter...  
  // CAN0.init_Filt(1,0,rx_can_id<<16);                // Init second filter...
  
  // CAN0.init_Mask(1,0,0x07FF0000);                // Init second mask... 
  // CAN0.init_Filt(2,0,rx_can_id<<16);                // Init third filter...
  // CAN0.init_Filt(3,0,rx_can_id<<16);                // Init fouth filter...
  // CAN0.init_Filt(4,0,rx_can_id<<16);                // Init fifth filter...
  // CAN0.init_Filt(5,0,rx_can_id<<16);                // Init sixth filter...


  CAN0.init_Mask(0,0,0x07FF0000);                // Init first mask...
  CAN0.init_Filt(0,0,rx_can_id << 16);                // Init first filter...
  CAN0.init_Filt(1,0,rx_can_id << 16);                // Init second filter...
  
  CAN0.init_Mask(1,0,0x07FF0000);                // Init second mask... 
  CAN0.init_Filt(2,0,rx_can_id << 16);                // Init third filter...
  CAN0.init_Filt(3,0,rx_can_id << 16);                // Init fourth filter...
  CAN0.init_Filt(4,0,rx_can_id << 16);                // Init fifth filter...
  CAN0.init_Filt(5,0,rx_can_id << 16);                // Init sixth filter...


  txMsg.Buffer = (uint8_t *)calloc(1000, sizeof(uint8_t));
  rxMsg.Buffer = (uint8_t *)calloc(MAX_MSGBUF, sizeof(uint8_t));

  rxMsg.tx_id = tx_can_id;
  rxMsg.rx_id = rx_can_id;

  // NOTE: We don't have any SPI0 TX pin physically connected on the board,
  // but it is unclear what pin the code will try to use if we don't specify one.
  // The SPI.begin code connects the SPI peripheral to the physical pin*, but
  // after that we can simply disconnect it. So here we use THERM_ADC_CS2 (GPIO3)
  // which is a valid SPI TX pin, and then after calling begin we will set that pin
  // back to a GPIO pin with the pinMode call, to properly connect it as CS2. This
  // should have the result that anything transmitted on SPI0 TX goes nowhere.
  // * https://github.com/earlephilhower/arduino-pico/blob/3778fbb833918c9a09433f85785456ec074edb22/libraries/SPI/src/SPI.cpp#L275
  SPI.setRX(THERM_ADC_DATA);
  SPI.setSCK(THERM_ADC_CLK);
  SPI.setTX(THERM_ADC_CS2);
  SPI.begin();
  pinMode(THERM_ADC_CS1, OUTPUT);
  pinMode(THERM_ADC_CS2, OUTPUT);
  digitalWrite(THERM_ADC_CS1, HIGH);
  digitalWrite(THERM_ADC_CS2, HIGH);
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

  debug_print("Setup complete.");
}

void print_buffer(INT32U id, uint8_t *buffer, uint32_t len, SerialUART uart)
{
  uint16_t i=0;

  uart.print(F("Buffer: "));
  uart.print(id,HEX); uart.print(F(" ["));
  uart.print(len); uart.print(F("] "));
  for(i=0;i<len;i++)
  {
    if(buffer[i] < 0x10) uart.print(F("0"));
    uart.print(buffer[i],HEX);
    uart.print(F(" "));
  }
  uart.println();
}


int count = 0;
int last_estop_val = 0;
unsigned long estop_last_millis = 0;
bool motion_allowed = true;

long ts = millis();

void send_buffer_interchip()
{
    uint32_t crc;
    crc =  crc16(rxMsg.Buffer, rxMsg.len);
    UART_INTERCHIP.write(rxMsg.len);
    UART_INTERCHIP.write(rxMsg.Buffer, rxMsg.len);
    UART_INTERCHIP.write((uint8_t)(0xFF & crc));
    UART_INTERCHIP.write((uint8_t)(0xFF & crc>>8));
}

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

void send_ping_reply(bool got_motor_ack)
{
    txMsg.Buffer[0] = rx_can_id;
    txMsg.Buffer[1] = SIMPLE_PING | 0x80;
    txMsg.Buffer[2] = 0xA | (got_motor_ack * 0xA0);
    txMsg.len = 3;
    txMsg.tx_id = tx_can_id;
    txMsg.rx_id = rx_can_id;
    sendcount += 1;
    isotp.send(&txMsg);
}

void send_sensors_can(float system_voltage)
{
      for (int i = 26; i < 30; ++i) {
      gpio_disable_pulls(i);
      gpio_set_input_enabled(i, false);
    }
    // Serial.print("system_voltage: ");
    // Serial.println(system_voltage);
    debug_print("system_voltage " + String(system_voltage));
    // Serial.print(", Sendcount: ");
    // Serial.println(sendcount);
    uint16_t adc1 = analogRead(ADC_AUX_1);
    uint16_t adc2 = analogRead(ADC_AUX_2);
    uint16_t motor_th1 = analogRead(ADC_MOTOR_TH1);
    uint16_t motor_th2 = analogRead(ADC_MOTOR_TH2);
    float motor1_temp = thermistor_calc(motor_th1, 1024);
    float motor2_temp = thermistor_calc(motor_th2, 1024);
    // Serial.print("ADC1: ");
    // Serial.print(adc1);
    // Serial.print(", ADC2: ");
    // Serial.println(adc2);
    int aux1 = digitalRead(AUX_1_GPIO);
    int aux2 = digitalRead(AUX_2_GPIO);

    float bridge_thermistor_1 = read_spi_thermistor(THERM_ADC_CS1);
    float bridge_thermistor_2 = read_spi_thermistor(THERM_ADC_CS2);

    byte aux12_motion = 0xFF & (aux1 | aux2 << 1 | motion_allowed << 2);

    txMsg.Buffer[0] = rx_can_id;
    txMsg.Buffer[1] = REQUEST_SENSORS | 0x80;

    memcpy (txMsg.Buffer+2, &adc1, 2);
    memcpy (txMsg.Buffer+4, &adc2, 2);
    txMsg.Buffer[6] = aux12_motion;
    txMsg.Buffer[7] = byte(bridge_thermistor_1);
    txMsg.Buffer[8] = byte(bridge_thermistor_2);
    txMsg.Buffer[9] = byte(motor1_temp);
    txMsg.Buffer[10] = byte(motor2_temp);
    memcpy (txMsg.Buffer+11, &system_voltage, 4);

    uint32_t crc;
    crc =  crc16(txMsg.Buffer, 16, 0x1021);
    txMsg.Buffer[16] = (uint8_t)(0xFF & crc);
    txMsg.Buffer[17] = (uint8_t)(0xFF & crc>>8);

    print_buffer(txMsg.rx_id, txMsg.Buffer, 18, UART_DEBUG);
    


    txMsg.len = 20;
    txMsg.tx_id = tx_can_id;
    txMsg.rx_id = rx_can_id;
    // print_buffer(txMsg.rx_id, txMsg.Buffer, txMsg.len, UART_DEBUG);
    sendcount += 1;
    // delay(100);
    isotp.send(&txMsg);
}

void loop()
{

  rx_can_id = CAN_BASE_ADDRESS + (digitalRead(CAN_ID_1)<<2 | digitalRead(CAN_ID_2)<<1 | digitalRead(CAN_ID_3));
  rxMsg.rx_id = rx_can_id;
  // CAN0.mcp2515_write_id(0x1, 0,rx_can_id);

  motion_allowed = (millis() - estop_last_millis < 100);
  int estop_val = digitalRead(ESTOP_SQUARE_INPUT);
  if (estop_val != last_estop_val)
  {
    // debug_print("Estop val: ");
    // debug_print(String(estop_val));
    last_estop_val = estop_val;
    estop_last_millis = millis();
  }
  // if (millis() - ts > 20)
  // {
  //   ts = millis();
  // debug_print("loop");
  // }



  // print every 0.3s
  // if (millis() - ts > 50)
  // {
  //   ts = millis();

  //   debug_print("rx_can_id: " + String(rx_can_id));
  //   // bool estop_status = digitalRead(ESTOP_SWITCH_STATUS);
  //   // setRingRed(estop_status);

  //   // debug_print("ESTOP_SWITCH_STATUS: " + String(estop_status));

  //   float thermistor_1 = read_spi_thermistor(THERM_ADC_CS1);
  //   float thermistor_2 = read_spi_thermistor(THERM_ADC_CS2);

  //    debug_print("temp: " + String(thermistor_1) + " | temp2: " + String(thermistor_2));
  //    debug_print("temp: " + String(byte(thermistor_1)) + " | temp2: " + String(byte(thermistor_2)));
  //    debug_print("-------");
  // }

  // // print motion allowed every 0.1s
  // if (millis() - ts > 100)
  // {
  //   ts = millis();
  //   if (motion_allowed)
  //   {
  //     debug_print("Motion allowed.");
  //   }
  //   else
  //   {
  //     debug_print("Motion not allowed.");
  //   }
  // }


  // debug_print("send_sensors");
  // send_sensors_can();
  // delay(2000);

  
  if(CAN0.checkReceive() == CAN_MSGAVAIL)
  {
    debug_print("GOT MESSAGE");

    // setLEDs(strip.Color(255,0,0));
    // uint32_t id;
    // uint8_t len;
    // uint8_t buf[50];
    // CAN0.readMsgBuf(&id, &len, buf);
    // print_buffer(id, buf, len, UART_DEBUG);
    // return;

    if(isotp.receive(&rxMsg)==0)
    {
      setLEDs(strip.Color(0,255,0));
      debug_print("ISOTP_RECEIVE");
      // delay(500);
      // readcount += 1;
      // debug_print("Received buffer of len: ");
      // debug_print(String(rxMsg.len));
      

      if(rxMsg.len<4096)
      {
        if (rxMsg.Buffer[0] == SEND_COMPLETE_SETTINGS)
        {
          // UART_DEBUG.println("GOT MESSAGE");
          // mbed::MbedCRC <POLY_16BIT_CCITT, 16> sd(0, 0, false, false);

          send_buffer_interchip();

          // motor1_setpoint_mode = rxMsg.Buffer[1];
          // memcpy (&motor1_setpoint, rxMsg.Buffer + 2, 4);
          // memcpy (&motor1_max_velocity, rxMsg.Buffer + 6, 4);
          // memcpy (&motor1_max_torque, rxMsg.Buffer + 10, 4);
          // memcpy (&motor1_acceleration, rxMsg.Buffer + 14, 4);
          // motor2_setpoint_mode = rxMsg.Buffer[18];
          // memcpy (&motor2_setpoint, rxMsg.Buffer + 19, 4);
          // memcpy (&motor2_max_velocity, rxMsg.Buffer + 23, 4);
          // memcpy (&motor2_max_torque, rxMsg.Buffer + 27, 4);
          // memcpy (&motor2_acceleration, rxMsg.Buffer + 31, 4);
          // Serial.println(motor2_max_velocity);
        }
        if ((rxMsg.Buffer[0] & 0x7F) == SEND_BASIC_UPDATE)
        {

          send_buffer_interchip();

          if(rxMsg.Buffer[0] & 0x40)
          {
            char rxbuf[4];
            UART_INTERCHIP.readBytes(rxbuf, 4);
            send_sensors_can(reinterpret_cast<float &>( rxbuf));
          }

        }
        if (rxMsg.Buffer[0] == REQUEST_SENSORS)
        {
          send_buffer_interchip();
          char rxbuf[4];
          UART_INTERCHIP.readBytes(rxbuf, 4);
          send_sensors_can(reinterpret_cast<float &>( rxbuf));


        }
        if (rxMsg.Buffer[0] == SIMPLE_PING)
        {
          send_buffer_interchip();
          byte rxbuf[2];
          UART_INTERCHIP.readBytes(rxbuf, 2);
          if(rxbuf[0] == ACK[0] && rxbuf[1] == ACK[1])
          {
            send_ping_reply(true);
          } else
          {
            send_ping_reply(false);
          }
        }
        if (rxMsg.Buffer[4] == SIMPLEFOC_PASS_THROUGH)
        {

          if (UART_INTERCHIP.available() > 0) {
            UART_INTERCHIP.read();
          }
          debug_print(String((char *)rxMsg.Buffer+1));
          //Serial1.println(String((char *)rxMsg.Buffer+1));
          UART_INTERCHIP.write(rxMsg.Buffer+1, rxMsg.len-1);
          byte serialbuffer[50];
          debug_print("Reading Serial");
          delay(10);
          int readlength = UART_INTERCHIP.available();
          UART_INTERCHIP.readBytes(serialbuffer, readlength);
          debug_print("Serial read complete. Got " + String(readlength) + " bytes back.");
          debug_print(String((char *)serialbuffer));
          memcpy (txMsg.Buffer, &serialbuffer, readlength);
          txMsg.len = readlength;
          txMsg.tx_id = tx_can_id;
          txMsg.rx_id = rx_can_id;
          isotp.send(&txMsg);


        }
      }else
      {
     
        if (rxMsg.Buffer[4] == FIRMWARE_UPDATE)
        {

            debug_print("Writing OTA image...");
            LittleFS.begin();
            File f = LittleFS.open("ota.bin", "w");
            if (rxMsg.len != f.write(rxMsg.Buffer+5, rxMsg.len)) {
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
            rp2040.reboot();


        }
     
        if (rxMsg.Buffer[4] == FIRMWARE_UPDATE_CPU2)
        {

          // String debug_val = "val debug: " + String(rxMsg.len) + " " + String((rxMsg.len)&0xFF) + " " + String((rxMsg.len>>16)&0xFF)+ " " + String((rxMsg.len>>24)&0xFF);
          // debug_print(debug_val);
          UART_INTERCHIP.write(0xFF);
          UART_INTERCHIP.write((rxMsg.len)&0xFF);
          UART_INTERCHIP.write((rxMsg.len>>8)&0xFF);
          UART_INTERCHIP.write((rxMsg.len>>16)&0xFF);
          UART_INTERCHIP.write(rxMsg.Buffer+5, rxMsg.len);


        }
      }
    } else
    {

      setLEDs(strip.Color(255,0,255));

    }
  }

} 