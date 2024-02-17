
#include <Arduino.h>

#define UART_DEBUG Serial2

#define LOGGING_DIVIDER '#'
#define LOGGER_LENGTH 2000
#define LOGGER_FULL "LOGGER FULL" + LOGGING_DIVIDER
#define LOGGER_FULL_LEN LOGGER_LENGTH - strlen(LOGGER_FULL)
char logger_string[LOGGER_LENGTH] = "";
uint32_t logger_index = 0;
bool logger_filled = false;




extern bool print_logging;
extern uint32_t serial_speed;

void init_debug(void){
  UART_DEBUG.setRX(25);
  UART_DEBUG.setTX(24);
  UART_DEBUG.setFIFOSize(128);
  UART_DEBUG.begin(serial_speed);
}

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
  if(print_logging)
  {
    if(logger_filled)
    {
      debug_print(LOGGER_FULL);
    }
    debug_print(text);
  }

}
