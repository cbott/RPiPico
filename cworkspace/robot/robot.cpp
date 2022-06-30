#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <iostream>

#include "AX12.h"
#include "robotlib.h"

// UART used for servo connection
#define UART_ID uart1
#define UART_TX_PIN 4
#define UART_RX_PIN 5
#define BAUD_RATE 1000000

const uint DIRECTION_PIN = 2;

int main(){
  stdio_init_all();

  uart_init(UART_ID, BAUD_RATE);
  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

  gpio_init(DIRECTION_PIN);
  gpio_set_dir(DIRECTION_PIN, GPIO_OUT);

  std::cout << "Initializing robot" << std::endl;

  // Initialize robot objects
  // Note: servo delay time should be set to 50 (100us)
  AX12 servo1(UART_ID, 0x1, DIRECTION_PIN);
  AX12 servo2(UART_ID, 0x2, DIRECTION_PIN);
  AX12 servo3(UART_ID, 0x3, DIRECTION_PIN);
  AX12 servo4(UART_ID, 0x4, DIRECTION_PIN);

  Robot(&servo1, &servo2, &servo3, &servo4);

  sleep_ms(10);
  // Clear out any junk we have in the UART buffer
  while(uart_is_readable(UART_ID)) uart_getc(UART_ID);

  // Main robot control
  /////////////////////////////////////////////////////////////////////////////
  while(1){

  }
}
