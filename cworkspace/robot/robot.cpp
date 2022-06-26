#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "robot.h"
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <iostream>

#include "AX12.h"

// UART used for servo connection
#define UART_ID uart1
#define UART_TX_PIN 4
#define UART_RX_PIN 5
#define BAUD_RATE 1000000

const uint ID = 0x1;
const uint DIRECTION_PIN = 2;


int main(){
  stdio_init_all();

  uart_init(UART_ID, BAUD_RATE);
  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

  gpio_init(DIRECTION_PIN);
  gpio_set_dir(DIRECTION_PIN, GPIO_OUT);

  // Initialize robot objects
  AX12 servo(UART_ID, ID, DIRECTION_PIN);

  std::cout << "Initializing robot" << std::endl;
  sleep_ms(100);
  // Clear out any junk we have in the UART buffer
  uint8_t buf[32];
  size_t n = read_all_available_data(UART_ID, buf, 32);
  std::cout << "Cleared " << n << std::endl;

  // servo.write_position(100);
  // sleep_ms(1000);
  servo.write_torque_enable(false);
  int j = 0;
  while(1){
    int pos = servo.read_position();
    if(pos < 1){
      std::cout << "Read position " << pos << "  n=" << j << std::endl;
    }
    j++;
    sleep_ms(1);
  }
}
