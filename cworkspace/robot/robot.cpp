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

// TODO: should probably be consistent with defines vs consts
const uint DIRECTION_PIN = 2;
const uint LED_PIN = 25;

// Fault state for unrecoverable software errors
void fault(){
  std::cout << "ROBOT FAULT" << std::endl;
  while(1){
    gpio_put(LED_PIN, 0);
    sleep_ms(200);
    gpio_put(LED_PIN, 1);
    sleep_ms(200);
  }
}

void print_status(AX12 servo){
  std::cout << "Errors for Servo ID " << unsigned(servo.get_id()) << ": " << std::endl;
  if(servo.device_status_error.error_components.angle_limit){
    std::cout << "Angle Limit ";
  }
  if(servo.device_status_error.error_components.checksum){
    std::cout << "Checksum ";
  }
  if(servo.device_status_error.error_components.input_voltage){
    std::cout << "Input Voltage ";
  }
  if(servo.device_status_error.error_components.instruction){
    std::cout << "Instruction ";
  }
  if(servo.device_status_error.error_components.overheating){
    std::cout << "Overheating ";
  }
  if(servo.device_status_error.error_components.overload){
    std::cout << "Overload ";
  }
  if(servo.device_status_error.error_components.range){
    std::cout << "Range ";
  }
  std::cout << std::endl;
}

int main(){
  stdio_init_all();

  uart_init(UART_ID, BAUD_RATE);
  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

  gpio_init(DIRECTION_PIN);
  gpio_set_dir(DIRECTION_PIN, GPIO_OUT);

  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  std::cout << "Initializing robot" << std::endl;

  sleep_ms(10);
  // Clear out any junk we have in the UART buffer
  while(uart_is_readable(UART_ID)) uart_getc(UART_ID);

  // Initialize robot objects
  // Note: servo delay time should be set to 50 (100us)
  AX12 servo1(UART_ID, 0x1, DIRECTION_PIN);
  AX12 servo2(UART_ID, 0x2, DIRECTION_PIN);
  AX12 servo3(UART_ID, 0x3, DIRECTION_PIN);
  AX12 servo4(UART_ID, 0x4, DIRECTION_PIN);
  Robot robert(&servo1, &servo2, &servo3, &servo4);

  // Main robot control
  /////////////////////////////////////////////////////////////////////////////
  while(1){
    if(!robert.set_position(50, 100, 100, -1.57, false, false, 150)){
      std::cout << "Failed to set (50, 100, 100)" << std::endl;
    }
    sleep_ms(3000);
    if(!robert.set_position(-50, 100, 100, 1.57, false, false, 150)){
      std::cout << "Failed to set (-50, 100, 100)" << std::endl;
    }
    sleep_ms(3000);
  }
  fault();
}
