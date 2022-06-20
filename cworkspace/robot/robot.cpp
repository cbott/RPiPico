#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "robot.h"
#include <cstdlib>
#include <cstring>
#include <cstdint>

#include "AX12.h"

#define UART_TX_PIN 0
#define UART_RX_PIN 1

const uint ID = 0x1;
const uint DIRECTION_PIN = 2;

// void send_command1(){
//   uint8_t instruction = CMD_WRITE;
//   uint8_t param1 = MEM_GOAL_POSITION;
//   uint16_t position = 100;
//   uint8_t param2 = position & 0xFF;
//   uint8_t param3 = position >> 8;

//   uint8_t length = 5; // len(params) + 2

//   uint8_t checksum = (~(ID + length + instruction + param1 + param2 + param3)) & 0xFF;

//   uint8_t message[] = {MESSAGE_HEADER, MESSAGE_HEADER, ID, length, instruction, param1, param2, param3, checksum};

//   uart_write_blocking(uart0, message, 9);
// }

// void send_command2(){
//   uint8_t instruction = CMD_WRITE;
//   uint8_t param1 = MEM_GOAL_POSITION;
//   uint16_t position = 500;
//   uint8_t param2 = position & 0xFF;
//   uint8_t param3 = position >> 8;

//   uint8_t length = 5; // len(params) + 2

//   uint8_t checksum = (~(ID + length + instruction + param1 + param2 + param3)) & 0xFF;

//   uint8_t message[] = {MESSAGE_HEADER, MESSAGE_HEADER, ID, length, instruction, param1, param2, param3, checksum};

//   uart_write_blocking(uart0, message, 9);
// }

int main(){
  uart_init(uart0, 1000000);

  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

  gpio_init(DIRECTION_PIN);
  gpio_set_dir(DIRECTION_PIN, GPIO_OUT);

  // gpio_put(DIRECTION_PIN, 0);

  AX12 servo(uart0, ID, DIRECTION_PIN);

  sleep_ms(100);

  while(1){
    servo.write_position(100);
    sleep_ms(3000);
    servo.write_position(400);
    sleep_ms(3000);
  }
}
