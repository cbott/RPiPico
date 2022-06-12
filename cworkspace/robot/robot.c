#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "robot.h"

#define UART_TX_PIN 0
#define UART_RX_PIN 1

const uint LED_PIN = 25;
const uint ID = 0x1;


void send_command(){
  // TODO: let's get the right sizes for these int16/32...
  uint instruction = CMD_WRITE;
  uint8_t param1 = MEM_GOAL_POSITION;
  uint16_t position = 100;
  uint8_t param2 = position & 0xFF;
  uint8_t param3 = position >> 8;

  uint8_t length = 5; // len(params) + 2

  uint8_t checksum = (~(ID + length + instruction + param1 + param2 + param3)) & 0xFF;

  uint8_t message[] = {MESSAGE_HEADER, MESSAGE_HEADER, ID, length, instruction, param1, param2, param3, checksum};

  uart_write_blocking(uart0, message, 9);
}

int main(){
  uart_init(uart0, 115200);

  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

  send_command();
}
