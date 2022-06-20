#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include "AX12.h"

AX12::AX12(uart_inst_t *uart, uint8_t id, uint rxpin){
  /*
  uart: UART object the servo is connected to
  id: Hardware id of servo
  rxpin: Pin object controlling the connected tri-state buffer. 0=tx, 1=rx
  */
  device_uart = uart;
  device_id = id;
  device_rxpin = rxpin;
}

int AX12::send_command(uint8_t instruction, uint8_t params[], uint8_t n_params){
  uint8_t length = n_params + 2;  // Dynamixel-defined length field
  uint8_t message_length = n_params + 6; // Number of bytes that will be sent over serial

  uint8_t message[message_length];
  message[0] = MESSAGE_HEADER;
  message[1] = MESSAGE_HEADER;
  message[2] = device_id;
  message[3] = length;
  message[4] = instruction;

  uint8_t param_sum = 0;
  for(int i=0; i<n_params; ++i){
    param_sum += params[i];
    message[i+5] = params[i];
  }
  uint8_t checksum = (~(device_id + length + instruction + param_sum)) & 0xFF;
  message[message_length - 1] = checksum;

  // Set tri-state buffer to transmit
  gpio_put(device_rxpin, MODE_TX);

  // Put message into the UART TX FIFO
  uart_write_blocking(device_uart, message, message_length);

  // Wait until all bytes have finished sending to the AX12
  uart_tx_wait_blocking(device_uart);

  // Set tri-state buffer to receive
  gpio_put(device_rxpin, MODE_RX);

  uint8_t rx_buffer[32]; // Just assume a maximum length, probably this is way too long
  for(int i=0; i<32; ++i){
    rx_buffer[i] = 0xBB;
  }

  // Provide time for AX12 to respond - configured delay is 500us
  if(!uart_is_readable_within_us(device_uart, 500)){ /// TODO: probably need to increase this
    // No response received
    /// TODO: raise error
    return -1;
  } else {
    // Read first 5 bytes of AX12 response
    /// TODO: add timeout
    uart_read_blocking(device_uart, rx_buffer, 5);
  }

  // Response should be as follows
  // 0x FF  FF  03  03   00   [XX] [XX] [...]  C6
  //    HDR HDR ID  LEN  ERR  [P1] [P2] [P...] CHK
  // But we also seem to read a 0x0 at the start always
  if(rx_buffer[1] != MESSAGE_HEADER || rx_buffer[2] != MESSAGE_HEADER){
    /// TODO: Raise error
    return -1;
  }
  uint8_t rx_id = rx_buffer[3];
  uint8_t rx_length = rx_buffer[4];

  // Read the remainder of the message, based on the reported message length
  uart_read_blocking(device_uart, rx_buffer + 5, rx_length);

  uint8_t rx_checksum = rx_buffer[rx_length + 4];

  param_sum = 0;
  for(int i=3; i<rx_length+3; ++i){
    param_sum += rx_buffer[i];
  }
  checksum = (~(param_sum)) & 0xFF;

  if(checksum != rx_checksum){
    /// TODO: Raise error
    return -1;
  }

  return 1;
  /// TODO: move error codes to header file constants
  // Error codes
  // 1000000 Instruction Error   In case of sending an undefined instruction or delivering the action instruction without the Reg Write instruction, it is set as 1
  // 0100000 Overload Error      When the current load cannot be controlled by the set Torque, it is set as 1
  // 0010000 Checksum Error      When the Checksum of the transmitted Instruction Packet is incorrect, it is set as 1
  // 0001000 Range Error         When an instruction is out of the range for use, it is set as 1
  // 0000100 Overheating Error   When internal temperature of DYNAMIXEL is out of the range of operating temperature set in the Control table, it is set as 1
  // 0000010 Angle Limit Error   When Goal Position is written out of the range from CW Angle Limit to CCW Angle Limit , it is set as 1
  // 0000001 Input Voltage Error When the applied voltage is out of the range of operating voltage set in the Control table, it is as 1


}

void AX12::write_position(uint16_t position){
  // position: 0-1023 target position to move to
  uint8_t param_list[] = {MEM_GOAL_POSITION, (uint8_t)(position & 0xFF), (uint8_t)(position >> 8)};
  send_command(CMD_WRITE, param_list, 3);
}
