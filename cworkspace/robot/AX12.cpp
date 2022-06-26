#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <iostream>
#include "AX12.h"

size_t read_all_available_data(uart_inst_t *uart, uint8_t *dst, size_t max_len){
  /*
  Read from the specified UART into provided destination until no more bytes
  are available or maximum length is reached.
  Return number of bytes read.
  */
  size_t i = 0;
  for(; i < max_len; ++i){
    if(!uart_is_readable(uart)) break;
    *dst++ = (uint8_t) uart_get_hw(uart)->dr;
  }
  return i;
}

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

int AX12::send_instruction(uint8_t instruction, uint8_t params[], uint8_t n_params){;
  ////////////////////////////// TX ///////////////////////////////////////////
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

  ////////////////////////////// RX ///////////////////////////////////////////
  /// TODO: might make sense to break RX functionality into its own method
  // Set tri-state buffer to receive
  gpio_put(device_rxpin, MODE_RX);

  // DYNAMIXEL response format
  // 0x FF  FF  03  03   00   [XX] [XX] C6
  //    HDR HDR ID  LEN  ERR  [P1] [P2] CHK
  const size_t OFFSET_HEADER1 = 0;
  const size_t OFFSET_HEADER2 = 1;
  const size_t OFFSET_ID = 2;
  const size_t OFFSET_LEN = 3;
  const size_t OFFSET_ERR = 4;
  const size_t OFFSET_PARAMS = 5;

  uint8_t rx_buffer[16]; // TODO: add length limit of 2 to prevent buffer overrun and reduce to 8 bytes (or 9 if leading 0)

  for(int i=0; i<16; ++i){
    rx_buffer[i] = 0xBB;  // TODO: remove, debug only
  }

  // Read AX12 response
  // First provide time for AX12 to respond - maximum delay is 508us
  if(!uart_is_readable_within_us(device_uart, 508)){ /// TODO: adjust based on actual configuration
    // No response received
    return -1; // ERR_NO_RESPONSE
    gpio_put(device_rxpin, MODE_TX);
  }

  // Then wait for all bytes to be received
  // Expect up to 8 bytes, each with 1 start bit, 1 stop bit, no parity bit (80 bits total)
  // 80 bits at 1 Mbaud is 80us, but life isn't that simple so 200us is based on testing
  sleep_us(200);
  size_t bytes_read = read_all_available_data(device_uart, rx_buffer, 16);
  gpio_put(device_rxpin, MODE_TX);
  if(bytes_read < 6){ // MIN_MSG_LEN
    return -2; // ERR_INCOMPLETE_RESPONSE
  }

  if(rx_buffer[OFFSET_HEADER1] != MESSAGE_HEADER || rx_buffer[OFFSET_HEADER2] != MESSAGE_HEADER){
    return -3;  // ERR_INVALID_HEADER
  }
  uint8_t rx_id = rx_buffer[OFFSET_ID];
  uint8_t rx_length = rx_buffer[OFFSET_LEN];

  if(rx_length < 2 || rx_length > 4){
    return -4; // ERR_INVALID_LENGTH
  }

  uint8_t rx_checksum = rx_buffer[OFFSET_LEN + rx_length];

  param_sum = 0;
  for(int i = OFFSET_ID; i <= OFFSET_ID + rx_length; ++i){
    param_sum += rx_buffer[i];
  }
  checksum = (~(param_sum)) & 0xFF;

  if(checksum != rx_checksum){
    return -5;  // ERR_CHECKSUM_MISMATCH
  }

  /// TODO: move error codes to header file constants
  // Error codes (from https://emanual.robotis.com/docs/en/dxl/protocol1/)
  // 1000000 Instruction Error   In case of sending an undefined instruction or delivering the action instruction without the Reg Write instruction, it is set as 1
  // 0100000 Overload Error      When the current load cannot be controlled by the set Torque, it is set as 1
  // 0010000 Checksum Error      When the Checksum of the transmitted Instruction Packet is incorrect, it is set as 1
  // 0001000 Range Error         When an instruction is out of the range for use, it is set as 1
  // 0000100 Overheating Error   When internal temperature of DYNAMIXEL is out of the range of operating temperature set in the Control table, it is set as 1
  // 0000010 Angle Limit Error   When Goal Position is written out of the range from CW Angle Limit to CCW Angle Limit , it is set as 1
  // 0000001 Input Voltage Error When the applied voltage is out of the range of operating voltage set in the Control table, it is as 1

  uint8_t rx_params = rx_length - 2;
  if(rx_params == 1){
    // If only a single parameter was returned, this is our data
    return rx_buffer[OFFSET_PARAMS];
  } else if(rx_params == 2){
    // If two parameters were returned, combine into a 16 bit number
    return rx_buffer[OFFSET_PARAMS] | (rx_buffer[OFFSET_PARAMS + 1] << 8);
  } else {
    // The command returned no data
    return 0;
  }
}

void AX12::write_led_status(bool status){
  /*
  status: desired LED status
  */
  uint8_t param_list[] = {MEM_LED, status};
  send_instruction(CMD_WRITE, param_list, 2);
}

void AX12::write_position(uint16_t position){
  // position: 0-1023 target position to move to
  std::cout << "write_position: " << position << std::endl;
  uint8_t param_list[] = {MEM_GOAL_POSITION, (uint8_t)(position & 0xFF), (uint8_t)(position >> 8)};
  int response = send_instruction(CMD_WRITE, param_list, 3);
  std::cout << "write_position response: " << response << std::endl;
}

// Note: set CW and CCW limit to 0 for "wheel" mode (no rotation limits)
void AX12::write_cw_limit(uint16_t position){
  /* Clockwise angle limit
  position: 0-1023 angle limit
  */
  uint8_t param_list[] = {MEM_CW_ANGLE_LIMIT, position & 0xFF, position >> 8};
  send_instruction(CMD_WRITE, param_list, 3);
}

void AX12::write_ccw_limit(uint16_t position){
  /* Counter-clockwise angle limit
  position: 0-1023 angle limit
  */
  uint8_t param_list[] = {MEM_CCW_ANGLE_LIMIT, position & 0xFF, position >> 8};
  send_instruction(CMD_WRITE, param_list, 3);
}

void AX12::write_speed(uint16_t speed){
  /* Moving Speed
  speed:
      If in Joint mode: 0-1023 corresponding to ~0-114 RPM
      If in Wheel mode: 0-2047 corresponding to 0-100% output power
  */
  uint8_t param_list[] = {MEM_MOVING_SPEED, speed & 0xFF, speed >> 8};
  send_instruction(CMD_WRITE, param_list, 3);
}

void AX12::write_id(uint8_t id){
  /* Change the network identifier of the servo

  id: the desired new ID for the servo [0-253]
  */
  uint8_t param_list[] = {MEM_ID, id};
  send_instruction(CMD_WRITE, param_list, 2);
  device_id = id;
}

void AX12::write_torque_enable(bool enable){
  /* Enable or disable servo torque output */
  std::cout << "write_torque_enable: " << enable << std::endl;
  uint8_t param_list[] = {MEM_TORQUE_ENABLE, enable};
  int response = send_instruction(CMD_WRITE, param_list, 2);
  std::cout << "write_torque_enable response: " << response << std::endl;
}

void AX12::write_return_delay_time(uint8_t delay_2us){
  /* Change the DYNAMIXEL delay between receiving an Instruction packet
     and sending a Status packet response

  delay_2us: Requested delay time in units of 2 microseconds
    valid range 0 (no delay) to 254 (508us delay)
  */
  uint8_t param_list[] = {MEM_RETURN_DELAY_TIME, delay_2us};
  send_instruction(CMD_WRITE, param_list, 2);
}

int AX12::read_temperature(){
  /* Present Tempearatrue

  Return temperature as read from the AX-12
  */
  // Send the read instruction
  uint8_t param_list[] = {MEM_PRESENT_TEMPERATURE, 0x1};
  return send_instruction(CMD_READ, param_list, 2);
}

int AX12::read_position(){
  /* Present Position

  Return AX-12 position in counts (0-1023)
  */
  uint8_t param_list[] = {MEM_PRESENT_POSITION, 0x2};
  return send_instruction(CMD_READ, param_list, 2);
}
