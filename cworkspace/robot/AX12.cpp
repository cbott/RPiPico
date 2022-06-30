#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <iostream>
#include "AX12.h"

/**
 * Read from the specified UART into provided destination until no more bytes
 * are available or maximum length is reached.
 *
 * @return number of bytes read
 */
size_t read_all_available_data(uart_inst_t *uart, uint8_t *dst, size_t max_len){
  size_t i = 0;
  for(; i < max_len; ++i){
    if(!uart_is_readable(uart)) break;
    *dst++ = (uint8_t) uart_get_hw(uart)->dr;
  }
  return i;
}

/**
 * @param uart UART object the servo is connected to
 * @param id Hardware id of servo
 * @param rxpin Pin object controlling the connected tri-state buffer. 0=tx, 1=rx
*/
AX12::AX12(uart_inst_t *uart, uint8_t id, uint rxpin){
  device_uart = uart;
  device_id = id;
  device_rxpin = rxpin;
}

int AX12::send_instruction_core(uint8_t instruction, uint8_t params[], uint8_t n_params){
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

  uint8_t rx_buffer[RX_BUFFER_SIZE];

  for(int i=0; i<RX_BUFFER_SIZE; ++i){
    rx_buffer[i] = 0xBB;  // TODO: remove, debug only
  }

  // Read AX12 response
  // First provide time for AX12 to respond - maximum delay is 508us, plus some margin
  if(!uart_is_readable_within_us(device_uart, 1000)){
    // No response received
    gpio_put(device_rxpin, MODE_TX);
    return ERR_NO_RESPONSE;
  }

  // Then wait for all bytes to be received
  // Expect up to 8 bytes, each with 1 start bit, 1 stop bit, no parity bit (80 bits total)
  // 80 bits at 1 Mbaud is 80us, but life isn't that simple so 250us is based on testing
  sleep_us(250);
  size_t bytes_read = read_all_available_data(device_uart, rx_buffer, RX_BUFFER_SIZE);
  gpio_put(device_rxpin, MODE_TX);
  if(bytes_read < 6){ // MIN_MSG_LEN
    return ERR_INCOMPLETE_RESPONSE;
  }

  if(rx_buffer[OFFSET_HEADER1] != MESSAGE_HEADER || rx_buffer[OFFSET_HEADER2] != MESSAGE_HEADER){
    return ERR_INVALID_HEADER;
  }
  uint8_t rx_id = rx_buffer[OFFSET_ID];
  if(rx_id != device_id){
    // We have somehow recieved a response from a different AX12
    return ERR_ID_MISMATCH;
  }

  uint8_t rx_length = rx_buffer[OFFSET_LEN];
  if(rx_length < 2 || rx_length > 4){
    return ERR_INVALID_LENGTH;
  }

  uint8_t rx_err = rx_buffer[OFFSET_ERR];
  uint8_t rx_checksum = rx_buffer[OFFSET_LEN + rx_length];

  // Calculate checksum
  param_sum = 0;
  for(int i = OFFSET_ID; i <= OFFSET_ID + rx_length; ++i){
    param_sum += rx_buffer[i];
  }
  checksum = (~(param_sum)) & 0xFF;

  if(checksum != rx_checksum){
    return ERR_CHECKSUM_MISMATCH;
  }

  // Update our error status with the latest status response
  device_status_error.error_byte = rx_err;

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

int AX12::send_instruction(uint8_t instruction, uint8_t params[], uint8_t n_params){;
  /// TODO: figure out a better way to pass along or track errors
  int result = send_instruction_core(instruction, params, n_params);

  if(result < 0){
    std::cout << "RX error " << result << " ID: " << device_id << " error count: " <<  ++error_counter << std::endl;
  }

  return result;
}

/**
 * Turn AX-12 LED on or off
 *
 * @param status desired LED state
 */
int AX12::write_led_status(bool status){
  uint8_t param_list[] = {MEM_LED, status};
  return send_instruction(CMD_WRITE, param_list, 2);
}

/**
 * Set servo position
 *
 * @param position target position (0-1023)
 */
int AX12::write_position(uint16_t position){
  uint8_t param_list[] = {MEM_GOAL_POSITION, (uint8_t)(position & 0xFF), (uint8_t)(position >> 8)};
  return send_instruction(CMD_WRITE, param_list, 3);
}

// Note: set CW and CCW limit to 0 for "wheel" mode (no rotation limits)
/**
 * Set clockwise angle limit
 * @param position angle limit (0-1023)
 */
int AX12::write_cw_limit(uint16_t position){
  uint8_t param_list[] = {MEM_CW_ANGLE_LIMIT, (uint8_t)(position & 0xFF), (uint8_t)(position >> 8)};
  return send_instruction(CMD_WRITE, param_list, 3);
}

/**
 * Set counter-clockwise angle limit
 *
 * @param position angle limit (0-1023)
 */
int AX12::write_ccw_limit(uint16_t position){
  uint8_t param_list[] = {MEM_CCW_ANGLE_LIMIT, (uint8_t)(position & 0xFF), (uint8_t)(position >> 8)};
  return send_instruction(CMD_WRITE, param_list, 3);
}


/**
 * Set AX-12 Movement Speed
 *
 * @param speed
 *  #If in Joint mode: 0-1023 corresponding to ~0-114 RPM
 *  #If in Wheel mode: 0-2047 corresponding to 0-100% output power
 */
int AX12::write_speed(uint16_t speed){
  uint8_t param_list[] = {MEM_MOVING_SPEED, (uint8_t)(speed & 0xFF), (uint8_t)(speed >> 8)};
  return send_instruction(CMD_WRITE, param_list, 3);
}

/**
 * Set AX-12 network identifier
 *
 * @param id the desired new ID for the servo [0-253]
 */
int AX12::write_id(uint8_t id){
  uint8_t param_list[] = {MEM_ID, id};
  return send_instruction(CMD_WRITE, param_list, 2);
  device_id = id;
}

/**
 * Enable or disable AX-12 torque output
 *
 * @param enable desired torque enable status
 */
int AX12::write_torque_enable(bool enable){
  uint8_t param_list[] = {MEM_TORQUE_ENABLE, enable};
  return send_instruction(CMD_WRITE, param_list, 2);
}

/**
 * Change the AX-12 delay between receiving an Instruction packet and sending a Status packet response
 *
 * @param delay_2us Requested delay time in units of 2 microseconds, valid range 0 (no delay) to 254 (508us delay)
 */
int AX12::write_return_delay_time(uint8_t delay_2us){
  uint8_t param_list[] = {MEM_RETURN_DELAY_TIME, delay_2us};
  return send_instruction(CMD_WRITE, param_list, 2);
}

/**
 * Present Tempearatrue
 *
 * @return AX-12 temperature in degrees celcius
 */
int AX12::read_temperature(){
  // Send the read instruction
  uint8_t param_list[] = {MEM_PRESENT_TEMPERATURE, 0x1};
  return send_instruction(CMD_READ, param_list, 2);
}

/**
 * Present Position
 *
 * @return AX-12 position in counts (0-1023)
 */
int AX12::read_position(){
  uint8_t param_list[] = {MEM_PRESENT_POSITION, 0x2};
  return send_instruction(CMD_READ, param_list, 2);
}
