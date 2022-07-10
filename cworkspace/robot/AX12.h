// AX12.h
#ifndef AX12_H
#define AX12_H

// TODO: this would be better in a utilities file, unless we want to use it before each instruction
size_t read_all_available_data(uart_inst_t *uart, uint8_t *dst, size_t max_len);

// Error codes (from https://emanual.robotis.com/docs/en/dxl/protocol1/)
union AX12StatusError {
  uint8_t error_byte;
  struct {
    uint8_t               : 1; // Unused
    uint8_t instruction   : 1; // In case of sending an undefined instruction or delivering the action instruction without the Reg Write instruction, it is set as 1
    uint8_t overload      : 1; // When the current load cannot be controlled by the set Torque, it is set as 1
    uint8_t checksum      : 1; // When the Checksum of the transmitted Instruction Packet is incorrect, it is set as 1
    uint8_t range         : 1; // When an instruction is out of the range for use, it is set as 1
    uint8_t overheating   : 1; // When internal temperature of DYNAMIXEL is out of the range of operating temperature set in the Control table, it is set as 1
    uint8_t angle_limit   : 1; // When Goal Position is written out of the range from CW Angle Limit to CCW Angle Limit , it is set as 1
    uint8_t input_voltage : 1; // When the applied voltage is out of the range of operating voltage set in the Control table, it is as 1
  } error_components;
};

/**
 * Interface class for DYNAMIXEL AX-12 servo line
 *
 * See https://emanual.robotis.com/docs/en/dxl/ax/ax-12a
 */
class AX12 {
private:
  uart_inst_t *device_uart;
  uint8_t device_id;
  uint device_rxpin;

  int send_instruction_core(uint8_t instruction, uint8_t params[], uint8_t n_params);
  int send_instruction(uint8_t instruction, uint8_t params[], uint8_t n_params);

public:
  AX12(uart_inst_t *uart, uint8_t id, uint rxpin);
  AX12StatusError device_status_error;
  uint32_t error_counter;

  // Control Methods (communicates to Dynamixel)
  int write_led_status(bool status);
  int write_position(uint16_t position);
  int write_cw_limit(uint16_t position);
  int write_ccw_limit(uint16_t position);
  int write_speed(uint16_t speed);
  int write_id(uint8_t id);
  int write_torque_enable(bool enable);
  int write_return_delay_time(uint8_t delay_2us);
  int read_temperature();
  int read_position();

  // Software Methods (no communication to Dynamixel)
  uint8_t get_id();

  // Instructions
  static const uint8_t CMD_PING          = 0X01;
  static const uint8_t CMD_READ          = 0X02;
  static const uint8_t CMD_WRITE         = 0X03;
  static const uint8_t CMD_REG_WRITE     = 0X04;
  static const uint8_t CMD_ACTION        = 0X05;
  static const uint8_t CMD_FACTORY_RESET = 0X06;
  static const uint8_t CMD_REBOOT        = 0X08;
  static const uint8_t CMD_SYNC_WRITE    = 0X83;
  static const uint8_t CMD_BULK_READ     = 0X92;

  // Memory map
  //                                               Address // Size    Access
  static const uint8_t MEM_MODEL_NUMBER          = 0;      // 2       R
  static const uint8_t MEM_FIRMWARE_VERSION      = 2;      // 1       R
  static const uint8_t MEM_ID                    = 3;      // 1       RW
  static const uint8_t MEM_BAUD_RATE             = 4;      // 1       RW
  static const uint8_t MEM_RETURN_DELAY_TIME     = 5;      // 1       RW
  static const uint8_t MEM_CW_ANGLE_LIMIT        = 6;      // 2       RW
  static const uint8_t MEM_CCW_ANGLE_LIMIT       = 8;      // 2       RW
  static const uint8_t MEM_TEMPERATURE_LIMIT     = 11;     // 1       RW
  static const uint8_t MEM_MIN_VOLTAGE_LIMIT     = 12;     // 1       RW
  static const uint8_t MEM_MAX_VOLTAGE_LIMIT     = 13;     // 1       RW
  static const uint8_t MEM_MAX_TORQUE            = 14;     // 2       RW
  static const uint8_t MEM_STATUS_RETURN_LEVEL   = 16;     // 1       RW
  static const uint8_t MEM_ALARM_LED             = 17;     // 1       RW
  static const uint8_t MEM_SHUTDOWN              = 18;     // 1       RW
  static const uint8_t MEM_TORQUE_ENABLE         = 24;     // 1       RW
  static const uint8_t MEM_LED                   = 25;     // 1       RW
  static const uint8_t MEM_CW_COMPLIANCE_MARGIN  = 26;     // 1       RW
  static const uint8_t MEM_CCW_COMPLIANCE_MARGIN = 27;     // 1       RW
  static const uint8_t MEM_CW_COMPLIANCE_SLOPE   = 28;     // 1       RW
  static const uint8_t MEM_CCW_COMPLIANCE_SLOPE  = 29;     // 1       RW
  static const uint8_t MEM_GOAL_POSITION         = 30;     // 2       RW
  static const uint8_t MEM_MOVING_SPEED          = 32;     // 2       RW
  static const uint8_t MEM_TORQUE_LIMIT          = 34;     // 2       RW
  static const uint8_t MEM_PRESENT_POSITION      = 36;     // 2       R
  static const uint8_t MEM_PRESENT_SPEED         = 38;     // 2       R
  static const uint8_t MEM_PRESENT_LOAD          = 40;     // 2       R
  static const uint8_t MEM_PRESENT_VOLTAGE       = 42;     // 1       R
  static const uint8_t MEM_PRESENT_TEMPERATURE   = 43;     // 1       R
  static const uint8_t MEM_REGISTERED            = 44;     // 1       R
  static const uint8_t MEM_MOVING                = 46;     // 1       R
  static const uint8_t MEM_LOCK                  = 47;     // 1       RW
  static const uint8_t MEM_PUNCH                 = 48;     // 2       RW

  // Communication Errors
  static const int ERR_NO_RESPONSE         = -1;
  static const int ERR_INCOMPLETE_RESPONSE = -2;
  static const int ERR_INVALID_HEADER      = -3;
  static const int ERR_INVALID_LENGTH      = -4;
  static const int ERR_CHECKSUM_MISMATCH   = -5;
  static const int ERR_ID_MISMATCH         = -6;
};

#define MODE_TX (false)
#define MODE_RX (true)

#define MESSAGE_HEADER _u(0xFF)

#define RX_BUFFER_SIZE 16

#endif
