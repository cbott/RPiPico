// AX12.h
#ifndef AX12_H
#define AX12_H

size_t read_all_available_data(uart_inst_t *uart, uint8_t *dst, size_t max_len);

class AX12 {
private:
  uart_inst_t *device_uart;
  uint8_t device_id;
  uint device_rxpin;
  int send_instruction(uint8_t instruction, uint8_t params[], uint8_t n_params);

public:
  AX12(uart_inst_t *uart, uint8_t id, uint rxpin);
  void write_led_status(bool status);
  void write_position(uint16_t position);
  void write_cw_limit(uint16_t position);
  void write_ccw_limit(uint16_t position);
  void write_speed(uint16_t speed);
  void write_id(uint8_t id);
  void write_torque_enable(bool enable);
  void write_return_delay_time(uint8_t delay_2us);
  int read_temperature();
  int read_position();

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
};

#define MODE_TX (false)
#define MODE_RX (true)

#define MESSAGE_HEADER _u(0xFF)

#endif
