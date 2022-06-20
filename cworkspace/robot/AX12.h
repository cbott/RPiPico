// AX12.h
#ifndef AX12_H
#define AX12_H

class AX12 {
private:
  uart_inst_t *device_uart;
  uint8_t device_id;
  uint device_rxpin;
  int send_command(uint8_t instruction, uint8_t params[], uint8_t n_params);

public:
  AX12(uart_inst_t *uart, uint8_t id, uint rxpin);
  void write_position(uint16_t position);
};

#define MODE_TX (false)
#define MODE_RX (true)

#define MESSAGE_HEADER _u(0xFF)

// Instructions
#define CMD_PING          _u(0X01)
#define CMD_READ          _u(0X02)
#define CMD_WRITE         _u(0X03)
#define CMD_REG_WRITE     _u(0X04)
#define CMD_ACTION        _u(0X05)
#define CMD_FACTORY_RESET _u(0X06)
#define CMD_REBOOT        _u(0X08)
#define CMD_SYNC_WRITE    _u(0X83)
#define CMD_BULK_READ     _u(0X92)

// Memory map
//                                Address // Size // Access
#define MEM_MODEL_NUMBER          _u(0)   // 2 R
#define MEM_FIRMWARE_VERSION      _u(2)   // 1 R
#define MEM_ID                    _u(3)   // 1 RW
#define MEM_BAUD_RATE             _u(4)   // 1 RW
#define MEM_RETURN_DELAY_TIME     _u(5)   // 1 RW
#define MEM_CW_ANGLE_LIMIT        _u(6)   // 2 RW
#define MEM_CCW_ANGLE_LIMIT       _u(8)   // 2 RW
#define MEM_TEMPERATURE_LIMIT     _u(11)  // 1 RW
#define MEM_MIN_VOLTAGE_LIMIT     _u(12)  // 1 RW
#define MEM_MAX_VOLTAGE_LIMIT     _u(13)  // 1 RW
#define MEM_MAX_TORQUE            _u(14)  // 2 RW
#define MEM_STATUS_RETURN_LEVEL   _u(16)  // 1 RW
#define MEM_ALARM_LED             _u(17)  // 1 RW
#define MEM_SHUTDOWN              _u(18)  // 1 RW
#define MEM_TORQUE_ENABLE         _u(24)  // 1 RW
#define MEM_LED                   _u(25)  // 1 RW
#define MEM_CW_COMPLIANCE_MARGIN  _u(26)  // 1 RW
#define MEM_CCW_COMPLIANCE_MARGIN _u(27)  // 1 RW
#define MEM_CW_COMPLIANCE_SLOPE   _u(28)  // 1 RW
#define MEM_CCW_COMPLIANCE_SLOPE  _u(29)  // 1 RW
#define MEM_GOAL_POSITION         _u(30)  // 2 RW
#define MEM_MOVING_SPEED          _u(32)  // 2 RW
#define MEM_TORQUE_LIMIT          _u(34)  // 2 RW
#define MEM_PRESENT_POSITION      _u(36)  // 2 R
#define MEM_PRESENT_SPEED         _u(38)  // 2 R
#define MEM_PRESENT_LOAD          _u(40)  // 2 R
#define MEM_PRESENT_VOLTAGE       _u(42)  // 1 R
#define MEM_PRESENT_TEMPERATURE   _u(43)  // 1 R
#define MEM_REGISTERED            _u(44)  // 1 R
#define MEM_MOVING                _u(46)  // 1 R
#define MEM_LOCK                  _u(47)  // 1 RW
#define MEM_PUNCH                 _u(48)  // 2 RW

#endif
