/*
  ACREL_DTSD1352.cpp - Library implementation
  Supports both ArduinoModbus and ModbusMaster
  Version 2.0 - Added batch reading support
*/

#include "ACREL_DTSD1352.h"

#ifdef USE_MODBUS_MASTER
ACREL_DTSD1352 *ACREL_DTSD1352::_instance = nullptr;
#endif

ACREL_DTSD1352::ACREL_DTSD1352(uint8_t slaveId) {
  _slaveId = slaveId;
  _ptRatio = 1;
  _ctRatio = 1;
  _lastError = 0;
#ifdef USE_MODBUS_MASTER
  _serial = nullptr;
  _dePin = -1;
  _instance = this;
#endif
}

#ifdef USE_MODBUS_MASTER
// ESP32/ESP8266 Implementation using ModbusMaster

bool ACREL_DTSD1352::begin(HardwareSerial *serial, long baudrate) {
  _serial = serial;
  _serial->begin(baudrate);
  _modbus.begin(_slaveId, *_serial);
  delay(100);
  return true;
}

bool ACREL_DTSD1352::begin(HardwareSerial *serial, int8_t dePin, long baudrate) {
  _serial = serial;
  _dePin = dePin;
  
  pinMode(_dePin, OUTPUT);
  digitalWrite(_dePin, LOW);
  
  _serial->begin(baudrate);
  _modbus.begin(_slaveId, *_serial);
  
  // Set callbacks for DE/RE control
  _modbus.preTransmission([]() { _instance->preTransmission(); });
  _modbus.postTransmission([]() { _instance->postTransmission(); });
  
  delay(100);
  return true;
}

void ACREL_DTSD1352::preTransmission() {
  if (_dePin >= 0) {
    digitalWrite(_dePin, HIGH);
  }
}

void ACREL_DTSD1352::postTransmission() {
  if (_dePin >= 0) {
    digitalWrite(_dePin, LOW);
  }
}

#else
// Arduino Implementation using ArduinoModbus

bool ACREL_DTSD1352::begin(long baudrate, uint16_t config) {
  if (!ModbusRTUClient.begin(baudrate, config)) {
    return false;
  }
  delay(100);
  return true;
}

bool ACREL_DTSD1352::begin(HardwareSerial &serial, long baudrate, uint16_t config) {
  serial.begin(baudrate, config);
  if (!ModbusRTUClient.begin(baudrate, config)) {
    return false;
  }
  delay(100);
  return true;
}

bool ACREL_DTSD1352::begin(int8_t txPin, int8_t rxPin, int8_t dePin, long baudrate, uint16_t config) {
  // Khởi tạo RS485 với các pin tùy chỉnh
  RS485.setSerial(&Serial);
  RS485.begin(baudrate, config);
  RS485.setPins(txPin, dePin, rxPin, dePin);
  
  if (!ModbusRTUClient.begin(RS485, baudrate)) {
    return false;
  }
  delay(100);
  return true;
}
#endif

void ACREL_DTSD1352::setPTRatio(uint16_t ratio) {
  _ptRatio = ratio;
}

void ACREL_DTSD1352::setCTRatio(uint16_t ratio) {
  _ctRatio = ratio;
}

float ACREL_DTSD1352::applyPTRatio(uint16_t value, float multiplier) {
  return value * _ptRatio * multiplier;
}

float ACREL_DTSD1352::applyCTRatio(uint16_t value, float multiplier) {
  return value * _ctRatio * multiplier;
}

float ACREL_DTSD1352::applyPTCTRatio(uint32_t value, float multiplier) {
  return value * _ptRatio * _ctRatio * multiplier;
}

uint32_t ACREL_DTSD1352::combine32(uint16_t high, uint16_t low) {
  return ((uint32_t)high << 16) | low;
}

int32_t ACREL_DTSD1352::combine32Signed(uint16_t high, uint16_t low) {
  return (int32_t)combine32(high, low);
}

#ifdef USE_MODBUS_MASTER
// ModbusMaster read functions

uint16_t ACREL_DTSD1352::readHoldingRegister(uint16_t address) {
  _lastError = 0;
  
  // Retry up to 3 times
  for (int retry = 0; retry < 3; retry++) {
    uint8_t result = _modbus.readHoldingRegisters(address, 1);
    
    if (result == _modbus.ku8MBSuccess) {
      return _modbus.getResponseBuffer(0);
    }
    
    _lastError = result;
    
    // Wait before retry
    if (retry < 2) {
      delay(100);
    }
  }
  
  return 0;
}

uint32_t ACREL_DTSD1352::readHoldingRegister32(uint16_t address) {
  _lastError = 0;
  uint8_t result = _modbus.readHoldingRegisters(address, 2);
  
  if (result == _modbus.ku8MBSuccess) {
    uint32_t high = _modbus.getResponseBuffer(0);
    uint32_t low = _modbus.getResponseBuffer(1);
    return (high << 16) | low;
  }
  
  _lastError = result;
  return 0;
}

int16_t ACREL_DTSD1352::readHoldingRegisterSigned(uint16_t address) {
  return (int16_t)readHoldingRegister(address);
}

int32_t ACREL_DTSD1352::readHoldingRegister32Signed(uint16_t address) {
  return (int32_t)readHoldingRegister32(address);
}

// BATCH READ IMPLEMENTATION for ModbusMaster
bool ACREL_DTSD1352::readHoldingRegistersBatch(uint16_t startAddress, uint16_t quantity, uint16_t* buffer) {
  _lastError = 0;
  
  // ModbusMaster giới hạn 125 registers mỗi lần đọc
  if (quantity > 125) {
    _lastError = 0xFF; // Error: too many registers
    return false;
  }
  
  // Retry up to 3 times
  for (int retry = 0; retry < 3; retry++) {
    uint8_t result = _modbus.readHoldingRegisters(startAddress, quantity);
    
    if (result == _modbus.ku8MBSuccess) {
      for (uint16_t i = 0; i < quantity; i++) {
        buffer[i] = _modbus.getResponseBuffer(i);
      }
      return true;
    }
    
    _lastError = result;
    
    // Wait before retry
    if (retry < 2) {
      delay(100);
    }
  }
  
  return false;
}

#else
// ArduinoModbus read functions

uint16_t ACREL_DTSD1352::readHoldingRegister(uint16_t address) {
  _lastError = 0;
  if (!ModbusRTUClient.requestFrom(_slaveId, HOLDING_REGISTERS, address, 1)) {
    _lastError = ModbusRTUClient.lastError();
    return 0;
  }
  return ModbusRTUClient.read();
}

uint32_t ACREL_DTSD1352::readHoldingRegister32(uint16_t address) {
  _lastError = 0;
  if (!ModbusRTUClient.requestFrom(_slaveId, HOLDING_REGISTERS, address, 2)) {
    _lastError = ModbusRTUClient.lastError();
    return 0;
  }
  uint32_t high = ModbusRTUClient.read();
  uint32_t low = ModbusRTUClient.read();
  return (high << 16) | low;
}

int16_t ACREL_DTSD1352::readHoldingRegisterSigned(uint16_t address) {
  return (int16_t)readHoldingRegister(address);
}

int32_t ACREL_DTSD1352::readHoldingRegister32Signed(uint16_t address) {
  return (int32_t)readHoldingRegister32(address);
}

// BATCH READ IMPLEMENTATION for ArduinoModbus
bool ACREL_DTSD1352::readHoldingRegistersBatch(uint16_t startAddress, uint16_t quantity, uint16_t* buffer) {
  _lastError = 0;
  
  if (!ModbusRTUClient.requestFrom(_slaveId, HOLDING_REGISTERS, startAddress, quantity)) {
    _lastError = ModbusRTUClient.lastError();
    return false;
  }
  
  for (uint16_t i = 0; i < quantity; i++) {
    buffer[i] = ModbusRTUClient.read();
  }
  
  return true;
}
#endif

int ACREL_DTSD1352::getLastError() {
  return _lastError;
}

// Energy readings
float ACREL_DTSD1352::readTotalActiveEnergy() {
  uint32_t value = readHoldingRegister32(REG_TOTAL_ACTIVE_ENERGY);
  return applyPTCTRatio(value, 0.01);
}

float ACREL_DTSD1352::readForwardActiveEnergy() {
  uint32_t value = readHoldingRegister32(REG_FORWARD_ACTIVE_ENERGY);
  return applyPTCTRatio(value, 0.01);
}

float ACREL_DTSD1352::readReverseActiveEnergy() {
  uint32_t value = readHoldingRegister32(REG_REVERSE_ACTIVE_ENERGY);
  return applyPTCTRatio(value, 0.01);
}

float ACREL_DTSD1352::readTotalReactiveEnergy() {
  uint32_t value = readHoldingRegister32(REG_TOTAL_REACTIVE_ENERGY);
  return applyPTCTRatio(value, 0.01);
}

float ACREL_DTSD1352::readForwardReactiveEnergy() {
  uint32_t value = readHoldingRegister32(REG_FORWARD_REACTIVE_ENERGY);
  return applyPTCTRatio(value, 0.01);
}

float ACREL_DTSD1352::readReverseReactiveEnergy() {
  uint32_t value = readHoldingRegister32(REG_REVERSE_REACTIVE_ENERGY);
  return applyPTCTRatio(value, 0.01);
}

// Voltage readings
float ACREL_DTSD1352::readVoltageA() {
  uint16_t value = readHoldingRegister(REG_VOLTAGE_A);
  return applyPTRatio(value, 0.1);
}

float ACREL_DTSD1352::readVoltageB() {
  uint16_t value = readHoldingRegister(REG_VOLTAGE_B);
  return applyPTRatio(value, 0.1);
}

float ACREL_DTSD1352::readVoltageC() {
  uint16_t value = readHoldingRegister(REG_VOLTAGE_C);
  return applyPTRatio(value, 0.1);
}

// Current readings
float ACREL_DTSD1352::readCurrentA() {
  uint16_t value = readHoldingRegister(REG_CURRENT_A);
  return applyCTRatio(value, 0.01);
}

float ACREL_DTSD1352::readCurrentB() {
  uint16_t value = readHoldingRegister(REG_CURRENT_B);
  return applyCTRatio(value, 0.01);
}

float ACREL_DTSD1352::readCurrentC() {
  uint16_t value = readHoldingRegister(REG_CURRENT_C);
  return applyCTRatio(value, 0.01);
}

// Frequency
float ACREL_DTSD1352::readFrequency() {
  uint16_t value = readHoldingRegister(REG_FREQUENCY);
  return value * 0.01;
}

// Active Power
float ACREL_DTSD1352::readActivePowerA() {
  int32_t value = readHoldingRegister32Signed(REG_ACTIVE_POWER_A);
  return value * _ptRatio * _ctRatio * 0.001;
}

float ACREL_DTSD1352::readActivePowerB() {
  int32_t value = readHoldingRegister32Signed(REG_ACTIVE_POWER_B);
  return value * _ptRatio * _ctRatio * 0.001;
}

float ACREL_DTSD1352::readActivePowerC() {
  int32_t value = readHoldingRegister32Signed(REG_ACTIVE_POWER_C);
  return value * _ptRatio * _ctRatio * 0.001;
}

float ACREL_DTSD1352::readTotalActivePower() {
  int32_t value = readHoldingRegister32Signed(REG_TOTAL_ACTIVE_POWER);
  return value * _ptRatio * _ctRatio * 0.001;
}

// Reactive Power
float ACREL_DTSD1352::readReactivePowerA() {
  int32_t value = readHoldingRegister32Signed(REG_REACTIVE_POWER_A);
  return value * _ptRatio * _ctRatio * 0.001;
}

float ACREL_DTSD1352::readReactivePowerB() {
  int32_t value = readHoldingRegister32Signed(REG_REACTIVE_POWER_B);
  return value * _ptRatio * _ctRatio * 0.001;
}

float ACREL_DTSD1352::readReactivePowerC() {
  int32_t value = readHoldingRegister32Signed(REG_REACTIVE_POWER_C);
  return value * _ptRatio * _ctRatio * 0.001;
}

float ACREL_DTSD1352::readTotalReactivePower() {
  int32_t value = readHoldingRegister32Signed(REG_TOTAL_REACTIVE_POWER);
  return value * _ptRatio * _ctRatio * 0.001;
}

// Apparent Power
float ACREL_DTSD1352::readApparentPowerA() {
  uint32_t value = readHoldingRegister32(REG_APPARENT_POWER_A);
  return value * _ptRatio * _ctRatio * 0.001;
}

float ACREL_DTSD1352::readApparentPowerB() {
  uint32_t value = readHoldingRegister32(REG_APPARENT_POWER_B);
  return value * _ptRatio * _ctRatio * 0.001;
}

float ACREL_DTSD1352::readApparentPowerC() {
  uint32_t value = readHoldingRegister32(REG_APPARENT_POWER_C);
  return value * _ptRatio * _ctRatio * 0.001;
}

float ACREL_DTSD1352::readTotalApparentPower() {
  uint32_t value = readHoldingRegister32(REG_TOTAL_APPARENT_POWER);
  return value * _ptRatio * _ctRatio * 0.001;
}

// Power Factor
float ACREL_DTSD1352::readPowerFactorA() {
  int16_t value = readHoldingRegisterSigned(REG_POWER_FACTOR_A);
  return value * 0.001;
}

float ACREL_DTSD1352::readPowerFactorB() {
  int16_t value = readHoldingRegisterSigned(REG_POWER_FACTOR_B);
  return value * 0.001;
}

float ACREL_DTSD1352::readPowerFactorC() {
  int16_t value = readHoldingRegisterSigned(REG_POWER_FACTOR_C);
  return value * 0.001;
}

float ACREL_DTSD1352::readTotalPowerFactor() {
  int16_t value = readHoldingRegisterSigned(REG_TOTAL_POWER_FACTOR);
  return value * 0.001;
}

// Demand
float ACREL_DTSD1352::readForwardActiveDemand() {
  uint16_t value = readHoldingRegister(REG_FORWARD_ACTIVE_DEMAND);
  return value * 0.001;
}

float ACREL_DTSD1352::readReverseActiveDemand() {
  uint16_t value = readHoldingRegister(REG_REVERSE_ACTIVE_DEMAND);
  return value * 0.001;
}

// Temperature
float ACREL_DTSD1352::readTemperatureT1() {
  uint16_t value = readHoldingRegister(REG_TEMP_T1);
  return value * 0.1;
}

float ACREL_DTSD1352::readTemperatureT2() {
  uint16_t value = readHoldingRegister(REG_TEMP_T2);
  return value * 0.1;
}

float ACREL_DTSD1352::readTemperatureT3() {
  uint16_t value = readHoldingRegister(REG_TEMP_T3);
  return value * 0.1;
}

// THD readings
float ACREL_DTSD1352::readTHDVoltageA() {
  uint16_t value = readHoldingRegister(REG_THD_VOLTAGE_A);
  return value * 0.1;
}

float ACREL_DTSD1352::readTHDVoltageB() {
  uint16_t value = readHoldingRegister(REG_THD_VOLTAGE_B);
  return value * 0.1;
}

float ACREL_DTSD1352::readTHDVoltageC() {
  uint16_t value = readHoldingRegister(REG_THD_VOLTAGE_C);
  return value * 0.1;
}

float ACREL_DTSD1352::readTHDCurrentA() {
  uint16_t value = readHoldingRegister(REG_THD_CURRENT_A);
  return value * 0.1;
}

float ACREL_DTSD1352::readTHDCurrentB() {
  uint16_t value = readHoldingRegister(REG_THD_CURRENT_B);
  return value * 0.1;
}

float ACREL_DTSD1352::readTHDCurrentC() {
  uint16_t value = readHoldingRegister(REG_THD_CURRENT_C);
  return value * 0.1;
}

// Read all basic data at once (original method)
ACREL_DTSD1352::BasicData ACREL_DTSD1352::readBasicData() {
  BasicData data;
  data.valid = false;
  
  data.voltageA = readVoltageA();
  if (_lastError != 0) return data;
  
  data.voltageB = readVoltageB();
  if (_lastError != 0) return data;
  
  data.voltageC = readVoltageC();
  if (_lastError != 0) return data;
  
  data.currentA = readCurrentA();
  if (_lastError != 0) return data;
  
  data.currentB = readCurrentB();
  if (_lastError != 0) return data;
  
  data.currentC = readCurrentC();
  if (_lastError != 0) return data;
  
  data.activePower = readTotalActivePower();
  if (_lastError != 0) return data;
  
  data.reactivePower = readTotalReactivePower();
  if (_lastError != 0) return data;
  
  data.apparentPower = readTotalApparentPower();
  if (_lastError != 0) return data;
  
  data.powerFactor = readTotalPowerFactor();
  if (_lastError != 0) return data;
  
  data.frequency = readFrequency();
  if (_lastError != 0) return data;
  
  data.valid = true;
  return data;
}

// Đọc tỉ số PT từ đồng hồ
uint16_t ACREL_DTSD1352::readPTRatio() {
  return readHoldingRegister(REG_PT_RATIO);
}

// Đọc tỉ số CT từ đồng hồ
uint16_t ACREL_DTSD1352::readCTRatio() {
  return readHoldingRegister(REG_CT_RATIO);
}

// Đọc và tự động set PT/CT ratio từ đồng hồ
bool ACREL_DTSD1352::readAndSetRatios() {
  uint16_t pt = readPTRatio();
  if (_lastError != 0) {
    return false;
  }
  delay(500);
  
  uint16_t ct = readCTRatio();
  if (_lastError != 0) {
    return false;
  }
  
  // Nếu đọc được giá trị hợp lệ (>0), set vào
  if (pt > 0 && pt < 10000) {
    _ptRatio = pt;
  }
  
  if (ct > 0 && ct < 10000) {
    _ctRatio = ct;
  }
  
  return true;
}

// ============================================================================
// BATCH READING FUNCTIONS - ĐỌC NHIỀU THANH GHI CÙNG LÚC (TỐI ƯU)
// Giảm xuống 6-14 thanh ghi mỗi lần để tránh lỗi
// ============================================================================

// Đọc Voltage và Current cùng lúc (0x0061-0x0066)
// 6 thanh ghi liên tục
ACREL_DTSD1352::VoltageCurrentData ACREL_DTSD1352::readVoltageCurrentBatch() {
  VoltageCurrentData data;
  data.valid = false;
  
  uint16_t buffer[6];
  
  // Đọc 6 thanh ghi từ 0x0061 (Voltage A) đến 0x0066 (Current C)
  if (!readHoldingRegistersBatch(REG_VOLTAGE_A, 6, buffer)) {
    Serial.printf("[ERROR] readVoltageCurrentBatch: failed, error=0x%02X\n", _lastError);
    return data;
  }
  
  // Parse data
  data.voltageA = applyPTRatio(buffer[0], 0.1);
  data.voltageB = applyPTRatio(buffer[1], 0.1);
  data.voltageC = applyPTRatio(buffer[2], 0.1);
  
  data.currentA = applyCTRatio(buffer[3], 0.01);
  data.currentB = applyCTRatio(buffer[4], 0.01);
  data.currentC = applyCTRatio(buffer[5], 0.01);
  
  Serial.println("[OK] readVoltageCurrentBatch: Success");
  data.valid = true;
  return data;
}

// Đọc Frequency riêng (1 thanh ghi)
ACREL_DTSD1352::FrequencyData ACREL_DTSD1352::readFrequencyBatch() {
  FrequencyData data;
  data.valid = false;
  
  data.frequency = readFrequency();
  if (_lastError == 0) {
    Serial.println("[OK] readFrequencyBatch: Success");
    data.valid = true;
  } else {
    Serial.printf("[ERROR] readFrequencyBatch: failed, error=0x%02X\n", _lastError);
  }
  
  return data;
}

// Đọc Active Power (0x0164-0x016B)
// 8 thanh ghi (4 giá trị 32-bit)
ACREL_DTSD1352::PowerData ACREL_DTSD1352::readPowerBatch() {
  PowerData data;
  data.valid = false;
  
  uint16_t buffer[8];
  
  // Đọc 8 thanh ghi Active Power
  if (!readHoldingRegistersBatch(REG_ACTIVE_POWER_A, 8, buffer)) {
    Serial.printf("[ERROR] readPowerBatch: Active Power failed, error=0x%02X\n", _lastError);
    return data;
  }
  
  // Active Power (signed 32-bit)
  data.activePowerA = combine32Signed(buffer[0], buffer[1]) * _ptRatio * _ctRatio * 0.001;
  data.activePowerB = combine32Signed(buffer[2], buffer[3]) * _ptRatio * _ctRatio * 0.001;
  data.activePowerC = combine32Signed(buffer[4], buffer[5]) * _ptRatio * _ctRatio * 0.001;
  data.totalActivePower = combine32Signed(buffer[6], buffer[7]) * _ptRatio * _ctRatio * 0.001;
  
  delay(100);
  
  // Đọc 8 thanh ghi Reactive Power (0x016C-0x0173)
  if (!readHoldingRegistersBatch(REG_REACTIVE_POWER_A, 8, buffer)) {
    Serial.printf("[ERROR] readPowerBatch: Reactive Power failed, error=0x%02X\n", _lastError);
    return data;
  }
  
  // Reactive Power (signed 32-bit)
  data.reactivePowerA = combine32Signed(buffer[0], buffer[1]) * _ptRatio * _ctRatio * 0.001;
  data.reactivePowerB = combine32Signed(buffer[2], buffer[3]) * _ptRatio * _ctRatio * 0.001;
  data.reactivePowerC = combine32Signed(buffer[4], buffer[5]) * _ptRatio * _ctRatio * 0.001;
  data.totalReactivePower = combine32Signed(buffer[6], buffer[7]) * _ptRatio * _ctRatio * 0.001;
  
  Serial.println("[OK] readPowerBatch: Success");
  data.valid = true;
  return data;
}

// Đọc Apparent Power (0x0174-0x017B)
// 8 thanh ghi (4 giá trị 32-bit)
ACREL_DTSD1352::ApparentPowerData ACREL_DTSD1352::readApparentPowerBatch() {
  ApparentPowerData data;
  data.valid = false;
  
  uint16_t buffer[8];
  
  // Đọc 8 thanh ghi Apparent Power
  if (!readHoldingRegistersBatch(REG_APPARENT_POWER_A, 8, buffer)) {
    Serial.printf("[ERROR] readApparentPowerBatch: Apparent Power failed, error=0x%02X\n", _lastError);
    return data;
  }
  
  // Apparent Power (unsigned 32-bit)
  data.apparentPowerA = combine32(buffer[0], buffer[1]) * _ptRatio * _ctRatio * 0.001;
  data.apparentPowerB = combine32(buffer[2], buffer[3]) * _ptRatio * _ctRatio * 0.001;
  data.apparentPowerC = combine32(buffer[4], buffer[5]) * _ptRatio * _ctRatio * 0.001;
  data.totalApparentPower = combine32(buffer[6], buffer[7]) * _ptRatio * _ctRatio * 0.001;
  
  delay(100);
  
  // Đọc 4 thanh ghi Power Factor (0x017C-0x017F)
  uint16_t pfBuffer[4];
  if (!readHoldingRegistersBatch(REG_POWER_FACTOR_A, 4, pfBuffer)) {
    Serial.printf("[ERROR] readApparentPowerBatch: Power Factor failed, error=0x%02X\n", _lastError);
    return data;
  }
  
  // Power Factor (signed 16-bit)
  data.powerFactorA = (int16_t)pfBuffer[0] * 0.001;
  data.powerFactorB = (int16_t)pfBuffer[1] * 0.001;
  data.powerFactorC = (int16_t)pfBuffer[2] * 0.001;
  data.totalPowerFactor = (int16_t)pfBuffer[3] * 0.001;
  
  Serial.println("[OK] readApparentPowerBatch: Success");
  data.valid = true;
  return data;
}

// Đọc tất cả Energy cùng lúc - CHIA NHỎ ĐỂ TRÁNH LỖI
// Đọc từng cặp energy riêng lẻ thay vì đọc cả 56 thanh ghi
ACREL_DTSD1352::EnergyData ACREL_DTSD1352::readEnergyBatch() {
  EnergyData data;
  data.valid = false;
  
  // Đọc từng loại energy (2 thanh ghi mỗi lần)
  uint32_t value;
  
  // Total Active Energy (0x0000-0x0001)
  value = readHoldingRegister32(REG_TOTAL_ACTIVE_ENERGY);
  if (_lastError != 0) return data;
  data.totalActiveEnergy = applyPTCTRatio(value, 0.01);
  
  delay(50); // Delay nhỏ giữa các lần đọc
  
  // Forward Active Energy (0x000A-0x000B)
  value = readHoldingRegister32(REG_FORWARD_ACTIVE_ENERGY);
  if (_lastError != 0) return data;
  data.forwardActiveEnergy = applyPTCTRatio(value, 0.01);
  
  delay(50);
  
  // Reverse Active Energy (0x0014-0x0015)
  value = readHoldingRegister32(REG_REVERSE_ACTIVE_ENERGY);
  if (_lastError != 0) return data;
  data.reverseActiveEnergy = applyPTCTRatio(value, 0.01);
  
  delay(50);
  
  // Total Reactive Energy (0x001E-0x001F)
  value = readHoldingRegister32(REG_TOTAL_REACTIVE_ENERGY);
  if (_lastError != 0) return data;
  data.totalReactiveEnergy = applyPTCTRatio(value, 0.01);
  
  delay(50);
  
  // Forward Reactive Energy (0x0028-0x0029)
  value = readHoldingRegister32(REG_FORWARD_REACTIVE_ENERGY);
  if (_lastError != 0) return data;
  data.forwardReactiveEnergy = applyPTCTRatio(value, 0.01);
  
  delay(50);
  
  // Reverse Reactive Energy (0x0032-0x0033)
  value = readHoldingRegister32(REG_REVERSE_REACTIVE_ENERGY);
  if (_lastError != 0) return data;
  data.reverseReactiveEnergy = applyPTCTRatio(value, 0.01);
  
  data.valid = true;
  return data;
}

// Đọc THD (Total Harmonic Distortion) - RIÊNG BIỆT
// 6 thanh ghi liên tục (0x00B0-0x00B5)
ACREL_DTSD1352::THDData ACREL_DTSD1352::readTHDBatch() {
  THDData data;
  data.valid = false;
  
  uint16_t buffer[6];
  
  // Đọc 6 thanh ghi THD
  if (!readHoldingRegistersBatch(REG_THD_VOLTAGE_A, 6, buffer)) {
    return data;
  }
  
  // Parse THD data (đơn vị %)
  data.thdVoltageA = buffer[0] * 0.1;
  data.thdVoltageB = buffer[1] * 0.1;
  data.thdVoltageC = buffer[2] * 0.1;
  data.thdCurrentA = buffer[3] * 0.1;
  data.thdCurrentB = buffer[4] * 0.1;
  data.thdCurrentC = buffer[5] * 0.1;
  
  data.valid = true;
  return data;
}

// Đọc TẤT CẢ dữ liệu cùng lúc - TỐI ƯU NHẤT
// Chia nhỏ thành nhiều batch 6-14 thanh ghi
// KHÔNG bao gồm THD - phải gọi riêng readTHDBatch()
ACREL_DTSD1352::AllMeterData ACREL_DTSD1352::readAllDataBatch() {
  AllMeterData data;
  data.valid = false;
  
  // 1. Đọc Voltage & Current (6 thanh ghi)
  VoltageCurrentData vcData = readVoltageCurrentBatch();
  if (!vcData.valid) return data;
  
  data.voltageA = vcData.voltageA;
  data.voltageB = vcData.voltageB;
  data.voltageC = vcData.voltageC;
  data.currentA = vcData.currentA;
  data.currentB = vcData.currentB;
  data.currentC = vcData.currentC;
  
  delay(100);
  
  // 2. Đọc Frequency (1 thanh ghi)
  FrequencyData freqData = readFrequencyBatch();
  if (!freqData.valid) return data;
  data.frequency = freqData.frequency;
  
  delay(100);
  
  // 3. Đọc Active & Reactive Power (16 thanh ghi)
  PowerData powerData = readPowerBatch();
  if (!powerData.valid) return data;
  
  data.activePowerA = powerData.activePowerA;
  data.activePowerB = powerData.activePowerB;
  data.activePowerC = powerData.activePowerC;
  data.totalActivePower = powerData.totalActivePower;
  
  data.reactivePowerA = powerData.reactivePowerA;
  data.reactivePowerB = powerData.reactivePowerB;
  data.reactivePowerC = powerData.reactivePowerC;
  data.totalReactivePower = powerData.totalReactivePower;
  
  delay(100);
  
  // 4. Đọc Apparent Power & Power Factor (12 thanh ghi)
  ApparentPowerData appData = readApparentPowerBatch();
  if (!appData.valid) return data;
  
  data.apparentPowerA = appData.apparentPowerA;
  data.apparentPowerB = appData.apparentPowerB;
  data.apparentPowerC = appData.apparentPowerC;
  data.totalApparentPower = appData.totalApparentPower;
  
  data.powerFactorA = appData.powerFactorA;
  data.powerFactorB = appData.powerFactorB;
  data.powerFactorC = appData.powerFactorC;
  data.totalPowerFactor = appData.totalPowerFactor;
  
  delay(100);
  
  // 5. Đọc Energy (đã chia nhỏ thành 6 lần đọc)
  EnergyData energyData = readEnergyBatch();
  if (!energyData.valid) return data;
  
  data.totalActiveEnergy = energyData.totalActiveEnergy;
  data.forwardActiveEnergy = energyData.forwardActiveEnergy;
  data.reverseActiveEnergy = energyData.reverseActiveEnergy;
  data.totalReactiveEnergy = energyData.totalReactiveEnergy;
  data.forwardReactiveEnergy = energyData.forwardReactiveEnergy;
  data.reverseReactiveEnergy = energyData.reverseReactiveEnergy;
  
  delay(100);
  
  // 6. Đọc Demand (2 thanh ghi riêng lẻ)
  data.forwardActiveDemand = readForwardActiveDemand();
  if (_lastError != 0) return data;
  
  delay(50);
  
  data.reverseActiveDemand = readReverseActiveDemand();
  if (_lastError != 0) return data;
  
  data.valid = true;
  return data;
}
