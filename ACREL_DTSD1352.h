/*
  ACREL_DTSD1352.h - Library for ACREL DTSD1352 Energy Meter
  Compatible with MODBUS-RTU protocol
  
  Supports both:
  - ArduinoModbus (for Arduino boards)
  - ModbusMaster (for ESP32, ESP8266, and other boards)
  
  Version 2.0 - Added batch reading support for faster data acquisition
*/

#ifndef ACREL_DTSD1352_H
#define ACREL_DTSD1352_H

#include <Arduino.h>

// Force ModbusMaster for ESP32/ESP8266
#if defined(ESP32) || defined(ESP8266) || defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
  #define USE_MODBUS_MASTER
  #include <ModbusMaster.h>
#else
  // Only include Arduino libraries if NOT ESP
  #include <ArduinoRS485.h>
  #include <ArduinoModbus.h>
#endif

// Register addresses
#define REG_TOTAL_ACTIVE_ENERGY         0x0000
#define REG_FORWARD_ACTIVE_ENERGY       0x000A
#define REG_REVERSE_ACTIVE_ENERGY       0x0014
#define REG_TOTAL_REACTIVE_ENERGY       0x001E
#define REG_FORWARD_REACTIVE_ENERGY     0x0028
#define REG_REVERSE_REACTIVE_ENERGY     0x0032

#define REG_VOLTAGE_A                   0x0061
#define REG_VOLTAGE_B                   0x0062
#define REG_VOLTAGE_C                   0x0063
#define REG_CURRENT_A                   0x0064
#define REG_CURRENT_B                   0x0065
#define REG_CURRENT_C                   0x0066
#define REG_FREQUENCY                   0x0077

#define REG_ACTIVE_POWER_A              0x0164
#define REG_ACTIVE_POWER_B              0x0166
#define REG_ACTIVE_POWER_C              0x0168
#define REG_TOTAL_ACTIVE_POWER          0x016A

#define REG_REACTIVE_POWER_A            0x016C
#define REG_REACTIVE_POWER_B            0x016E
#define REG_REACTIVE_POWER_C            0x0170
#define REG_TOTAL_REACTIVE_POWER        0x0172

#define REG_APPARENT_POWER_A            0x0174
#define REG_APPARENT_POWER_B            0x0176
#define REG_APPARENT_POWER_C            0x0178
#define REG_TOTAL_APPARENT_POWER        0x017A

#define REG_POWER_FACTOR_A              0x017C
#define REG_POWER_FACTOR_B              0x017D
#define REG_POWER_FACTOR_C              0x017E
#define REG_TOTAL_POWER_FACTOR          0x017F

#define REG_FORWARD_ACTIVE_DEMAND       0x007B
#define REG_REVERSE_ACTIVE_DEMAND       0x007E

#define REG_PT_RATIO                    0x008D
#define REG_CT_RATIO                    0x008E

#define REG_TEMP_T1                     0x2000
#define REG_TEMP_T2                     0x2001
#define REG_TEMP_T3                     0x2002

// THD Registers (Total Harmonic Distortion)
#define REG_THD_VOLTAGE_A               0x00B0
#define REG_THD_VOLTAGE_B               0x00B1
#define REG_THD_VOLTAGE_C               0x00B2
#define REG_THD_CURRENT_A               0x00B3
#define REG_THD_CURRENT_B               0x00B4
#define REG_THD_CURRENT_C               0x00B5

class ACREL_DTSD1352 {
  public:
    // Constructor
    ACREL_DTSD1352(uint8_t slaveId = 1);
    
    // Initialization methods
#ifdef USE_MODBUS_MASTER
    // For ESP32/ESP8266 using ModbusMaster
    bool begin(HardwareSerial *serial, long baudrate = 9600);
    bool begin(HardwareSerial *serial, int8_t dePin, long baudrate = 9600);
    void preTransmission();
    void postTransmission();
#else
    // For Arduino using ArduinoModbus
    bool begin(long baudrate = 9600, uint16_t config = SERIAL_8N1);
    bool begin(HardwareSerial &serial, long baudrate = 9600, uint16_t config = SERIAL_8N1);
    bool begin(int8_t txPin, int8_t rxPin, int8_t dePin, long baudrate = 9600, uint16_t config = SERIAL_8N1);
#endif
    
    // Set PT and CT ratios
    void setPTRatio(uint16_t ratio);
    void setCTRatio(uint16_t ratio);
    
    // Energy readings (kWh, kVarh)
    float readTotalActiveEnergy();
    float readForwardActiveEnergy();
    float readReverseActiveEnergy();
    float readTotalReactiveEnergy();
    float readForwardReactiveEnergy();
    float readReverseReactiveEnergy();
    
    // Voltage readings (V)
    float readVoltageA();
    float readVoltageB();
    float readVoltageC();
    
    // Current readings (A)
    float readCurrentA();
    float readCurrentB();
    float readCurrentC();
    
    // Frequency (Hz)
    float readFrequency();
    
    // Power readings (kW, kVar, kVA)
    float readActivePowerA();
    float readActivePowerB();
    float readActivePowerC();
    float readTotalActivePower();
    
    float readReactivePowerA();
    float readReactivePowerB();
    float readReactivePowerC();
    float readTotalReactivePower();
    
    float readApparentPowerA();
    float readApparentPowerB();
    float readApparentPowerC();
    float readTotalApparentPower();
    
    // Power Factor
    float readPowerFactorA();
    float readPowerFactorB();
    float readPowerFactorC();
    float readTotalPowerFactor();
    
    // Demand (kW, kVar)
    float readForwardActiveDemand();
    float readReverseActiveDemand();
    
    // Temperature readings (°C)
    float readTemperatureT1();
    float readTemperatureT2();
    float readTemperatureT3();
    
    // THD readings (%)
    float readTHDVoltageA();
    float readTHDVoltageB();
    float readTHDVoltageC();
    float readTHDCurrentA();
    float readTHDCurrentB();
    float readTHDCurrentC();
    
    // Đọc tỉ số PT và CT từ đồng hồ
    uint16_t readPTRatio();
    uint16_t readCTRatio();
    bool readAndSetRatios(); // Đọc và tự động set PT/CT ratio

    // Read all basic parameters at once
    struct BasicData {
      float voltageA, voltageB, voltageC;
      float currentA, currentB, currentC;
      float activePower, reactivePower, apparentPower;
      float powerFactor;
      float frequency;
      bool valid;
    };
    BasicData readBasicData();

    // BATCH READING - Đọc nhiều thanh ghi cùng lúc (TỐI ƯU)
    // Giảm xuống 10-14 thanh ghi mỗi batch để tránh lỗi
    
    struct VoltageCurrentData {
      float voltageA, voltageB, voltageC;
      float currentA, currentB, currentC;
      bool valid;
    };
    VoltageCurrentData readVoltageCurrentBatch();
    
    struct FrequencyData {
      float frequency;
      bool valid;
    };
    FrequencyData readFrequencyBatch();
    
    struct PowerData {
      float activePowerA, activePowerB, activePowerC, totalActivePower;
      float reactivePowerA, reactivePowerB, reactivePowerC, totalReactivePower;
      bool valid;
    };
    PowerData readPowerBatch();
    
    struct ApparentPowerData {
      float apparentPowerA, apparentPowerB, apparentPowerC, totalApparentPower;
      float powerFactorA, powerFactorB, powerFactorC, totalPowerFactor;
      bool valid;
    };
    ApparentPowerData readApparentPowerBatch();
    
    struct EnergyData {
      float totalActiveEnergy, forwardActiveEnergy, reverseActiveEnergy;
      float totalReactiveEnergy, forwardReactiveEnergy, reverseReactiveEnergy;
      bool valid;
    };
    EnergyData readEnergyBatch();
    
    // THD Data - RIÊNG BIỆT, không trong readAllDataBatch
    struct THDData {
      float thdVoltageA, thdVoltageB, thdVoltageC;
      float thdCurrentA, thdCurrentB, thdCurrentC;
      bool valid;
    };
    THDData readTHDBatch();
    
    struct AllMeterData {
      // Voltage & Current
      float voltageA, voltageB, voltageC;
      float currentA, currentB, currentC;
      
      // Power
      float activePowerA, activePowerB, activePowerC, totalActivePower;
      float reactivePowerA, reactivePowerB, reactivePowerC, totalReactivePower;
      float apparentPowerA, apparentPowerB, apparentPowerC, totalApparentPower;
      float powerFactorA, powerFactorB, powerFactorC, totalPowerFactor;
      
      // Energy
      float totalActiveEnergy, forwardActiveEnergy, reverseActiveEnergy;
      float totalReactiveEnergy, forwardReactiveEnergy, reverseReactiveEnergy;
      
      // Demand
      float forwardActiveDemand, reverseActiveDemand;
      
      // Frequency (đọc riêng)
      float frequency;
      
      bool valid;
    };
    AllMeterData readAllDataBatch();
    
    // Low-level read functions
    uint16_t readHoldingRegister(uint16_t address);
    uint32_t readHoldingRegister32(uint16_t address);
    int16_t readHoldingRegisterSigned(uint16_t address);
    int32_t readHoldingRegister32Signed(uint16_t address);
    
    // Batch read function - đọc nhiều thanh ghi liên tục
    bool readHoldingRegistersBatch(uint16_t startAddress, uint16_t quantity, uint16_t* buffer);
    
    // Check last error
    int getLastError();
    
  private:
    uint8_t _slaveId;
    uint16_t _ptRatio;
    uint16_t _ctRatio;
    int _lastError;
    
#ifdef USE_MODBUS_MASTER
    ModbusMaster _modbus;
    HardwareSerial *_serial;
    int8_t _dePin;
    static ACREL_DTSD1352 *_instance; // For static callbacks
#endif
    
    float applyPTRatio(uint16_t value, float multiplier);
    float applyCTRatio(uint16_t value, float multiplier);
    float applyPTCTRatio(uint32_t value, float multiplier);
    
    // Helper functions for batch reading
    uint32_t combine32(uint16_t high, uint16_t low);
    int32_t combine32Signed(uint16_t high, uint16_t low);
};

#endif
