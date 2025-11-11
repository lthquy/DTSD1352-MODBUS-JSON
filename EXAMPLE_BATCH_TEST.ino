/*
  ACREL DTSD1352 - Simple Batch Reading Test
  
  Test đơn giản để kiểm tra hiệu năng batch reading
  So sánh giữa Sequential vs Batch Reading
*/

#include "ACREL_DTSD1352.h"

// Cấu hình RS485
#define RX_PIN 6
#define TX_PIN 7
#define DE_PIN 10
#define SLAVE_ID 1
#define BAUDRATE 9600

ACREL_DTSD1352 meter(SLAVE_ID);

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n==========================================");
  Serial.println("ACREL DTSD1352 - Batch Reading Test");
  Serial.println("==========================================\n");
  
  // Khởi tạo Serial1 cho Modbus
  Serial1.begin(BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN);
  
  // Khởi tạo meter với DE pin
  if (!meter.begin(&Serial1, DE_PIN, BAUDRATE)) {
    Serial.println("❌ Lỗi khởi tạo Modbus!");
    while(1) delay(1000);
  }
  
  Serial.println("✅ Khởi tạo Modbus thành công!\n");
  delay(1000);
  
  // Test kết nối
  Serial.println("Test kết nối...");
  float testVoltage = meter.readVoltageA();
  if (meter.getLastError() != 0) {
    Serial.printf("❌ Lỗi kết nối: 0x%02X\n", meter.getLastError());
    Serial.println("Kiểm tra kết nối RS485!");
    while(1) delay(1000);
  }
  Serial.printf("✅ Kết nối OK! Voltage A: %.1f V\n\n", testVoltage);
  delay(1000);
  
  // Đọc và set PT/CT ratio
  Serial.println("Đọc PT/CT ratio từ đồng hồ...");
  if (meter.readAndSetRatios()) {
    delay(500);
    Serial.printf("  PT Ratio: %d\n", meter.readPTRatio());
    delay(500);
    Serial.printf("  CT Ratio: %d\n", meter.readCTRatio());
  } else {
    Serial.println("  Sử dụng ratio mặc định: PT=1, CT=1");
    meter.setPTRatio(1);
    meter.setCTRatio(1);
  }
  
  Serial.println("\n==========================================");
  Serial.println("Bắt đầu test hiệu năng");
  Serial.println("==========================================\n");
  delay(2000);
}

void loop() {
  // ========================================
  // TEST 1: SEQUENTIAL READING (Cách cũ)
  // ========================================
  Serial.println("\n>>> TEST 1: Sequential Reading (Cách cũ)");
  Serial.println("-------------------------------------------");
  
  unsigned long startTime = millis();
  
  float voltageA = meter.readVoltageA();
  float voltageB = meter.readVoltageB();
  float voltageC = meter.readVoltageC();
  
  float currentA = meter.readCurrentA();
  float currentB = meter.readCurrentB();
  float currentC = meter.readCurrentC();
  
  float frequency = meter.readFrequency();
  
  float activePowerA = meter.readActivePowerA();
  float activePowerB = meter.readActivePowerB();
  float activePowerC = meter.readActivePowerC();
  float totalActivePower = meter.readTotalActivePower();
  
  unsigned long sequentialTime = millis() - startTime;
  
  Serial.printf("Thời gian: %lu ms\n", sequentialTime);
  Serial.printf("Số lần gọi: 11\n");
  Serial.printf("Voltage A: %.1f V\n", voltageA);
  Serial.printf("Current A: %.2f A\n", currentA);
  Serial.printf("Power: %.2f kW\n", totalActivePower);
  
  delay(3000);
  
  // ========================================
  // TEST 2: BATCH READING (Cách mới)
  // ========================================
  Serial.println("\n>>> TEST 2: Batch Reading (Cách mới)");
  Serial.println("-------------------------------------------");
  
  startTime = millis();
  
  // Đọc Voltage & Current batch
  ACREL_DTSD1352::VoltageCurrentData vcData = meter.readVoltageCurrentBatch();
  
  // Đọc Power batch
  ACREL_DTSD1352::PowerData powerData = meter.readPowerBatch();
  
  unsigned long batchTime = millis() - startTime;
  
  if (vcData.valid && powerData.valid) {
    Serial.printf("Thời gian: %lu ms\n", batchTime);
    Serial.printf("Số lần gọi: 2\n");
    Serial.printf("Voltage A: %.1f V\n", vcData.voltageA);
    Serial.printf("Current A: %.2f A\n", vcData.currentA);
    Serial.printf("Power: %.2f kW\n", powerData.totalActivePower);
    
    // So sánh
    Serial.println("\n-------------------------------------------");
    Serial.println("KẾT QUẢ SO SÁNH");
    Serial.println("-------------------------------------------");
    Serial.printf("Sequential: %lu ms (11 lần gọi)\n", sequentialTime);
    Serial.printf("Batch:      %lu ms (2 lần gọi)\n", batchTime);
    
    float speedup = (float)sequentialTime / batchTime;
    Serial.printf("Tăng tốc:   %.1fx nhanh hơn\n", speedup);
    
    int reduction = (int)(100.0 * (1.0 - (float)batchTime / sequentialTime));
    Serial.printf("Giảm thời gian: %d%%\n", reduction);
    
  } else {
    Serial.println("❌ Lỗi batch reading!");
    Serial.printf("Error code: 0x%02X\n", meter.getLastError());
  }
  
  delay(5000);
  
  // ========================================
  // TEST 3: READ ALL DATA BATCH
  // ========================================
  Serial.println("\n>>> TEST 3: Read All Data (Tất cả dữ liệu)");
  Serial.println("-------------------------------------------");
  
  startTime = millis();
  
  ACREL_DTSD1352::AllMeterData allData = meter.readAllDataBatch();
  
  unsigned long allDataTime = millis() - startTime;
  
  if (allData.valid) {
    Serial.printf("Thời gian: %lu ms\n", allDataTime);
    Serial.printf("Số lần gọi: 4\n\n");
    
    Serial.println("Điện áp (V):");
    Serial.printf("  A: %.1f, B: %.1f, C: %.1f\n", 
                  allData.voltageA, allData.voltageB, allData.voltageC);
    
    Serial.println("Dòng điện (A):");
    Serial.printf("  A: %.2f, B: %.2f, C: %.2f\n", 
                  allData.currentA, allData.currentB, allData.currentC);
    
    Serial.println("Công suất tác dụng (kW):");
    Serial.printf("  A: %.2f, B: %.2f, C: %.2f, Total: %.2f\n", 
                  allData.activePowerA, allData.activePowerB, 
                  allData.activePowerC, allData.totalActivePower);
    
    Serial.println("Công suất biểu kiến (kVA):");
    Serial.printf("  A: %.2f, B: %.2f, C: %.2f, Total: %.2f\n", 
                  allData.apparentPowerA, allData.apparentPowerB, 
                  allData.apparentPowerC, allData.totalApparentPower);
    
    Serial.println("Hệ số công suất:");
    Serial.printf("  A: %.3f, B: %.3f, C: %.3f, Total: %.3f\n", 
                  allData.powerFactorA, allData.powerFactorB, 
                  allData.powerFactorC, allData.totalPowerFactor);
    
    Serial.println("Điện năng (kWh):");
    Serial.printf("  Total: %.2f, Forward: %.2f, Reverse: %.2f\n", 
                  allData.totalActiveEnergy, allData.forwardActiveEnergy, 
                  allData.reverseActiveEnergy);
    
    Serial.printf("Tần số: %.2f Hz\n", allData.frequency);
    
  } else {
    Serial.println("❌ Lỗi đọc dữ liệu!");
    Serial.printf("Error code: 0x%02X\n", meter.getLastError());
  }
  
  Serial.println("\n==========================================");
  Serial.println("Chờ 10 giây trước khi test tiếp...");
  Serial.println("==========================================");
  
  delay(10000);
}
