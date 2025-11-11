/*
  ACREL DTSD1352 - THD Test Example
  
  Test đọc THD (Total Harmonic Distortion) của điện áp và dòng điện
  THD là riêng biệt, KHÔNG nằm trong readAllDataBatch()
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
  Serial.println("ACREL DTSD1352 - THD Test");
  Serial.println("==========================================\n");
  
  // Khởi tạo Modbus
  Serial1.begin(BAUDRATE, SERIAL_8E1, RX_PIN, TX_PIN);
  
  if (!meter.begin(&Serial1, DE_PIN, BAUDRATE)) {
    Serial.println("❌ Lỗi khởi tạo Modbus!");
    while(1) delay(1000);
  }
  
  Serial.println("✓ Modbus OK!\n");
  delay(1000);
  
  // Test kết nối
  float voltage = meter.readVoltageA();
  if (meter.getLastError() == 0) {
    Serial.printf("✓ Kết nối OK! Voltage A: %.1f V\n\n", voltage);
  } else {
    Serial.printf("❌ Lỗi kết nối: 0x%02X\n", meter.getLastError());
    while(1) delay(1000);
  }
  
  delay(1000);
}

void loop() {
  Serial.println("\n========================================");
  Serial.println(">>> TEST 1: Đọc THD từng giá trị");
  Serial.println("========================================");
  
  Serial.println("THD Điện áp:");
  Serial.printf("  Phase A: %.2f %%\n", meter.readTHDVoltageA());
  delay(200);
  Serial.printf("  Phase B: %.2f %%\n", meter.readTHDVoltageB());
  delay(200);
  Serial.printf("  Phase C: %.2f %%\n", meter.readTHDVoltageC());
  
  Serial.println("\nTHD Dòng điện:");
  Serial.printf("  Phase A: %.2f %%\n", meter.readTHDCurrentA());
  delay(200);
  Serial.printf("  Phase B: %.2f %%\n", meter.readTHDCurrentB());
  delay(200);
  Serial.printf("  Phase C: %.2f %%\n", meter.readTHDCurrentC());
  
  delay(2000);
  
  Serial.println("\n========================================");
  Serial.println(">>> TEST 2: Đọc THD Batch (6 thanh ghi)");
  Serial.println("========================================");
  
  unsigned long startTime = millis();
  ACREL_DTSD1352::THDData thdData = meter.readTHDBatch();
  unsigned long duration = millis() - startTime;
  
  if (thdData.valid) {
    Serial.printf("✓ Đọc thành công trong %lu ms\n\n", duration);
    
    Serial.println("THD Điện áp:");
    Serial.printf("  Phase A: %.2f %%\n", thdData.thdVoltageA);
    Serial.printf("  Phase B: %.2f %%\n", thdData.thdVoltageB);
    Serial.printf("  Phase C: %.2f %%\n", thdData.thdVoltageC);
    
    Serial.println("\nTHD Dòng điện:");
    Serial.printf("  Phase A: %.2f %%\n", thdData.thdCurrentA);
    Serial.printf("  Phase B: %.2f %%\n", thdData.thdCurrentB);
    Serial.printf("  Phase C: %.2f %%\n", thdData.thdCurrentC);
    
    // Đánh giá chất lượng điện
    Serial.println("\n--- Đánh giá chất lượng điện ---");
    
    float avgVoltageTHD = (thdData.thdVoltageA + thdData.thdVoltageB + thdData.thdVoltageC) / 3.0;
    float avgCurrentTHD = (thdData.thdCurrentA + thdData.thdCurrentB + thdData.thdCurrentC) / 3.0;
    
    Serial.printf("THD Điện áp trung bình: %.2f %%\n", avgVoltageTHD);
    if (avgVoltageTHD < 5.0) {
      Serial.println("  => Tốt (THD < 5%)");
    } else if (avgVoltageTHD < 8.0) {
      Serial.println("  => Chấp nhận được (5% < THD < 8%)");
    } else {
      Serial.println("  => Xấu (THD > 8%) - Cần kiểm tra!");
    }
    
    Serial.printf("\nTHD Dòng điện trung bình: %.2f %%\n", avgCurrentTHD);
    if (avgCurrentTHD < 10.0) {
      Serial.println("  => Tốt (THD < 10%)");
    } else if (avgCurrentTHD < 15.0) {
      Serial.println("  => Chấp nhận được (10% < THD < 15%)");
    } else {
      Serial.println("  => Xấu (THD > 15%) - Tải có nhiễu cao!");
    }
    
  } else {
    Serial.printf("❌ Lỗi đọc THD! Error: 0x%02X\n", meter.getLastError());
  }
  
  delay(3000);
  
  Serial.println("\n========================================");
  Serial.println(">>> TEST 3: Đọc ALL DATA + THD");
  Serial.println("========================================");
  
  startTime = millis();
  
  // Đọc tất cả data cơ bản (KHÔNG có THD)
  ACREL_DTSD1352::AllMeterData allData = meter.readAllDataBatch();
  
  unsigned long allDataTime = millis() - startTime;
  
  if (allData.valid) {
    Serial.printf("✓ All Data OK trong %lu ms\n", allDataTime);
    Serial.printf("  Voltage A: %.1f V\n", allData.voltageA);
    Serial.printf("  Current A: %.2f A\n", allData.currentA);
    Serial.printf("  Power: %.2f kW\n", allData.totalActivePower);
    Serial.printf("  Frequency: %.2f Hz\n", allData.frequency);
    
    delay(200);
    
    // Đọc THD riêng
    startTime = millis();
    thdData = meter.readTHDBatch();
    unsigned long thdTime = millis() - startTime;
    
    if (thdData.valid) {
      Serial.printf("\n✓ THD Data OK trong %lu ms\n", thdTime);
      Serial.printf("  THD Voltage A: %.2f %%\n", thdData.thdVoltageA);
      Serial.printf("  THD Current A: %.2f %%\n", thdData.thdCurrentA);
      
      Serial.printf("\n--- Tổng thời gian: %lu ms ---\n", allDataTime + thdTime);
    } else {
      Serial.printf("\n❌ Lỗi đọc THD: 0x%02X\n", meter.getLastError());
    }
    
  } else {
    Serial.printf("❌ Lỗi đọc All Data: 0x%02X\n", meter.getLastError());
  }
  
  Serial.println("\nChờ 10 giây...\n");
  delay(10000);
}
