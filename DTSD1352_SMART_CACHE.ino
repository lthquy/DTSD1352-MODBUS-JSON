/*
  ESP32 - ACREL DTSD1352 with SMART CACHE
  
  Version 3.0 - Cache-based reading
  - Không dùng readAllDataBatch định kỳ
  - Đọc từng nhóm khi gọi endpoint
  - Cache 10 giây
  - Logging chi tiết lỗi
*/

#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include "ACREL_DTSD1352.h"
#include "secret.h"

// Cấu hình RS485
#define RX_PIN 6
#define TX_PIN 7
#define DE_PIN 10
#define SLAVE_ID 1
#define BAUDRATE 9600

// Cache timeout (ms)
#define CACHE_TIMEOUT 10000  // 10 giây

ACREL_DTSD1352 meter(SLAVE_ID);
WebServer server(80);

// ============================================================================
// CACHE STRUCTURES
// ============================================================================

struct CachedVoltageCurrentData {
  ACREL_DTSD1352::VoltageCurrentData data;
  unsigned long timestamp;
  bool valid;
};
CachedVoltageCurrentData cacheVC;

struct CachedFrequencyData {
  ACREL_DTSD1352::FrequencyData data;
  unsigned long timestamp;
  bool valid;
};
CachedFrequencyData cacheFreq;

struct CachedPowerData {
  ACREL_DTSD1352::PowerData data;
  unsigned long timestamp;
  bool valid;
};
CachedPowerData cachePower;

struct CachedApparentPowerData {
  ACREL_DTSD1352::ApparentPowerData data;
  unsigned long timestamp;
  bool valid;
};
CachedApparentPowerData cacheApparent;

struct CachedEnergyData {
  ACREL_DTSD1352::EnergyData data;
  unsigned long timestamp;
  bool valid;
};
CachedEnergyData cacheEnergy;

struct CachedTHDData {
  ACREL_DTSD1352::THDData data;
  unsigned long timestamp;
  bool valid;
};
CachedTHDData cacheTHD;

// ============================================================================
// SMART CACHE FUNCTIONS
// ============================================================================

// Kiểm tra cache còn hiệu lực không
bool isCacheValid(unsigned long cacheTime) {
  return (millis() - cacheTime) < CACHE_TIMEOUT;
}

// Đọc Voltage & Current với cache
ACREL_DTSD1352::VoltageCurrentData getVoltageCurrentData() {
  if (cacheVC.valid && isCacheValid(cacheVC.timestamp)) {
    Serial.println("[CACHE] Using cached Voltage/Current data");
    return cacheVC.data;
  }
  
  Serial.println("[READ] Reading Voltage/Current from meter...");
  cacheVC.data = meter.readVoltageCurrentBatch();
  cacheVC.timestamp = millis();
  cacheVC.valid = cacheVC.data.valid;
  
  return cacheVC.data;
}

// Đọc Frequency với cache
ACREL_DTSD1352::FrequencyData getFrequencyData() {
  if (cacheFreq.valid && isCacheValid(cacheFreq.timestamp)) {
    Serial.println("[CACHE] Using cached Frequency data");
    return cacheFreq.data;
  }
  
  Serial.println("[READ] Reading Frequency from meter...");
  cacheFreq.data = meter.readFrequencyBatch();
  cacheFreq.timestamp = millis();
  cacheFreq.valid = cacheFreq.data.valid;
  
  return cacheFreq.data;
}

// Đọc Power với cache
ACREL_DTSD1352::PowerData getPowerData() {
  if (cachePower.valid && isCacheValid(cachePower.timestamp)) {
    Serial.println("[CACHE] Using cached Power data");
    return cachePower.data;
  }
  
  Serial.println("[READ] Reading Power from meter...");
  cachePower.data = meter.readPowerBatch();
  cachePower.timestamp = millis();
  cachePower.valid = cachePower.data.valid;
  
  return cachePower.data;
}

// Đọc Apparent Power với cache
ACREL_DTSD1352::ApparentPowerData getApparentPowerData() {
  if (cacheApparent.valid && isCacheValid(cacheApparent.timestamp)) {
    Serial.println("[CACHE] Using cached Apparent Power data");
    return cacheApparent.data;
  }
  
  Serial.println("[READ] Reading Apparent Power from meter...");
  cacheApparent.data = meter.readApparentPowerBatch();
  cacheApparent.timestamp = millis();
  cacheApparent.valid = cacheApparent.data.valid;
  
  return cacheApparent.data;
}

// Đọc Energy với cache
ACREL_DTSD1352::EnergyData getEnergyData() {
  if (cacheEnergy.valid && isCacheValid(cacheEnergy.timestamp)) {
    Serial.println("[CACHE] Using cached Energy data");
    return cacheEnergy.data;
  }
  
  Serial.println("[READ] Reading Energy from meter...");
  cacheEnergy.data = meter.readEnergyBatch();
  cacheEnergy.timestamp = millis();
  cacheEnergy.valid = cacheEnergy.data.valid;
  
  return cacheEnergy.data;
}

// Đọc THD với cache
ACREL_DTSD1352::THDData getTHDData() {
  if (cacheTHD.valid && isCacheValid(cacheTHD.timestamp)) {
    Serial.println("[CACHE] Using cached THD data");
    return cacheTHD.data;
  }
  
  Serial.println("[READ] Reading THD from meter...");
  cacheTHD.data = meter.readTHDBatch();
  cacheTHD.timestamp = millis();
  cacheTHD.valid = cacheTHD.data.valid;
  
  return cacheTHD.data;
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Khởi tạo cache
  cacheVC.valid = false;
  cacheVC.timestamp = 0;
  cacheFreq.valid = false;
  cacheFreq.timestamp = 0;
  cachePower.valid = false;
  cachePower.timestamp = 0;
  cacheApparent.valid = false;
  cacheApparent.timestamp = 0;
  cacheEnergy.valid = false;
  cacheEnergy.timestamp = 0;
  cacheTHD.valid = false;
  cacheTHD.timestamp = 0;
  
  Serial.println("\n\n========================================");
  Serial.println("ACREL DTSD1352 Smart Cache v3.0");
  Serial.println("========================================\n");
  
  // Khởi tạo Modbus
  Serial.println("1. Khoi tao Modbus RS485...");
  Serial1.begin(BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN);
  
  if (!meter.begin(&Serial1, DE_PIN, BAUDRATE)) {
    Serial.println("   X Loi khoi tao Modbus!");
    while(1) delay(1000);
  }
  Serial.println("   √ Modbus OK!\n");
  
  delay(500);
  
  // Test kết nối
  Serial.println("2. Test ket noi...");
  float voltage = meter.readVoltageA();
  if (meter.getLastError() == 0) {
    Serial.printf("   √ OK! Voltage A: %.1f V\n\n", voltage);
  } else {
    Serial.printf("   X Loi: 0x%02X\n", meter.getLastError());
    while(1) delay(1000);
  }
  
  delay(500);
  
  // Đọc PT/CT ratio
  Serial.println("3. Doc PT/CT ratio...");
  if (meter.readAndSetRatios()) {
    Serial.println("   √ Doc thanh cong!");
    delay(500);
    Serial.printf("   PT: %d, CT: %d\n", meter.readPTRatio(), meter.readCTRatio());
  } else {
    Serial.println("   X Loi! Dung gia tri mac dinh.");
    meter.setPTRatio(1);
    meter.setCTRatio(30);
  }
  
  // Kết nối WiFi
  Serial.println("\n4. Ket noi WiFi...");
  Serial.printf("   SSID: %s\n", ssid);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n   √ WiFi connected!");
    Serial.printf("   IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n   X WiFi failed!");
  }
  
  // Cấu hình WebServer
  Serial.println("\n5. Cau hinh WebServer...");
  server.on("/", handleRoot);
  server.on("/voltage", handleVoltage);
  server.on("/power", handlePower);
  server.on("/energy", handleEnergy);
  server.on("/thd", handleTHD);
  server.on("/status", handleStatus);
  server.on("/cache/clear", handleCacheClear);
  server.onNotFound(handleNotFound);
  
  server.begin();
  Serial.println("   √ WebServer started!");
  
  Serial.println("\n========================================");
  Serial.println("He thong san sang!");
  Serial.println("========================================");
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nTruy cap: http://%s\n", WiFi.localIP().toString().c_str());
  }
  
  Serial.println("\nEndpoints:");
  Serial.println("  /voltage    - Dien ap & dong dien");
  Serial.println("  /power      - Cong suat");
  Serial.println("  /energy     - Dien nang");
  Serial.println("  /thd        - Total Harmonic Distortion");
  Serial.println("  /status     - Trang thai & cache info");
  Serial.println("  /cache/clear - Xoa cache");
  Serial.println("\n========================================\n");
}

void loop() {
  server.handleClient();
  // Không đọc data định kỳ - chỉ đọc khi có request!
}

// ============================================================================
// WEB HANDLERS
// ============================================================================

void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<title>ACREL Smart Cache v3.0</title>";
  html += "<style>";
  html += "body{font-family:Arial,sans-serif;margin:0;padding:20px;background:#f0f0f0}";
  html += ".container{max-width:800px;margin:0 auto;background:white;padding:30px;border-radius:10px;box-shadow:0 2px 10px rgba(0,0,0,0.1)}";
  html += "h1{color:#333}";
  html += ".badge{display:inline-block;padding:5px 10px;border-radius:5px;font-size:12px;font-weight:bold;margin:5px}";
  html += ".badge-success{background:#4CAF50;color:white}";
  html += ".badge-info{background:#2196F3;color:white}";
  html += ".endpoint{background:#f5f5f5;padding:15px;margin:10px 0;border-radius:5px;border-left:4px solid #2196F3}";
  html += ".endpoint a{color:#2196F3;text-decoration:none;font-weight:bold}";
  html += ".endpoint a:hover{text-decoration:underline}";
  html += ".feature{color:#666;margin:5px 0}";
  html += "</style></head><body>";
  html += "<div class='container'>";
  html += "<h1>⚡ ACREL Smart Cache v3.0</h1>";
  html += "<span class='badge badge-success'>Cache: 10s</span>";
  html += "<span class='badge badge-info'>On-demand Reading</span>";
  
  html += "<h2>✨ Features</h2>";
  html += "<div class='feature'>📦 Smart cache - Tự động cập nhật khi cũ hơn 10s</div>";
  html += "<div class='feature'>🎯 On-demand - Chỉ đọc khi cần</div>";
  html += "<div class='feature'>📊 Logging - Theo dõi lỗi chi tiết</div>";
  html += "<div class='feature'>🚀 Hiệu quả - Giảm tải Modbus</div>";
  
  html += "<h2>📡 API Endpoints</h2>";
  
  html += "<div class='endpoint'>";
  html += "<a href='/voltage'>/voltage</a><br>";
  html += "Điện áp & dòng điện (6 thanh ghi)";
  html += "</div>";
  
  html += "<div class='endpoint'>";
  html += "<a href='/power'>/power</a><br>";
  html += "Công suất tác dụng & phản kháng (16 thanh ghi)";
  html += "</div>";
  
  html += "<div class='endpoint'>";
  html += "<a href='/energy'>/energy</a><br>";
  html += "Điện năng (6 cặp thanh ghi)";
  html += "</div>";
  
  html += "<div class='endpoint'>";
  html += "<a href='/thd'>/thd</a><br>";
  html += "Total Harmonic Distortion (6 thanh ghi)";
  html += "</div>";
  
  html += "<div class='endpoint'>";
  html += "<a href='/status'>/status</a><br>";
  html += "Trạng thái hệ thống & cache";
  html += "</div>";
  
  html += "<div class='endpoint'>";
  html += "<a href='/cache/clear'>/cache/clear</a><br>";
  html += "Xóa tất cả cache - Force refresh";
  html += "</div>";
  
  html += "</div></body></html>";
  
  server.send(200, "text/html", html);
}

void handleVoltage() {
  Serial.println("\n>>> /voltage endpoint called");
  
  StaticJsonDocument<512> doc;
  
  // Đọc Voltage & Current
  auto vcData = getVoltageCurrentData();
  
  if (!vcData.valid) {
    doc["status"] = "error";
    doc["message"] = "Failed to read voltage/current";
    doc["error_code"] = meter.getLastError();
  } else {
    doc["status"] = "ok";
    doc["cache_age"] = (millis() - cacheVC.timestamp) / 1000.0;
    
    JsonObject voltage = doc.createNestedObject("voltage");
    voltage["A"] = vcData.voltageA;
    voltage["B"] = vcData.voltageB;
    voltage["C"] = vcData.voltageC;
    voltage["unit"] = "V";
    
    JsonObject current = doc.createNestedObject("current");
    current["A"] = vcData.currentA;
    current["B"] = vcData.currentB;
    current["C"] = vcData.currentC;
    current["unit"] = "A";
    
    // Thêm frequency
    auto freqData = getFrequencyData();
    if (freqData.valid) {
      doc["frequency"] = freqData.frequency;
    }
  }
  
  String response;
  serializeJsonPretty(doc, response);
  server.send(200, "application/json", response);
}

void handlePower() {
  Serial.println("\n>>> /power endpoint called");
  
  StaticJsonDocument<768> doc;
  
  // Đọc Active & Reactive Power
  auto powerData = getPowerData();
  
  if (!powerData.valid) {
    doc["status"] = "error";
    doc["message"] = "Failed to read power";
    doc["error_code"] = meter.getLastError();
  } else {
    doc["status"] = "ok";
    doc["cache_age"] = (millis() - cachePower.timestamp) / 1000.0;
    
    JsonObject active = doc.createNestedObject("active_power");
    active["A"] = powerData.activePowerA;
    active["B"] = powerData.activePowerB;
    active["C"] = powerData.activePowerC;
    active["total"] = powerData.totalActivePower;
    active["unit"] = "kW";
    
    JsonObject reactive = doc.createNestedObject("reactive_power");
    reactive["A"] = powerData.reactivePowerA;
    reactive["B"] = powerData.reactivePowerB;
    reactive["C"] = powerData.reactivePowerC;
    reactive["total"] = powerData.totalReactivePower;
    reactive["unit"] = "kVar";
    
    // Thêm Apparent Power & PF
    auto appData = getApparentPowerData();
    if (appData.valid) {
      JsonObject apparent = doc.createNestedObject("apparent_power");
      apparent["A"] = appData.apparentPowerA;
      apparent["B"] = appData.apparentPowerB;
      apparent["C"] = appData.apparentPowerC;
      apparent["total"] = appData.totalApparentPower;
      apparent["unit"] = "kVA";
      
      JsonObject pf = doc.createNestedObject("power_factor");
      pf["A"] = appData.powerFactorA;
      pf["B"] = appData.powerFactorB;
      pf["C"] = appData.powerFactorC;
      pf["total"] = appData.totalPowerFactor;
    }
  }
  
  String response;
  serializeJsonPretty(doc, response);
  server.send(200, "application/json", response);
}

void handleEnergy() {
  Serial.println("\n>>> /energy endpoint called");
  
  StaticJsonDocument<512> doc;
  
  auto energyData = getEnergyData();
  
  if (!energyData.valid) {
    doc["status"] = "error";
    doc["message"] = "Failed to read energy";
    doc["error_code"] = meter.getLastError();
  } else {
    doc["status"] = "ok";
    doc["cache_age"] = (millis() - cacheEnergy.timestamp) / 1000.0;
    
    JsonObject active = doc.createNestedObject("active_energy");
    active["total"] = energyData.totalActiveEnergy;
    active["forward"] = energyData.forwardActiveEnergy;
    active["reverse"] = energyData.reverseActiveEnergy;
    active["unit"] = "kWh";
    
    JsonObject reactive = doc.createNestedObject("reactive_energy");
    reactive["total"] = energyData.totalReactiveEnergy;
    reactive["forward"] = energyData.forwardReactiveEnergy;
    reactive["reverse"] = energyData.reverseReactiveEnergy;
    reactive["unit"] = "kVarh";
  }
  
  String response;
  serializeJsonPretty(doc, response);
  server.send(200, "application/json", response);
}

void handleTHD() {
  Serial.println("\n>>> /thd endpoint called");
  
  StaticJsonDocument<384> doc;
  
  auto thdData = getTHDData();
  
  if (!thdData.valid) {
    doc["status"] = "error";
    doc["message"] = "Failed to read THD";
    doc["error_code"] = meter.getLastError();
  } else {
    doc["status"] = "ok";
    doc["cache_age"] = (millis() - cacheTHD.timestamp) / 1000.0;
    
    JsonObject voltage = doc.createNestedObject("thd_voltage");
    voltage["A"] = thdData.thdVoltageA;
    voltage["B"] = thdData.thdVoltageB;
    voltage["C"] = thdData.thdVoltageC;
    voltage["unit"] = "%";
    
    JsonObject current = doc.createNestedObject("thd_current");
    current["A"] = thdData.thdCurrentA;
    current["B"] = thdData.thdCurrentB;
    current["C"] = thdData.thdCurrentC;
    current["unit"] = "%";
    
    // Đánh giá
    float avgV = (thdData.thdVoltageA + thdData.thdVoltageB + thdData.thdVoltageC) / 3.0;
    if (avgV < 5.0) {
      doc["voltage_quality"] = "Good (< 5%)";
    } else if (avgV < 8.0) {
      doc["voltage_quality"] = "Fair (5-8%)";
    } else {
      doc["voltage_quality"] = "Poor (> 8%)";
    }
  }
  
  String response;
  serializeJsonPretty(doc, response);
  server.send(200, "application/json", response);
}

void handleStatus() {
  Serial.println("\n>>> /status endpoint called");
  
  StaticJsonDocument<768> doc;
  
  // WiFi info
  JsonObject wifi = doc.createNestedObject("wifi");
  wifi["connected"] = (WiFi.status() == WL_CONNECTED);
  wifi["ssid"] = WiFi.SSID();
  wifi["ip"] = WiFi.localIP().toString();
  wifi["rssi"] = WiFi.RSSI();
  
  // System info
  JsonObject system = doc.createNestedObject("system");
  system["uptime"] = millis() / 1000;
  system["free_heap"] = ESP.getFreeHeap();
  system["version"] = "3.0 Smart Cache";
  
  // Cache info
  JsonObject cache = doc.createNestedObject("cache");
  cache["timeout"] = CACHE_TIMEOUT / 1000;
  
  JsonObject cacheStatus = cache.createNestedObject("status");
  cacheStatus["voltage_current"] = cacheVC.valid ? "valid" : "empty";
  cacheStatus["frequency"] = cacheFreq.valid ? "valid" : "empty";
  cacheStatus["power"] = cachePower.valid ? "valid" : "empty";
  cacheStatus["apparent"] = cacheApparent.valid ? "valid" : "empty";
  cacheStatus["energy"] = cacheEnergy.valid ? "valid" : "empty";
  cacheStatus["thd"] = cacheTHD.valid ? "valid" : "empty";
  
  JsonObject cacheAge = cache.createNestedObject("age_seconds");
  if (cacheVC.valid) cacheAge["voltage_current"] = (millis() - cacheVC.timestamp) / 1000.0;
  if (cacheFreq.valid) cacheAge["frequency"] = (millis() - cacheFreq.timestamp) / 1000.0;
  if (cachePower.valid) cacheAge["power"] = (millis() - cachePower.timestamp) / 1000.0;
  if (cacheApparent.valid) cacheAge["apparent"] = (millis() - cacheApparent.timestamp) / 1000.0;
  if (cacheEnergy.valid) cacheAge["energy"] = (millis() - cacheEnergy.timestamp) / 1000.0;
  if (cacheTHD.valid) cacheAge["thd"] = (millis() - cacheTHD.timestamp) / 1000.0;
  
  String response;
  serializeJsonPretty(doc, response);
  server.send(200, "application/json", response);
}

void handleCacheClear() {
  Serial.println("\n>>> /cache/clear called - Clearing all cache");
  
  cacheVC.valid = false;
  cacheFreq.valid = false;
  cachePower.valid = false;
  cacheApparent.valid = false;
  cacheEnergy.valid = false;
  cacheTHD.valid = false;
  
  StaticJsonDocument<128> doc;
  doc["status"] = "ok";
  doc["message"] = "All cache cleared";
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleNotFound() {
  StaticJsonDocument<128> doc;
  doc["error"] = "Not Found";
  doc["message"] = "Endpoint not found";
  
  String response;
  serializeJson(doc, response);
  server.send(404, "application/json", response);
}
