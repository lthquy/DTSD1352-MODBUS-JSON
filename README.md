Tôi sử dụng board ESP32 M5Stack C3U kết nối module RS485 qua chân rx 6, tx 7. Chân A,B của module RS485 kết nối chân 21, 22 của đồng hồ Acrel DTSD1352.
Dữ liệu của đồng hồ được đọc thành 4 nhóm voltage, power, energy, thd và in ra kiểu json.
Dữ liệu đọc từ đồng hồ sẽ được lưu vào bộ đệm, khi quá thời hạn CACHE_TIMEOUT mặc định là 10 giây thì lại đọc từ đồng hồ.
File secret.h chứa tên và mật khẩu wifi.
Code tự động đọc giá tri PT và CT từ đồng hồ để tính công suất, để đảm bảo ổn định thì nên set giá trị này mặc định ở 2 dòng 233 và 234 file DTSD1352_SMART_CACHE.ino
  meter.setPTRatio(1); # Lắp trực tiếp, không dùng biến áp
  meter.setCTRatio(30); # Ví dụ dùng biến dòng 150A/5A

## 📁 Cấu trúc thư mục Arduino
Arduino/
├── libraries/
│   └── ACREL_DTSD1352/
│       ├── ACREL_DTSD1352.h
│       └── ACREL_DTSD1352.cpp
│
└── sketches/
    ├── MINIMAL_TEST/
    │   └── MINIMAL_TEST.ino
    │
    ├── DEBUG_CONNECTION/
    │   └── DEBUG_CONNECTION.ino
    │
    └── DTSD1352_SMART_CACHE/
        ├── DTSD1352_SMART_CACHE.ino
        └── secret.h

Email: lthquy@gmail.com
