# esp32-mpu6050

PHẦN 1 — Nạp Firmware Lên ESP32
Cách A: Dùng Arduino IDE (khuyên dùng)
Tải Arduino IDE 2: https://www.arduino.cc/en/software
Mở File → Open → chọn esp32_sender.ino
Tools → Board → ESP32 Arduino → ESP32 Dev Module
Tools → Port → COM7
Nhấn nút Upload (→)
PHẦN 2 — Chạy Python Analyzer
Mở Command Prompt tại thư mục dự án
Cách 1: Giữ Shift + Click chuột phải vào thư mục cambien goc nghien → Open PowerShell window here

Cách 2: Gõ cmd vào thanh địa chỉ của File Explorer
.venv\Scripts\activate
Kích hoạt môi trường Python
Dấu nhắc đổi thành (.venv) C:\...\cambien goc nghien>
Chạy chương trình
python fft_analysis.py --port COM7
Cửa sổ matplotlib sẽ mở ra với 5 panels realtime.
