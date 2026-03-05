/**
 * esp32_sender.ino — ESP32 + MPU6050
 * ====================================
 * Gửi gia tốc trục Z (m/s²) lên Serial @ 50 Hz, định dạng:
 *   Z:<float>\r\n
 *
 * Ví dụ output:
 *   Z:9.7813
 *   Z:9.7724
 *   Z:9.7901
 *
 * Kết nối phần cứng:
 *   MPU6050  →  ESP32
 *   VCC      →  3V3
 *   GND      →  GND
 *   SCL      →  GPIO 22
 *   SDA      →  GPIO 21
 *   AD0      →  GND  (địa chỉ I2C = 0x68)
 *
 * Thư viện cần cài (Arduino Library Manager):
 *   - Adafruit MPU6050
 *   - Adafruit BusIO
 *   - Adafruit Unified Sensor
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ── Tham số ──────────────────────────────────────────────────────────────────
#define SAMPLE_RATE_HZ   50          // Tần số lấy mẫu (Hz)
#define BAUD_RATE        115200      // Baud rate Serial (phải khớp Python)
#define SAMPLE_INTERVAL  (1000UL / SAMPLE_RATE_HZ)  // 20 ms

// ── Đối tượng ────────────────────────────────────────────────────────────────
static Adafruit_MPU6050 mpu;

// ── Biến thời gian ───────────────────────────────────────────────────────────
static unsigned long s_last_ms = 0;

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(BAUD_RATE);
    delay(200);

    // Khởi tạo I2C + MPU6050
    Wire.begin();
    if (!mpu.begin()) {
        Serial.println("[ERROR] MPU6050 khong tim thay!");
        for (;;) delay(1000);
    }

    // Cấu hình tối ưu cho cảm biến hô hấp
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);    // ±2g — độ nhạy cao nhất
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);      // Bộ lọc nội: 10 Hz

    Serial.println("[OK] MPU6050 san sang. Bat dau gui du lieu...");
    s_last_ms = millis();
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    unsigned long now = millis();

    // Chặn vòng lặp cho đến khi đủ 20 ms — đảm bảo sample rate ổn định
    if (now - s_last_ms < SAMPLE_INTERVAL) return;
    s_last_ms = now;

    // Đọc dữ liệu
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    // In trục Z (m/s²) — thay đổi theo chuyển động ngực/bụng khi hô hấp
    // Định dạng: "Z:<4 chữ số thập phân>\r\n"
    Serial.print("Z:");
    Serial.println(accel.acceleration.z, 4);
}
