
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

// ==================== I2C ====================

static constexpr int SDA_PIN = 6;
static constexpr int SCL_PIN = 7;

static constexpr uint8_t MPU_ADDR = 0x68;
static constexpr uint8_t OLED_ADDR = 0x3C;

// ==================== OLED ====================

static constexpr int SCREEN_WIDTH = 128;
static constexpr int SCREEN_HEIGHT = 64;

Adafruit_SSD1306 display(
    SCREEN_WIDTH,
    SCREEN_HEIGHT,
    &Wire,
    -1
);

// ==================== MPU6050 ====================

static constexpr float ACC_SCALE = 16384.0f;
static constexpr float GYRO_SCALE = 131.0f;

static constexpr float ALPHA = 0.98f;

float gyro_offset_x = 0;
float gyro_offset_y = 0;

float roll = 0;
float pitch = 0;

static constexpr int CALIB_SAMPLES = 500;
static constexpr int CALIB_INTERVAL_MS = 10;

unsigned long last_sample_time = 0;
unsigned long last_display_time = 0;

// ==================== MPU Functions ====================

bool readMPU6050(int16_t &ax, int16_t &ay, int16_t &az,
                 int16_t &gx, int16_t &gy, int16_t &gz) {

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);

    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    if (Wire.requestFrom(MPU_ADDR, (uint8_t)14) != 14) {
        return false;
    }

    ax = (Wire.read() << 8) | Wire.read();
    ay = (Wire.read() << 8) | Wire.read();
    az = (Wire.read() << 8) | Wire.read();

    Wire.read();
    Wire.read(); // Temperature

    gx = (Wire.read() << 8) | Wire.read();
    gy = (Wire.read() << 8) | Wire.read();
    gz = (Wire.read() << 8) | Wire.read();

    return true;
}

void calibrateGyro() {
    Serial.println("Calibrating gyro... Keep MPU6050 still.");

    long sum_x = 0;
    long sum_y = 0;
    int  valid  = 0;

    for (int i = 0; i < CALIB_SAMPLES; i++) {
        int16_t ax, ay, az, gx, gy, gz;

        if (readMPU6050(ax, ay, az, gx, gy, gz)) {
            sum_x += gx;
            sum_y += gy;
            valid++;
        }

        delay(CALIB_INTERVAL_MS);
    }

    if (valid == 0) {
        Serial.println("Calibration FAILED: no sample read from MPU6050.");
        Serial.println("Check wiring, 3V3 power and I2C address (0x68).");
        return;
    }

    // 只对成功读到的样本求平均，避免失败样本把零偏拉向 0
    gyro_offset_x = (float)sum_x / (float)valid;
    gyro_offset_y = (float)sum_y / (float)valid;

    Serial.printf(
        "Calibration done: %d/%d samples, offset x=%.2f y=%.2f\n",
        valid, CALIB_SAMPLES, gyro_offset_x, gyro_offset_y
    );

    if (valid < CALIB_SAMPLES / 2) {
        Serial.println("Warning: too many failed samples, angle may drift.");
    }
}

// ==================== Setup ====================

void setup() {
    Serial.begin(115200);

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);

    // Wake up MPU6050
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();

    // Initialize OLED
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println("OLED initialization failed!");
        while (true) {
            delay(1000);
        }
    }

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("MPU6050 Monitor");
    display.println();
    display.println("Initializing...");
    display.display();

    calibrateGyro();

    last_sample_time = micros();

    display.clearDisplay();
    display.display();

    Serial.println("System ready.");
}

// ==================== Loop ====================

void loop() {
    unsigned long now = micros();

    float dt = (now - last_sample_time) / 1000000.0f;

    if (dt >= 0.01f) {
        last_sample_time = now;

        int16_t ax_raw, ay_raw, az_raw;
        int16_t gx_raw, gy_raw, gz_raw;

        if (readMPU6050(
                ax_raw, ay_raw, az_raw,
                gx_raw, gy_raw, gz_raw)) {

            float ax = ax_raw / ACC_SCALE;
            float ay = ay_raw / ACC_SCALE;
            float az = az_raw / ACC_SCALE;

            float gx = (gx_raw - gyro_offset_x) / GYRO_SCALE;
            float gy = (gy_raw - gyro_offset_y) / GYRO_SCALE;

            // Accelerometer angles
            float roll_acc =
                atan2f(ay, az) * 180.0f / PI;

            float pitch_acc =
                atan2f(-ax, sqrtf(ay * ay + az * az))
                * 180.0f / PI;

            // Complementary filter
            float roll_gyro = roll + gx * dt;
            float pitch_gyro = pitch + gy * dt;

            roll = ALPHA * roll_gyro
                 + (1.0f - ALPHA) * roll_acc;

            pitch = ALPHA * pitch_gyro
                  + (1.0f - ALPHA) * pitch_acc;
        }
    }

    // Update OLED every 200ms
    if (millis() - last_display_time >= 200) {
        last_display_time = millis();

        display.clearDisplay();

        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println("MPU6050  IMU Monitor");

        display.drawLine(0, 12, 127, 12, SSD1306_WHITE);

        display.setTextSize(1);
        display.setCursor(0, 19);
        display.print("ROLL");

        display.setTextSize(2);
        display.setCursor(0, 29);
        display.print(roll, 1);
        display.print(" deg");

        display.setTextSize(1);
        display.setCursor(0, 49);
        display.print("PITCH");

        display.setTextSize(2);
        display.setCursor(50, 46);
        display.print(pitch, 1);

        display.display();

        Serial.printf(
            "Roll: %.2f | Pitch: %.2f\n",
            roll, pitch
        );
    }
}

