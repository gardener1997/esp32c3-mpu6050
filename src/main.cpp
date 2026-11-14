
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

// 寄存器地址
static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
static constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
static constexpr uint8_t REG_CONFIG       = 0x1A;
static constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
static constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
static constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
static constexpr uint8_t REG_WHO_AM_I     = 0x75;

// DLPF_CFG = 3: 加速度 44 Hz / 陀螺仪 42 Hz 低通，约 4.9 ms 延迟
static constexpr uint8_t DLPF_CFG = 0x03;

// 开启 DLPF 后陀螺仪输出为 1 kHz，分频 1+9 得到 100 Hz 采样率
static constexpr uint8_t SAMPLE_RATE_DIV = 0x09;

bool writeMPURegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

int readMPURegister(uint8_t reg) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);

    if (Wire.endTransmission(false) != 0) {
        return -1;
    }

    if (Wire.requestFrom(MPU_ADDR, (uint8_t)1) != 1) {
        return -1;
    }

    return Wire.read();
}

// 初始化失败时扫一遍总线，方便定位接线/地址问题
void scanI2CBus() {
    Serial.print("I2C scan:");

    int found = 0;

    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);

        if (Wire.endTransmission() == 0) {
            Serial.printf(" 0x%02X", addr);
            found++;
        }
    }

    if (found == 0) {
        Serial.print(" no device found");
    }

    Serial.println();
}

bool initMPU6050() {
    int who = readMPURegister(REG_WHO_AM_I);

    if (who < 0) {
        Serial.println("MPU6050 is not responding on the I2C bus.");
        scanI2CBus();
        return false;
    }

    Serial.printf("MPU6050 WHO_AM_I = 0x%02X\n", who);

    // 0x68 = MPU6050，0x70/0x71 = MPU6500/MPU9250 兼容芯片，0x34 = 部分克隆
    if (who != 0x68 && who != 0x70 && who != 0x71 && who != 0x34) {
        Serial.println("Unexpected WHO_AM_I, continuing anyway.");
    }

    // 软复位，复位后芯片处于 sleep 状态
    if (!writeMPURegister(REG_PWR_MGMT_1, 0x80)) {
        Serial.println("MPU6050 reset failed.");
        return false;
    }

    delay(100);

    // 唤醒，时钟源选 PLL(X 轴陀螺仪)，比内部 RC 振荡器稳定
    if (!writeMPURegister(REG_PWR_MGMT_1, 0x01)) {
        Serial.println("MPU6050 wake-up failed.");
        return false;
    }

    if (!writeMPURegister(REG_CONFIG, DLPF_CFG)) {
        Serial.println("MPU6050 DLPF config failed.");
        return false;
    }

    if (!writeMPURegister(REG_SMPLRT_DIV, SAMPLE_RATE_DIV)) {
        Serial.println("MPU6050 sample rate config failed.");
        return false;
    }

    // 显式指定量程：±250 °/s、±2 g，与 GYRO_SCALE / ACC_SCALE 对应
    if (!writeMPURegister(REG_GYRO_CONFIG, 0x00) ||
        !writeMPURegister(REG_ACCEL_CONFIG, 0x00)) {
        Serial.println("MPU6050 range config failed.");
        return false;
    }

    // 等待 DLPF 输出稳定
    delay(50);

    return true;
}

bool readMPU6050(int16_t &ax, int16_t &ay, int16_t &az,
                 int16_t &gx, int16_t &gy, int16_t &gz) {

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_ACCEL_XOUT_H);

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

    if (!initMPU6050()) {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("MPU6050 ERROR");
        display.println();
        display.println("No I2C response");
        display.println("Check SDA(GPIO6)");
        display.println("SCL(GPIO7) / 3V3");
        display.println("See serial log.");
        display.display();

        while (true) {
            delay(1000);
        }
    }

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

