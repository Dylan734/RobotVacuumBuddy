
#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <SparkFunLSM6DS3.h>
#include <math.h>

// ============================================================
// ESP32-C3 SUPER MINI
// SparkFun LSM6DS3 + SH1106 OLED
// ============================================================

// Hardware configuration
#define I2C_SDA 6
#define I2C_SCL 7
#define IMU_ADDRESS 0x6B

// Sampling configuration
#define HISTORY_DURATION 10000UL
#define SAMPLE_INTERVAL 50UL

// Serial debugging
// 0 = Completely disable Serial output
// 1 = Enable Serial output
#define SERIAL_DEBUG 0

// ============================================================
// OLED
// ============================================================

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(
    U8G2_R0,
    U8X8_PIN_NONE
);

// ============================================================
// IMU
// ============================================================

LSM6DS3 imu(I2C_MODE, IMU_ADDRESS);

// ============================================================
// Maximum acceleration history
// ============================================================

struct AccelSample
{
    unsigned long timestamp;
    float x;
    float y;
    float z;
};

const int MAX_SAMPLES = 220;

AccelSample samples[MAX_SAMPLES];

int sampleIndex = 0;
int sampleCount = 0;

unsigned long lastSampleMillis = 0;

// Rolling maximum values
float maxX = 0.0f;
float maxY = 0.0f;
float maxZ = 0.0f;

// ============================================================
// Update maximum values over the last 10 seconds
// ============================================================

void updateMaxValues()
{
    unsigned long now = millis();

    maxX = 0.0f;
    maxY = 0.0f;
    maxZ = 0.0f;

    for (int i = 0; i < sampleCount; i++)
    {
        int index = (sampleIndex - sampleCount + i + MAX_SAMPLES)
                    % MAX_SAMPLES;

        if (now - samples[index].timestamp <= HISTORY_DURATION)
        {
            float absX = fabsf(samples[index].x);
            float absY = fabsf(samples[index].y);
            float absZ = fabsf(samples[index].z);

            if (absX > maxX) maxX = absX;
            if (absY > maxY) maxY = absY;
            if (absZ > maxZ) maxZ = absZ;
        }
    }
}

// ============================================================
// Store acceleration sample
// ============================================================

void storeSample(float x, float y, float z)
{
    samples[sampleIndex].timestamp = millis();

    samples[sampleIndex].x = x;
    samples[sampleIndex].y = y;
    samples[sampleIndex].z = z;

    sampleIndex = (sampleIndex + 1) % MAX_SAMPLES;

    if (sampleCount < MAX_SAMPLES)
    {
        sampleCount++;
    }
}

// ============================================================
// OLED Display
// ============================================================

void drawScreen(float x, float y, float z)
{
    char buffer[24];

    u8g2.clearBuffer();

    // Header
    u8g2.setFont(u8g2_font_profont12_tr);
    u8g2.drawStr(0, 11, "LSM6DS3 Monitor");

    // Header separator
    u8g2.drawHLine(0, 15, 128);

    // Vertical separator
    u8g2.drawVLine(78, 18, 46);

    // Left: Live readings
    u8g2.setFont(u8g2_font_5x8_tr);
    u8g2.drawStr(3, 25, "LIVE");

    u8g2.setFont(u8g2_font_6x12_tr);

    snprintf(buffer, sizeof(buffer), "X:%+.4f", x);
    u8g2.drawStr(3, 36, buffer);

    snprintf(buffer, sizeof(buffer), "Y:%+.4f", y);
    u8g2.drawStr(3, 48, buffer);

    snprintf(buffer, sizeof(buffer), "Z:%+.4f", z);
    u8g2.drawStr(3, 60, buffer);

    // Right: Maximum absolute readings
    u8g2.setFont(u8g2_font_5x8_tr);
    u8g2.drawStr(84, 25, "MAX 10s");

    u8g2.setFont(u8g2_font_6x12_tr);

    snprintf(buffer, sizeof(buffer), "X:%.4f", maxX);
    u8g2.drawStr(80, 36, buffer);

    snprintf(buffer, sizeof(buffer), "Y:%.4f", maxY);
    u8g2.drawStr(80, 48, buffer);

    snprintf(buffer, sizeof(buffer), "Z:%.4f", maxZ);
    u8g2.drawStr(80, 60, buffer);

    u8g2.sendBuffer();
}

// ============================================================
// Setup
// ============================================================

void setup()
{
#if SERIAL_DEBUG
    Serial.begin(115200);
    Serial.println();
    Serial.println("ESP32-C3 LSM6DS3 Monitor");
#endif

    // Initialize I2C
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000);

    // Initialize OLED
    u8g2.begin();
    u8g2.setFlipMode(1);

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tr);
    u8g2.drawStr(0, 25, "Initializing IMU...");
    u8g2.sendBuffer();

    // Initialize IMU
    if (imu.begin() != 0)
    {
#if SERIAL_DEBUG
        Serial.println("IMU initialization FAILED!");
#endif

        u8g2.clearBuffer();
        u8g2.drawStr(0, 25, "IMU FAILED!");
        u8g2.sendBuffer();

        while (1)
        {
            delay(100);
        }
    }

#if SERIAL_DEBUG
    Serial.println("IMU initialized successfully!");
    Serial.println("Rolling maximum: 10 seconds");
#endif
}

// ============================================================
// Main Loop
// ============================================================

void loop()
{
    unsigned long now = millis();

    if (now - lastSampleMillis >= SAMPLE_INTERVAL)
    {
        lastSampleMillis = now;

        // Read acceleration
        float x = imu.readFloatAccelX();
        float y = imu.readFloatAccelY();
        float z = imu.readFloatAccelZ();

        // Store new sample
        storeSample(x, y, z);

        // Calculate rolling maximum
        updateMaxValues();

        // Serial output (disabled by default)
#if SERIAL_DEBUG
        Serial.print(x, 4);
        Serial.print("\t");
        Serial.print(y, 4);
        Serial.print("\t");
        Serial.println(z, 4);
#endif

        // Update OLED
        drawScreen(x, y, z);
    }
}
