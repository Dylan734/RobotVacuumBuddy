
#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <SparkFunLSM6DS3.h>
#include <math.h>

// ============================================================
// ESP32-C3 SUPER MINI
// ROBOT VACUUM DESK BUDDY
//
// Hardware:
// - SH1106 128x64 OLED
// - SparkFun LSM6DS3 IMU
//
// I2C:
// SDA = GPIO6
// SCL = GPIO7
// OLED = 0x3C
// IMU  = 0x6B
// ============================================================


// ============================================================
// CONFIGURATION
// ============================================================

#define I2C_SDA 6
#define I2C_SCL 7

#define IMU_ADDRESS 0x6B

#define SERIAL_DEBUG 0

// IMU sampling
#define SAMPLE_INTERVAL 20UL

// OLED animation
#define FRAME_INTERVAL 33UL

// Turning detection (degrees per second)
#define TURN_THRESHOLD 25.0f
#define TURN_RELEASE_THRESHOLD 12.0f

// Bump detection (g)
#define BUMP_THRESHOLD_G 0.35f

// Pickup detection
#define PICKUP_TILT_THRESHOLD 40.0f
#define PICKUP_RELEASE_THRESHOLD 20.0f

// Timing
#define BUMP_COOLDOWN 1200UL
#define BUMP_DURATION 900UL

#define PICKUP_CONFIRM_TIME 300UL
#define PICKUP_RELEASE_TIME 800UL

#define CALIBRATION_DURATION 2000UL


// ============================================================
// HARDWARE
// ============================================================

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(
    U8G2_R0,
    U8X8_PIN_NONE
);

LSM6DS3 imu(I2C_MODE, IMU_ADDRESS);


// ============================================================
// FACE STATES
// ============================================================

enum FaceMood
{
    MOOD_IDLE,
    MOOD_TURN_LEFT,
    MOOD_TURN_RIGHT,
    MOOD_BUMP,
    MOOD_PICKED_UP
};

FaceMood mood = MOOD_IDLE;


// ============================================================
// TIMERS
// ============================================================

unsigned long lastSampleMillis = 0;
unsigned long lastFrameMillis = 0;

unsigned long lastBumpMillis = 0;

unsigned long moodUntil = 0;

unsigned long tiltStartMillis = 0;
unsigned long pickupReleaseMillis = 0;

unsigned long turnReleaseMillis = 0;

unsigned long nextBlinkMillis = 0;
unsigned long blinkUntil = 0;


// ============================================================
// IMU VALUES
// ============================================================

float accelX = 0;
float accelY = 0;
float accelZ = 0;

float gyroX = 0;
float gyroY = 0;
float gyroZ = 0;

float accelMagnitude = 0;
float linearMagnitude = 0;


// ============================================================
// GRAVITY REFERENCE
// ============================================================

// Low-pass filtered gravity estimate.
// Used to separate sudden acceleration from gravity.

float gravityX = 0;
float gravityY = 0;
float gravityZ = 0;

bool gravityInitialized = false;

// Initial mounting orientation
float referenceX = 0;
float referenceY = 0;
float referenceZ = 0;

float calibrationSumX = 0;
float calibrationSumY = 0;
float calibrationSumZ = 0;

int calibrationSamples = 0;

bool calibrated = false;

unsigned long calibrationStartMillis = 0;


// ============================================================
// STATE FLAGS
// ============================================================

bool pickedUp = false;
bool turning = false;

int turnDirection = 0;


// ============================================================
// HELPER FUNCTIONS
// ============================================================

float vectorMagnitude(float x, float y, float z)
{
    return sqrtf(x * x + y * y + z * z);
}


float calculateTilt()
{
    float referenceMagnitude = vectorMagnitude(
        referenceX,
        referenceY,
        referenceZ
    );

    float currentMagnitude = accelMagnitude;

    if (referenceMagnitude < 0.1f ||
        currentMagnitude < 0.1f)
    {
        return 0;
    }

    float dot =
        referenceX * accelX +
        referenceY * accelY +
        referenceZ * accelZ;

    float cosine = dot /
        (referenceMagnitude * currentMagnitude);

    cosine = constrain(cosine, -1.0f, 1.0f);

    return acosf(cosine) * 180.0f / PI;
}


// ============================================================
// CALIBRATION
// ============================================================

void calibrateIMU()
{
    if (calibrated)
        return;

    calibrationSumX += accelX;
    calibrationSumY += accelY;
    calibrationSumZ += accelZ;

    calibrationSamples++;

    if (millis() - calibrationStartMillis >=
        CALIBRATION_DURATION)
    {
        referenceX =
            calibrationSumX / calibrationSamples;

        referenceY =
            calibrationSumY / calibrationSamples;

        referenceZ =
            calibrationSumZ / calibrationSamples;

        calibrated = true;

        #if SERIAL_DEBUG
        Serial.println("IMU calibration complete");
        #endif
    }
}


// ============================================================
// IMU SAMPLING
// ============================================================

void readIMU()
{
    accelX = imu.readFloatAccelX();
    accelY = imu.readFloatAccelY();
    accelZ = imu.readFloatAccelZ();

    gyroX = imu.readFloatGyroX();
    gyroY = imu.readFloatGyroY();
    gyroZ = imu.readFloatGyroZ();

    accelMagnitude = vectorMagnitude(
        accelX,
        accelY,
        accelZ
    );

    // Initialise gravity estimate
    if (!gravityInitialized)
    {
        gravityX = accelX;
        gravityY = accelY;
        gravityZ = accelZ;

        gravityInitialized = true;
    }

    // Low-pass filter to estimate gravity
    const float alpha = 0.08f;

    gravityX += alpha * (accelX - gravityX);
    gravityY += alpha * (accelY - gravityY);
    gravityZ += alpha * (accelZ - gravityZ);

    // Remove gravity to estimate linear acceleration
    float linearX = accelX - gravityX;
    float linearY = accelY - gravityY;
    float linearZ = accelZ - gravityZ;

    linearMagnitude = vectorMagnitude(
        linearX,
        linearY,
        linearZ
    );

    if (!calibrated)
        calibrateIMU();
}


// ============================================================
// PICKUP DETECTION
// ============================================================

void detectPickup()
{
    if (!calibrated)
        return;

    unsigned long now = millis();

    float tilt = calculateTilt();

    bool freefall =
        accelMagnitude < 0.55f;

    bool tilted =
        tilt > PICKUP_TILT_THRESHOLD;

    if (!pickedUp)
    {
        if (tilted || freefall)
        {
            if (tiltStartMillis == 0)
                tiltStartMillis = now;

            if (now - tiltStartMillis >=
                PICKUP_CONFIRM_TIME)
            {
                pickedUp = true;

                mood = MOOD_PICKED_UP;

                tiltStartMillis = 0;
            }
        }
        else
        {
            tiltStartMillis = 0;
        }
    }
    else
    {
        // Release only after the robot returns
        // close to its original orientation.

        if (tilt < PICKUP_RELEASE_THRESHOLD &&
            accelMagnitude > 0.85f &&
            accelMagnitude < 1.15f)
        {
            if (pickupReleaseMillis == 0)
                pickupReleaseMillis = now;

            if (now - pickupReleaseMillis >=
                PICKUP_RELEASE_TIME)
            {
                pickedUp = false;

                pickupReleaseMillis = 0;

                mood = MOOD_IDLE;
            }
        }
        else
        {
            pickupReleaseMillis = 0;
        }
    }
}


// ============================================================
// BUMP DETECTION
// ============================================================

void detectBump()
{
    unsigned long now = millis();

    if (pickedUp)
        return;

    if (linearMagnitude > BUMP_THRESHOLD_G)
    {
        if (now - lastBumpMillis >= BUMP_COOLDOWN)
        {
            lastBumpMillis = now;

            mood = MOOD_BUMP;

            moodUntil = now + BUMP_DURATION;
        }
    }
}


// ============================================================
// TURN DETECTION
// ============================================================

void detectTurning()
{
    if (pickedUp)
        return;

    unsigned long now = millis();

    if (fabsf(gyroZ) > TURN_THRESHOLD)
    {
        turning = true;

        turnDirection = (gyroZ > 0) ? 1 : -1;

        turnReleaseMillis = 0;

        // Don't override a bump animation
        if (mood != MOOD_BUMP ||
            now >= moodUntil)
        {
            mood = (turnDirection > 0)
                ? MOOD_TURN_RIGHT
                : MOOD_TURN_LEFT;
        }
    }
    else if (turning)
    {
        if (fabsf(gyroZ) < TURN_RELEASE_THRESHOLD)
        {
            if (turnReleaseMillis == 0)
                turnReleaseMillis = now;

            if (now - turnReleaseMillis > 200)
            {
                turning = false;
                turnReleaseMillis = 0;

                if (mood == MOOD_TURN_LEFT ||
                    mood == MOOD_TURN_RIGHT)
                {
                    mood = MOOD_IDLE;
                }
            }
        }
        else
        {
            turnReleaseMillis = 0;
        }
    }
}


// ============================================================
// UPDATE SENSOR EVENTS
// ============================================================

void updateSensors()
{
    readIMU();

    detectPickup();

    if (pickedUp)
        return;

    detectBump();

    detectTurning();
}


// ============================================================
// BLINK ANIMATION
// ============================================================

void updateBlink()
{
    unsigned long now = millis();

    if (now >= nextBlinkMillis &&
        blinkUntil == 0)
    {
        blinkUntil = now + 340;

        nextBlinkMillis = now + random(6000,12000);
    }

    if (blinkUntil != 0 && now >= blinkUntil)
    {
        blinkUntil = 0;
    }
}


// ============================================================
// DRAW FACE
// ============================================================

void drawEyes(
    int leftX,
    int rightX,
    int eyeY,
    int eyeWidth,
    int eyeHeight
)
{
    u8g2.drawRBox(
        leftX,
        eyeY,
        eyeWidth,
        eyeHeight,
        5
    );

    u8g2.drawRBox(
        rightX,
        eyeY,
        eyeWidth,
        eyeHeight,
        5
    );
}


void drawSmile()
{
    u8g2.drawLine(59, 51, 64, 54);
    u8g2.drawLine(64, 54, 69, 51);
}


void drawSurprisedMouth()
{
    u8g2.drawCircle(64, 52, 4);
}

void drawClosedEyes()
{
    // Left closed eye
    u8g2.drawLine(34, 31, 40, 27);
    u8g2.drawLine(40, 27, 46, 31);

    // Right closed eye
    u8g2.drawLine(72, 31, 78, 27);
    u8g2.drawLine(78, 27, 84, 31);
}

void drawFace()
{
    unsigned long now = millis();

    u8g2.clearBuffer();

    // --------------------------------------------------------
    // Default face positioning
    // --------------------------------------------------------

    int leftX = 34;
    int rightX = 72;

    int eyeY = 17;

    int eyeWidth = 22;
    int eyeHeight = 26;

    bool blink =
        (blinkUntil != 0 && now < blinkUntil);

    // --------------------------------------------------------
    // Normal expression
    // --------------------------------------------------------

    if (mood == MOOD_IDLE)
    {
        if (blink)
        {
            drawClosedEyes();
        }
        else
        {
            drawEyes(
                leftX,
                rightX,
                eyeY,
                eyeWidth,
                eyeHeight
            );
        }

        drawSmile();
    }

    // --------------------------------------------------------
    // Turning left
    // --------------------------------------------------------

    else if (mood == MOOD_TURN_LEFT)
    {
        drawEyes(
            leftX - 5,
            rightX - 5,
            eyeY,
            eyeWidth,
            eyeHeight
        );

        drawSmile();
    }

    // --------------------------------------------------------
    // Turning right
    // --------------------------------------------------------

    else if (mood == MOOD_TURN_RIGHT)
    {
        drawEyes(
            leftX + 5,
            rightX + 5,
            eyeY,
            eyeWidth,
            eyeHeight
        );

        drawSmile();
    }

    // --------------------------------------------------------
    // Bump / startled
    // --------------------------------------------------------

    else if (mood == MOOD_BUMP)
    {
        drawEyes(
            31,
            75,
            13,
            25,
            30
        );

        drawSurprisedMouth();
    }

    // --------------------------------------------------------
    // Picked up
    // --------------------------------------------------------

    else if (mood == MOOD_PICKED_UP)
    {
        drawEyes(
            34,
            72,
            13,
            22,
            25
        );

        // Happy eyebrows
        u8g2.drawLine(35, 9, 46, 6);
        u8g2.drawLine(82, 6, 93, 9);

        drawSmile();
    }

    u8g2.sendBuffer();
}


// ============================================================
// OLED CALIBRATION SCREEN
// ============================================================

void drawCalibrationScreen()
{
    u8g2.clearBuffer();

    u8g2.setFont(u8g2_font_6x12_tr);

    u8g2.drawStr(17, 23, "Waking up...");

    u8g2.drawRFrame(15, 34, 98, 9, 3);

    int progress = map(
        millis() - calibrationStartMillis,
        0,
        CALIBRATION_DURATION,
        0,
        94
    );

    progress = constrain(progress, 0, 94);

    u8g2.drawBox(17, 36, progress, 5);

    u8g2.sendBuffer();
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    #if SERIAL_DEBUG
    Serial.begin(115200);
    #endif

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000);

    u8g2.begin();
    u8g2.setFlipMode(1);

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tr);
    u8g2.drawStr(20, 30, "Hello there!");
    u8g2.sendBuffer();

    delay(700);

    // Initialise IMU
    if (imu.begin() != 0)
    {
        u8g2.clearBuffer();
        u8g2.drawStr(20, 30, "IMU ERROR!");
        u8g2.sendBuffer();

        while (1)
        {
            delay(100);
        }
    }

    calibrationStartMillis = millis();

    nextBlinkMillis = millis() + 2500;
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
    unsigned long now = millis();

    // --------------------------------------------------------
    // IMU sampling
    // --------------------------------------------------------

    if (now - lastSampleMillis >= SAMPLE_INTERVAL)
    {
        lastSampleMillis = now;

        updateSensors();

        #if SERIAL_DEBUG
        if (Serial.availableForWrite() >= 32)
        {
            Serial.print(accelX, 3);
            Serial.print("\t");

            Serial.print(accelY, 3);
            Serial.print("\t");

            Serial.print(accelZ, 3);
            Serial.print("\t");

            Serial.println(gyroZ, 2);
        }
        #endif
    }

    // --------------------------------------------------------
    // Display calibration screen
    // --------------------------------------------------------

    if (!calibrated)
    {
        if (now - lastFrameMillis >= FRAME_INTERVAL)
        {
            lastFrameMillis = now;

            drawCalibrationScreen();
        }

        return;
    }

    // --------------------------------------------------------
    // Return to idle after bump animation
    // --------------------------------------------------------

    if (mood == MOOD_BUMP &&
        now >= moodUntil)
    {
        mood = turning
            ? ((turnDirection > 0)
                ? MOOD_TURN_RIGHT
                : MOOD_TURN_LEFT)
            : MOOD_IDLE;
    }

    // --------------------------------------------------------
    // Update animation
    // --------------------------------------------------------

    updateBlink();

    if (now - lastFrameMillis >= FRAME_INTERVAL)
    {
        lastFrameMillis = now;

        drawFace();
    }
}
