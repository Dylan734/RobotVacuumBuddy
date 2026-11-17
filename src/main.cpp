
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
//
// BOOT BUTTON:
// GPIO9, active LOW
//
// BUTTON:
// Short press = Next expression
// Hold 3 seconds = Automatic expression demo
// 8 seconds without another short press = Return to normal
// ============================================================


// ============================================================
// CONFIGURATION
// ============================================================

#define I2C_SDA 6
#define I2C_SCL 7

#define IMU_ADDRESS 0x6B

#define SERIAL_DEBUG 0

// BOOT button
#define BOOT_BUTTON_PIN 9
#define BUTTON_DEBOUNCE_TIME 35UL
#define LONG_PRESS_TIME 3000UL

// Expression demo
#define DEMO_EXPRESSION_DURATION 1500UL
#define MANUAL_PREVIEW_TIMEOUT 8000UL

// IMU sampling
#define SAMPLE_INTERVAL 20UL
#define SLEEP_SAMPLE_INTERVAL 100UL

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
#define BUMP_DURATION 1800UL

#define PICKUP_CONFIRM_TIME 300UL
#define PICKUP_RELEASE_TIME 800UL

#define CALIBRATION_DURATION 2000UL

// Inactivity
#define SLEEPY_TIMEOUT 60000UL
#define INACTIVITY_TIMEOUT 180000UL

// Sleep animation before OLED turns off
#define SLEEP_TRANSITION_DURATION 1800UL

// Movement detection thresholds
#define MOVEMENT_ACCEL_THRESHOLD 0.10f
#define MOVEMENT_GYRO_THRESHOLD 8.0f

// Other expression timings
#define DIZZY_DURATION 2500UL
#define ANNOYED_DURATION 1800UL
#define CURIOUS_DURATION 2000UL

// Repeated bump detection
#define BUMP_WINDOW 8000UL
#define REPEATED_BUMP_COUNT 3


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
    MOOD_PICKED_UP,
    MOOD_DIZZY,
    MOOD_SLEEPY,
    MOOD_SLEEPING,
    MOOD_ANNOYED,
    MOOD_CURIOUS
};

FaceMood mood = MOOD_IDLE;


// ============================================================
// EXPRESSION DEMO STATES
// ============================================================

enum DemoMode
{
    DEMO_NONE,
    DEMO_MANUAL,
    DEMO_AUTOMATIC
};

DemoMode demoMode = DEMO_NONE;

const FaceMood demoExpressions[] =
{
    MOOD_IDLE,
    MOOD_TURN_LEFT,
    MOOD_TURN_RIGHT,
    MOOD_BUMP,
    MOOD_PICKED_UP,
    MOOD_DIZZY,
    MOOD_SLEEPY,
    MOOD_SLEEPING,
    MOOD_ANNOYED,
    MOOD_CURIOUS
};

const int DEMO_EXPRESSION_COUNT =
    sizeof(demoExpressions) / sizeof(demoExpressions[0]);

int demoIndex = 0;

unsigned long demoStepMillis = 0;
unsigned long lastManualPressMillis = 0;


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

unsigned long lastMovementMillis = 0;

unsigned long sleepTransitionStart = 0;

unsigned long bumpWindowStart = 0;
unsigned long nextCuriousMillis = 0;


// ============================================================
// BOOT BUTTON STATE
// ============================================================

bool buttonRawState = HIGH;
bool buttonStableState = HIGH;

unsigned long buttonDebounceMillis = 0;
unsigned long buttonPressStartMillis = 0;

bool longPressTriggered = false;


// ============================================================
// POWER SAVING STATE
// ============================================================

bool displaySleeping = false;
bool sleepTransition = false;


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
float gyroMagnitude = 0;


// ============================================================
// GRAVITY REFERENCE
// ============================================================

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

int recentBumpCount = 0;


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

        lastMovementMillis = millis();

        nextCuriousMillis =
            millis() + random(15000, 30000);

        #if SERIAL_DEBUG
        Serial.println("IMU calibration complete");
        #endif
    }
}


// ============================================================
// DISPLAY POWER MANAGEMENT
// ============================================================

void enterPowerSaving()
{
    if (displaySleeping)
        return;

    u8g2.setPowerSave(1);

    displaySleeping = true;
}


void wakeDisplay()
{
    if (displaySleeping)
    {
        u8g2.setPowerSave(0);
        displaySleeping = false;
    }

    sleepTransition = false;

    lastFrameMillis = 0;

    blinkUntil = 0;
    nextBlinkMillis = millis() + 2500;

    lastMovementMillis = millis();

    mood = MOOD_IDLE;
}


// ============================================================
// MOVEMENT DETECTION
// ============================================================

void detectMovement()
{
    if (!calibrated)
        return;

    bool accelerationDetected =
        linearMagnitude > MOVEMENT_ACCEL_THRESHOLD;

    bool rotationDetected =
        gyroMagnitude > MOVEMENT_GYRO_THRESHOLD;

    if (accelerationDetected || rotationDetected)
    {
        lastMovementMillis = millis();

        if (displaySleeping || sleepTransition)
        {
            wakeDisplay();
        }

        if (mood == MOOD_SLEEPY)
        {
            mood = MOOD_IDLE;
        }
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

    gyroMagnitude = vectorMagnitude(
        gyroX,
        gyroY,
        gyroZ
    );

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

                wakeDisplay();

                mood = MOOD_PICKED_UP;

                tiltStartMillis = 0;
                moodUntil = 0;
            }
        }
        else
        {
            tiltStartMillis = 0;
        }
    }
    else
    {
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

                lastMovementMillis = now;

                mood = MOOD_DIZZY;
                moodUntil = now + DIZZY_DURATION;
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

            wakeDisplay();

            // Count bumps within a time window
            if (bumpWindowStart == 0 ||
                now - bumpWindowStart > BUMP_WINDOW)
            {
                bumpWindowStart = now;
                recentBumpCount = 0;
            }

            recentBumpCount++;

            if (recentBumpCount >= REPEATED_BUMP_COUNT)
            {
                mood = MOOD_ANNOYED;
                moodUntil = now + ANNOYED_DURATION;

                recentBumpCount = 0;
                bumpWindowStart = 0;
            }
            else
            {
                mood = MOOD_BUMP;
                moodUntil = now + BUMP_DURATION;
            }
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

        wakeDisplay();

        // Don't override higher-priority expressions
        if (mood == MOOD_IDLE ||
            mood == MOOD_TURN_LEFT ||
            mood == MOOD_TURN_RIGHT)
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

    detectMovement();

    // Do not allow sensor events to interrupt demo
    if (demoMode != DEMO_NONE)
        return;

    detectPickup();

    if (pickedUp)
        return;

    detectBump();

    detectTurning();
}


// ============================================================
// POWER SAVING / INACTIVITY EXPRESSIONS
// ============================================================

void updatePowerSaving()
{
    if (!calibrated)
        return;

    // Do not enter sleep during expression demo
    if (demoMode != DEMO_NONE)
        return;

    if (pickedUp || turning)
        return;

    unsigned long now = millis();

    unsigned long inactiveTime =
        now - lastMovementMillis;

    // --------------------------------------------------------
    // Sleep transition
    // --------------------------------------------------------

    if (inactiveTime >= INACTIVITY_TIMEOUT)
    {
        if (!sleepTransition && !displaySleeping)
        {
            sleepTransition = true;
            sleepTransitionStart = now;

            mood = MOOD_SLEEPING;

            lastFrameMillis = 0;
        }

        if (sleepTransition &&
            now - sleepTransitionStart >=
            SLEEP_TRANSITION_DURATION)
        {
            enterPowerSaving();
        }

        return;
    }

    // --------------------------------------------------------
    // Sleepy expression
    // --------------------------------------------------------

    if (!displaySleeping &&
        !sleepTransition &&
        inactiveTime >= SLEEPY_TIMEOUT)
    {
        if (mood == MOOD_IDLE)
        {
            mood = MOOD_SLEEPY;
        }
    }
}


// ============================================================
// UPDATE TEMPORARY EXPRESSIONS
// ============================================================

void updateTimedExpressions()
{
    if (demoMode != DEMO_NONE)
        return;

    unsigned long now = millis();

    // End temporary expressions
    if ((mood == MOOD_BUMP ||
         mood == MOOD_ANNOYED ||
         mood == MOOD_DIZZY ||
         mood == MOOD_CURIOUS) &&
        now >= moodUntil)
    {
        if (turning)
        {
            mood = (turnDirection > 0)
                ? MOOD_TURN_RIGHT
                : MOOD_TURN_LEFT;
        }
        else
        {
            mood = MOOD_IDLE;
        }
    }

    // Don't introduce random expressions
    // while another event is active.
    if (mood != MOOD_IDLE || turning || pickedUp)
        return;

    // --------------------------------------------------------
    // Random curious expression
    // --------------------------------------------------------

    if (now >= nextCuriousMillis)
    {
        mood = MOOD_CURIOUS;

        moodUntil = now + CURIOUS_DURATION;

        nextCuriousMillis =
            now + random(20000, 40000);
    }
}


// ============================================================
// EXPRESSION DEMO CONTROL
// ============================================================

void endExpressionDemo()
{
    demoMode = DEMO_NONE;

    demoIndex = 0;

    mood = MOOD_IDLE;

    lastMovementMillis = millis();

    blinkUntil = 0;
    nextBlinkMillis = millis() + 2500;

    moodUntil = 0;

    sleepTransition = false;

    lastFrameMillis = 0;
}


void startAutomaticDemo()
{
    wakeDisplay();

    demoMode = DEMO_AUTOMATIC;

    demoIndex = 0;

    mood = demoExpressions[demoIndex];

    demoStepMillis = millis();

    sleepTransition = false;

    lastMovementMillis = millis();

    lastFrameMillis = 0;
}


void advanceManualDemo()
{
    wakeDisplay();

    if (demoMode != DEMO_MANUAL)
    {
        demoMode = DEMO_MANUAL;

        demoIndex = 0;
    }
    else
    {
        demoIndex++;

        if (demoIndex >= DEMO_EXPRESSION_COUNT)
        {
            demoIndex = 0;
        }
    }

    mood = demoExpressions[demoIndex];

    lastManualPressMillis = millis();

    lastMovementMillis = millis();

    sleepTransition = false;

    lastFrameMillis = 0;
}


void updateExpressionDemo()
{
    unsigned long now = millis();

    // --------------------------------------------------------
    // Manual preview timeout
    // --------------------------------------------------------

    if (demoMode == DEMO_MANUAL)
    {
        if (now - lastManualPressMillis >=
            MANUAL_PREVIEW_TIMEOUT)
        {
            endExpressionDemo();
        }

        return;
    }

    // --------------------------------------------------------
    // Automatic demonstration
    // --------------------------------------------------------

    if (demoMode == DEMO_AUTOMATIC)
    {
        if (now - demoStepMillis >=
            DEMO_EXPRESSION_DURATION)
        {
            demoStepMillis = now;

            demoIndex++;

            if (demoIndex >= DEMO_EXPRESSION_COUNT)
            {
                endExpressionDemo();
                return;
            }

            mood = demoExpressions[demoIndex];

            lastFrameMillis = 0;
        }
    }
}


// ============================================================
// BOOT BUTTON HANDLING
// ============================================================

void updateBootButton()
{
    unsigned long now = millis();

    bool reading = digitalRead(BOOT_BUTTON_PIN);

    // Detect raw state changes
    if (reading != buttonRawState)
    {
        buttonRawState = reading;

        buttonDebounceMillis = now;
    }

    // Apply debounce
    if (now - buttonDebounceMillis >=
        BUTTON_DEBOUNCE_TIME)
    {
        if (buttonStableState != buttonRawState)
        {
            buttonStableState = buttonRawState;

            // Button pressed
            if (buttonStableState == LOW)
            {
                buttonPressStartMillis = now;

                longPressTriggered = false;
            }

            // Button released
            else
            {
                if (!longPressTriggered)
                {
                    advanceManualDemo();
                }
            }
        }
    }

    // Long press detection
    if (buttonStableState == LOW &&
        !longPressTriggered)
    {
        if (now - buttonPressStartMillis >=
            LONG_PRESS_TIME)
        {
            longPressTriggered = true;

            startAutomaticDemo();
        }
    }
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

        nextBlinkMillis =
            now + random(6000, 12000);
    }

    if (blinkUntil != 0 && now >= blinkUntil)
    {
        blinkUntil = 0;
    }
}


// ============================================================
// DRAW EYES
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


// ============================================================
// MOUTH EXPRESSIONS
// ============================================================

void drawSmile()
{
    u8g2.drawLine(59, 51, 64, 54);
    u8g2.drawLine(64, 54, 69, 51);
}


void drawBigSmile()
{
    u8g2.drawLine(55, 49, 60, 55);
    u8g2.drawLine(60, 55, 64, 57);
    u8g2.drawLine(64, 57, 68, 55);
    u8g2.drawLine(68, 55, 73, 49);
}


void drawFlatMouth()
{
    u8g2.drawLine(58, 53, 70, 53);
}


void drawSadMouth()
{
    u8g2.drawLine(58, 56, 64, 51);
    u8g2.drawLine(64, 51, 70, 56);
}


void drawSurprisedMouth()
{
    u8g2.drawDisc(64, 53, 7);

    u8g2.setDrawColor(0);
    u8g2.drawDisc(64, 53, 3);
    u8g2.setDrawColor(1);
}


// ============================================================
// CLOSED EYES
// ============================================================

void drawClosedEyes()
{
    u8g2.drawLine(34, 31, 40, 27);
    u8g2.drawLine(40, 27, 46, 31);

    u8g2.drawLine(72, 31, 78, 27);
    u8g2.drawLine(78, 27, 84, 31);
}


// ============================================================
// SLEEPY EYES
// ============================================================

void drawSleepyEyes()
{
    u8g2.drawRBox(34, 30, 22, 12, 5);
    u8g2.drawRBox(72, 30, 22, 12, 5);

    u8g2.drawLine(34, 30, 56, 30);
    u8g2.drawLine(72, 30, 94, 30);
}


// ============================================================
// SURPRISED EXPRESSION
// ============================================================

void drawSurprisedFace()
{
    drawEyes(
        33,
        71,
        12,
        24,
        32
    );

    u8g2.drawLine(34, 7, 45, 4);
    u8g2.drawLine(83, 4, 94, 7);

    drawSurprisedMouth();
}


// ============================================================
// EXCITED EXPRESSION
// ============================================================

void drawExcitedFace()
{
    drawEyes(
        33,
        71,
        12,
        24,
        29
    );

    u8g2.drawLine(35, 9, 46, 5);
    u8g2.drawLine(82, 5, 93, 9);

    drawBigSmile();

    u8g2.drawLine(21, 18, 17, 14);
    u8g2.drawLine(107, 18, 111, 14);
}


// ============================================================
// DIZZY EXPRESSION
// ============================================================

void drawDizzyFace()
{
    drawEyes(
        34,
        72,
        17,
        22,
        26
    );

    u8g2.setDrawColor(0);

    u8g2.drawLine(39, 24, 51, 36);
    u8g2.drawLine(51, 24, 39, 36);

    u8g2.drawLine(77, 24, 89, 36);
    u8g2.drawLine(89, 24, 77, 36);

    u8g2.setDrawColor(1);

    drawSurprisedMouth();

    unsigned long phase = millis() / 180;

    int offset = (phase % 2 == 0) ? 0 : 3;

    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(16 + offset, 15, "*");
    u8g2.drawStr(101 - offset, 15, "*");
}


// ============================================================
// ANNOYED EXPRESSION
// ============================================================

void drawAnnoyedFace()
{
    drawEyes(
        34,
        72,
        19,
        22,
        24
    );

    u8g2.drawLine(32, 12, 56, 20);
    u8g2.drawLine(72, 20, 96, 12);

    drawSadMouth();
}


// ============================================================
// CURIOUS EXPRESSION
// ============================================================

void drawCuriousFace()
{
    unsigned long phase = millis() / 350;

    int offset = (phase % 2 == 0) ? 3 : 0;

    drawEyes(
        34 + offset,
        72 + offset,
        17,
        22,
        26
    );

    u8g2.drawLine(34, 8, 48, 5);
    u8g2.drawLine(78, 7, 94, 12);

    drawFlatMouth();
}


// ============================================================
// SLEEPING EXPRESSION
// ============================================================

void drawSleepingFace()
{
    drawClosedEyes();

    drawFlatMouth();

    unsigned long phase = millis() / 350;

    int x = 88 - (phase % 3) * 4;
    int y = 25 - (phase % 3) * 8;

    u8g2.setFont(u8g2_font_6x10_tr);

    u8g2.drawStr(x, y, "Z");
    u8g2.drawStr(x + 10, y - 10, "Z");
}


// ============================================================
// DRAW FACE
// ============================================================

void drawFace()
{
    unsigned long now = millis();

    u8g2.clearBuffer();

    bool blink =
        (blinkUntil != 0 && now < blinkUntil);

    // Normal / happy
    if (mood == MOOD_IDLE)
    {
        if (blink)
        {
            drawClosedEyes();
        }
        else
        {
            drawEyes(
                34,
                72,
                17,
                22,
                26
            );
        }

        drawSmile();
    }

    // Turning left
    else if (mood == MOOD_TURN_LEFT)
    {
        drawEyes(
            29,
            67,
            17,
            22,
            26
        );

        drawSmile();
    }

    // Turning right
    else if (mood == MOOD_TURN_RIGHT)
    {
        drawEyes(
            39,
            77,
            17,
            22,
            26
        );

        drawSmile();
    }

    // Surprised / bump
    else if (mood == MOOD_BUMP)
    {
        drawSurprisedFace();
    }

    // Excited / picked up
    else if (mood == MOOD_PICKED_UP)
    {
        drawExcitedFace();
    }

    // Dizzy
    else if (mood == MOOD_DIZZY)
    {
        drawDizzyFace();
    }

    // Sleepy
    else if (mood == MOOD_SLEEPY)
    {
        drawSleepyEyes();

        drawFlatMouth();

        u8g2.setFont(u8g2_font_6x10_tr);
        u8g2.drawStr(96, 17, "z");
    }

    // Sleeping
    else if (mood == MOOD_SLEEPING)
    {
        drawSleepingFace();
    }

    // Annoyed
    else if (mood == MOOD_ANNOYED)
    {
        drawAnnoyedFace();
    }

    // Curious
    else if (mood == MOOD_CURIOUS)
    {
        drawCuriousFace();
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

    randomSeed(micros());

    pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);

    buttonRawState = digitalRead(BOOT_BUTTON_PIN);
    buttonStableState = buttonRawState;

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

    unsigned long currentSampleInterval =
        displaySleeping
        ? SLEEP_SAMPLE_INTERVAL
        : SAMPLE_INTERVAL;

    if (now - lastSampleMillis >= currentSampleInterval)
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
    // Calibration screen
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
    // BOOT button
    // --------------------------------------------------------

    updateBootButton();

    // --------------------------------------------------------
    // Expression demonstration
    // --------------------------------------------------------

    updateExpressionDemo();

    // --------------------------------------------------------
    // Power saving and inactivity
    // --------------------------------------------------------

    updatePowerSaving();

    if (displaySleeping)
        return;

    // --------------------------------------------------------
    // Temporary expression handling
    // --------------------------------------------------------

    updateTimedExpressions();

    // --------------------------------------------------------
    // Animation updates
    // --------------------------------------------------------

    if (mood == MOOD_IDLE)
    {
        updateBlink();
    }

    // --------------------------------------------------------
    // Render OLED
    // --------------------------------------------------------

    if (now - lastFrameMillis >= FRAME_INTERVAL)
    {
        lastFrameMillis = now;

        drawFace();
    }
}
