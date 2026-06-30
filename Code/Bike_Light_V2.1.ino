#include "esp_sleep.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// ═══════════════════════════════════════════════════════
//  TUNING 
// ═══════════════════════════════════════════════════════

const int   SMOOTH_SAMPLES  = 10;
const int   JERK_SAMPLES    = 10;
const float JERK_THRESHOLD  = -1.2;
const float ACCEL_THRESHOLD = -0.5;
const int   CONFIRM_MS      = 200;
const int   BRAKE_TIMEOUT   = 400;
const float MAX_ACCEL       = -9.0;
const int   BLINK_SLOW      = 750;
const int   BLINK_FAST      = 50;
const int   SAMPLE_MS       = 20;

// ═══════════════════════════════════════════════════════
//  BATTERY
// ═══════════════════════════════════════════════════════

const float BATTERY_EMPTY_V = 3.2;
const float BATTERY_FULL_V  = 4.2;

// ═══════════════════════════════════════════════════════
//  DAY / NIGHT MODE
// ═══════════════════════════════════════════════════════

const int LIGHT_SENSOR = 3;  // GPIO3 = D1 (TEPT4400 analog out)
float NIGHT_THRESHOLD_LOW  = 0.8;  // drop below this → enter night mode
float NIGHT_THRESHOLD_HIGH = 1.2;  // rise above this → return to day mode

const unsigned long LIGHT_CHECK_INTERVAL_MS = 100; 

int NIGHT_BRIGHTNESS_PERCENT = 40;  //dimming level at night

const unsigned long IDLE_FLASH_INTERVAL_MS = 5000; 
const unsigned long IDLE_FLASH_DURATION_MS = 100;  

const int PWM_FREQ       = 1000; // 1kHz 
const int PWM_RESOLUTION = 8;    // 8-bit = 256 brightness steps

// ═══════════════════════════════════════════════════════
//  PINS
// ═══════════════════════════════════════════════════════

const int TLC_LE    = 21;
const int TLC_OE    = 20;
const int ACCEL_SDA = 6;
const int ACCEL_SCL = 7;
const int HALL      = 5;

// ═══════════════════════════════════════════════════════
//  TLC5927 LED DRIVER — grid mapping
// ═══════════════════════════════════════════════════════

const uint16_t GRID[16] = {
  0x0000, 0x0400, 0x0010, 0x0008, 0x0200, 0x0800, 0x2000, 0x0100,
  0x1000, 0x4000, 0x0080, 0x0020, 0x0001, 0x0040, 0x0004, 0x0002,
};
const uint16_t ALL_ON = 0x7FFF;

uint16_t pos(int p) { return GRID[p]; }

// ─────────────────────────────────────────────
//  Brightness control — PWM on -OE
//
//  -OE is active LOW (LOW = outputs enabled).
//  So the PWM "duty" (HIGH time) must be the
//  INVERSE of the brightness we want:
//    100% bright → OE constantly LOW  → duty 0
//      0% bright → OE constantly HIGH → duty 255
// ─────────────────────────────────────────────
void tlcSetBrightness(int percent) {
  percent = constrain(percent, 0, 100);
  int duty = (int)round((100 - percent) / 100.0 * 255);
  ledcWrite(TLC_OE, duty);
}

void tlcBegin() {
  pinMode(TLC_LE, OUTPUT);
  digitalWrite(TLC_LE, LOW);


  ledcAttach(TLC_OE, PWM_FREQ, PWM_RESOLUTION);
  tlcSetBrightness(0);   

  SPI.begin();
  tlcSend(0x0000);
  tlcSetBrightness(100); // default full brightness; day/night logic adjusts
}

void tlcSend(uint16_t data) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(TLC_LE, LOW);
  SPI.transfer(data >> 8);
  SPI.transfer(data & 0xFF);
  SPI.endTransaction();
  digitalWrite(TLC_LE, HIGH);
  delayMicroseconds(1);
  digitalWrite(TLC_LE, LOW);
}

void tlcAllOn()  { tlcSend(ALL_ON); }
void tlcAllOff() { tlcSend(0x0000); }

uint16_t batteryRowMask(int row) {
  switch (row) {
    case 0: return pos(13) | pos(14) | pos(15);
    case 1: return pos(10) | pos(11) | pos(12);
    case 2: return pos(7)  | pos(8)  | pos(9);
    case 3: return pos(4)  | pos(5)  | pos(6);
    case 4: return pos(1)  | pos(2)  | pos(3);
    default: return 0;
  }
}

int batteryLevel(float voltage) {
  float lvl = mapFloat(voltage, BATTERY_EMPTY_V, BATTERY_FULL_V, 0, 5);
  return constrain((int)round(lvl), 0, 5);
}

void batteryGaugePattern(float voltage) {
  int level = batteryLevel(voltage);
  Serial.println("Battery level: " + String(level) + "/5 ("
                  + String(voltage, 2) + "V)");

  uint16_t pattern = 0;
  for (int row = 0; row < level; row++) {
    pattern |= batteryRowMask(row);
    tlcSend(pattern);
    delay(150);
  }

  if (level == 0) {
    tlcSend(batteryRowMask(0));
    delay(150);
    tlcAllOff();
    delay(150);
    tlcSend(batteryRowMask(0));
  }

  delay(1500);
  tlcAllOff();
}

// ═══════════════════════════════════════════════════════
//  BATTERY VOLTAGE
// ═══════════════════════════════════════════════════════

float readBatteryVoltage() {
  uint32_t total = 0;
  for (int i = 0; i < 16; i++) {
    total += analogReadMilliVolts(A0);
  }
  return 2.0f * total / 16 / 1000.0f;
}

// ═══════════════════════════════════════════════════════
//  AMBIENT LIGHT SENSOR 
// ═══════════════════════════════════════════════════════

float readLightLevel() {
  // average a handful of quick samples to smooth out flicker
  // (passing under power lines, dappled shade, etc.)
  uint32_t total = 0;
  const int samples = 5;
  for (int i = 0; i < samples; i++) {
    total += analogReadMilliVolts(LIGHT_SENSOR);
    delayMicroseconds(200);
  }
  return total / (float)samples / 1000.0f;  // mV → V
}

// ═══════════════════════════════════════════════════════
//  ACCELEROMETER
// ═══════════════════════════════════════════════════════

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(1);
RTC_DATA_ATTR int bootCount = 0;

float readAccelZ() {
  sensors_event_t event;
  accel.getEvent(&event);
  return event.acceleration.z;
}

// ═══════════════════════════════════════════════════════
//  ROLLING AVERAGE + LONG-WINDOW JERK
// ═══════════════════════════════════════════════════════

float smoothBuf[SMOOTH_SAMPLES];
float smoothSum  = 0;
int   smoothHead = 0;

void initSmooth(float v) {
  for (int i = 0; i < SMOOTH_SAMPLES; i++) smoothBuf[i] = v;
  smoothSum  = v * SMOOTH_SAMPLES;
  smoothHead = 0;
}

float addSmooth(float v) {
  smoothSum -= smoothBuf[smoothHead];
  smoothBuf[smoothHead] = v;
  smoothSum += v;
  smoothHead = (smoothHead + 1) % SMOOTH_SAMPLES;
  return smoothSum / SMOOTH_SAMPLES;
}

float jerkBuf[JERK_SAMPLES];
int   jerkHead = 0;

void initJerk(float v) {
  for (int i = 0; i < JERK_SAMPLES; i++) jerkBuf[i] = v;
  jerkHead = 0;
}

float computeJerk(float smoothed) {
  float oldest      = jerkBuf[jerkHead];
  jerkBuf[jerkHead] = smoothed;
  jerkHead          = (jerkHead + 1) % JERK_SAMPLES;
  return smoothed - oldest;
}

float mapFloat(float val, float inMin, float inMax,
               float outMin, float outMax) {
  return (val - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

// ═══════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("Alive");

  tlcBegin();
  Wire.begin(ACCEL_SDA, ACCEL_SCL);
  pinMode(HALL, INPUT_PULLUP);
  pinMode(A0, INPUT);
  pinMode(LIGHT_SENSOR, INPUT);

  bootCount++;
  Serial.println("Boot: " + String(bootCount));

  // ── Determine day/night before any LED pattern runs ──
  bool isNight = (readLightLevel() < NIGHT_THRESHOLD_LOW);
  tlcSetBrightness(isNight ? NIGHT_BRIGHTNESS_PERCENT : 100);
  Serial.println(isNight ? "Starting in NIGHT mode" : "Starting in DAY mode");

  // ── Battery check on wake ──
  float batteryV = readBatteryVoltage();
  batteryGaugePattern(batteryV);

  // ── ADXL343 ──
  if (!accel.begin()) {
    Serial.println("ADXL343 not found — check wiring");
    while (1);
  }
  accel.setRange(ADXL345_RANGE_4_G);
  accel.writeRegister(0x2D, 0x08);
  Serial.println("ADXL343 ready");

  float firstReading = readAccelZ();
  initSmooth(firstReading);
  initJerk(firstReading);

  // ── State variables ──
  bool          braking          = false;
  bool          jerkFlagged      = false;
  unsigned long confirmStart     = 0;
  bool          confirming       = false;
  unsigned long lastBrakeMs      = 0;
  int           blinkDelay       = BLINK_SLOW;
  bool          ledState         = false;
  unsigned long lastBlink        = 0;
  unsigned long lastSample       = 0;

  unsigned long lastLightCheck   = millis();
  unsigned long lastIdleFlash    = millis();
  bool          idleFlashOn      = false;
  unsigned long idleFlashStart   = 0;

  //Main loop 
  while (digitalRead(HALL) == LOW) {

    unsigned long now = millis();

    // ambient light check (with hysteresis)
    if (now - lastLightCheck >= LIGHT_CHECK_INTERVAL_MS) {
      lastLightCheck = now;
      float lightV  = readLightLevel();
      bool  wasNight = isNight;

      if (!isNight && lightV < NIGHT_THRESHOLD_LOW)  isNight = true;
      if ( isNight && lightV > NIGHT_THRESHOLD_HIGH) isNight = false;

      if (isNight != wasNight) {
        tlcSetBrightness(isNight ? NIGHT_BRIGHTNESS_PERCENT : 100);
        if (!isNight) { tlcAllOff(); idleFlashOn = false; }
        Serial.println(isNight ? ">>> NIGHT MODE" : ">>> DAY MODE");
      }
    }

    //LED control
    if (braking) {
      // Braking always takes priority over the idle flash
      if (now - lastBlink >= (unsigned long)blinkDelay) {
        ledState = !ledState;
        ledState ? tlcAllOn() : tlcAllOff();
        lastBlink = now;
      }
    } else {
      if (ledState) { ledState = false; tlcAllOff(); }

      // Night-only idle flash
      if (isNight) {
        if (!idleFlashOn && now - lastIdleFlash >= IDLE_FLASH_INTERVAL_MS) {
          tlcAllOn();
          idleFlashOn    = true;
          idleFlashStart = now;
          lastIdleFlash  = now;
        }
        if (idleFlashOn && now - idleFlashStart >= IDLE_FLASH_DURATION_MS) {
          tlcAllOff();
          idleFlashOn = false;
        }
      }
    }

    //Sample at fixed rate 
    if (now - lastSample < (unsigned long)SAMPLE_MS) continue;
    lastSample = now;

    float raw      = readAccelZ();
    float smoothed = addSmooth(raw);
    float jerk     = computeJerk(smoothed);

    if (jerk < JERK_THRESHOLD) jerkFlagged = true;

    if (jerkFlagged && smoothed < ACCEL_THRESHOLD) {
      if (!confirming) { confirming = true; confirmStart = now; }
      if (!braking && (now - confirmStart >= (unsigned long)CONFIRM_MS)) {
        braking = true;
        Serial.println(">>> BRAKING START");
      }
    } else {
      confirming  = false;
      jerkFlagged = false;
    }

    if (braking) {
      if (smoothed < ACCEL_THRESHOLD) {
        lastBrakeMs = now;
        float clamped = max(smoothed, MAX_ACCEL);
        blinkDelay = (int)mapFloat(clamped, ACCEL_THRESHOLD, MAX_ACCEL,
                                   BLINK_SLOW, BLINK_FAST);
        blinkDelay = constrain(blinkDelay, BLINK_FAST, BLINK_SLOW);
      }
      if (now - lastBrakeMs > (unsigned long)BRAKE_TIMEOUT) {
        braking = false; jerkFlagged = false; confirming = false;
        Serial.println("<<< BRAKING END");
      }
    }

    Serial.print("Z:"); Serial.print(raw, 2);
    Serial.print(" sm:"); Serial.print(smoothed, 2);
    Serial.print(" jk:"); Serial.print(jerk, 2);
    Serial.print(" B:"); Serial.println(braking ? "YES" : "no");
    Serial.println(" night:" + String(isNight) + " bright:" + String(NIGHT_BRIGHTNESS_PERCENT));
     float Charge = readBatteryVoltage();
    Serial.print("Charge:");Serial.print(Charge);
  }

  //  Magnet removed — shut down and sleep 
  Serial.println("Magnet removed - sleeping in 1 second");
  tlcAllOff();
  tlcSetBrightness(0);              // OE forced HIGH — outputs disabled
  accel.writeRegister(0x2D, 0x0C);
  delay(1000);

  esp_deep_sleep_enable_gpio_wakeup(BIT(HALL), ESP_GPIO_WAKEUP_GPIO_LOW);
  Serial.println("Going to sleep");
  delay(200);
  esp_deep_sleep_start();
}

void loop() {}
