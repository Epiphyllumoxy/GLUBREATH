/*
  GluBreath — ESP32 Firmware (No DHT, No Flow Sensor)

  Features
  - Warm-up for MQ-138
  - Guided breath sampling (fixed window)
  - MQ-138 reading (analog), moving-average smoothing
  - Threshold classification displayed on 16×2 I2C LCD
  - Buzzer + LEDs + buttons (Start / Repeat / Back)

  NOTE: Calibrate MQ-138! Replace Ro, A, B with your values.
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ======== LCD ========
LiquidCrystal_I2C lcd(0x27, 20, 4);   // Change to 0x3F if needed

// ======== Pins ========
// Analog input (ESP32 ADC1 pins recommended)
const int PIN_MQ138_AIN = 34;

// Buttons (active LOW, with pull-ups)
const int PIN_BTN_START  = 13;
const int PIN_BTN_REPEAT = 14;
const int PIN_BTN_BACK   = 25;

// LEDs
const int PIN_LED_PWR   = 2;   // onboard or external
const int PIN_LED_OK    = 26;  // green
const int PIN_LED_WARN  = 33;  // red

// Buzzer (active)
const int PIN_BUZZER    = 15;

// ======== Timings (ms) ========
const unsigned long WARMUP_MS        = 90UL * 1000UL; // tune as needed
const unsigned long BREATH_WINDOW_MS = 8000UL;        // 5–10s typical
const unsigned long PROMPT_BEEP_MS   = 120;

// ======== Sampling / filtering ========
const int   MQ_SAMPLES         = 25;   // moving average window (reserved; simple avg used)
const int   MQ_SAMPLE_DELAY_MS = 20;   // ~50 Hz

// ======== Thresholds (ppm) — tune for your calibration ========
const float PPM_NORMAL_MAX = 0.5f;
const float PPM_WARN_MAX   = 1.5f;

// ======== MQ-138 calibration placeholders ========
struct MQ138Cal {
  float vref;     // assumed ADC reference (~3.3V)
  float rLoad;    // load resistor (Ohms), e.g., 47k
  float Ro;       // sensor resistance in clean air (Ohms) — MEASURE!
  // log curve: log10(ppm) = A * log10(Rs/Ro) + B
  float A;
  float B;
} cal = {
  3.3f,
  47000.0f,
  120000.0f,   // <— replace with YOUR measured Ro
  -1.7f,       // <— replace with YOUR fitted slope
  1.2f         // <— replace with YOUR fitted intercept
};

// ======== State machine ========
enum class State { INIT, READY, WARMUP, SAMPLING, PROCESSING, RESULT, ERROR };
State state = State::INIT;

// ======== Buttons (debounce) — renamed to avoid collisions ========
struct GBButton {
  int pin;
  bool last;
  unsigned long lastChange;
};

GBButton btnStart{PIN_BTN_START, HIGH, 0};
GBButton btnRepeat{PIN_BTN_REPEAT, HIGH, 0};
GBButton btnBack{PIN_BTN_BACK, HIGH, 0};

const unsigned long DEBOUNCE_MS = 60;

// Function declaration
bool checkButtonPressed(GBButton &b);

// ======== Utils ========
void beep(unsigned ms = PROMPT_BEEP_MS) {
  digitalWrite(PIN_BUZZER, HIGH);
  delay(ms);
  digitalWrite(PIN_BUZZER, LOW);
}

void setLEDs(bool pwr, bool ok, bool warn) {
  digitalWrite(PIN_LED_PWR,  pwr ? HIGH : LOW);
  digitalWrite(PIN_LED_OK,   ok  ? HIGH : LOW);
  digitalWrite(PIN_LED_WARN, warn? HIGH : LOW);
}

void lcdCenterLine(uint8_t row, const String &s) {
  int len = s.length();
  int left = max(0, (20 - len) / 2);
  lcd.setCursor(0, row);
  for (int i=0; i<left; i++) lcd.print(' ');
  lcd.print(s.substring(0, min(20, len)));
  for (int i=left + len; i<20; i++) lcd.print(' ');
}

String fmt1(float v) {
  char buf[16];
  snprintf(buf, sizeof(buf), "%.1f", v);
  return String(buf);
}

// ADC helper
float adcToV(int raw) {
  return (3.3f * (float)raw) / 4095.0f;   // idealized
}

// Rs from Vout across Rload
float rsFromV(float vOut, const MQ138Cal& c) {
  vOut = max(0.01f, min(c.vref - 0.01f, vOut));
  return c.rLoad * (c.vref - vOut) / vOut;
}

// Convert Rs/Ro -> ppm via placeholder log curve
float rsRatioToPPM(float rs, const MQ138Cal& c) {
  float ratio = rs / c.Ro;
  ratio = max(0.05f, min(20.0f, ratio));
  float logppm = c.A * log10(ratio) + c.B;
  float ppm = pow(10.0f, logppm);
  return max(0.0f, ppm);
}

// Button checking function - moved after variable declarations
bool checkButtonPressed(GBButton &b) {
  bool raw = digitalRead(b.pin);  // active LOW
  if (raw != b.last && (millis() - b.lastChange) > DEBOUNCE_MS) {
    b.last = raw;
    b.lastChange = millis();
    if (raw == LOW) return true;  // pressed
  }
  return false;
}

// ======== Sampling window (moving average only) ========
struct SampleResult {
  float ppm;
  float avgADC;
  float avgV;
};

struct SampleResult captureBreathWindow(unsigned long windowMs) {
  unsigned long t0 = millis();
  long sumADC = 0;
  int count = 0;

  // Simple moving average sampling
  while (millis() - t0 < windowMs) {
    int raw = analogRead(PIN_MQ138_AIN);
    sumADC += raw;
    count++;
    delay(MQ_SAMPLE_DELAY_MS);
  }

  float avgADC = (count > 0) ? (float)sumADC / (float)count : 0.0f;
  float v = adcToV((int)avgADC);
  float rs = rsFromV(v, cal);
  float ppm = rsRatioToPPM(rs, cal);

  return { ppm, avgADC, v };
}

// ======== UI ========
void uiSplash() {
  lcd.clear();
  lcdCenterLine(0, "GLUBREATH");
  lcdCenterLine(1, "Acetone Breath Test");
  lcdCenterLine(2, "Press START to begin");
  lcdCenterLine(3, "Back=Menu  Repeat=—");
}

void uiWarmupCountdown(unsigned long remainMs) {
  lcd.clear();
  lcdCenterLine(0, "Warming up sensor");
  char buf[21];
  snprintf(buf, sizeof(buf), "Ready in %lus", remainMs/1000UL);
  lcdCenterLine(1, buf);
  lcdCenterLine(2, "Keep device steady");
  lcdCenterLine(3, "and ventilated");
}

void uiSamplingProgress(unsigned long elapsed, unsigned long total) {
  lcd.clear();
  lcdCenterLine(0, "Exhale steadily...");
  char buf[21];
  snprintf(buf, sizeof(buf), "Time: %lus/%lus", elapsed/1000UL, total/1000UL);
  lcdCenterLine(1, buf);
  lcdCenterLine(2, "Don't inhale device");
  lcdCenterLine(3, "Listen for beep");
}

void uiProcessing() {
  lcd.clear();
  lcdCenterLine(1, "Processing sample...");
}

void uiResult(const SampleResult& r) {
  lcd.clear();

  // Classification (simple thresholds)
  String interp;
  bool ok=false, warn=false;

  if (r.ppm < PPM_NORMAL_MAX) {
    interp = "Normal glucose";
    ok = true;
  } else if (r.ppm <= PPM_WARN_MAX) {
    interp = "Check for Hyperglycemia";
    warn = true;
  } else {
    interp = "Ketoacidosis risk";
    warn = true;
  }

  char line0[21], line1[21], line3[21];
  snprintf(line0, sizeof(line0), "Acetone: %s ppm", fmt1(r.ppm).c_str());
  snprintf(line1, sizeof(line1), "%s", interp.c_str());
  snprintf(line3, sizeof(line3), "ADC:%d  V:%s", (int)r.avgADC, fmt1(r.avgV).c_str());

  lcdCenterLine(0, line0);
  lcdCenterLine(1, line1);
  lcdCenterLine(2, "—");
  lcdCenterLine(3, line3);

  setLEDs(true, ok, warn);
}

// ======== Setup ========
void setup() {
  Serial.begin(115200);

  pinMode(PIN_MQ138_AIN, INPUT);

  pinMode(PIN_BTN_START,  INPUT_PULLUP);
  pinMode(PIN_BTN_REPEAT, INPUT_PULLUP);
  pinMode(PIN_BTN_BACK,   INPUT_PULLUP);

  pinMode(PIN_LED_PWR,  OUTPUT);
  pinMode(PIN_LED_OK,   OUTPUT);
  pinMode(PIN_LED_WARN, OUTPUT);
  pinMode(PIN_BUZZER,   OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  setLEDs(true, false, false);

  lcd.init();
  lcd.backlight();

  uiSplash();
  state = State::READY;
}

// ======== Loop ========
void loop() {
  switch (state) {
    case State::READY: {
      if (checkButtonPressed(btnStart)) {
        state = State::WARMUP;
      } else if (checkButtonPressed(btnRepeat)) {
        state = State::SAMPLING;
      } else if (checkButtonPressed(btnBack)) {
        uiSplash();
      }
      delay(30);
    } break;

    case State::WARMUP: {
      unsigned long t0 = millis();
      while (millis() - t0 < WARMUP_MS) {
        unsigned long remain = WARMUP_MS - (millis() - t0);
        uiWarmupCountdown(remain);
        setLEDs(true, false, false);
        if (checkButtonPressed(btnBack)) { state = State::READY; uiSplash(); return; }
        delay(250);
      }
      beep();
      state = State::SAMPLING;
    } break;

    case State::SAMPLING: {
      unsigned long t0 = millis();
      while (millis() - t0 < BREATH_WINDOW_MS) {
        uiSamplingProgress(millis() - t0, BREATH_WINDOW_MS);
        if (checkButtonPressed(btnBack)) { state = State::READY; uiSplash(); return; }
        delay(150);
      }
      state = State::PROCESSING;
    } break;

    case State::PROCESSING: {
      uiProcessing();
      beep();
      // Optional short stabilization reads
      (void)captureBreathWindow(800);
      // Main window read
      SampleResult res = captureBreathWindow(1200);

      uiResult(res);
      state = State::RESULT;
    } break;

    case State::RESULT: {
      if (checkButtonPressed(btnRepeat)) {
        setLEDs(true, false, false);
        state = State::SAMPLING;
      } else if (checkButtonPressed(btnBack)) {
        setLEDs(true, false, false);
        uiSplash();
        state = State::READY;
      }
      delay(30);
    } break;

    default:
      uiSplash();
      state = State::READY;
      delay(200);
      break;
  }
}