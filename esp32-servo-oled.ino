// ESP32 + SSD1306 OLED(128x64, yellow top band) + SG90-class servo
//   OLED SDA -> GPIO21, SCL -> GPIO22, VCC 3.3V
//   Servo signal -> GPIO23, VCC 5V (never power the servo from the 3.3V rail)
//
// Yellow band (top 16 px) : mode + angle readout
// Blue area               : 180 deg dial with ticks, progress arc and a needle
// Motion                  : cosine-eased sweep, or serial-commanded manual move (slew limited)
//
// The panel is probed on both pin orders (21/22 and 22/21) and re-probed every 2 s
// until it answers, so the servo demo keeps running while the display is being wired.

#include <Wire.h>
#include <U8g2lib.h>
#include <ESP32Servo.h>
#include <esp_log.h>

static const uint8_t PIN_SDA   = 21;
static const uint8_t PIN_SCL   = 22;
static const uint8_t PIN_SERVO = 23;

static const int ANG_MIN = 0;
static const int ANG_MAX = 180;
static const int US_MIN  = 500;    // SG90 pulse range; trim if the horn hits its stops
static const int US_MAX  = 2400;

static const uint32_t SWEEP_MS = 2500;   // one-way travel time
static const uint32_t DWELL_MS = 400;    // pause at each end
static const float    SLEW_DPS = 150.0;  // manual-move speed limit, deg/s
static const uint32_t SERVO_DT = 20;     // 50 Hz servo refresh
static const uint32_t OLED_DT  = 50;     // 20 Hz redraw
static const uint32_t PROBE_MS = 2000;   // rescan interval while no panel answers

// dial geometry, in the blue part of the panel
static const int CX = 64, CY = 62, R_ARC = 40, R_TICK = 34, R_NEEDLE = 31;

// one instance per pin order; only the one that answers is ever used
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8gNormal(U8G2_R0, U8X8_PIN_NONE, PIN_SCL, PIN_SDA);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8gSwapped(U8G2_R0, U8X8_PIN_NONE, PIN_SDA, PIN_SCL);
U8G2 *disp = nullptr;

Servo servo;

// PINHUNT drives each candidate pin in turn so a mis-plugged signal wire can be
// found by watching which pin number is on screen when the horn twitches.
static const uint8_t HUNT_PINS[] = {23, 19, 18, 5, 17, 16, 4, 13, 14, 27, 26, 25, 33, 32, 2, 15};
static const uint8_t HUNT_N = sizeof(HUNT_PINS);
static const uint32_t HUNT_MS = 3000;   // time spent on each candidate pin

enum Mode { MODE_SWEEP, MODE_MANUAL, MODE_PINHUNT };
Mode  mode     = MODE_SWEEP;
float curAngle = 90.0;   // what the servo is holding
float tgtAngle = 90.0;   // where manual mode is heading
int   curUs    = 1500;
uint8_t oledAddr = 0;
bool  oledSwapped = false;

uint32_t tServo = 0, tOled = 0, tPhase = 0, tProbe = 0;
bool sweepUp = true, dwelling = false;

uint8_t  huntIdx  = 0;        // candidate pin currently being driven
uint8_t  servoPin = PIN_SERVO;
uint32_t tHunt    = 0;

// servo 0 deg -> right side of the dial, 180 deg -> left side
static inline float dialRad(float deg) { return PI * (1.0f - deg / 180.0f); }

static void polar(float deg, int r, int &x, int &y) {
  float a = dialRad(deg);
  x = CX + (int)lroundf(cosf(a) * r);
  y = CY - (int)lroundf(sinf(a) * r);
}

static void arc(int r, float fromDeg, float toDeg) {
  if (toDeg < fromDeg) { float t = fromDeg; fromDeg = toDeg; toDeg = t; }
  for (float d = fromDeg; d <= toDeg; d += 1.5f) {
    int x, y;
    polar(d, r, x, y);
    disp->drawPixel(x, y);
  }
}

// returns the SSD1306 address answering on this pin order, 0 if the bus is silent
uint8_t probeBus(uint8_t sda, uint8_t scl, bool verbose) {
  Wire.end();
  Wire.begin(sda, scl, 400000);
  uint8_t found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      if (verbose) Serial.printf("  SDA%u/SCL%u: device at 0x%02X\n", sda, scl, a);
      if ((a == 0x3C || a == 0x3D) && !found) found = a;
    }
  }
  return found;
}

// try the documented wiring first, then the swapped one
bool findPanel(bool verbose) {
  uint8_t a = probeBus(PIN_SDA, PIN_SCL, verbose);
  bool swapped = false;
  if (!a) { a = probeBus(PIN_SCL, PIN_SDA, verbose); swapped = true; }
  if (!a) { Wire.end(); Wire.begin(PIN_SDA, PIN_SCL, 400000); return false; }

  oledAddr    = a;
  oledSwapped = swapped;
  disp        = swapped ? (U8G2 *)&u8gSwapped : (U8G2 *)&u8gNormal;
  disp->setI2CAddress(a << 1);
  disp->begin();
  disp->setBusClock(400000);
  Serial.printf("OLED 0x%02X on SDA%u/SCL%u%s\n", a,
                swapped ? PIN_SCL : PIN_SDA, swapped ? PIN_SDA : PIN_SCL,
                swapped ? "  (SDA/SCL are swapped vs the wiring note)" : "");

  disp->clearBuffer();
  disp->setFont(u8g2_font_6x12_tr);
  disp->drawStr(0, 11, "ESP32 SERVO");
  disp->setFont(u8g2_font_5x7_tr);
  disp->drawStr(0, 34, "OLED  SDA21 SCL22");
  disp->drawStr(0, 46, "SERVO GPIO23  50Hz");
  disp->drawStr(0, 58, "serial 115200");
  disp->sendBuffer();
  return true;
}

// ---- loopback pulse check ------------------------------------------------
// Jumper the servo signal pin to MON_PIN and the board measures its own output,
// which settles whether the pin is really producing servo pulses.
static const uint8_t MON_PIN = 19;
volatile uint32_t monRise = 0, monHigh = 0, monPeriod = 0, monCount = 0;

volatile uint8_t monPin = 0;

void IRAM_ATTR monIsrPin() {
  uint32_t t = micros();
  if (digitalRead(monPin)) {
    if (monRise) monPeriod = t - monRise;
    monRise = t;
    monCount++;
  } else if (monRise) {
    monHigh = t - monRise;
  }
}

// pin == servoPin reads the driven pin back with no jumper at all; any other pin
// expects a jumper from the servo signal pin to it.
void measurePulse(uint8_t pin) {
  monPin = pin;
  if (pin != servoPin) pinMode(pin, INPUT_PULLDOWN);
  monRise = monHigh = monPeriod = monCount = 0;
  attachInterrupt(digitalPinToInterrupt(pin), monIsrPin, CHANGE);
  uint32_t t0 = millis();
  while (millis() - t0 < 500) delay(10);
  detachInterrupt(digitalPinToInterrupt(pin));

  Serial.printf("measure GPIO%u (servo on GPIO%u): %u pulses / 500 ms", pin, servoPin, monCount);
  if (monCount)
    Serial.printf(", high=%u us, period=%u us (%.1f Hz), commanded=%d us\n",
                  monHigh, monPeriod, monPeriod ? 1000000.0 / monPeriod : 0.0, curUs);
  else if (pin == servoPin)
    Serial.printf(" - readback unavailable on a driven pin; jumper GPIO%u to GPIO%u and run m%u\n",
                  servoPin, MON_PIN, MON_PIN);
  else
    Serial.printf(" - nothing: no jumper GPIO%u->GPIO%u, or the pin is not driving\n", servoPin, pin);
}

const char *modeName() {
  return mode == MODE_SWEEP ? "SWEEP" : (mode == MODE_MANUAL ? "MANUAL" : "PINHUNT");
}

// full-screen readout of the pin being driven right now
void drawHunt() {
  char buf[24];
  disp->clearBuffer();
  disp->setFont(u8g2_font_6x12_tr);
  disp->drawStr(0, 11, "PINHUNT");
  snprintf(buf, sizeof(buf), "%u/%u", huntIdx + 1, HUNT_N);
  disp->drawStr(128 - disp->getStrWidth(buf), 11, buf);
  disp->drawHLine(0, 15, 128);

  disp->setFont(u8g2_font_logisoso28_tn);
  snprintf(buf, sizeof(buf), "%u", HUNT_PINS[huntIdx]);
  disp->drawStr((128 - disp->getStrWidth(buf)) / 2, 50, buf);

  disp->setFont(u8g2_font_5x7_tr);
  disp->drawStr(0, 26, "GPIO");
  disp->drawStr(0, 63, "moving? this is the pin");
  disp->sendBuffer();
}

void draw() {
  char buf[16];
  if (mode == MODE_PINHUNT) { drawHunt(); return; }
  disp->clearBuffer();

  // ---- yellow status band -------------------------------------------------
  disp->setFont(u8g2_font_6x12_tr);
  disp->drawStr(0, 11, modeName());

  disp->setFont(u8g2_font_helvB12_tr);
  snprintf(buf, sizeof(buf), "%d", (int)lroundf(curAngle));
  int w = disp->getStrWidth(buf);
  disp->drawStr(128 - w - 8, 13, buf);          // leave 8 px for the degree mark
  disp->drawCircle(128 - 4, 3, 2);              // degree symbol

  disp->drawHLine(0, 15, 128);

  // ---- dial ---------------------------------------------------------------
  int x, y, x2, y2;
  arc(R_ARC, ANG_MIN, ANG_MAX);                       // full travel range
  for (int d = ANG_MIN; d <= ANG_MAX; d += 30) {      // ticks every 30 deg
    polar(d, R_TICK, x, y);
    polar(d, (d % 90 == 0) ? R_TICK - 7 : R_TICK - 4, x2, y2);
    disp->drawLine(x, y, x2, y2);
  }
  arc(R_ARC - 3, ANG_MIN, curAngle);                  // travelled portion
  arc(R_ARC - 4, ANG_MIN, curAngle);

  polar(curAngle, R_NEEDLE, x, y);                    // needle
  disp->drawLine(CX, CY, x, y);
  disp->drawLine(CX - 1, CY, x, y);
  disp->drawDisc(CX, CY, 3);

  disp->setFont(u8g2_font_5x7_tr);
  disp->drawStr(0, 63, "180");
  disp->drawStr(118, 63, "0");
  snprintf(buf, sizeof(buf), "%dus", curUs);
  disp->drawStr(0, 26, buf);
  disp->drawStr(97, 26, mode == MODE_SWEEP ? "auto" : "man");

  disp->sendBuffer();
}

void applyServo(float ang) {
  ang = constrain(ang, (float)ANG_MIN, (float)ANG_MAX);
  curUs = (int)lroundf(US_MIN + (US_MAX - US_MIN) * (ang - ANG_MIN) / (float)(ANG_MAX - ANG_MIN));
  servo.writeMicroseconds(curUs);
}

// move the servo signal to another pin at runtime
void useServoPin(uint8_t pin) {
  servoPin = pin;
  if (servo.attached()) servo.detach();
  servo.setPeriodHertz(50);
  servo.attach(pin, US_MIN, US_MAX);
  applyServo(curAngle);
}

void setMode(Mode m) {
  mode   = m;
  tPhase = millis();
  if (m == MODE_PINHUNT) {
    huntIdx = 0;
    tHunt   = millis();
    useServoPin(HUNT_PINS[0]);
    Serial.printf("pinhunt: driving GPIO%u\n", HUNT_PINS[0]);
  }
  if (m == MODE_SWEEP) {  // resume the sweep from wherever the horn currently sits
    sweepUp  = curAngle < (ANG_MIN + ANG_MAX) / 2.0f;
    dwelling = false;
    float done = (sweepUp ? (curAngle - ANG_MIN) : (ANG_MAX - curAngle)) / (float)(ANG_MAX - ANG_MIN);
    tPhase = millis() - (uint32_t)(SWEEP_MS * done);
  }
}

void handleSerial() {
  static char buf[16];
  static uint8_t n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c != '\n' && n < sizeof(buf) - 1) { buf[n++] = c; continue; }
    buf[n] = '\0';
    n = 0;
    if (buf[0] == '\0') continue;
    if (buf[0] == 's' || buf[0] == 'S')      { setMode(MODE_SWEEP); Serial.println("mode=SWEEP"); }
    else if (buf[0] == 'c' || buf[0] == 'C') { tgtAngle = 90; setMode(MODE_MANUAL); Serial.println("center"); }
    else if (buf[0] == 'i' || buf[0] == 'I') { Serial.println("i2c scan:"); if (!findPanel(true)) Serial.println("  bus silent"); }
    else if (buf[0] == 'p' || buf[0] == 'P') { setMode(MODE_PINHUNT); }
    else if (buf[0] == 'm' || buf[0] == 'M') { // m -> read the servo pin back, m19 -> read GPIO19
      long p = strtol(buf + 1, nullptr, 10);
      measurePulse((buf[1] && p >= 0 && p <= 39) ? (uint8_t)p : servoPin);
    }
    else if (buf[0] == 'u' || buf[0] == 'U') { // u23 -> keep driving GPIO23
      long p = strtol(buf + 1, nullptr, 10);
      if (p >= 0 && p <= 39) { useServoPin((uint8_t)p); setMode(MODE_SWEEP); Serial.printf("servo pin=GPIO%ld\n", p); }
      else Serial.println("usage: u<gpio>");
    }
    else if (buf[0] == '?')                  { Serial.printf("mode=%s angle=%.1f us=%d pin=GPIO%u oled=0x%02X%s\n",
                                                 modeName(), curAngle, curUs, servoPin,
                                                 oledAddr, oledSwapped ? " (swapped)" : ""); }
    else {
      char *end;
      long v = strtol(buf, &end, 10);
      if (end != buf) {
        tgtAngle = constrain((float)v, (float)ANG_MIN, (float)ANG_MAX);
        setMode(MODE_MANUAL);
        Serial.printf("target=%.0f\n", tgtAngle);
      } else {
        Serial.println("cmd: <0-180> | s | c | i | p(inhunt) | u<gpio> | m[gpio] measure | ?");
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  esp_log_level_set("i2c.master", ESP_LOG_NONE);   // a missing panel must not flood the console

  if (!findPanel(true)) Serial.println("OLED not answering yet - servo runs, bus is rescanned every 2 s");

  ESP32PWM::allocateTimer(0);
  servo.setPeriodHertz(50);
  servo.attach(PIN_SERVO, US_MIN, US_MAX);
  applyServo(curAngle);

  delay(1500);
  Serial.println("cmd: <0-180> | s | c | i | p(inhunt) | u<gpio> | m[gpio] measure | ?");
  setMode(MODE_SWEEP);
}

void loop() {
  handleSerial();
  uint32_t now = millis();

  if (!disp && now - tProbe >= PROBE_MS) {   // keep looking so live rewiring is picked up
    tProbe = now;
    findPanel(false);
  }

  if (now - tServo >= SERVO_DT) {
    tServo = now;
    if (mode == MODE_SWEEP) {
      uint32_t el = now - tPhase;
      if (dwelling) {
        if (el >= DWELL_MS) { dwelling = false; sweepUp = !sweepUp; tPhase = now; }
      } else if (el >= SWEEP_MS) {
        curAngle = sweepUp ? ANG_MAX : ANG_MIN;
        dwelling = true;
        tPhase   = now;
      } else {
        float t = (float)el / SWEEP_MS;
        float e = 0.5f * (1.0f - cosf(PI * t));   // ease in/out: no jerk at the ends
        curAngle = sweepUp ? ANG_MIN + (ANG_MAX - ANG_MIN) * e
                           : ANG_MAX - (ANG_MAX - ANG_MIN) * e;
      }
    } else if (mode == MODE_PINHUNT) {
      if (now - tHunt >= HUNT_MS) {               // next candidate pin
        tHunt   = now;
        huntIdx = (huntIdx + 1) % HUNT_N;
        useServoPin(HUNT_PINS[huntIdx]);
        Serial.printf("pinhunt: driving GPIO%u\n", HUNT_PINS[huntIdx]);
      }
      // a big, obvious 1 Hz twitch so it is unmistakable which pin is live
      curAngle = ((now - tHunt) / 500) % 2 ? 130.0f : 50.0f;
    } else {
      float step = SLEW_DPS * SERVO_DT / 1000.0f;  // rate limited so it never slams
      if (fabsf(tgtAngle - curAngle) <= step) curAngle = tgtAngle;
      else curAngle += (tgtAngle > curAngle) ? step : -step;
    }
    applyServo(curAngle);
  }

  if (disp && now - tOled >= OLED_DT) { tOled = now; draw(); }
}
