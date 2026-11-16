/*
 * ============================================================================
 *  Astronaut Sleep HR & Stress Monitor  —  Part 1: Sensor Node  (Phase 0)
 *  Team "(print)" — see PROJECT.md at the repository root.
 * ============================================================================
 *
 *  Board   : Seeed Studio XIAO nRF52840 / XIAO nRF52840 Sense
 *  Sensor  : SparkFun SEN-11574 "Pulse Sensor Amped" (analog PPG,
 *            World Famous Electronics — https://pulsesensor.com)
 *            NOTE: SEN-11574 is NOT an I2C MAX3010x part; it is an analog
 *            sensor (Signal / VCC / GND) that is sampled at ~500 Hz.
 *
 *  What this sketch does
 *  ---------------------
 *   - Samples the Pulse Sensor at 500 Hz and detects heartbeats.
 *   - Exposes a standard Bluetooth LE Heart Rate Service so any BLE client
 *     (the Android app, nRF Connect, LightBlue, an OS health stack) can
 *     read/subscribe to the live BPM stream.
 *   - Exposes a standard Battery Service and a custom "Session" service that
 *     lets the app start/stop a checkup session (PROJECT.md §3.1).
 *   - Drops obviously invalid readings (no contact / out-of-range BPM)
 *     before they are sent — PROJECT.md §3.1 "sanity check".
 *
 *  Out of scope: journaling or any text input. This node has no display or
 *  keyboard and stores no journal text; the private journal lives in the
 *  Android app (primary) and the browser-local web page (secondary), never in
 *  firmware and never on the shared crew dashboard — PROJECT.md §14.
 *
 *  Wiring
 *  ------
 *    Pulse Sensor purple wire (Signal)  ->  XIAO D0 (A0, P0.02)
 *    Pulse Sensor red wire    (VCC)     ->  XIAO 3V3
 *    Pulse Sensor black wire  (GND)     ->  XIAO GND
 *  (Signal wire color per the PulseSensor starter project; check your
 *   cable silkscreen. The sensor runs fine at 3.3 V, ~4 mA.)
 *  The battery-ADC path (P0.31 + P0.14) is on-board; nothing to wire.
 *
 *  GATT layout (what the Android app should implement against)
 *  -----------------------------------------------------------
 *    0x180D Heart Rate Service (standard, Bluetooth SIG HRS 1.0)
 *      0x2A37 Heart Rate Measurement   notify   [flags][bpm]
 *             flags bit0 = 0 (uint8 bpm); bits1-2 = sensor contact status
 *             (10b = supported, not detected -> 0x04; 11b = supported,
 *             detected -> 0x06). bpm = 0 means "no valid reading" (sensor
 *             not worn / noise).
 *      0x2A38 Body Sensor Location     read     0x03 = finger
 *      0x2A39 Heart Rate Control Point write   0x01 = reset energy (ack'd)
 *    0x180F Battery Service (standard)
 *      0x2A19 Battery Level            read/notify   0..100 (%)
 *    f2a10001-4ba7-4f0e-8d3c-2b6e9d1a5c77  Session service (team vendor UUID)
 *      f2a10002-... Session Control    write     0x01 = start, 0x00 = stop
 *             (ASCII "start"/"stop" is also accepted for generic clients)
 *      f2a10003-... Session Status     read/notify
 *             0x00 stopped, 0x01 running (signal ok), 0x02 running (no signal)
 *
 *  Toolchain (verified against Seeed wiki)
 *  ---------------------------------------
 *   - Arduino IDE board package: "Seeed nRF52 Boards"  (the NON-mbed one)
 *     Boards Manager URL:
 *     https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json
 *   - Board: "Seeed XIAO nRF52840" or "Seeed XIAO nRF52840 Sense"
 *   - BLE comes from the Adafruit Bluefruit nRF52 library that is bundled
 *     with that core (no extra library to install).
 *   - On this core, USB CDC Serial requires including Adafruit_TinyUSB.h
 *     (done below) — see Seeed wiki "XIAO nRF52840 Getting Started".
 *
 *  Algorithm provenance (do not "fix" without reading these)
 *  ---------------------------------------------------------
 *   The beat detector below is a faithful port of
 *   WorldFamousElectronics/PulseSensorPlayground v2.1.0
 *   (src/utility/PulseSensor.cpp, PulseSensor::processLatestSample, MIT),
 *   the official library of the sensor manufacturer. It is driven from
 *   loop() at 500 Hz using micros() — the library's own documented
 *   "software timer" sampling mode (call at least every 2 ms, no delay()).
 *   The library's nRF52 interrupt path would additionally require the
 *   third-party NRF52TimerInterrupt library; the software path keeps this
 *   sketch dependency-free and BLE-friendly. Revisit if sampling jitter
 *   becomes a problem (PULSE_SENSOR_TIMING_ANALYSIS in the upstream lib).
 *
 *  References
 *  ----------
 *   - Seeed wiki XIAO nRF52840:        https://wiki.seeedstudio.com/XIAO_BLE/
 *     (pin map, battery-ADC FAQ Q3: P0.14 must stay LOW while reading)
 *   - Seeed wiki Bluetooth usage:      https://wiki.seeedstudio.com/XIAO-BLE-Sense-Bluetooth_Usage/
 *   - PulseSensor / SEN-11574:         https://pulsesensor.com
 *     https://github.com/WorldFamousElectronics/PulseSensorStarterProject
 *   - Beat algorithm (official):       https://github.com/WorldFamousElectronics/PulseSensorPlayground
 *   - Bluefruit library & examples:    https://github.com/adafruit/Adafruit_nRF52_Arduino
 *   - BLE Heart Rate Service spec:     Bluetooth SIG, HRS 1.0 (0x180D/0x2A37)
 *
 *  This is a wellness/situational-awareness tool, not a medical device
 *  (PROJECT.md §1). Not for clinical use.
 * ============================================================================
 */

// ---------------------------------------------------------------------------
// Core / library requirements
// ---------------------------------------------------------------------------

#if defined(ARDUINO_ARCH_MBED)
#error "This sketch needs the NON-mbed 'Seeed nRF52 Boards' package \
(Tools > Board > Seeed nRF52 Boards > Seeed XIAO nRF52840 [Sense])."
#endif

#if defined(ARDUINO_NRF52_ADAFRUIT)      // Seeed nRF52 Boards core
#include <Adafruit_TinyUSB.h>            // USB CDC Serial on this core (Seeed wiki)
#endif
#include <bluefruit.h>                   // bundled with the Seeed nRF52 Boards core

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

#define DEVICE_NAME            "AstroHR-Node"  // BLE device name (app scans for it)

#define FIRMWARE_VERSION       "0.1.0"

// --- Pulse sensor ----------------------------------------------------------
// PulseSensor Amped must be sampled as close to 500 Hz as possible
// (PulseSensor.com documentation / PulseSensorPlayground::MICROS_PER_READ).
#define SAMPLE_PERIOD_US       2000UL          // 2 ms -> 500 Hz
#define PULSE_PIN              A0              // XIAO D0 = A0 = P0.02
// Initial beat threshold on a 10-bit scale. The signal idles at ~VCC/2
// (~512 counts); the official library default sits slightly above it.
// Tune with the serial plotter output below if your unit/skin contact
// differs (PulseSensor "How to adjust threshold").
#define THRESHOLD_SETTING      530

// Reporting sanity gate (PROJECT.md §3.1 "drop obviously invalid readings").
// Sleep HR can legitimately dip low, so the floor is kept permissive.
#define BPM_MIN_VALID          25
#define BPM_MAX_VALID          230
// A detected beat with an amplitude below this (10-bit counts) is treated
// as noise, not contact — keeps the "sensor not worn" state honest.
#define MIN_PULSE_AMPLITUDE    50

// --- Battery ---------------------------------------------------------------
// Per Seeed wiki (XIAO nRF52840 FAQ Q3): P0.14 = READ_BAT_ENABLE controls
// the battery-reading path, P0.31 = AIN7 is the battery ADC input (internal,
// not on the header). P0.14 must be LOW while reading; do NOT drive it HIGH
// while a battery is charging.
#ifndef VBAT_ENABLE
#define VBAT_ENABLE            14              // P0.14
#endif
#ifndef PIN_VBAT
#define PIN_VBAT               31              // P0.31 (AIN7)
#endif
#define VBAT_MV_PER_LSB        (3600.0f / 1024.0f)   // AR_DEFAULT = 0.6 V x 6 = 3.6 V, 10-bit
#define VBAT_DIVIDER           (1510.0f / 510.0f)    // ~1/3 divider (Seeed wiki / verified community formula)
#define VBAT_EMPTY_MV          3300            // coarse 1S LiPo curve; calibrate later
#define VBAT_FULL_MV           4200
#define BATTERY_SAMPLE_MS      30000UL         // battery re-read period

// --- Session / reporting ---------------------------------------------------
#define REPORT_PERIOD_MS       1000UL          // BPM notification rate: 1 Hz
#define SESSION_START_DEFAULT  true            // Phase 0: stream from boot;
                                               // the app can stop it via GATT.

// --- Debug ---------------------------------------------------------------
#define SERIAL_BOOT_WAIT_MS    1500            // 0 = don't wait at boot (low power)
#define PULSE_SERIAL_PLOTTER   1               // 1: print Signal/BPM at 10 Hz for tuning

// --- Custom session service UUIDs (team vendor base — replace if you
//     register your own 128-bit base) ---------------------------------------
#define SESSION_SERVICE_UUID   "f2a10001-4ba7-4f0e-8d3c-2b6e9d1a5c77"
#define SESSION_CTRL_UUID      "f2a10002-4ba7-4f0e-8d3c-2b6e9d1a5c77"
#define SESSION_STATUS_UUID    "f2a10003-4ba7-4f0e-8d3c-2b6e9d1a5c77"

// Heartbeat indicator: XIAO's 3-in-1 user LED is wired common anode ->
// LOW = lit (Seeed wiki FAQ Q4; the core's own initVariant() turns LEDs off
// by writing HIGH). LED_STATE_ON in the variant header contradicts the
// physical wiring, so we set polarity explicitly. Flip to 0 if a future
// board revision is active-high.
#define LED_ACTIVE_LOW         1

#if defined(LED_RED)
#define HEARTBEAT_LED          LED_RED
#elif defined(LED_BUILTIN)
#define HEARTBEAT_LED          LED_BUILTIN
#else
#define HEARTBEAT_LED          -1
#endif

// ---------------------------------------------------------------------------
// BLE GATT objects
// ---------------------------------------------------------------------------

BLEService        hrsService(0x180D);        // Heart Rate Service (standard)
BLECharacteristic hrmChar(0x2A37);           // Heart Rate Measurement (notify)
BLECharacteristic bslChar(0x2A38);           // Body Sensor Location (read)
BLECharacteristic hrcpChar(0x2A39);          // Heart Rate Control Point (write)

BLEService        batService(0x180F);        // Battery Service (standard)
BLECharacteristic batLevelChar(0x2A19);      // Battery Level (read, notify)

BLEService        sessionService(SESSION_SERVICE_UUID);
BLECharacteristic sessionCtrlChar(SESSION_CTRL_UUID);    // write
BLECharacteristic sessionStatusChar(SESSION_STATUS_UUID);// read + notify

BLEDis            bleDis;                    // Device Information Service

// ---------------------------------------------------------------------------
// PulseSensor state (ported 1:1 from PulseSensorPlayground v2.1.0,
// PulseSensor::resetVariables / processLatestSample)
// ---------------------------------------------------------------------------

int       Signal = 0;                 // latest 10-bit ADC sample
int       BPM    = 0;                  // beats per minute (0 = no valid rate)
uint16_t  IBI    = 750;                // inter-beat interval, ms (seed = 80 bpm)

int       rate[10];                   // last 10 IBI values for the running average
bool      pulseInside = false;        // "Pulse" flag: inside a beat
uint32_t  sampleCounter = 0;          // ms since boot, +2 per sample
uint32_t  lastBeatTime  = 0;          // sampleCounter at the last detected beat
int       P      = 512;               // pulse-wave peak   (10-bit scale)
int       T      = 512;               // pulse-wave trough (10-bit scale)
int       thresh = THRESHOLD_SETTING; // adaptive threshold
int       amp    = 100;               // last beat amplitude
bool      firstBeat  = true;          // beat priming, per upstream library
bool      secondBeat = false;

bool      contactOk = false;          // derived: signal looks like a pulse
bool      sessionRunning = SESSION_START_DEFAULT;

unsigned long nextSampleMicros = 0;   // software-timer schedule (library mode)
unsigned long lastReportMs  = 0;      // 1 Hz BLE report tick
unsigned long lastBatteryMs = 0;      // battery re-read tick
uint8_t       lastBatPercent = 0xFF;
uint8_t       lastStatusByte = 0xFF;
int           lastLedState   = -1;

// ---------------------------------------------------------------------------
// PulseSensor beat detection — faithful port of PulseSensorPlayground v2.1.0
// (src/utility/PulseSensor.cpp). Variable names kept to match the upstream
// implementation and the PulseSensor.com documentation.
// ---------------------------------------------------------------------------

static void pulseResetVariables() {                    // == PulseSensor::resetVariables()
  for (int i = 0; i < 10; ++i) rate[i] = 0;
  BPM = 0;
  IBI = 750;                 // 750 ms per beat = 80 BPM
  pulseInside = false;
  P = 512; T = 512;
  thresh = THRESHOLD_SETTING;
  amp = 100;
  firstBeat = true;
  secondBeat = false;
}

static void pulseProcessLatestSample() {               // == PulseSensor::processLatestSample()
  sampleCounter += 2;                                  // ms elapsed this sample
  uint32_t N = sampleCounter - lastBeatTime;           // ms since last beat

  // Find the peak and trough of the pulse wave.
  // Avoid the dicrotic notch by waiting 3/5 of the last IBI.
  if (Signal < thresh && N > (IBI / 5) * 3) {
    if (Signal < T) T = Signal;                        // T is the trough
  }
  if (Signal > thresh && Signal > P) {
    P = Signal;                                        // P is the peak
  }

  // Look for the heart beat: the signal surges up on every pulse.
  if (N > 250) {                                       // avoid high-frequency noise
    if (Signal > thresh && !pulseInside && N > (IBI / 5) * 3) {
      pulseInside = true;
      IBI = sampleCounter - lastBeatTime;              // time between beats, ms
      lastBeatTime = sampleCounter;

      if (secondBeat) {                                // second beat: seed the
        secondBeat = false;                            // running average so BPM
        for (int i = 0; i <= 9; i++) rate[i] = IBI;    // becomes valid quickly
      }
      if (firstBeat) {                                 // first beat after idle:
        firstBeat = false;                             // IBI is unreliable,
        secondBeat = true;                             // discard it
        return;
      }

      uint16_t runningTotal = 0;                       // average of the last 10 IBIs
      for (int i = 0; i <= 8; i++) {
        rate[i] = rate[i + 1];
        runningTotal += rate[i];
      }
      rate[9] = IBI;
      runningTotal += rate[9];
      runningTotal /= 10;
      BPM = 60000 / runningTotal;                      // beats per minute
    }
  }

  if (Signal < thresh && pulseInside) {                // beat is over (falling edge)
    pulseInside = false;
    amp = P - T;                                       // amplitude of this pulse
    thresh = amp / 2 + T;                              // threshold = 50% of amplitude
    P = thresh;                                        // re-center for next beat
    T = thresh;
  }

  if (N > 2500) {                                      // 2.5 s without a beat:
    thresh = THRESHOLD_SETTING;                        // full reset, as upstream
    P = 512; T = 512;
    lastBeatTime = sampleCounter;
    firstBeat = true;
    secondBeat = false;
    BPM = 0;
    IBI = 600;
    pulseInside = false;
    amp = 100;
  }
}

// ---------------------------------------------------------------------------
// Battery (Seeed wiki: XIAO nRF52840 battery ADC)
// ---------------------------------------------------------------------------

static uint8_t readBatteryPercent() {
  // analogReadResolution stays at 10 bits (the PulseSensor math depends on
  // it); the battery formula below is expressed for 10-bit counts.
  uint32_t counts = analogRead(PIN_VBAT);
  float mv = counts * VBAT_MV_PER_LSB * VBAT_DIVIDER;
  if (mv > VBAT_FULL_MV)  return 100;   // USB powered / divider tolerance
  if (mv < VBAT_EMPTY_MV) return 0;
  return (uint8_t)((mv - VBAT_EMPTY_MV) * 100.0f / (VBAT_FULL_MV - VBAT_EMPTY_MV));
}

// ---------------------------------------------------------------------------
// Session service helpers
// ---------------------------------------------------------------------------

static uint8_t sessionStatusByte() {
  if (!sessionRunning) return 0x00;                    // stopped
  return contactOk ? 0x01 : 0x02;                      // running: signal / no signal
}

static void publishSessionStatus(bool notify) {
  uint8_t st = sessionStatusByte();
  if (st != lastStatusByte) {
    sessionStatusChar.write8(st);                      // fresh value for reads
    if (notify) sessionStatusChar.notify8(st);
    lastStatusByte = st;
  }
}

// Session Control write handler. Canonical protocol: 0x01 = start session,
// 0x00 = stop session. For convenience with generic BLE clients, ASCII text
// "start" / "stop" (case-insensitive) is accepted as well.
static bool asciiWord(const uint8_t* d, uint16_t len, const char* word) {
  uint16_t n = (uint16_t)strlen(word);
  if (len != n) return false;
  for (uint16_t i = 0; i < n; i++) {
    char c = (char)d[i];
    if (c >= 'A' && c <= 'Z') c += 32;                 // case-insensitive
    if (c != word[i]) return false;
  }
  return true;
}

static void sessionCtrlWriteCb(uint16_t conn_hdl, BLECharacteristic* chr,
                               uint8_t* data, uint16_t len) {
  if (len < 1) return;

  bool start = false;
  bool matched = false;
  if (len == 1) {
    matched = (data[0] == 0x01 || data[0] == 0x00);
    start = (data[0] == 0x01);
  } else if (asciiWord(data, len, "start")) {
    matched = true;
    start = true;
  } else if (asciiWord(data, len, "stop")) {
    matched = true;
    start = false;
  }
  if (!matched) {
    publishSessionStatus(true);                        // unknown command: no-op
    return;
  }

  if (start && !sessionRunning) {
    sessionRunning = true;
    pulseResetVariables();                             // fresh start
    Serial.println("[Session] started by app");
  } else if (!start && sessionRunning) {
    sessionRunning = false;
    Serial.println("[Session] stopped by app");
  }
  publishSessionStatus(true);
}

// Heart Rate Control Point: only 0x01 (reset Energy Expended) is defined and
// we don't transmit an energy field, so we just acknowledge and log.
static void hrcpWriteCb(uint16_t conn_hdl, BLECharacteristic* chr,
                        uint8_t* data, uint16_t len) {
  if (len >= 1 && data[0] == 0x01) {
    Serial.println("[HRS] control point: reset energy expended (no-op)");
  }
}

// ---------------------------------------------------------------------------
// Peripheral connection callbacks
// ---------------------------------------------------------------------------

static void prphConnectCb(uint16_t conn_hdl) {
  Serial.printf("[BLE] connected, handle %u\n", conn_hdl);
  publishSessionStatus(true);
}

static void prphDisconnectCb(uint16_t conn_hdl, uint8_t reason) {
  Serial.printf("[BLE] disconnected, handle %u, reason 0x%02X\n", conn_hdl, reason);
  // Advertising restarts automatically (restartOnDisconnect(true)).
}

#if LED_ACTIVE_LOW
static const int LED_ON_LEVEL = LOW;    // common-anode user LED
static const int LED_OFF_LEVEL = HIGH;
#else
static const int LED_ON_LEVEL = HIGH;
static const int LED_OFF_LEVEL = LOW;
#endif

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  if (SERIAL_BOOT_WAIT_MS > 0) {
    uint32_t t0 = millis();
    while (!Serial && (millis() - t0) < SERIAL_BOOT_WAIT_MS) { /* bring-up aid */ }
  }

  Serial.println();
  Serial.printf("AstroHR sensor node %s (Seeed XIAO nRF52840 + Pulse Sensor SEN-11574)\n",
                FIRMWARE_VERSION);

  // Status LED: polarity per LED_ACTIVE_LOW (XIAO user LED = common anode).
  if (HEARTBEAT_LED >= 0) {
    pinMode(HEARTBEAT_LED, OUTPUT);
    digitalWrite(HEARTBEAT_LED, LED_OFF_LEVEL);
  }

  // Battery path: Seeed wiki FAQ Q3 — keep P0.14 LOW to enable the reading
  // path; never drive it HIGH while charging. The divider drain is ~2 uA.
  pinMode(VBAT_ENABLE, OUTPUT);
  digitalWrite(VBAT_ENABLE, LOW);

  // The ported beat detector is calibrated for 10-bit counts (idle ~512).
  analogReadResolution(10);

  pulseResetVariables();
  nextSampleMicros = micros() + SAMPLE_PERIOD_US;
  lastBatteryMs = millis();

  // ---- BLE ---------------------------------------------------------------
  Bluefruit.autoConnLed(false);        // no conn-blink LED (low power)
  Bluefruit.begin();                   // SoftDevice S140 + peripheral role
  Bluefruit.setTxPower(4);             // +4 dBm (nRF52840 max)
  Bluefruit.setName(DEVICE_NAME);
  Bluefruit.Periph.setConnectCallback(prphConnectCb);
  Bluefruit.Periph.setDisconnectCallback(prphDisconnectCb);

  // Heart Rate Service (standard 16-bit UUIDs; Android can consume HRS natively)
  hrsService.begin();
  hrmChar.setProperties(CHR_PROPS_NOTIFY);
  hrmChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  hrmChar.setMaxLen(2);                // [flags][bpm] — variable length
                                     // (avoids the known fixed-len notify bug,
                                     //  adafruit/Adafruit_nRF52_Arduino#856)
  hrmChar.setUserDescriptor("Heart Rate Measurement");
  hrmChar.begin();

  bslChar.setProperties(CHR_PROPS_READ);
  bslChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  bslChar.setFixedLen(1);
  bslChar.setUserDescriptor("Body Sensor Location");
  bslChar.begin();
  bslChar.write8(0x03);                // 3 = finger (Pulse Sensor finger/ear placement)

  hrcpChar.setProperties(CHR_PROPS_WRITE);
  hrcpChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  hrcpChar.setFixedLen(1);
  hrcpChar.setUserDescriptor("Heart Rate Control Point");
  hrcpChar.begin();
  hrcpChar.setWriteCallback(hrcpWriteCb);

  // Battery Service
  batService.begin();
  batLevelChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  batLevelChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  batLevelChar.setFixedLen(1);
  batLevelChar.setUserDescriptor("Battery Level");
  batLevelChar.begin();
  lastBatPercent = readBatteryPercent();
  batLevelChar.write8(lastBatPercent);

  // Session service (custom)
  sessionService.begin();
  sessionCtrlChar.setProperties(CHR_PROPS_WRITE);
  sessionCtrlChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  sessionCtrlChar.setFixedLen(1);
  sessionCtrlChar.setUserDescriptor("Session Control (0x01 start, 0x00 stop)");
  sessionCtrlChar.begin();
  sessionCtrlChar.setWriteCallback(sessionCtrlWriteCb);

  sessionStatusChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  sessionStatusChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  sessionStatusChar.setMaxLen(1);
  sessionStatusChar.setUserDescriptor("Session Status");
  sessionStatusChar.begin();
  publishSessionStatus(false);

  // Device Information Service (nice to have for nRF Connect / the app)
  bleDis.setModel("XIAO nRF52840 HR Node");
  bleDis.setManufacturer("team (print)");
  bleDis.setFirmwareRev(FIRMWARE_VERSION);
  bleDis.begin();

  // ---- Advertising (structure mirrors the Bluefruit bleuart example) ------
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(hrsService);
  Bluefruit.Advertising.addService(batService);
  Bluefruit.Advertising.addService(sessionService);
  Bluefruit.ScanResponse.addName();    // name goes in the scan response

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);   // 20 ms fast, 152.5 ms slow
  Bluefruit.Advertising.setFastTimeout(30);     // seconds of fast advertising
  Bluefruit.Advertising.start(0);               // 0 = advertise forever

  Serial.printf("[BLE] advertising as \"%s\"\n", DEVICE_NAME);
}

// ---------------------------------------------------------------------------
// Main loop — the software-timer sampling mode from PulseSensorPlayground:
// call the sample processing at least every 2 ms and never delay() here.
// ---------------------------------------------------------------------------

void loop() {
  unsigned long now = micros();
  if ((long)(nextSampleMicros - now) > 0L) return;     // not time for a sample yet
  nextSampleMicros = now + SAMPLE_PERIOD_US;           // 500 Hz schedule

  Signal = analogRead(PULSE_PIN);                      // == readNextSample()
  pulseProcessLatestSample();                          // == processLatestSample()

  // Contact heuristic: the state machine zeroes BPM after 2.5 s without a
  // beat, and a real pulse wave has meaningful amplitude.
  contactOk = (BPM != 0) && (amp >= MIN_PULSE_AMPLITUDE);

  // Heartbeat LED follows the beat.
  if (HEARTBEAT_LED >= 0 && (int)pulseInside != lastLedState) {
    digitalWrite(HEARTBEAT_LED, pulseInside ? LED_ON_LEVEL : LED_OFF_LEVEL);
    lastLedState = pulseInside;
  }

#if PULSE_SERIAL_PLOTTER
  static uint32_t lastPlotMs = 0;
  uint32_t plotMs = millis();
  if (plotMs - lastPlotMs >= 100) {                    // 10 Hz plotter feed
    lastPlotMs = plotMs;
    Serial.printf("Signal:%d,Threshold:%d,BPM:%d,Contact:%d\n",
                  Signal, thresh, contactOk ? BPM : 0, contactOk ? 1 : 0);
  }
#endif

  // ---- 1 Hz BLE report tick ----------------------------------------------
  uint32_t ms = millis();
  if (ms - lastReportMs >= REPORT_PERIOD_MS) {
    lastReportMs = ms;

    if (Bluefruit.connected()) {
      // Heart Rate Measurement: [flags][bpm]
      //  flags bit0    = 0 -> bpm is uint8
      //  flags bits1-2 = sensor contact status:
      //                  0b11 supported + detected     -> 0x06
      //                  0b10 supported + not detected -> 0x04
      uint8_t bpmOut = 0;
      uint8_t flags;
      if (sessionRunning && contactOk && BPM >= BPM_MIN_VALID && BPM <= BPM_MAX_VALID) {
        bpmOut = (uint8_t)BPM;
        flags = 0x06;                                // contact detected
      } else {
        flags = 0x04;                                // supported, not detected
      }
      uint8_t hrm[2] = { flags, bpmOut };
      hrmChar.notify(hrm, sizeof(hrm));

      publishSessionStatus(true);

      // Battery: refresh periodically, notify only on change.
      if (ms - lastBatteryMs >= BATTERY_SAMPLE_MS) {
        lastBatteryMs = ms;
        uint8_t pct = readBatteryPercent();
        if (pct != lastBatPercent) {
          lastBatPercent = pct;
          batLevelChar.write8(pct);
          batLevelChar.notify8(pct);
        }
      }
    }
  }

  // TODO(Phase 2/4, PROJECT.md roadmap):
  //  - Power: drive sampling from an RTC timer and sleep between samples
  //    (sd_app_evt_wait) for a full night on a small LiPo.
  //  - Add RR intervals to the HRM payload (flags bit4) if the app wants HRV.
  //  - Deep sleep (sd_power_system_off) when no session and no connection.
}
