

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <RTClib.h>
#include "memory_storage.h"
#include "config_settings.h"

// If your module is a DS1307 instead of a DS3231, this is the only line
// that needs to change -- the rest of the RTClib API is identical.
RTC_DS3231 rtc;

// ---------------------------------------------------------------------------
// Storage design:
// The actual persistent storage is the ATmega2560's internal EEPROM (4KB,
// no external chip or extra wiring needed, survives power-off indefinitely).
// The RTC is NOT used to store data -- it's only used to stamp each
// checkpoint with a real-world time, so that on reboot we can calculate how
// long the power was actually out (millis() can't tell you that, since it
// resets to 0 every boot).
// ---------------------------------------------------------------------------

struct PersistedSettings {
  uint16_t magic;
  float targetHeatTemp;
  unsigned long heatDurMs;
  float targetCoolTemp;
  unsigned long coolDurMs;
  unsigned long mixDurMs;
  float targetHoldTemp;
  unsigned long holdDurMs;
};

struct PersistedRunState {
  uint16_t magic;
  uint8_t processState;
  uint8_t operationMode;
  uint8_t manualState;
  bool timerWasActive;
  unsigned long phaseElapsedMs;  // valid only if timerWasActive
  uint32_t rtcTimestamp;         // rtc.now().unixtime() at last checkpoint
};

const uint16_t MAGIC_SETTINGS = 0xBEEF;
const uint16_t MAGIC_RUNSTATE = 0xCAFE;

const int SETTINGS_ADDR = 0;
const int RUNSTATE_ADDR = SETTINGS_ADDR + sizeof(PersistedSettings);

static bool rtcOk = false;

void initMemoryStorage() {
  Wire.begin(); // Hardware I2C on the Mega: SDA=RTC_SDA_PIN(20), SCL=RTC_SCL_PIN(21) -- fixed by silicon, not configurable
  rtcOk = rtc.begin();

  if (!rtcOk) {
    Serial.println("MEMORY: RTC not detected on I2C bus! Check SDA(20)/SCL(21) wiring.");
    return; // Settings/run-state EEPROM functions still work fine without the RTC --
            // you just lose the outage-duration calculation.
  }

  if (rtc.lostPower()) {
    Serial.println("MEMORY: RTC lost backup battery power -- clock is unreliable. Resetting to compile time.");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
}

bool rtcBatteryLost() {
  return rtcOk && rtc.lostPower();
}

bool loadSettings() {
  PersistedSettings s;
  EEPROM.get(SETTINGS_ADDR, s);

  if (s.magic != MAGIC_SETTINGS) {
    Serial.println("MEMORY: No saved settings found, using compiled defaults.");
    return false;
  }

  TARGET_HEAT_TEMP = s.targetHeatTemp;
  HEAT_DUR_MS      = s.heatDurMs;
  TARGET_COOL_TEMP = s.targetCoolTemp;
  COOL_DUR_MS      = s.coolDurMs;
  MIX_DUR_MS       = s.mixDurMs;
  TARGET_HOLD_TEMP = s.targetHoldTemp;
  HOLD_DUR_MS      = s.holdDurMs;

  Serial.println("MEMORY: Restored saved settings from EEPROM.");
  return true;
}

void saveSettings() {
  PersistedSettings s;
  s.magic          = MAGIC_SETTINGS;
  s.targetHeatTemp = TARGET_HEAT_TEMP;
  s.heatDurMs      = HEAT_DUR_MS;
  s.targetCoolTemp = TARGET_COOL_TEMP;
  s.coolDurMs      = COOL_DUR_MS;
  s.mixDurMs       = MIX_DUR_MS;
  s.targetHoldTemp = TARGET_HOLD_TEMP;
  s.holdDurMs      = HOLD_DUR_MS;
  EEPROM.put(SETTINGS_ADDR, s); // EEPROM.put only rewrites bytes that actually changed
}

void saveRunState(ProcessState state, OperationMode mode, ProcessState manualState,
                   bool timerWasActive, unsigned long phaseElapsedMs) {
  PersistedRunState s;
  s.magic          = MAGIC_RUNSTATE;
  s.processState   = (uint8_t)state;
  s.operationMode  = (uint8_t)mode;
  s.manualState    = (uint8_t)manualState;
  s.timerWasActive = timerWasActive;
  s.phaseElapsedMs = phaseElapsedMs;
  s.rtcTimestamp   = rtcOk ? rtc.now().unixtime() : 0;
  EEPROM.put(RUNSTATE_ADDR, s);
}

bool loadRunState(ProcessState &state, OperationMode &mode, ProcessState &manualState,
                   bool &timerWasActive, unsigned long &phaseElapsedMs,
                   unsigned long &outageDurationMs) {
  PersistedRunState s;
  EEPROM.get(RUNSTATE_ADDR, s);

  if (s.magic != MAGIC_RUNSTATE) {
    return false;
  }

  state            = (ProcessState)s.processState;
  mode             = (OperationMode)s.operationMode;
  manualState      = (ProcessState)s.manualState;
  timerWasActive   = s.timerWasActive;
  phaseElapsedMs   = s.phaseElapsedMs;

  outageDurationMs = 0;
  if (rtcOk && s.rtcTimestamp != 0) {
    uint32_t now = rtc.now().unixtime();
    if (now > s.rtcTimestamp) {
      outageDurationMs = (unsigned long)(now - s.rtcTimestamp) * 1000UL;
    }
  }
  return true;
}

void clearRunState() {
  PersistedRunState s;
  memset(&s, 0, sizeof(s)); // magic = 0 -> invalid, loadRunState() will reject it
  EEPROM.put(RUNSTATE_ADDR, s);
}
