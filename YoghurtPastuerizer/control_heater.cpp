

#include <Arduino.h>
#include "control_heater.h"
#include "config_settings.h"

void initHeater() {
  // Set output pins HIGH before configuration to keep active-low relays from pulsing on boot
  digitalWrite(RELAY_HEATER_PIN, HIGH);   
  digitalWrite(RELAY_AGITATOR_PIN, HIGH); 
  digitalWrite(RELAY_COOLING_PIN, HIGH);  
  digitalWrite(BUZZER_PIN, LOW);          
  
  pinMode(RELAY_HEATER_PIN, OUTPUT);
  pinMode(RELAY_AGITATOR_PIN, OUTPUT);
  pinMode(RELAY_COOLING_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  pinMode(BUTTON_START_PIN, INPUT_PULLUP);
  pinMode(BUTTON_STOP_PIN, INPUT_PULLUP);
  
  turnOffAllActuators();
}

void turnOnAgitator() { 
  digitalWrite(RELAY_AGITATOR_PIN, LOW); // LOW turns active-low relay ON
}

void turnOffHeater() { 
  digitalWrite(RELAY_HEATER_PIN, HIGH); // HIGH turns active-low relay OFF
}

void turnOnCoolingValve() { 
  digitalWrite(RELAY_COOLING_PIN, LOW); // LOW turns active-low cooling valve ON
}

void turnOffAllActuators() {
  digitalWrite(RELAY_HEATER_PIN, HIGH);   
  digitalWrite(RELAY_AGITATOR_PIN, HIGH); 
  digitalWrite(RELAY_COOLING_PIN, HIGH);  
  digitalWrite(BUZZER_PIN, LOW);         
}

// -------------------------------------------------------------------------
// Jacket-driven control: the jacket probe responds almost immediately to
// the relay, so the heater is cut off exactly when the JACKET reaches
// target -- no more guessing an early cutoff band based on the slow,
// laggy product probe. The product is left to passively catch up to the
// jacket via conduction after cutoff (phase completion/display still watch
// the product probe separately, in the .ino).
//
// HEAT_HYSTERESIS exists to stop relay chatter: without it, normal probe
// noise right at the exact target causes `error` to bounce back and forth
// across zero, so the relay follows every bounce and clicks rapidly. With
// hysteresis, once the heater turns off at target, it won't turn back on
// until the jacket temp drops a further 1C below target -- a small amount
// of noise can no longer re-trigger it.
// -------------------------------------------------------------------------
const float HEAT_HYSTERESIS = 1.0; // Dead zone below target before the heater is allowed back on

void runPIDControl(float currentTemp, float targetTemp) {
  static bool heaterOn = true; // Heating always starts well below target, so default to ON

  float error = targetTemp - currentTemp;

  if (heaterOn) {
    if (error <= 0) {
      heaterOn = false; // Jacket reached/passed target: stop, let product catch up
    }
  } else {
    if (error > HEAT_HYSTERESIS) {
      heaterOn = true; // Jacket dropped back out past the hysteresis margin: allow it back on
    }
  }

  digitalWrite(RELAY_HEATER_PIN, heaterOn ? LOW : HIGH);
}

// -------------------------------------------------------------------------
// Same jacket-driven approach, mirrored for cooling. Same hysteresis logic
// as runPIDControl() above, for the same chatter-prevention reason.
// -------------------------------------------------------------------------
const float COOL_HYSTERESIS = 1.0; // Dead zone above target before the valve is allowed back on

void runCoolingControl(float currentTemp, float targetTemp) {
  static bool coolerOn = true; // Cooling always starts well above target, so default to ON

  float error = currentTemp - targetTemp; // positive while still above target

  if (coolerOn) {
    if (error <= 0) {
      coolerOn = false; // Jacket reached/passed target: stop, let product catch up
    }
  } else {
    if (error > COOL_HYSTERESIS) {
      coolerOn = true; // Jacket rose back out past the hysteresis margin: allow it back on
    }
  }

  digitalWrite(RELAY_COOLING_PIN, coolerOn ? LOW : HIGH);
}

// -------------------------------------------------------------------------
// Holding-phase "Thermal Blanket" control (Pure Jacket-Driven)
//
// Philosophy: Keep the water jacket maintained right at target temperature
// (e.g. 43C). Because the milk is surrounded by this thermal blanket, it
// equalises to target naturally without needing slow probe interference.
//
// Control Rules:
//   - Heater turns ON if jacket drops below (target - HOLD_HEATER_HYSTERESIS).
//   - Heater turns OFF once jacket reaches target.
//   - Cooler turns ON if jacket rises above (target + HOLD_COOLER_HYSTERESIS).
//   - Cooler turns OFF once jacket drops back down to target.
//   - Coasting band between (target - 1.0) and (target + 1.0): Both OFF.
//   - HOLD_MIN_DWELL_MS (30s) prevents relay chatter/cracking.
// -------------------------------------------------------------------------

const float HOLD_HEATER_HYSTERESIS        = 1.0;    // Turn heater ON if jacket <= target - 1.0C
const float HOLD_COOLER_HYSTERESIS        = 1.0;    // Turn cooler ON if jacket >= target + 1.0C
const unsigned long HOLD_MIN_DWELL_MS     = 30000;  // Hard min dwell: relay locked for 30s after any state change

void runHoldingControl(float jacketTemp, float targetTemp) {
  static bool heaterOn = false;
  static bool coolerOn = false;
  static unsigned long lastSwitchTime = 0;

  bool wantHeaterOn = heaterOn;
  bool wantCoolerOn = coolerOn;

  // Heater control
  if (heaterOn) {
    if (jacketTemp >= targetTemp) {
      wantHeaterOn = false; // Reached target, turn heater OFF and coast
    }
  } else {
    if (jacketTemp <= targetTemp - HOLD_HEATER_HYSTERESIS) {
      wantHeaterOn = true;  // Dropped below deadband, turn heater ON
    }
  }

  // Cooler control (safety/upper guard)
  if (coolerOn) {
    if (jacketTemp <= targetTemp) {
      wantCoolerOn = false; // Brought back down to target, turn cooler OFF
    }
  } else {
    if (jacketTemp >= targetTemp + HOLD_COOLER_HYSTERESIS) {
      wantCoolerOn = true;  // Rose above deadband, turn cooler ON
    }
  }

  // Interlock: never allow heater and cooler to run simultaneously
  if (wantHeaterOn && wantCoolerOn) {
    wantCoolerOn = false;
  }

  // Dwell guard: enforce minimum time before relay can change state (no cracking)
  unsigned long now = millis();
  if ((wantHeaterOn != heaterOn || wantCoolerOn != coolerOn) &&
      (now - lastSwitchTime >= HOLD_MIN_DWELL_MS)) {
    heaterOn = wantHeaterOn;
    coolerOn = wantCoolerOn;
    lastSwitchTime = now;
  }

  digitalWrite(RELAY_HEATER_PIN, heaterOn ? LOW : HIGH);
  digitalWrite(RELAY_COOLING_PIN, coolerOn ? LOW : HIGH);
}
