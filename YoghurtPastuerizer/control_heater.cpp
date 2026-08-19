

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
// Product-driven cooling control:
// To prevent thermal overshoot (where cold jacket water in the walls keeps
// pulling heat and over-chills the milk into the 30s), the cooling valve is
// cut off when the product reaches (targetTemp + COOL_EARLY_CUTOFF).
// The agitator remains running to let residual jacket cold bring the product
// smoothly down to the exact target.
// -------------------------------------------------------------------------
const float COOL_EARLY_CUTOFF = 5.0; // Cut off valve 5C before target to prevent thermal overshoot
const float COOL_HYSTERESIS   = 1.0; // Dead zone above cutoff threshold before valve can re-engage

void runCoolingControl(float currentProductTemp, float targetTemp) {
  static bool coolerOn = true; // Cooling starts well above target, default to ON

  float cutoffThreshold = targetTemp + COOL_EARLY_CUTOFF;
  float error = currentProductTemp - cutoffThreshold; // > 0 while product is still above cutoff threshold

  if (coolerOn) {
    if (error <= 0) {
      coolerOn = false; // Product reached target + 5C: cut off valve and coast down
    }
  } else {
    if (error > COOL_HYSTERESIS) {
      coolerOn = true; // Product temperature rose above cutoff + hysteresis: turn valve back on
    }
  }

  digitalWrite(RELAY_COOLING_PIN, coolerOn ? LOW : HIGH);
}

// -------------------------------------------------------------------------
// Holding-phase "Time-Proportional" Duty Cycle Control (Jacket-Driven)
//
// Period Window: 60 Seconds (60,000 ms)
//
// Stepped Duty Cycle based on Temperature Deviation from target:
//   - Deviation <= 1.0C : Rest zone (0s / 60s -> Actuators OFF)
//   - Deviation >= 2.0C : 10s ON / 60s
//   - Deviation >= 3.0C : 15s ON / 60s
//   - Deviation >= 4.0C : 25s ON / 60s
//   - Deviation >= 5.0C : 60s ON / 60s (Fully OPEN / Continuous ON)
//
// Anti-Cracking Protection:
//   - The duty duration is evaluated and latched at the start of each 60s
//     window so sensor noise at boundary thresholds cannot chatter the relay.
//   - Max switches per minute = 2 (one ON, one OFF).
// -------------------------------------------------------------------------

const unsigned long HOLD_CYCLE_PERIOD_MS = 60000UL; // 60-second cycle period

void runHoldingControl(float jacketTemp, float targetTemp) {
  static unsigned long windowStartTime = 0;
  static unsigned long latchedOnDurationMs = 0;
  static int latchedMode = 0; // 0 = rest, 1 = heat, 2 = cool

  unsigned long now = millis();

  // Evaluate and lock the duty cycle decision at the beginning of each 60-second cycle
  if (now - windowStartTime >= HOLD_CYCLE_PERIOD_MS || windowStartTime == 0) {
    windowStartTime = now;

    float error = targetTemp - jacketTemp; // positive = cold (needs heat), negative = hot (needs cool)
    float dev = abs(error);

    if (dev >= 5.0) {
      latchedOnDurationMs = 60000UL; // Fully ON (60s / 60s)
    } else if (dev >= 4.0) {
      latchedOnDurationMs = 25000UL; // 25s / 60s
    } else if (dev >= 3.0) {
      latchedOnDurationMs = 15000UL; // 15s / 60s
    } else if (dev >= 2.0) {
      latchedOnDurationMs = 10000UL; // 10s / 60s
    } else {
      latchedOnDurationMs = 0;       // <= 1.0C Rest zone (0s / 60s)
    }

    if (latchedOnDurationMs > 0) {
      latchedMode = (error > 0) ? 1 : 2; // 1 = Heat, 2 = Cool
    } else {
      latchedMode = 0; // Rest
    }
  }

  unsigned long windowElapsed = now - windowStartTime;

  bool heaterOn = false;
  bool coolerOn = false;

  // Active during the latched ON window
  if (latchedMode != 0 && windowElapsed < latchedOnDurationMs) {
    if (latchedMode == 1) {
      heaterOn = true;
      coolerOn = false;
    } else if (latchedMode == 2) {
      coolerOn = true;
      heaterOn = false;
    }
  }

  // Active-low relay output (LOW = ON, HIGH = OFF)
  digitalWrite(RELAY_HEATER_PIN, heaterOn ? LOW : HIGH);
  digitalWrite(RELAY_COOLING_PIN, coolerOn ? LOW : HIGH);
}
