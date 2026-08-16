

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
// Holding-phase "Thermal Blanket" control
//
// Philosophy: keep the jacket at ~TARGET temperature at all times. If the
// jacket is at 43C, the milk physically cannot drift far from 43C,
// regardless of probe lag or lack of agitation. The product probe
// provides a slow trim bias (+/-1C max) to the jacket setpoint to
// compensate for systematic ambient heat loss.
//
// Three layers:
//   1. Product trim  -- biases the jacket setpoint by +/-1C max
//   2. Jacket dead band -- relay only fires when jacket drifts meaningfully
//   3. Committed switching -- relay stays ON until jacket is restored past
//      setpoint, with a hard minimum dwell time to cap switching rate
//
// Expected behaviour: ~3-4 relay switches per hour (heater top-ups).
// Cooler fires rarely (only if product enters holding above target).
// -------------------------------------------------------------------------

const float HOLD_JACKET_DEADBAND          = 1.0;    // Jacket must drift this far from setpoint before relay fires
const float HOLD_JACKET_RESTORE           = 1.5;    // Relay stays ON until jacket is pushed this far past setpoint
const float HOLD_PRODUCT_DEADBAND         = 0.5;    // Product dead band -- no bias applied inside this range
const float HOLD_PRODUCT_BIAS_MAX         = 1.0;    // Max bias on jacket setpoint (caps jacket to TARGET +/- 1C)
const unsigned long HOLD_MIN_DWELL_MS     = 30000;  // Hard min dwell: relay locked for 30s after any state change

void runHoldingControl(float jacketTemp, float productTemp, float targetTemp) {
  static bool heaterOn = false;
  static bool coolerOn = false;
  static unsigned long lastSwitchTime = 0;

  // === LAYER 1: Product Trim (slow, gentle jacket setpoint bias) ===
  float productError = targetTemp - productTemp;  // positive = product is cold
  float productBias = 0.0;

  if (productError > HOLD_PRODUCT_DEADBAND) {
    // Product is cold -- nudge jacket setpoint warmer
    productBias = min(productError, HOLD_PRODUCT_BIAS_MAX);
  } else if (productError < -HOLD_PRODUCT_DEADBAND) {
    // Product is hot -- nudge jacket setpoint cooler
    productBias = max(productError, -HOLD_PRODUCT_BIAS_MAX);
  }
  // else: product within dead band -- no bias, jacket stays at target

  // === LAYER 2: Jacket Setpoint (always 42C - 44C) ===
  float jacketSetpoint = targetTemp + productBias;

  // === LAYER 3: Relay Control (committed bang-bang with dead band) ===
  float jacketError = jacketSetpoint - jacketTemp;  // positive = jacket is cold

  bool wantHeaterOn = heaterOn;
  bool wantCoolerOn = coolerOn;

  if (jacketError > 0) {
    // Jacket is below setpoint -- may need heating
    wantCoolerOn = false;
    if (!heaterOn) {
      // Only trigger if jacket has drifted meaningfully below setpoint
      wantHeaterOn = (jacketError >= HOLD_JACKET_DEADBAND);
    } else {
      // Already heating -- stay ON until jacket is restored well past setpoint
      wantHeaterOn = true;  // stay committed -- checked below in the overshoot path
    }
  } else {
    // Jacket is at or above setpoint
    float jacketOvershoot = -jacketError;  // positive = how far above setpoint

    if (heaterOn) {
      // Heating -- turn off once jacket has been restored past setpoint
      wantHeaterOn = (jacketOvershoot < HOLD_JACKET_RESTORE) ? true : false;
    } else {
      wantHeaterOn = false;
    }

    // May need cooling if jacket is significantly above setpoint
    if (!coolerOn) {
      wantCoolerOn = (jacketOvershoot >= HOLD_JACKET_DEADBAND);
    } else {
      // Already cooling -- stay committed until jacket drops back below setpoint
      wantCoolerOn = (jacketOvershoot > 0);  // stay on while still above setpoint
    }
  }

  // Never run heater and cooler simultaneously
  if (wantHeaterOn && wantCoolerOn) {
    wantCoolerOn = false;
  }

  // === DWELL GUARD: Hard rate limiter ===
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
