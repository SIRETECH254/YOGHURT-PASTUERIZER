

#include <Arduino.h>
#include "control_heater.h"
#include "config_settings.h"

// -------------------------------------------------------------------------
// ANTI-CHATTER & ACTUATOR PROTECTION TIMERS
// Physical relays, contactors, and solenoid valve coils will burn out if
// rapidly switched. These dwell times enforce minimum ON and OFF run times.
// -------------------------------------------------------------------------
const unsigned long MIN_HEATER_DWELL_MS   = 6000UL; // 6s minimum ON/OFF time for heating element
const unsigned long MIN_COOLER_DWELL_MS   = 3500UL; // 3.5s minimum ON/OFF dwell time to allow 4s pulses
const unsigned long MIN_AGITATOR_DWELL_MS = 3000UL; // 3s minimum ON/OFF time for agitator motor

static bool heaterState = false;
static unsigned long lastHeaterSwitchTime = 0;

static bool coolerState = false;
static unsigned long lastCoolerSwitchTime = 0;

static bool agitatorState = false;
static unsigned long lastAgitatorSwitchTime = 0;

// Internal state tracking for phase controllers
static bool heatingDemand = true;
static bool coolingDemand = true;

static unsigned long holdWindowStartTime = 0;
static unsigned long holdHeaterPulseDuration = 0;
static bool holdCoolingPulseActive = false;
static unsigned long holdCoolingPulseStartTime = 0;
static unsigned long lastHoldCoolPulseEndTime = 0;

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
  
  turnOffAllActuators(true);
}

// -------------------------------------------------------------------------
// SAFE RELAY DRIVERS (Hardware protection layer)
// All relay activations must pass through these functions to guarantee
// that no relay chattering or rapid clicking can occur.
// -------------------------------------------------------------------------
void setHeaterRelay(bool on, bool forceImmediate) {
  if (forceImmediate) {
    heaterState = false;
    lastHeaterSwitchTime = millis();
    digitalWrite(RELAY_HEATER_PIN, HIGH); // Active-low: HIGH turns OFF
    return;
  }

  if (on == heaterState) return;

  unsigned long now = millis();
  if (now - lastHeaterSwitchTime >= MIN_HEATER_DWELL_MS || lastHeaterSwitchTime == 0) {
    heaterState = on;
    lastHeaterSwitchTime = now;
    digitalWrite(RELAY_HEATER_PIN, on ? LOW : HIGH); // Active-low: LOW turns ON
  }
}

void setCoolingRelay(bool on, bool forceImmediate) {
  if (forceImmediate) {
    coolerState = false;
    lastCoolerSwitchTime = millis();
    digitalWrite(RELAY_COOLING_PIN, HIGH); // Active-low: HIGH turns OFF
    return;
  }

  if (on == coolerState) return;

  unsigned long now = millis();
  if (now - lastCoolerSwitchTime >= MIN_COOLER_DWELL_MS || lastCoolerSwitchTime == 0) {
    coolerState = on;
    lastCoolerSwitchTime = now;
    digitalWrite(RELAY_COOLING_PIN, on ? LOW : HIGH); // Active-low: LOW turns ON
  }
}

void setAgitatorRelay(bool on, bool forceImmediate) {
  if (forceImmediate) {
    agitatorState = false;
    lastAgitatorSwitchTime = millis();
    digitalWrite(RELAY_AGITATOR_PIN, HIGH); // Active-low: HIGH turns OFF
    return;
  }

  if (on == agitatorState) return;

  unsigned long now = millis();
  if (now - lastAgitatorSwitchTime >= MIN_AGITATOR_DWELL_MS || lastAgitatorSwitchTime == 0) {
    agitatorState = on;
    lastAgitatorSwitchTime = now;
    digitalWrite(RELAY_AGITATOR_PIN, on ? LOW : HIGH); // Active-low: LOW turns ON
  }
}

void turnOnAgitator() { 
  setAgitatorRelay(true);
}

void turnOffHeater(bool forceImmediate) { 
  setHeaterRelay(false, forceImmediate);
}

void turnOnCoolingValve() { 
  setCoolingRelay(true);
}

void turnOffAllActuators(bool forceImmediate) {
  setHeaterRelay(false, forceImmediate);
  setCoolingRelay(false, forceImmediate);
  setAgitatorRelay(false, forceImmediate);
  digitalWrite(BUZZER_PIN, LOW);         
}

void resetActuatorControl() {
  heatingDemand = true;
  coolingDemand = true;
  holdWindowStartTime = 0;
  holdHeaterPulseDuration = 0;
  holdCoolingPulseActive = false;
  holdCoolingPulseStartTime = 0;
  lastHoldCoolPulseEndTime = 0;
}

// -------------------------------------------------------------------------
// HEATING PHASE: Jacket-driven control with 2.0°C hysteresis + dwell safety
//
// The jacket probe responds rapidly to the heating element. The heater turns
// OFF immediately when the JACKET reaches the target temperature.
// To eliminate clicking/chattering, a 2.0°C hysteresis deadband and the 6s
// minimum dwell timer ensure that small analog noise or rapid thermal ripples
// cannot re-trigger the relay until temperature drops cleanly below target.
// -------------------------------------------------------------------------
const float HEAT_HYSTERESIS = 2.0; // 2.0C dead zone below target before heater can turn back on

void runPIDControl(float jacketTemp, float targetTemp) {
  // Jacket reached or exceeded target: turn heater demand OFF
  if (jacketTemp >= targetTemp) {
    heatingDemand = false;
  }
  // Only turn heater demand back ON once jacket temperature drops below (target - hysteresis)
  else if (jacketTemp <= (targetTemp - HEAT_HYSTERESIS)) {
    heatingDemand = true;
  }

  setHeaterRelay(heatingDemand);
}

// -------------------------------------------------------------------------
// COOLING PHASE: Product-driven cooling with 2.0°C hysteresis + dwell safety
//
// To prevent thermal overshoot (cold jacket water pulling heat past the
// target), the cooling valve cuts off when the product reaches
// (targetTemp + COOL_EARLY_CUTOFF). The agitator continues running to let
// residual jacket cold bring product smoothly down to target.
// -------------------------------------------------------------------------
const float COOL_EARLY_CUTOFF = 5.0; // Cut off valve 5C before target to prevent thermal overshoot
const float COOL_HYSTERESIS   = 2.0; // 2.0C dead zone above cutoff before valve can re-engage

void runCoolingControl(float productTemp, float targetTemp) {
  float cutoffThreshold = targetTemp + COOL_EARLY_CUTOFF; // e.g. 45.0 + 5.0 = 50.0C

  // Product reached or dropped below cutoff: close valve
  if (productTemp <= cutoffThreshold) {
    coolingDemand = false;
  }
  // Only re-open valve if product temperature warms back up past cutoff + hysteresis
  else if (productTemp >= (cutoffThreshold + COOL_HYSTERESIS)) {
    coolingDemand = true;
  }

  setCoolingRelay(coolingDemand);
}

// -------------------------------------------------------------------------
// HOLDING PHASE: Controlled Incubation with Strict Pulsed Cooling Relief
//
// 1. Rest Zone (Normal Incubation: target - 1.0°C <= Product <= target + 2.0°C):
//    Both Heater and Cooling Valve remain completely OFF.
//    At 43.0°C (for 43°C - 45°C setpoint), system is in the Rest Zone (0% cooling).
//
// 2. Heating Maintenance: Product < (target - 1.0°C) AND Jacket < 46.0°C:
//    Applies a smooth, gentle heat pulse (15s ON / 60s window).
//    Cut off immediately if jacket reaches 46.0°C or product recovers.
//
// 3. Emergency Cooling Relief: Product >= (target + 2.5°C) ONLY:
//    STRICTLY PULSED: 4 seconds of every minute (4s ON / 56s rest).
//    Jacket temperature alone NEVER triggers cooling.
//
// 4. Agitator: Remains OFF to let yoghurt curd set undisturbed.
// -------------------------------------------------------------------------
const float HOLD_HEAT_TRIGGER_OFFSET  = 1.0;     // Heat starts when Product < (target - 1.0C) (e.g. < 42.0C)
const float HOLD_COOL_TRIGGER_OFFSET  = 2.5;     // Cool relief ONLY if Product >= (target + 2.5C) (e.g. >= 45.5C)
const float JACKET_HOLD_HEAT_LIMIT    = 46.0;    // Heater cut off if jacket reaches 46.0C to prevent overshoot
const unsigned long HOLD_HEAT_WINDOW_MS  = 60000UL;  // 60-second heat cycle window
const unsigned long HOLD_COOL_PULSE_MS   = 10000UL;   // Exactly 10 seconds cold pulse
const unsigned long HOLD_COOL_LOCKOUT_MS = 50000UL;  // 50 seconds rest (4s ON + 56s rest = 1 pulse of 4s every minute)

void runHoldingControl(float productTemp, float jacketTemp, float targetTemp) {
  unsigned long now = millis();

  // =========================================================================
  // 1. HEATING SUBSYSTEM (Maintenance)
  // =========================================================================
  if (now - holdWindowStartTime >= HOLD_HEAT_WINDOW_MS || holdWindowStartTime == 0) {
    holdWindowStartTime = now;
    holdHeaterPulseDuration = 0;

    // Only heat if PRODUCT is cold (< target - 1.0C) and jacket is safe (< 46.0C)
    if (productTemp < (targetTemp - HOLD_HEAT_TRIGGER_OFFSET) && jacketTemp < JACKET_HOLD_HEAT_LIMIT) {
      float deficit = (targetTemp - HOLD_HEAT_TRIGGER_OFFSET) - productTemp;
      if (deficit >= 2.0) {
        holdHeaterPulseDuration = 25000UL; // 25s pulse if very cold
      } else {
        holdHeaterPulseDuration = 15000UL; // 15s gentle maintenance pulse
      }
    }
  }

  unsigned long elapsedHeat = now - holdWindowStartTime;
  bool wantHeater = (holdHeaterPulseDuration > 0 && elapsedHeat < holdHeaterPulseDuration);

  // Soft safety limits for heater:
  // - If jacket exceeds safe heat limit (46.0C), cut heater immediately
  if (jacketTemp >= JACKET_HOLD_HEAT_LIMIT) {
    wantHeater = false;
  }
  // - If product recovers into comfort zone, finish heat pulse early
  if (productTemp >= (targetTemp - 0.2)) {
    wantHeater = false;
  }

  // =========================================================================
  // 2. COOLING SUBSYSTEM (Strictly Pulsed: 4s of every minute - NEVER Continuous)
  // =========================================================================
  bool wantCooler = false;

  if (holdCoolingPulseActive) {
    // Currently executing a short cold pulse: check if pulse time (4s) expired or product cooled
    if (now - holdCoolingPulseStartTime >= HOLD_COOL_PULSE_MS || productTemp <= (targetTemp + 1.0)) {
      holdCoolingPulseActive = false;
      lastHoldCoolPulseEndTime = now; // Start the 56s rest lockout (totalling 1 minute per cycle)
      wantCooler = false;
    } else {
      wantCooler = true; // Continue remaining fraction of the 4s pulse
    }
  } else {
    // Pulse is NOT active: cooling is OFF by default.
    // Can only start a new 4s pulse if:
    // 1) Product core is genuinely hot (>= target + 2.5C, e.g. >= 45.5C for 43C target)
    // 2) The 56-second rest lockout has elapsed (1 pulse of 4s every minute)
    bool lockoutExpired = (lastHoldCoolPulseEndTime == 0 || (now - lastHoldCoolPulseEndTime >= HOLD_COOL_LOCKOUT_MS));
    bool productOverheated = (productTemp >= (targetTemp + HOLD_COOL_TRIGGER_OFFSET));

    if (productOverheated && lockoutExpired) {
      holdCoolingPulseActive = true;
      holdCoolingPulseStartTime = now;
      wantCooler = true;
    } else {
      wantCooler = false; // Forced 100% OFF
    }
  }

  // If heating is active, cooling is unconditionally forbidden
  if (wantHeater) {
    wantCooler = false;
    holdCoolingPulseActive = false;
  }

  // All outputs route through anti-chatter safe drivers (enforcing minimum dwell times)
  setHeaterRelay(wantHeater);
  setCoolingRelay(wantCooler);
}





