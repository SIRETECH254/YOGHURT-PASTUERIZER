

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

void turnOffCoolingValve() {
  digitalWrite(RELAY_COOLING_PIN, HIGH); // HIGH turns active-low cooling valve OFF
}

void turnOffAllActuators() {
  digitalWrite(RELAY_HEATER_PIN, HIGH);   
  digitalWrite(RELAY_AGITATOR_PIN, HIGH); 
  digitalWrite(RELAY_COOLING_PIN, HIGH);  
  digitalWrite(BUZZER_PIN, LOW);         
}

// -------------------------------------------------------------------------
// Jacket-maintenance control (thermodynamically guaranteed no overshoot):
// This function is now driven by the JACKET probe, not the product probe.
// The heater cycles to hold the jacket at exactly the phase target. Since
// heat only flows hot -> cold, the product cannot exceed the jacket temp
// no matter how long the phase runs -- overshoot is impossible by physics.
// The product asymptotes to jacket temp via heat transfer; phase completion
// is decided separately by the product probe reaching target.
//
// HEAT_HYSTERESIS exists to stop relay chatter: without it, normal probe
// noise right at the exact cutoff point causes `error` to bounce back and
// forth across the threshold, so the relay follows every bounce and clicks
// rapidly. With hysteresis, once the heater turns off at target, it won't
// turn back on until the jacket drops a further 1C below target.
// -------------------------------------------------------------------------
const float HEAT_CUTOFF_BAND = 0.0;  // Heater off exactly when jacket >= target
const float HEAT_HYSTERESIS = 2.0;   // Reactivate 2C below target -- comfortably above probe-noise floor

void runPIDControl(float currentTemp, float targetTemp) {
  static bool heaterOn = true; // Heating always starts well below target, so default to ON

  float error = targetTemp - currentTemp;

  if (heaterOn) {
    if (error <= HEAT_CUTOFF_BAND) {
      heaterOn = false; // Jacket at/above target: turn off until it drifts past the hysteresis margin
    }
  } else {
    if (error > HEAT_CUTOFF_BAND + HEAT_HYSTERESIS) {
      heaterOn = true; // Dropped back out past the hysteresis margin: allow it back on
    }
  }

  digitalWrite(RELAY_HEATER_PIN, heaterOn ? LOW : HIGH);
}

// -------------------------------------------------------------------------
// Mirrored jacket-maintenance for cooling:
// Driven by the JACKET probe. The valve cycles to hold the jacket at the
// cooling target. Product cannot fall below jacket temp (heat only flows
// hot -> cold), so undershoot is impossible. Same hysteresis logic and
// rationale as runPIDControl() above.
// -------------------------------------------------------------------------
const float COOL_CUTOFF_BAND = 0.0;  // Valve off exactly when jacket <= target
const float COOL_HYSTERESIS = 2.0;   // Reactivate 2C above target -- comfortably above probe-noise floor

void runCoolingControl(float currentTemp, float targetTemp) {
  static bool coolerOn = true; // Cooling always starts well above target, so default to ON

  float error = currentTemp - targetTemp; // positive while still above target

  if (coolerOn) {
    if (error <= COOL_CUTOFF_BAND) {
      coolerOn = false; // Jacket at/below target: turn off until it drifts past the hysteresis margin
    }
  } else {
    if (error > COOL_CUTOFF_BAND + COOL_HYSTERESIS) {
      coolerOn = true; // Dropped back out past the hysteresis margin: allow it back on
    }
  }

  digitalWrite(RELAY_COOLING_PIN, coolerOn ? LOW : HIGH);
}
