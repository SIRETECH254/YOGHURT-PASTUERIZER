

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
// Thermal-lag overshoot fix (values tuned from bench testing):
// The jacket heats faster than the product probe can report, so cutting
// the heater at full power right at the setpoint leaves residual heat in
// the jacket/piping that keeps transferring into the product after cutoff
// (this was showing up as ~3-5C overshoot, e.g. target 85C -> actual 88-90C).
//
// Fix: run full power until 10C before target, then cut the heater
// completely and let residual jacket heat carry the product the rest of
// the way to target.
//
// HEAT_HYSTERESIS exists to stop relay chatter: without it, normal probe
// noise right at the exact cutoff point causes `error` to bounce back and
// forth across the threshold, so the relay follows every bounce and clicks
// rapidly. With hysteresis, once the heater turns off at the cutoff, it
// won't turn back on until the temp drops a further 1C below that point --
// a small amount of noise can no longer re-trigger it.
// -------------------------------------------------------------------------
const float HEAT_CUTOFF_BAND = 15.0; // Heater goes full OFF this many degrees before target
const float HEAT_HYSTERESIS = 1.0;   // Dead zone below the cutoff before the heater is allowed back on

void runPIDControl(float currentTemp, float targetTemp) {
  static bool heaterOn = true; // Heating always starts well below target, so default to ON

  float error = targetTemp - currentTemp;

  if (heaterOn) {
    if (error <= HEAT_CUTOFF_BAND) {
      heaterOn = false; // Crossed into the cutoff band: turn off, coast on residual jacket heat
    }
  } else {
    if (error > HEAT_CUTOFF_BAND + HEAT_HYSTERESIS) {
      heaterOn = true; // Dropped back out past the hysteresis margin: allow it back on
    }
  }

  digitalWrite(RELAY_HEATER_PIN, heaterOn ? LOW : HIGH);
}

// -------------------------------------------------------------------------
// Same fix, mirrored for cooling (values tuned from bench testing):
// The cooling jacket pulls heat out faster than the product probe reports
// it, so running the valve wide open right up to the setpoint leaves the
// jacket/piping colder than the product, which keeps pulling temperature
// down after cutoff -> undershoot below target.
//
// Fix: run full open until 7C before target, then cut the valve completely
// and let residual jacket cold carry the product the rest of the way to
// target. Same hysteresis logic as runPIDControl() above, for the same
// chatter-prevention reason.
// -------------------------------------------------------------------------
const float COOL_CUTOFF_BAND = 10.0; // Cooling valve goes full OFF this many degrees before target
const float COOL_HYSTERESIS = 1.0;   // Dead zone below the cutoff before the valve is allowed back on

void runCoolingControl(float currentTemp, float targetTemp) {
  static bool coolerOn = true; // Cooling always starts well above target, so default to ON

  float error = currentTemp - targetTemp; // positive while still above target

  if (coolerOn) {
    if (error <= COOL_CUTOFF_BAND) {
      coolerOn = false; // Crossed into the cutoff band: turn off, coast on residual jacket cold
    }
  } else {
    if (error > COOL_CUTOFF_BAND + COOL_HYSTERESIS) {
      coolerOn = true; // Dropped back out past the hysteresis margin: allow it back on
    }
  }

  digitalWrite(RELAY_COOLING_PIN, coolerOn ? LOW : HIGH);
}
