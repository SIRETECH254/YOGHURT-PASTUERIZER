

#ifndef CONTROL_HEATER_H
#define CONTROL_HEATER_H

// Core hardware initialization
void initHeater();

// Anti-chatter safe relay drivers with minimum dwell time protection
void setHeaterRelay(bool on, bool forceImmediate = false);
void setCoolingRelay(bool on, bool forceImmediate = false);
void setAgitatorRelay(bool on, bool forceImmediate = false);

// High-level actuator control
void turnOnAgitator();
void turnOffHeater(bool forceImmediate = false);
void turnOnCoolingValve();
void turnOffAllActuators(bool forceImmediate = true);

// Resets internal actuator demand and timing state between phases
void resetActuatorControl();

// Phase-specific control routines (chatter-free)
void runPIDControl(float jacketTemp, float targetTemp);
void runCoolingControl(float productTemp, float targetTemp);
void runHoldingControl(float productTemp, float jacketTemp, float targetTemp);

#endif

