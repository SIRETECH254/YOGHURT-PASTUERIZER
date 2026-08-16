

#ifndef CONTROL_HEATER_H
#define CONTROL_HEATER_H

void initHeater();
void turnOnAgitator();
void runPIDControl(float currentTemp, float targetTemp);
void runCoolingControl(float currentTemp, float targetTemp);
void runHoldingControl(float jacketTemp, float productTemp, float targetTemp);
void turnOffHeater();
void turnOnCoolingValve();
void turnOffAllActuators();

#endif
