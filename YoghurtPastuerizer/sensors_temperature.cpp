



#include <Arduino.h>
#include "sensors_temperature.h"
#include "config_settings.h"

// Thermistor Math Constants (Standard 10k NTC parameters)
const float SERIES_RESISTOR = 10000.0;    // 10k fixed resistor
const float THERMISTOR_NOMINAL = 10000.0; // Resistance at 25 degrees C
const float TEMPERATURE_NOMINAL = 25.0;   // Nominal temperature reference
const float B_COEFFICIENT = 3950.0;       // Beta coefficient of standard probes

// Exponential Moving Average (EMA) filter coefficient (0.25 provides fast response without jitter)
const float EMA_ALPHA = 0.25f;

static float filteredHeatTemp = -999.0;
static float filteredCoolTemp = -999.0;
static int consecutiveHeatFaults = 0;
static int consecutiveCoolFaults = 0;

void initTemperatureSensor() {
  pinMode(ANALOG_HEAT_PIN, INPUT);
  pinMode(ANALOG_COOL_PIN, INPUT);
  resetSensorFilters();
}

void resetSensorFilters() {
  filteredHeatTemp = -999.0;
  filteredCoolTemp = -999.0;
  consecutiveHeatFaults = 0;
  consecutiveCoolFaults = 0;
}

// Internal precision math translation loop with multi-sample filtering & outlier rejection
float convertAnalogToThermistor(int pinNumber) {
  // Clear out any residual charge on ADC multiplexer
  analogRead(pinNumber);
  delay(2);
  
  // Take 5 consecutive samples
  int samples[5];
  for (int i = 0; i < 5; i++) {
    samples[i] = analogRead(pinNumber);
    delay(2);
  }
  
  // Sort 5 samples to find median and reject min/max outliers (transient EMI spikes)
  for (int i = 1; i < 5; i++) {
    int key = samples[i];
    int j = i - 1;
    while (j >= 0 && samples[j] > key) {
      samples[j + 1] = samples[j];
      j = j - 1;
    }
    samples[j + 1] = key;
  }
  
  // Average the middle 3 samples (discards lowest and highest spike)
  float rawADC = (float)(samples[1] + samples[2] + samples[3]) / 3.0;
  
  // Safety limits: Open-circuit or dead-shorted line parameters
  if (rawADC <= 5.0 || rawADC >= 1018.0) {
    return 999.0; 
  }

  // Resistor-On-Top Calculation Math
  float resistance = SERIES_RESISTOR * ((1023.0 / rawADC) - 1.0);
  
  // Steinhart-Hart Equation Calculation
  float steinhart;
  steinhart = resistance / THERMISTOR_NOMINAL;     
  steinhart = log(steinhart);                      
  steinhart /= B_COEFFICIENT;                      
  steinhart += 1.0 / (TEMPERATURE_NOMINAL + 273.15); 
  steinhart = 1.0 / steinhart;                     
  steinhart -= 273.15;                             
  
  return steinhart;
}

float readRawHeatingTemperature() {
  float tempC = convertAnalogToThermistor(ANALOG_HEAT_PIN);
  if (tempC >= 999.0 || tempC < MIN_SAFE_TEMP) {
    return 999.0; 
  }
  return tempC + HEAT_PROBE_OFFSET;
}

float readRawCoolingTemperature() {
  float tempC = convertAnalogToThermistor(ANALOG_COOL_PIN);
  if (tempC >= 999.0 || tempC < MIN_SAFE_TEMP) {
    return 999.0; 
  }
  return tempC + COOL_PROBE_OFFSET;
}

float readHeatingTemperature() {
  float rawTemp = readRawHeatingTemperature();

  if (rawTemp >= 999.0) {
    consecutiveHeatFaults++;
    // If sustained disconnection (10 consecutive cycles = approx 1s), return 999.0 to trip safety
    if (consecutiveHeatFaults >= 10 || filteredHeatTemp < -100.0) {
      return 999.0;
    }
    // Single-frame EMI noise spike: return last known valid filtered temperature to prevent actuator chatter
    return filteredHeatTemp;
  }

  consecutiveHeatFaults = 0;
  if (filteredHeatTemp < -100.0) {
    filteredHeatTemp = rawTemp; // Seed filter on first valid reading
  } else {
    filteredHeatTemp = (EMA_ALPHA * rawTemp) + ((1.0f - EMA_ALPHA) * filteredHeatTemp);
  }

  return filteredHeatTemp;
}

float readCoolingTemperature() {
  float rawTemp = readRawCoolingTemperature();

  if (rawTemp >= 999.0) {
    consecutiveCoolFaults++;
    // If sustained disconnection (10 consecutive cycles = approx 1s), return 999.0 to trip safety
    if (consecutiveCoolFaults >= 10 || filteredCoolTemp < -100.0) {
      return 999.0;
    }
    // Single-frame EMI noise spike: return last known valid filtered temperature to prevent actuator chatter
    return filteredCoolTemp;
  }

  consecutiveCoolFaults = 0;
  if (filteredCoolTemp < -100.0) {
    filteredCoolTemp = rawTemp; // Seed filter on first valid reading
  } else {
    filteredCoolTemp = (EMA_ALPHA * rawTemp) + ((1.0f - EMA_ALPHA) * filteredCoolTemp);
  }

  return filteredCoolTemp;
}

void runSensorDiagnostics() {
  Serial.println("\n--- RUNNING THERMISTOR BUS DIAGNOSTICS ---");
  float testHeat = readRawHeatingTemperature();
  float testCool = readRawCoolingTemperature();
  
  if (testHeat == 999.0) {
    Serial.println("HEATING JACKET PROBE [A1]: FAULT");
  } else {
    Serial.print("HEATING JACKET PROBE [A1]: "); Serial.print(testHeat, 1); Serial.println(" C");
  }
  
  if (testCool == 999.0) {
    Serial.println("PRODUCT COOLING PROBE [A0]: FAULT");
  } else {
    Serial.print("PRODUCT COOLING PROBE [A0]: "); Serial.print(testCool, 1); Serial.println(" C");
  }
}

