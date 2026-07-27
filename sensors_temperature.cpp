



#include <Arduino.h>
#include "sensors_temperature.h"
#include "config_settings.h"

// Thermistor Math Constants (Standard 10k NTC parameters)
const float SERIES_RESISTOR = 10000.0;    // 10k fixed resistor
const float THERMISTOR_NOMINAL = 10000.0; // Resistance at 25 degrees C
const float TEMPERATURE_NOMINAL = 25.0;   // Nominal temperature reference
const float B_COEFFICIENT = 3950.0;       // Beta coefficient of standard probes

void initTemperatureSensor() {
  pinMode(ANALOG_HEAT_PIN, INPUT);
  pinMode(ANALOG_COOL_PIN, INPUT);
}

// Internal precision math translation loop with multi-sample filtering
float convertAnalogToThermistor(int pinNumber) {
  // Clear out any residual charge on ADC multiplexer
  analogRead(pinNumber);
  delay(5);
  
  // Take 5 consecutive samples to filter out transient industrial switching noise
  long rawAccumulator = 0;
  for (int i = 0; i < 5; i++) {
    rawAccumulator += analogRead(pinNumber);
    delay(2);
  }
  float rawADC = (float)rawAccumulator / 5.0;
  
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

float readHeatingTemperature() {
  float tempC = convertAnalogToThermistor(ANALOG_HEAT_PIN);
  if (tempC >= 999.0 || tempC < MIN_SAFE_TEMP) {
    return 999.0; 
  }
  return tempC + HEAT_PROBE_OFFSET; // Apply calibration offset
}

float readCoolingTemperature() {
  float tempC = convertAnalogToThermistor(ANALOG_COOL_PIN);
  if (tempC >= 999.0 || tempC < MIN_SAFE_TEMP) {
    return 999.0; 
  }
  return tempC + COOL_PROBE_OFFSET; // Apply calibration offset
}

void runSensorDiagnostics() {
  Serial.println("\n--- RUNNING THERMISTOR BUS DIAGNOSTICS ---");
  float testHeat = convertAnalogToThermistor(ANALOG_HEAT_PIN);
  float testCool = convertAnalogToThermistor(ANALOG_COOL_PIN);
  
  if (testHeat == 999.0) {
    Serial.println("HEATING JACKET PROBE [A0]: FAULT");
  } else {
    Serial.print("HEATING JACKET PROBE [A0]: "); Serial.print(testHeat + HEAT_PROBE_OFFSET, 1); Serial.println(" C");
  }
  
  if (testCool == 999.0) {
    Serial.println("PRODUCT COOLING PROBE [A1]: FAULT");
  } else {
    Serial.print("PRODUCT COOLING PROBE [A1]: "); Serial.print(testCool + COOL_PROBE_OFFSET, 1); Serial.println(" C");
  }
}
