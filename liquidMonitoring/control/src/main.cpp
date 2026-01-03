#include <Arduino.h>
#define PRESSURE_SENSOR_PIN A1
#define FLOWRATE_SENSOR_PIN 2

struct SensorData
{
  char name[20];
  float value;
  char unit[10];
  float calibrationFactor;
  float (*measureFunction)(float calibrationFactor);
};

// Global variables
volatile byte pulseCount = 0;
unsigned long oldTime = 0;
unsigned long totalMilliLitres = 0;

// Sensor calibration factors
float flowRateCalibrationFactor = 4.5;
float pressureCalibrationFactor = 0.3;

// Measurement functions prototypes
// Flowrate and volume
float measureFlowRate(float calibrationFactor);
float measureFlowVolume(float calibrationFactor);
float measureTotalVolume(float calibrationFactor);
// Pressure
float measurePressure(float calibrationFactor);

void updateParameter(SensorData &parameterSet);

// Logging functions prototypes
void logMeasuredParameter(SensorData &parameterSet);

// Interrupt Service Routine
void pulseCounter();

SensorData flowRate = {"Flow Rate", 0.0, "L/min", flowRateCalibrationFactor, measureFlowRate};
SensorData flowVolume = {"Flow Volume", 0.0, "mL", 0.0, measureFlowVolume};
SensorData totalVolume = {"Total Volume", 0.0, "L/min", 0.0, measureTotalVolume};
SensorData pressure = {"Pressure", 0.0, "MPa", pressureCalibrationFactor, measurePressure};

void setup()
{
  // Initialize a serial connection for reporting values to the host
  Serial.begin(9600);
  // Configure sensor pins
  pinMode(FLOWRATE_SENSOR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(FLOWRATE_SENSOR_PIN), pulseCounter, FALLING);
  pinMode(PRESSURE_SENSOR_PIN, INPUT);
}

void loop()
{
  updateParameter(pressure);
  updateParameter(flowRate);
  updateParameter(flowVolume);
  updateParameter(totalVolume);
  Serial.println("-----------------------");
  logMeasuredParameter(pressure);
  logMeasuredParameter(flowRate);
  logMeasuredParameter(flowVolume);
  logMeasuredParameter(totalVolume);

  delay(3000);
}

float measureFlowRate(float calibrationFactor)
{
  detachInterrupt(digitalPinToInterrupt(FLOWRATE_SENSOR_PIN));
  float flowRate = 0.0;
  if (millis() - oldTime > 1000)
  {
    flowRate = ((1000.0 / (millis() - oldTime)) * pulseCount) / calibrationFactor;
    attachInterrupt(digitalPinToInterrupt(FLOWRATE_SENSOR_PIN), pulseCounter, FALLING);
    pulseCount = 0;
    oldTime = millis();
  }
  return flowRate;
}

float measureFlowVolume(float calibrationFactor)
{
  float flowMilliLitres = (flowRate.value / 60) * 1000;
  return flowMilliLitres;
}

float measureTotalVolume(float calibrationFactor)
{
  totalMilliLitres += flowVolume.value;
  return totalMilliLitres;
}

float measurePressure(float calibrationFactor)
{
  float sensorValue = analogRead(PRESSURE_SENSOR_PIN);
  float voltage = sensorValue * (5.0 / 1023.0);
  float pressureMPa = (voltage - 0.5) * (10.0 / 4.0);
  return pressureMPa;
}

void updateParameter(SensorData &parameterSet)
{
  parameterSet.value = parameterSet.measureFunction(parameterSet.calibrationFactor);
}

void logMeasuredParameter(SensorData &parameterSet)
{
  Serial.print(parameterSet.name);
  Serial.print(": ");
  Serial.print(parameterSet.value);
  Serial.print(" ");
  Serial.println(parameterSet.unit);
}

// Interrupt Service Routine
void pulseCounter()
{
  pulseCount++;
}
