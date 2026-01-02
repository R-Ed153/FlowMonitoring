#include <Arduino.h>

int sensorPin = 2; // Pin connected to the flow sensor
float calibrationFactor = 4.5;
volatile byte pulseCount = 0;
float flowRate = 0.0;
unsigned int flowMilliLitres = 0;
unsigned long totalMilliLitres = 0;
unsigned long oldTime = 0;

int statusLed = 13; // Status LED pin

void pulseCounter();
void updateFlowParameters(float calibrationFactor, unsigned int &flowMilliLitres, unsigned long &totalMilliLitres);
void logFlowData();
float measureFlowRate(float calibrationFactor);

void setup()
{

  // Initialize a serial connection for reporting values to the host
  Serial.begin(9600);
  // Set up the status LED line as an output
  pinMode(statusLed, OUTPUT);
  digitalWrite(statusLed, HIGH); // We have an active-low LED attached
  pinMode(sensorPin, INPUT);
  // The Hall-effect sensor is connected to pin 2 which uses interrupt 0.
  // Configured to trigger on a FALLING state change (transition from HIGH
  // state to LOW state)
  attachInterrupt(digitalPinToInterrupt(sensorPin), pulseCounter, FALLING);
}

/**
 * Main program loop
 */
void loop()
{
  updateFlowParameters(calibrationFactor, flowMilliLitres, totalMilliLitres);
  logFlowData();
  delay(3000);
}



void updateFlowParameters(float calibrationFactor, unsigned int &flowMilliLitres, unsigned long &totalMilliLitres)
{
  Serial.println("Updating flow rate parameters...");
  if ((millis() - oldTime) >= 1000)
  {
    flowRate = measureFlowRate(calibrationFactor);
    flowMilliLitres = (flowRate / 60) * 1000;
    totalMilliLitres += flowMilliLitres;
    oldTime = millis();
  }
}

float measureFlowRate(float calibrationFactor)
{
  detachInterrupt(digitalPinToInterrupt(sensorPin));
  flowRate = ((1000.0 / (millis() - oldTime)) * pulseCount) / calibrationFactor;
  attachInterrupt(digitalPinToInterrupt(sensorPin), pulseCounter, FALLING);
  pulseCount = 0;
  return flowRate;
}

//Interrupt Service Routine
void pulseCounter()
{
  // Increment the pulse counter
  pulseCount++;
}


void logFlowData()
{
  Serial.print("Flow rate: ");
  Serial.print(flowRate);
  Serial.print(" L/min");
  Serial.print("\tTotal: ");
  Serial.print(totalMilliLitres);
  Serial.println(" mL");
}
