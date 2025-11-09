#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//defining OLED display parameters
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define flowSensorPin 3         // Digital pin for flow sensor (must be interrupt-capable pin)
#define temperatureSensorPin A0 // Analog pin for temperature sensor
#define pressureSensorPin A1    // Analog pin for pressure sensor

volatile byte pulseCount;
unsigned long oldTime = 0;
unsigned int flowMilliLitres = 0;
unsigned long totalMilliLitres = 0;
float calibrationFactor = 7.5; // Calibration factor for the flow sensor

float pressure = 0.0;
float temperatureC = 0.0;

int position[2] = {0,0};
int* const positionPointer = position; // Position to display text on OLED 

void pulseCounter();
float measureFlow();
float measurePressure();
float measureTemperature();

void initializeDisplay();
void displayText(String text, int* const positionPointer, int textSize = 1);

void setup()
{
  Serial.begin(9600);
  //initializing the OLED display
  //initializeDisplay();
 
  //displayText("Hello World!",position, 8);
  
  //initializing the pressure sensor
  attachInterrupt(digitalPinToInterrupt(flowSensorPin), pulseCounter, RISING);
}

void loop()
{
  if(millis() - oldTime > 1000) // Update every second
  {
    flowMilliLitres = measureFlow(); //flow in mL/sec
    totalMilliLitres += flowMilliLitres;
  }
  //pressure = measurePressure(); //pressure in kPas
  //temperatureC = measureTemperature(); //temperature in Celsius
  Serial.println("Flow: " + String(flowMilliLitres) + " mL/sec");
  delay(1000);
}

void pulseCounter()
{
  pulseCount++;
}


float measureFlow()
{
  
    oldTime = millis();
    detachInterrupt(digitalPinToInterrupt(3));
    float flowRate = ((1000.0 / (millis() - oldTime)) * pulseCount) / calibrationFactor; // Calculate flow rate in L/min
    float flowMilliLitres = (flowRate / 60) * 1000; // Convert to mL/sec
    pulseCount = 0;                           // Reset pulse counter
    attachInterrupt(digitalPinToInterrupt(3), pulseCounter, RISING);
    return flowMilliLitres;
}

float measurePressure()
{
  int sensorValue = analogRead(pressureSensorPin);
  float voltage = sensorValue * (5.0 / 1023.0); // Convert ADC value to voltage
  float pressure = (voltage - 0.5) * 100.0;     // Convert voltage to pressure in kPa (example conversion)
  return pressure;
}

float measureTemperature()
{
  int sensorValue = analogRead(temperatureSensorPin);
  float voltage = sensorValue * (5.0 / 1023.0); // Convert ADC value to voltage
  float temperatureC = (voltage - 0.5) * 100.0; // Convert voltage to temperature in Celsius (example conversion)
  return temperatureC;
}

void initializeDisplay()
{
 if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("OLED screen initialization failed"));
    for(;;); // Don't proceed, loop forever
  }
  Serial.println(F("OLED screen initialized"));
  display.display();
  delay(2000); // Pause for 2 seconds
  display.clearDisplay();
}

void displayText(String text, int* const positionPointer, int textSize)
{
  display.clearDisplay();
  display.setTextSize(textSize);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(*positionPointer,*(positionPointer + 1));     // Start at top-left corner
  display.println(text);
  display.display();
}
