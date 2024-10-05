#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_BMP085_U.h>

// Vibration Sensor
const int vibrationPin = 2; 

// LM35 Temperature Sensor
const int tempPin = A0; 

// Create an ADXL345 object
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// Create an object for BMP180
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

void setup() {
  // Start serial communication
  Serial.begin(9600); 

  // Set up vibration sensor
  pinMode(vibrationPin, INPUT);

  // Initialize the ADXL345 sensor
  if (!accel.begin()) {
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while (1);
  }
  accel.setRange(ADXL345_RANGE_16_G);
  
  // Initialize the BMP180
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085/BMP180 sensor, check wiring!");
    while (1);
  }
}

void loop() {
  // Read vibration sensor
  int vibrationValue = digitalRead(vibrationPin);
  if (vibrationValue == HIGH) {
    Serial.println("Vibration Detected!");
  } else {
    Serial.println("No Vibration.");
  }

  // Read temperature from LM35
  int sensorValue = analogRead(tempPin);
  float temperatureLM35 = (sensorValue * 5.0 * 100.0) / 1024;
  Serial.print("LM35 Temperature: ");
  Serial.print(temperatureLM35);
  Serial.println(" °C");

  // Read accelerometer data
  sensors_event_t event;
  accel.getEvent(&event);
  Serial.print("Accelerometer - X: "); Serial.print(event.acceleration.x); Serial.print(" m/s^2 ");
  Serial.print("| Y: "); Serial.print(event.acceleration.y); Serial.print(" m/s^2 ");
  Serial.print("| Z: "); Serial.print(event.acceleration.z); Serial.println(" m/s^2 ");
  
  // Read pressure and temperature from BMP180
  sensors_event_t pressureEvent;
  bmp.getEvent(&pressureEvent);
  if (pressureEvent.pressure) {
    Serial.print("Pressure: ");
    Serial.print(pressureEvent.pressure);
    Serial.println(" hPa");

    float temperatureBMP;
    bmp.getTemperature(&temperatureBMP);
    Serial.print("BMP180 Temperature: ");
    Serial.print(temperatureBMP);
    Serial.println(" °C");
  }

  // Wait for a second before the next reading
  delay(1000);
}
