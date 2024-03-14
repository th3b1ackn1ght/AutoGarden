#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ArduinoJson.h>

// Moisture Sensor Calibration
const int dryValue = 500; // Adjust this value based on your soil conditions
const int wetValue = 200; // Adjust this value based on your soil conditions

// Solenoid Valve Control
const int solenoidValvePin = 3;
const int pulseWidth = 30;

// PID Controller
double setpoint = 50.0; // Desired moisture level (adjust as needed)
double Kp = 2.0, Ki = 0.5, Kd = 1.0; // PID tuning parameters (adjust as needed)
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Time Tracking
unsigned long lastWateringTime = 0;
const unsigned long wateringInterval = 300000; // 5 minutes in milliseconds
unsigned long lastMicros = 0;

// Other Constants
const int chipSelect = 10;
const int sensorPin = A0;
#define ONE_WIRE_BUS 9

// Class Instances
RTC_PCF8523 rtc;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
File dataFile;

void setup() {
  // Open serial communications and wait for port to open
  Serial.begin(9600);
  pinMode(sensorPin, INPUT);

  // Initialize SD card
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card initialization failed!");
    while (1); // Wait forever if SD card initialization fails
  }

  // Open or create the data file
  dataFile = SD.open("datalog.json", FILE_WRITE);
  if (!dataFile) {
    Serial.println("Error opening datalog.json");
    while (1); // Wait forever if data file cannot be opened
  }

  // Initialize RTC
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1); // Wait forever if RTC initialization fails
  } else {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // Start up the temperature sensor
  sensors.begin();

  // Set up the solenoid valve pin
  pinMode(solenoidValvePin, OUTPUT);
  digitalWrite(solenoidValvePin, LOW);

  // Initialize the PID controller
  pid.SetMode(AUTOMATIC);
}

void loop() {
  unsigned long currentMicros = micros();
  int sensorValue = analogRead(sensorPin);

  // Check for sensor read error
  if (sensorValue == 0) {
    Serial.println("Moisture sensor read error!");
    return;
  }

  // Calibrate the sensor value
  int percentageHumidity = map(sensorValue, dryValue, wetValue, 0, 100);

  sensors.requestTemperatures();
  double temperature = sensors.getTempCByIndex(0);

  // Update the PID controller input
  input = percentageHumidity;
  pid.Compute();

  // Control the solenoid valve based on PID output
  if (output > 0) {
    openSolenoidValve();
  } else {
    closeSolenoidValve();
  }

  // Water at 6:00 AM and 6:00 PM for 3 minutes
  DateTime now = rtc.now();
  if ((now.hour() == 6 && now.minute() == 0) || (now.hour() == 18 && now.minute() == 0)) {
    waterForDuration(180000000); // Water for 3 minutes (180,000,000 microseconds)
  }

  // Log data to the JSON file
  logData(temperature, percentageHumidity);

  // Delay for the next reading
  delay(10);
}

void openSolenoidValve() {
  digitalWrite(solenoidValvePin, HIGH);
  delay(pulseWidth);
  digitalWrite(solenoidValvePin, LOW);
}

void closeSolenoidValve() {
  digitalWrite(solenoidValvePin, LOW);
  delay(pulseWidth);
  digitalWrite(solenoidValvePin, LOW);
}

void waterForDuration(unsigned long duration) {
  unsigned long startTime = micros();
  while (micros() - startTime < duration) {
    if (micros() - lastWateringTime >= wateringInterval) {
      openSolenoidValve();
      lastWateringTime = micros();
    }
  }
  closeSolenoidValve();
}

void logData(double temperature, int humidity) {
  DateTime now = rtc.now();
  unsigned long currentMicros = micros();

  StaticJsonDocument<200> doc;
  doc["timestamp"] = String(now.year()) + "-" + String(now.month()) + "-" + String(now.day()) + " " +
                     String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second()) + "." + String(currentMicros - lastMicros);
  doc["temperature"] = temperature;
  doc["humidity"] = humidity;

  serializeJson(doc, Serial);
  Serial.println();

  dataFile.print("{");
  serializeJson(doc, dataFile);
  dataFile.println("},");
  dataFile.flush();

  lastMicros = currentMicros;
