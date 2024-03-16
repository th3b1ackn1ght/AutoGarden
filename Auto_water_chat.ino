//Made by Parker Duhon 03-15-24

#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ArduinoJson.h>
#include <AutoPID.h>

double temperature, setpoint, output; // Declare global variables

#define POT_PIN A0         // Potentiometer pin for setpoint
#define TEMP_PROBE_PIN 9   // Temperature probe pin
#define TEMP_READ_DELAY 800 // Temperature read delay (milliseconds)
#define KP 0.12            // Proportional gain
#define KI 0.0003          // Integral gain
#define KD 0               // Derivative gain

OneWire oneWire(TEMP_PROBE_PIN);
DallasTemperature sensors(&oneWire);
AutoPID myPID(&temperature, &setpoint, &output, 0, 100, KP, KI, KD); // Initialize AutoPID object

unsigned long lastTempUpdate; // Track the last temperature update time

bool updateTemperature() {
    if (millis() - lastTempUpdate >= TEMP_READ_DELAY) {
        sensors.requestTemperatures();
        temperature = sensors.getTempCByIndex(0);
        lastTempUpdate = millis();
        return true;
    }
    return false;
}

// Moisture Sensor Calibration
const int dryValue = 500; // Adjust this value based on your soil conditions
const int wetValue = 200; // Adjust this value based on your soil conditions

// Solenoid Valve Control
const int solenoidValvePin = 3;
const int pulseWidth = 30;

// PID Controller
double Kp = 2.0, Ki = 0.5, Kd = 1.0; // PID tuning parameters (adjust as needed)

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
File dataFile;

void setup() {
    sensors.begin();
    while (!updateTemperature()) {} // Wait for the initial temperature update
    myPID.setBangBang(4); // Set the bang-bang range to 4 degrees
    myPID.setTimeStep(4000); // Set the PID update interval to 4 seconds

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

    // Set up the solenoid valve pin
    pinMode(solenoidValvePin, OUTPUT);
    digitalWrite(solenoidValvePin, LOW);
}

void loop() {
    unsigned long currentMicros = micros();
    int sensorValue = analogRead(sensorPin);

    if (updateTemperature()) {
        setpoint = map(analogRead(POT_PIN), 0, 1023, 0, 100); // Map potentiometer value to setpoint range
        myPID.run();
        // Control the solenoid valve based on PID output
        if (output > 0) {
            openSolenoidValve();
        } else {
            closeSolenoidValve();
        }
    }

    // Check for sensor read error
    if (sensorValue == 0) {
        Serial.println("Moisture sensor read error!");
        return;
    }

    // Calibrate the sensor value
    int percentageHumidity = map(sensorValue, dryValue, wetValue, 0, 100);

    // Update the PID controller input (not needed with AutoPID)
    myPID.run();

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

void logData(double temp, int humidity) {
    DateTime now = rtc.now();
    unsigned long currentMicros = micros();

    JsonDocument doc; // Allocate a temporary JSON document
    doc["timestamp"] = String(now.year()) + "-" + String(now.month()) + "-" + String(now.day()) + " " +
                       String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second()) + "." + String(currentMicros - lastMicros);
    doc["temperature"] = temp;
    doc["humidity"] = humidity;

    serializeJson(doc, Serial);
    Serial.println();

    dataFile.print("{");
    serializeJson(doc, dataFile);
    dataFile.println("},");
    dataFile.flush();

    lastMicros = currentMicros;
}
