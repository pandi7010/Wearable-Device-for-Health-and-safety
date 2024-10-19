#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include "MAX30100_PulseOximeter.h"
#define REPORTING_PERIOD_MS     1000
// Define pins
const int heartbeatPin = A0;
const int tempPin = A1;
const int redButtonPin = 4;
const int greenButtonPin = 5;
const int blackButtonPin = 6;
const int buzzerPin = 9;
// GSM Module setup
SoftwareSerial gsmSerial(7, 8);  // GSM RX, TX
TinyGPSPlus gps;
SoftwareSerial gpsSerial(2, 3);  // GPS RX, TX
PulseOximeter pox;
uint32_t tsLastReport = 0;
// Setup for Pulse Oximeter
void onBeatDetected()
{
    Serial.println("Beat Detected!");
}
void setup() {
    Serial.begin(9600);
    // Pin setups
    pinMode(redButtonPin, INPUT_PULLUP);
    pinMode(greenButtonPin, INPUT_PULLUP);
    pinMode(blackButtonPin, INPUT_PULLUP);
    pinMode(buzzerPin, OUTPUT);
    // Setup GSM module
    gsmSerial.begin(9600);
    gpsSerial.begin(9600);
    // Initialize MAX30100 Pulse Oximeter
    if (!pox.begin()) {
        Serial.println("FAILED to initialize PulseOximeter");
        for(;;);
    }
    pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
    pox.setOnBeatDetectedCallback(onBeatDetected);
}
void loop() {
    // Reading Heartbeat sensor
    int heartRateValue = analogRead(heartbeatPin);
    Serial.print("Heartbeat Sensor Value: ");
   Serial.println(heartRateValue);
    // Reading Temperature Sensor
    int tempValue = analogRead(tempPin);
    float temperature = (tempValue * 5.0 * 100.0) / 1024;
    Serial.print("Temperature: ");
    Serial.println(temperature);
    // Reading Pulse Oximeter (Oxygen Level)
    pox.update();
    if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
        Serial.print("Heart rate:");
        Serial.print(pox.getHeartRate());
        Serial.print(" bpm / SpO2:");
        Serial.print(pox.getSpO2());
        Serial.println(" %");
        tsLastReport = millis();
    }
    // Reading GPS data
    while (gpsSerial.available() > 0)
        gps.encode(gpsSerial.read());
    if (gps.location.isValid()) {
        Serial.print("Latitude: ");
        Serial.println(gps.location.lat(), 6);
        Serial.print("Longitude: ");
        Serial.println(gps.location.lng(), 6);
    }
    // Checking Red Button (Safety)
    if (digitalRead(redButtonPin) == LOW) {
        sendSMS("Emergency! Location: ", gps.location.lat(), gps.location.lng(), "Police");
    }
    // Checking Green Button (Health Emergency)
    if (digitalRead(greenButtonPin) == LOW) {
        sendSMS("Health Emergency! Location: ", gps.location.lat(), gps.location.lng(), "Hospital");
    }
    // Checking Black Button (Buzzer Control)
    if (digitalRead(blackButtonPin) == LOW) {
        digitalWrite(buzzerPin, LOW);  // Turn off buzzer
    }
}
void sendSMS(String message, float lat, float lng, String authority) {
    gsmSerial.print("AT+CMGF=1\r");  // Set SMS to text mode
    delay(100);
    gsmSerial.print("AT+CMGS=\"+911234567890\"\r");  // Replace with the correct number
    delay(100);
    gsmSerial.print(message);
    gsmSerial.print("Lat: ");
    gsmSerial.print(lat, 6);
    gsmSerial.print(", Lng: ");
    gsmSerial.print(lng, 6);
    gsmSerial.print(" | Nearest: ");
    gsmSerial.print(authority);
    gsmSerial.write(26);  // Ctrl+Z to send the message
    delay(1000);
}
