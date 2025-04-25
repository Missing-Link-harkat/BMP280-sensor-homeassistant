#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// WiFi Configuration
const char* ssid = "";               // Write your WiFi SSID here (same where home assisntant is connected)
const char* password = "AADN7UQT5W8JY";       // Write your WiFi Password here

// MQTT Configuration
const char* mqtt_server = "192.168.100.10";   // IP address of your MQTT broker (same as Home Assistant)

WiFiClient espClient;
PubSubClient client(espClient);

// BMP280 Sensor Setup
Adafruit_BMP280 bmp; // I2C interface

#define SDA_PIN 0  // ESP32 I2C SDA
#define SCL_PIN 2  // ESP32 I2C SCL
#define SEALEVELPRESSURE_HPA (1013.25)  // Sea level pressure at your location

// Timing for MQTT Messages
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (100)
char msg[MSG_BUFFER_SIZE];

// Connect to Wi-Fi
void setup_wifi() {
  Serial.print("Connecting to WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  // Wait until connected
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nConnected to WiFi!");     // Connected
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());             // IP address
}

// MQTT Reconnection
void reconnect() {
  // Loop until we are connected to MQTT
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    
    //Generate unique client ID
    String clientId = "ESP32Client-";         // You can change ID to your own
    clientId += String(random(0xffff), HEX);

    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println(" Connected!");
    } else {
      Serial.print(" Failed, rc=");
      Serial.print(client.state());
      Serial.println(" Trying again in 5 seconds...");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Start Wi-Fi
  setup_wifi();
  
  // Configure MQTT client
  client.setServer(mqtt_server, 1883);
  
  // Start I2C for BMP280
  Wire.begin(SDA_PIN, SCL_PIN);

  // Initialize BMP280
  if (!bmp.begin(0x76)) { // Try 0x77 if 0x76 doesn't work
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);  // Stop execution
  }
}

void loop() {
  // Reconnect to MQTT if needed
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Read sensor values
  float t = bmp.readTemperature();                  // Celsius
  float p = bmp.readPressure() / 100.0F;            // Convert to hPa
  float a = bmp.readAltitude(SEALEVELPRESSURE_HPA);

  // Print to Serial Monitor
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.println(" °C");

  Serial.print("Pressure: ");
  Serial.print(p);
  Serial.println(" hPa");

  Serial.print("Altitude: ");
  Serial.print(a);
  Serial.println(" m");

  Serial.println("-----------------------------");

  // Send MQTT message every 5 seconds
  unsigned long now = millis();
  if (now - lastMsg > 5000) {
    lastMsg = now;
    
    // Format sensor data as JSON
    snprintf(msg, MSG_BUFFER_SIZE, "{\"temperature\": %.2f, \"pressure\": %.2f, \"altitude\": %.2f}", t, p, a);
    
    // Publish to topic
    Serial.print("Publishing message: ");
    Serial.println(msg);
    client.publish("BMP280/01", msg);   // You can change this topic
  }

  delay(5000);  // Wait before next reading
}
