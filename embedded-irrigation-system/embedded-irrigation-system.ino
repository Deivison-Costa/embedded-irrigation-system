#include <Arduino.h>
#include <ModbusMaster.h>
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <DHT.h>
#include <DHT_U.h>
#include <BH1750.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <TinyGPSPlus.h>

#define MAX485_DE      17
#define MAX485_RE_NEG  16
#define MAX485_RX      17
#define MAX485_TX      16
#define RXD0 18
#define TXD0 19
#define DHTPIN 4
#define DHTTYPE DHT22
#define LM393_PIN 13

const char* ssid = "";
const char* password = "";

const char* mqtt_server = "";
const int mqtt_port = 8883;
const char* mqtt_username = "";
const char* mqtt_password = "";

const char* ca_cert = \
"-----BEGIN CERTIFICATE-----\n" \

"-----END CERTIFICATE-----\n";

ModbusMaster node;
DHT dht(DHTPIN, DHTTYPE);
BH1750 lightMeter;
Adafruit_BMP280 bmp;
WiFiClientSecure espClient;
PubSubClient client(espClient);
TinyGPSPlus gps;

void preTransmission() {
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission() {
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}

void setup_wifi() {
  delay(10);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}

void reconnect() {
  while (!client.connected()) {
    if (client.connect("ESP32Client", mqtt_username, mqtt_password)) {
    } else {
      delay(5000);
    }
  }
}

void setup() {
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

  Serial.begin(115200);
  Serial2.begin(4800, SERIAL_8N1, MAX485_RX, MAX485_TX);
  Serial1.begin(9600, SERIAL_8N1, RXD0, TXD0);
  
  node.begin(1, Serial2);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
  
  dht.begin();
  Wire.begin();
  bmp.begin(0x76);
  lightMeter.begin();
  pinMode(LM393_PIN, INPUT);

  setup_wifi();

  espClient.setCACert(ca_cert);
  
  client.setServer(mqtt_server, mqtt_port);

  Serial.println("Inicialização completa...");
}

void publish_error(String sensor, String message) {
  String error_message = "Error in sensor " + sensor + ": " + message;
  client.publish("sensors/errors", error_message.c_str());
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Anemometer

  uint8_t result;
  uint16_t data[1];

  result = node.readHoldingRegisters(0x0000, 1);

  if (result == node.ku8MBSuccess) {
    float windSpeed = node.getResponseBuffer(0x00) / 10.0;
    Serial.print("Wind Speed: ");
    Serial.println(windSpeed);

    if (windSpeed < 0 || windSpeed > 100) {
      publish_error("windSpeed", "Out of expected range: " + String(windSpeed));
    } else {
      client.publish("sensors/windSpeed", String(windSpeed).c_str());
    }
  } else {
    Serial.println("Error: Modbus read failure for wind speed");
    Serial.println(result, HEX);
    publish_error("windSpeed", "Modbus read failure");
  }

  // DHT22
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  
  Serial.print("Temperature: ");
  Serial.println(temperature);
  Serial.print("Humidity: ");
  Serial.println(humidity);

  if (isnan(temperature)) {
    publish_error("temperature", "Failed to read");
  } else if (temperature < -40 || temperature > 80) {
    publish_error("temperature", "Out of expected range: " + String(temperature));
  } else {
    client.publish("sensors/temperature", String(temperature).c_str());
  }

  if (isnan(humidity)) {
    publish_error("humidity", "Failed to read");
  } else if (humidity < 0 || humidity > 100) {
    publish_error("humidity", "Out of expected range: " + String(humidity));
  } else {
    client.publish("sensors/humidity", String(humidity).c_str());
  }

  // BH1750
  uint16_t lux = lightMeter.readLightLevel();
  Serial.print("Luminosity: ");
  Serial.println(lux);

  if (lux < 0 || lux > 65535) {
    publish_error("luminosity", "Out of expected range: " + String(lux));
  } else {
    client.publish("sensors/luminosity", String(lux).c_str());
  }

  // BMP280
  float pressure = bmp.readPressure() / 100.0F;
  float bmpTemperature = bmp.readTemperature();
  
  Serial.print("Pressure: ");
  Serial.println(pressure);
  Serial.print("BMP Temperature: ");
  Serial.println(bmpTemperature);

  if (pressure < 300 || pressure > 1100) {
    publish_error("pressure", "Out of expected range: " + String(pressure));
  } else {
    client.publish("sensors/pressure", String(pressure).c_str());
  }

  // LM393
  int lm393_value = digitalRead(LM393_PIN);
  Serial.print("LM393 Value: ");
  Serial.println(lm393_value);
  client.publish("sensors/lm393", String(lm393_value).c_str());

  // GPS
  while (Serial1.available()) {
    gps.encode(Serial1.read());
  }

  if (gps.location.isValid()) {
    double latitude = gps.location.lat();
    double longitude = gps.location.lng();
    double altitude = gps.altitude.meters();

    Serial.print("Latitude: ");
    Serial.println(latitude);
    Serial.print("Longitude: ");
    Serial.println(longitude);
    Serial.print("Altitude: ");
    Serial.println(altitude);

    client.publish("sensors/latitude", String(latitude, 6).c_str());
    client.publish("sensors/longitude", String(longitude, 6).c_str());
    client.publish("sensors/altitude", String(altitude, 2).c_str());
  } else {
    publish_error("GPS", "Location data is invalid");
  }

  delay(2000);
}