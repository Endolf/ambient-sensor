#define MQTT_KEEPALIVE 5

#include <WiFiNINA.h>
#include <RTCZero.h>
#include <DHT.h>
#include <PubSubClient.h>
#include <avr/dtostrf.h>
#include <stdlib.h>

#include "arduino_secrets.h"

#define DHTPIN 7     // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino
RTCZero rtc;
WiFiSSLClient wifiClient;
PubSubClient mqttClient(wifiClient);

const String deviceId = "testambientsensor";

unsigned long lastBasicLoopTime = 0, lastDhtLoopTime = 0;
const int basicLoopTime = 10000, dhtLoopTime = 10000;

struct Data {
  float humidity;
  float temperature;
  float batteryVoltage;
  float rssi;
};

Data data;

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);

  rtc.begin();
  dht.begin();

  mqttClient.setServer("computerbooth.tinamous.com", 8883);
  mqttClient.setCallback(messageReceived);

  checkAndConnectToWifi();
}

void loop() {
  unsigned long currentLoopTime = millis();

  digitalWrite(LED_BUILTIN, (WiFi.status() == WL_CONNECTED) && mqttClient.connected());
  if ((currentLoopTime - lastBasicLoopTime) >= basicLoopTime || currentLoopTime < lastBasicLoopTime) {
    checkAndConnectToWifi();
    printDate();
    printTime();
    printBatteryVoltage(&data);
    printTempAndHumdity(&data);
    data.rssi = WiFi.RSSI();
    sendData(data);
    lastBasicLoopTime = (currentLoopTime / basicLoopTime) * basicLoopTime;
  }
  mqttClient.loop();
}

void printTime()
{
  print2digits(rtc.getHours());
  Serial.print(":");
  print2digits(rtc.getMinutes());
  Serial.print(":");
  print2digits(rtc.getSeconds());
  Serial.println(" UTC");
}

void printDate()
{
  Serial.print(rtc.getDay());
  Serial.print("/");
  Serial.print(rtc.getMonth());
  Serial.print("/");
  Serial.print(rtc.getYear());

  Serial.print(" ");
}

void printWiFiStatus() {
  Serial.print("Firmware version: ");
  Serial.println(WiFi.firmwareVersion());
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void print2digits(int number) {
  if (number < 10) {
    Serial.print("0");
  }
  Serial.print(number);
}

void checkAndConnectToWifi() {
  int status = WiFi.status();

  // check if the WiFi module works
  if (status == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }

  // attempt to connect to WiFi network:
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    Serial.flush();
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    int numberOfTries = 0, maxTries = 10;
    while ( (status != WL_CONNECTED) && (numberOfTries < maxTries)) {
      delay(1000);
      status = WiFi.status();
      numberOfTries++;
    }
    // you're connected now, so print out the status:
    printWiFiStatus();

    unsigned long epoch;
    numberOfTries = 0;
    do {
      Serial.println("Attempting to get NTP time");
      epoch = WiFi.getTime();
      numberOfTries++;
      delay(1000);
    }
    while ((epoch == 0) && (numberOfTries < maxTries));

    if (numberOfTries >= maxTries) {
      Serial.print("NTP unreachable!!");
      while (1);
    }
    else {
      Serial.print("Epoch received: ");
      Serial.println(epoch);
      rtc.setEpoch(epoch);

      Serial.println();
    }
  }

  if (!mqttClient.connected()) {
    Serial.print("MQTT not connected: ");
    Serial.println(mqttClient.state());
    Serial.println("Connecting to mqtt");
    mqttClient.disconnect();
    if (!mqttClient.connect(deviceId.c_str(), mqttUser, mqttPass)) {
      Serial.print("Failed to connect: ");
      Serial.println(mqttClient.state());
    }
  }
}

void printBatteryVoltage(Data* data) {
  // read the input on analog pin 0:
  int sensorValue = analogRead(ADC_BATTERY);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 4.3V):
  float voltage = sensorValue * (4.3 / 1023.0);
  // print out the value you read:
  Serial.print(voltage);
  Serial.println("V");
  data->batteryVoltage = voltage;
}

void printTempAndHumdity(Data* data) {
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.print(" %, Temp: ");
  Serial.print(temperature);
  Serial.println(" Celsius");
  data->humidity = humidity;
  data->temperature = temperature;
}

void messageReceived(char* topic, byte* payload, unsigned int length) {
  Serial.print("incoming: " + String(topic) + " - ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void sendData(Data data) {
  if(mqttClient.connected()) {
    String message = "{\"rssi\":" + String(data.rssi) + ",\"humidity\":" + String(data.humidity) + ",\"temperature\":" + String(data.temperature) + ",\"batteryvoltage\":" + String(data.batteryVoltage) + "}";
    Serial.println("Sending: " + message);

    mqttClient.publish("/Tinamous/V1/Measurements/Json", message.c_str());
//    mqttClient.publish(("ambient-sensor/" + deviceId + "/rssi").c_str(), String(data.rssi).c_str());
//    mqttClient.publish(("ambient-sensor/" + deviceId + "/humidity").c_str(), String(data.humidity).c_str());
//    mqttClient.publish(("ambient-sensor/" + deviceId + "/temperature").c_str(), String(data.temperature).c_str());
//    mqttClient.publish(("ambient-sensor/" + deviceId + "/batteryvoltage").c_str(), String(data.batteryVoltage).c_str());
  } else {
    Serial.println("MQTT not connected, not sending data");
  }
}
