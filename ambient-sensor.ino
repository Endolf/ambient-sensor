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

const int dataSendFrequency = 1000 * 60 * 10, sampleFrequency=10000;
const float dataSmoothingRatio = 0.7;

struct Data {
  float humidity;
  float temperature;
  float batteryVoltage;
  float rssi;
} data,smoothedData;

unsigned long nextSampleTime = 0L, lastDataSent = 0L, nextSendTime = dataSendFrequency;

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);

  rtc.begin();
  dht.begin();

  mqttClient.setServer("computerbooth.tinamous.com", 8883);
  mqttClient.setCallback(messageReceived);

}

void loop() {
  unsigned long currentLoopTime = millis();

  digitalWrite(LED_BUILTIN, (WiFi.status() == WL_CONNECTED));
  if (currentLoopTime >= nextSampleTime) {
    printDate();
    printTime();
    digitalWrite(LED_BUILTIN, true);
    sampleBatteryVoltage(&data);
    sampleTempAndHumdity(&data);

    if(WiFi.status()==WL_CONNECTED) {
      data.rssi = WiFi.RSSI();
    }

    smoothedData.rssi = smoothedData.rssi==0?data.rssi:(data.rssi * dataSmoothingRatio) + (smoothedData.rssi * (1-dataSmoothingRatio));
    smoothedData.humidity = smoothedData.humidity==0?data.humidity:(data.humidity * dataSmoothingRatio) + (smoothedData.humidity * (1-dataSmoothingRatio));
    smoothedData.temperature = smoothedData.temperature==0?data.temperature:(data.temperature * dataSmoothingRatio) + (smoothedData.temperature * (1-dataSmoothingRatio));
    smoothedData.batteryVoltage = smoothedData.batteryVoltage==0?data.batteryVoltage:(data.batteryVoltage * dataSmoothingRatio) + (smoothedData.batteryVoltage * (1-dataSmoothingRatio));
    digitalWrite(LED_BUILTIN, WiFi.status()==WL_CONNECTED);
    Serial.println("data: {\"rssi\":" + String(data.rssi) + ",\"humidity\":" + String(data.humidity) + ",\"temperature\":" + String(data.temperature) + ",\"batteryvoltage\":" + String(data.batteryVoltage) + "}");
    Serial.println("smoothedData: {\"rssi\":" + String(smoothedData.rssi) + ",\"humidity\":" + String(smoothedData.humidity) + ",\"temperature\":" + String(smoothedData.temperature) + ",\"batteryvoltage\":" + String(smoothedData.batteryVoltage) + "}");

    while(nextSampleTime<=currentLoopTime) nextSampleTime+=sampleFrequency;
    Serial.print("Sample loop ran at ");
    Serial.print(currentLoopTime);
    Serial.print(" in ");
    Serial.print(millis() - currentLoopTime);
    Serial.println("ms");
  }
  if (currentLoopTime >= nextSendTime) {
    checkAndConnectToWifi();
    if(smoothedData.rssi==0) {
      //Force override of RSSI data
      smoothedData.rssi = WiFi.RSSI();
    }
    sendData(smoothedData);
    while(nextSendTime<=currentLoopTime) nextSendTime+=dataSendFrequency;
    Serial.print("Data loop ran at ");
    Serial.print(currentLoopTime);
    Serial.print(" in ");
    Serial.print(millis() - currentLoopTime);
    Serial.println("ms");
  }
  mqttClient.loop();
  if (((millis() - lastDataSent) > 10000) && (WiFi.status() == WL_CONNECTED)) {
    Serial.print("Turning off WiFi: ");
    WiFi.end();
    Serial.println(WiFi.status());
    digitalWrite(LED_BUILTIN, false);
  }
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
  int numberOfTries = 0;

  // check if the WiFi module works
  if (status == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }

  // attempt to connect to WiFi network:
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID ");
    Serial.print(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    digitalWrite(LED_BUILTIN, true);

    numberOfTries = 0;
    while ( (status != WL_CONNECTED) && (numberOfTries < 10)) {
      Serial.print(".");
      delay(1000);
      status = WiFi.status();
      numberOfTries++;
    }
    Serial.println();
    // you're connected now, so print out the status:
    printWiFiStatus();
  }

  unsigned long epoch;
  numberOfTries = 0;
  Serial.print("Attempting to get NTP time");
  do {
    Serial.print(".");
    epoch = WiFi.getTime();
    numberOfTries++;
    delay(1000);
  }
  while ((epoch == 0) && (numberOfTries < 30));
  Serial.println();

  if (epoch == 0) {
    Serial.println("NTP unreachable!!");
    //Try again in a bit...
    //WiFi.disconnect();
  }
  else {
    Serial.print("Epoch received: ");
    Serial.println(epoch);
    rtc.setEpoch(epoch);
  }

  if ((WiFi.status() == WL_CONNECTED) && !mqttClient.connected()) {
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

void sampleBatteryVoltage(Data* data) {
  // read the input on analog pin 0:
  int sensorValue = analogRead(ADC_BATTERY);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 4.3V):
  float voltage = sensorValue * (4.3 / 1023.0);
  data->batteryVoltage = voltage;
}

void sampleTempAndHumdity(Data* data) {
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  data->humidity = humidity;
  data->temperature = temperature;
}

void messageReceived(char* topic, byte* payload, unsigned int length) {
  lastDataSent = millis();
  Serial.print("incoming: " + String(topic) + " - ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    lastDataSent = millis();
  }
  Serial.println();
}

void sendData(Data data) {
  if (mqttClient.connected()) {
    String message = "{\"rssi\":" + String(data.rssi) + ",\"humidity\":" + String(data.humidity) + ",\"temperature\":" + String(data.temperature) + ",\"batteryvoltage\":" + String(data.batteryVoltage) + "}";
    Serial.println("Sending: " + message);

    mqttClient.publish("/Tinamous/V1/Measurements/Json", message.c_str());
    //    mqttClient.publish(("ambient-sensor/" + deviceId + "/rssi").c_str(), String(data.rssi).c_str());
    //    mqttClient.publish(("ambient-sensor/" + deviceId + "/humidity").c_str(), String(data.humidity).c_str());
    //    mqttClient.publish(("ambient-sensor/" + deviceId + "/temperature").c_str(), String(data.temperature).c_str());
    //    mqttClient.publish(("ambient-sensor/" + deviceId + "/batteryvoltage").c_str(), String(data.batteryVoltage).c_str());
    lastDataSent = millis();
  } else {
    Serial.println("MQTT not connected, not sending data");
  }
}
