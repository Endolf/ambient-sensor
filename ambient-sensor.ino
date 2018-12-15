#include <WiFiNINA.h>
#include <RTCZero.h>
#include <DHT.h>
#include <MQTT.h>

#include "arduino_secrets.h"

#define DHTPIN 7     // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino
RTCZero rtc;
WiFiSSLClient wifiClient;
MQTTClient mqttClient;

const String deviceId = "test";

unsigned long lastBasicLoopTime = 0, lastDhtLoopTime = 0;
const int basicLoopTime = 10000, dhtLoopTime = 10000;

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);

  rtc.begin();
  dht.begin();

  mqttClient.begin("io.adafruit.com", 8883, wifiClient);
  mqttClient.onMessage(messageReceived);

  checkAndConnectToWifi();
}

void loop() {
  unsigned long currentLoopTime = millis();
  if ((currentLoopTime - lastBasicLoopTime) >= basicLoopTime || currentLoopTime < lastBasicLoopTime) {
    checkAndConnectToWifi();
    printDate();
    printTime();
    printBatteryVoltage();
    mqttClient.publish("Endolf/f/rssi", String(WiFi.RSSI()));
    lastBasicLoopTime = (currentLoopTime / basicLoopTime) * basicLoopTime;
  }
  if ((currentLoopTime - lastDhtLoopTime) >= dhtLoopTime || currentLoopTime < lastDhtLoopTime) {
    printTempAndHumdity();
    lastDhtLoopTime = (currentLoopTime / dhtLoopTime) * dhtLoopTime;
  }
  if(mqttClient.connected()) {
    mqttClient.loop();
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

  digitalWrite(LED_BUILTIN, status == WL_CONNECTED);

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
    digitalWrite(LED_BUILTIN, status == WL_CONNECTED);
    // you're connected now, so print out the status:
    printWiFiStatus();

    unsigned long epoch;
    numberOfTries = 0;
    do {
      epoch = WiFi.getTime();
      numberOfTries++;
    }
    while ((epoch == 0) && (numberOfTries < maxTries));

    if (numberOfTries > maxTries) {
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

  if(!mqttClient.connected()) {
    Serial.println("Connecting to mqtt");
    if(!mqttClient.connect(deviceId.c_str(), IO_USERNAME, IO_KEY)) {    
      Serial.print("Failed to connect: ");
      Serial.print(mqttClient.lastError());
      Serial.print(", ");
      Serial.print(mqttClient.connected());
      Serial.print(", ");
      Serial.println(mqttClient.returnCode());
    }
  }
}

void printBatteryVoltage() {
  // read the input on analog pin 0:
  int sensorValue = analogRead(ADC_BATTERY);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 4.3V):
  float voltage = sensorValue * (4.3 / 1023.0);
  // print out the value you read:
  Serial.print(voltage);
  Serial.println("V");
  mqttClient.publish("Endolf/f/battery", String(voltage));
}

void printTempAndHumdity() {
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.print(" %, Temp: ");
  Serial.print(temperature);
  Serial.println(" Celsius");
  mqttClient.publish("Endolf/f/humidity", String(humidity));
  mqttClient.publish("Endolf/f/temperature", String(temperature));
}

void messageReceived(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);
}
