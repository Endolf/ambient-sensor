#define MQTT_KEEPALIVE 5

#include <Arduino.h>

#ifdef USE_WIFI_NINA
#include <WiFiNINA.h>
#else
#include <WiFi.h>
#include <WiFiClientSecure.h>
#endif
#ifdef USE_RTCZero
#include <RTCZero.h>
#endif
#ifdef USE_ESP32_DHT
#include <DHTesp.h>
#else
#include <DHT.h>
#endif

#include <PubSubClient.h>

#include DEVICE_SECRETS_H

#ifdef USE_ESP32_DHT
DHTesp dht;
#else
#define DHTTYPE DHT22 // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino
#endif
#ifdef USE_RTCZero
RTCZero rtc;
#endif

#ifdef USE_WIFI_NINA
WiFiSSLClient wifiClient;
#else
WiFiClientSecure wifiClient;
#endif
PubSubClient mqttClient(wifiClient);

const int dataSendFrequency = 1000 * 60 * 10, sampleFrequency = 10000;
const float dataSmoothingRatio = 0.7;

struct Data
{
    float humidity = 0;
    float temperature = 0;
    float batteryVoltage = 0;
    float rssi = 0;
} data, smoothedData;

unsigned long nextSampleTime = 0L, lastDataSent = 0L, lastLoopTime = 0L, nextSendTime = dataSendFrequency;

void print2digits(int number)
{
    if (number < 10)
    {
        Serial.print("0");
    }
    Serial.print(number);
}

void printTime()
{
#ifdef USE_RTCZero
    print2digits(rtc.getHours());
    Serial.print(":");
    print2digits(rtc.getMinutes());
    Serial.print(":");
    print2digits(rtc.getSeconds());
    Serial.println(" UTC");
#else
    Serial.print("No RTC: ");
    Serial.println(millis());
#endif
}

void printDate()
{
#ifdef USE_RTCZero
    Serial.print(rtc.getDay());
    Serial.print("/");
    Serial.print(rtc.getMonth());
    Serial.print("/");
    Serial.print(rtc.getYear());

    Serial.print(" ");
#endif
}

void printWiFiStatus()
{
#ifdef USE_WIFI_NINA
    Serial.print("Firmware version: ");
    Serial.println(WiFi.firmwareVersion());
#endif
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

void checkAndConnectToWifi()
{
    int status = WiFi.status();
    int numberOfTries = 0;

#ifdef USE_WIFI_NINA
    // check if the WiFi module works
    if (status == WL_NO_SHIELD)
    {
        Serial.println("WiFi shield not present");
        // don't continue:
        while (true)
            ;
    }
#endif

    // attempt to connect to WiFi network:
    while (status != WL_CONNECTED)
    {
        Serial.print("Attempting to connect to SSID ");
        Serial.print(ssid);
        // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
        status = WiFi.begin(ssid, pass);
#ifdef USE_BUILT_IN_LED
        digitalWrite(LED_BUILTIN, true);
#endif

        numberOfTries = 0;
        while ((status != WL_CONNECTED) && (numberOfTries < 10))
        {
            Serial.print(".");
            delay(1000);
            status = WiFi.status();
            numberOfTries++;
        }
        Serial.println();
        // you're connected now, so print out the status:
        printWiFiStatus();
    }

#ifdef USE_WIFI_NINA
    unsigned long epoch;
    numberOfTries = 0;
    Serial.print("Attempting to get NTP time");
    do
    {
        Serial.print(".");
        epoch = WiFi.getTime();
        numberOfTries++;
        delay(1000);
    } while ((epoch == 0) && (numberOfTries < 30));
    Serial.println();

    if (epoch == 0)
    {
        Serial.println("NTP unreachable!!");
        //Try again in a bit...
        //WiFi.disconnect();
    }
    else
    {
        Serial.print("Epoch received: ");
        Serial.println(epoch);
#ifdef USE_RTCZero
        rtc.setEpoch(epoch);
#endif
    }
#endif

    if ((WiFi.status() == WL_CONNECTED) && !mqttClient.connected())
    {
        Serial.print("MQTT not connected: ");
        Serial.println(mqttClient.state());
        Serial.println("Connecting to mqtt");
        mqttClient.disconnect();
        if (!mqttClient.connect(deviceId, mqttUser, mqttPass))
        {
            Serial.print("Failed to connect: ");
            Serial.println(mqttClient.state());
        }
    }
}

void sampleBatteryVoltage(Data *data)
{
#ifdef USE_BATTERY_VOLTAGE
    // read the input on analog pin 0:
    int sensorValue = analogRead(ADC_BATTERY);
    // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 4.3V):
    float voltage = sensorValue * (4.3 / 1023.0);
    if (!isnan(voltage))
    {
        data->batteryVoltage = voltage;
    }
#endif
}

void sampleTempAndHumdity(Data *data)
{
#ifdef USE_ESP32_DHT
    float humidity = dht.getHumidity();
    float temperature = dht.getTemperature();
#else
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();
#endif
    if (!isnan(humidity))
    {
        data->humidity = humidity;
    }
    if (!isnan(temperature))
    {
        data->temperature = temperature;
    }
}

void messageReceived(char *topic, byte *payload, unsigned int length)
{
    lastDataSent = millis();
    Serial.print("incoming: " + String(topic) + " - ");
    for (unsigned int i = 0; i < length; i++)
    {
        Serial.print((char)payload[i]);
        lastDataSent = millis();
    }
    Serial.println();
}

void sendData(Data data)
{
    if (mqttClient.connected())
    {
        String message = "{\"rssi\":" + String(data.rssi) + ",\"humidity\":" + String(data.humidity) + ",\"temperature\":" + String(data.temperature)
#ifdef USE_BATTERY_VOLTAGE
                         + ",\"batteryvoltage\":" + String(data.batteryVoltage)
#endif
                         + "}";
        Serial.println("Sending: " + message);

        mqttClient.publish("/Tinamous/V1/Measurements/Json", message.c_str());
        //    mqttClient.publish(("ambient-sensor/" + deviceId + "/rssi").c_str(), String(data.rssi).c_str());
        //    mqttClient.publish(("ambient-sensor/" + deviceId + "/humidity").c_str(), String(data.humidity).c_str());
        //    mqttClient.publish(("ambient-sensor/" + deviceId + "/temperature").c_str(), String(data.temperature).c_str());
        //    mqttClient.publish(("ambient-sensor/" + deviceId + "/batteryvoltage").c_str(), String(data.batteryVoltage).c_str());
        lastDataSent = millis();
    }
    else
    {
        Serial.println("MQTT not connected, not sending data");
    }
}

void setup()
{

#ifdef USE_BUILT_IN_LED
    pinMode(LED_BUILTIN, OUTPUT);
#endif

    Serial.begin(115200);

#ifdef USE_RTCZero
    rtc.begin();
#endif
#ifdef USE_ESP32_DHT
    dht.setup(DHTPIN, DHTesp::DHT22);
#else
    dht.begin();
#endif

    mqttClient.setServer("computerbooth.tinamous.com", 8883);
    mqttClient.setCallback(messageReceived);
}

void loop()
{
    unsigned long currentLoopTime = millis();

    if (currentLoopTime < lastLoopTime)
    {
        //Handle millis overflow;
        Serial.println("millis() overflow detected, lastLoopTime: " + String(lastLoopTime) + ", currentLoopTime: " + String(currentLoopTime));
        Serial.println("original values, nextSampleTime: " + String(nextSampleTime) + ", nextSendTime: " + String(nextSendTime));
        nextSampleTime = currentLoopTime;
        nextSendTime = currentLoopTime + dataSendFrequency;
        Serial.println("updated values, nextSampleTime: " + String(nextSampleTime) + ", nextSendTime: " + String(nextSendTime));
    }

#ifdef USE_BUILT_IN_LED
    digitalWrite(LED_BUILTIN, (WiFi.status() == WL_CONNECTED));
#endif
    if (currentLoopTime >= nextSampleTime)
    {
        printDate();
        printTime();
#ifdef USE_BUILT_IN_LED
        digitalWrite(LED_BUILTIN, true);
#endif
        sampleBatteryVoltage(&data);
        sampleTempAndHumdity(&data);

        if (WiFi.status() == WL_CONNECTED)
        {
            data.rssi = WiFi.RSSI();
        }

        smoothedData.rssi = smoothedData.rssi == 0 ? data.rssi : (data.rssi * dataSmoothingRatio) + (smoothedData.rssi * (1 - dataSmoothingRatio));
        smoothedData.humidity = smoothedData.humidity == 0 ? data.humidity : (data.humidity * dataSmoothingRatio) + (smoothedData.humidity * (1 - dataSmoothingRatio));
        smoothedData.temperature = smoothedData.temperature == 0 ? data.temperature : (data.temperature * dataSmoothingRatio) + (smoothedData.temperature * (1 - dataSmoothingRatio));
        smoothedData.batteryVoltage = smoothedData.batteryVoltage == 0 ? data.batteryVoltage : (data.batteryVoltage * dataSmoothingRatio) + (smoothedData.batteryVoltage * (1 - dataSmoothingRatio));
#ifdef USE_BUILT_IN_LED
        digitalWrite(LED_BUILTIN, WiFi.status() == WL_CONNECTED);
#endif
        Serial.println("data: {\"rssi\":" + String(data.rssi) + ",\"humidity\":" + String(data.humidity) + ",\"temperature\":" + String(data.temperature) + ",\"batteryvoltage\":" + String(data.batteryVoltage) + "}");
        Serial.println("smoothedData: {\"rssi\":" + String(smoothedData.rssi) + ",\"humidity\":" + String(smoothedData.humidity) + ",\"temperature\":" + String(smoothedData.temperature) + ",\"batteryvoltage\":" + String(smoothedData.batteryVoltage) + "}");

        while (nextSampleTime <= currentLoopTime)
            nextSampleTime += sampleFrequency;
        Serial.print("Sample loop ran at ");
        Serial.print(currentLoopTime);
        Serial.print(" in ");
        Serial.print(millis() - currentLoopTime);
        Serial.print("ms, next sample millis: ");
        Serial.println(nextSampleTime);
    }
    if (currentLoopTime >= nextSendTime)
    {
        checkAndConnectToWifi();
        if (smoothedData.rssi == 0)
        {
            //Force override of RSSI data
            smoothedData.rssi = WiFi.RSSI();
        }
        sendData(smoothedData);
        while (nextSendTime <= currentLoopTime)
            nextSendTime += dataSendFrequency;
        Serial.print("Data loop ran at ");
        Serial.print(currentLoopTime);
        Serial.print(" in ");
        Serial.print(millis() - currentLoopTime);
        Serial.print("ms, next send millis: ");
        Serial.println(nextSendTime);
    }
    mqttClient.loop();
    if (((millis() - lastDataSent) > 10000) && (WiFi.status() == WL_CONNECTED))
    {
        if (mqttClient.connected())
        {
            mqttClient.disconnect();
        }
        Serial.print("Turning off WiFi: ");
#ifdef USE_WIFI_NINA
        WiFi.end();
#else
        WiFi.disconnect(true);
//        WiFi.mode(WIFI_OFF);
#endif
        Serial.println(WiFi.status());
#ifdef USE_BUILT_IN_LED
        digitalWrite(LED_BUILTIN, false);
#endif
    }

    lastLoopTime = currentLoopTime;
}
