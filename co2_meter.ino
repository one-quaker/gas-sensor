#include <Arduino.h>
#include <MHZ19.h>                                         // include main library
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include "config.h"


#define RX_PIN 16                                          // Rx pin which the MHZ19 Tx pin is attached to
#define TX_PIN 17                                          // Tx pin which the MHZ19 Rx pin is attached to
#define CO_PIN 34

#define BAUDRATE 9600                                      // Native to the sensor (do not change)
#define EEPROM_SIZE 512

//!!!!! patch PubSubClient.h to MQTT_MAX_PACKET_SIZE 1024  // fix for MQTT client dropping messages over 128B

const int CO2ZERO = 400;
const int CO_ALARM_VALUE = 350;
const char MQTT_PUB_TOPIC[16] = "gas-sensor";

MHZ19 myMHZ19;                                             // Constructor for MH-Z19 class
HardwareSerial mySerial(1);                              // ESP32 Example


const int sendDataIntervalDefault = 20000;
int sendDataInterval = sendDataIntervalDefault;
unsigned long getDataTimer = 0;                             // Variable to store timer interval
const int co2MinSaveInterval = 12 * 60 * 60 * 1000;         // 12 hours
unsigned long co2MinSaveTimer = 0;
bool co2Ready = false;
const int co2PreheatTime = 3 * 60 * 1000;                   // 3 minutes
// const int co2PreheatTime = 5000;

int co2MinVal;
int co2StoredMinVal;


WiFiClient espClient;
PubSubClient client(espClient);


void setup()
{
    Serial.begin(9600);                                     // For ESP32 baudarte is 115200 etc.
    mySerial.begin(BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN); // ESP32 Example
    myMHZ19.begin(mySerial);                                // *Important, Pass your Stream reference

    Serial.println("\nTesting EEPROM Library\n");
    if (!EEPROM.begin(EEPROM_SIZE)) {
        Serial.println("Failed to initialise EEPROM");
        Serial.println("Restarting...");
        delay(1000);
        ESP.restart();
    }
    co2StoredMinVal = EEPROM.readUInt(0);

    myMHZ19.autoCalibration(false);                              // Turn auto calibration ON (disable with autoCalibration(false))

    setup_wifi();
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
}

void loop() {
    StaticJsonDocument<512> doc;
    char jsonMsg[512];

    int hallVal = hallRead();
    int coValue = analogRead(CO_PIN);

    if (coValue > CO_ALARM_VALUE) {
        sendDataInterval = 1000;
        co2Ready = true;
        Serial.print("CO ALARM: ");
        Serial.println(coValue);
    } else {
        sendDataInterval = sendDataIntervalDefault;
        co2Ready = false;
    }

    if (!client.connected()) {
      reconnect();
    }
    client.loop();


    if (millis() - co2MinSaveTimer >= co2MinSaveInterval) {
        co2MinSaveTimer = millis();
        co2StoredMinVal = saveCo2Val(co2MinVal);
    }

    if (millis() >= co2PreheatTime && !co2Ready) {
      co2Ready = true;
    } else if (!co2Ready) {
      Serial.println("CO2 sensor is preheating...");
      Serial.print("ready in ");
      Serial.print((co2PreheatTime - millis()) / 1000);
      Serial.println(" seconds...");

      doc["uptime"] = millis();
      doc["status"] = "heating";
      serializeJson(doc, jsonMsg);
      client.publish(MQTT_PUB_TOPIC, jsonMsg);

      delay(10000);
    }

    if (millis() - getDataTimer >= sendDataInterval && co2Ready)                    // Check if interval has elapsed (non-blocking delay() equivilant)
    {
        int CO2RAW;                                            // Buffer for CO2
        int CO2REAL;
        int CO2DIFF;

        CO2RAW = myMHZ19.getCO2();                             // Request CO2 (as ppm)
        CO2DIFF = CO2RAW - CO2_REAL_ZERO;

        if (CO2DIFF <= 0) {
            CO2REAL = CO2ZERO;
        } else {
            CO2REAL = CO2ZERO + CO2DIFF;
        }

        if (co2MinVal == 0) {
            co2MinVal = CO2RAW;
        } else if (co2MinVal > CO2RAW) {
            co2MinVal = CO2RAW;
        }

        Serial.print("CO2 raw (ppm): ");
        Serial.println(CO2RAW);
        Serial.print("CO2 min (ppm): ");
        Serial.println(co2MinVal);
        Serial.print("CO2 (ppm): ");
        Serial.println(CO2REAL);
        Serial.print("CO: ");
        Serial.println(coValue);

        Serial.print("Hall sensor: ");
        Serial.println(hallVal);

        int8_t Temp;                                         // Buffer for temperature
        Temp = myMHZ19.getTemperature();                     // Request Temperature (as Celsius)
        Serial.print("Temperature (C): ");
        Serial.println(Temp);

        doc["co2"] = CO2REAL;
        doc["temp"] = Temp;
        doc["co"] = coValue;
        doc["co2_raw"] = CO2RAW;
        doc["co2_min"] = co2MinVal;
        doc["co2_stored_min"] = co2StoredMinVal;
        doc["co2_min_manual"] = CO2_REAL_ZERO;
        doc["mqtt_server"] = mqtt_server;
        doc["wifi_ssid"] = ssid;
        doc["local_ip"] = ipToString(WiFi.localIP());
        doc["uptime"] = millis();
        serializeJson(doc, jsonMsg);

        if (strlen(jsonMsg) > MQTT_MAX_PACKET_SIZE) {
            StaticJsonDocument<64> doc;
            char errMsg[64];

            doc["status"] = "mqtt message length is too big";
            serializeJson(doc, errMsg);

            client.publish(MQTT_PUB_TOPIC, errMsg);

            Serial.print("MQTT_MAX_PACKET_SIZE: ");
            Serial.println(MQTT_MAX_PACKET_SIZE);
            Serial.print("mqtt message length: ");
            Serial.println(strlen(jsonMsg));
            Serial.println("increase MQTT_MAX_PACKET_SIZE");
        } else {
            client.publish(MQTT_PUB_TOPIC, jsonMsg);
        }

        getDataTimer = millis();                            // Update interval
    }

}


void setup_wifi() {
    delay(10);
    // We start by connecting to a WiFi network
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
    Serial.print("Message arrived on topic: ");
    Serial.print(topic);
    Serial.print(". Message: ");
    String messageTemp;

    for (int i = 0; i < length; i++) {
        Serial.print((char)message[i]);
        messageTemp += (char)message[i];
    }
    Serial.println();

    // Feel free to add more if statements to control more GPIOs with MQTT

    // If a message is received on the topic esp32/output, you check if the message is either "on" or "off".
    // Changes the output state according to the message
    if (String(topic) == "esp32/output") {
        Serial.print("Changing output to ");
        if (messageTemp == "on") {
            Serial.println("on");
        }
        else if (messageTemp == "off") {
            Serial.println("off");
        }
    }
}

void reconnect() {
    // Loop until we're reconnected
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (client.connect("ESPClient")) {
            Serial.println("connected");
            // Subscribe
            // client.subscribe("esp32/output");
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }
}


String ipToString(IPAddress ip){
    String s="";
    for (int i=0; i<4; i++) {
        s += i  ? "." + String(ip[i]) : String(ip[i]);
    }
    return s;
}


int saveCo2Val(int saveVal) {
    int curVal;
    curVal = EEPROM.readUInt(0);

    if (curVal != saveVal) {
        Serial.println("EEPROM write");
        EEPROM.writeInt(0, saveVal);
        EEPROM.commit();
        return saveVal;
    } else {
        Serial.println("EEPROM skip write, value is not changed");
        return curVal;
    }
}
