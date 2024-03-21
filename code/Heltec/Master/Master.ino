#include "heltec.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

//PINOUT
#define VOLTAGE_SKY_1 33

#define VOLTAGE_BATTERY_H1 32
#define VOLTAGE_PANEL_H3 37
#define VOLTAGE_BATTERY_H3 38

#define CURRENT_H1 36
#define CURRENT_H3 39


//TOKEN
#define H2_TOKEN "dvs2lT4FEUHnztz91IdN"
#define H4_TOKEN "a2X4DrtBW1t9TRBT2xa9"
#define H6_TOKEN "2it4NbBFttYXnjhpjvVS"
#define SKY1_TOKEN "kCNX3s7Q2mIRVhfjaA8A"
#define H1_TOKEN "8nLoG5DUdN9q3GPFb7J9"
#define H3_TOKEN "1qjB5rAoWnLBSMn4cWGN"

//[Wi-Fi]
const char* ssid = "Fastweb";
const char* password = "password";
const char* mqtt_server = "192.168.125.130";
const int mqtt_port = 1883;
const char* telemetryTopic = "v1/devices/me/telemetry";
WiFiClient espClient;
PubSubClient client(espClient);

//VOLTAGE_SETUP
float R1 = 30000.0;
float R2 = 7500.0;  
float ref_voltage = 3.3;
float offset_voltage = 0.60;
float offset_voltage2 = 0.50;


byte buffer[256];
int packetSize = 0;
int rssi = 0;

void onReceive(int size) {
  packetSize = size;

  int i = 0;
  while (LoRa.available()) {
    byte b = LoRa.read();
    buffer[i] = b;
    i++;
  }
  rssi = LoRa.packetRssi();
}
 
void setup(){
  Serial.begin(115200);
  LoRa.begin(866E6, true);
  LoRa.onReceive(onReceive);
  delay(100);
  Serial.println("LoRa Init");
  LoRa.receive();

  WiFi.begin(ssid, password);
  delay(1000);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
}
 
void loop(){
  analogReadResolution(12);

  if (rssi != 0) {
    byte sensorType = buffer[0];

    float voltP_H2, cur_H2, voltB_H4, cur_H4, cur_H6;

    StaticJsonDocument<1024> jsonDoc;
    String jsonDoc_string;

    switch (sensorType){
      case 0:
        jsonDoc.clear();
        readFloatFromBuffer(buffer, 1, &voltP_H2);
        readFloatFromBuffer(buffer, 5, &cur_H2);
        jsonDoc["Voltage_Panel_H2"] = voltP_H2;
        jsonDoc["Current_H2"] = cur_H2;
        serializeJson(jsonDoc, jsonDoc_string);
        sendTelemetry(jsonDoc_string.c_str(), H2_TOKEN);
        Serial.println(jsonDoc_string);
        break;
      case 1:
        jsonDoc.clear();
        //readFloatFromBuffer(buffer, 1, &voltP_H4);
        readFloatFromBuffer(buffer, 1, &voltB_H4);
        readFloatFromBuffer(buffer, 5, &cur_H4);
        //jsonDoc["Voltage_Panel_H4"] = voltP_H4;
        jsonDoc["Voltage_Battery_H4"] = voltB_H4;
        jsonDoc["Current_H4"] = cur_H4;
        serializeJson(jsonDoc, jsonDoc_string);
        sendTelemetry(jsonDoc_string.c_str(), H4_TOKEN);
        Serial.println(jsonDoc_string);
        break;
      case 2:
        jsonDoc.clear();
        readFloatFromBuffer(buffer, 1, &cur_H6);
        jsonDoc["Current_H6"] = cur_H6;
        serializeJson(jsonDoc, jsonDoc_string);
        sendTelemetry(jsonDoc_string.c_str(), H6_TOKEN);
        Serial.println(jsonDoc_string);
        break;
        }

    StaticJsonDocument<1024> jsonDoc0;
    String jsonDoc_string0;
    StaticJsonDocument<1024> jsonDoc1;
    String jsonDoc_string1;
    StaticJsonDocument<1024> jsonDoc2;
    String jsonDoc_string2;

    float voltP_SKY1 = read_voltage(VOLTAGE_SKY_1, offset_voltage);
    float voltB_H1 = read_voltage(VOLTAGE_BATTERY_H1, offset_voltage2);
    float voltP_H3 = read_voltage(VOLTAGE_PANEL_H3, offset_voltage);
    float voltB_H3 = read_voltage(VOLTAGE_BATTERY_H3,  offset_voltage);
    float cur_H1 = read_current(CURRENT_H1);
    float cur_H3 = read_current(CURRENT_H3); 
    delay(2000);   
    jsonDoc0.clear();
    jsonDoc0["Voltage_Panel_Sky1"] = voltP_SKY1;
    serializeJson(jsonDoc0, jsonDoc_string0);
    sendTelemetry(jsonDoc_string0.c_str(), SKY1_TOKEN);
    Serial.println(jsonDoc_string0);
    jsonDoc1.clear();
    jsonDoc1["Voltage_Battery_H1"] = voltB_H1;
    jsonDoc1["Current_H1"] = cur_H1;
    serializeJson(jsonDoc1, jsonDoc_string1);
    sendTelemetry(jsonDoc_string1.c_str(), H1_TOKEN);
    Serial.println(jsonDoc_string1);
    jsonDoc2.clear();
    jsonDoc2["Voltage_Panel_H3"] = voltP_H3;
    jsonDoc2["Voltage_Battery_H3"] = voltB_H3;
    jsonDoc2["Current_H3"] = cur_H3;
    serializeJson(jsonDoc2, jsonDoc_string2);
    sendTelemetry(jsonDoc_string2.c_str(), H3_TOKEN);
    Serial.println(jsonDoc_string2);

    rssi = 0;
  }

  delay(500);
}


float read_voltage(int PIN, float offset){
  int adc_value = analogRead(PIN);
  float adc_voltage  = (adc_value * ref_voltage) / 4095.0; 
  float in_voltage = adc_voltage / (R2/(R1+R2)) + offset; 
   
  /*Serial.print("Voltage: ");
  Serial.print(PIN);
  Serial.print("_PIN = ");
  Serial.println(in_voltage, 2);*/

  return in_voltage;
  delay(100);
}

float read_current(int PIN){
  float current = 0;
  float adc_voltage = 0;

  for(int i=0; i<100; i++){
    float adc = analogRead(PIN);
    adc_voltage = (adc * (3.3 / 4095.0));
    current = current + ((adc_voltage - 1.5) / 0.100)/100;
    //current = current + ((0.000805664 * analogRead(currentpin) - 1.49)/0.100) /100;
    delay(10);
  }
  /*
  Serial.print("adc_Current: ");
  Serial.print(PIN);
  Serial.print("_PIN = ");
  Serial.println(adc_voltage);
  Serial.print("Current: ");
  Serial.print(PIN);
  Serial.print("_PIN = ");
  Serial.println(current);*/

  return adc_voltage;
}

void readFloatFromBuffer(byte* buffer, int offset, float* value) {
  byte tmp_float_buffer[4];
  for (int i = offset, j = 0; j < 4; i++, j++) {
    tmp_float_buffer[j] = buffer[i];
  }
  memcpy(value, &tmp_float_buffer, 4);
}

void sendTelemetry(const char* jsonData, char* access_token) {
  if (client.connect("ESP32 Client", access_token, NULL)) {
    client.publish(telemetryTopic, jsonData);
    client.disconnect();
  } else {
    if (WiFi.status() != WL_CONNECTED) {
      WiFi.reconnect();
    }

    delay(5000);
    setup_wifi();
  }
}

void setup_wifi() {
  if (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    WiFi.reconnect();
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}