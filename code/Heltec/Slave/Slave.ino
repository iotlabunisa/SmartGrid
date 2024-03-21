#include <Arduino.h>
#include "ArduinoJson.h"
#include <string>
#include <RadioLib.h>
#include "heltec.h"
#include <Wire.h>

//PINOUT
#define VOLTAGE_PANEL_H2 13
#define CURRENT_H2 12

#define VOLTAGE_PANEL_H4 33
#define VOLTAGE_BATTERY_H4 38
#define CURRENT_H4 39

#define CURRENT_H6 32

//VOLTAGE_SETUP
float R1 = 30000.0;
float R2 = 7500.0;  
float ref_voltage = 3.3;
float offset_voltage = 0.60;
float offset_voltage2 = 0.50;


void setup() {
  Serial.begin(115200);
  LoRa.begin(866E6, true);
}

void loop() {
  analogReadResolution(12);

  float voltP_H2 = read_voltage(VOLTAGE_PANEL_H2, offset_voltage);
  //float voltP_H4 = read_voltage(VOLTAGE_PANEL_H4, offset_voltage);
  float voltB_H4 = read_voltage(VOLTAGE_BATTERY_H4,  offset_voltage);

  float cur_H2 = read_current(CURRENT_H2);
  float cur_H4 = read_current(CURRENT_H4);
  float cur_H6 = read_current(CURRENT_H6);

  unsigned char buffer[255];
  delay(5000);

  //HOUSE_2
  int index = 0;
  buffer[index++] = 0;
  insertIntoBuffer(&index, buffer, voltP_H2);
  insertIntoBuffer(&index, buffer, cur_H2);
  sendBuffer(buffer, index);
  delay(5000);
  
  //HOUSE_4
  index = 0;
  buffer[index++] = 1;
  //insertIntoBuffer(&index, buffer, voltP_H4);
  insertIntoBuffer(&index, buffer, voltB_H4);
  insertIntoBuffer(&index, buffer, cur_H4);
  sendBuffer(buffer, index);
  delay(5000);

  //HOUSE_6
  index = 0;
  buffer[index++] = 2;
  insertIntoBuffer(&index, buffer, cur_H6);
  sendBuffer(buffer, index);
  delay(5000);
}

float read_voltage(int PIN, float offset){
  int adc_value = analogRead(PIN);
  float adc_voltage  = (adc_value * ref_voltage) / 4095.0; 
  float in_voltage = adc_voltage / (R2/(R1+R2)) + offset; 
   
  /*
  Serial.print("Voltage: ");
  Serial.print(PIN);
  Serial.print("_PIN = ");
  Serial.println(in_voltage, 2);
  */

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
  Serial.print("Current: ");
  Serial.print(PIN);
  Serial.print("_PIN = ");
  Serial.println(adc_voltage);
  */

  return adc_voltage;
}

void sendBuffer(unsigned char* buffer, int size) {
  /*Serial.println(size);
  if (size > 255) Serial.println("Buffer overlow!!!!!!!");
  for (int i = 0; i < size; i++) {
    byte b = buffer[i];
    Serial.print(b);
    Serial.print(" ");
  }*/

  //INVIO DATI LORA
  LoRa.beginPacket();
  LoRa.setTxPower(20,RF_PACONFIG_PASELECT_PABOOST);
  LoRa.write(buffer, size);
  int ack = LoRa.endPacket();
  //Serial.println(ack);
}

void insertIntoBuffer(int* index, unsigned char* buffer, float value) {
  byte tmp_float[4];
  float2Bytes(tmp_float, value);
  buffer[(*index)++] = (char) (tmp_float[0]);
  buffer[(*index)++] = (char) (tmp_float[1]);
  buffer[(*index)++] = (char) (tmp_float[2]);
  buffer[(*index)++] = (char) (tmp_float[3]);
}

void float2Bytes(byte bytes_temp[4],float float_variable){ 
  memcpy(bytes_temp, (unsigned char*) (&float_variable), 4);
}