/*============================================================

Project name : Machine Monitoring System
Project purpose : Preventive Maintenance, Monitoring
Description: System includes 
                2 x CT sensor (600A)
                1 x K Type Thermocouple 
                1 x IR Temperature Sensor
                Data sent every 1 second to ThingsBoard
                4 Tasks running on ESP32
                Sending Data over WiFi
Last updated date : 02/06/2025
Author: 

*============================================================/

/*--------Libraries------------*/
#include "Arduino.h"
#include "max6675.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <config.h>

//========Print Macro========================================/
#define DEBUG 1

#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#define debugf(...) Serial.printf(__VA_ARGS__)
#else
#define debug(x)
#define debugln(x)
#define debugf(...)
#endif

//===========================================================/
WiFiClient espClient;
PubSubClient client(espClient);

/*----PIN ASSIGNING------*/
#define CT1_PIN 33 
#define CT2_PIN 34

#define ThermoDO 19 
#define ThermoCS 23
#define ThermoCLK 5
MAX6675 thermocouple(ThermoCLK, ThermoCS, ThermoDO);

#define IR_PIN 32

/*-------TASK NAMING---------*/
TaskHandle_t CT1;
TaskHandle_t CT2;
TaskHandle_t Thermo;
TaskHandle_t IR;

/*-----CT SENSORS CONFIGURATION--------*/
const float ADC_REF_V = 3.3;
const int ADC_MAX = 4095;

const int SAMPLE_INTERVAL_uS = 50;
const int NUM_SAMPLES = 20000;

float* voltageSamples1 = nullptr;
float* voltageSamples2 = nullptr;

unsigned long lastSampleTime1 = 0;
unsigned long lastSampleTime2 = 0;

/*-----Global Volatile Variable for ThingsBoard-------*/
volatile float Ip1 = 0.0;
volatile float Ip2 = 0.0;
volatile float temperatureC = 0.0;
volatile float irTemperature = 0.0;

unsigned long lastSendTime = 0;

#define _WIFI_CONNECT 1
#define _MQTT_CONNECT 2
#define _SEND_DATA 3
#define _DISCONNECT 4

int state;
int error_count = 0;
#define Error_count_th 5

// Function Declarations
void Task1code( void * Parameters );
void Task2code( void * Parameters );
void Task3code( void * Parameters );
void Task4code( void * Parameters );


void setupWiFi() {
  WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  debug("Connecting to WiFi");
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    debug(".");
    retries++;
    if (retries > 20) break;
  }
  debugln(WiFi.localIP());
}

void setupMQTT() {
  client.setServer(MQTT_SERVER, MQTT_PORT);
}

void reconnectMQTT() {
  while (!client.connected()) {
    debug("Attempting MQTT connection...");
    if (client.connect(MQTT_CLIENTID, MQTT_USERNAME, NULL)) {
      debugln("connected");
    } else {
      debug("failed, rc=");
      debug(client.state());
      delay(1000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  debugln("Starting...");
  state = _WIFI_CONNECT;

  xTaskCreatePinnedToCore(Task1code, "CT1", 10000, NULL, 1, &CT1, 1);
  xTaskCreatePinnedToCore(Task2code, "CT2", 10000, NULL, 1, &CT2, 1);
  xTaskCreatePinnedToCore(Task3code, "Thermo", 10000, NULL, 1, &Thermo, 1);
  xTaskCreatePinnedToCore(Task4code, "IR", 10000, NULL, 1, &IR, 1);

  delay(100);
}

void Task1code( void * Parameters ){
  debug("Task1 running on core ");
  debugln(xPortGetCoreID());

  pinMode(CT1_PIN, INPUT); 

  // Print available heap memory before allocation
  // debugf("Free heap before malloc1: %u bytes\n", ESP.getFreeHeap());

  voltageSamples1 = (float*) malloc(NUM_SAMPLES * sizeof(float));
  if (voltageSamples1 == nullptr) {
    debugln("Memory1 allocation failed!");
    while (true); // Halt
  }

  // Print available heap memory after allocation
  // debugf("Free heap after malloc1: %u bytes\n", ESP.getFreeHeap());

  delay(100);

  for(;;){
    static int sampleIndex1 = 0;
    static unsigned long startTime1 = 0;
    unsigned long currentTime1 = micros();

    if (sampleIndex1 == 0) {
      startTime1 = millis(); // Start timing when beginning new sample batch
    }

    if (currentTime1 - lastSampleTime1 >= SAMPLE_INTERVAL_uS) {
      lastSampleTime1 = currentTime1;

      // Read and convert to voltage
      uint16_t adc1 = analogRead(CT1_PIN);
      float voltage1 = (adc1 * ADC_REF_V) / ADC_MAX;

      // Store the sample
      voltageSamples1[sampleIndex1++] = voltage1;

      // If all samples collected
      if (sampleIndex1 >= NUM_SAMPLES) {
        // Calculate Vavg
        float Vavg1 = 0.0;
        for (int i = 0; i < NUM_SAMPLES; i++) {
          Vavg1 += voltageSamples1[i];
        }
        Vavg1 /= NUM_SAMPLES;

        // Calculate Vrms
        float sumsq1 = 0.0;
        for (int i = 0; i < NUM_SAMPLES; i++) {
          float centered1 = voltageSamples1[i] - Vavg1;
          sumsq1 += centered1 * centered1;
        }
        float Vrms1 = sqrt(sumsq1 / NUM_SAMPLES);

        float Is1 = 0.0;

        if(Vrms1 > 0.015){
          if(Vrms1 > 0.5){
            // Calculate current
            Is1 = (2.8469*Vrms1*Vrms1) + (2.8225*Vrms1) + 0.1549;
            // Is1 = (2.8469*Vrms1*Vrms1) + (2.8965*Vrms1) + 0.1921;
          }else{
            Is1 = (-1.4588*Vrms1*Vrms1) + (5.0522*Vrms1) - 0.014;
            // Is1 = (-1.4588*Vrms1*Vrms1) + (5.0143*Vrms1) + 0.0514;
          }
          // Calculate current
          Ip1 = Is1 * 120;
        }else{
          Ip1 = 0.0;
        }
        unsigned long endTime1 = millis();
        unsigned long elapsedTime1 = endTime1 - startTime1;

        debugf("Vavg1: %.3f V, Vrms1: %.3f V, Is1: %.3f A, Ip1: %.3f A, Time1: %lu ms\n", Vavg1, Vrms1, Is1, Ip1, elapsedTime1);
        
        // Reset sample index to reuse memory
        sampleIndex1 = 0;

        // Print available heap memory after one loop
        // debugf("Free heap after one loop1: %u bytes\n", ESP.getFreeHeap());

        delay(500);
      }
    }
  }
}

void Task2code( void * Parameters ){
  debug("Task2 running on core ");
  debugln(xPortGetCoreID());

  pinMode(CT2_PIN, INPUT);

  // Print available heap memory before allocation
  // debugf("Free heap before malloc2: %u bytes\n", ESP.getFreeHeap());

  voltageSamples2 = (float*) malloc(NUM_SAMPLES * sizeof(float));
  if (voltageSamples2 == nullptr) {
    debugln("Memory2 allocation failed!");
    while (true); // Halt
  }

  // Print available heap memory after allocation
  // debugf("Free heap after malloc2: %u bytes\n", ESP.getFreeHeap());

  delay(100);

  for(;;){
    static int sampleIndex2 = 0;
    static unsigned long startTime2 = 0;
    unsigned long currentTime2 = micros();

    if (sampleIndex2 == 0) {
      startTime2 = millis(); // Start timing when beginning new sample batch
    }

    if (currentTime2 - lastSampleTime2 >= SAMPLE_INTERVAL_uS) {
      lastSampleTime2 = currentTime2;

      // Read and convert to voltage
      uint16_t adc2 = analogRead(CT2_PIN);
      float voltage2 = (adc2 * ADC_REF_V) / ADC_MAX;

      // Store the sample
      voltageSamples2[sampleIndex2++] = voltage2;

      // If all samples collected
      if (sampleIndex2 >= NUM_SAMPLES) {
        // Calculate Vavg
        float Vavg2 = 0.0;
        for (int i = 0; i < NUM_SAMPLES; i++) {
          Vavg2 += voltageSamples2[i];
        }
        Vavg2 /= NUM_SAMPLES;

        // Calculate Vrms
        float sumsq2 = 0.0;
        for (int i = 0; i < NUM_SAMPLES; i++) {
          float centered2 = voltageSamples2[i] - Vavg2;
          sumsq2 += centered2 * centered2;
        }
        float Vrms2 = sqrt(sumsq2 / NUM_SAMPLES);

        float Is2 = 0.0;

        if(Vrms2 > 0.015){
          if(Vrms2 > 0.5){
            // Calculate current
            Is2 = (2.8469*Vrms2*Vrms2) + (2.8225*Vrms2) + 0.1549;
            // Is2 = (2.8469*Vrms2*Vrms2) + (2.8965*Vrms2) + 0.1921;
          }else{
            Is2 = (-1.4588*Vrms2*Vrms2) + (5.0522*Vrms2) - 0.014;
            // Is2 = (-1.4588*Vrms2*Vrms2) + (5.0143*Vrms2) + 0.0514;
          }
          // Calculate current
          Ip2 = Is2 * 120;
        }else{
          Ip2 = 0.0;
        }

        unsigned long endTime2 = millis();
        unsigned long elapsedTime2 = endTime2 - startTime2;

        debugf("Vavg2: %.3f V, Vrms2: %.3f V, Is2: %.3f A, Ip2: %.3f A, Time2: %lu ms\n", Vavg2, Vrms2, Is2, Ip2, elapsedTime2);
        
        // Reset sample index to reuse memory
        sampleIndex2 = 0;

        // Print available heap memory after one loop
        // debugf("Free heap after one loop2: %u bytes\n", ESP.getFreeHeap());

        delay(500);
      }
    }
  }
}

void Task3code( void * Parameters ){
  debug("Task3 running on core ");
  debugln(xPortGetCoreID());

  // debugln("MAX6675 Starting......");
  // wait for MAX chip to stabilize
  delay(1000);

  // Print available heap memory before allocation
  // debugf("Free heap before Task3: %u bytes\n", ESP.getFreeHeap());

  for(;;){
    unsigned long startTime3 = millis(); // Start timing when beginning new sample batch

    float temper = thermocouple.readCelsius();
    float temperatureF = thermocouple.readFahrenheit();

    if(temper > 85){
      temperatureC = 85;
    }
    temperatureC = temper;

    unsigned long endTime3 = millis();
    unsigned long elapsedTime3 = endTime3 - startTime3;

    debugf("KTemperatureC: %.2f °C\n | KTemperatureF: %.2f °C\n | Time3: %lu ms\n", temperatureC, temperatureF, elapsedTime3);

    // Print available heap memory after one loop
    // debugf("Free heap after one loop4: %u bytes\n", ESP.getFreeHeap());

    delay(1000);
  }
}

void Task4code( void * Parameters ){
  debug("Task4 running on core ");
  debugln(xPortGetCoreID());

  pinMode(IR_PIN, INPUT);

  // debugln("MD0662 Starting.....");
  // wait for MD chip to stabilize
  delay(1000);

  // Print available heap memory before allocation
  // debugf("Free heap before Task4: %u bytes\n", ESP.getFreeHeap());

  for(;;){
    float irTempSum = 0.0;
    float irVoltage = 0.0;
    int irAdcRawSum = 0;

    unsigned long startTime4 = millis(); // Start timing when beginning new sample batch

    for (int i = 0; i < 10; i++) {
      int adc4 = analogRead(IR_PIN);
      irVoltage = (adc4 * ADC_REF_V) / ADC_MAX;
      float temp = (58.82 * irVoltage) + 6.94;
      irTempSum += temp;
      irAdcRawSum += adc4;
    }

    irTemperature = irTempSum / 10.0;
    float irAdcRaw = irAdcRawSum / 10.0;
    if (irTemperature < 6.94) irTemperature = 0;

    unsigned long endTime4 = millis();
    unsigned long elapsedTime4 = endTime4 - startTime4;

    debugf("IR Sensor -> ADC: %d | Temperature: %.2f °C\n | Time4: %lu ms\n", irAdcRaw, irTemperature, elapsedTime4);

    // Print available heap memory after one loop
    // debugf("Free heap after one loop4: %u bytes\n", ESP.getFreeHeap());

    delay(1000);
  }
}

void loop() {
  switch (state) {
    case _WIFI_CONNECT:
      setupWiFi();
      if (WiFi.status() == WL_CONNECTED) {
        setupMQTT();
        state = _MQTT_CONNECT;
      }
      break;

    case _MQTT_CONNECT:
      reconnectMQTT();
      if (client.connected()) {
        lastSendTime = millis();
        error_count = 0;
        state = _SEND_DATA;
      }
      break;

    case _SEND_DATA:
      if (WiFi.status() != WL_CONNECTED || !client.connected()) {
        state = _DISCONNECT;
        break;
      }
      client.loop();
      if (millis() - lastSendTime > sendInterval) {
        lastSendTime = millis();
        String payload = "{";
        payload += "\"currentCT11\": " + String(Ip1, 1) + ",";
        payload += "\"currentCT12\": " + String(Ip2, 1) + ",";
        payload += "\"temperatureK\": " + String(temperatureC, 1) + ",";
        payload += "\"temperatureIR\": " + String(irTemperature, 1);
        payload += "}";
        if (client.publish(MQTT_TELE_TOPIC, payload.c_str())) {
          debugln("Data sent: " + payload);
        } else {
          debugln("Failed to send data");
          error_count++;
          if (error_count >= Error_count_th) state = _DISCONNECT;
        }
      }
      break;

    case _DISCONNECT:
      WiFi.disconnect();
      client.disconnect();
      error_count = 0;
      delay(1000);
      state = _WIFI_CONNECT;
      break;
  }
}