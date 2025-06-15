/*============================================================

Project name : Machine Monitoring System
Project purpose : Preventive Maintenance, Monitoring
Description: System includes 
                2 x CT sensor (600A)
                1 x K Type Thermocouple 
                1 x IR Temperature Sensor
                Data sent every 1 second to ThingsBoard
                4 Tasks running on ESP32
                Sending Data over 4G
Last updated date : 04/06/2025
Author: 

*============================================================/

/*--------Libraries------------*/
#include <IIOTDEVKIT4G.h>
#include "max6675.h"
#include "Arduino.h"
/*----PIN ASSIGHNING------*/
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

/*-----------MQTT configuration-------------*/
#define Debug 0
IIOTDEVKIT4G IIOT_Dev_kit;

//========================================
bool G_start_connect() {
  uint8_t start_count = 0;
  bool init = false;
  do {
    init = IIOT_Dev_kit.Init(115200);
    start_count++;
  } while (!init && start_count < 5);

  if (!init) {
    Serial.println("init failed");
    return false;
  }

  long starttime = millis();
  while (!IIOT_Dev_kit.IS_ATTACH() && millis() - starttime < 20000) {
    Serial.print(".");
    delay(10);
  }
  if (!IIOT_Dev_kit.IS_ATTACH()) {
    Serial.println("PDP attach error");
    return false;
  }

  Serial.println("PDP attached");
  return true;
}
//========================================
Broker TB_Broker0;

#define MQTT_SERVER "thingsboard.cloud"
#define MQTT_PORT "1883"
#define MQTT_CLIENTID "DW1"
#define MQTT_USERNAME "D03"
#define MQTT_TELE_TOPIC "v1/devices/me/telemetry"

bool connected = false;
bool MQTT_STARTED = false;

#define _ON_Init_4G 1
#define _MQTT_START 2
#define _MQTT_Connect 3

#define _send_data 4
#define _MQTT_Disconnect 5
#define _MQTT_Stop 6
#define _off_4G 7

#define _fst_send_data 8

void Task1code( void * Parameters );
void Task2code( void * Parameters );
void Task3code( void * Parameters );
void Task4code( void * Parameters );

int state;

int error_count = 0;
#define Error_count_th 5

unsigned long lastSendTime = 0;
const unsigned long sendInterval = 1000;

void setup(){
  Serial.begin(115200);
  delay(1000);

  Serial.println("Starting......");
  IIOT_Dev_kit.PWRDOWN();
  delay(1000);

  state = _ON_Init_4G;
  delay(100);

  xTaskCreatePinnedToCore(
    Task1code, /* Task function. */
    "CT1",     /* name of task. */
    10000,     /* Stack size of task */
    NULL,      /* parameter of the task */
    1,         /* priority of the task */
    &CT1,      /* Task handle to keep track of created task */
    1          /* pin task to core 0 */
  ); 

  xTaskCreatePinnedToCore(
    Task2code,  /* Task function. */
    "CT2",      /* name of task. */
    10000,      /* Stack size of task */
    NULL,       /* parameter of the task */
    1,          /* priority of the task */
    &CT2,       /* Task handle to keep track of created task */
    1           /* pin task to core 0 */      
  );

    xTaskCreatePinnedToCore(
    Task3code,  /* Task function. */
    "Thermo",   /* name of task. */
    10000,      /* Stack size of task */
    NULL,       /* parameter of the task */
    1,          /* priority of the task */
    &Thermo,    /* Task handle to keep track of created task */
    1           /* pin task to core 0 */
  ); 

  xTaskCreatePinnedToCore(
    Task4code,  /* Task function. */
    "IR",       /* name of task. */
    10000,      /* Stack size of task */
    NULL,       /* parameter of the task */
    1,          /* priority of the task */
    &IR,        /* Task handle to keep track of created task */
    1           /* pin task to core 0 */      
  );

  delay(100);
}

void Task1code( void * Parameters ){
  // Serial.print("Task1 running on core ");
  // Serial.println(xPortGetCoreID());

  pinMode(CT1_PIN, INPUT); 

  // Print available heap memory before allocation
  // Serial.printf("Free heap before malloc1: %u bytes\n", ESP.getFreeHeap());

  voltageSamples1 = (float*) malloc(NUM_SAMPLES * sizeof(float));
  if (voltageSamples1 == nullptr) {
    Serial.println("Memory1 allocation failed!");
    while (true); // Halt
  }

  // Print available heap memory after allocation
  // Serial.printf("Free heap after malloc1: %u bytes\n", ESP.getFreeHeap());

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

        // Serial.printf("Vavg1: %.3f V, Vrms1: %.3f V, Is1: %.3f A, Ip1: %.3f A, Time1: %lu ms\n", Vavg1, Vrms1, Is1, Ip1, elapsedTime1);
        
        // Reset sample index to reuse memory
        sampleIndex1 = 0;

        // Print available heap memory after one loop
        // Serial.printf("Free heap after one loop1: %u bytes\n", ESP.getFreeHeap());

        delay(100);
      }
    }
  }
}

void Task2code( void * Parameters ){
  // Serial.print("Task2 running on core ");
  // Serial.println(xPortGetCoreID());

  pinMode(CT2_PIN, INPUT);

  // Print available heap memory before allocation
  // Serial.printf("Free heap before malloc2: %u bytes\n", ESP.getFreeHeap());

  voltageSamples2 = (float*) malloc(NUM_SAMPLES * sizeof(float));
  if (voltageSamples2 == nullptr) {
    Serial.println("Memory2 allocation failed!");
    while (true); // Halt
  }

  // Print available heap memory after allocation
  // Serial.printf("Free heap after malloc2: %u bytes\n", ESP.getFreeHeap());

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

        // Serial.printf("Vavg2: %.3f V, Vrms2: %.3f V, Is2: %.3f A, Ip2: %.3f A, Time2: %lu ms\n", Vavg2, Vrms2, Is2, Ip2, elapsedTime2);
        
        // Reset sample index to reuse memory
        sampleIndex2 = 0;

        // Print available heap memory after one loop
        // Serial.printf("Free heap after one loop2: %u bytes\n", ESP.getFreeHeap());

        delay(100);
      }
    }
  }
}

void Task3code( void * Parameters ){
  // Serial.print("Task3 running on core ");
  // Serial.println(xPortGetCoreID());

  // Serial.println("MAX6675 Starting......");
  // wait for MAX chip to stabilize
  delay(1000);

  // Print available heap memory before allocation
  // Serial.printf("Free heap before Task3: %u bytes\n", ESP.getFreeHeap());

  for(;;){
    // unsigned long startTime3 = millis(); // Start timing when beginning new sample batch

    float temper = thermocouple.readCelsius();
    float temperatureF = thermocouple.readFahrenheit();

    if(temper > 85){
      temperatureC = 85;
    }
    temperatureC = temper;

    // unsigned long endTime3 = millis();
    // unsigned long elapsedTime3 = endTime3 - startTime3;

    // Serial.printf("KTemperatureC: %.2f °C\n | KTemperatureF: %.2f °C\n | Time3: %lu ms\n", temperatureC, temperatureF, elapsedTime3);

    // Print available heap memory after one loop
    // Serial.printf("Free heap after one loop4: %u bytes\n", ESP.getFreeHeap());

    delay(500);
  }
}

void Task4code( void * Parameters ){
  // Serial.print("Task4 running on core ");
  // Serial.println(xPortGetCoreID());

  pinMode(IR_PIN, INPUT);

  // Serial.println("MD0662 Starting.....");
  // wait for MD chip to stabilize
  delay(1000);

  // Print available heap memory before allocation
  // Serial.printf("Free heap before Task4: %u bytes\n", ESP.getFreeHeap());

  for(;;){
    float irTempSum = 0.0;
    float irVoltage = 0.0;
    int irAdcRawSum = 0;

    // unsigned long startTime4 = millis(); // Start timing when beginning new sample batch

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

    // unsigned long endTime4 = millis();
    // unsigned long elapsedTime4 = endTime4 - startTime4;

    // Serial.printf("IR Sensor -> ADC: %d | Temperature: %.2f °C\n | Time4: %lu ms\n", irAdcRaw, irTemperature, elapsedTime4);

    // Print available heap memory after one loop
    // Serial.printf("Free heap after one loop4: %u bytes\n", ESP.getFreeHeap());

    delay(100);
  }
}

void loop(){
  delay(100);

  switch (state) {
    case _ON_Init_4G:
      Serial.println("state - _ON_Init_4G");
      if (G_start_connect()) {
        state = _MQTT_START;
      } else {
        state = _off_4G;
      }
      break;

    case _MQTT_START:
      Serial.println("state - _MQTT_START");
      if (IIOT_Dev_kit.MQTT_SETUP(&TB_Broker0, MQTT_SERVER, MQTT_PORT)) {
        Serial.println("MQTT START");
        state = _MQTT_Connect;
      } else {
        Serial.println("start error");
        state = _off_4G;
      }
      break;

    case _MQTT_Connect:
      Serial.println("state - _MQTT_Connect");
      if (IIOT_Dev_kit.MQTT_CONNECT(&TB_Broker0, MQTT_CLIENTID, MQTT_USERNAME)) {
        Serial.println("Connect to server");
        state = _send_data;
        error_count = 0;
        lastSendTime = millis(); // Ensure timing is reset after connection
      } else {
        Serial.println("connect error");
        error_count++;
        if (error_count >= Error_count_th) {
          state = _off_4G;
        } else {
          delay(100); // Small delay before retry
          state = _MQTT_Connect;
        }
      }
      break;

    case _send_data:
      if (millis() - lastSendTime >= sendInterval) {
        //Serial.println("state - _send_data");
        lastSendTime = millis();

        // IIOT_Dev_kit.CSQ();

        float ct1 = Ip1;
        float ct2 = Ip2;
        float temp = temperatureC;
        float irtemp = irTemperature;

        String data = "{";
        data += "\"currentCT31\": " + String(ct1, 1) + ",";
        data += "\"currentCT32\": " + String(ct2, 1) + ",";
        data += "\"temperatureK3\": " + String(temp, 1) + ",";
        data += "\"temperatureIR3\": " + String(irtemp, 1);
        data += "}";

        Serial.println("Publishing: " + data);
        if (IIOT_Dev_kit.MQTT_PUB(&TB_Broker0, MQTT_TELE_TOPIC, data)) {
          Serial.println("Data sent");
          error_count = 0;
        } else {
          error_count++;
          if(error_count >= Error_count_th){
            error_count = 0;
            Serial.println("Failed to send data");
            state = _MQTT_Disconnect;
          }
        }
      }
      break;
    
    case _MQTT_Disconnect:
      Serial.println("state - _MQTT_Disconnect");
      if (IIOT_Dev_kit.MQTT_DISCONNECT(&TB_Broker0)) {
        Serial.println("MQTT Disconnect");
      } else {
        Serial.println("disconnect error");
      }
      state = _MQTT_Stop;
      break;

    case _MQTT_Stop:
      Serial.println("state - _MQTT_Stop");
      if (IIOT_Dev_kit.MQTT_STOP()) {
        Serial.println("MQTT STOP");
      } else {
        Serial.println("MQTT stop error");
      }
      state = _off_4G;
      break;

    case _off_4G:
      Serial.println("state - _off_4G");
      error_count = 0;
      IIOT_Dev_kit.PWRDOWN();
      delay(1000);
      state = _ON_Init_4G;
      break;
  }
}