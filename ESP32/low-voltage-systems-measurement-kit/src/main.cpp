#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// ----------------------------
// CONFIGURAÇÃO DOS SENSORES
// ----------------------------
const int voltagePin1 = 34;
const float voltageCalibration1 = 1410;
const float offsetVadc1 = 1679;

const int currentPin1 = 26;
const float currentSensitivity1 = 0.0373;
const float offsetIadc1 = 1686;

const int voltagePin2 = 35;
const float voltageCalibration2 = 1410;
const float offsetVadc2 = 1679;

const int currentPin2 = 25;
const float currentSensitivity2 = 0.0543;
const float offsetIadc2 = 1676;

const int voltagePin3 = 32;
const float voltageCalibration3 = 1410;
const float offsetVadc3 = 1672;

const int currentPin3 = 33;
const float currentSensitivity3 = 0.0564;
const float offsetIadc3 = 1662;

// ----------------------------
// CONSTANTES DO ADC
// ----------------------------
const float VCC_VOLTAGE = 3.0;
const float ADC_RESOLUTION = 4095.0;
const float voltageWaveformFactor = 1000;

// ----------------------------
// ESTRUTURAS E BUFFERS
// ----------------------------
#define BUFFER_SIZE 512
#define QUEUE_SIZE 10

struct SensorData {
  unsigned long timestamp;
  float v1, i1, v2, i2, v3, i3;
};

struct VoltageReadings {
  unsigned long timestamp;
  float v1, v2, v3;
};

struct CurrentReadings {
  unsigned long timestamp;
  float i1, i2, i3;
};

// Buffers e controles
SensorData buffer[BUFFER_SIZE];
volatile int bufferIndex = 0;
volatile bool sampleFlag = false;

// Queues para comunicação entre cores
QueueHandle_t voltageQueue;
QueueHandle_t currentQueue;
QueueHandle_t dataQueue;

// Semáforos e mutex
SemaphoreHandle_t bufferMutex;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Timer
hw_timer_t *timer = NULL;
volatile unsigned long isrMicros = 0;

// ----------------------------
// INTERRUPÇÃO DO TIMER
// ----------------------------
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  isrMicros = micros();
  sampleFlag = true;
  portEXIT_CRITICAL_ISR(&timerMux);
}

// ----------------------------
// TASK CORE 0: LEITURA DE TENSÕES
// ----------------------------
void voltageReadingTask(void *parameter) {
  VoltageReadings voltageData;
  
  while(1) {
    if(sampleFlag) {
      // Capturar timestamp
      portENTER_CRITICAL(&timerMux);
      voltageData.timestamp = isrMicros;
      portEXIT_CRITICAL(&timerMux);
      
      // Leitura otimizada das tensões (core 0)
      int rawV1 = analogRead(voltagePin1);
      int rawV2 = analogRead(voltagePin2);  
      int rawV3 = analogRead(voltagePin3);
      
      // Conversão das tensões
      float vCentered1 = (rawV1 - offsetVadc1) * VCC_VOLTAGE / ADC_RESOLUTION;
      voltageData.v1 = vCentered1 * voltageWaveformFactor;
      
      float vCentered2 = (rawV2 - offsetVadc2) * VCC_VOLTAGE / ADC_RESOLUTION;
      voltageData.v2 = vCentered2 * voltageWaveformFactor;
      
      float vCentered3 = (rawV3 - offsetVadc3) * VCC_VOLTAGE / ADC_RESOLUTION;
      voltageData.v3 = vCentered3 * voltageWaveformFactor;
      
      // Enviar para queue (non-blocking)
      xQueueSend(voltageQueue, &voltageData, 0);
    }
    
    // Pequeno delay para não sobrecarregar
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

// ----------------------------
// TASK CORE 1: LEITURA DE CORRENTES
// ----------------------------
void currentReadingTask(void *parameter) {
  CurrentReadings currentData;
  
  while(1) {
    if(sampleFlag) {
      // Capturar timestamp
      portENTER_CRITICAL(&timerMux);
      currentData.timestamp = isrMicros;
      sampleFlag = false; // Reset flag após ambas as tasks lerem
      portEXIT_CRITICAL(&timerMux);
      
      // Leitura otimizada das correntes (core 1)
      int rawI1 = analogRead(currentPin1);
      int rawI2 = analogRead(currentPin2);
      int rawI3 = analogRead(currentPin3);
      
      // Conversão das correntes
      float iCentered1 = (rawI1 - offsetIadc1) * VCC_VOLTAGE / ADC_RESOLUTION;
      currentData.i1 = iCentered1 / currentSensitivity1;
      
      float iCentered2 = (rawI2 - offsetIadc2) * VCC_VOLTAGE / ADC_RESOLUTION;
      currentData.i2 = iCentered2 / currentSensitivity2;
      
      float iCentered3 = (rawI3 - offsetIadc3) * VCC_VOLTAGE / ADC_RESOLUTION;
      currentData.i3 = iCentered3 / currentSensitivity3;
      
      // Enviar para queue (non-blocking)
      xQueueSend(currentQueue, &currentData, 0);
    }
    
    // Pequeno delay para não sobrecarregar
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

// ----------------------------
// TASK CORE 1: PROCESSAMENTO E ENVIO DE DADOS
// ----------------------------
void dataProcessingTask(void *parameter) {
  VoltageReadings voltageData;
  CurrentReadings currentData;
  SensorData combinedData;
  
  while(1) {
    // Esperar por dados de tensão e corrente
    if(xQueueReceive(voltageQueue, &voltageData, portMAX_DELAY) == pdTRUE &&
       xQueueReceive(currentQueue, &currentData, portMAX_DELAY) == pdTRUE) {
      
      // Combinar dados (usar timestamp mais recente)
      combinedData.timestamp = max(voltageData.timestamp, currentData.timestamp);
      combinedData.v1 = voltageData.v1;
      combinedData.v2 = voltageData.v2;
      combinedData.v3 = voltageData.v3;
      combinedData.i1 = currentData.i1;
      combinedData.i2 = currentData.i2;
      combinedData.i3 = currentData.i3;
      
      // Armazenar no buffer com proteção
      if(xSemaphoreTake(bufferMutex, portMAX_DELAY) == pdTRUE) {
        buffer[bufferIndex] = combinedData;
        bufferIndex++;
        
        // Enviar quando buffer estiver cheio
        if(bufferIndex >= BUFFER_SIZE) {
          for(int i = 0; i < bufferIndex; i++) {
            Serial.print(buffer[i].timestamp);
            Serial.print(", ");
            Serial.print(buffer[i].v1, 0);
            Serial.print(", ");
            Serial.print(buffer[i].i1, 2);
            Serial.print(", ");
            Serial.print(buffer[i].v2, 0);
            Serial.print(", ");
            Serial.print(buffer[i].i2, 2);
            Serial.print(", ");
            Serial.print(buffer[i].v3, 0);
            Serial.print(", ");
            Serial.println(buffer[i].i3, 2);
          }
          bufferIndex = 0;
        }
        
        xSemaphoreGive(bufferMutex);
      }
    }
  }
}

// ----------------------------
// SETUP
// ----------------------------
void setup() {
  Serial.begin(921600); // Alterar para baud rate usado em py
  delay(1000);
  
  // Headers CSV
  Serial.println("timestamp_us,V1,I1,V2,I2,V3,I3");
  
  // Configurar pinos
  pinMode(voltagePin1, INPUT);
  pinMode(currentPin1, INPUT);
  pinMode(voltagePin2, INPUT);
  pinMode(currentPin2, INPUT);
  pinMode(voltagePin3, INPUT);
  pinMode(currentPin3, INPUT);
  
  // Criar queues e semáforos
  voltageQueue = xQueueCreate(QUEUE_SIZE, sizeof(VoltageReadings));
  currentQueue = xQueueCreate(QUEUE_SIZE, sizeof(CurrentReadings));
  bufferMutex = xSemaphoreCreateMutex();
  
  // Configurar timer de 1kHz
  timer = timerBegin(0, 80, true);             // 80 MHz / 80 = 1 MHz
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000, true);          // 1000 µs = 1 ms = 1 kHz
  timerAlarmEnable(timer);
  
  // Criar tasks em cores específicos
  // Core 0: Leitura de tensões
  xTaskCreatePinnedToCore(
    voltageReadingTask,
    "VoltageTask",
    4096,        // Stack size
    NULL,        // Parameters
    2,           // Priority
    NULL,        // Task handle
    0            // Core 0
  );
  
  // Core 1: Leitura de correntes
  xTaskCreatePinnedToCore(
    currentReadingTask,
    "CurrentTask", 
    4096,        // Stack size
    NULL,        // Parameters
    2,           // Priority
    NULL,        // Task handle
    1            // Core 1
  );
  
  // Core 1: Processamento e envio (pode compartilhar com correntes)
  xTaskCreatePinnedToCore(
    dataProcessingTask,
    "DataTask",
    8192,        // Stack size maior para Serial
    NULL,        // Parameters
    1,           // Priority menor
    NULL,        // Task handle
    1            // Core 1
  );
  
  Serial.println("Sistema inicializado com dual core!");
}

// ----------------------------
// LOOP PRINCIPAL (praticamente vazio)
// ----------------------------
void loop() {
  // O loop principal fica livre para outras tarefas
  // Todo o processamento está nas tasks
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  


}