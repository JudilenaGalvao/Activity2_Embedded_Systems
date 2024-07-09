#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

TimerHandle_t timerHandle;
TaskHandle_t taskIntReadings;
TaskHandle_t taskIntToVoltage;
TaskHandle_t taskPlotVoltage;
TaskHandle_t taskControlLed;

QueueHandle_t queueIntReadings;
QueueHandle_t queueVoltage;

void setup() {
  Serial.begin(9600);
  pinMode(34, INPUT);
  pinMode(2, OUTPUT);

  timerHandle = xTimerCreate("taskIntReadings",pdMS_TO_TICKS(300),pdTRUE,(void*)0,timerCallback);

  if (timerHandle == NULL) {
    Serial.println("Erro ao criar o timer");
  } else {
    if (xTimerStart(timerHandle, 0) != pdPASS) {
      Serial.println("Erro ao iniciar o timer");
    }
  }

  queueIntReadings = xQueueCreate( 10, sizeof( int ) );
  queueVoltage = xQueueCreate( 10, sizeof( double ) );
   
  xTaskCreatePinnedToCore(readPotenciometer, "taskIntReadings" , 4096 , NULL, 1 , &taskIntReadings,0);
  xTaskCreatePinnedToCore(ConvertToVoltage, "taskIntToVoltage" , 4096 , NULL, 1 , NULL, 1);
  xTaskCreatePinnedToCore(plottInTheSerial, "taskPlotVoltage" , 4096 , NULL, 1 , NULL,0);
  xTaskCreatePinnedToCore(ledController, "taskControlLed" , 4096 , NULL, 1, NULL,1);
}

void readPotenciometer(void *arg){
  int value;
  while(true){
    value = analogRead(34);
    value = map(value, 0, 4095, 0, 4095);
    xQueueSend(queueIntReadings, &value, pdMS_TO_TICKS(0));
    vTaskSuspend(NULL);
  }
}

void ConvertToVoltage(void *parameter) {
  int intValue;
  double voltage;
  while(1){
    if (xQueueReceive(queueIntReadings, &intValue, portMAX_DELAY)) {
      voltage = intValue * (3.3 / 4096);
      xQueueSend(queueVoltage, &voltage, portMAX_DELAY);
    }
  }
}

void plottInTheSerial(void *parameter) {
  double mensage;
  while(1){
    if (xQueueReceive(queueVoltage, &mensage, portMAX_DELAY)) {
      Serial.println(mensage);
    }
  }
}

void ledController(void *parameter) {
  double mensage;
  while(1){
    if (xQueueReceive(queueVoltage, &mensage, portMAX_DELAY)) {
      if (mensage > 1.5) {
        digitalWrite(2, HIGH);
      } else {
        digitalWrite(2, LOW);
      }
    }
  }
}

void timerCallback(TimerHandle_t xTimer) {
  vTaskResume(taskIntReadings);
}

void loop() {
  vTaskDelete(NULL);
}