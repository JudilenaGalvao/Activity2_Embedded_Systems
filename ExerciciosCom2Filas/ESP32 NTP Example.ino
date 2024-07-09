#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#define LED_PIN 2
//#define TEMPO_DEBOUNCE 500// Tempo de debounce em milissegundos (não está sendo usado)
volatile unsigned long timestamp_ultimo_acionamento = 0;

TaskHandle_t tarefaA;
TaskHandle_t tarefaB;

QueueHandle_t fila1;
QueueHandle_t fila2;

//SemaphoreHandle_t SMF1;
//SemaphoreHandle_t SMF2;

volatile int delayTime = 1000;
volatile int ledPiscaCount = 0;

void setup() {
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);// Configura o pino do LED como saída

  fila1 = xQueueCreate(10, sizeof(int));// Cria fila1 para inteiros
  fila2 = xQueueCreate(10, sizeof(char[50]));// Cria fila2 para strings de até 50 caracteres

//  SMF1 = xSemaphoreCreateBinary();
//  SMF2 = xSemaphoreCreateBinary();

  // Cria as tarefas com stack size de 4096 bytes, prioridade 1, e associadas ao núcleo 1
  xTaskCreatePinnedToCore(funcaoA, "tarefaA", 4096, NULL, 1, &tarefaA, 0);
  xTaskCreatePinnedToCore(funcaoB, "tarefaB", 4096, NULL, 1, &tarefaB, 1);
}

void loop() {
  vTaskDelete(NULL);
}

void funcaoA(void *arg) {
  char buffer[100];// Buffer para leitura da serial
  int bufferPos = 0;// Posição no buffer
  char mensagem[50];// Buffer para mensagens da fila2

  while (true) {
    // Verifica se há mensagens na fila2
    if (xQueueReceive(fila2, &mensagem, pdMS_TO_TICKS(10)) == pdPASS) {
      Serial.println(mensagem);// Imprime a mensagem recebida
    }

    // Leitura da serial
    while (Serial.available() > 0) {
      char c = Serial.read();// Lê um caractere da serial

      if (c == '\n' || c == '\r') {
        buffer[bufferPos] = '\0';// Finaliza a string no buffer
        Serial.println(buffer);// Exibe a entrada na serial

        // Verifica se a entrada contém a palavra "delay"
        if (strstr(buffer, "delay") == buffer) {
          char *ptr = buffer + 6; // Move o ponteiro para depois de "delay "
          int novoDelay = atoi(ptr); // Converte a string para inteiro

          // Envia o novo delay para a fila1
          if (novoDelay > 0) {
            xQueueSend(fila1, &novoDelay, pdMS_TO_TICKS(0));
          }
        }

        bufferPos = 0;// Reseta a posição do buffer para a próxima leitura
      } else {
        buffer[bufferPos++] = c; // Adiciona o caractere ao buffer
        if (bufferPos >= 100) bufferPos = 99; // para buffer passar do tamanho
      }
    }

    vTaskDelay(pdMS_TO_TICKS(100));// Delay para a próxima iteração
  }
}

void funcaoB(void *arg) {
  int novoDelayTime;
  char mensagem[50];

  while (true) {
    if (xQueueReceive(fila1, &novoDelayTime, pdMS_TO_TICKS(10)) == pdPASS) {
      delayTime = novoDelayTime;// Atualiza o tempo de delay
      ledPiscaCount = 0;
    }

    // Pisca o LED com o delay atual
    digitalWrite(LED_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(delayTime));
    digitalWrite(LED_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(delayTime));

    ledPiscaCount++;
    Serial.println(ledPiscaCount);

    // Após 10 piscadas, envia a mensagem para a fila2
    if (ledPiscaCount >= 10) {
      sprintf(mensagem, "Pisquei o LED %d vezes", ledPiscaCount);
      xQueueSend(fila2, &mensagem, pdMS_TO_TICKS(0));
      ledPiscaCount = 0;
    }
  }
}