#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define LED_PIN 10
#define INPUT_PIN_1 5
#define INPUT_PIN_2 6
#define ANALOG_PIN 4
#define PUSH_BUTTON 2

#define DEBOUNCE_TIME 50
//store the data collected from task 2 & 3
typedef struct
{
  int freq_task2;
  int freq_task3;
}FrequencyData;

FrequencyData freq_data;
//semaphore to protect access to the frequency data structure
SemaphoreHandle_t frequencySemaphore;

//task handles
TaskHandle_t Task1Handle, Task2Handle, Task3Handle, Task4Handle, Task5Handle, Task6Handle, Task7Handle, Task8Handle;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println(F("In Setup function"));

  // led
  pinMode(LED_PIN, OUTPUT);
  // button
  pinMode(PUSH_BUTTON, INPUT_PULLUP);

  frequencySemaphore = xSemaphoreCreateMutex();
  if(frequencySemaphore == NULL)
  {
    Serial.println("Semaphore creation failed.");
  }

  //create tasks(function name, name, capacity of stack, NULL, priority, task handle)
  xTaskCreate(Task1, "Task1", 2048, NULL, 1, &Task1Handle);
  xTaskCreate(Task2, "Task2", 2048, NULL, 2, &Task2Handle);
  xTaskCreate(Task3, "Task3", 2048, NULL, 3, &Task3Handle);
  xTaskCreate(Task4, "Task4", 2048, NULL, 4, &Task4Handle);
  xTaskCreate(Task5, "Task5", 2048, NULL, 5, &Task5Handle);
  // xTaskCreate(Task6, "Task6", 2048, NULL, 6, &Task6Handle);
  xTaskCreate(Task7, "Task7", 2048, NULL, 7, &Task7Handle);
  xTaskCreate(periodicTask, "Task8", 2048, NULL, 8, &Task8Handle);
}

void loop() {
  // put your main code here, to run repeatedly:

}

void Task1(void* pvParameters)
{
  while(1)
  {
    digitalWrite(LED_PIN, HIGH);
    Serial.println(F("Task1"));
    vTaskDelay(0.18/portTICK_PERIOD_MS); //delay 180us
    digitalWrite(LED_PIN, LOW);
    vTaskDelay(0.04/portTICK_PERIOD_MS); //delay 40us
    digitalWrite(LED_PIN, HIGH);
    vTaskDelay(0.53/portTICK_PERIOD_MS); //delay 530us
    digitalWrite(LED_PIN, LOW);
    vTaskDelay(3.25/portTICK_PERIOD_MS); //delay 3.25ms
  }
}

//task2 & 3
volatile unsigned long lastTime_task2 = 0; // 上次信号变化的时间
volatile unsigned long lastFrequency_task2 = 0; // 最近一次计算的频率

void IRAM_ATTR handleInterrupt_task2() {
    unsigned long currentTime = micros(); // 获取当前时间
    unsigned long interval = currentTime - lastTime_task2; // 计算时间间隔

    if (interval > 0) { // 避免除以零
        lastFrequency_task2 = 1000000 / interval; // 计算频率（Hz）
    }

    lastTime_task2 = currentTime; // 更新上次变化的时间
}

void Task2(void *pvParameters) {
  pinMode(INPUT_PIN_1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INPUT_PIN_1), handleInterrupt_task2, CHANGE);

  while(1)
  {
    if (xSemaphoreTake(frequencySemaphore, portMAX_DELAY) == pdTRUE) 
    {
      freq_data.freq_task2 = lastFrequency_task2; // 从中断服务程序更新频率值
      xSemaphoreGive(frequencySemaphore); // 释放信号量
    }
    vTaskDelay(pdMS_TO_TICKS(20)); // 保持任务同步，每20毫秒更新一次
  }
}

volatile unsigned long lastTime_task3 = 0; // 上次信号变化的时间
volatile unsigned long lastFrequency_task3 = 0; // 最近一次计算的频率

void IRAM_ATTR handleInterrupt_task3() {
    unsigned long currentTime = micros(); // 获取当前时间
    unsigned long interval = currentTime - lastTime_task3; // 计算时间间隔

    if (interval > 0) { // 避免除以零
        lastFrequency_task3 = 1000000 / interval; // 计算频率（Hz）
    }

    lastTime_task3 = currentTime; // 更新上次变化的时间
}

void Task3(void *pvParameters) 
{
  pinMode(INPUT_PIN_2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INPUT_PIN_2), handleInterrupt_task3, CHANGE);

  while(1)
  {
    if (xSemaphoreTake(frequencySemaphore, portMAX_DELAY) == pdTRUE) 
    {
      freq_data.freq_task3 = lastFrequency_task3; // 从中断服务程序更新频率值
      xSemaphoreGive(frequencySemaphore); // 释放信号量
    }
    vTaskDelay(pdMS_TO_TICKS(20)); // 调整为与task2相同的延时以保持一致性
  }
}

void Task4(void* pvParameters)
{
  int readings[10] = {0};
  int readIndex = 0;
  long total = 0;
  long average = 0;
  while(1)
  {
    total -= readings[readIndex]; //deposit the oldest reading
    readings[readIndex] = analogRead(ANALOG_PIN); // new reading
    total += readings[readIndex]; // add new reading
    readIndex = (readIndex + 1) % 10; // move to the next index
    
    average = total / 10; // calculate the average

    if (average > 2047) {
        digitalWrite(LED_PIN, HIGH); // if reach half of miximum, light on
    } else {
        digitalWrite(LED_PIN, LOW); // or swith off the led
    }

    vTaskDelay(20/portTICK_PERIOD_MS); // delay 20ms
  }
}

//task 5
int scaled_freq2, scaled_freq3;
int scale_frequency(int freq, int lower_bound, int upper_bound)
{
  if(freq <=  lower_bound) return 0;
  if (freq >= upper_bound) return 99;
  return (freq - lower_bound) * 99 / (upper_bound - lower_bound);
}
void log_frequencies(int freq_task2, int freq_task3) {
    scaled_freq2 = scale_frequency(freq_task2, 333, 1000); // 任务2的缩放
    scaled_freq3 = scale_frequency(freq_task3, 500, 1000); // 任务3的缩放
}
void Task5(void* pvParameters)
{
  while(1)
  {
    if(xSemaphoreTake(frequencySemaphore, portMAX_DELAY) == pdTRUE)
    {
      int task2Freq = freq_data.freq_task2;
      int task3Freq = freq_data.freq_task3;

      scaled_freq2 = scale_frequency(task2Freq, 333, 1000);
      scaled_freq3 = scale_frequency(task3Freq, 500, 1000);

      Serial.print("Task 2 Frequency: ");
      Serial.print(scaled_freq2);
      Serial.print(", Task 3 Frequency: ");
      Serial.println(scaled_freq3);
      xSemaphoreGive(frequencySemaphore); // 在打印之后释放信号量

    }
    vTaskDelay(200/portTICK_PERIOD_MS);
  }
}

void Task7(void* pvParameters)
{
  int buttonState = HIGH; // current state
  int lastButtonState = HIGH; // last state
  unsigned long lastDebounceTime = 0; // last time of state changing

  while (1) {
    int readButton = digitalRead(BUTTON_PIN);
    
    // test if the state is changed
    if (readButton != lastButtonState) {
      lastDebounceTime = millis();
    }
    
    if ((millis() - lastDebounceTime) > DEBOUNCE_TIME) {
      // if the state is stable after debounce time, uplode the button state
      if (readButton != buttonState) {
        buttonState = readButton;

        // if button is pressed, switch the state of LED
        if (buttonState == LOW) {
          digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        }
      }
    }

    lastButtonState = readButton;
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void CPU_work(int time)
{
  volatile long i = 0;
  long iterations = time * 10000;
  for (i = 0; i < iterations; i++)
  {

  }
}

void periodicTask(void *pvParameters)
{
  while(1)
  {
    CPU_work(2);
    vTaskDelay(20/portTICK_PERIOD_MS);
  }
}
