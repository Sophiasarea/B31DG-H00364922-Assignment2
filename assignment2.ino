#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp32/rom/ets_sys.h"

// Declaring I/O pins
#define DIGITAL_PIN GPIO_NUM_8
#define LED_PIN 10
#define INPUT_PIN_1 1
#define INPUT_PIN_2 3
#define ANALOG_PIN  19
#define BUTTON_PIN 2
#define ERR_PIN 18
#define DEBOUNCE_TIME 50

//store the data collected from task 2 & 3
typedef struct
{
  int freq_task2;
  int freq_task3;
}FrequencyData;

FrequencyData freq_data;

// Semaphore to protect access to the frequency data structure
SemaphoreHandle_t frequencySemaphore;

// Queue for botto press events
QueueHandle_t buttonEventQueue;

// Task handles
TaskHandle_t Task1Handle, Task2Handle, Task3Handle, Task4Handle, Task5Handle, Task6Handle, Task7Handle, Task8Handle;

void setup() {
  // put your setup code here, to run once:
  // Initialise serial communication with the 96-- baud rate
  Serial.begin(9600);
  Serial.println(F("In Setup function"));

  // Digital signal
  pinMode(DIGITAL_PIN, OUTPUT);
  // Analog input
  pinMode(ANALOG_PIN, INPUT);
  // LED
  pinMode(LED_PIN, OUTPUT);
  pinMode(ERR_PIN, OUTPUT);
  // Push button
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  buttonEventQueue = xQueueCreate(1, sizeof(uint8_t));

  // Create semaphore
  frequencySemaphore = xSemaphoreCreateMutex();
  if(frequencySemaphore == NULL)
  {
    Serial.println("Semaphore creation failed.");
  }

  // Create tasks(function name, name, capacity of stack, NULL, priority, task handle)
  xTaskCreate(digitalSignal, "Task1", 1640, NULL, 1, &Task1Handle);
  xTaskCreate(freqMeasure1, "Task2", 1680, NULL, 5, &Task2Handle);
  xTaskCreate(freqMeasure2, "Task3", 1900, NULL, 5, &Task3Handle);
  xTaskCreate(analogRead, "Task4", 1640, NULL, 2, &Task4Handle);
  xTaskCreate(freqDisplay, "Task5", 1900, NULL, 2, &Task5Handle);
  xTaskCreate(monitorButton, "Task6", 1608, NULL, 3, &Task6Handle);
  xTaskCreate(controlLED, "Task7", 1648, NULL, 3, &Task7Handle);
  xTaskCreate(periodicTask, "Task8", 1660, NULL, 2, &Task8Handle);
}

void loop() {
  // put your main code here, to run repeatedly:
  // Since it is RTOs, the loop function is not used.
}

void digitalSignal(void* pvParameters)
{
  // Use gpio to control the digital output, since is more sensitive to high real-time performance task
  gpio_pad_select_gpio(DIGITAL_PIN);
  gpio_set_direction(DIGITAL_PIN, GPIO_MODE_OUTPUT);
  while(1)
  {
    // digitalWrite(DIGITAL_PIN, HIGH);
    gpio_set_level(DIGITAL_PIN, 1);
    ets_delay_us(180); // Use 
    gpio_set_level(DIGITAL_PIN, 0);
    ets_delay_us(40);
    gpio_set_level(DIGITAL_PIN, 1);
    ets_delay_us(530);
    gpio_set_level(DIGITAL_PIN, 0);
    ets_delay_us(3250);
  }

}

//task2 & 3
volatile unsigned long lastTime_task2 = 0; // last changing time
volatile unsigned long lastFrequency_task2 = 0; // last frequency

void IRAM_ATTR handleInterrupt_task2() {
    unsigned long currentTime = micros(); // get current time
    unsigned long interval = currentTime - lastTime_task2; // calculate the interval

    if (interval > 0) { 
        lastFrequency_task2 = 1000000 / interval; // calculate frequency
    }

    lastTime_task2 = currentTime; // upload changing time
}

void freqMeasure1(void *pvParameters) {
  pinMode(INPUT_PIN_1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INPUT_PIN_1), handleInterrupt_task2, RISING);

  while(1)
  {
    if (xSemaphoreTake(frequencySemaphore, portMAX_DELAY) == pdTRUE) 
    {
      freq_data.freq_task2 = lastFrequency_task2; // upload the frequency from interrupt
      xSemaphoreGive(frequencySemaphore); 
    }
    vTaskDelay(pdMS_TO_TICKS(20)); 
  }
}

volatile unsigned long lastTime_task3 = 0;
volatile unsigned long lastFrequency_task3 = 0; 

void IRAM_ATTR handleInterrupt_task3() {
    unsigned long currentTime = micros(); 
    unsigned long interval = currentTime - lastTime_task3;

    if (interval > 0) { 
        lastFrequency_task3 = 1000000 / interval; 
    }

    lastTime_task3 = currentTime;
}

void freqMeasure2(void *pvParameters) 
{
  pinMode(INPUT_PIN_2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INPUT_PIN_2), handleInterrupt_task3, RISING);

  while(1)
  {
    if (xSemaphoreTake(frequencySemaphore, portMAX_DELAY) == pdTRUE) 
    {
      freq_data.freq_task3 = lastFrequency_task3; 
      xSemaphoreGive(frequencySemaphore); 
    }
    vTaskDelay(pdMS_TO_TICKS(20)); 
  }
}

void analogRead(void* pvParameters)
{
  float readings[10] = {0};
  int readIndex = 0;
  float total = 0;
  float average = 0;
  while(1)
  {
    total -= readings[readIndex]; //deposit the oldest reading
    readings[readIndex] = analogRead(ANALOG_PIN); // new reading
    total += readings[readIndex]; // add new reading
    readIndex = (readIndex + 1) % 10; // move to the next index
    
    average = total / 10; // calculate the average

    if (average > 2047) {
        digitalWrite(ERR_PIN, HIGH); // if reach half of maximum, light on
    } else {
        digitalWrite(ERR_PIN, LOW); // or swith off the led
    }

    vTaskDelay(pdMS_TO_TICKS(20)); // delay 20ms
    // Serial.printf("Input Voltage avg(in V): %.2f \n",average);
  }
}

//task 5
int scaled_freq2, scaled_freq3;
int scale_frequency(int freq, int lower_bound, int upper_bound)
{
  if(freq <=  lower_bound) return 0;
  if (freq >= upper_bound) return 99;
  else return ((freq - lower_bound) * 99 / (upper_bound - lower_bound));
}
void freqDisplay(void* pvParameters)
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
      xSemaphoreGive(frequencySemaphore);

    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void monitorButton(void* pvParameters)
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
          uint8_t event = 1;
          xQueueSend(buttonEventQueue, &event, portMAX_DELAY);
        }
      }
    }

    lastButtonState = readButton;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void controlLED(void* pvParameters) {
  uint8_t event;
  pinMode(LED_PIN, OUTPUT);

  while (1) {
    // Receive button state from queue
    if (xQueueReceive(buttonEventQueue, &event, portMAX_DELAY) == pdTRUE) {
      // Switch LED state
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
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
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
