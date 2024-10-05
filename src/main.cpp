#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

const int timer_div_us = 80;
const int dht11_pin = 2;
const int led_pin = 38; //48
const int la1_pin = 36;
const int la2_pin = 37;
const int btn_pin = 0;

const uint8_t flag_id = (1 << 0);

typedef struct {
  int16_t temperature;
  int16_t humidity;
} sensor_data_t;

hw_timer_t *timer0_cfg = nullptr;
QueueHandle_t queue_uart, queue_dht11_timestamp;
EventGroupHandle_t event_group;
BLECharacteristic *pTempCharacteristic;
BLECharacteristic *pHumidCharacteristic;
bool deviceConnected = false;


#define SERVICE_UUID           "fcadd1ed-a33e-478a-a9d5-48c88c5062ee"
#define TEMP_CHAR_UUID         "6a36b2d3-55d6-4dd4-9cba-e030db78a88a"
#define HUMID_CHAR_UUID        "91005b3e-4715-4e29-9419-001a82256bf6"

// Callback
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    }

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

void handle_error() {
  Serial.println("Error occurred!");
  digitalWrite(led_pin, HIGH);
  delay(1000);
  digitalWrite(led_pin, LOW);
}

void IRAM_ATTR dht11_pin_isr() {
  uint64_t value = timerRead(timer0_cfg);
  BaseType_t higher_priority_task_woken = pdFALSE;
  if (xQueueSendFromISR(queue_dht11_timestamp, &value, &higher_priority_task_woken) != pdPASS)
    handle_error();
}

void IRAM_ATTR timer0_isr() {
  pinMode(dht11_pin, INPUT_PULLUP);
  timerAlarmDisable(timer0_cfg);
}

void dht11_measurement(int16_t *temperature, int16_t *humidity) {
  // 
  uint64_t value, last = 0;
  bool started = false;
  uint32_t low_portion = 0;
  uint8_t val = 0, humidity1, humidity2, temperature1, temperature2;
  uint32_t count = 0;
  uint8_t retry = 0;

  while (xQueueReceive(queue_dht11_timestamp, &value, 0) == pdPASS); //Drain timestamp

  pinMode(dht11_pin, OUTPUT);
  digitalWrite(dht11_pin, LOW);
  timerWrite(timer0_cfg, 0);
  timerAlarmWrite(timer0_cfg, 18000, true);
  timerAlarmEnable(timer0_cfg);

  while (count < 40) {
    if (xQueueReceive(queue_dht11_timestamp, &value, portMAX_DELAY) != pdPASS) {
      handle_error();
      return;
    }

    uint32_t duration = value - last;
    if (duration == 0) {
      handle_error();
      return;
    }
    last = value;

    if (!started) {
      if (low_portion > 60 && low_portion < 100 && duration > 60 && duration < 100) {
        started = true;
        low_portion = 0;
      } else {
        low_portion = duration;
        retry++;
        if (retry == 20) {
          handle_error();
          return;
        }
      }
    } else {
      if (low_portion == 0)
        low_portion = duration;
      else {
        if (low_portion > 30 && low_portion < 70 && duration > 10 && duration < 45) {
          val <<= 1;
          count++;
        } else if (low_portion > 30 && low_portion < 70 && duration > 50 && duration < 90) {
          val <<= 1;
          val |= 1;
          count++;
        } else {
          handle_error();
          return;
        }
        low_portion = 0;

        if (count == 8) {
          humidity1 = val;
          val = 0;
        } else if (count == 16) {
          humidity2 = val;
          val = 0;
        } else if (count == 24) {
          temperature1 = val;
          val = 0;
        } else if (count == 32) {
          temperature2 = val;
          val = 0;
        } else if (count == 40) {
          if (humidity1 + humidity2 + temperature1 + temperature2 != val) {
            handle_error();
            return;
          } else {
            *humidity = (humidity1 << 8) | humidity2;
            *temperature = (temperature1 << 8) | temperature2;
          }
        }
      }
    }
  }
}

void task_heartbeat(void *args) {
  pinMode(la1_pin, OUTPUT);
  for (;;) {
    digitalWrite(la1_pin, !digitalRead(la1_pin));
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

void task_fastbeat(void *args) {
  pinMode(la2_pin, OUTPUT);
  for (;;) {
    vTaskDelay(1L / portTICK_PERIOD_MS);
    digitalWrite(la2_pin, !digitalRead(la2_pin));
  }
}


void task_id(void *args) {
  uint8_t btn_last = HIGH;
  pinMode(btn_pin, INPUT_PULLUP);  

  for (;;) {
    uint8_t btn = digitalRead(btn_pin);
    if (btn == LOW && btn_last == HIGH) {  // Button press detected (LOW signal)
      Serial.println("Button Pressed!");  // Debugging print for button press

      if (xEventGroupSetBits(event_group, flag_id) != pdPASS) {
        handle_error();
      }
    }
    btn_last = btn;
    vTaskDelay(10L / portTICK_PERIOD_MS);  
  }
}

void task_dht11(void *args) {
  pinMode(dht11_pin, INPUT_PULLUP);
  attachInterrupt(dht11_pin, dht11_pin_isr, CHANGE);
  timer0_cfg = timerBegin(0, timer_div_us, true);
  timerAttachInterrupt(timer0_cfg, &timer0_isr, true);

  while (true) {
    int16_t temperature = INT16_MIN, humidity = INT16_MIN;

    vTaskDelay(5000 / portTICK_PERIOD_MS);
    if (deviceConnected) {
      dht11_measurement(&temperature, &humidity);
      if (temperature > INT16_MIN && humidity > INT16_MIN) {
        sensor_data_t data;
        data.temperature = temperature;
        data.humidity = humidity;
        if (xQueueSend(queue_uart, &data, portMAX_DELAY) != pdPASS)
          handle_error();

        // Convert temperature and humidity to human-readable
        char tempStr[10];
        char humidStr[10];
        snprintf(tempStr, sizeof(tempStr), "%d.%01dC", temperature >> 8, temperature & 0xff);
        snprintf(humidStr, sizeof(humidStr), "%d.%01d%%", humidity >> 8, humidity & 0xff);

        // Send BLE notific
        pTempCharacteristic->setValue(tempStr);
        pTempCharacteristic->notify();

        pHumidCharacteristic->setValue(humidStr);
        pHumidCharacteristic->notify();
      }
    }
  }
}

void task_uart(void *args) {
  sensor_data_t data;
  uint32_t idx = 0;

  Serial.begin(115200);
  while (!Serial);

  while (true) {
    while (xQueueReceive(queue_uart, &data, 0) == pdPASS) {
      Serial.printf("Sensor Node - FreeRTOS: %3u: %u.%01u%% %u.%01uC\n", idx, data.humidity >> 8, data.humidity & 0xff, data.temperature >> 8, data.temperature & 0xff);
      idx++;
    }
    EventBits_t bits = xEventGroupClearBits(event_group, flag_id);
    if (bits & flag_id == flag_id) {
      Serial.println("Hello from polzert!");
    }
    vTaskDelay(10L / portTICK_PERIOD_MS);
  }
}

void setup_ble() {
  BLEDevice::init("Benjamin_HW_io24m025");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pTempCharacteristic = pService->createCharacteristic(
                          TEMP_CHAR_UUID,
                          BLECharacteristic::PROPERTY_NOTIFY
                        );
  pTempCharacteristic->addDescriptor(new BLE2902());

  pHumidCharacteristic = pService->createCharacteristic(
                          HUMID_CHAR_UUID,
                          BLECharacteristic::PROPERTY_NOTIFY
                        );
  pHumidCharacteristic->addDescriptor(new BLE2902());

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->start();
}

void setup() {
  neopixelWrite(led_pin, 0, RGB_BRIGHTNESS, 0);
  if ((queue_uart = xQueueCreate(8, sizeof(sensor_data_t))) == NULL) {
    handle_error();
    return;
  }
  if ((queue_dht11_timestamp = xQueueCreate(128, sizeof(uint64_t))) == NULL) {
    handle_error();
    return;
  }
  if ((event_group = xEventGroupCreate()) == NULL) {
    handle_error();
    return;
  }

  // Create tasks
  if (xTaskCreatePinnedToCore(task_heartbeat, "Task Heartbeat", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL, 1) != pdPASS) {
    handle_error();
    return;
  }
  if (xTaskCreatePinnedToCore(task_fastbeat, "Task Fastbeat", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, NULL, 1) != pdPASS) {
    handle_error();
    return;
  }
  if (xTaskCreatePinnedToCore(task_dht11, "Task DHT11", configMINIMAL_STACK_SIZE + 2048, NULL, tskIDLE_PRIORITY + 1, NULL, 1) != pdPASS) {
    handle_error();
    return;
  }
  if (xTaskCreatePinnedToCore(task_uart, "Task UART", configMINIMAL_STACK_SIZE + 2048, NULL, tskIDLE_PRIORITY + 1, NULL, 1) != pdPASS) {
    handle_error();
    return;
  }
  if (xTaskCreatePinnedToCore(task_id, "Task ID", configMINIMAL_STACK_SIZE + 512, NULL, tskIDLE_PRIORITY + 1, NULL, 1) != pdPASS) {
    handle_error();
    return;
  }
  
  setup_ble(); // Initialize BLE
}

void loop() {
  // 
}
