#include <stdio.h>
#include <Preferences.h>
#include "driver/uart.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
//#include "EasyBLE2.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <queue>

const char APP_VERSION[] = "2019.07.03.01";

//-----------------------------------------------------------------------------------
#define DEVICE_NAME         "HR-1"
#define SERVICE_UUID        "082E82AF-450D-4EA4-95FA-F0383A86377F"
#define CHARACTERISTIC_UUID "DB5A95D9-D347-4A9F-8348-89CF8E396C13"
#define CONNECT_LED (2)

//-----------------------------------------------------------------------------------
#define EX_UART_NUM UART_NUM_0
//#define BUF_SIZE (256)
#define BUF_SIZE (512)
//static QueueHandle_t uart0_queue;
//uint8_t* uart_data = (uint8_t*)malloc(BUF_SIZE);
uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
};

/*
static void uart_event_task(void *pvParameters)
{
  uart_event_t event;
  //uint8_t* dtmp = (uint8_t*)malloc(BUF_SIZE);
 
  while (1) {
    if (xQueueReceive(uart0_queue, (void*)&event, (portTickType)portMAX_DELAY)) {
      switch (event.type) {
      case UART_DATA:
        break;
      case UART_FIFO_OVF:
        uart_flush(EX_UART_NUM);
        break;
      case UART_BUFFER_FULL:
        uart_flush(EX_UART_NUM);
        break;
      case UART_BREAK:
        break;
      case UART_PARITY_ERR:
        break;
      case UART_FRAME_ERR:
        break;
      case UART_PATTERN_DET:
        break;
      default:
        break;
      }
    }
  }
  //free(dtmp);
  //dtmp = NULL;
  vTaskDelete(NULL);
}
*/

void uart_setup()
{
  uart_param_config(EX_UART_NUM, &uart_config);
  uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  //uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 10, &uart0_queue, 0);
  uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0);
  //uart_enable_pattern_det_intr(EX_UART_NUM, '+', 3, 10000, 10, 10);
  //xTaskCreatePinnedToCore(uart_event_task, "uart_event_task", 10000, NULL, 1, NULL, 0);
}

int uart_reads(void* buf, int max_size = BUF_SIZE)
{
  size_t length;
  uart_get_buffered_data_len(EX_UART_NUM, &length);
  if (length > 0) {
    if (max_size > length) max_size = length;
    length = uart_read_bytes(EX_UART_NUM, (uint8_t*)buf, max_size, 1);
    return length;
  }
  return 0;
}

void uart_writes(const void* data, int size)
{
  uart_write_bytes(EX_UART_NUM, (const char *)data, size);
}


//-----------------------------------------------------------------------------------
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
xSemaphoreHandle ble_mux, ble_mux_sub;
std::queue<std::string> ble_msg_queue, ble_msg_queue_sub;

static void ble_msg_task(void *pvParameters)
{
  // this function will be called in CALLBACK function. so tricky !!!
  while (true) {
    if (xSemaphoreTake(ble_mux_sub, (portTickType)10) == pdTRUE) {
      if (ble_msg_queue_sub.size() > 0) {
        if (xSemaphoreTake(ble_mux, (portTickType)10) == pdTRUE) {
          while (ble_msg_queue_sub.size() > 0) {
            ble_msg_queue.push(ble_msg_queue_sub.front());
            ble_msg_queue_sub.pop();
            delay(1);
          }
          xSemaphoreGive(ble_mux);
        }
      }
      xSemaphoreGive(ble_mux_sub);
    }
    delay(1);
  }
}

void push_ble_msg(std::string str)
{
  xSemaphoreTake(ble_mux_sub, portMAX_DELAY);
  ble_msg_queue_sub.push(str);
  xSemaphoreGive(ble_mux_sub);
}

std::string pop_ble_msg()
{
  std::string str = "";
  xSemaphoreTake(ble_mux, portMAX_DELAY);
  if (ble_msg_queue.size() > 0) {
    str = ble_msg_queue.front();
    ble_msg_queue.pop();
  }
  xSemaphoreGive(ble_mux);
  return str;
}

void clr_ble_msg()
{
  xSemaphoreTake(ble_mux, portMAX_DELAY);
  while (ble_msg_queue.size() > 8) {
    ble_msg_queue.pop();
  }
  xSemaphoreGive(ble_mux);
}

class MyBLEServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      digitalWrite(CONNECT_LED, HIGH);
      // printf("ble connected\n");
      push_ble_msg("connected");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      digitalWrite(CONNECT_LED, LOW);
      //printf("ble disconnected\n");
      push_ble_msg("disconnected");
    }
};

class MyBLECharacteristicCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    //printf("ble %s\n", value.c_str());
    push_ble_msg(value);
  }
};

void setup_ble()
{
  ble_mux = xSemaphoreCreateMutex();
  ble_mux_sub = xSemaphoreCreateMutex();
  BLEDevice::init(DEVICE_NAME);
  BLEDevice::setPower(ESP_PWR_LVL_P7);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyBLEServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      //BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE_NR |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new MyBLECharacteristicCallbacks());
  //pCharacteristic->setValue("Hello World");
  pService->start();
  BLEAdvertising* advertising = pServer->getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->start();
  xTaskCreatePinnedToCore(ble_msg_task, "ble_msg_task", 10000, NULL, 2, NULL, 0);
}

//-----------------------------------------------------------------------------------
Preferences preferences;
bool echo_on = false;

void setup()
{
  // LED
  pinMode(CONNECT_LED, OUTPUT);
  digitalWrite(CONNECT_LED, LOW);
  
  // Serial speed
  //Serial.begin(115200);
  uart_setup();
  while (true) {
    char c;
    int n = uart_reads(&c, 1);
    if (n == 0) break;
  }
  printf("ble controller\n");
  printf("ver. %s\n", APP_VERSION);
  printf("running core %d\n", xPortGetCoreID()); 

  // Preferences maximum key length is 15
  preferences.begin("esp32-ble-cont");
  echo_on = preferences.getBool("echo_on", true);

  // ble
  setup_ble();
}

void loop()
{
  char command[256];
  int index = 0;
  uint8_t data[20];
  while (true) {
    char c;
    int n = uart_reads(&c, 1);
    if (n > 0) {
        if (c == '\r' || c == '\n' || c == 0) {
          if (echo_on) printf("\n");
          break;
        }
        if (c == '\b' || c == 0x7f) {
          if (index > 0) {
            if (echo_on) {
              printf("\b \b");
              fflush(stdout);
            }
            index--;
          }
          continue;
        }
        if (echo_on) {
          printf("%c", c);
          fflush(stdout);
        }
        command[index] = c;
        if (++index == 255) break;
    }
    else {
      delay(1);
    }
  }
  command[index] = 0;

  char cmd[256] = {0};
  int32_t val = 0;
  int c = sscanf(command, "%s %d", cmd, &val);
  if (!strcmp(cmd, "echo")) {
    char s[256];
    int c = sscanf(command, "%s %s", cmd, s);
    if (c == 1) {
      printf("echo : %s\n", echo_on ? "on" : "off");
    }
    else if (!strcmp(s, "on")) {
      echo_on = true;
      preferences.putBool("echo_on", echo_on);
    }
    else if (!strcmp(s, "off")) {
      echo_on = false;
      preferences.putBool("echo_on", echo_on);
    }
    else {
      printf("invalid param: %s\n", s);
    }
  }

  else if (!strcmp(cmd, "show")) {
    printf("version : %s\n", APP_VERSION);
    printf("ble : %s\n", deviceConnected ? "connected" : "disconnected");
    delay(1);
  }
  else if (!strcmp(cmd, "ble")) {
    char s[256];
    int c = sscanf(command, "%s %s", cmd, s);
    if (c == 1) {
      //printf("%s\n", deviceConnected ? "connected" : "disconnected");
      std::string str = pop_ble_msg();
      printf("%s\n", str.c_str());
    }
    else {
      if (deviceConnected) {
        pCharacteristic->setValue((uint8_t*)&command[4], strlen(&command[4]) + 1);
        pCharacteristic->notify();
      }
    }
  }
  else if (!strcmp(cmd, "status")) {
    printf("ble : %s\n", deviceConnected ? "connected" : "disconnected");
  }
  else {
    printf("commands:\n");
    printf("setting:\n");
    printf("    echo [on|off]\n");
    printf("params:\n");
    printf("    show\n");
    printf("ble:\n");
    printf("    status ... connected/disconnected\n");
    printf("    ble ... pop message\n");
    printf("    ble strings ... push message\n");
  }
}
