//-------------------------------------------------------------------------------------------
// 2019/08/08 rewrite for publish
//-------------------------------------------------------------------------------------------
#include <stdio.h>
#include <Preferences.h>
#include "driver/uart.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <queue>

const char APP_VERSION[] = "2019.08.29.01";

//-----------------------------------------------------------------------------------
#define DEVICE_NAME         "ROB-1"
#define SERVICE_UUID        "74829A60-9471-4804-AD29-9497AD731EC9"
#define CHARACTERISTIC_UUID "1C05C777-D455-4194-8196-F176E656A90F"
#define CONNECT_LED (2)

//-----------------------------------------------------------------------------------
#define EX_UART_NUM UART_NUM_0
#define BUF_SIZE (1024)

uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
};

void uart_setup()
{
  uart_param_config(EX_UART_NUM, &uart_config);
  uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0);
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
          ble_msg_queue.push(ble_msg_queue_sub.front());
          ble_msg_queue_sub.pop();
          xSemaphoreGive(ble_mux);
        }
      }
      xSemaphoreGive(ble_mux_sub);
    }
    delay(10);
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
    void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t* param) {
      esp_ble_conn_update_params_t conn_params = {0};
      memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
      conn_params.latency = 0;
      conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
      conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
      conn_params.timeout = 400;     // timeout = 400*10ms = 4000ms
      esp_ble_gap_update_conn_params(&conn_params);

      deviceConnected = true;
      digitalWrite(CONNECT_LED, HIGH);
      push_ble_msg("connected");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      digitalWrite(CONNECT_LED, LOW);
      push_ble_msg("disconnected");
    }
};

class MyBLECharacteristicCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    push_ble_msg(value);
  }
};

String device_name;
String service_uuid;
String charact_uuid;

void setup_ble()
{
  ble_mux = xSemaphoreCreateMutex();
  ble_mux_sub = xSemaphoreCreateMutex();
  BLEDevice::init(device_name.c_str());
  BLEDevice::setPower(ESP_PWR_LVL_P7);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyBLEServerCallbacks());
  BLEService *pService = pServer->createService(service_uuid.c_str());
  pCharacteristic = pService->createCharacteristic(
                      charact_uuid.c_str(),
                      //BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE_NR |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new MyBLECharacteristicCallbacks());
  pService->start();
  BLEAdvertising* advertising = pServer->getAdvertising();
  advertising->addServiceUUID(service_uuid.c_str());
  advertising->start();
  xTaskCreatePinnedToCore(ble_msg_task, "ble_msg_task", 4096*2, NULL, 1, NULL, xPortGetCoreID());
}

//-----------------------------------------------------------------------------------
Preferences preferences;

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
  device_name = preferences.getString("device_name", DEVICE_NAME);
  service_uuid = preferences.getString("service_uuid", SERVICE_UUID);
  charact_uuid = preferences.getString("charact_uuid", CHARACTERISTIC_UUID);  

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
          printf("\n");
          break;
        }
        if (c == '\b' || c == 0x7f) {
          if (index > 0) {
            printf("\b \b");
            fflush(stdout);
            index--;
          }
          continue;
        }
        printf("%c", c);
        fflush(stdout);
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

  if (!strcmp(cmd, "restart")) {
    delay(100);
    ESP.restart();
    while (true);
  }

  else if (!strcmp(cmd, "device_name")) {
    char s[256];
    int c = sscanf(command, "%s %s", cmd, s);
    if (c == 1) {
      printf("device_name : %s\n", device_name.c_str());
    }
    else if (strlen(s) > 5) {
      printf("error: device_name's max char length is 5.\n");
    }
    else {
      device_name = s;
      preferences.putString("device_name", device_name);
    }
  }
  else if (!strcmp(cmd, "service_uuid")) {
    char s[256];
    int c = sscanf(command, "%s %s", cmd, s);
    if (c == 1) {
      printf("service_uuid : %s\n", service_uuid.c_str());
    }
    else {
      service_uuid = s;
      preferences.putString("service_uuid", service_uuid);
    }
  }
  else if (!strcmp(cmd, "charact_uuid")) {
    char s[256];
    int c = sscanf(command, "%s %s", cmd, s);
    if (c == 1) {
      printf("charact_uuid : %s\n", charact_uuid.c_str());
    }
    else {
      charact_uuid = s;
      preferences.putString("charact_uuid", charact_uuid);
    }
  }

  else if (!strcmp(cmd, "show")) {
    printf("     version : %s\n", APP_VERSION);
    printf(" device_name : %s\n", device_name.c_str());
    printf("service_uuid : %s\n", service_uuid.c_str());
    printf("charact_uuid : %s\n", charact_uuid.c_str());
    printf("         ble : %s\n", deviceConnected ? "connected" : "disconnected");
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
    printf("    restart ... to reflect the params\n");
    printf("params:\n");
    printf("    show ... show current params\n");
    printf("    device_name [name] ... show/change device name\n");
    printf("    service_uuid [uuid] ... show/change service uuid\n");
    printf("    charact_uuid [uuid] ... show/change characteristic uuid\n");
    printf("ble:\n");
    printf("    status ... show ble status, connected/disconnected\n");
    printf("    ble ... pop message\n");
    printf("    ble strings ... push message\n");
  }
}
