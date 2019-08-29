# esp32_ble_cont
ble controller  

![ESP32-DevKitC-32D and ESP-32S](https://github.com/koki-ogura/esp32_ble_cont/blob/master/esp32.jpeg)  
left: ESP32-DevKitC-32D, right: ESP-32S  
When using DevKit, connect the GPIO2 pin in the order of 1K resistor, LED, and GND. This is the BLE connection indicator. When using ESP-32S, the LED is on the board from the beginning.  
As for the radio sensitivity, DevKit is better than ESP-32S.

## log
2019/08/29 change ble_msg_task's priority and execute core.  
2019/08/08 rewrite for publish.  
2019/07/08 resolve timeout problem.  
2019/07/04 first commit.  

## initial settings
```
#define DEVICE_NAME         "ROB-1"
#define SERVICE_UUID        "74829A60-9471-4804-AD29-9497AD731EC9"
#define CHARACTERISTIC_UUID "1C05C777-D455-4194-8196-F176E656A90F"
#define CONNECT_LED (2)
```

## used methods
```
iOS -> ESP32 : Write Without Response
ESP32 -> iOS : Notify
```

## commands
```
restart ... to reflect the params
show ... show current params
device_name [name] ... show/change device name
service_uuid [uuid] ... show/change service uuid
charact_uuid [uuid] ... show/change characteristic uuid
status ... show ble status, connected/disconnected
ble ... pop message
ble strings ... push message
```
