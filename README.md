# Quell Coding Exercise

I'll be using only 1 ESP32 to do the exercise.

# Q1 Firmware

For Question I have implemented a modbus like packet structure.
UART0 has been used for exchanging packet. This UART is configurable and can be changed by simply changing few macros in main.c

```
#define HANDHELD_UART  (UART_NUM_0)
#define HANDHELD_TXD  UART_PIN_NO_CHANGE//(GPIO_NUM_4)
#define HANDHELD_RXD  UART_PIN_NO_CHANGE//(GPIO_NUM_5)
#define HANDHELD_RTS  (UART_PIN_NO_CHANGE)
#define HANDHELD_CTS  (UART_PIN_NO_CHANGE)
```

Incoming packets that should be sent to the esp32 is 
>#MARCO[ID][DATA in HEX][CRC]

The CRC in the packet insures that we are receiveing valid incoming packet.

Outgoing packet for valid reception is
>#POLO[ID]OK

Outgoing packet for invalid reception is
>#POLO[ID]FAIL

When a valid packet is received the program store the relevant data, ID and size of data into a structure is than appended in the history queue which has been intialised at 32 size

To read the last history stored on esp32 simply send the following command on serial.
>#GETDATA

This will return the ID, Size and the Data on the serial terminal.

# Q2 Design

## Sleep Conditions

### a When not in use for an extended period of time

For this condition I have implemented a timer. This timer generates an alarm after certain time which can be set in a macro in main.c
```
#define INACTIVITY_TIMER_COUNT      3*10*1000000 // period = 30s
```

If the count completes the ESP32 will go to sleep. To prevent this from happeing either a packet should be received on the UART port or the user button should be pressed within the set amount of time.

Watchdog timer can be also used to prevent the device from getting stuck, restarting the device automatically. But not applicable in this condition.

### b When disconnected from the network

Whenever esp32 disconnects from a network, it will retry to connection 10 times after that it goes into sleep
This can be seen in the event handler function for wifi module in wifi.c at line 41. To change the wifi usename and password and the number of retries just simply edit the macros in the wifi.c file

```
#define ESP_WIFI_SSID      ""
#define ESP_WIFI_PASS      ""
#define ESP_MAXIMUM_RETRY  10
```

### c When the connection is unstable.

To test this, few macros need to be assigned in file udp.c

```
#define HOST_IP_ADDR        ""
#define PORT                8080
#define NUM_RETRY           10
```

To check whether the UDP connection is stable the program is sending a periodic packet to the server, which it expects a reply in exchange. After certian number of consecutive fails to receive the reply, the ESP32 goes to sleep.

> Outgoing Packet Data >> PING
> Incoming Packet Data >> OK

### d Immediately when placed on the charging dock

For this sleep event I have selected GPIO Pin 4. Whenever this pin is set to high, ESP32 will go into deep sleep and can only be woken up when it is removed from the dock, which in this case is just disconnecting the pin from HIGH signal.

## Wake Up condition

### a When removed from the charging dock

GPIO Pin 4 is used a the charging dock pin. This pin is pulled down internally so as to detect whenever the pin is set high when placed on the charging dock. Since the pin is internally pulled down, whenever the device is removed from the dock, the pin reverts low state and the ESP32 wakes up

### b After a short press of one of the buttons

GPIO 2 is used for waking up from deep sleep whenever ESP32 goes to sleep for reasons other than charging. This pin is also internally pulled down, in sleep condition it is set as RTC EXTI 1 pin to wake up ESP 32 from sleep by detecting HIGH signal on the Pin.


