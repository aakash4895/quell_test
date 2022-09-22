#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_sleep.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_event.h"

#include "driver/rtc_io.h"
#include "soc/rtc.h"
#include "soc/soc_caps.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

typedef enum{
    charging,
    other
}sleepReason;

void wifi_task();
void udp_client_task(void *pvParameters);
void enable_sleep(sleepReason s);