#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "main.h"

#include "defines.h"
#include "crc.h"

#define bytes_to_u16(MSB,LSB) (((unsigned int) ((unsigned char) MSB)) & 255)<<8 | (((unsigned char) LSB)&255) 

#define HANDHELD_UART  (UART_NUM_0)
#define HANDHELD_TXD  UART_PIN_NO_CHANGE//(GPIO_NUM_4)
#define HANDHELD_RXD  UART_PIN_NO_CHANGE//(GPIO_NUM_5)
#define HANDHELD_RTS  (UART_PIN_NO_CHANGE)
#define HANDHELD_CTS  (UART_PIN_NO_CHANGE)

#define BUF_SIZE (1024)

#define GPIO_INPUT_IO_0     GPIO_NUM_4
#define GPIO_INPUT_IO_1     GPIO_NUM_2
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1))
#define ESP_INTR_FLAG_DEFAULT 0

static QueueHandle_t gpio_evt_queue = NULL;

#define PATTERN_CHR_NUM 1

#define MAX_DATA_LEN 24

union{
    int hex;
    float f;
}DataConv;

union{
    uint8_t crc8[2];
    uint16_t crc16;
}uCRC;

float getRegVal(int hex){
    DataConv.hex = hex;
    return DataConv.f;
}

int getHexVal(float data){
    DataConv.f = data;
    return DataConv.hex;
}

typedef struct{
    uint8_t data[MAX_DATA_LEN];
    uint8_t id;
    uint8_t size;
}IMUData;

IMUData nullData = {.data="",.id=0};

typedef struct{
    int front, rear, size;
    uint8_t capacity;
    IMUData *data;
}IMUHistory;

IMUHistory *history;

static QueueHandle_t uart1_queue;

static const char *TAG_UART = "uart_events";

IMUHistory* createIMUHistoryQueue(uint8_t capacity){
    IMUHistory* history = (IMUHistory*)malloc(sizeof(IMUHistory));
    history->capacity = capacity;
    history->front = history->size = 0;

    history->rear = capacity - 1;
    history->data = (IMUData*)malloc(history->capacity * sizeof(IMUData));

    return history;
}

bool isFull(IMUHistory *history){
    return (history->size == history->capacity);
}

bool isEmpty(IMUHistory *history){
    return (history->size == 0);
}

IMUData removeHistory(IMUHistory *history){
    if(isEmpty(history))
        return nullData;
    IMUData data = history->data[history->front];
    history->front = (history->front + 1)  % history->capacity;
    history->size = history->size - 1;
    return data;
}

void addHistory(IMUHistory *history, IMUData data){
    if(isFull(history)){
        removeHistory(history);
    }
        
    history->rear = (history->rear + 1) % history->capacity;
    history->data[history->rear] = data;
    history->size = history->size + 1;
}

void enable_sleep(sleepReason s){

    if(s==charging){
        const int ext_wakeup_pin_0 = GPIO_INPUT_IO_0;

        printf("Enabling EXT0 wakeup on pin GPIO%d\n", ext_wakeup_pin_0);
        esp_sleep_enable_ext0_wakeup(ext_wakeup_pin_0, 0);

        rtc_gpio_pullup_dis(ext_wakeup_pin_0);
        rtc_gpio_pulldown_en(ext_wakeup_pin_0);
    }
    else if(s==other){

        const int ext_wakeup_pin_1 = GPIO_INPUT_IO_1;
        const uint64_t ext_wakeup_pin_1_mask = 1ULL << ext_wakeup_pin_1;

        printf("Enabling EXT1 wakeup on pins GPIO%d\n", ext_wakeup_pin_1);
        esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask , ESP_EXT1_WAKEUP_ANY_HIGH);

        rtc_gpio_pullup_dis(ext_wakeup_pin_1);
        rtc_gpio_pulldown_en(ext_wakeup_pin_1); 
    }

    esp_deep_sleep_start();
}

static void uart_task(void *arg)
{

    uart_event_t event;
    size_t buffered_size;
    uint16_t crc;
    int crcRecv;
    uint8_t crcHi, crcLo;
    uint8_t i,id;
    IMUData recvData;

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_driver_install(HANDHELD_UART, BUF_SIZE * 2, BUF_SIZE *2, 20, &uart1_queue, 0);
    uart_param_config(HANDHELD_UART, &uart_config);
    uart_set_pin(HANDHELD_UART, HANDHELD_TXD, HANDHELD_RXD, HANDHELD_RTS, HANDHELD_CTS);

    uart_enable_pattern_det_baud_intr(HANDHELD_UART, '#', PATTERN_CHR_NUM, 9, 0, 0);
    //Reset the pattern queue length to record at most 20 pattern positions.
    uart_pattern_queue_reset(HANDHELD_UART, 20);

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

    while (1) {
        if(xQueueReceive(uart1_queue, (void * )&event, (TickType_t)portMAX_DELAY)) {
            bzero(data, BUF_SIZE);
            ESP_LOGI(TAG_UART, "uart[%d] event:", HANDHELD_UART);
            switch(event.type) {
                
                case UART_DATA:
                    ESP_LOGI(TAG_UART, "[UART DATA]: %d", event.size);
                    uart_read_bytes(HANDHELD_UART, data, event.size, portMAX_DELAY);
                    ESP_LOGI(TAG_UART, "[DATA EVT]:");
                    if(strstr(data,"MARCO")){
                        id = data[6];
                        recvData.id = id;
                        recvData.size = event.size-2-6;
                        memcpy(recvData.data,data+6,recvData.size);
                        crc = usMBCRC16(data+5,event.size-5-2);
                        crcRecv = bytes_to_u16(data[event.size-1],data[event.size-2]);
                        bzero(data, BUF_SIZE);
                        sprintf(data,"#POLO");
                        data[5]=id;
                        if(crc == crcRecv){
                            printf("Packet OK\n");
                            strcat(data,"OK");
                            addHistory(history,recvData);
                        }
                        else{
                            printf("Packet FAIL\n");
                            strcat(data,"FAIL");
                        }
                        uart_write_bytes(HANDHELD_UART, (const char*) data, strlen(data));
                        printf("Data Sent: %s\n",data);
                    }
                    else if(strstr(data,"GETDATA")){
                        recvData = removeHistory(history);
                        if(recvData.size==0){
                            printf("No More Data\n");
                        }
                        else{
                            bzero(data, BUF_SIZE);
                            sprintf(data,"ID = %d Size=%d Data = ",recvData.id,recvData.size);
                            strncat(data,recvData.data,recvData.size);
                        }
                        printf("History Data = %s\n",data);
                    }
                    break;
                
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG_UART, "hw fifo overflow");
                    
                    uart_flush_input(HANDHELD_UART);
                    xQueueReset(uart1_queue);
                    break;
                
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG_UART, "ring buffer full");
                    
                    uart_flush_input(HANDHELD_UART);
                    xQueueReset(uart1_queue);
                    break;
                
                case UART_BREAK:
                    ESP_LOGI(TAG_UART, "uart rx break");
                    break;
                
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG_UART, "uart parity error");
                    break;
                
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG_UART, "uart frame error");
                    break;
                
                case UART_PATTERN_DET:
                    uart_get_buffered_data_len(HANDHELD_UART, &buffered_size);
                    int pos = uart_pattern_pop_pos(HANDHELD_UART);
                    ESP_LOGI(TAG_UART, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                    if (pos == -1) {
                        uart_flush_input(HANDHELD_UART);
                    } else {
                        uart_read_bytes(HANDHELD_UART, data, pos, 100 / portTICK_PERIOD_MS);
                        uint8_t pat[PATTERN_CHR_NUM + 1];
                        memset(pat, 0, sizeof(pat));
                        uart_read_bytes(HANDHELD_UART, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
                        ESP_LOGI(TAG_UART, "read data: %s", data);
                        ESP_LOGI(TAG_UART, "read pat : %s", pat);
                    }
                    break;
                //Others
                default:
                    ESP_LOGI(TAG_UART, "uart event type: %d", event.type);
                    break;
            }
        }
    }
}

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            printf("GPIO[%"PRIu32"] intr, val: %d\n", io_num, gpio_get_level(io_num));
            if(gpio_get_level(io_num)==1&&io_num==4){
                enable_sleep(charging);
            }
        }
    }
}

void gpio_init(){
    gpio_config_t io_conf = {};

    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    
    io_conf.pull_up_en = 0;
    io_conf.pull_down_en = 1;
    gpio_config(&io_conf);

    //change gpio interrupt type for one pin
    // gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_ANYEDGE);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
    
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);
}



void app_main()
{
    switch (esp_sleep_get_wakeup_cause()) {
        case ESP_SLEEP_WAKEUP_EXT0: {
            printf("Wake up from ext0\n");
            break;
        }
        case ESP_SLEEP_WAKEUP_EXT1: {
            printf("Wake up from ext1\n");
            break;
        }
    }

    history = createIMUHistoryQueue(32);


    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_task();

    gpio_init();

    xTaskCreate(uart_task, "uart_task", 2* 1024, NULL, 10, NULL);

    // xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);
}