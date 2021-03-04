/* Hello World Example

This example code is in the Public Domain (or CC0 licensed, at your option.)

Unless required by applicable law or agreed to in writing, this
software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_system.h"
#include <sys/time.h>

#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"

#include <string.h>

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"

#ifdef CONFIG_IDF_TARGET_ESP32
#define CHIP_NAME "ESP32"
#endif

#ifdef CONFIG_IDF_TARGET_ESP32S2BETA
#define CHIP_NAME "ESP32-S2 Beta"
#endif

static const char* TAG = "MyModule";
static const char* STR_TEST = "HELLO WORLD!";

#define ESP_INTR_FLAG_DEFAULT 0
SemaphoreHandle_t xSemaphore = NULL;

spi_transaction_t trans;
esp_err_t ret;
spi_device_handle_t spi;

int interrupt_count = 0;
void IRAM_ATTR vblank_handler(void *args) {
  xSemaphoreGiveFromISR(xSemaphore, NULL);
}
int sendQSPI = 0;

void qspi_task(void* arg) {
  while(1) {
    if (xSemaphoreTake(xSemaphore,portMAX_DELAY) == pdTRUE && sendQSPI) {
      interrupt_count++;
      spi_device_transmit(spi, &trans);
      sendQSPI = 0;
    }
  }
}

char time_string[] = "hello"; //[255];
int tick_count;
void app_main(void)
{
  printf("Launching...\n");
  //settimeofday()
  time_t rawtime;
  struct tm * timeinfo;

  printf ( "The current date/time is: %s", time_string );

  // printf("Hello world!\n");

  /* Print chip information */
  // esp_chip_info_t chip_info;
  // esp_chip_info(&chip_info);
  // printf("This is %s chip with %d CPU cores, WiFi%s%s, ",
  //         CHIP_NAME,
  //         chip_info.cores,
  //         (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
  //         (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");
  //
  // printf("silicon revision %d, ", chip_info.revision);
  //
  // printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
  //         (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
  //
  // for (int i = 10; i >= 0; i--) {
  //     printf("Restarting in %d seconds...\n", i);
  //     vTaskDelay(1000 / portTICK_PERIOD_MS);
  // }
  // printf("Restarting now.\n");
  // fflush(stdout);
  spi_bus_config_t buscfg={
    .miso_io_num = GPIO_NUM_19,
    .mosi_io_num = GPIO_NUM_23,
    .sclk_io_num = GPIO_NUM_18,
    .quadwp_io_num = GPIO_NUM_22,
    .quadhd_io_num = GPIO_NUM_21,
    .max_transfer_sz= 16*320*2+8,
    .flags = 0 //SPICOMMON_BUSFLAG_IOMUX_PINS
  };
  spi_device_interface_config_t devcfg={
    .clock_speed_hz = 10*1000*1000, //80
    .mode = 0,
    .spics_io_num = 5, // previous example had -1 and did cs manually
    .queue_size = 7, //We want to be able to queue 7 transactions at a time
    //.address_bits = 16,
    .flags=SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_NO_DUMMY //SPI_DEVICE_NODUMMY
  };
  //Initialize the SPI bus
  ret = spi_bus_initialize(VSPI_HOST, &buscfg, 1);
  if(ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to add QSPI device");
    return;
  }
  ret=spi_bus_add_device(VSPI_HOST, &devcfg, &spi);
  if(ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to add QSPI device");
    return;
  }
  // gpio_config_t io_conf = {
  //   .intr_type = GPIO_PIN_INTR_DISABLE,
  //   .mode = GPIO_MODE_OUTPUT,
  //   .pin_bit_mask = 1LL << GPIO_NUM_5,
  // };
  //
  // ret = gpio_config(&io_conf);
  // if(ret != ESP_OK)
  // {
  //   ESP_LOGE(TAG, "Failed to add CS pin");
  //   return;
  // }
  // gpio_set_level(GPIO_NUM_5, 1);

  /* Transaction to read a register */
  uint16_t addr = 0x21C0;
  uint8_t data[255] = {addr >> 8, addr & 0xFF};
  data[2] = 0x10;
  data[3] = 0x10;
  data[4] = 0x10;
  data[5] = 0x04;
  data[6] = 0x10;
  data[7] = 0x0A;
  // for (int i = 0; i < 12; i++)
    // data[i+2] = STR_TEST[i];
  // while(1) {
    // gpio_set_level(GPIO_NUM_5, 0); //active CS
    uint32_t reg_data = 0xFF00FF00;

    //Send reg addr and data
    memset(&trans, 0, sizeof(trans)); // Zero out the transaction
    trans.tx_buffer = &data; //point to user buffer for Tx data
    trans.rxlength = 0;
    trans.length = 2*8;
    trans.flags = 0;
    trans.rx_buffer = NULL;
    trans.flags |= SPI_TRANS_MODE_QIO;

    // create the binary semaphore
  	xSemaphore = xSemaphoreCreateBinary();

    gpio_set_direction(GPIO_NUM_27, GPIO_MODE_INPUT);
    gpio_set_intr_type(GPIO_NUM_27, GPIO_INTR_POSEDGE);
    xTaskCreate(qspi_task, "qspi_task", 2048, NULL, 10, NULL);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_NUM_27, vblank_handler, NULL);

    //Read back reg data
    // spi_transaction_t rx_trans;
    // memset(&rx_trans, 0, sizeof(rx_trans)); // Zero out the transaction
    // rx_trans.length = 0; // transaction length is in bits.
    // rx_trans.tx_buffer = NULL; // TX Data
    // rx_trans.rx_buffer = &reg_data; // Rx buffer
    // rx_trans.rxlength = 4 * 8; //4bytes of reg data
    // rx_trans.flags |= SPI_TRANS_MODE_QIO;
    // spi_device_transmit(spi, &rx_trans);
    //close the transaction
    // gpio_set_level(GPIO_NUM_5, 1); //inactive CS
    time_t nowtime = {1608495248, 0};
    settimeofday(&nowtime, 0);
while(1) {
    //for (int i = 1; i >= 0; i--) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    //}
    time (&rawtime);
    timeinfo = localtime(&rawtime);
    //int str_length = strftime(time_string, sizeof(data)-2, "%x %X %a", &timeinfo);
    char* asc_time = asctime(timeinfo);
    int str_length = 24;
    printf("%s\n", asc_time);
    for (int i = 0; i < str_length; i++) {
      char translated;
      if (asc_time[i] >= 'a' && asc_time[i] <= 'z') {
        data[2+i*2] = 0x10;
        data[2+i*2+1] = asc_time[i] - 97;
      }
      else if (asc_time[i] >= 'A' && asc_time[i] <= 'Z') {
        data[2+i*2] = 0x10;
        data[2+i*2+1] = asc_time[i] - 65;
      }
      else if (asc_time[i] >= '0' && asc_time[i] <= '9') {
    	data[2+i*2] = 0x10;
	    data[2+i*2+1] = asc_time[i] +26 - 48;
      }
      else if (asc_time[i] == ' ') {
      	data[2+i*2] = 0x10;
  	    data[2+i*2+1] = 0x78;
      }
      else if (asc_time[i] == ':') {
		data[2+i*2] = 0x10;
		data[2+i*2+1] = 0x27;
      }
    }
    trans.length = (2+str_length*2)*8;
    sendQSPI = 1;
    //tick_count = (tick_count+1)%10;
    //data[3] = 26+tick_count;
    interrupt_count = 0;
  }


//  esp_restart();
}
