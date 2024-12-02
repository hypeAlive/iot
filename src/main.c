#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"


void app_main() {
    while(1) {
        printf("Failed to read Lux value\n");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}