#include"freertos/FreeRTOS.h"
#include"freertos/task.h"
#include"driver/gpio.h"
#include"sdkconfig.h"

#define GPIO_LED 5

void app_main() {
    gpio_set_direction(GPIO_LED, GPIO_MODE_OUTPUT);
    while (1) {
        gpio_set_level(GPIO_LED, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(GPIO_LED, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}