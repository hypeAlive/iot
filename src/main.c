#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include <stdio.h>
#include <string.h>

#define GPIO_LED 5
#define GPIO_LED1 26
#define GPIO_LED2 4

typedef struct {
    int led;
    int frequency;
} led_command_t;

QueueHandle_t led_queue;

// Funktion zur Zuordnung von LED-ID zu GPIO-Pin
int led_id_to_gpio(int led_id) {
    switch (led_id) {
        case 0: return GPIO_LED;
        case 1: return GPIO_LED1;
        case 2: return GPIO_LED2;
        default: return -1; // Ungültige LED-ID
    }
}

void blink_task(void *pvParameter) {
    int gpio_num = *(int *)pvParameter; // GPIO-Nummer
    int delay = 1000; // Standardwert
    led_command_t cmd;

    gpio_set_direction(gpio_num, GPIO_MODE_OUTPUT);

    while (1) {
        if (xQueuePeek(led_queue, &cmd, pdMS_TO_TICKS(100)) == pdPASS) {
            int target_gpio = led_id_to_gpio(cmd.led);
            printf("Task for GPIO %d received command for LED %d\n", gpio_num, cmd.led);
            printf("Comparing %d with %d\n", target_gpio, gpio_num);
            if (target_gpio == gpio_num) {
                delay =(int) cmd.frequency;
                printf("Task for GPIO %d received new frequency: %d Hz, updated delay: %d ms\n", gpio_num, cmd.frequency, delay);
                xQueueReceive(led_queue, &cmd, 0);
            } else {
                printf("Task for GPIO %d ignored command for LED %d\n", gpio_num, cmd.led);
            }
        }
        gpio_set_level(gpio_num, 0);
        vTaskDelay(pdMS_TO_TICKS(delay));
        gpio_set_level(gpio_num, 1);
        vTaskDelay(pdMS_TO_TICKS(delay));
    }
}

void input_task(void *pvParameters) {
    char command[100];
    led_command_t cmd;
    while (1) {
        if (fgets(command, sizeof(command), stdin) != NULL) {
            command[strcspn(command, "\n")] = 0;
            if (strlen(command) > 0) {
                printf("Received input: %s\n", command);
                if (sscanf(command, "%d:%d", &cmd.led, &cmd.frequency) == 2) {
                    if (xQueueSend(led_queue, &cmd, 10) == pdPASS) {
                        printf("Sent command to queue: LED %d, Frequency %d Hz\n", cmd.led, cmd.frequency);
                    } else {
                        printf("Failed to send command to queue\n");
                    }
                } else {
                    printf("Invalid command format. Use <LED>:<Frequency>, e.g., 1:200\n");
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main() {
    int gpio_leds[3] = {GPIO_LED, GPIO_LED1, GPIO_LED2};

    led_queue = xQueueCreate(10, sizeof(led_command_t));

    if (led_queue == NULL) {
        printf("Failed to create queue\n");
        return;
    }

    // Increase the stack size for blink_task
    xTaskCreate(blink_task, "blink_task_1", 2048, &gpio_leds[0], 5, NULL);
    xTaskCreate(blink_task, "blink_task_2", 2048, &gpio_leds[1], 5, NULL);
    xTaskCreate(blink_task, "blink_task_3", 2048, &gpio_leds[2], 5, NULL);
    xTaskCreate(input_task, "input_task", 4096, NULL, 5, NULL);
}