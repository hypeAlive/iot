#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#define GPIO_LED 26

#define I2C_MASTER_PORT 0 // port number
#define I2C_MASTER_SDA_IO 21 // GPIO data
#define I2C_MASTER_SCL_IO 22 // GPIO clock
#define I2C_MASTER_FREQ_HZ 400000 // I2C frequency
#define TSL2561_ADDR 0x29 // I2C address of the TSL2561 sensor
#define TSL2561_COMMAND_BIT 0x80
#define TSL2561_REG_ID (TSL2561_COMMAND_BIT | 0x0A)
#define TSL2561_REG_CONTROL (TSL2561_COMMAND_BIT | 0x00)
#define TSL2561_REG_DATA0LOW  (TSL2561_COMMAND_BIT | 0x0C) // DATA0 Low Byte
#define TSL2561_REG_DATA0HIGH (TSL2561_COMMAND_BIT | 0x0D) // DATA0 High Byte
#define TSL2561_REG_DATA1LOW  (TSL2561_COMMAND_BIT | 0x0E) // DATA1 Low Byte
#define TSL2561_REG_DATA1HIGH (TSL2561_COMMAND_BIT | 0x0F) // DATA1 High Byte

float lux_calculate(uint16_t ch0, uint16_t ch1) {
    if (ch0 == 0) {
        return 0;
    }

    float ratio = (float)ch1 / (float)ch0;

    // Lux-Berechnung gemäß Ratio
    if (ratio > 1.30) {
        return 0;
    } else if (ratio > 0.80) {
        return 0.00146 * ch0 - 0.00112 * ch1;
    } else if (ratio > 0.61) {
        return 0.0128 * ch0 - 0.0153 * ch1;
    } else if (ratio > 0.50) {
        return 0.0224 * ch0 - 0.031 * ch1;
    } else {
        return 0.0304 * ch0 - 0.062 * ch0 * pow(ratio, 1.4);
    }
}

void i2c_master_init() {
    i2c_config_t conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = I2C_MASTER_SDA_IO,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_io_num = I2C_MASTER_SCL_IO,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master .clk_speed = I2C_MASTER_FREQ_HZ
    };
    i2c_param_config(I2C_MASTER_PORT, &conf);
    i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0);
}

uint8_t i2c_register_read(i2c_port_t port_num, uint8_t dev_addr, uint8_t reg_addr) {
    uint8_t data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // Start I2C communication
    i2c_master_start(cmd);

    // Send slave address with write bit
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);

    // Send register address
    i2c_master_write_byte(cmd, reg_addr, true);

    // Restart I2C communication
    i2c_master_start(cmd);

    // Send slave address with read bit
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);

    // Read data from register
    i2c_master_read_byte(cmd, &data, I2C_MASTER_LAST_NACK);

    // Stop I2C communication
    i2c_master_stop(cmd);

    // Execute the I2C command
    i2c_master_cmd_begin(port_num, cmd, pdMS_TO_TICKS(1000));

    // Delete the I2C command link
    i2c_cmd_link_delete(cmd);

    return data;
}

esp_err_t i2c_register_write(i2c_port_t port_num, uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t err;

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(port_num, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    return err;
}

esp_err_t i2c_multi_register_read(
        i2c_port_t port_num,
        uint8_t dev_addr,
        uint8_t reg_addr,
        uint8_t *data,
        size_t size
) {
    if (data == NULL || size == 0) {
        return ESP_ERR_INVALID_ARG; // Ungültige Argumente prüfen
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t err;

    // I2C-Start und Schreibphase (Registeradresse senden)
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);

    // Lesephase starten
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);

    // Daten lesen
    if (size > 1) {
        i2c_master_read(cmd, data, size - 1, I2C_MASTER_ACK); // Mehrere Bytes
    }
    i2c_master_read_byte(cmd, &data[size - 1], I2C_MASTER_LAST_NACK); // Letztes Byte

    // Stop-Bedingung senden
    i2c_master_stop(cmd);

    // Übertragung starten und Ergebnis prüfen
    err = i2c_master_cmd_begin(port_num, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    return err;
}

uint8_t lux_read_sensor_id() {
    return i2c_register_read(I2C_MASTER_PORT, TSL2561_ADDR, TSL2561_REG_ID);
}

esp_err_t sensor_power_on() {
    uint8_t power_on_command = 0x03;
    return i2c_register_write(I2C_MASTER_PORT, TSL2561_ADDR, TSL2561_REG_CONTROL, power_on_command);
}

void enable_sensor() {
    // Power On the sensor (wake up from sleep)
    if (sensor_power_on() != ESP_OK) {
        printf("Failed to power on the sensor\n");
    } else {
        printf("Sensor powered on successfully\n");
    }
    vTaskDelay(pdMS_TO_TICKS(100));  // Wait for the sensor to stabilize and take a measurement
}

float lux_read_sensor_value() {
    uint8_t data0_low = 0, data0_high = 0;
    uint8_t data1_low = 0, data1_high = 0;

    // Lese DATA0 Low/High Byte
    if (i2c_multi_register_read(I2C_MASTER_PORT, TSL2561_ADDR, TSL2561_REG_DATA0LOW, &data0_low, 1) != ESP_OK ||
        i2c_multi_register_read(I2C_MASTER_PORT, TSL2561_ADDR, TSL2561_REG_DATA0HIGH, &data0_high, 1) != ESP_OK) {
        printf("Failed to read DATA0\n");
        return -1;
    }

    // Lese DATA1 Low/High Byte
    if (i2c_multi_register_read(I2C_MASTER_PORT, TSL2561_ADDR, TSL2561_REG_DATA1LOW, &data1_low, 1) != ESP_OK ||
        i2c_multi_register_read(I2C_MASTER_PORT, TSL2561_ADDR, TSL2561_REG_DATA1HIGH, &data1_high, 1) != ESP_OK) {
        printf("Failed to read DATA1\n");
        return -1;
    }

    // Kombiniere High- und Low-Bytes zu 16-Bit-Werten
    uint16_t ch0 = (data0_high << 8) | data0_low;
    uint16_t ch1 = (data1_high << 8) | data1_low;

    // Berechne Lux-Wert
    float lux = lux_calculate(ch0, ch1);

    return lux;
}

void test_i2c_functions() {
    printf("Testing I2C functions...\n");

    // Test: Power On the sensor
    if (i2c_register_write(I2C_MASTER_PORT, TSL2561_ADDR, TSL2561_REG_CONTROL, 0x03) == ESP_OK) {
        printf("Sensor powered on successfully\n");
        printf("wrote 0x03");
    } else {
        printf("Failed to power on sensor\n");
        return;
    }

    // Test: Read the ID register
    uint8_t id = 0;
    if (i2c_multi_register_read(I2C_MASTER_PORT, TSL2561_ADDR, TSL2561_REG_CONTROL, &id, 1) == ESP_OK) {
        printf("Read ID register successfully");
        printf("read 0x%02X\n", id);
    } else {
        printf("Failed to read ID register\n");
    }

    // Test: Write and Multi-Read Back
    uint8_t write_value = 0x02, read_buffer[2] = {0};
    if (i2c_register_write(I2C_MASTER_PORT, TSL2561_ADDR, TSL2561_REG_CONTROL, write_value) == ESP_OK &&
        i2c_multi_register_read(I2C_MASTER_PORT, TSL2561_ADDR, TSL2561_REG_CONTROL, read_buffer, 1) == ESP_OK &&
        write_value == read_buffer[0]) {
        printf("Write and Multi-Read Back test passed\n");
    } else {
        printf("Write and Multi-Read Back test failed\n");
    }

    // Test: Read Lux Value
    float lux = lux_read_sensor_value();
    if (lux >= 0) {
        printf("Lux value: %.2f\n", lux);
    } else {
        printf("Failed to read Lux value\n");
    }
}

void app_main() {
    printf("Hello, world!\n");
    i2c_master_init();
    enable_sensor();
    while(1) {
        float lux = lux_read_sensor_value();
        if (lux >= 0) {
            printf("Lux value: %.2f\n", lux);
        } else {
            printf("Failed to read Lux value\n");
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}