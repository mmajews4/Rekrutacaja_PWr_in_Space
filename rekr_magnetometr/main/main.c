#include "driver/i2c.h"
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_SCL_IO           22          // gpio number for I2C master clock
#define I2C_MASTER_SDA_IO           21          // gpio number for I2C master data
#define I2C_SLAVE_ADDR              0x1A        // I2C slave address for slave dev
#define I2C_MASTER_NUM              I2C_NUM_0   // I2C port number for master dev
#define I2C_MASTER_TX_BUF_DISABLE   0           // I2C master do not need buffer
#define I2C_MASTER_RX_BUF_DISABLE   0           // I2C master do not need buffer
#define I2C_MASTER_FREQ_HZ          100000      // I2C master clock frequency
#define WRITE_BIT                   I2C_MASTER_WRITE // I2C master write
#define READ_BIT                    I2C_MASTER_READ  // I2C master read
#define ACK_CHECK_EN                0x1     // I2C master will check ack from slave
#define ACK_CHECK_DIS               0x0     // I2C master will not check ack from slave
#define ACK_VAL                     0x0     // I2C ack value
#define NACK_VAL                    0x1     // I2C nack value
#define MMC5983MA_ADDRES            0x30    // I2C sensor addres


esp_err_t i2c_init() {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(I2C_MASTER_NUM, &conf);
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

// setup to read data from register
esp_err_t i2c_read_register(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data_rd, size_t size) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, dev_addr << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, dev_addr << 1 | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}


void readData(uint32_t *sensor_data) {
    uint8_t rawData[7];  // x/y/z mag register data stored here
    esp_err_t ret = i2c_read_register(MMC5983MA_ADDRES, 0x00, rawData, 7);  // Read the 7 raw data registers into data array

    if (ret == ESP_OK) {

        sensor_data[0] = (uint32_t)(rawData[0] << 10 | rawData[1] << 2 | (rawData[6] & 0xC0) >> 6); // Turn the 18 bits into a unsigned 32-bit value
        sensor_data[1] = (uint32_t)(rawData[2] << 10 | rawData[3] << 2 | (rawData[6] & 0x30) >> 4); // Turn the 18 bits into a unsigned 32-bit value
        sensor_data[2] = (uint32_t)(rawData[4] << 10 | rawData[5] << 2 | (rawData[6] & 0x0C) >> 2); // Turn the 18 bits into a unsigned 32-bit value
    }
}


esp_err_t i2c_master_write_slave(uint8_t *data, size_t size) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, I2C_SLAVE_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

void app_main() {
    i2c_init();
    uint32_t sensor_data[3]; // Array to store sensor data

    while(1){
        readData(sensor_data);   // Reading sensor data

        printf("Sensor data[x,y,z]: ");
        for (int i = 0; i < 3; i++) {
            printf("%ld, ", sensor_data[i]);
        }
        printf("\n");

        // Send sensor data to slave ESP32
        esp_err_t ret = i2c_master_write_slave((uint8_t *)sensor_data, sizeof(sensor_data));
        if (ret == ESP_OK) {
            printf("Udało się wydłać do slavea.\n");
        } else {
            printf("Nie udało się wysłać, error number: %d\n", ret);
        }
        
        vTaskDelay(pdMS_TO_TICKS(5)); // So that we don't spam SD card
    }
}
