#include <stdio.h>
#include <stdlib.h>
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"

#define I2C_SLAVE_SCL_IO           22          /*!< GPIO number for I2C slave clock */
#define I2C_SLAVE_SDA_IO           21          /*!< GPIO number for I2C slave data  */
#define I2C_SLAVE_ADDR             0x1A        /*!< I2C slave address for slave device */
#define I2C_SLAVE_NUM              I2C_NUM_0   /*!< I2C port number for slave device */

#define SD_CARD_PIN_NUM_CS         5
#define SD_CARD_PIN_NUM_MOSI       23
#define SD_CARD_PIN_NUM_MISO       19
#define SD_CARD_PIN_NUM_CLK        18

#define SPI_DMA_CHAN               1

esp_err_t i2c_slave_init()
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_SLAVE;
    conf.sda_io_num = I2C_SLAVE_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_SLAVE_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    return i2c_param_config(I2C_SLAVE_NUM, &conf);
}

uint32_t receive_sensor_data()
{
    uint8_t buffer[4];
    uint32_t data = 0;
    i2c_slave_read_buffer(I2C_SLAVE_NUM, buffer, sizeof(buffer), portMAX_DELAY);
    for (int i = 0; i < sizeof(buffer); i++) {
        data |= ((uint32_t)buffer[i]) << (8 * (3 - i)); // Shift bytes to its correct position
    }
    return data;
}

esp_err_t initialize_sd_card()
{
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SD_CARD_PIN_NUM_MOSI,
        .miso_io_num = SD_CARD_PIN_NUM_MISO,
        .sclk_io_num = SD_CARD_PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    // Initialize SPI bus
    esp_err_t ret = spi_bus_initialize(VSPI_HOST, &bus_cfg, SPI_DMA_CHAN);
    if (ret != ESP_OK) {
        return ret;
    }

    // Attach SD card to SPI bus
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_CARD_PIN_NUM_CS;
    sdmmc_card_t *card;
    return esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, NULL, &card);
}


void app_main()
{
    // Mount SD card
    if (initialize_sd_card() != ESP_OK) {
        printf("Failed to initialize SD card\n");
        return;
    }

    // Initialize I2C as a slaveS
    if (i2c_slave_init() != ESP_OK) {
        printf("Failed to initialize I2C slave\n");
        return;
    }

    // Open file on SD card
    FILE *file = fopen("/sdcard/sensor_data.csv", "w");
    if (file == NULL) {
        printf("Failed to open file for writing\n");
        return;
    }

    // Listen for data seent all the time
    while (1) {
        // Receive three sensor values over I2C
        uint32_t data1 = receive_sensor_data();
        uint32_t data2 = receive_sensor_data();
        uint32_t data3 = receive_sensor_data();

        // Write received data to the SD card
        fprintf(file, "%lu, %lu, %lu\n", data1, data2, data3);

        // Flush the file to ensure data is written immediately
        fflush(file);
    }

    // Close file (will not be reached in this example)
    fclose(file);
}
