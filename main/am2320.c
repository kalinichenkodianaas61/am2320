#include <stdio.h>
#include <stdint.h>
#include "driver/i2c_types.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

// i2c
#define SDA_GPIO 4
#define SCL_GPIO 5
#define AM2320_ADDR 0x5C // 7-bit i2c address; datasheet gives 0xB8 = 0x5C << 1 with R/W bit 
#define AM2320_SCL_SPEED_HZ 100000 // 100kHz, datasheet maximum for AM2320 

// commands (I2C-Modbus) 
#define AM2320_CMD_READ_REGS 0x03 // function code: read registers 
#define AM2320_REG_HUM_HIGH 0x00 // first data register: humidity high byte 
#define AM2320_DATA_REG_COUNT 4 // hum high/low + temp high/low 
#define AM2320_REG_MODEL_HIGH 0x08 // first info register: model high byte 
#define AM2320_INFO_REG_COUNT 7 // model (2) + version (1) + device ID (4) 
#define AM2320_MAX_REGS 10 // sensor returns max 10 registers per read 

// response layout 
#define AM2320_HEADER_LEN 2 // function code + byte count 
#define AM2320_CRC_LEN 2 // CRC16, low byte first 

// crc16 (Modbus)
#define AM2320_CRC_INIT 0xFFFF // initial CRC register value 
#define AM2320_CRC_POLY 0xA001 // reversed polynomial from datasheet 

// conversion 
#define AM2320_TEMP_SIGN_BIT 0x8000 // bit15 of temp: 1 - negative 
#define AM2320_TEMP_VALUE_MASK 0x7FFF // bit14..bit0: temp value 
#define AM2320_RAW_DIVIDER 10.0f // raw value = real value * 10 
#define AM2320_DP_NUMBER 1 // number of decimal places, sensor resolution is 0.1 

// delays in ms 
#define AM2320_DELAY_POWER_ON_MS 2000 // wait after power-on, datasheet: 2 s, no commands allowed 
#define AM2320_DELAY_WAKE_MS 10 // pause after wake-up, datasheet needs >= 0.8 
#define AM2320_DELAY_CMD_MS 10 // pause between read command and reading data, datasheet needs >= 1.5 
#define AM2320_PERIOD_MS 3000 // pause between measurements, datasheet needs >= 2000 

// i2c transaction timeout 
#define AM2320_XFER_TIMEOUT_MS 100


static const char *TAG = "am2320";


// crc16 as in datasheet, i2c address is not included 
static uint16_t am2320_crc16 (const uint8_t *buf, size_t len) {
	uint16_t crc = AM2320_CRC_INIT;

	while (len--) {
		crc ^= *buf++;
		for (int i = 0; i < 8; i++) {
			if (crc & 0x01) {
				crc = (crc >> 1) ^ AM2320_CRC_POLY;
			} else {
				crc >>= 1;
			}
		}
	}

	return crc;
}


// wake + read command + read answer, returns only register bytes in out 
static esp_err_t am2320_read_regs (i2c_master_bus_handle_t bus, i2c_master_dev_handle_t dev,
                                   uint8_t start_reg, uint8_t count, uint8_t *out) {
	uint8_t read_cmd[3] = {AM2320_CMD_READ_REGS, start_reg, count};
	uint8_t data[AM2320_HEADER_LEN + AM2320_MAX_REGS + AM2320_CRC_LEN];
	size_t data_len = AM2320_HEADER_LEN + count + AM2320_CRC_LEN;
	esp_err_t err;

	if (count == 0 || count > AM2320_MAX_REGS) {
		return ESP_ERR_INVALID_ARG;
	}

	// (1) wake up sensor: address only, sensor does not ACK, so result is ignored
	i2c_master_probe(bus, AM2320_ADDR, AM2320_XFER_TIMEOUT_MS);
	vTaskDelay(pdMS_TO_TICKS(AM2320_DELAY_WAKE_MS));

	// (2) send read command: function code + start register + number of registers 
	err = i2c_master_transmit(dev, read_cmd, sizeof(read_cmd), AM2320_XFER_TIMEOUT_MS);
	if (err != ESP_OK) {
		return err;
	}

	// (3) wait until the sensor prepares the answer 
	vTaskDelay(pdMS_TO_TICKS(AM2320_DELAY_CMD_MS));

	// (4) read answer: function code + byte count + registers + CRC 
	err = i2c_master_receive(dev, data, data_len, AM2320_XFER_TIMEOUT_MS);
	if (err != ESP_OK) {
		return err;
	}

	// (5) check header: must repeat our function code and count 
	if (data[0] != AM2320_CMD_READ_REGS || data[1] != count) {
		return ESP_ERR_INVALID_RESPONSE;
	}

	// (6) check CRC: received low byte first 
	uint16_t crc_received = ((uint16_t)data[data_len - 1] << 8) | data[data_len - 2];
	uint16_t crc_calculated = am2320_crc16(data, data_len - AM2320_CRC_LEN);
	if (crc_received != crc_calculated) {
		return ESP_ERR_INVALID_CRC;
	}

	// (7) copy register bytes without header and CRC 
	for (uint8_t i = 0; i < count; i++) {
		out[i] = data[AM2320_HEADER_LEN + i];
	}

	return ESP_OK;
}


static esp_err_t am2320_init (i2c_master_bus_handle_t *bus, i2c_master_dev_handle_t *dev) {
	esp_err_t err;

	// (1) wait for sensor to wake up 
	vTaskDelay(pdMS_TO_TICKS(AM2320_DELAY_POWER_ON_MS));

	// (2) adding bus 
	i2c_master_bus_config_t bus_cfg = {
		.i2c_port = I2C_NUM_0,
		.sda_io_num = SDA_GPIO,
		.scl_io_num = SCL_GPIO,
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.glitch_ignore_cnt = 7,
		.flags.enable_internal_pullup = true,
	};
	err = i2c_new_master_bus(&bus_cfg, bus);
	if (err != ESP_OK) {
		return err;
	}

	// (3) adding device 
	i2c_device_config_t dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = AM2320_ADDR,
		.scl_speed_hz = AM2320_SCL_SPEED_HZ,
	};
	err = i2c_master_bus_add_device(*bus, &dev_cfg, dev);
	if (err != ESP_OK) {
		return err;
	}

	// (4) check if am2320 answers: read model, version and device ID 
	uint8_t info[AM2320_INFO_REG_COUNT];

	err = am2320_read_regs(*bus, *dev, AM2320_REG_MODEL_HIGH, AM2320_INFO_REG_COUNT, info);
	if (err != ESP_OK) {
		return err;
	}
	uint16_t model = ((uint16_t)info[0] << 8) | info[1];
	uint8_t version = info[2];
	uint32_t device_id = ((uint32_t)info[3] << 24) | ((uint32_t)info[4] << 16) | ((uint32_t)info[5] << 8) | info[6];
	ESP_LOGI(TAG, "model: 0x%04X | version: 0x%02X | id: 0x%08lX", model, version, (unsigned long)device_id);

	// (5) every communication starts a new measurement, so keep the minimum interval before the first read 
	vTaskDelay(pdMS_TO_TICKS(AM2320_PERIOD_MS));

	return ESP_OK;
}




static esp_err_t am2320_read (i2c_master_bus_handle_t bus, i2c_master_dev_handle_t dev, float *hum, float *temp) {
	uint8_t data[AM2320_DATA_REG_COUNT];
	esp_err_t err;

	// (1) wake + command + read 4 data registers (CRC is checked inside) 
	err = am2320_read_regs(bus, dev, AM2320_REG_HUM_HIGH, AM2320_DATA_REG_COUNT, data);
	if (err != ESP_OK) {
		return err;
	}

	// (2) combine bytes 
	// hum = 16 bits -> data[0] (high), data[1] (low)
	// temp = 16 bits -> data[2] (high), data[3] (low), bit15 = sign 
	uint16_t raw_hum = ((uint16_t)data[0] << 8) | data[1];
	uint16_t raw_temp = ((uint16_t)data[2] << 8) | data[3];

	// (3) convert to real values 
	*hum = raw_hum / AM2320_RAW_DIVIDER;
	*temp = (raw_temp & AM2320_TEMP_VALUE_MASK) / AM2320_RAW_DIVIDER;
	if (raw_temp & AM2320_TEMP_SIGN_BIT) {
		*temp = -*temp;
	}

	return ESP_OK;

}



void app_main(void) {
	i2c_master_bus_handle_t bus;
	i2c_master_dev_handle_t am2320;
	ESP_ERROR_CHECK(am2320_init(&bus, &am2320));


	// reading temp/hum in loop 
	while (1) {
		float hum = 0, temp = 0;

		esp_err_t err = am2320_read(bus, am2320, &hum, &temp);
		if (err != ESP_OK) {
			ESP_LOGW(TAG, "am2320 read failed: %s", esp_err_to_name(err));
		} else {
			ESP_LOGI(TAG, "hum: %+7.*f%% | temp: %+7.*fC ",
				AM2320_DP_NUMBER, hum,
				AM2320_DP_NUMBER, temp);
		}

		vTaskDelay(pdMS_TO_TICKS(AM2320_PERIOD_MS));

	}

}