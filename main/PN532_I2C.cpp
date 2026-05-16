#include <cstring>
#include "PN532_I2C.h"
#include "driver/i2c.h"
#include "esp_log.h"
static const char *TAG = "DAEMON";

#define PN532_I2C_ADDRESS       (0x48 >> 1)
#define I2C_TIMEOUT_MS		1000

//#define DEBUG

void PN532_I2C::begin()
{
	const i2c_config_t conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = _sda,
		.scl_io_num = _scl,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master = {.clk_speed = 100000},
		.clk_flags = 0,
	};
	i2c_param_config(_i2cPort, &conf);
	ESP_ERROR_CHECK(i2c_driver_install(_i2cPort, conf.mode, 0, 0, 0));
	i2c_set_timeout(_i2cPort, 31);	// necessary for _8ENCODER_REG_BUTTON
}

int8_t PN532_I2C::writeCommand(const uint8_t *header, uint8_t hlen, const uint8_t *body, uint8_t blen)
{
	int ret;

	uint8_t length = 1+hlen+blen;

	uint8_t i2cBuf[64] = { 
		0x00,0x00,0xFF,
		length,
		(uint8_t)(~length+1),			// checksum of length
		PN532_HOSTTOPN532,
	};
	memcpy(i2cBuf+5+1,      header, hlen);
	memcpy(i2cBuf+5+1+hlen, body, blen);

	uint8_t sum = 0;
	for(uint8_t i = 0; i < length; i++) {
		sum += i2cBuf[5+i];
	}

	i2cBuf[5+length+0] = ~sum+1;
	i2cBuf[5+length+1] = 0x00;
#ifdef DEBUG
	for(int i=0;i<5+length+2;i++) printf("%02x ",i2cBuf[i]);
	printf("\n");
#endif
	ret = i2c_master_write_to_device(_i2cPort, PN532_I2C_ADDRESS, i2cBuf, 5+length+2, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
	if(ret) {
		ESP_LOGE(TAG, "Error(%d,%x)", __LINE__, ret); 
		return ret;
	}

	const uint8_t PN532_ACK[] = {0x00,0x00,0xFF,0x00,0xFF,0x00};

	// ack
	while(1) {
		ret = i2c_master_read_from_device(_i2cPort, PN532_I2C_ADDRESS, i2cBuf, 1+sizeof(PN532_ACK), I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
		if(ret) {
			continue;
		}
		if(i2cBuf[0] & 1) break;
	}

	if(memcmp(i2cBuf+1, PN532_ACK, sizeof(PN532_ACK))) {
		ESP_LOGE(TAG, "Error(%d,%x)", __LINE__, ret); 
		return PN532_INVALID_ACK;
	}

	command = header[0];
	return 0;
}


int16_t PN532_I2C::readResponse(uint8_t buf[], uint8_t len, uint16_t timeout)
{
	int ret;
    uint8_t length;

    const uint8_t PN532_NACK[] = {0x00,0x00,0xFF,0xFF,0x00,0x00};
	uint8_t i2cBuf[64];

	// length(4byte)
	while(1) {
		ret = i2c_master_read_from_device(_i2cPort, PN532_I2C_ADDRESS, i2cBuf, 1+4, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
		if(ret) {
			continue;
		}
		if(i2cBuf[0] & 1) break;
	}

	if(memcmp(i2cBuf+1, PN532_NACK, 3)) {
		ESP_LOGE(TAG, "Error(%d,%x)", __LINE__, ret); 
		return PN532_INVALID_ACK;
	}

	length = i2cBuf[1+3];

	ret = i2c_master_write_to_device(_i2cPort, PN532_I2C_ADDRESS, PN532_NACK, sizeof(PN532_NACK), I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
	if(ret) {
		ESP_LOGE(TAG, "Error(%d,%x)", __LINE__, ret); 
		return ret;
	}

	// recv
	while(1) {
		ret = i2c_master_read_from_device(_i2cPort, PN532_I2C_ADDRESS, i2cBuf, 1+5+length+1, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
		if(ret) {
			continue;
		}
		if(i2cBuf[0] & 1) break;
	}
#ifdef DEBUG
	for(int i=0;i<5+length+1;i++) printf("%02x ",i2cBuf[1+i]);
	printf("\n");
#endif
	uint8_t sum = 0;
	for(uint8_t i = 0; i < length; i++) {
		sum += i2cBuf[1+5+i];
	}

	if(memcmp(i2cBuf+1, PN532_NACK, 3)
	|| i2cBuf[1+3] != length
	|| i2cBuf[1+4] != (uint8_t)(~length+1)
	|| i2cBuf[1+5] != PN532_PN532TOHOST
	|| i2cBuf[1+6] != command+1
	|| i2cBuf[1+5+length+0] != (uint8_t)(~sum+1)) {
		ESP_LOGE(TAG, "Error(%d,%x)", __LINE__, ret); 
		return PN532_INVALID_ACK;
	}

	memcpy(buf, i2cBuf+1+7, length-2);
	return length-2;
}
