#ifndef __PN532_I2C_H__
#define __PN532_I2C_H__

#include "driver/i2c.h"
#include "PN532Interface.h"

class PN532_I2C : public PN532Interface {
public:
    PN532_I2C(i2c_port_t i2cPort, gpio_num_t sda, gpio_num_t scl) {
    	_i2cPort = i2cPort;
    	_sda = sda;
    	_scl = scl;
    }

    void begin();
    void wakeup() {;}
    int8_t writeCommand(const uint8_t *header, uint8_t hlen, const uint8_t *body = 0, uint8_t blen = 0);
    int16_t readResponse(uint8_t buf[], uint8_t len, uint16_t timeout);

private:
    i2c_port_t _i2cPort = I2C_NUM_1;
    gpio_num_t _sda = GPIO_NUM_0;
    gpio_num_t _scl = GPIO_NUM_0;
    uint8_t command = 0;
};

#endif
