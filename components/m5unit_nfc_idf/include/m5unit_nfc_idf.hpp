#pragma once

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"

#include <cstddef>
#include <cstdint>

namespace m5unit::nfc {

enum class FelicaRequestCode : uint8_t {
    None = 0,
    SystemCode = 1,
    CommunicationPerformance = 2,
};

enum class FelicaTimeSlot : uint8_t {
    Slot1 = 0,
    Slot2 = 1,
    Slot4 = 3,
    Slot8 = 7,
    Slot16 = 15,
};

struct FelicaPollingResult {
    uint8_t idm[8]{};
    uint8_t pmm[8]{};
    uint16_t request_data{};
};

class St25r3916 {
public:
    St25r3916(i2c_port_t port, gpio_num_t sda, gpio_num_t scl, uint32_t clock_hz);

    esp_err_t begin();
    esp_err_t polling(FelicaPollingResult& out,
                      uint16_t system_code,
                      FelicaRequestCode request_code = FelicaRequestCode::None,
                      FelicaTimeSlot time_slot = FelicaTimeSlot::Slot1);
    esp_err_t request_service(const FelicaPollingResult& card, uint16_t service_code, uint16_t& key_version);
    esp_err_t read_without_encryption(const FelicaPollingResult& card,
                                      uint16_t service_code,
                                      uint16_t block_number,
                                      uint8_t* out,
                                      size_t out_size);

private:
    esp_err_t configure_i2c();
    esp_err_t configure_nfc_f();
    esp_err_t nfc_initial_field_on();
    esp_err_t transceive(uint8_t* rx, size_t& rx_len, const uint8_t* tx, size_t tx_len, uint32_t timeout_ms);
    esp_err_t transmit(const uint8_t* tx, size_t tx_len, uint32_t timeout_ms);
    esp_err_t receive(uint8_t* rx, size_t& rx_len, uint32_t timeout_ms);

    esp_err_t i2c_write(const uint8_t* data, size_t len);
    esp_err_t i2c_write_read(uint16_t op, uint8_t op_len, uint8_t* data, size_t len);

    esp_err_t read_reg8(uint16_t reg, uint8_t& value);
    esp_err_t write_reg8(uint16_t reg, uint8_t value);
    esp_err_t read_reg16(uint16_t reg, uint16_t& value);
    esp_err_t write_reg16(uint16_t reg, uint16_t value);
    esp_err_t read_reg32(uint16_t reg, uint32_t& value);
    esp_err_t write_reg32(uint16_t reg, uint32_t value);
    esp_err_t modify_reg8(uint16_t reg, uint8_t set_mask, uint8_t clear_mask);
    esp_err_t direct_command(uint8_t command, const uint8_t* data = nullptr, size_t len = 0);

    esp_err_t read_interrupts(uint32_t& value);
    esp_err_t clear_interrupts();
    uint32_t wait_for_interrupt(uint32_t bits, uint32_t timeout_ms);
    esp_err_t write_mask_interrupts(uint32_t mask);
    esp_err_t write_fifo(const uint8_t* data, size_t len);
    esp_err_t read_fifo_size(uint16_t& bytes, uint8_t& bits);
    esp_err_t read_fifo(uint8_t* data, size_t capacity, size_t& actual);
    esp_err_t write_fwt_timer(uint32_t timeout_ms);
    esp_err_t enable_oscillator();

    i2c_port_t port_;
    gpio_num_t sda_;
    gpio_num_t scl_;
    uint32_t clock_hz_;
    uint32_t stored_irq_{};
};

} // namespace m5unit::nfc
