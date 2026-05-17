#include "m5unit_nfc_idf.hpp"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <algorithm>
#include <cstring>

namespace m5unit::nfc {
namespace {
constexpr char TAG[] = "m5unit_nfc";
constexpr uint8_t I2C_ADDR = 0x50;
constexpr uint32_t I2C_TIMEOUT_MS = 100;

constexpr uint8_t REG_IO_CONFIGURATION_1 = 0x00;
constexpr uint8_t REG_IO_CONFIGURATION_2 = 0x01;
constexpr uint8_t REG_OPERATION_CONTROL = 0x02;
constexpr uint8_t REG_MODE_DEFINITION = 0x03;
constexpr uint8_t REG_BITRATE_DEFINITION = 0x04;
constexpr uint8_t REG_NFCIP_1_PASSIVE_TARGET_DEFINITION = 0x08;
constexpr uint8_t REG_AUXILIARY_DEFINITION = 0x0A;
constexpr uint8_t REG_RECEIVER_CONFIGURATION_1 = 0x0B;
constexpr uint8_t REG_RECEIVER_CONFIGURATION_2 = 0x0C;
constexpr uint8_t REG_RECEIVER_CONFIGURATION_3 = 0x0D;
constexpr uint8_t REG_RECEIVER_CONFIGURATION_4 = 0x0E;
constexpr uint8_t REG_NO_RESPONSE_TIMER_1 = 0x10;
constexpr uint8_t REG_TIMER_AND_EMV_CONTROL = 0x12;
constexpr uint8_t REG_MASK_MAIN_INTERRUPT = 0x16;
constexpr uint8_t REG_MAIN_INTERRUPT = 0x1A;
constexpr uint8_t REG_ERROR_AND_WAKEUP_INTERRUPT = 0x1C;
constexpr uint8_t REG_PASSIVE_TARGET_INTERRUPT = 0x1D;
constexpr uint8_t REG_FIFO_STATUS_1 = 0x1E;
constexpr uint8_t REG_NUMBER_OF_TRANSMITTED_BYTES_1 = 0x22;
constexpr uint8_t REG_TX_DRIVER = 0x28;
constexpr uint8_t REG_PASSIVE_TARGET_MODULATION = 0x29;
constexpr uint8_t REG_EXTERNAL_FIELD_DETECTOR_ACTIVATION_THRESHOLD = 0x2A;
constexpr uint8_t REG_EXTERNAL_FIELD_DETECTOR_DEACTIVATION_THRESHOLD = 0x2B;
constexpr uint8_t REG_IC_IDENTITY = 0x3F;

constexpr uint16_t SPACE_B_TAG = 0x1000;
constexpr uint16_t REG_EMD_SUPPRESSION_CONFIGURATION = SPACE_B_TAG | 0x0005;
constexpr uint16_t REG_CORRELATOR_CONFIGURATION_1 = SPACE_B_TAG | 0x000C;
constexpr uint16_t REG_CORRELATOR_CONFIGURATION_2 = SPACE_B_TAG | 0x000D;
constexpr uint16_t REG_RESISTIVE_AM_MODULATION = SPACE_B_TAG | 0x002A;
constexpr uint16_t REG_ANTENNA_TUNING_CONTROL_1 = SPACE_B_TAG | 0x0026;
constexpr uint16_t REG_ANTENNA_TUNING_CONTROL_2 = SPACE_B_TAG | 0x0027;
constexpr uint16_t REG_REGULATOR_DISPLAY = SPACE_B_TAG | 0x002C;

constexpr uint8_t CMD_SET_DEFAULT = 0xC1;
constexpr uint8_t CMD_TRANSMIT_WITH_CRC = 0xC4;
constexpr uint8_t CMD_NFC_INITIAL_FIELD_ON = 0xC8;
constexpr uint8_t CMD_ADJUST_REGULATORS = 0xD6;
constexpr uint8_t CMD_CLEAR_FIFO = 0xDB;
constexpr uint8_t CMD_TEST_ACCESS = 0xFC;

constexpr uint8_t OP_WRITE_REGISTER = 0x00;
constexpr uint8_t OP_READ_REGISTER = 0x40;
constexpr uint8_t OP_LOAD_FIFO = 0x80;
constexpr uint8_t OP_READ_FIFO = 0x9F;
constexpr uint16_t PREFIX_SPACE_B = static_cast<uint16_t>(0xFB) << 8;

constexpr uint8_t VALID_IDENTIFY_TYPE = 0x05;

constexpr uint8_t sup3v = 0x80;
constexpr uint8_t aat_en = 0x20;
constexpr uint8_t io_drv_lvl = 0x04;
constexpr uint16_t i2c_thd016 = 0x1000;
constexpr uint16_t i2c_thd116 = 0x2000;

constexpr uint8_t en = 0x80;
constexpr uint8_t rx_en = 0x40;
constexpr uint8_t tx_en = 0x08;
constexpr uint8_t en_fd_c1 = 0x02;
constexpr uint8_t en_fd_c0 = 0x01;
constexpr uint8_t tr_am = 0x04;
constexpr uint8_t no_crc_rx = 0x80;
constexpr uint8_t nrt_step = 0x01;

constexpr uint8_t I_osc = 0x80;
constexpr uint8_t I_rxs = 0x20;
constexpr uint8_t I_rxe = 0x10;
constexpr uint8_t I_txe = 0x08;
constexpr uint8_t I_nre = 0x40;
constexpr uint32_t I_osc32 = static_cast<uint32_t>(I_osc) << 24;
constexpr uint32_t I_rxs32 = static_cast<uint32_t>(I_rxs) << 24;
constexpr uint32_t I_rxe32 = static_cast<uint32_t>(I_rxe) << 24;
constexpr uint32_t I_txe32 = static_cast<uint32_t>(I_txe) << 24;
constexpr uint32_t I_nre32 = static_cast<uint32_t>(I_nre) << 16;

constexpr uint8_t FELICA_CMD_POLLING = 0x00;
constexpr uint8_t FELICA_RES_POLLING = 0x01;
constexpr uint8_t FELICA_CMD_REQUEST_SERVICE = 0x02;
constexpr uint8_t FELICA_RES_REQUEST_SERVICE = 0x03;
constexpr uint8_t FELICA_CMD_READ_WITHOUT_ENCRYPTION = 0x06;
constexpr uint8_t FELICA_RES_READ_WITHOUT_ENCRYPTION = 0x07;

constexpr uint16_t KEY_VERSION_NONE = 0xFFFF;

uint64_t millis()
{
    return static_cast<uint64_t>(esp_timer_get_time() / 1000);
}

uint8_t to_read_reg8(uint8_t reg)
{
    return (reg & 0x3F) | OP_READ_REGISTER;
}

uint8_t to_write_reg8(uint8_t reg)
{
    return (reg & 0x3F) | OP_WRITE_REGISTER;
}

uint16_t to_read_reg16(uint16_t reg)
{
    return (reg & 0x3F) | PREFIX_SPACE_B | OP_READ_REGISTER;
}

uint16_t to_write_reg16(uint16_t reg)
{
    return (reg & 0x3F) | PREFIX_SPACE_B | OP_WRITE_REGISTER;
}

uint16_t calculate_nrt(uint32_t ms, bool step4096)
{
    constexpr uint32_t FC_HZ = 13560000;
    const uint64_t step_num = static_cast<uint64_t>(step4096 ? 4096 : 64) * 1000000ull;
    const uint64_t us = static_cast<uint64_t>(ms) * 1000ull;
    const uint64_t nrt = (us * FC_HZ + step_num - 1) / step_num;
    return static_cast<uint16_t>(std::clamp<uint64_t>(nrt, 1ull, 0xFFFFull));
}

uint8_t timeslot_to_slots(FelicaTimeSlot slot)
{
    switch (slot) {
    case FelicaTimeSlot::Slot16:
        return 16;
    case FelicaTimeSlot::Slot8:
        return 8;
    case FelicaTimeSlot::Slot4:
        return 4;
    case FelicaTimeSlot::Slot2:
        return 2;
    case FelicaTimeSlot::Slot1:
    default:
        return 1;
    }
}
} // namespace

St25r3916::St25r3916(i2c_port_t port, gpio_num_t sda, gpio_num_t scl, uint32_t clock_hz)
    : port_(port), sda_(sda), scl_(scl), clock_hz_(clock_hz)
{
}

esp_err_t St25r3916::begin()
{
    ESP_RETURN_ON_ERROR(configure_i2c(), TAG, "I2C init failed");

    uint8_t identity = 0;
    ESP_RETURN_ON_ERROR(read_reg8(REG_IC_IDENTITY, identity), TAG, "read identity failed");
    const uint8_t type = (identity >> 3) & 0x1F;
    const uint8_t rev = identity & 0x07;
    if (type != VALID_IDENTIFY_TYPE || rev == 0) {
        ESP_LOGE(TAG, "ST25R3916 not detected: identity=%02X type=%02X rev=%02X", identity, type, rev);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "ST25R3916 identity=%02X type=%02X rev=%u", identity, type, rev);

    ESP_RETURN_ON_ERROR(direct_command(CMD_SET_DEFAULT), TAG, "set default failed");
    const uint8_t protection_command[] = {0x04, 0x10};
    ESP_RETURN_ON_ERROR(direct_command(CMD_TEST_ACCESS, protection_command, sizeof(protection_command)),
                        TAG, "protection command failed");

    uint16_t io_config = io_drv_lvl | sup3v;
    if (clock_hz_ >= 1000000) {
        io_config |= i2c_thd016 | i2c_thd116;
    } else if (clock_hz_ >= 400000) {
        io_config |= i2c_thd016;
    }
    ESP_RETURN_ON_ERROR(write_reg16(REG_IO_CONFIGURATION_1, io_config), TAG, "IO config failed");
    ESP_RETURN_ON_ERROR(write_reg8(REG_TX_DRIVER, 0x00), TAG, "TX driver failed");

    ESP_RETURN_ON_ERROR(modify_reg8(REG_IO_CONFIGURATION_1, 0x07, 0x07), TAG, "disable MCU clock failed");
    ESP_RETURN_ON_ERROR(write_reg8(REG_RESISTIVE_AM_MODULATION, 0x80), TAG, "AM modulation setup failed");
    ESP_RETURN_ON_ERROR(modify_reg8(REG_IO_CONFIGURATION_2, aat_en, 0x00), TAG, "AAT enable failed");
    ESP_RETURN_ON_ERROR(write_reg8(REG_RESISTIVE_AM_MODULATION, 0x00), TAG, "AM modulation normal failed");
    ESP_RETURN_ON_ERROR(write_reg8(REG_EXTERNAL_FIELD_DETECTOR_ACTIVATION_THRESHOLD, 0x13), TAG, "EFD on failed");
    ESP_RETURN_ON_ERROR(write_reg8(REG_EXTERNAL_FIELD_DETECTOR_DEACTIVATION_THRESHOLD, 0x02), TAG, "EFD off failed");
    ESP_RETURN_ON_ERROR(modify_reg8(REG_NFCIP_1_PASSIVE_TARGET_DEFINITION, 0x50, 0xF0), TAG, "FDT failed");
    ESP_RETURN_ON_ERROR(write_reg8(REG_PASSIVE_TARGET_MODULATION, 0x5F), TAG, "PT modulation failed");
    ESP_RETURN_ON_ERROR(write_reg8(REG_EMD_SUPPRESSION_CONFIGURATION, 0x40), TAG, "EMD failed");
    ESP_RETURN_ON_ERROR(write_reg8(REG_ANTENNA_TUNING_CONTROL_1, 0x82), TAG, "AAT1 failed");
    ESP_RETURN_ON_ERROR(write_reg8(REG_ANTENNA_TUNING_CONTROL_2, 0x82), TAG, "AAT2 failed");
    ESP_RETURN_ON_ERROR(modify_reg8(REG_OPERATION_CONTROL, en_fd_c1 | en_fd_c0, 0x00), TAG, "EFD auto failed");
    ESP_RETURN_ON_ERROR(direct_command(CMD_CLEAR_FIFO), TAG, "clear FIFO failed");

    ESP_RETURN_ON_ERROR(write_mask_interrupts(0xFFFF00FF), TAG, "mask interrupts failed");
    ESP_RETURN_ON_ERROR(clear_interrupts(), TAG, "clear interrupts failed");
    ESP_RETURN_ON_ERROR(enable_oscillator(), TAG, "oscillator failed");
    ESP_RETURN_ON_ERROR(write_mask_interrupts(0), TAG, "unmask interrupts failed");
    ESP_RETURN_ON_ERROR(direct_command(CMD_ADJUST_REGULATORS), TAG, "adjust regulators failed");
    vTaskDelay(pdMS_TO_TICKS(5));

    uint8_t regulator = 0;
    if (read_reg8(REG_REGULATOR_DISPLAY, regulator) == ESP_OK) {
        ESP_LOGI(TAG, "regulator display=%02X", regulator);
    }

    return configure_nfc_f();
}

esp_err_t St25r3916::polling(FelicaPollingResult& out,
                             uint16_t system_code,
                             FelicaRequestCode request_code,
                             FelicaTimeSlot time_slot)
{
    std::memset(&out, 0, sizeof(out));
    const uint8_t packet[] = {
        FELICA_CMD_POLLING,
        static_cast<uint8_t>(system_code >> 8),
        static_cast<uint8_t>(system_code & 0xFF),
        static_cast<uint8_t>(request_code),
        static_cast<uint8_t>(time_slot),
    };

    uint8_t response[20]{};
    size_t response_len = sizeof(response);
    const uint32_t timeout_ms = 5 * 2 * timeslot_to_slots(time_slot);
    esp_err_t err = transceive(response, response_len, packet, sizeof(packet), timeout_ms);
    if (err != ESP_OK) {
        return err;
    }

    const size_t minimum_for_idm = 10;
    if (response_len < minimum_for_idm || response[1] != FELICA_RES_POLLING) {
        ESP_LOGW(TAG, "unexpected polling response len=%u code=%02X len_byte=%02X",
                 static_cast<unsigned>(response_len), response_len > 1 ? response[1] : 0, response_len ? response[0] : 0);
        return ESP_ERR_INVALID_RESPONSE;
    }

    std::memcpy(out.idm, response + 2, sizeof(out.idm));
    const size_t pmm_len = response_len > 10 ? std::min<size_t>(sizeof(out.pmm), response_len - 10) : 0;
    if (pmm_len) {
        std::memcpy(out.pmm, response + 10, pmm_len);
    }
    if (response_len >= 20) {
        out.request_data = (static_cast<uint16_t>(response[18]) << 8) | response[19];
    }
    return ESP_OK;
}

esp_err_t St25r3916::request_service(const FelicaPollingResult& card, uint16_t service_code, uint16_t& key_version)
{
    key_version = KEY_VERSION_NONE;
    uint8_t packet[1 + 8 + 1 + 2]{};
    packet[0] = FELICA_CMD_REQUEST_SERVICE;
    std::memcpy(packet + 1, card.idm, sizeof(card.idm));
    packet[9] = 1;
    packet[10] = service_code & 0xFF;
    packet[11] = service_code >> 8;

    uint8_t response[1 + 1 + 8 + 1 + 2]{};
    size_t response_len = sizeof(response);
    ESP_RETURN_ON_ERROR(transceive(response, response_len, packet, sizeof(packet), 20), TAG, "request service");

    if (response_len < sizeof(response) || response[1] != FELICA_RES_REQUEST_SERVICE || response[10] < 1) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    key_version = (static_cast<uint16_t>(response[12]) << 8) | response[11];
    return ESP_OK;
}

esp_err_t St25r3916::read_without_encryption(const FelicaPollingResult& card,
                                             uint16_t service_code,
                                             uint16_t block_number,
                                             uint8_t* out,
                                             size_t out_size)
{
    if (!out || out_size < 16) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t packet[1 + 8 + 1 + 2 + 1 + 3]{};
    size_t offset = 0;
    packet[offset++] = FELICA_CMD_READ_WITHOUT_ENCRYPTION;
    std::memcpy(packet + offset, card.idm, sizeof(card.idm));
    offset += sizeof(card.idm);
    packet[offset++] = 1;
    packet[offset++] = service_code & 0xFF;
    packet[offset++] = service_code >> 8;
    packet[offset++] = 1;
    if (block_number > 0xFF) {
        packet[offset++] = 0x00;
        packet[offset++] = block_number & 0xFF;
        packet[offset++] = block_number >> 8;
    } else {
        packet[offset++] = 0x80;
        packet[offset++] = block_number & 0xFF;
    }

    uint8_t response[1 + 1 + 8 + 1 + 1 + 1 + 16]{};
    size_t response_len = sizeof(response);
    ESP_RETURN_ON_ERROR(transceive(response, response_len, packet, offset, 50), TAG, "read without encryption");

    if (response_len < 13 || response[1] != FELICA_RES_READ_WITHOUT_ENCRYPTION ||
        response[10] != 0x00 || response[11] != 0x00 || response[12] < 1) {
        ESP_LOGW(TAG, "read response invalid len=%u len_byte=%02X code=%02X s1=%02X s2=%02X blocks=%02X",
                 static_cast<unsigned>(response_len), response_len > 0 ? response[0] : 0,
                 response_len > 1 ? response[1] : 0, response_len > 10 ? response[10] : 0,
                 response_len > 11 ? response[11] : 0, response_len > 12 ? response[12] : 0);
        if (response_len) {
            ESP_LOG_BUFFER_HEX_LEVEL(TAG, response, response_len, ESP_LOG_WARN);
        }
        return ESP_ERR_INVALID_RESPONSE;
    }

    const size_t data_len = response_len > 13 ? std::min<size_t>(16, response_len - 13) : 0;
    if (data_len < 8) {
        ESP_LOGW(TAG, "read response has too little block data len=%u data=%u len_byte=%02X",
                 static_cast<unsigned>(response_len), static_cast<unsigned>(data_len), response[0]);
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, response, response_len, ESP_LOG_WARN);
        return ESP_ERR_INVALID_RESPONSE;
    }
    std::memset(out, 0, out_size);
    std::memcpy(out, response + 13, data_len);
    return ESP_OK;
}

esp_err_t St25r3916::configure_i2c()
{
    i2c_config_t conf{};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda_;
    conf.scl_io_num = scl_;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = clock_hz_;
    conf.clk_flags = 0;

    esp_err_t err = i2c_param_config(port_, &conf);
    if (err != ESP_OK) {
        return err;
    }
    err = i2c_driver_install(port_, conf.mode, 0, 0, 0);
    if (err == ESP_ERR_INVALID_STATE) {
        return ESP_OK;
    }
    return err;
}

esp_err_t St25r3916::configure_nfc_f()
{
    ESP_RETURN_ON_ERROR(write_reg8(REG_MODE_DEFINITION, 0x18 | tr_am), TAG, "mode FeliCa failed");
    ESP_RETURN_ON_ERROR(write_reg8(REG_BITRATE_DEFINITION, 0x11), TAG, "bitrate 212k failed");
    ESP_RETURN_ON_ERROR(modify_reg8(REG_OPERATION_CONTROL, en_fd_c1 | en_fd_c0, 0x00), TAG, "operation failed");
    ESP_RETURN_ON_ERROR(write_reg8(REG_IO_CONFIGURATION_1, 0x07), TAG, "io config 1 failed");
    ESP_RETURN_ON_ERROR(write_reg8(REG_AUXILIARY_DEFINITION, 0x00), TAG, "aux failed");
    ESP_RETURN_ON_ERROR(write_reg8(REG_RECEIVER_CONFIGURATION_1, 0x13), TAG, "rx1 failed");
    ESP_RETURN_ON_ERROR(write_reg8(REG_RECEIVER_CONFIGURATION_2, 0x3D), TAG, "rx2 failed");
    ESP_RETURN_ON_ERROR(write_reg8(REG_RECEIVER_CONFIGURATION_3, 0x00), TAG, "rx3 failed");
    ESP_RETURN_ON_ERROR(write_reg8(REG_RECEIVER_CONFIGURATION_4, 0x00), TAG, "rx4 failed");
    ESP_RETURN_ON_ERROR(write_reg8(REG_CORRELATOR_CONFIGURATION_1, 0x54), TAG, "corr1 failed");
    ESP_RETURN_ON_ERROR(write_reg8(REG_CORRELATOR_CONFIGURATION_2, 0x00), TAG, "corr2 failed");
    ESP_RETURN_ON_ERROR(write_mask_interrupts(0), TAG, "mask failed");
    return nfc_initial_field_on();
}

esp_err_t St25r3916::nfc_initial_field_on()
{
    uint8_t op = 0;
    ESP_RETURN_ON_ERROR(read_reg8(REG_OPERATION_CONTROL, op), TAG, "read op failed");
    if (op & tx_en) {
        return ESP_ERR_INVALID_STATE;
    }
    ESP_RETURN_ON_ERROR(direct_command(CMD_NFC_INITIAL_FIELD_ON), TAG, "field on failed");
    vTaskDelay(pdMS_TO_TICKS(5));
    return modify_reg8(REG_OPERATION_CONTROL, tx_en | rx_en, 0x00);
}

esp_err_t St25r3916::transceive(uint8_t* rx, size_t& rx_len, const uint8_t* tx, size_t tx_len, uint32_t timeout_ms)
{
    ESP_RETURN_ON_ERROR(transmit(tx, tx_len, timeout_ms), TAG, "transmit failed");
    return receive(rx, rx_len, timeout_ms);
}

esp_err_t St25r3916::transmit(const uint8_t* tx, size_t tx_len, uint32_t timeout_ms)
{
    if (!tx || tx_len == 0 || tx_len > 512) {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_RETURN_ON_ERROR(write_fwt_timer(timeout_ms), TAG, "timer failed");
    ESP_RETURN_ON_ERROR(modify_reg8(REG_AUXILIARY_DEFINITION, 0x00, no_crc_rx), TAG, "crc rx failed");
    ESP_RETURN_ON_ERROR(clear_interrupts(), TAG, "clear irq failed");
    ESP_RETURN_ON_ERROR(direct_command(CMD_CLEAR_FIFO), TAG, "clear fifo failed");
    ESP_RETURN_ON_ERROR(write_fifo(tx, tx_len), TAG, "write fifo failed");
    ESP_RETURN_ON_ERROR(write_reg16(REG_NUMBER_OF_TRANSMITTED_BYTES_1, static_cast<uint16_t>(tx_len << 3)),
                        TAG, "tx bytes failed");
    ESP_RETURN_ON_ERROR(direct_command(CMD_TRANSMIT_WITH_CRC), TAG, "tx command failed");

    const uint32_t irq = wait_for_interrupt(I_txe32, timeout_ms);
    return (irq & I_txe32) ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t St25r3916::receive(uint8_t* rx, size_t& rx_len, uint32_t timeout_ms)
{
    if (!rx || rx_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    const size_t capacity = rx_len;
    rx_len = 0;
    const uint64_t deadline = millis() + timeout_ms + 20;
    uint32_t irq = wait_for_interrupt(I_rxe32 | I_rxs32, timeout_ms);
    if ((irq & I_rxe32) == 0 && (irq & I_rxs32) == 0) {
        return ESP_ERR_TIMEOUT;
    }

    uint16_t bytes = 0;
    uint8_t bits = 0;
    do {
        if (irq & I_rxe32) {
            break;
        }
        if (read_fifo_size(bytes, bits) == ESP_OK && bytes >= 2) {
            break;
        }
        uint32_t extra_irq = 0;
        if (read_interrupts(extra_irq) == ESP_OK) {
            irq |= extra_irq;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    } while (millis() <= deadline);

    vTaskDelay(pdMS_TO_TICKS(2));
    return read_fifo(rx, capacity, rx_len);
}

esp_err_t St25r3916::i2c_write(const uint8_t* data, size_t len)
{
    return i2c_master_write_to_device(port_, I2C_ADDR, data, len, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

esp_err_t St25r3916::i2c_write_read(uint16_t op, uint8_t op_len, uint8_t* data, size_t len)
{
    uint8_t cmd[2]{static_cast<uint8_t>(op >> 8), static_cast<uint8_t>(op & 0xFF)};
    const uint8_t* cmd_ptr = op_len == 2 ? cmd : cmd + 1;
    return i2c_master_write_read_device(port_, I2C_ADDR, cmd_ptr, op_len, data, len, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

esp_err_t St25r3916::read_reg8(uint16_t reg, uint8_t& value)
{
    const bool space_b = reg > 0xFF;
    const uint16_t op = space_b ? to_read_reg16(reg) : to_read_reg8(static_cast<uint8_t>(reg));
    return i2c_write_read(op, space_b ? 2 : 1, &value, 1);
}

esp_err_t St25r3916::write_reg8(uint16_t reg, uint8_t value)
{
    const bool space_b = reg > 0xFF;
    const uint16_t op = space_b ? to_write_reg16(reg) : to_write_reg8(static_cast<uint8_t>(reg));
    uint8_t data[3]{static_cast<uint8_t>(op >> 8), static_cast<uint8_t>(op & 0xFF), value};
    return i2c_write(space_b ? data : data + 1, space_b ? 3 : 2);
}

esp_err_t St25r3916::read_reg16(uint16_t reg, uint16_t& value)
{
    uint8_t data[2]{};
    const bool space_b = reg > 0xFF;
    const uint16_t op = space_b ? to_read_reg16(reg) : to_read_reg8(static_cast<uint8_t>(reg));
    ESP_RETURN_ON_ERROR(i2c_write_read(op, space_b ? 2 : 1, data, sizeof(data)), TAG, "read16");
    value = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    return ESP_OK;
}

esp_err_t St25r3916::write_reg16(uint16_t reg, uint16_t value)
{
    const bool space_b = reg > 0xFF;
    const uint16_t op = space_b ? to_write_reg16(reg) : to_write_reg8(static_cast<uint8_t>(reg));
    uint8_t data[4]{static_cast<uint8_t>(op >> 8), static_cast<uint8_t>(op & 0xFF),
                    static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value & 0xFF)};
    return i2c_write(space_b ? data : data + 1, space_b ? 4 : 3);
}

esp_err_t St25r3916::read_reg32(uint16_t reg, uint32_t& value)
{
    uint8_t data[4]{};
    const bool space_b = reg > 0xFF;
    const uint16_t op = space_b ? to_read_reg16(reg) : to_read_reg8(static_cast<uint8_t>(reg));
    ESP_RETURN_ON_ERROR(i2c_write_read(op, space_b ? 2 : 1, data, sizeof(data)), TAG, "read32");
    value = (static_cast<uint32_t>(data[0]) << 24) | (static_cast<uint32_t>(data[1]) << 16) |
            (static_cast<uint32_t>(data[2]) << 8) | data[3];
    return ESP_OK;
}

esp_err_t St25r3916::write_reg32(uint16_t reg, uint32_t value)
{
    const bool space_b = reg > 0xFF;
    const uint16_t op = space_b ? to_write_reg16(reg) : to_write_reg8(static_cast<uint8_t>(reg));
    uint8_t data[6]{static_cast<uint8_t>(op >> 8), static_cast<uint8_t>(op & 0xFF),
                    static_cast<uint8_t>(value >> 24), static_cast<uint8_t>(value >> 16),
                    static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value)};
    return i2c_write(space_b ? data : data + 1, space_b ? 6 : 5);
}

esp_err_t St25r3916::modify_reg8(uint16_t reg, uint8_t set_mask, uint8_t clear_mask)
{
    uint8_t value = 0;
    ESP_RETURN_ON_ERROR(read_reg8(reg, value), TAG, "modify read");
    const uint8_t next = (value & ~clear_mask) | set_mask;
    return next == value ? ESP_OK : write_reg8(reg, next);
}

esp_err_t St25r3916::direct_command(uint8_t command, const uint8_t* data, size_t len)
{
    if (!data || len == 0) {
        return i2c_write(&command, 1);
    }
    uint8_t buffer[16]{};
    if (len + 1 > sizeof(buffer)) {
        return ESP_ERR_INVALID_ARG;
    }
    buffer[0] = command;
    std::memcpy(buffer + 1, data, len);
    return i2c_write(buffer, len + 1);
}

esp_err_t St25r3916::read_interrupts(uint32_t& value)
{
    value = 0;
    uint8_t error = 0;
    uint16_t main_nfc = 0;
    uint8_t passive = 0;
    ESP_RETURN_ON_ERROR(read_reg8(REG_ERROR_AND_WAKEUP_INTERRUPT, error), TAG, "read error irq");
    ESP_RETURN_ON_ERROR(read_reg16(REG_MAIN_INTERRUPT, main_nfc), TAG, "read main irq");
    ESP_RETURN_ON_ERROR(read_reg8(REG_PASSIVE_TARGET_INTERRUPT, passive), TAG, "read passive irq");
    value = (static_cast<uint32_t>(main_nfc) << 16) | (static_cast<uint32_t>(error) << 8) | passive;
    return ESP_OK;
}

esp_err_t St25r3916::clear_interrupts()
{
    stored_irq_ = 0;
    uint32_t discard = 0;
    return read_reg32(REG_MAIN_INTERRUPT, discard);
}

uint32_t St25r3916::wait_for_interrupt(uint32_t bits, uint32_t timeout_ms)
{
    const uint64_t deadline = millis() + timeout_ms;
    do {
        uint32_t irq = 0;
        if (read_interrupts(irq) == ESP_OK) {
            stored_irq_ |= irq;
        }
        const uint32_t matched = stored_irq_ & bits;
        if (matched) {
            stored_irq_ &= ~matched;
            return matched;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    } while (millis() <= deadline);
    return stored_irq_ | I_nre32;
}

esp_err_t St25r3916::write_mask_interrupts(uint32_t mask)
{
    return write_reg32(REG_MASK_MAIN_INTERRUPT, mask);
}

esp_err_t St25r3916::write_fifo(const uint8_t* data, size_t len)
{
    if (!data || len == 0 || len > 512) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t buffer[256]{};
    if (len + 1 > sizeof(buffer)) {
        return ESP_ERR_INVALID_ARG;
    }
    buffer[0] = OP_LOAD_FIFO;
    std::memcpy(buffer + 1, data, len);
    return i2c_write(buffer, len + 1);
}

esp_err_t St25r3916::read_fifo_size(uint16_t& bytes, uint8_t& bits)
{
    uint16_t status = 0;
    ESP_RETURN_ON_ERROR(read_reg16(REG_FIFO_STATUS_1, status), TAG, "fifo status");
    bytes = (status >> 8) | ((status & 0x00C0) << 2);
    bits = (status >> 1) & 0x07;
    return ESP_OK;
}

esp_err_t St25r3916::read_fifo(uint8_t* data, size_t capacity, size_t& actual)
{
    actual = 0;
    uint16_t bytes = 0;
    uint8_t bits = 0;
    ESP_RETURN_ON_ERROR(read_fifo_size(bytes, bits), TAG, "fifo size");
    const size_t to_read = std::min<size_t>(bytes, capacity);
    if (to_read == 0) {
        return ESP_ERR_INVALID_SIZE;
    }
    ESP_RETURN_ON_ERROR(i2c_write_read(OP_READ_FIFO, 1, data, to_read), TAG, "read fifo");
    actual = to_read;
    return ESP_OK;
}

esp_err_t St25r3916::write_fwt_timer(uint32_t timeout_ms)
{
    uint8_t timer_ctrl = 0;
    ESP_RETURN_ON_ERROR(read_reg8(REG_TIMER_AND_EMV_CONTROL, timer_ctrl), TAG, "read timer ctrl");
    const uint16_t nrt = calculate_nrt(timeout_ms, (timer_ctrl & nrt_step) != 0);
    return write_reg16(REG_NO_RESPONSE_TIMER_1, nrt);
}

esp_err_t St25r3916::enable_oscillator()
{
    uint8_t op = 0;
    ESP_RETURN_ON_ERROR(read_reg8(REG_OPERATION_CONTROL, op), TAG, "read op");
    if ((op & en) == 0) {
        ESP_RETURN_ON_ERROR(modify_reg8(REG_MASK_MAIN_INTERRUPT, 0x00, I_osc), TAG, "unmask osc");
        ESP_RETURN_ON_ERROR(clear_interrupts(), TAG, "clear irq");
        ESP_RETURN_ON_ERROR(modify_reg8(REG_OPERATION_CONTROL, en, 0x00), TAG, "enable osc");
        const uint32_t irq = wait_for_interrupt(I_osc32, 10);
        ESP_RETURN_ON_ERROR(modify_reg8(REG_MASK_MAIN_INTERRUPT, I_osc, 0x00), TAG, "mask osc");
        if ((irq & I_osc32) == 0) {
            return ESP_ERR_TIMEOUT;
        }
    }
    return ESP_OK;
}

} // namespace m5unit::nfc
