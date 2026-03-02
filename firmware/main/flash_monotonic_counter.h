#pragma once

#include <array>
#include <cstdint>

#include "esp_err.h"
#include "esp_partition.h"

class FlashMonotonicCounter {
public:
    FlashMonotonicCounter();

    esp_err_t init(const char *partition_label);
    esp_err_t increment(uint32_t steps = 1);
    esp_err_t reset();

    uint64_t value() const;

private:
    esp_err_t derive_nvs_keys_from_partition_label_(const char *partition_label);
    esp_err_t load_base_from_nvs_();
    esp_err_t load_rollover_pending_from_nvs_(bool *pending);
    esp_err_t save_rollover_state_to_nvs_(int64_t base_value, bool pending) const;
    int64_t signed_value_() const;
    esp_err_t count_zero_bits_in_partition_();
    esp_err_t clear_bits_range_(uint32_t start_bit, uint32_t bit_count);
    esp_err_t verify_written_bytes_(uint32_t start_byte, const uint8_t *expected, uint32_t bytes_to_check);
    esp_err_t rollover_();

    const esp_partition_t *partition_;
    std::array<char, 16> nvs_base_key_;
    std::array<char, 16> nvs_pending_key_;

    int64_t base_value_;
    uint32_t used_bits_;
    uint32_t total_bits_;
    bool initialized_;
};
