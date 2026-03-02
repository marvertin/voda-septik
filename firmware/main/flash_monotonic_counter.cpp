#include "flash_monotonic_counter.h"

extern "C" {
#include "nvs.h"
#include "esp_log.h"
}

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>

namespace {
constexpr size_t SCAN_CHUNK_SIZE = 256;
constexpr size_t WRITE_CHUNK_SIZE = 256;
constexpr const char *NVS_NAMESPACE = "flash_ctr";
constexpr const char *TAG = "monotonic";

uint32_t fnv1a32(const char *text)
{
    uint32_t hash = 2166136261u;
    while (*text != '\0') {
        hash ^= static_cast<uint8_t>(*text);
        hash *= 16777619u;
        ++text;
    }
    return hash;
}
}

#ifndef FLASH_MONOTONIC_COUNTER_VERIFY_WRITES
#define FLASH_MONOTONIC_COUNTER_VERIFY_WRITES 0
#endif

FlashMonotonicCounter::FlashMonotonicCounter()
    : partition_(nullptr),
      nvs_base_key_{},
      nvs_pending_key_{},
      base_value_(0),
      used_bits_(0),
      total_bits_(0),
      initialized_(false)
{
}

esp_err_t FlashMonotonicCounter::init(const char *partition_label)
{
    if (partition_label == nullptr || partition_label[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    partition_ = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA,
        ESP_PARTITION_SUBTYPE_ANY,
        partition_label);
    if (partition_ == nullptr) {
        return ESP_ERR_NOT_FOUND;
    }

    esp_err_t result = derive_nvs_keys_from_partition_label_(partition_label);
    if (result != ESP_OK) {
        return result;
    }

    total_bits_ = partition_->size * 8;

    result = load_base_from_nvs_();
    if (result != ESP_OK) {
        return result;
    }

    bool rollover_pending = false;
    result = load_rollover_pending_from_nvs_(&rollover_pending);
    if (result != ESP_OK) {
        return result;
    }

    if (rollover_pending) {
        result = esp_partition_erase_range(partition_, 0, partition_->size);
        if (result != ESP_OK) {
            return result;
        }

        used_bits_ = 0;
        result = save_rollover_state_to_nvs_(base_value_, false);
        if (result != ESP_OK) {
            return result;
        }
    } else {
        result = count_zero_bits_in_partition_();
        if (result != ESP_OK) {
            return result;
        }
    }

    initialized_ = true;
    return ESP_OK;
}

esp_err_t FlashMonotonicCounter::derive_nvs_keys_from_partition_label_(const char *partition_label)
{
    const uint32_t hash = fnv1a32(partition_label);

    const int base_written = std::snprintf(
        nvs_base_key_.data(),
        nvs_base_key_.size(),
        "b_%08lx",
        static_cast<unsigned long>(hash));
    const int pending_written = std::snprintf(
        nvs_pending_key_.data(),
        nvs_pending_key_.size(),
        "p_%08lx",
        static_cast<unsigned long>(hash));

    if (base_written <= 0 || static_cast<size_t>(base_written) >= nvs_base_key_.size()) {
        return ESP_ERR_INVALID_SIZE;
    }
    if (pending_written <= 0 || static_cast<size_t>(pending_written) >= nvs_pending_key_.size()) {
        return ESP_ERR_INVALID_SIZE;
    }

    return ESP_OK;
}

esp_err_t FlashMonotonicCounter::increment(uint32_t steps)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t remaining_steps = steps;
    while (remaining_steps > 0) {
        if (used_bits_ >= total_bits_) {
            esp_err_t rollover_result = rollover_();
            if (rollover_result != ESP_OK) {
                return rollover_result;
            }
        }

        const uint32_t free_bits = total_bits_ - used_bits_;
        const uint32_t block_bits = std::min<uint32_t>(remaining_steps, std::min<uint32_t>(free_bits, WRITE_CHUNK_SIZE * 8));

        esp_err_t clear_result = clear_bits_range_(used_bits_, block_bits);
        if (clear_result != ESP_OK) {
            return clear_result;
        }

        used_bits_ += block_bits;
        remaining_steps -= block_bits;
    }

    return ESP_OK;
}

esp_err_t FlashMonotonicCounter::reset()
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t result = esp_partition_erase_range(partition_, 0, partition_->size);
    if (result != ESP_OK) {
        return result;
    }

    base_value_ = 0;
    used_bits_ = 0;

    return save_rollover_state_to_nvs_(base_value_, false);
}

uint64_t FlashMonotonicCounter::value() const
{
    const int64_t current = signed_value_();
    return current <= 0 ? 0 : static_cast<uint64_t>(current);
}

int64_t FlashMonotonicCounter::signed_value_() const
{
    return base_value_ + static_cast<int64_t>(used_bits_);
}

esp_err_t FlashMonotonicCounter::load_base_from_nvs_()
{
    nvs_handle_t handle = 0;
    esp_err_t result = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (result != ESP_OK) {
        return result;
    }

    int64_t base_value = 0;
    result = nvs_get_i64(handle, nvs_base_key_.data(), &base_value);
    if (result == ESP_ERR_NVS_NOT_FOUND) {
        base_value = 0;
        result = nvs_set_i64(handle, nvs_base_key_.data(), base_value);
        if (result == ESP_OK) {
            result = nvs_commit(handle);
        }
    }

    if (result == ESP_OK) {
        base_value_ = base_value;
    }

    nvs_close(handle);
    return result;
}

esp_err_t FlashMonotonicCounter::load_rollover_pending_from_nvs_(bool *pending)
{
    if (pending == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle = 0;
    esp_err_t result = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (result != ESP_OK) {
        return result;
    }

    uint8_t pending_value = 0;
    result = nvs_get_u8(handle, nvs_pending_key_.data(), &pending_value);
    if (result == ESP_ERR_NVS_NOT_FOUND) {
        pending_value = 0;
        result = ESP_OK;
    }

    if (result == ESP_OK) {
        *pending = (pending_value != 0);
    }

    nvs_close(handle);
    return result;
}

esp_err_t FlashMonotonicCounter::save_rollover_state_to_nvs_(int64_t base_value, bool pending) const
{
    nvs_handle_t handle = 0;
    esp_err_t result = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (result != ESP_OK) {
        return result;
    }

    result = nvs_set_i64(handle, nvs_base_key_.data(), base_value);
    if (result == ESP_OK) {
        result = nvs_set_u8(handle, nvs_pending_key_.data(), pending ? 1 : 0);
    }
    if (result == ESP_OK) {
        result = nvs_commit(handle);
    }

    nvs_close(handle);
    return result;
}

esp_err_t FlashMonotonicCounter::count_zero_bits_in_partition_()
{
    std::array<uint8_t, SCAN_CHUNK_SIZE> buffer = {};
    used_bits_ = 0;

    size_t offset = 0;
    while (offset < partition_->size) {
        const size_t bytes_to_read = std::min<size_t>(buffer.size(), static_cast<size_t>(partition_->size) - offset);
        esp_err_t result = esp_partition_read(partition_, offset, buffer.data(), bytes_to_read);
        if (result != ESP_OK) {
            return result;
        }

        for (size_t i = 0; i < bytes_to_read; ++i) {
            used_bits_ += __builtin_popcount(static_cast<unsigned int>(~buffer[i] & 0xFF));
        }

        offset += bytes_to_read;
    }

    return ESP_OK;
}

esp_err_t FlashMonotonicCounter::clear_bits_range_(uint32_t start_bit, uint32_t bit_count)
{
    if (bit_count == 0) {
        return ESP_OK;
    }

    if (start_bit >= total_bits_ || bit_count > (total_bits_ - start_bit)) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint32_t start_byte = start_bit / 8;
    const uint32_t end_bit = start_bit + bit_count;
    const uint32_t end_byte = (end_bit + 7) / 8;
    const uint32_t bytes_to_write = end_byte - start_byte;

    std::array<uint8_t, WRITE_CHUNK_SIZE> buffer = {};
    if (bytes_to_write > buffer.size()) {
        return ESP_ERR_INVALID_SIZE;
    }

    esp_err_t read_result = esp_partition_read(partition_, start_byte, buffer.data(), bytes_to_write);
    if (read_result != ESP_OK) {
        return read_result;
    }

    for (uint32_t byte_offset = 0; byte_offset < bytes_to_write; ++byte_offset) {
        const uint32_t abs_byte = start_byte + byte_offset;
        const uint32_t byte_first_bit = abs_byte * 8;
        const uint32_t clear_from = std::max(start_bit, byte_first_bit);
        const uint32_t clear_to = std::min(end_bit, byte_first_bit + 8);

        for (uint32_t bit = clear_from; bit < clear_to; ++bit) {
            buffer[byte_offset] = static_cast<uint8_t>(buffer[byte_offset] & ~(1U << (bit - byte_first_bit)));
        }
    }

    esp_err_t write_result = esp_partition_write(partition_, start_byte, buffer.data(), bytes_to_write);
    if (write_result != ESP_OK) {
        return write_result;
    }

#if FLASH_MONOTONIC_COUNTER_VERIFY_WRITES
    return verify_written_bytes_(start_byte, buffer.data(), bytes_to_write);
#else
    return ESP_OK;
#endif
}

esp_err_t FlashMonotonicCounter::verify_written_bytes_(uint32_t start_byte, const uint8_t *expected, uint32_t bytes_to_check)
{
    if (expected == nullptr || bytes_to_check == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    std::array<uint8_t, WRITE_CHUNK_SIZE> verify_buffer = {};
    if (bytes_to_check > verify_buffer.size()) {
        return ESP_ERR_INVALID_SIZE;
    }

    esp_err_t read_result = esp_partition_read(partition_, start_byte, verify_buffer.data(), bytes_to_check);
    if (read_result != ESP_OK) {
        return read_result;
    }

    if (std::memcmp(verify_buffer.data(), expected, bytes_to_check) != 0) {
        uint32_t mismatch_offset = 0;
        while (mismatch_offset < bytes_to_check && verify_buffer[mismatch_offset] == expected[mismatch_offset]) {
            ++mismatch_offset;
        }

        const uint8_t expected_value = expected[mismatch_offset];
        const uint8_t actual_value = verify_buffer[mismatch_offset];
        ESP_LOGE(
            TAG,
            "Verify mismatch: start_byte=%lu bytes=%lu mismatch_offset=%lu expected=0x%02x actual=0x%02x",
            static_cast<unsigned long>(start_byte),
            static_cast<unsigned long>(bytes_to_check),
            static_cast<unsigned long>(mismatch_offset),
            expected_value,
            actual_value);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t FlashMonotonicCounter::rollover_()
{
    const int64_t new_base = signed_value_();

    esp_err_t result = save_rollover_state_to_nvs_(new_base, true);
    if (result != ESP_OK) {
        return result;
    }

    result = esp_partition_erase_range(partition_, 0, partition_->size);
    if (result != ESP_OK) {
        return result;
    }

    base_value_ = new_base;
    used_bits_ = 0;

    result = save_rollover_state_to_nvs_(base_value_, false);
    if (result != ESP_OK) {
        return result;
    }

    return ESP_OK;
}
