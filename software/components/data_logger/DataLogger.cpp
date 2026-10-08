#include "DataLogger.h"

#include <algorithm>
#include <cstring>

#include "BoardConfig.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr uint32_t kRecordMagic = 0x314D4C54U;
constexpr uint16_t kDataStartAddress = 0;
constexpr uint16_t kRecordSize = 64;
constexpr uint32_t kRecordCapacity =
    DEMO_AT24C256_CAPACITY_BYTES / kRecordSize;
constexpr uint32_t kWriteCycleDelayMs = 10;

void putU32(uint8_t *data, uint32_t value)
{
    data[0] = static_cast<uint8_t>(value);
    data[1] = static_cast<uint8_t>(value >> 8);
    data[2] = static_cast<uint8_t>(value >> 16);
    data[3] = static_cast<uint8_t>(value >> 24);
}

uint32_t getU32(const uint8_t *data)
{
    return static_cast<uint32_t>(data[0]) |
           (static_cast<uint32_t>(data[1]) << 8) |
           (static_cast<uint32_t>(data[2]) << 16) |
           (static_cast<uint32_t>(data[3]) << 24);
}

void putU64(uint8_t *data, uint64_t value)
{
    for (uint8_t i = 0; i < 8; ++i) {
        data[i] = static_cast<uint8_t>(value >> (i * 8));
    }
}

uint64_t getU64(const uint8_t *data)
{
    uint64_t value = 0;
    for (uint8_t i = 0; i < 8; ++i) {
        value |= static_cast<uint64_t>(data[i]) << (i * 8);
    }
    return value;
}

void putFloat(uint8_t *data, float value)
{
    uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    putU32(data, bits);
}

float getFloat(const uint8_t *data)
{
    const uint32_t bits = getU32(data);
    float value = 0.0F;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

void putDouble(uint8_t *data, double value)
{
    uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    putU64(data, bits);
}

double getDouble(const uint8_t *data)
{
    const uint64_t bits = getU64(data);
    double value = 0.0;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

uint32_t crc32(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xFFFFFFFFU;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ ((crc & 1U) ? 0xEDB88320U : 0U);
        }
    }
    return ~crc;
}

bool isValidRecord(const uint8_t *data)
{
    return getU32(data) == kRecordMagic &&
           getU32(data + 60) == crc32(data, 60);
}

uint16_t recordAddress(uint32_t index)
{
    return static_cast<uint16_t>(kDataStartAddress + (index * kRecordSize));
}

} // namespace

esp_err_t DataLogger::initialize()
{
    constexpr size_t kScanChunkSize = 256;
    constexpr size_t kDataSize = kRecordCapacity * kRecordSize;
    uint8_t chunk[kScanChunkSize] = {};
    bool foundRecord = false;
    uint32_t newestSequence = 0;
    uint32_t newestIndex = 0;
    uint32_t validRecordCount = 0;

    for (size_t offset = 0; offset < kDataSize; offset += sizeof(chunk)) {
        const size_t chunkLength = std::min(sizeof(chunk), kDataSize - offset);
        esp_err_t err = readBytes(
            static_cast<uint16_t>(offset),
            chunk,
            chunkLength);
        if (err != ESP_OK) {
            return err;
        }

        for (size_t recordOffset = 0; recordOffset < chunkLength;
             recordOffset += kRecordSize) {
            const uint8_t *record = chunk + recordOffset;
            if (!isValidRecord(record)) {
                continue;
            }

            ++validRecordCount;
            const uint32_t candidateSequence = getU32(record + 4);
            if (!foundRecord ||
                static_cast<int32_t>(candidateSequence - newestSequence) > 0) {
                foundRecord = true;
                newestSequence = candidateSequence;
                newestIndex = static_cast<uint32_t>(
                    (offset + recordOffset) / kRecordSize);
            }
        }
    }

    if (foundRecord) {
        sequence_ = newestSequence;
        nextIndex_ = (newestIndex + 1U) % kRecordCapacity;
        recordCount_ = std::min(validRecordCount, kRecordCapacity);
    } else {
        sequence_ = 0;
        nextIndex_ = 0;
        recordCount_ = 0;
    }
    initialized_ = true;
    return ESP_OK;
}

esp_err_t DataLogger::append(const TelemetryRecord &record)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }
    if (record.throttlePercent > 100U) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data[kRecordSize] = {};
    putU32(data, kRecordMagic);
    const uint32_t nextSequence = sequence_ + 1U;
    putU32(data + 4, nextSequence);
    putU32(data + 8, record.uptimeMilliseconds);
    putFloat(data + 12, record.tiltDegrees);
    putU32(data + 16, record.throttlePercent);
    putFloat(data + 20, record.busVoltageVolts);
    putFloat(data + 24, record.currentMilliamps);
    data[28] = record.gps.valid ? 1U : 0U;
    putDouble(data + 32, record.gps.latitudeDegrees);
    putDouble(data + 40, record.gps.longitudeDegrees);
    putFloat(data + 48, record.gps.speedMetersPerSecond);
    putU32(data + 52, record.gps.satellitesInView);
    putU32(data + 60, crc32(data, 60));

    esp_err_t err = writeBytes(recordAddress(nextIndex_), data, sizeof(data));
    if (err != ESP_OK) {
        return err;
    }

    sequence_ = nextSequence;
    nextIndex_ = (nextIndex_ + 1U) % kRecordCapacity;
    if (recordCount_ < kRecordCapacity) {
        ++recordCount_;
    }
    return ESP_OK;
}

esp_err_t DataLogger::readLatest(TelemetryRecord *record)
{
    if (record == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }
    if (recordCount_ == 0U) {
        return ESP_ERR_NOT_FOUND;
    }

    const uint32_t latestIndex =
        (nextIndex_ + kRecordCapacity - 1U) % kRecordCapacity;
    uint8_t data[kRecordSize] = {};
    esp_err_t err = readBytes(recordAddress(latestIndex), data, sizeof(data));
    if (err != ESP_OK) {
        return err;
    }
    if (!isValidRecord(data) || getU32(data + 4) != sequence_) {
        return ESP_ERR_INVALID_CRC;
    }

    record->uptimeMilliseconds = getU32(data + 8);
    record->tiltDegrees = getFloat(data + 12);
    record->throttlePercent = getU32(data + 16);
    record->busVoltageVolts = getFloat(data + 20);
    record->currentMilliamps = getFloat(data + 24);
    record->gps.valid = data[28] != 0U;
    record->gps.latitudeDegrees = getDouble(data + 32);
    record->gps.longitudeDegrees = getDouble(data + 40);
    record->gps.speedMetersPerSecond = getFloat(data + 48);
    record->gps.satellitesInView = getU32(data + 52);
    return ESP_OK;
}

esp_err_t DataLogger::readBytes(uint16_t address, uint8_t *data, size_t length)
{
    if (data == nullptr || length == 0U ||
        (static_cast<size_t>(address) + length) > DEMO_AT24C256_CAPACITY_BYTES) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t addressBytes[] = {
        static_cast<uint8_t>(address >> 8),
        static_cast<uint8_t>(address & 0xFFU),
    };
    return i2c_master_write_read_device(
        I2C_NUM_0,
        DEMO_AT24C256_ADDRESS,
        addressBytes,
        sizeof(addressBytes),
        data,
        length,
        pdMS_TO_TICKS(200));
}

esp_err_t DataLogger::writeBytes(
    uint16_t address,
    const uint8_t *data,
    size_t length)
{
    if (data == nullptr || length == 0U ||
        (static_cast<size_t>(address) + length) > DEMO_AT24C256_CAPACITY_BYTES) {
        return ESP_ERR_INVALID_ARG;
    }

    while (length > 0U) {
        const size_t pageOffset = address % DEMO_AT24C256_PAGE_SIZE_BYTES;
        const size_t pageSpace = DEMO_AT24C256_PAGE_SIZE_BYTES - pageOffset;
        const size_t chunkLength = std::min(length, pageSpace);

        uint8_t transaction[DEMO_AT24C256_PAGE_SIZE_BYTES + 2U] = {};
        transaction[0] = static_cast<uint8_t>(address >> 8);
        transaction[1] = static_cast<uint8_t>(address & 0xFFU);
        std::memcpy(transaction + 2, data, chunkLength);

        esp_err_t err = i2c_master_write_to_device(
            I2C_NUM_0,
            DEMO_AT24C256_ADDRESS,
            transaction,
            chunkLength + 2U,
            pdMS_TO_TICKS(200));
        if (err != ESP_OK) {
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(kWriteCycleDelayMs));

        address = static_cast<uint16_t>(address + chunkLength);
        data += chunkLength;
        length -= chunkLength;
    }
    return ESP_OK;
}
