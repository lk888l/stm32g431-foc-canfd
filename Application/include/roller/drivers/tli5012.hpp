#pragma once
#include "roller/control/math.hpp"
#include <array>
#include <cstdint>
#include <expected>
#include <span>

namespace roller::drivers {
enum class EncoderError { timeout, crc, status, disconnected };
inline std::uint8_t crc8(std::span<const std::uint8_t> bytes) noexcept {
    std::uint8_t crc = 0xFF;
    for (auto byte : bytes) {
        crc ^= byte;
        for (int bit = 0; bit < 8; ++bit) {
            crc = static_cast<std::uint8_t>((crc & 0x80) ? (crc << 1) ^ 0x1D : crc << 1);
        }
    }
    return static_cast<std::uint8_t>(~crc);
}
inline std::expected<float, EncoderError> decode_angle(std::uint16_t angle, std::uint16_t safety) noexcept {
    constexpr std::uint16_t command = 0x8021;
    if ((angle == 0xFFFF && safety == 0xFFFF) || (angle == 0 && safety == 0)) {
        return std::unexpected(EncoderError::disconnected);
    }
    const std::array<std::uint8_t, 4> bytes{command >> 8, command & 0xFF,
                                            static_cast<std::uint8_t>(angle >> 8),
                                            static_cast<std::uint8_t>(angle)};
    if (crc8(bytes) != (safety & 0xFF)) {
        return std::unexpected(EncoderError::crc);
    }
    if ((safety & 0xF000) != 0xF000) {
        return std::unexpected(EncoderError::status);
    }
    return static_cast<float>(angle & 0x7FFF) * control::tau / 32768.0F;
}
} // namespace roller::drivers
