#pragma once
#include "roller/control/math.hpp"
#include <cstdint>
#include <optional>

namespace roller::drivers {
// MT6701 v1.9, section 7.8.2: 14 angle + 4 status + 6 CRC bits, MSB first.
// CRC-6: x^6+x+1, init 0, no reflection, no final XOR.
constexpr std::uint8_t mt6701_crc(std::uint32_t payload) noexcept {
    std::uint8_t crc = 0;
    for (int bit = 17; bit >= 0; --bit) {
        const auto feedback = ((payload >> bit) ^ (crc >> 5)) & 1U;
        crc = static_cast<std::uint8_t>(((crc << 1) ^ (feedback ? 0x03U : 0U)) & 0x3FU);
    }
    return crc;
}
struct EncoderReading {
    float angle{};
    std::uint16_t counts{};
    std::uint8_t status{}; // field strength, PUSH, overspeed; diagnostic, not a trip.
};
inline std::optional<EncoderReading> decode_mt6701(std::uint32_t frame) noexcept {
    if (frame > 0xFFFFFFU || frame == 0xFFFFFFU || mt6701_crc(frame >> 6) != (frame & 0x3FU)) {
        return std::nullopt;
    }
    const auto counts = static_cast<std::uint16_t>(frame >> 10);
    return EncoderReading{static_cast<float>(counts) * control::tau / 16384.0F, counts,
                          static_cast<std::uint8_t>((frame >> 6) & 15U)};
}
} // namespace roller::drivers
