#pragma once
#include <cstdint>
#include <optional>

namespace roller::platform {
// PCLK1 / FDCAN_CLOCK_DIV1 = 160 MHz. Values are HAL time quanta, not registers.
struct CanBitTiming {
    std::uint32_t prescaler, segment1, segment2, sjw;
};
constexpr bool can_tdc_enabled(const CanBitTiming& timing) noexcept {
    // Bosch M_CAN User's Manual v3.3.1, section 2.3.4: when TDC is enabled,
    // DBRP is restricted to 0..1. HAL's actual divisor is DBRP + 1.
    // https://www.bosch-semiconductors.com/media/ip_modules/pdf_2/m_can/mcan_users_manual_v331.pdf
    return timing.prescaler >= 1 && timing.prescaler <= 2;
}
constexpr std::optional<CanBitTiming> can_nominal_timing(std::uint32_t bitrate) noexcept {
    switch (bitrate) {
    case 125000: return CanBitTiming{40, 25, 6, 4};
    case 250000: return CanBitTiming{20, 25, 6, 4};
    case 500000: return CanBitTiming{10, 25, 6, 4};
    case 1000000: return CanBitTiming{5, 25, 6, 4};
    default: return std::nullopt;
    }
}
constexpr std::optional<CanBitTiming> can_data_timing(std::uint32_t bitrate) noexcept {
    // 75% sample point, including the synchronization quantum.
    switch (bitrate) {
    case 1000000: return CanBitTiming{5, 23, 8, 4};
    case 2000000: return CanBitTiming{2, 29, 10, 4};
    case 4000000: return CanBitTiming{1, 29, 10, 4};
    case 5000000: return CanBitTiming{1, 23, 8, 4};
    case 8000000: return CanBitTiming{1, 14, 5, 4};
    default: return std::nullopt;
    }
}
} // namespace roller::platform
