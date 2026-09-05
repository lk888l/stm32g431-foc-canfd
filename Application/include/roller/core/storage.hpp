#pragma once
#include <cstddef>
#include <cstdint>
#include <span>

namespace roller {
class ParameterStore {
  public:
    [[nodiscard]] virtual bool read(std::span<std::byte> data) noexcept = 0;
    [[nodiscard]] virtual bool write(std::span<const std::byte> data) noexcept = 0;

  protected:
    ~ParameterStore() = default;
};
inline std::uint32_t crc32(std::span<const std::byte> data, std::uint32_t previous = 0) noexcept {
    std::uint32_t crc = ~previous;
    for (auto byte : data) {
        crc ^= std::to_integer<std::uint8_t>(byte);
        for (int i = 0; i < 8; ++i) {
            crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320U : 0);
        }
    }
    return ~crc;
}
} // namespace roller
