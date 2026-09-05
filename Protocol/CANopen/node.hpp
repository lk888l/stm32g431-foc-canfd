#pragma once
#include "CANopen.h"
#include "OD.h"
#include "roller/core/channels.hpp"
#include "roller/core/module.hpp"
#include "roller/core/storage.hpp"
#include <array>

namespace roller::protocol {
struct SavedSettings {
    std::uint32_t version{1};
    drive::MotorConfig motor{};
    RollerOD dictionary{};
};
class Node final : public Module {
  public:
    Node(RollerCanTransport& transport, Channels& channels, ParameterStore& store) noexcept
        : transport_(transport), channels_(channels), store_(store) {}
    [[nodiscard]] std::string_view name() const noexcept override {
        return "canopen";
    }
    [[nodiscard]] drive::MotorConfig load_settings() noexcept;
    void receive(const RollerCanFrame& frame) noexcept;
    void process(std::uint32_t elapsed_us) noexcept;
    [[nodiscard]] CO_t* stack() noexcept {
        return co_;
    }
    [[nodiscard]] bool reset_requested() const noexcept {
        return reset_requested_;
    }

  private:
    Result on_start() noexcept override;
    Result on_stop() noexcept override;
    static ODR_t write(OD_stream_t*, const void*, OD_size_t, OD_size_t*);
    ODR_t on_write(OD_stream_t*, const void*, OD_size_t, OD_size_t*);
    void publish() noexcept;
    void report() noexcept;
    bool save(bool defaults) noexcept;
    [[nodiscard]] bool compressed_mapping_valid() const noexcept;
    RollerCanTransport& transport_;
    Channels& channels_;
    ParameterStore& store_;
    CO_t* co_{};
    std::array<OD_extension_t, 32> extensions_{};
    drive::Command command_{};
    drive::Telemetry telemetry_{};
    drive::Fault reported_fault_{};
    std::uint32_t command_idle_us_{}, sequence_{}, telemetry_idle_us_{};
    bool reset_requested_{}, seen_motion_command_{}, processing_rpdo_{}, invalid_pdo_{};
    bool seen_compressed_frame_{};
    std::uint8_t rpdo_half_mask_{};
};
} // namespace roller::protocol
