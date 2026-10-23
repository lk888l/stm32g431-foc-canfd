#pragma once
#include <array>
#include <cstdint>

namespace roller::platform {
// Sole owner: Board's control ISR, or startup/maintenance with sampling stopped.
bool initialize_pwm_dma() noexcept;
// Transfer neutral duty to the preloads before the timer is started.
bool start_pwm_dma() noexcept;
void stop_pwm_dma() noexcept;
// Transfer CCR1/CCR2/CCR3 (phase C/B/A), with a bounded wait for completion.
// Reject an unfinished previous burst before replacing its persistent source.
bool write_pwm_dma(std::array<std::uint16_t, 3> cba) noexcept;
// Transfer errors/timeouts remain visible until stop/start.
bool pwm_dma_ok() noexcept;
} // namespace roller::platform
