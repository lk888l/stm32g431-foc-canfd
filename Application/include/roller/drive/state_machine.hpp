#pragma once
#include <cstdint>

namespace roller::drive {
enum class State : std::uint8_t { disabled, ready, switched_on, enabled, quick_stop, fault };
// CiA 402 command masks, not an enumeration of complete controlword values.
class StateMachine {
  public:
    void trip() noexcept {
        state_ = State::fault;
    }
    State step(std::uint16_t cw, bool healthy, bool permit, bool stopped) noexcept {
        const bool reset_edge = (cw & 0x80) && !(previous_ & 0x80);
        previous_ = cw;
        if (!healthy) {
            state_ = State::fault;
        }
        if (state_ == State::fault) {
            if (healthy && reset_edge) {
                state_ = State::disabled;
            }
            return state_;
        }
        if (!permit || (cw & 0x02) == 0) {
            state_ = State::disabled;
            return state_;
        }
        const bool shutdown = (cw & 0x87) == 0x06;
        const bool switch_on = (cw & 0x8F) == 0x07;
        const bool enable = (cw & 0x8F) == 0x0F;
        const bool quick = (cw & 0x86) == 0x02;
        switch (state_) {
        case State::disabled:
            if (shutdown) {
                state_ = State::ready;
            }
            break;
        case State::ready:
            if (switch_on || enable) {
                state_ = State::switched_on;
            }
            break;
        case State::switched_on:
            if (shutdown) {
                state_ = State::ready;
            } else if (enable) {
                state_ = State::enabled;
            } else if (quick) {
                state_ = State::disabled;
            }
            break;
        case State::enabled:
            if (quick) {
                state_ = State::quick_stop;
            } else if (shutdown) {
                state_ = State::ready;
            } else if (switch_on) {
                state_ = State::switched_on;
            }
            break;
        case State::quick_stop:
            if (stopped) {
                state_ = State::disabled;
            }
            break;
        case State::fault:
            break;
        }
        return state_;
    }
    [[nodiscard]] State state() const noexcept {
        return state_;
    }
    [[nodiscard]] std::uint16_t status() const noexcept {
        switch (state_) {
        case State::disabled:
            return 0x0240;
        case State::ready:
            return 0x0221;
        case State::switched_on:
            return 0x0233;
        case State::enabled:
            return 0x0237;
        case State::quick_stop:
            return 0x0217;
        case State::fault:
            return 0x0208;
        }
        return 0x0208;
    }

  private:
    State state_{State::disabled};
    std::uint16_t previous_{};
};
} // namespace roller::drive
