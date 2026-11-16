#pragma once
#include "roller/control/math.hpp"

namespace roller::control {
// Motor 4015 is delta connected. These are equivalent star line-current
// parameters: Rline/2 = 1.8 ohm, Ldelta/3 = 0.433 mH (datasheet estimate).
inline constexpr float motor_resistance = 1.8F;
inline constexpr float motor_inductance = 0.0013F / 3.0F;
inline constexpr float current_bandwidth = 500.0F; // Hz; tune under load after commissioning.

class CurrentObserver {
  public:
    Abc update(Abc measured, std::uint8_t valid, AlphaBeta voltage, AlphaBeta emf, float dt) noexcept {
        auto predicted = clarke(current_);
        predicted.alpha +=
            dt / motor_inductance * (voltage.alpha - emf.alpha - motor_resistance * predicted.alpha);
        predicted.beta +=
            dt / motor_inductance * (voltage.beta - emf.beta - motor_resistance * predicted.beta);
        Abc next{predicted.alpha, -0.5F * predicted.alpha + 0.5F * sqrt3 * predicted.beta,
                 -0.5F * predicted.alpha - 0.5F * sqrt3 * predicted.beta};
        if (valid & 1U) {
            next.a = measured.a;
        }
        if (valid & 2U) {
            next.b = measured.b;
        }
        if (valid & 4U) {
            next.c = measured.c;
        }
        // Two measured currents fully determine the third. With one, project
        // the prediction onto that measurement and ia+ib+ic=0.
        if (valid == 3) {
            next.c = -next.a - next.b;
        } else if (valid == 5) {
            next.b = -next.a - next.c;
        } else if (valid == 6) {
            next.a = -next.b - next.c;
        } else {
            const float sum = next.a + next.b + next.c;
            const float correction = sum / (valid == 7 || valid == 0 ? 3.0F : 2.0F);
            if (valid == 7 || !(valid & 1U)) {
                next.a -= correction;
            }
            if (valid == 7 || !(valid & 2U)) {
                next.b -= correction;
            }
            if (valid == 7 || !(valid & 4U)) {
                next.c -= correction;
            }
        }
        current_ = next;
        return next;
    }
    void reset() noexcept {
        current_ = {};
    }

  private:
    Abc current_{};
};
} // namespace roller::control
