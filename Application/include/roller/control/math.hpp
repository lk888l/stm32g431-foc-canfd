#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>

namespace roller::control {
inline constexpr float pi = std::numbers::pi_v<float>;
inline constexpr float tau = 2.0F * pi;
inline constexpr float sqrt3 = 1.7320508075688772F;
struct Abc {
    float a{}, b{}, c{};
};
struct AlphaBeta {
    float alpha{}, beta{};
};
struct Dq {
    float d{}, q{};
};
inline float wrap(float radians) noexcept {
    if (!std::isfinite(radians)) {
        return 0.0F;
    }
    radians = std::fmod(radians, tau);
    return radians < 0.0F ? radians + tau : radians;
}
inline float wrap_signed(float radians) noexcept {
    return wrap(radians + pi) - pi;
}
inline AlphaBeta clarke(Abc i) noexcept {
    return {(2.0F * i.a - i.b - i.c) / 3.0F, (i.b - i.c) / sqrt3};
}
inline Dq park(AlphaBeta i, float sine, float cosine) noexcept {
    return {i.alpha * cosine + i.beta * sine, -i.alpha * sine + i.beta * cosine};
}
inline AlphaBeta inverse_park(Dq v, float sine, float cosine) noexcept {
    return {v.d * cosine - v.q * sine, v.d * sine + v.q * cosine};
}
inline Dq limit_vector(Dq v, float limit) noexcept {
    const auto norm = std::hypot(v.d, v.q);
    if (!std::isfinite(norm) || !(limit > 0.0F)) {
        return {};
    }
    const auto scale = norm > limit ? limit / norm : 1.0F;
    return {v.d * scale, v.q * scale};
}
inline Abc svpwm(AlphaBeta v, float bus) noexcept {
    if (!(bus > 1.0F) || !std::isfinite(bus) || !std::isfinite(v.alpha) || !std::isfinite(v.beta)) {
        return {0.5F, 0.5F, 0.5F};
    }
    const Abc phase{v.alpha, -0.5F * v.alpha + 0.5F * sqrt3 * v.beta,
                    -0.5F * v.alpha - 0.5F * sqrt3 * v.beta};
    const auto common =
        -0.5F * (std::max({phase.a, phase.b, phase.c}) + std::min({phase.a, phase.b, phase.c}));
    // 10% zero-vector margin guarantees a low-side sampling window at ARR.
    const auto duty = [=](float x) { return std::clamp(0.5F + (x + common) / bus, 0.10F, 0.90F); };
    return {duty(phase.a), duty(phase.b), duty(phase.c)};
}
class Pi {
  public:
    constexpr Pi(float kp, float ki) : kp_(kp), ki_(ki) {}
    float step(float error, float dt, float limit) noexcept {
        if (!std::isfinite(error) || !(dt > 0.0F) || !(limit >= 0.0F)) {
            reset();
            return 0;
        }
        const auto candidate = integral_ + ki_ * error * dt;
        const auto raw = kp_ * error + candidate;
        const auto output = std::clamp(raw, -limit, limit);
        // Conditional integration lets a saturated controller unwind immediately.
        if (raw == output || error * raw < 0.0F) {
            integral_ = std::clamp(candidate, -limit, limit);
        }
        return output;
    }
    void track(float applied, float requested) noexcept {
        integral_ += applied - requested;
    }
    void reset() noexcept {
        integral_ = 0;
    }

  private:
    float kp_, ki_, integral_{};
};
inline float slew(float value, float target, float rate, float dt) noexcept {
    return value + std::clamp(target - value, -rate * dt, rate * dt);
}
class AngleTracker {
  public:
    void resynchronize(float angle) noexcept {
        if (!std::isfinite(angle) || angle < 0 || angle >= tau) {
            return;
        }
        // Explicit fault reset accepts a new absolute angle; missed full turns
        // cannot be reconstructed from a single-turn encoder.
        if (valid_ && !advance_turn(angle - previous_)) {
            turns_ = 0;
        }
        previous_ = angle;
        velocity_ = 0;
        valid_ = true;
    }
    bool update(float angle, float dt) noexcept {
        if (!std::isfinite(angle) || angle < 0 || angle >= tau || !(dt > 0)) {
            return false;
        }
        if (!valid_) {
            previous_ = angle;
            valid_ = true;
            return true;
        }
        const auto raw = angle - previous_;
        const auto delta = raw > pi ? raw - tau : (raw < -pi ? raw + tau : raw);
        // Physical limit 100 rev/s plus 3 encoder LSBs; reject a corrupt jump.
        if (std::abs(delta) > 100.0F * tau * dt + 3.0F * tau / 32768.0F) {
            return false;
        }
        if (!advance_turn(raw)) {
            return false;
        }
        previous_ = angle;
        velocity_ += (delta / dt - velocity_) * std::min(1.0F, dt * 200.0F);
        return true;
    }
    [[nodiscard]] float position() const noexcept {
        // Preserve each absolute encoder sample instead of accumulating tiny
        // float increments, which stop changing after enough full revolutions.
        return static_cast<float>(turns_) * tau + previous_;
    }
    [[nodiscard]] float velocity() const noexcept {
        return velocity_;
    }

  private:
    bool advance_turn(float raw) noexcept {
        if (raw < -pi) {
            if (turns_ == std::numeric_limits<std::int32_t>::max()) {
                return false;
            }
            ++turns_;
        } else if (raw > pi) {
            if (turns_ == std::numeric_limits<std::int32_t>::min()) {
                return false;
            }
            --turns_;
        }
        return true;
    }
    float previous_{}, velocity_{};
    std::int32_t turns_{};
    bool valid_{};
};
} // namespace roller::control
