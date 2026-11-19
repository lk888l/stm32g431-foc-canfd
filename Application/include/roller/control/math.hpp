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
inline constexpr std::int32_t encoder_cpr = 16384;
inline float count_difference_turns(std::int64_t difference) noexcept {
    return static_cast<float>(difference) / encoder_cpr;
}

// The OD bounds position commands to +/-100000 revolutions, so the scaled
// REAL32 command fits a signed 32-bit count. Relative origins can be much larger.
inline std::int32_t command_counts(float turns) noexcept {
    return static_cast<std::int32_t>(std::lround(turns * encoder_cpr));
}

struct PositionProfile {
    std::int64_t counts{}, target{};
    float fraction{}; // Signed sub-count remainder of the 1 kHz trajectory.

    void hold(std::int64_t measured) noexcept {
        counts = target = measured;
        fraction = 0;
    }
    void set_target(std::int64_t measured, float turns, bool relative, bool single) noexcept {
        const auto requested = command_counts(turns);
        target = relative ? measured + requested : requested;
        if (single) {
            const auto offset = static_cast<std::uint64_t>(target - measured + encoder_cpr / 2);
            target = measured + static_cast<int>(offset & (encoder_cpr - 1)) - encoder_cpr / 2;
        }
    }
    float remaining() const noexcept {
        return count_difference_turns(target - counts) - fraction / encoder_cpr;
    }
    float error(std::int64_t measured) const noexcept {
        return count_difference_turns(counts - measured) + fraction / encoder_cpr;
    }
    void advance(float turns) noexcept {
        const float increment = fraction + turns * encoder_cpr;
        const auto whole = static_cast<std::int32_t>(increment);
        counts += whole;
        fraction = increment - static_cast<float>(whole);
    }
};
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
struct SinCos {
    float sine{}, cosine{1};
};
using Trigonometry = SinCos (*)(float) noexcept;
inline SinCos software_sincos(float angle) noexcept {
    return {std::sin(angle), std::cos(angle)};
}
inline float wrap(float radians) noexcept {
    if (!std::isfinite(radians)) {
        return 0.0F;
    }
    // Encoder electrical angles are bounded by 32 pole pairs. Avoid libm's
    // general remainder loop on that path; retain it for arbitrary host inputs.
    if (std::abs(radians) < 64.0F * tau) {
        radians -= static_cast<float>(static_cast<std::int32_t>(radians / tau)) * tau;
    } else {
        radians = std::fmod(radians, tau);
    }
    if (radians < 0) {
        radians += tau;
    }
    return radians < tau ? radians : 0.0F;
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
    if (!std::isfinite(v.d) || !std::isfinite(v.q) || !std::isfinite(limit) || !(limit > 0)) {
        return {};
    }
    const auto maximum = std::max(std::abs(v.d), std::abs(v.q));
    // Most current-loop outputs are unsaturated. The inscribed square needs
    // no square root; normalize the remaining cases to avoid float overflow.
    if (maximum <= limit * 0.70710678F) {
        return v;
    }
    const auto d = v.d / maximum, q = v.q / maximum;
    const auto scale = std::min(1.0F, (limit / maximum) / std::sqrt(d * d + q * q));
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
    // Full linear SVPWM: |Vdq| <= Vbus/sqrt(3), no artificial zero-vector margin.
    // Scale the whole vector if a caller requests outside the hexagon.
    const float span = std::max({phase.a, phase.b, phase.c}) - std::min({phase.a, phase.b, phase.c});
    const float scale = span > bus ? bus / span : 1.0F;
    const auto duty = [=](float x) { return std::clamp(0.5F + (x + common) * scale / bus, 0.0F, 1.0F); };
    return {duty(phase.a), duty(phase.b), duty(phase.c)};
}
class Pi {
  public:
    constexpr Pi(float kp, float ki) : kp_(kp), ki_(ki) {}
    void gains(float kp, float ki) noexcept {
        kp_ = kp;
        ki_ = ki;
    }
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
inline float velocity_ramp(float value, float target, float acceleration, float deceleration,
                           float dt) noexcept {
    if (value * target < 0) {
        const float stop_time = std::abs(value) / deceleration;
        if (stop_time >= dt) {
            return slew(value, 0, deceleration, dt);
        }
        return slew(0, target, acceleration, dt - stop_time);
    }
    return slew(value, target, std::abs(target) < std::abs(value) ? deceleration : acceleration, dt);
}
// Differentiate over a short time window before filtering. At 20 kHz a single
// 14-bit encoder count would otherwise become a 1.22 rev/s impulse. Keeping
// local angle increments also avoids loss of resolution after many full turns.
class VelocityWindow {
  public:
    float update(float delta, float dt) noexcept {
        while (count_ && (count_ == history_.size() || elapsed_ + dt > 0.00100001F)) {
            displacement_ -= history_[oldest_].delta;
            elapsed_ -= history_[oldest_].dt;
            oldest_ = (oldest_ + 1U) % history_.size();
            --count_;
        }
        if (!count_) {
            displacement_ = elapsed_ = 0;
        }
        history_[(oldest_ + count_) % history_.size()] = {delta, dt};
        ++count_;
        displacement_ += delta;
        elapsed_ += dt;
        // Rebase once per window: subtract/add rounding must not leave a
        // fictitious speed after a long run followed by standstill.
        if (++updates_ == history_.size()) {
            displacement_ = elapsed_ = 0;
            auto index = oldest_;
            for (std::size_t n = 0; n < count_; ++n) {
                displacement_ += history_[index].delta;
                elapsed_ += history_[index].dt;
                if (++index == history_.size()) {
                    index = 0;
                }
            }
            updates_ = 0;
        }
        return displacement_ / elapsed_;
    }
    void reset() noexcept {
        oldest_ = count_ = updates_ = 0;
        displacement_ = elapsed_ = 0;
    }

  private:
    struct Increment {
        float delta{}, dt{};
    };
    std::array<Increment, 20> history_{};
    std::size_t oldest_{}, count_{}, updates_{};
    float displacement_{}, elapsed_{};
};
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
        previous_code_ = angle_code(angle);
        velocity_ = 0;
        velocity_window_.reset();
        valid_ = true;
    }
    bool update(float angle, float dt, std::uint16_t code = encoder_cpr) noexcept {
        if (!std::isfinite(angle) || angle < 0 || angle >= tau || !(dt > 0)) {
            return false;
        }
        if (!valid_) {
            previous_ = angle;
            previous_code_ = code < encoder_cpr ? code : angle_code(angle);
            valid_ = true;
            return true;
        }
        const auto raw = angle - previous_;
        const auto delta = raw > pi ? raw - tau : (raw < -pi ? raw + tau : raw);
        // Physical limit 100 rev/s plus 3 encoder LSBs; reject a corrupt jump.
        if (std::abs(delta) > 100.0F * tau * dt + 3.0F * tau / 16384.0F) {
            return false;
        }
        if (!advance_turn(raw)) {
            return false;
        }
        previous_ = angle;
        previous_code_ = code < encoder_cpr ? code : angle_code(angle);
        // 1 ms differentiation window + 0.25 ms low-pass: approximately
        // 0.75 ms low-frequency delay, with less quantization noise than a
        // one-sample derivative. Missing frames retain their actual duration.
        velocity_ += (velocity_window_.update(delta, dt) - velocity_) * (dt / (0.00025F + dt));
        if (std::abs(velocity_) < 1.0e-6F) {
            velocity_ = 0;
        }
        return true;
    }
    [[nodiscard]] float position() const noexcept {
        // Preserve each absolute encoder sample instead of accumulating tiny
        // float increments, which stop changing after enough full revolutions.
        return static_cast<float>(turns_) * tau + previous_;
    }
    [[nodiscard]] std::int64_t counts() const noexcept {
        return static_cast<std::int64_t>(turns_) * encoder_cpr + previous_code_;
    }
    [[nodiscard]] float position_turns(float predicted_radians) const noexcept {
        return static_cast<float>(turns_) + (previous_ + predicted_radians) / tau;
    }
    [[nodiscard]] float velocity() const noexcept {
        return velocity_;
    }

  private:
    static std::uint16_t angle_code(float angle) noexcept {
        return static_cast<std::uint16_t>(angle * (encoder_cpr / tau) + 0.5F);
    }
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
    VelocityWindow velocity_window_{};
    std::int32_t turns_{};
    std::uint16_t previous_code_{};
    bool valid_{};
};
} // namespace roller::control
