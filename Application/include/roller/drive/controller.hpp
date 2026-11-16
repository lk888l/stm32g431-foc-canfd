#pragma once
#include "roller/control/current_observer.hpp"
#include "roller/control/math.hpp"
#include "roller/drive/state_machine.hpp"
#include <cstdint>
#include <optional>

namespace roller::drive {
enum class Mode : std::int8_t {
    single_turn_position = -1, // Manufacturer mode: nearest equivalent mechanical angle.
    none = 0,
    multi_turn_position = 1,
    position = multi_turn_position, // Preserve the original mode 1 API and wire format.
    velocity = 3,
    torque = 4,
    mit = 5
};
constexpr bool is_position_mode(Mode mode) noexcept {
    return mode == Mode::multi_turn_position || mode == Mode::single_turn_position;
}
enum class Fault : std::uint16_t {
    none = 0,
    overcurrent = 0x2310,
    overvoltage = 0x3210,
    undervoltage = 0x3220,
    temperature = 0x4210,
    driver = 0x5000,
    timing = 0x5300,
    configuration = 0x6320,
    encoder = 0x7340,
    communication = 0x8130,
    calibration = 0x8312,
    following = 0x8500
};
enum Warning : std::uint32_t {
    encoder_prediction = 1U,
    current_prediction = 2U,
    communication_warning = 4U,
    following_warning = 8U,
    voltage_warning = 16U,
    temperature_warning = 32U,
    current_warning = 64U,
    math_warning = 128U
};
enum class Calibration : std::uint8_t { required, align, sweep, settle, complete, failed };
struct MotorConfig {
    float electrical_offset{};
    float torque_constant{0.23F}; // 4015 datasheet; line-current dq convention still needs measurement.
    float current_limit{1.4F};
    std::uint16_t pole_pairs{11};
    std::int8_t direction{1};
    std::uint8_t calibrated{};
};
struct Command {
    std::uint32_t sequence{}, calibration_request{};
    std::uint16_t controlword{}, max_torque{1000};
    std::int16_t torque{};
    Mode mode{Mode::none};
    bool operational{}, network_ok{true}, calibration_permitted{true};
    // Revolutions and revolutions/second. Mode 1 uses the continuous encoder
    // position; mode -1 wraps absolute/relative requests to the nearest turn.
    float position{}, velocity{};
    float profile_velocity{2}, acceleration{2}, deceleration{2}, quick_deceleration{4};
    float mit_position{}, mit_velocity{}, mit_torque{}, mit_kp{}, mit_kd{};
    float following_window{0.25F};
    std::uint16_t following_ms{1000};
};
// Validation belongs to the producer task. Only this immutable value may cross
// the task/ISR boundary, so the current loop does not repeat all OD range checks.
class ValidatedCommand {
  public:
    ValidatedCommand() noexcept = default;
    static std::optional<ValidatedCommand> make(const Command& command) noexcept;
    const Command& get() const noexcept {
        return value_;
    }

  private:
    explicit ValidatedCommand(const Command& command) noexcept : value_(command) {}
    Command value_{};
};
struct Sample {
    control::Abc current{};
    float bus_voltage{}, angle{}, mcu_temperature{25};
    bool encoder_ok{}, driver_ok{}, adc_ok{true}, current_calibrated{};
    std::uint8_t current_mask{7};
    float current_angle_delay{}, voltage_angle_delay{};
    control::AlphaBeta applied_voltage{};
    bool applied_voltage_valid{};
};
struct Output {
    control::Abc duty{0.5F, 0.5F, 0.5F};
    bool enable{};
};
struct Telemetry {
    std::uint32_t samples{}, command_sequence{}, warnings{}, encoder_missed{}, current_predicted{};
    float position{}, velocity{}, bus_voltage{}, mcu_temperature{}, iq{}, id{};
    control::Abc phase_current{}, phase_voltage{};
    std::uint16_t statusword{0x0240};
    Fault fault{};
    Mode mode{};
    Calibration calibration{Calibration::required};
    MotorConfig config{};
};
class Controller {
  public:
    static constexpr float dt = 1.0F / 20000.0F;
    explicit Controller(MotorConfig config = {},
                        control::Trigonometry trig = control::software_sincos) noexcept
        : config_(config), trig_(trig ? trig : control::software_sincos) {
        telemetry_.config = config_;
        calibration_ = config.calibrated ? Calibration::complete : Calibration::required;
        if (!valid_config(config_)) {
            fault_ = Fault::configuration;
        }
    }
    void command(const Command& value) noexcept;
    void command(const ValidatedCommand& value) noexcept;
    [[nodiscard]] Output step(const Sample& sample) noexcept;
    [[nodiscard]] const Telemetry& telemetry() const noexcept {
        return telemetry_;
    }
    void trip(Fault fault) noexcept {
        if (fault_ == Fault::none) {
            fault_ = fault;
        }
    }
    static bool valid_config(const MotorConfig& config) noexcept;
    static bool valid_command(const Command& command) noexcept;

  private:
    Output calibration_step(const Sample& sample) noexcept;
    void outer_step(const Sample& sample) noexcept;
    void reset_loops() noexcept;
    MotorConfig config_;
    control::Trigonometry trig_;
    Command command_{};
    Telemetry telemetry_{};
    StateMachine state_{};
    control::AngleTracker tracker_{};
    control::Pi d_pi_{control::motor_inductance * control::tau * control::current_bandwidth,
                      control::motor_resistance * control::tau * control::current_bandwidth},
        q_pi_{control::motor_inductance * control::tau * control::current_bandwidth,
              control::motor_resistance * control::tau * control::current_bandwidth},
        velocity_pi_{0.06F, 0.15F};
    control::CurrentObserver observer_{};
    Output previous_output_{};
    std::uint32_t missed_angles_{}, calibration_segments_{}, offset_count_{};
    float last_angle_{}, calibration_segment_start_{}, offset_sine_{}, offset_cosine_{};
    std::int8_t calibration_direction_{};
    bool angle_initialized_{};
    Fault fault_{};
    Calibration calibration_{Calibration::required};
    std::uint32_t ticks_{}, command_age_{}, calibration_tick_{}, calibration_seen_{};
    std::uint16_t previous_cw_{}, following_ticks_{};
    float iq_target_{}, velocity_setpoint_{}, position_setpoint_{}, target_position_{};
    float calibration_start_{};
    bool setpoint_ack_{}, initialized_{};
};
} // namespace roller::drive
