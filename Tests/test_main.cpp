#include "node.hpp"
#include "roller/control/math.hpp"
#include "roller/core/module.hpp"
#include "roller/core/parameter_journal.hpp"
#include "roller/core/progress_watchdog.hpp"
#include "roller/core/spsc_queue.hpp"
#include "roller/drive/controller.hpp"
#include "roller/drive/state_machine.hpp"
#include "roller/drivers/rollercan_sensing.hpp"
#include "roller/drivers/tli5012.hpp"
#include <bit>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <thread>
#include <vector>

static unsigned checks = 0;
#define CHECK(expr)                                                                                          \
    do {                                                                                                     \
        ++checks;                                                                                            \
        if (!(expr)) {                                                                                       \
            std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr);                             \
            std::exit(1);                                                                                    \
        }                                                                                                    \
    } while (0)
using namespace roller;
using namespace roller::control;

static void math_tests() {
    CHECK(std::abs(drivers::bus_voltage(2238, 3.304F) - 11.655F) < 0.015F);
    for (int i = -40000; i <= 40000; ++i) {
        const auto input = static_cast<float>(i) * 0.01F;
        auto expected = std::fmod(input, tau);
        if (expected < 0) {
            expected += tau;
        }
        CHECK(std::abs(wrap_signed(wrap(input) - expected)) < 3e-5F);
    }
    for (int x = -10; x <= 10; ++x) {
        for (int y = -10; y <= 10; ++y) {
            const Dq input{static_cast<float>(x), static_cast<float>(y)};
            const auto limited = limit_vector(input, 3);
            CHECK(std::hypot(limited.d, limited.q) <= 3.00001F);
            CHECK(std::abs(limited.d * input.q - limited.q * input.d) < 1e-5F);
        }
    }
    const auto huge = limit_vector({1e30F, -1e30F}, 3);
    CHECK(std::abs(std::hypot(huge.d, huge.q) - 3) < 1e-5F);
    CHECK(limit_vector({0, 0}, 3).d == 0);
    CHECK(limit_vector({0, std::numeric_limits<float>::quiet_NaN()}, 3).q == 0);
    for (int n = -720; n <= 720; ++n) {
        const float angle = static_cast<float>(n) * pi / 180;
        const auto sine = std::sin(angle), cosine = std::cos(angle);
        Dq target{0.3F, -0.2F};
        const auto v = inverse_park(target, sine, cosine);
        const auto actual = park(v, sine, cosine);
        CHECK(std::abs(actual.d - target.d) < 1e-5F && std::abs(actual.q - target.q) < 1e-5F);
        auto duty = svpwm(v, 12);
        CHECK(duty.a >= 0.1F && duty.a <= 0.9F && duty.b >= 0.1F && duty.b <= 0.9F && duty.c >= 0.1F &&
              duty.c <= 0.9F);
        CHECK(wrap(angle) >= 0 && wrap(angle) < tau);
    }
    CHECK(std::abs(clarke({1, -0.5F, -0.5F}).alpha - 1) < 1e-6F);
    CHECK(svpwm({1, 2}, 0).a == 0.5F);
    CHECK(svpwm({std::numeric_limits<float>::quiet_NaN(), 2}, 12).a == 0.5F);
    CHECK(std::hypot(limit_vector({3, 4}, 2).d, limit_vector({3, 4}, 2).q) <= 2.0001F);
    Pi pi_control{1, 10};
    for (int i = 0; i < 1000; ++i) {
        CHECK(pi_control.step(10, 0.001F, 1) <= 1);
    }
    CHECK(pi_control.step(-1, 0.001F, 1) < 0);
    AngleTracker tracker;
    CHECK(tracker.update(tau - 0.01F, 0.001F));
    CHECK(tracker.update(0.01F, 0.001F));
    CHECK(tracker.position() > tau);
    CHECK(!tracker.update(pi, 0.001F));
    AngleTracker long_running;
    for (int turn = 0; turn < 10000; ++turn) {
        for (int quarter = 0; quarter < 4; ++quarter) {
            CHECK(long_running.update(static_cast<float>(quarter) * pi / 2, 0.01F));
        }
    }
    CHECK(long_running.update(0, 0.01F));
    CHECK(std::abs(long_running.position() - 10000 * tau) < 0.01F);
    const auto start = long_running.position();
    for (int step = 1; step <= 1000; ++step) {
        CHECK(long_running.update(static_cast<float>(step) * 0.001F, 0.00005F));
    }
    CHECK(std::abs(long_running.position() - start - 1) < 0.01F);
}
static void state_tests() {
    using namespace drive;
    Command raw;
    raw.velocity = 1;
    auto valid = ValidatedCommand::make(raw);
    CHECK(valid && valid->get().velocity == 1);
    raw.velocity = std::numeric_limits<float>::quiet_NaN();
    CHECK(!ValidatedCommand::make(raw));
    CHECK(valid->get().velocity == 1);
    ProgressWatchdog progress;
    CHECK(progress.observe(100, 5, 1));
    CHECK(progress.observe(200, 6, 1));
    CHECK(!progress.observe(200, 7, 1));
    CHECK(progress.observe(200, 7, 2));
    CHECK(!progress.observe(200, 7, 2));
    CHECK(progress.observe(400, 9, 2));
    CHECK(!progress.observe(500, 9, 2));
    StateMachine sm;
    CHECK(sm.step(0x0F, true, true, false) == State::disabled);
    CHECK(sm.step(0x06, true, true, false) == State::ready);
    CHECK(sm.step(0x07, true, true, false) == State::switched_on);
    CHECK(sm.step(0x0F, true, true, false) == State::enabled);
    CHECK((sm.status() & 0x6F) == 0x27);
    CHECK(sm.step(0x02, true, true, false) == State::quick_stop);
    CHECK(sm.step(0x0F, true, true, false) == State::quick_stop);
    CHECK(sm.step(0x0F, true, true, true) == State::disabled);
    CHECK(sm.step(0x06, false, true, false) == State::fault);
    CHECK(sm.step(0x80, false, true, false) == State::fault);
    CHECK(sm.step(0x80, true, true, false) == State::fault);
    CHECK(sm.step(0, true, true, false) == State::fault);
    CHECK(sm.step(0x80, true, true, false) == State::disabled);
    CHECK(sm.step(0x06, true, false, false) == State::disabled);
    CHECK(sm.step(0x06, true, true, false) == State::ready);
    CHECK(sm.step(0x07, true, true, false) == State::switched_on);
    CHECK(sm.step(0x0F, true, true, false) == State::enabled);
    CHECK(sm.step(0x80, true, true, false) == State::disabled);
}
static void encoder_tests() {
    // CRC-8/SAE-J1850 published check vector; encoder uses the same polynomial/init/xor.
    const std::uint8_t check[]{'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    CHECK(drivers::crc8(check) == 0x4B);
    for (unsigned value = 0; value < 65536; ++value) {
        const std::uint8_t frame[]{0x80, 0x21, static_cast<std::uint8_t>(value >> 8),
                                   static_cast<std::uint8_t>(value)};
        std::uint8_t crc = 0xFF;
        for (auto byte : frame) {
            crc ^= byte;
            for (int bit = 0; bit < 8; ++bit) {
                crc = static_cast<std::uint8_t>((crc & 0x80) ? (crc << 1) ^ 0x1D : crc << 1);
            }
        }
        CHECK(drivers::crc8(frame) == static_cast<std::uint8_t>(~crc));
    }
    const std::uint8_t payload[]{0x80, 0x21, 0x20, 0x00};
    const auto safety = static_cast<std::uint16_t>(0xFE00 | drivers::crc8(payload));
    auto angle = drivers::decode_angle(0x2000, safety);
    CHECK(angle.has_value());
    CHECK(std::abs(*angle - pi / 2) < 1e-5F);
    CHECK(!drivers::decode_angle(0x2001, safety));
    CHECK(!drivers::decode_angle(0x2000, safety & ~0x4000));
    CHECK(!drivers::decode_angle(0xFFFF, 0xFFFF));
    CHECK(!drivers::decode_angle(0, 0));
}
static void queue_tests() {
    SpscQueue<std::uint32_t, 64> queue;
    for (std::uint32_t i = 0; i < 64; ++i) {
        CHECK(queue.push(i));
    }
    CHECK(!queue.push(99));
    std::uint32_t v = 0;
    for (std::uint32_t i = 0; i < 64; ++i) {
        CHECK(queue.pop(v));
        CHECK(v == i);
    }
    CHECK(!queue.pop(v));
    std::thread producer([&] {
        for (std::uint32_t i = 0; i < 100000; ++i) {
            while (!queue.push(i)) {
                std::this_thread::yield();
            }
        }
    });
    for (std::uint32_t i = 0; i < 100000; ++i) {
        while (!queue.pop(v)) {
            std::this_thread::yield();
        }
        CHECK(v == i);
    }
    producer.join();
}
struct TestModule : Module {
    std::string_view label;
    std::vector<int>& events;
    int id;
    bool fail{}, fail_stop{};
    TestModule(std::string_view n, std::vector<int>& e, int i) : label(n), events(e), id(i) {}
    std::string_view name() const noexcept override {
        return label;
    }
    Result on_start() noexcept override {
        events.push_back(id);
        if (fail) {
            return std::unexpected(Error::hardware);
        }
        return {};
    }
    Result on_stop() noexcept override {
        events.push_back(-id);
        if (fail_stop) {
            return std::unexpected(Error::cleanup);
        }
        return {};
    }
};
static void lifecycle_tests() {
    std::vector<int> events;
    TestModule a{"a", events, 1}, b{"b", events, 2};
    ModuleManager<2> manager;
    CHECK(manager.add(a));
    CHECK(!manager.add(a));
    CHECK(manager.add(b));
    b.fail = true;
    CHECK(!manager.start());
    CHECK(events == std::vector<int>({1, 2, -2, -1}));
    events.clear();
    b.fail_stop = true;
    CHECK(!manager.start());
    CHECK(events == std::vector<int>({1, 2, -2}));
    CHECK(!manager.start());
    b.fail_stop = false;
    CHECK(manager.stop());
}
// Exercise controller outputs against encoder trajectories supplied independently
// of its profile generator. The fake rotor never follows hidden controller state.
struct ControlFixture {
    drive::MotorConfig config;
    drive::Controller controller;
    drive::Sample sample{{}, 12, 0, 25, true, true, true, true};
    drive::Command command;
    explicit ControlFixture(drive::Mode mode, float initial = 0, std::int8_t direction = 1)
        : config{0, 0.05F, 0.25F, 7, direction, 1}, controller{config} {
        sample.angle = wrap(initial * static_cast<float>(direction) * tau);
        command.mode = mode;
        command.operational = true;
        command.following_window = 100;
        for (auto cw : {6, 7, 15}) {
            command.controlword = static_cast<std::uint16_t>(cw);
            run(20);
        }
        CHECK(controller.telemetry().mode == mode);
        CHECK(controller.telemetry().fault == drive::Fault::none);
    }
    void run(unsigned ticks) {
        for (unsigned tick = 0; tick < ticks; ++tick) {
            if (controller.telemetry().samples % 20 == 0) {
                ++command.sequence;
                controller.command(command);
            }
            (void)controller.step(sample);
        }
        CHECK(controller.telemetry().fault == drive::Fault::none);
    }
    void target(float position, bool relative = false) {
        command.position = position;
        command.controlword = static_cast<std::uint16_t>(0x0F | (relative ? 0x40 : 0));
        run(20);
        command.controlword |= 0x10;
        run(200);
        CHECK(controller.telemetry().statusword & 0x1000);
    }
    float q_voltage() const {
        const auto angle = wrap(static_cast<float>(config.direction) * config.pole_pairs * sample.angle);
        return park(clarke(controller.telemetry().phase_voltage), std::sin(angle), std::cos(angle)).q;
    }
    void move_to(float target) {
        auto position = controller.telemetry().position;
        while (std::abs(position - target) > 0.00005F) {
            position = slew(position, target, 2, drive::Controller::dt);
            sample.angle = wrap(position * static_cast<float>(config.direction) * tau);
            run(1);
        }
        sample.angle = wrap(target * static_cast<float>(config.direction) * tau);
        run(4000);
        CHECK(std::abs(controller.telemetry().position - target) < 0.00001F);
    }
};
static void position_mode_tests() {
    using namespace drive;
    CHECK(Mode::position == Mode::multi_turn_position);
    Command valid;
    valid.mode = Mode::single_turn_position;
    CHECK(Controller::valid_command(valid));
    valid.mode = static_cast<Mode>(-2);
    CHECK(!Controller::valid_command(valid));
    // Crossing zero clockwise takes +0.2 turns in single-turn mode. The legacy
    // mode 1 must still take -0.8 turns to the exact continuous coordinate.
    {
        ControlFixture single{Mode::single_turn_position, 0.9F};
        single.target(0.1F);
        CHECK(single.q_voltage() > 0);
        single.move_to(1.1F);
        CHECK(single.controller.telemetry().statusword & 0x0400);
        single.command.controlword &= static_cast<std::uint16_t>(~0x10);
        single.run(20);
        CHECK(!(single.controller.telemetry().statusword & 0x1000));
    }
    {
        ControlFixture multiple{Mode::multi_turn_position, 0.9F};
        multiple.target(0.1F);
        CHECK(multiple.q_voltage() < 0);
        multiple.move_to(0.1F);
        CHECK(multiple.controller.telemetry().statusword & 0x0400);
    }
    // Counterclockwise wrap, negative targets and the half-turn tie rule.
    {
        ControlFixture single{Mode::single_turn_position, 0.1F};
        single.target(-0.1F);
        CHECK(single.q_voltage() < 0);
        single.move_to(-0.1F);
        CHECK(single.controller.telemetry().statusword & 0x0400);
    }
    {
        ControlFixture half{Mode::single_turn_position};
        half.target(0.5F);
        CHECK(half.q_voltage() < 0);
        half.move_to(-0.5F);
        CHECK(half.controller.telemetry().statusword & 0x0400);
    }
    // Relative requests retain all requested turns only in mode 1.
    for (auto mode : {Mode::single_turn_position, Mode::multi_turn_position}) {
        ControlFixture fixture{mode, 0.9F};
        fixture.target(1.3F, true);
        CHECK(fixture.q_voltage() > 0);
        fixture.move_to(mode == Mode::single_turn_position ? 1.2F : 2.2F);
        CHECK(fixture.controller.telemetry().statusword & 0x0400);
    }
    // A reversed encoder uses the same user-facing position convention.
    {
        ControlFixture reversed{Mode::single_turn_position, -0.9F, -1};
        reversed.target(-0.1F);
        CHECK(reversed.q_voltage() < 0);
        reversed.move_to(-1.1F);
        CHECK(reversed.controller.telemetry().statusword & 0x0400);
    }
    // Switching from torque to position clears the current integrators, holds
    // the measured coordinate and waits for a fresh set-point edge.
    {
        ControlFixture fixture{Mode::torque, 0.25F};
        fixture.command.torque = 1000;
        fixture.run(400);
        CHECK(fixture.q_voltage() > 0.1F);
        fixture.command.mode = Mode::single_turn_position;
        fixture.command.position = 0.9F;
        fixture.run(20);
        CHECK(std::abs(fixture.q_voltage()) < 0.00001F);
        CHECK(fixture.controller.telemetry().statusword & 0x0400);
        CHECK(!(fixture.controller.telemetry().statusword & 0x1000));
        fixture.target(0.9F);
        CHECK(fixture.q_voltage() < 0);
    }
}
static void controller_tests() {
    position_mode_tests();
    using namespace drive;
    Controller controller;
    Sample sample{{0, 0, 0}, 12, 0, 25, true, true, true, true};
    Command command{};
    command.sequence = 1;
    command.operational = true;
    command.mode = Mode::torque;
    command.controlword = 0xF;
    controller.command(command);
    for (int i = 0; i < 100; ++i) {
        CHECK(!controller.step(sample).enable);
    }
    MotorConfig cfg;
    cfg.calibrated = 1;
    Controller ready{cfg};
    for (auto cw : {0x06, 0x07, 0x0F}) {
        command.controlword = static_cast<std::uint16_t>(cw);
        ++command.sequence;
        ready.command(command);
        for (int i = 0; i < 20; ++i) {
            (void)ready.step(sample);
        }
    }
    CHECK(ready.step(sample).enable);
    sample.current.a = 0.9F;
    CHECK(!ready.step(sample).enable);
    CHECK(ready.telemetry().fault == Fault::overcurrent);
    sample.current.a = 0;
    CHECK(!ready.step(sample).enable);
    command.controlword = 0x80;
    ++command.sequence;
    ready.command(command);
    for (int i = 0; i < 20; ++i) {
        (void)ready.step(sample);
    }
    CHECK(ready.telemetry().fault == Fault::none);
    CHECK(!ready.step(sample).enable);
    command.mode = Mode::torque;
    command.controlword = 6;
    for (auto cw : {6, 7, 15}) {
        command.controlword = static_cast<std::uint16_t>(cw);
        ++command.sequence;
        ready.command(command);
        for (int i = 0; i < 20; ++i) {
            (void)ready.step(sample);
        }
    }
    sample.bus_voltage = 17;
    CHECK(!ready.step(sample).enable);
    CHECK(ready.telemetry().fault == Fault::overvoltage);
    CHECK(!Controller::valid_command(Command{.position = std::numeric_limits<float>::quiet_NaN()}));
    CHECK(!Controller::valid_config(MotorConfig{.current_limit = 1}));
    Controller stale{cfg};
    sample.bus_voltage = 12;
    for (auto cw : {6, 7, 15}) {
        command.controlword = static_cast<std::uint16_t>(cw);
        ++command.sequence;
        stale.command(command);
        for (int i = 0; i < 20; ++i) {
            (void)stale.step(sample);
        }
    }
    for (int i = 0; i < 2001; ++i) {
        (void)stale.step(sample);
    }
    CHECK(stale.telemetry().fault == Fault::communication);
}
struct MemoryStore : ParameterStore {
    std::vector<std::byte> data;
    bool read(std::span<std::byte> out) noexcept override {
        if (out.size() != data.size()) {
            return false;
        }
        std::copy(data.begin(), data.end(), out.begin());
        return true;
    }
    bool write(std::span<const std::byte> in) noexcept override {
        data.assign(in.begin(), in.end());
        return true;
    }
};
struct FakeBus {
    std::vector<RollerCanFrame> sent;
    bool blocked{};
    std::uint16_t errors{};
    std::uint32_t nominal{}, data_bitrate{}, configure_calls{};
    bool brs{}, reject_configuration{};
    RollerCanTransport transport() {
        return {this,
                [](void*) { return true; },
                [](void*) {},
                [](void* p, const RollerCanFrame* f) {
                    auto& self = *static_cast<FakeBus*>(p);
                    if (self.blocked) {
                        return false;
                    }
                    self.sent.push_back(*f);
                    return true;
                },
                [](void* p) { return static_cast<FakeBus*>(p)->errors; },
                [](void*) {},
                [](void* p, std::uint32_t nominal, std::uint32_t data, bool brs) {
                    auto& self = *static_cast<FakeBus*>(p);
                    self.nominal = nominal;
                    self.data_bitrate = data;
                    self.brs = brs;
                    ++self.configure_calls;
                    return !self.reject_configuration;
                }};
    }
};
static void protocol_tests() {
    FakeBus bus;
    auto transport = bus.transport();
    Channels channels;
    MemoryStore storage;
    protocol::Node node{transport, channels, storage};
    (void)node.load_settings();
    CHECK(node.start());
    drive::Telemetry telem{};
    telem.statusword = 0x240;
    auto process = [&] {
        CHECK(channels.telemetry.push(telem));
        node.process(1000);
        drive::ValidatedCommand c;
        CHECK(channels.commands.latest(c));
        return c.get();
    };
    (void)process();
    CHECK(!bus.sent.empty());
    CHECK(bus.sent[0].ident == 0x701 && bus.sent[0].data[0] == 0);
    auto sdo = [&](std::uint16_t index, std::uint8_t sub, std::uint8_t cs, std::uint32_t value) {
        RollerCanFrame f{
            0x601,
            8,
            {cs, static_cast<std::uint8_t>(index), static_cast<std::uint8_t>(index >> 8), sub, 0, 0, 0, 0},
            false};
        std::memcpy(f.data + 4, &value, 4);
        bus.sent.clear();
        node.receive(f);
        (void)process();
        for (const auto& out : bus.sent) {
            if (out.ident == 0x581) {
                return out;
            }
        }
        CHECK(false);
        return RollerCanFrame{};
    };
    auto upload = sdo(0x1000, 0, 0x40, 0);
    CHECK(upload.data[0] == 0x43);
    std::uint32_t type;
    std::memcpy(&type, upload.data + 4, 4);
    CHECK(type == 0x20192);
    CHECK(sdo(0x6041, 0, 0x2B, 15).data[0] == 0x80);
    CHECK(sdo(0x6666, 0, 0x40, 0).data[0] == 0x80);
    CHECK(sdo(0x6060, 0, 0x2F, 8).data[0] == 0x80);
    CHECK(sdo(0x60FF, 0, 0x23, 0x7FC00000).data[0] == 0x80);
    CHECK(sdo(0x6060, 0, 0x2F, 3).data[0] == 0x60);
    CHECK(sdo(0x60FF, 0, 0x23, std::bit_cast<std::uint32_t>(1.5F)).data[0] == 0x60);
    CHECK(OD_RAM.x60ff_00 == 1.5F);
    CHECK(sdo(0x1400, 1, 0x23, 0x201).data[0] == 0x60);
    node.receive({0, 2, {1, 1}, false});
    (void)process();
    RollerCanFrame rpdo{0x201, 7, {}, false};
    float speed = 2;
    std::memcpy(rpdo.data, &speed, 4);
    rpdo.data[4] = 6;
    rpdo.data[6] = 3;
    node.receive(rpdo);
    auto cmd = process();
    CHECK(cmd.operational);
    CHECK(cmd.velocity == 2);
    CHECK(cmd.controlword == 6);
    rpdo.size = 6;
    speed = 4;
    std::memcpy(rpdo.data, &speed, 4);
    node.receive(rpdo);
    (void)process();
    CHECK(OD_RAM.x60ff_00 == 2);
    bus.blocked = true;
    auto* tx = CO_CANtxBufferInit(node.stack()->CANmodule, 0, 0x123, false, 1, false);
    tx->data[0] = 7;
    CHECK(CO_CANsend(node.stack()->CANmodule, tx) == CO_ERROR_NO);
    CHECK(CO_CANsend(node.stack()->CANmodule, tx) == CO_ERROR_TX_OVERFLOW);
    bus.blocked = false;
    CO_CANmodule_process(node.stack()->CANmodule);
    CHECK(!tx->bufferFull);
    bus.blocked = true;
    auto* sync_tx = CO_CANtxBufferInit(node.stack()->CANmodule, 1, 0x321, false, 1, true);
    CHECK(CO_CANsend(node.stack()->CANmodule, tx) == CO_ERROR_NO);
    CHECK(CO_CANsend(node.stack()->CANmodule, sync_tx) == CO_ERROR_NO);
    CO_CANclearPendingSyncPDOs(node.stack()->CANmodule);
    CHECK(!sync_tx->bufferFull);
    CHECK(tx->bufferFull);
    bus.blocked = false;
    CO_CANmodule_process(node.stack()->CANmodule);
    CHECK(!tx->bufferFull);
    CHECK(node.stop());
}
struct RamPages final : ParameterPages {
    std::array<std::array<std::byte, page_size>, 2> memory;
    int budget{10000};
    RamPages() {
        for (auto& p : memory) {
            p.fill(std::byte{0xFF});
        }
    }
    std::span<const std::byte, page_size> page(unsigned index) const noexcept override {
        return memory[index];
    }
    bool erase(unsigned index) noexcept override {
        if (budget-- <= 0) {
            return false;
        }
        memory[index].fill(std::byte{0xFF});
        return true;
    }
    bool program_word(unsigned index, std::size_t offset, std::uint64_t value) noexcept override {
        CHECK(offset % 8 == 0 && offset + 8 <= page_size);
        if (budget-- <= 0) {
            return false;
        }
        const auto bytes = std::as_bytes(std::span{&value, 1});
        for (unsigned i = 0; i < 8; ++i) {
            memory[index][offset + i] &= bytes[i];
        }
        return true;
    }
};
static void journal_tests() {
    const std::array payload{0x12345678U, 0xFEEDBEEFU, 3U, 4U, 5U};
    auto newer = payload;
    newer[2] = 999;
    std::array<std::uint32_t, 5> actual{};
    RamPages initial;
    ParameterJournal journal{initial};
    CHECK(!journal.read(std::as_writable_bytes(std::span{actual})));
    CHECK(journal.write(std::as_bytes(std::span{payload})));
    // Simulate power loss at every erase/double-word write boundary, including
    // before the final commit. Reconstruct a journal as a fresh boot would.
    for (int cut = 0; cut <= 8; ++cut) {
        auto pages = initial;
        pages.budget = cut;
        ParameterJournal interrupted{pages};
        const auto completed = interrupted.write(std::as_bytes(std::span{newer}));
        ParameterJournal boot{pages};
        CHECK(boot.read(std::as_writable_bytes(std::span{actual})));
        CHECK(actual == (completed ? newer : payload));
    }
    auto both = initial;
    ParameterJournal second{both};
    CHECK(second.write(std::as_bytes(std::span{newer})));
    // Any bit error in the latest header or payload must fall back to the old page.
    for (unsigned byte = 0; byte < 16 + sizeof(payload); ++byte) {
        auto damaged = both;
        damaged.memory[0][byte] ^= std::byte{1};
        ParameterJournal recovered{damaged};
        CHECK(recovered.read(std::as_writable_bytes(std::span{actual})));
        CHECK(actual == payload);
    }
    auto damaged = both;
    damaged.memory[0][2040] ^= std::byte{1};
    ParameterJournal recovered{damaged};
    CHECK(recovered.read(std::as_writable_bytes(std::span{actual})));
    CHECK(actual == payload);
    const std::array<std::byte, 9> check{std::byte{'1'}, std::byte{'2'}, std::byte{'3'},
                                         std::byte{'4'}, std::byte{'5'}, std::byte{'6'},
                                         std::byte{'7'}, std::byte{'8'}, std::byte{'9'}};
    CHECK(crc32(check) == 0xCBF43926);
    CHECK(crc32(std::span{check}.subspan(4), crc32(std::span{check}.first(4))) == crc32(check));
}
static void current_loop_tests() {
    using namespace drive;
    MotorConfig config;
    config.calibrated = 1;
    Controller controller{config};
    Sample sample{{}, 12, 0, 25, true, true, true, true};
    Command command{};
    command.operational = true;
    command.mode = Mode::torque;
    for (auto cw : {6, 7, 15}) {
        command.controlword = static_cast<std::uint16_t>(cw);
        ++command.sequence;
        controller.command(command);
        for (int i = 0; i < 20; ++i) {
            (void)controller.step(sample);
        }
    }
    // Locked-rotor RL plant, independent of the PI implementation. Both torque
    // directions must converge without current saturation or sustained d-axis error.
    float alpha = 0, beta = 0;
    for (auto target : {400, -400, 0}) {
        command.torque = static_cast<std::int16_t>(target);
        for (int i = 0; i < 10000; ++i) {
            if (i % 20 == 0) {
                ++command.sequence;
                controller.command(command);
            }
            sample.current = {alpha, -0.5F * alpha + sqrt3 * 0.5F * beta,
                              -0.5F * alpha - sqrt3 * 0.5F * beta};
            const auto output = controller.step(sample);
            CHECK(output.enable);
            const auto v = clarke({output.duty.a * 12, output.duty.b * 12, output.duty.c * 12});
            alpha += (v.alpha - 2 * alpha) * Controller::dt / 0.001F;
            beta += (v.beta - 2 * beta) * Controller::dt / 0.001F;
        }
        CHECK(std::abs(beta - 0.25F * static_cast<float>(target) / 1000) < 0.003F);
        CHECK(std::abs(alpha) < 0.001F);
    }
    // MIT uses torque feedforward plus position and velocity impedance terms.
    // Check their physical current result with the same independent RL plant,
    // including saturation, halt and resumption, rather than duplicating the PI.
    command.mode = Mode::mit;
    struct MitTrial {
        float position, velocity, torque, kp, kd, expected_torque;
        std::uint16_t max_torque, controlword;
    };
    for (const auto trial : std::array{
             MitTrial{0, 0, 0.001F, 0, 0, 0.001F, 1000, 15},
             MitTrial{0.2F, 0, 0, 0.01F, 0, 0.002F, 1000, 15},
             MitTrial{0, -0.5F, 0, 0, 0.002F, -0.001F, 1000, 15},
             MitTrial{0.2F, 0.5F, 0.001F, 0.01F, 0.002F, 0.004F, 1000, 15},
             MitTrial{-0.2F, -0.5F, -0.001F, 0.01F, 0.002F, -0.004F, 1000, 15},
             MitTrial{0, 0, 1, 0, 0, 1, 300, 15},
             MitTrial{0, 0, 1, 0, 0, 0, 300, 0x10F},
             MitTrial{0, 0, -1, 0, 0, -1, 300, 15}}) {
        command.mit_position = trial.position;
        command.mit_velocity = trial.velocity;
        command.mit_torque = trial.torque;
        command.mit_kp = trial.kp;
        command.mit_kd = trial.kd;
        command.max_torque = trial.max_torque;
        command.controlword = trial.controlword;
        for (int i = 0; i < 5000; ++i) {
            if (i % 20 == 0) {
                ++command.sequence;
                controller.command(command);
            }
            sample.current = {alpha, -0.5F * alpha + sqrt3 * 0.5F * beta,
                              -0.5F * alpha - sqrt3 * 0.5F * beta};
            const auto output = controller.step(sample);
            CHECK(output.enable);
            const auto v = clarke({output.duty.a * 12, output.duty.b * 12, output.duty.c * 12});
            alpha += (v.alpha - 2 * alpha) * Controller::dt / 0.001F;
            beta += (v.beta - 2 * beta) * Controller::dt / 0.001F;
        }
        const auto limit = config.current_limit * static_cast<float>(trial.max_torque) / 1000;
        const auto expected = std::clamp(trial.expected_torque / config.torque_constant, -limit, limit);
        CHECK(std::abs(beta - expected) < 0.003F);
        CHECK(std::abs(alpha) < 0.001F);
    }
    sample.angle = 2;
    CHECK(!controller.step(sample).enable);
    CHECK(controller.telemetry().fault == Fault::encoder);
    command.controlword = 0x80;
    ++command.sequence;
    controller.command(command);
    for (int i = 0; i < 20; ++i) {
        CHECK(!controller.step(sample).enable);
    }
    CHECK(controller.telemetry().fault == Fault::none);
    CHECK((controller.telemetry().statusword & 0x6F) == 0x40);
}
static void commissioning_tests() {
    using namespace drive;
    {
        Controller controller{
            {}, [](float) noexcept -> SinCos { return {std::numeric_limits<float>::quiet_NaN(), 1}; }};
        Sample sample{{}, 12, 1, 25, true, true, true, true};
        CHECK(!controller.step(sample).enable);
        CHECK(controller.telemetry().fault == Fault::configuration);
        CHECK(std::isfinite(controller.telemetry().iq));
        Command command;
        command.controlword = 0x80;
        controller.command(command);
        for (int i = 0; i < 19; ++i) {
            (void)controller.step(sample);
        }
        // A reset edge on the 1 kHz tick cannot clear a still-failing CORDIC.
        CHECK(controller.telemetry().fault == Fault::configuration);
    }
    {
        Controller controller;
        Command command;
        command.calibration_request = 1;
        Sample sample{{}, 12, 1, 25, true, true, true, true};
        controller.command(command);
        CHECK(controller.step(sample).enable);
        command.calibration_permitted = false;
        controller.command(command);
        CHECK(!controller.step(sample).enable);
        CHECK(controller.telemetry().calibration == Calibration::failed);
        CHECK(controller.telemetry().fault == Fault::communication);
    }
    for (auto direction : {1, -1, 0}) {
        Controller controller;
        Command command{};
        command.calibration_request = 1;
        Sample sample{{}, 12, 1, 25, true, true, true, true};
        float previous = 0, total = 0;
        for (int i = 0; i < 140020; ++i) {
            if (i % 20 == 0) {
                ++command.sequence;
                controller.command(command);
            }
            const auto output = controller.step(sample);
            if (output.enable && direction) {
                const auto voltage = clarke(output.duty);
                const auto theta = std::atan2(voltage.beta, voltage.alpha);
                total += wrap_signed(theta - previous);
                previous = theta;
                sample.angle = wrap(1 + static_cast<float>(direction) * total / 7);
            }
        }
        const auto result = controller.telemetry();
        if (direction) {
            CHECK(result.calibration == Calibration::complete);
            CHECK(result.config.calibrated == 1);
            CHECK(result.config.pole_pairs == 7);
            CHECK(result.config.direction == direction);
            CHECK(result.fault == Fault::none);
        } else {
            CHECK(result.calibration == Calibration::failed);
            CHECK(result.config.calibrated == 0);
            CHECK(result.fault == Fault::calibration);
        }
        CHECK(!controller.step(sample).enable);
    }
}
struct ProtocolFixture {
    FakeBus bus;
    RollerCanTransport transport{bus.transport()};
    Channels channels;
    MemoryStore storage;
    protocol::Node node{transport, channels, storage};
    drive::Telemetry telemetry{};
    std::uint8_t node_id{1};
    explicit ProtocolFixture(std::span<const std::byte> saved = {}) {
        telemetry.statusword = 0x240;
        if (!saved.empty()) {
            CHECK(storage.write(saved));
        }
        telemetry.config = node.load_settings();
        node_id = OD_RAM.x2001_01;
        CHECK(node.start());
        (void)process();
    }
    ~ProtocolFixture() {
        CHECK(node.stop());
    }
    drive::Command process(std::uint32_t us = 1000) {
        CHECK(channels.telemetry.push(telemetry));
        node.process(us);
        drive::ValidatedCommand command;
        CHECK(channels.commands.latest(command));
        return command.get();
    }
    RollerCanFrame sdo(std::uint16_t index, std::uint8_t sub, std::uint8_t cs, std::uint32_t value = 0) {
        RollerCanFrame frame{
            static_cast<std::uint16_t>(0x600 + node_id),
            8,
            {cs, static_cast<std::uint8_t>(index), static_cast<std::uint8_t>(index >> 8), sub, 0, 0, 0, 0},
            false};
        std::memcpy(frame.data + 4, &value, 4);
        bus.sent.clear();
        node.receive(frame);
        (void)process();
        for (const auto& out : bus.sent) {
            if (out.ident == 0x580 + node_id) {
                return out;
            }
        }
        CHECK(false);
        return {};
    }
    void nmt(std::uint8_t state) {
        node.receive({0, 2, {state, node_id}, false});
        (void)process();
    }
};
static void protocol_boundary_tests() {
    {
        ProtocolFixture f;
        CHECK(f.sdo(0x1403, 1, 0x23, 0x501).data[0] == 0x60);
        CHECK(f.sdo(0x2004, 1, 0x2F, 1).data[0] == 0x60);
        CHECK(f.sdo(0x6060, 0, 0x2F, 5).data[0] == 0x60);
        f.telemetry.statusword = 0x208;
        f.telemetry.fault = drive::Fault::communication;
        (void)f.process();
        CHECK(f.sdo(0x6040, 0, 0x2B, 0x80).data[0] == 0x60);
        const auto reset = f.process();
        CHECK(!reset.operational && reset.network_ok);
        CHECK(reset.controlword == 0x80); // fault reset does not need a motion packet
        CHECK(f.sdo(0x6040, 0, 0x2B, 0).data[0] == 0x60);
        CHECK(f.process().controlword == 0); // allow a later reset rising edge
        CHECK(f.sdo(0x6040, 0, 0x2B, 0x8F).data[0] == 0x60);
        CHECK(f.process().controlword == 0x80); // still remove all enable bits
    }
    {
        ProtocolFixture f;
        // The manufacturer single-turn mode is a signed int8, including in PDO.
        CHECK(f.sdo(0x6060, 0, 0x2F, 0xFF).data[0] == 0x60);
        CHECK(f.process().mode == drive::Mode::single_turn_position);
        CHECK(f.sdo(0x6060, 0, 0x40).data[4] == 0xFF);
        CHECK(f.sdo(0x607A, 0, 0x23, std::bit_cast<std::uint32_t>(0.95F)).data[0] == 0x60);
        CHECK(f.process().position == 0.95F);
        CHECK(f.sdo(0x1401, 1, 0x23, 0x301).data[0] == 0x60);
        f.nmt(1);
        RollerCanFrame pdo{0x301, 7, {}, false};
        const auto target = -1.25F;
        std::memcpy(pdo.data, &target, 4);
        pdo.data[4] = 0x1F;
        pdo.data[6] = 0xFF;
        f.node.receive(pdo);
        const auto command = f.process();
        CHECK(command.mode == drive::Mode::single_turn_position);
        CHECK(command.position == target);
        CHECK(command.controlword == 0x1F);
        f.telemetry.statusword = 0x237;
        f.telemetry.mode = drive::Mode::single_turn_position;
        (void)f.process();
        CHECK(f.sdo(0x6061, 0, 0x40).data[4] == 0xFF);
        CHECK(f.sdo(0x6060, 0, 0x2F, 1).data[0] == 0x80);
        CHECK(f.sdo(0x6060, 0, 0x2F, 0xFF).data[0] == 0x60);
        f.telemetry.statusword = 0x240;
        CHECK(f.sdo(0x6060, 0, 0x2F, 1).data[0] == 0x60);
        CHECK(f.process().mode == drive::Mode::multi_turn_position);
        CHECK(f.sdo(0x607A, 0, 0x23, std::bit_cast<std::uint32_t>(12.5F)).data[0] == 0x60);
        CHECK(f.process().position == 12.5F);
        CHECK(f.sdo(0x6060, 0, 0x2F, 0xFE).data[0] == 0x80);
    }
    {
        ProtocolFixture f;
        CHECK(f.sdo(0x6060, 0, 0x2F, 5).data[0] == 0x60);
        CHECK(f.sdo(0x2003, 1, 0x23, std::bit_cast<std::uint32_t>(2.25F)).data[0] == 0x60);
        CHECK(f.sdo(0x2003, 2, 0x23, std::bit_cast<std::uint32_t>(-0.5F)).data[0] == 0x60);
        CHECK(f.sdo(0x2003, 3, 0x23, std::bit_cast<std::uint32_t>(0.004F)).data[0] == 0x60);
        CHECK(f.sdo(0x2003, 4, 0x2B, 200).data[0] == 0x60);
        CHECK(f.sdo(0x2003, 5, 0x2B, 10).data[0] == 0x60);
        CHECK(f.sdo(0x2003, 6, 0x2B, 300).data[0] == 0x60);
        const auto command = f.process();
        CHECK(command.mode == drive::Mode::mit);
        CHECK(command.mit_position == 2.25F && command.mit_velocity == -0.5F);
        CHECK(command.mit_torque == 0.004F);
        CHECK(std::abs(command.mit_kp - 0.2F) < 1e-6F);
        CHECK(std::abs(command.mit_kd - 0.01F) < 1e-6F);
        CHECK(command.max_torque == 300);
    }
    {
        ProtocolFixture f;
        CHECK(f.sdo(0x2100, 1, 0x23, 0x696C6163).data[0] == 0x80);
        f.telemetry.samples = 2000;
        CHECK(f.sdo(0x2100, 1, 0x23, 0x696C6163).data[0] == 0x60);
        CHECK(f.process().calibration_request == 1);
        f.telemetry.calibration = drive::Calibration::align;
        CHECK(f.sdo(0x2100, 1, 0x23, 0x696C6163).data[0] == 0x80);
    }
    {
        ProtocolFixture f;
        CHECK(f.sdo(0x6060, 0, 0x2F, 3).data[0] == 0x60);
        CHECK(f.sdo(0x1400, 1, 0x23, 0x201).data[0] == 0x60);
        f.nmt(1);
        RollerCanFrame pdo{0x201, 7, {}, false};
        const auto nan = 0x7FC00000U;
        std::memcpy(pdo.data, &nan, 4);
        pdo.data[4] = 15;
        pdo.data[6] = 3;
        f.node.receive(pdo);
        CHECK(!f.process().network_ok);
        CHECK(f.channels.asynchronous_fault.load() ==
              static_cast<std::uint32_t>(drive::Fault::configuration));
        CHECK(f.sdo(0x6040, 0, 0x2B, 0x80).data[0] == 0x60);
        CHECK(f.process().network_ok);
        f.telemetry.statusword = 0x237;
        for (int i = 0; i < 110; ++i) {
            (void)f.process();
        }
        CHECK(!f.process().network_ok);
        CHECK(f.sdo(0x6040, 0, 0x2B, 6).data[0] == 0x60);
        CHECK(f.process().network_ok);
        f.nmt(2);
        CHECK(!f.process().operational);
    }
    {
        ProtocolFixture f;
        CHECK(f.sdo(0x1016, 1, 0x23, (5U << 16) | 20U).data[0] == 0x60);
        f.nmt(1);
        f.node.receive({0x705, 1, {127}, false});
        CHECK(f.process().network_ok);
        for (int i = 0; i < 25; ++i) {
            (void)f.process();
        }
        CHECK(!f.process().network_ok);
    }
    {
        ProtocolFixture f;
        CHECK(f.sdo(0x1400, 2, 0x2F, 1).data[0] == 0x60);
        CHECK(f.sdo(0x1400, 1, 0x23, 0x201).data[0] == 0x60);
        f.nmt(1);
        RollerCanFrame pdo{0x201, 7, {}, false};
        float speed = 2.5F;
        std::memcpy(pdo.data, &speed, 4);
        pdo.data[4] = 6;
        pdo.data[6] = 3;
        f.node.receive(pdo);
        CHECK(f.process().velocity == 0);
        f.node.receive({0x80, 0, {}, false});
        CHECK(f.process().velocity == 2.5F);
    }
    {
        ProtocolFixture f;
        CHECK(f.sdo(0x2004, 1, 0x2F, 1).data[0] == 0x80); // no enabled atomic mapping yet
        CHECK(f.sdo(0x1403, 1, 0x23, 0x501).data[0] == 0x60);
        CHECK(f.sdo(0x2004, 1, 0x2F, 1).data[0] == 0x60);
        CHECK(f.sdo(0x2004, 2, 0x23, 0).data[0] == 0x80); // cannot assemble a live packet over SDO
        CHECK(f.sdo(0x2004, 9, 0x23, std::bit_cast<std::uint32_t>(101.0F)).data[0] == 0x80);
        CHECK(f.sdo(0x2003, 5, 0x2B, 20001).data[0] == 0x80);
        CHECK(f.sdo(0x6060, 0, 0x2F, 5).data[0] == 0x60);
        CHECK(f.sdo(0x6040, 0, 0x2B, 6).data[0] == 0x60);
        CHECK(f.process().controlword == 0);
        f.nmt(1);
        f.node.receive({0x501, 8, {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, false});
        const auto command = f.process();
        CHECK(command.controlword == 6);
        CHECK(command.mit_position == 0.5F);
        CHECK(command.mit_velocity == 10);
        CHECK(command.mit_kp == 100);
        CHECK(command.mit_kd == 20);
        CHECK(std::abs(command.mit_torque - 0.05F) < 1e-6F);
        f.telemetry.statusword = 0x237;
        CHECK(f.sdo(0x6040, 0, 0x2B, 0x0F).data[0] == 0x60);
        CHECK(f.sdo(0x2004, 14, 0x2B, 500).data[0] == 0x60);
        const auto limited = f.process();
        CHECK(limited.max_torque == 500);
        CHECK(limited.controlword == 0x0F && limited.network_ok);
        for (unsigned i = 0; i < 101; ++i) {
            CHECK(f.sdo(0x2004, 14, 0x2B, 300).data[0] == 0x60);
        }
        const auto expired = f.process();
        CHECK(expired.max_torque == 300 && expired.controlword == 0x0F);
        CHECK(!expired.network_ok); // dynamic limit writes cannot renew a motion packet
        f.node.receive({0x501, 8, {}, false});
        CHECK(f.process().network_ok);
        for (unsigned i = 0; i < 55; ++i) {
            // Unused ordinary MIT targets and controlwords cannot renew a packet.
            CHECK(f.sdo(0x2003, 3, 0x23, 0).data[0] == 0x60);
            CHECK(f.sdo(0x6040, 0, 0x2B, 6).data[0] == 0x60);
        }
        CHECK(!f.process().network_ok);
        f.node.receive({0x501, 8, {}, false});
        CHECK(f.process().network_ok);
        f.telemetry.statusword = 0x240;
        f.nmt(128);
        f.nmt(1);
        CHECK(f.process().controlword == 0); // a stopped session cannot reuse a packet
        f.node.receive({0x501, 8, {}, false});
        CHECK(f.process().controlword == 6);
        CHECK(f.sdo(0x2004, 5, 0x23, std::bit_cast<std::uint32_t>(0.6F)).data[0] == 0x60);
        CHECK(f.process().controlword == 0); // changed quantization requires a fresh packet
        f.node.receive({0x501, 8, {}, false});
        CHECK(f.process().controlword == 6);
        CHECK(f.sdo(0x6060, 0, 0x2F, 3).data[0] == 0x60);
        CHECK(f.sdo(0x6060, 0, 0x2F, 5).data[0] == 0x60);
        CHECK(f.process().controlword == 0);
    }
}
static void communication_settings_tests() {
    {
        ProtocolFixture f;
        CHECK(f.bus.nominal == 1000000 && f.bus.data_bitrate == 5000000 && f.bus.brs);
        CHECK(f.bus.configure_calls == 1);
        for (const auto rate : {125000U, 250000U, 500000U, 1000000U}) {
            CHECK(f.sdo(0x2001, 2, 0x23, rate).data[0] == 0x60);
        }
        for (const auto rate : {1000000U, 2000000U, 4000000U, 5000000U, 8000000U}) {
            CHECK(f.sdo(0x2001, 3, 0x23, rate).data[0] == 0x60);
        }
        CHECK(f.sdo(0x2001, 2, 0x23, 250000).data[0] == 0x60);
        CHECK(f.sdo(0x2001, 2, 0x23, 333333).data[0] == 0x80);
        CHECK(f.sdo(0x2001, 3, 0x23, 6000000).data[0] == 0x80);
        CHECK(f.sdo(0x2001, 4, 0x2F, 0).data[0] == 0x80);
        CHECK(OD_RAM.x2001_02 == 250000 && OD_RAM.x2001_03 == 8000000 && OD_RAM.x2001_04 == 1);
        CHECK(f.bus.configure_calls == 1 && f.bus.nominal == 1000000 && f.bus.data_bitrate == 5000000);
        CHECK(f.sdo(0x6040, 0, 0x2B, 6).data[0] == 0x60);
        CHECK(f.sdo(0x2001, 2, 0x23, 500000).data[0] == 0x80);
        CHECK(f.sdo(0x6040, 0, 0x2B, 0).data[0] == 0x60);
        f.telemetry.calibration = drive::Calibration::align;
        CHECK(f.sdo(0x2001, 2, 0x23, 500000).data[0] == 0x80);
        f.telemetry.calibration = drive::Calibration::required;
        f.telemetry.statusword = 0x237;
        CHECK(f.sdo(0x2001, 2, 0x23, 500000).data[0] == 0x80);
        f.telemetry.statusword = 0x240;
        f.nmt(1);
        CHECK(f.sdo(0x2001, 2, 0x23, 500000).data[0] == 0x80);
        CHECK(f.bus.configure_calls == 1);
        f.nmt(128);
        CHECK(f.sdo(0x1010, 1, 0x23, 0x65766173).data[0] == 0x60);
        CHECK(f.bus.configure_calls == 1); // saving never changes the live transport
        CHECK(f.sdo(0x2001, 2, 0x23, 125000).data[0] == 0x60); // intentionally not saved
        f.nmt(130);
        CHECK(f.node.reset_requested());
        CHECK(f.node.stop());
        protocol::Node rebooted{f.transport, f.channels, f.storage};
        (void)rebooted.load_settings();
        CHECK(OD_RAM.x2001_02 == 250000 && OD_RAM.x2001_03 == 8000000);
        CHECK(rebooted.start());
        CHECK(f.bus.configure_calls == 2 && f.bus.nominal == 250000 && f.bus.data_bitrate == 8000000);
        CHECK(rebooted.stop());
    }
    {
        FakeBus bus;
        auto transport = bus.transport();
        Channels channels;
        MemoryStore storage;
        protocol::Node node{transport, channels, storage};
        (void)node.load_settings();
        bus.reject_configuration = true;
        CHECK(!node.start());
        CHECK(node.stop());
    }
}
static void legacy_settings_tests() {
    protocol::SavedSettingsV1 legacy;
    legacy.motor = {1.25F, 0.05F, 0.2F, 11, -1, 1};
    legacy.dictionary.x2001_01 = 7;
    legacy.dictionary.x2001_02 = 125000; // ignored: v1 speed/BRS fields were read-only
    legacy.dictionary.x2001_03 = 1000000;
    legacy.dictionary.x2001_04 = 0;
    legacy.dictionary.x1005_00 = 0x80;
    legacy.dictionary.x1017_00 = 321;
    legacy.dictionary.x1400_01 = 0x222;
    legacy.dictionary.x1400_02 = 255;
    legacy.dictionary.x1600_00 = 2;
    legacy.dictionary.x1600_01 = 0x60FF0020;
    legacy.dictionary.x1600_02 = 0x60400010;
    legacy.dictionary.x1600_08 = 0x607A0020;
    legacy.dictionary.x1a03_08 = 0x60640020;
    legacy.dictionary.x6065_00 = 0.5F;
    legacy.dictionary.x6066_00 = 700;
    legacy.dictionary.x6072_00 = 800;
    legacy.dictionary.x6081_00 = 3;
    legacy.dictionary.x6083_00 = 4;
    legacy.dictionary.x6084_00 = 5;
    legacy.dictionary.x6085_00 = 6;
    legacy.dictionary.x6040_00 = 15;
    legacy.dictionary.x6060_00 = 3;
    legacy.dictionary.x60ff_00 = 4;
    const auto bytes = std::as_bytes(std::span{&legacy, 1});
    {
        ProtocolFixture f{bytes};
        CHECK(f.storage.data.size() == bytes.size());
        CHECK(std::equal(f.storage.data.begin(), f.storage.data.end(), bytes.begin()));
        CHECK(f.telemetry.config.electrical_offset == 1.25F && f.telemetry.config.torque_constant == 0.05F);
        CHECK(f.telemetry.config.pole_pairs == 11 && f.telemetry.config.direction == -1);
        CHECK(f.telemetry.config.current_limit == 0.2F && f.telemetry.config.calibrated == 1);
        CHECK(OD_RAM.x2001_01 == 7 && OD_RAM.x1017_00 == 321);
        CHECK(f.bus.nominal == 1000000 && f.bus.data_bitrate == 5000000 && f.bus.brs);
        CHECK(OD_RAM.x1400_01 == 0x222 && OD_RAM.x1600_00 == 2 && OD_RAM.x1600_08 == 0x607A0020);
        CHECK(OD_RAM.x1a03_08 == 0x60640020 && OD_RAM.x1600_09 == 0 && OD_RAM.x1a03_10 == 0);
        CHECK(OD_RAM.x6081_00 == 3 && OD_RAM.x6083_00 == 4 && OD_RAM.x6084_00 == 5 && OD_RAM.x6085_00 == 6);
        CHECK(OD_RAM.x6040_00 == 0 && OD_RAM.x6060_00 == 0 && OD_RAM.x60ff_00 == 0);
        CHECK(f.sdo(0x1010, 1, 0x23, 0x65766173).data[0] == 0x60);
        protocol::SavedSettings saved;
        CHECK(f.storage.read(std::as_writable_bytes(std::span{&saved, 1})));
        CHECK(saved.version == 2 && saved.motor.calibrated == 1 && saved.motor.electrical_offset == 1.25F);
        CHECK(saved.dictionary.x2001_02 == 1000000 && saved.dictionary.x2001_03 == 5000000);
        CHECK(f.node.stop());
        protocol::Node rebooted{f.transport, f.channels, f.storage};
        const auto motor = rebooted.load_settings();
        CHECK(motor.calibrated == 1 && motor.direction == -1 && motor.pole_pairs == 11);
        CHECK(OD_RAM.x1600_08 == 0x607A0020 && OD_RAM.x2001_01 == 7);
    }
    // A v2 commit has a different payload length. At every flash operation,
    // interruption must preserve the only v1 record until v2 is committed.
    RamPages initial;
    ParameterJournal journal{initial};
    CHECK(journal.write(bytes));
    protocol::SavedSettings newer;
    newer.motor = legacy.motor;
    newer.dictionary = OD_RAM;
    newer.dictionary.x2001_03 = 8000000;
    const auto new_bytes = std::as_bytes(std::span{&newer, 1});
    const auto operation_count = 4 + (new_bytes.size() + 7) / 8;
    for (std::size_t cut = 0; cut <= operation_count; ++cut) {
        auto pages = initial;
        pages.budget = static_cast<int>(cut);
        ParameterJournal upgrading{pages};
        const auto completed = upgrading.write(new_bytes);
        ParameterJournal boot{pages};
        protocol::SavedSettings restored;
        CHECK(boot.read(std::as_writable_bytes(std::span{&restored, 1})) == completed);
        if (completed) {
            CHECK(restored.version == 2 && restored.dictionary.x2001_03 == 8000000);
        }
        protocol::SavedSettingsV1 previous;
        CHECK(boot.read(std::as_writable_bytes(std::span{&previous, 1})));
        CHECK(std::memcmp(&previous, &legacy, sizeof(legacy)) == 0);
    }
}
static void saved_settings_tests() {
    communication_settings_tests();
    legacy_settings_tests();
    ProtocolFixture f;
    f.telemetry.config.calibrated = 1;
    f.telemetry.config.pole_pairs = 11;
    CHECK(f.sdo(0x2100, 6, 0x23, std::bit_cast<std::uint32_t>(0.2F)).data[0] == 0x60);
    CHECK(f.sdo(0x2001, 1, 0x2F, 2).data[0] == 0x60);
    CHECK(f.sdo(0x60FF, 0, 0x23, std::bit_cast<std::uint32_t>(3.0F)).data[0] == 0x60);
    CHECK(f.sdo(0x6060, 0, 0x2F, 3).data[0] == 0x60);
    CHECK(f.sdo(0x1010, 1, 0x23, 0x65766173).data[0] == 0x60);
    CHECK(f.node.stop());
    protocol::Node restarted{f.transport, f.channels, f.storage};
    const auto config = restarted.load_settings();
    CHECK(config.pole_pairs == 11 && config.calibrated == 1 && config.current_limit == 0.2F);
    CHECK(OD_RAM.x2001_01 == 2);
    CHECK(OD_RAM.x6060_00 == 0);
    CHECK(OD_RAM.x60ff_00 == 0);
    CHECK(OD_RAM.x6040_00 == 0);
}
int main() {
    math_tests();
    state_tests();
    encoder_tests();
    queue_tests();
    lifecycle_tests();
    controller_tests();
    protocol_tests();
    journal_tests();
    current_loop_tests();
    commissioning_tests();
    protocol_boundary_tests();
    saved_settings_tests();
    std::printf("PASS: %u checks\n", checks);
}
