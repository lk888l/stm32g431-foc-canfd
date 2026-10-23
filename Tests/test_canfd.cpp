#include "node.hpp"
#include "fdcan_timing.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

using namespace roller;
static unsigned checks;
#define CHECK(condition)                                                                                    \
    do {                                                                                                    \
        ++checks;                                                                                           \
        if (!(condition)) {                                                                                 \
            std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);                         \
            std::exit(1);                                                                                   \
        }                                                                                                   \
    } while (false)

struct Bus {
    std::vector<RollerCanFrame> sent;
    bool blocked{};
    RollerCanTransport transport() {
        RollerCanTransport result{};
        result.context = this;
        result.start = [](void*) { return true; };
        result.stop = [](void*) {};
        result.send = [](void* context, const RollerCanFrame* frame) {
            auto& self = *static_cast<Bus*>(context);
            if (self.blocked) {
                return false;
            }
            self.sent.push_back(*frame);
            return true;
        };
        result.errors = [](void*) -> std::uint16_t { return 0; };
        result.cancel_sync = [](void*) {};
        result.configure = [](void*, std::uint32_t nominal, std::uint32_t data, bool brs) {
            return nominal == 1000000 && data == 5000000 && brs;
        };
        return result;
    }
};
struct EmptyStore final : ParameterStore {
    bool read(std::span<std::byte>) noexcept override { return false; }
    bool write(std::span<const std::byte>) noexcept override { return true; }
};
struct Fixture {
    Bus bus;
    RollerCanTransport transport{bus.transport()};
    Channels channels;
    EmptyStore store;
    protocol::Node node{transport, channels, store};
    drive::Telemetry telemetry{};
    Fixture() {
        (void)node.load_settings();
        CHECK(node.start());
        (void)process();
        CHECK(!bus.sent.empty());
        CHECK(bus.sent.front().ident == 0x701 && !bus.sent.front().fd && !bus.sent.front().brs);
    }
    ~Fixture() { CHECK(node.stop()); }
    drive::Command process(std::uint32_t elapsed = 1000) {
        CHECK(channels.telemetry.push(telemetry));
        node.process(elapsed);
        drive::ValidatedCommand command;
        CHECK(channels.commands.latest(command));
        return command.get();
    }
    RollerCanFrame sdo(std::uint16_t index, std::uint8_t sub, std::uint8_t cs,
                       std::uint32_t value = 0) {
        RollerCanFrame frame{};
        frame.ident = 0x601;
        frame.size = 8;
        frame.data[0] = cs;
        frame.data[1] = static_cast<std::uint8_t>(index);
        frame.data[2] = static_cast<std::uint8_t>(index >> 8);
        frame.data[3] = sub;
        std::memcpy(frame.data + 4, &value, 4);
        bus.sent.clear();
        node.receive(frame);
        (void)process();
        for (const auto& reply : bus.sent) {
            if (reply.ident == 0x581) {
                CHECK(!reply.fd && !reply.brs && reply.size == 8);
                return reply;
            }
        }
        CHECK(false);
        return {};
    }
    void write(std::uint16_t index, std::uint8_t sub, std::uint32_t value,
               std::uint8_t cs = 0x23) {
        CHECK(sdo(index, sub, cs, value).data[0] == 0x60);
    }
    void nmt(std::uint8_t state) {
        RollerCanFrame frame{};
        frame.size = 2;
        frame.data[0] = state;
        frame.data[1] = 1;
        node.receive(frame);
        (void)process();
    }
    void map(bool transmit, const std::vector<std::uint32_t>& entries,
             std::uint8_t type = 255) {
        const std::uint16_t communication = transmit ? 0x1800 : 0x1400;
        const std::uint16_t mapping = transmit ? 0x1A00 : 0x1600;
        const std::uint32_t cob_id = transmit ? 0x181 : 0x201;
        write(communication, 1, cob_id | 0x80000000U);
        write(mapping, 0, 0, 0x2F);
        for (std::size_t i = 0; i < entries.size(); ++i) {
            write(mapping, static_cast<std::uint8_t>(i + 1), entries[i]);
        }
        write(mapping, 0, static_cast<std::uint32_t>(entries.size()), 0x2F);
        write(communication, 2, type, 0x2F);
        write(communication, 1, cob_id);
    }
};

static constexpr std::array<std::uint8_t, 19> lengths{
    1, 7, 8, 9, 10, 11, 12, 13, 16, 17, 20, 21, 24, 25, 32, 33, 48, 49, 64};

static std::vector<std::uint32_t> mapping(std::uint8_t size) {
    std::vector<std::uint32_t> result;
    while (size >= 4) {
        result.push_back(0x30000320);
        size -= 4;
    }
    if (size >= 2) {
        result.push_back(0x30000210);
        size -= 2;
    }
    if (size) {
        result.push_back(0x30000108);
    }
    CHECK(result.size() <= 16);
    return result;
}
static RollerCanFrame pdo(std::uint8_t size) {
    RollerCanFrame frame{};
    frame.ident = 0x201;
    frame.fd = size > 8;
    frame.brs = frame.fd;
    frame.size = roller_can_wire_size(size);
    for (std::uint8_t i = 0; i < size; ++i) {
        frame.data[i] = static_cast<std::uint8_t>(i + 1);
    }
    // RX must ignore physical padding, even if the sender does not zero it.
    std::fill(frame.data + size, frame.data + frame.size, 0xA5);
    return frame;
}
static void check_received(const RollerCanFrame& frame, const std::vector<std::uint32_t>& entries) {
    std::size_t offset = 0;
    std::array<std::uint32_t, 5> expected{};
    std::array<bool, 5> mapped{};
    for (const auto entry : entries) {
        const auto size = (entry & 0xFF) / 8;
        expected[size] = 0;
        std::memcpy(&expected[size], frame.data + offset, size);
        mapped[size] = true;
        offset += size;
    }
    if (mapped[4]) { CHECK(OD_RAM.x3000_03 == expected[4]); }
    if (mapped[2]) { CHECK(OD_RAM.x3000_02 == expected[2]); }
    if (mapped[1]) { CHECK(OD_RAM.x3000_01 == expected[1]); }
}
static void rx_lengths() {
    for (const auto length : lengths) {
        Fixture f;
        const auto entries = mapping(length);
        f.map(false, entries);
        f.nmt(1);
        const auto frame = pdo(length);
        f.node.receive(frame);
        CHECK(f.process().network_ok);
        check_received(frame, entries);
        const std::array before{OD_RAM.x3000_03, static_cast<std::uint32_t>(OD_RAM.x3000_02),
                                static_cast<std::uint32_t>(OD_RAM.x3000_01)};
        const auto reject = [&](RollerCanFrame wrong) {
            std::fill(std::begin(wrong.data), std::end(wrong.data), 0xEE);
            f.node.receive(wrong);
            (void)f.process();
            CHECK(OD_RAM.x3000_03 == before[0]);
            CHECK(OD_RAM.x3000_02 == before[1]);
            CHECK(OD_RAM.x3000_01 == before[2]);
        };
        auto wrong = frame;
        wrong.fd = !frame.fd;
        wrong.brs = wrong.fd;
        reject(wrong); // long classic frame or FD for a short classic mapping
        wrong = frame;
        wrong.size = length > 8 ? 8 : static_cast<std::uint8_t>(length - 1);
        reject(wrong);
        wrong = frame;
        wrong.size = length > 8 ? (frame.size == 64 ? 48 : 64) : 8;
        if (wrong.size != frame.size) { reject(wrong); }
        wrong = frame;
        wrong.size = 65;
        reject(wrong);
        wrong = frame;
        wrong.fd = false;
        wrong.brs = true;
        reject(wrong); // BRS is not a valid flag on a classic frame
        if (length > 8) {
            wrong = frame;
            wrong.size = 9; // no 9-byte physical CAN FD DLC
            reject(wrong);
        }
    }
}
static void tx_lengths() {
    for (const auto length : lengths) {
        Fixture f;
        const auto entries = mapping(length);
        f.map(true, entries, 1);
        f.write(0x3000, 1, 0x81, 0x2F);
        f.write(0x3000, 2, 0x8283, 0x2B);
        f.write(0x3000, 3, 0x84858687);
        f.nmt(1);
        f.bus.sent.clear();
        RollerCanFrame sync{};
        sync.ident = 0x80;
        f.node.receive(sync);
        (void)f.process();
        const auto found = std::find_if(f.bus.sent.begin(), f.bus.sent.end(),
                                       [](const auto& frame) { return frame.ident == 0x181; });
        CHECK(found != f.bus.sent.end());
        CHECK(found->size == roller_can_wire_size(length));
        CHECK(found->fd == (length > 8) && found->brs == found->fd);
        std::size_t offset = 0;
        for (const auto entry : entries) {
            const auto size = (entry & 0xFF) / 8;
            const std::uint32_t expected = size == 4 ? 0x84858687 : size == 2 ? 0x8283 : 0x81;
            CHECK(std::memcmp(found->data + offset, &expected, size) == 0);
            offset += size;
        }
        for (auto i = length; i < found->size; ++i) { CHECK(found->data[i] == 0); }
    }
}
static void service_boundaries() {
    Fixture f;
    RollerCanFrame fd_sdo{};
    fd_sdo.ident = 0x601;
    fd_sdo.size = 8;
    fd_sdo.fd = fd_sdo.brs = true;
    fd_sdo.data[0] = 0x40;
    fd_sdo.data[2] = 0x10;
    f.bus.sent.clear();
    f.node.receive(fd_sdo);
    (void)f.process();
    CHECK(std::none_of(f.bus.sent.begin(), f.bus.sent.end(),
                       [](const auto& frame) { return frame.ident == 0x581; }));
    fd_sdo.size = 64;
    f.node.receive(fd_sdo);
    (void)f.process();
    CHECK(std::none_of(f.bus.sent.begin(), f.bus.sent.end(),
                       [](const auto& frame) { return frame.ident == 0x581; }));
    CHECK(f.sdo(0x1000, 0, 0x40).data[0] == 0x43);
    RollerCanFrame nmt{};
    nmt.size = 2;
    nmt.data[0] = 1;
    nmt.data[1] = 1;
    nmt.fd = nmt.brs = true;
    f.node.receive(nmt);
    CHECK(!f.process().operational);
    f.map(false, mapping(9), 1);
    f.nmt(1);
    f.node.receive(pdo(9));
    (void)f.process();
    CHECK(OD_RAM.x3000_03 == 0 && OD_RAM.x3000_01 == 0);
    RollerCanFrame sync{};
    sync.ident = 0x80;
    sync.fd = sync.brs = true;
    f.node.receive(sync);
    (void)f.process();
    CHECK(OD_RAM.x3000_03 == 0 && OD_RAM.x3000_01 == 0);
    sync.fd = sync.brs = false;
    f.node.receive(sync);
    (void)f.process();
    check_received(pdo(9), mapping(9));
    f.nmt(128);
    f.write(0x1016, 1, (5U << 16) | 20U);
    RollerCanFrame heartbeat{};
    heartbeat.ident = 0x705;
    heartbeat.size = 1;
    heartbeat.data[0] = 127;
    f.node.receive(heartbeat);
    CHECK(f.process().network_ok);
    heartbeat.fd = heartbeat.brs = true;
    for (unsigned i = 0; i < 25; ++i) {
        f.node.receive(heartbeat);
        (void)f.process();
    }
    CHECK(!f.process().network_ok); // FD heartbeat cannot keep a classic consumer alive
}
static void mit_fd_command() {
    Fixture f;
    f.map(false, {0x20030120, 0x20030220, 0x20030320, 0x20030410, 0x20030510,
                  0x60400010, 0x60600008});
    f.nmt(1);
    auto frame = pdo(19);
    const std::array<float, 3> targets{0.25F, -0.5F, 0.002F};
    std::memcpy(frame.data, targets.data(), sizeof(targets));
    const std::array<std::uint16_t, 3> integers{100, 10, 6};
    std::memcpy(frame.data + 12, integers.data(), sizeof(integers));
    frame.data[18] = 5;
    f.node.receive(frame);
    const auto command = f.process();
    CHECK(command.operational && command.network_ok && command.mode == drive::Mode::mit);
    CHECK(command.controlword == 6 && command.mit_position == 0.25F);
    CHECK(command.mit_velocity == -0.5F && command.mit_torque == 0.002F);
    CHECK(std::abs(command.mit_kp - 0.1F) < 1e-6F);
    CHECK(std::abs(command.mit_kd - 0.01F) < 1e-6F);
}
static void driver_boundaries() {
    Bus bus;
    auto transport = bus.transport();
    CO_CANmodule_t module{};
    CO_CANrx_t receive[1]{};
    CO_CANtx_t transmit[1]{};
    for (const auto speed : {125, 250, 500, 1000}) {
        CHECK(CO_CANmodule_init(&module, &transport, receive, 1, transmit, 1,
                               static_cast<std::uint16_t>(speed)) == CO_ERROR_NO);
    }
    CHECK(CO_CANmodule_init(&module, &transport, receive, 1, transmit, 1, 800) ==
          CO_ERROR_ILLEGAL_ARGUMENT);
    CHECK(CO_CANtxBufferInit(&module, 0, 0x181, false, 65, false) == nullptr);
    auto* buffer = CO_CANtxBufferInit(&module, 0, 0x181, false, 9, false);
    CHECK(buffer != nullptr);
    std::fill(std::begin(buffer->data), std::end(buffer->data), 0xA7);
    bus.blocked = true;
    CHECK(CO_CANsend(&module, buffer) == CO_ERROR_NO && buffer->bufferFull);
    bus.blocked = false;
    CO_CANmodule_process(&module);
    CHECK(!buffer->bufferFull && bus.sent.size() == 1);
    CHECK(bus.sent[0].fd && bus.sent[0].brs && bus.sent[0].size == 12);
    CHECK(bus.sent[0].data[8] == 0xA7 && bus.sent[0].data[9] == 0 && bus.sent[0].data[11] == 0);
    buffer->DLC = 65;
    CHECK(CO_CANsend(&module, buffer) == CO_ERROR_ILLEGAL_ARGUMENT);
}
static void dlc_and_timing() {
    constexpr std::array<std::uint8_t, 16> physical{0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
    for (std::uint8_t dlc = 0; dlc < physical.size(); ++dlc) {
        CHECK(roller_can_dlc_to_length(dlc) == physical[dlc]);
    }
    for (std::uint8_t length = 0; length <= 64; ++length) {
        const auto expected = std::lower_bound(physical.begin(), physical.end(), length);
        CHECK(roller_can_length_to_dlc(length) == expected - physical.begin());
        CHECK(roller_can_wire_size(length) == *expected);
    }
    CHECK(roller_can_dlc_to_length(16) == 255);
    CHECK(roller_can_length_to_dlc(65) == 255 && roller_can_wire_size(255) == 255);
    for (const std::uint32_t bitrate : {125000U, 250000U, 500000U, 1000000U}) {
        const auto timing = platform::can_nominal_timing(bitrate);
        CHECK(timing.has_value() && roller_can_valid_nominal_bitrate(bitrate));
        CHECK(std::uint64_t{bitrate} * timing->prescaler * (1 + timing->segment1 + timing->segment2) ==
              160000000);
        CHECK(timing->prescaler >= 1 && timing->prescaler <= 512);
        CHECK(timing->segment1 >= 1 && timing->segment1 <= 256);
        CHECK(timing->segment2 >= 1 && timing->segment2 <= 128);
        CHECK(timing->sjw >= 1 && timing->sjw <= 128 && timing->sjw <= timing->segment2);
    }
    for (const std::uint32_t bitrate : {1000000U, 2000000U, 4000000U, 5000000U, 8000000U}) {
        const auto timing = platform::can_data_timing(bitrate);
        CHECK(timing.has_value() && roller_can_valid_data_bitrate(bitrate));
        CHECK(std::uint64_t{bitrate} * timing->prescaler * (1 + timing->segment1 + timing->segment2) ==
              160000000);
        CHECK(timing->prescaler >= 1 && timing->prescaler <= 32);
        CHECK(timing->segment1 >= 1 && timing->segment1 <= 32);
        CHECK(timing->segment2 >= 1 && timing->segment2 <= 16);
        CHECK(timing->sjw >= 1 && timing->sjw <= 16 && timing->sjw <= timing->segment2);
        CHECK(4 * (1 + timing->segment1) == 3 * (1 + timing->segment1 + timing->segment2));
        // The 1 Mbit/s timing uses prescaler 5, which cannot enable M_CAN TDC.
        CHECK(platform::can_tdc_enabled(*timing) == (bitrate != 1000000U));
    }
    CHECK(!platform::can_tdc_enabled({0, 23, 8, 4}));
    CHECK(platform::can_tdc_enabled({1, 23, 8, 4}));
    CHECK(platform::can_tdc_enabled({2, 29, 10, 4}));
    CHECK(!platform::can_tdc_enabled({3, 23, 8, 4}));
    CHECK(!platform::can_tdc_enabled({32, 23, 8, 4}));
    for (const auto invalid : {0U, 100000U, 800000U, 1234567U, 16000000U}) {
        CHECK(!platform::can_nominal_timing(invalid) && !roller_can_valid_nominal_bitrate(invalid));
        CHECK(!platform::can_data_timing(invalid) && !roller_can_valid_data_bitrate(invalid));
    }
}
int main() {
    static_assert(CO_PDO_MAX_SIZE == 64 && CO_PDO_MAX_MAPPED_ENTRIES == 16);
    rx_lengths();
    tx_lengths();
    service_boundaries();
    mit_fd_command();
    driver_boundaries();
    dlc_and_timing();
    std::printf("PASS CAN FD: %u checks\n", checks);
}
