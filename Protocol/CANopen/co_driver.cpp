#include "301/CO_driver.h"
#include <cstring>

namespace {
auto& transport(CO_CANmodule_t* m) {
    return *static_cast<RollerCanTransport*>(m->CANptr);
}
bool send(CO_CANmodule_t* m, CO_CANtx_t* b) {
    RollerCanFrame frame{};
    frame.ident = static_cast<uint16_t>(b->ident);
    frame.fd = b->DLC > 8;
    frame.brs = frame.fd;
    frame.size = roller_can_wire_size(b->DLC);
    frame.synchronous = b->syncFlag;
    // Value initialization above leaves every DLC padding byte zero.
    std::memcpy(frame.data, b->data, b->DLC);
    auto& t = transport(m);
    return t.send(t.context, &frame);
}
} // namespace
extern "C" {
void CO_CANsetConfigurationMode(void* pointer) {
    if (pointer) {
        auto& t = *static_cast<RollerCanTransport*>(pointer);
        t.stop(t.context);
    }
}
void CO_CANsetNormalMode(CO_CANmodule_t* m) {
    auto& t = transport(m);
    m->CANnormal = t.start(t.context);
}
CO_ReturnError_t CO_CANmodule_init(CO_CANmodule_t* m, void* pointer, CO_CANrx_t rx[], uint16_t rn,
                                   CO_CANtx_t tx[], uint16_t tn, uint16_t bitrate) {
    if (!m || !pointer || !rx || !tx ||
        !roller_can_valid_nominal_bitrate(static_cast<uint32_t>(bitrate) * 1000U)) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }
    *m = {};
    m->CANptr = pointer;
    m->rxArray = rx;
    m->txArray = tx;
    m->rxSize = rn;
    m->txSize = tn;
    m->firstCANtxMessage = true;
    for (uint16_t i = 0; i < rn; ++i) {
        rx[i] = {};
    }
    for (uint16_t i = 0; i < tn; ++i) {
        tx[i] = {};
    }
    return CO_ERROR_NO;
}
void CO_CANmodule_disable(CO_CANmodule_t* m) {
    if (m && m->CANptr) {
        CO_CANsetConfigurationMode(m->CANptr);
        m->CANnormal = false;
    }
}
CO_ReturnError_t CO_CANrxBufferInit(CO_CANmodule_t* m, uint16_t index, uint16_t ident, uint16_t mask,
                                    bool_t rtr, void* object, void (*callback)(void*, void*)) {
    if (!m || index >= m->rxSize || !object || !callback || rtr) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }
    m->rxArray[index] = {static_cast<uint16_t>(ident & 0x7FF), static_cast<uint16_t>(mask & 0x7FF), object,
                         callback};
    return CO_ERROR_NO;
}
CO_CANtx_t* CO_CANtxBufferInit(CO_CANmodule_t* m, uint16_t index, uint16_t ident, bool_t rtr, uint8_t size,
                               bool_t sync) {
    if (!m || index >= m->txSize || size > CO_PDO_MAX_SIZE || ident > 0x7FF || rtr) {
        return nullptr;
    }
    auto& b = m->txArray[index];
    b = {};
    b.ident = ident;
    b.DLC = size;
    b.syncFlag = sync;
    return &b;
}
CO_ReturnError_t CO_CANsend(CO_CANmodule_t* m, CO_CANtx_t* b) {
    if (!m || !b || b->DLC > CO_PDO_MAX_SIZE || b->ident > 0x7FF) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }
    if (b->bufferFull) {
        m->CANerrorStatus |= CO_CAN_ERRTX_OVERFLOW;
        return CO_ERROR_TX_OVERFLOW;
    }
    if (m->CANtxCount == 0 && send(m, b)) {
        m->firstCANtxMessage = false;
        return CO_ERROR_NO;
    }
    b->bufferFull = true;
    ++m->CANtxCount;
    return CO_ERROR_NO;
}
void CO_CANclearPendingSyncPDOs(CO_CANmodule_t* m) {
    auto& t = transport(m);
    t.cancel_sync(t.context);
    for (uint16_t i = 0; i < m->txSize; ++i) {
        auto& b = m->txArray[i];
        if (b.bufferFull && b.syncFlag) {
            b.bufferFull = false;
            --m->CANtxCount;
            m->CANerrorStatus |= CO_CAN_ERRTX_PDO_LATE;
        }
    }
}
void CO_CANmodule_process(CO_CANmodule_t* m) {
    auto& t = transport(m);
    constexpr uint16_t hw = CO_CAN_ERRTX_BUS_OFF | CO_CAN_ERRTX_WARNING | CO_CAN_ERRRX_WARNING |
                            CO_CAN_ERRTX_PASSIVE | CO_CAN_ERRRX_PASSIVE;
    m->CANerrorStatus = static_cast<uint16_t>((m->CANerrorStatus & ~hw) | t.errors(t.context));
    if (m->CANerrorStatus & CO_CAN_ERRTX_BUS_OFF) {
        return;
    }
    for (uint16_t i = 0; i < m->txSize; ++i) {
        auto& b = m->txArray[i];
        if (b.bufferFull) {
            if (!send(m, &b)) {
                break;
            }
            b.bufferFull = false;
            --m->CANtxCount;
            m->firstCANtxMessage = false;
        }
    }
}
}
