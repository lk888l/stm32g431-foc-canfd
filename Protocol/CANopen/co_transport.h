#ifndef ROLLER_CO_TRANSPORT_H
#define ROLLER_CO_TRANSPORT_H
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    uint16_t ident;
    uint8_t size;
    uint8_t data[8];
    bool synchronous;
} RollerCanFrame;
typedef struct {
    void* context;
    bool (*start)(void*);
    void (*stop)(void*);
    bool (*send)(void*, const RollerCanFrame*);
    uint16_t (*errors)(void*);
    void (*cancel_sync)(void*);
} RollerCanTransport;
#ifdef __cplusplus
}
#endif
#endif
