#ifndef ROLLER_APP_BRIDGE_H
#define ROLLER_APP_BRIDGE_H
#ifdef __cplusplus
extern "C" {
#endif
void roller_app_start(void);
void roller_app_encoder_trigger(void);
void roller_panic(const char* reason);
#ifdef __cplusplus
}
#endif
#endif
