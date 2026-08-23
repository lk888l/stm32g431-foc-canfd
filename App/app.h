#ifndef BRUSHLESS_APP_H
#define BRUSHLESS_APP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cia402_od.h"

void app_init(void);
void app_background(void);
void app_current_sample_irq(uint16_t phase_a_raw,
                            uint16_t phase_b_raw,
                            uint32_t timestamp_us);
void app_supervisor_irq(uint32_t timestamp_us);
void app_break_irq(void);
void app_adc_sync_fault_irq(void);

/* Thread-safe binding points for the CANopen adapter. */
cia402_od_result_t app_cia402_od_write(uint16_t index,
                                      uint8_t subindex,
                                      const uint8_t *data,
                                      size_t data_length);
/* Fixed RPDO2 transaction: validate 0x607A and 0x60FF, then publish both and
 * their generations together. */
cia402_od_result_t app_cia402_od_write_target_batch(
    const uint8_t *target_position_data,
    size_t target_position_length,
    const uint8_t *target_velocity_data,
    size_t target_velocity_length);
cia402_od_result_t app_cia402_od_read(uint16_t index,
                                     uint8_t subindex,
                                     uint8_t *data,
                                     size_t data_capacity,
                                     size_t *data_length);
/* Call only after an atomic, range-checked, complete command/PDO commit.
 * Never call for arbitrary SDO writes or partially received MIT records. */
void app_note_canopen_command(uint32_t local_rx_timestamp_us);
/* The RPDO adapter calls this when CANopenNode cannot propagate a mapped
 * object rejection to the sender. */
void app_note_canopen_protocol_fault(void);

#endif
