#include "lab_stlink.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

typedef struct {
    volatile uint32_t magic;
    volatile uint32_t encoder_direction_bits;
    volatile uint32_t electrical_zero_bits;
    volatile uint32_t guard;
} lab_stlink_retained_calibration_t;

/* The legacy ST-LINK CLI resets the target after a RAM write.  A NOLOAD
 * mailbox survives that reset long enough for the restarted firmware to
 * consume the guarded, one-shot command. */
lab_stlink_command_mailbox_t g_lab_stlink_command
    __attribute__((section(".noinit.lab_stlink_command"), used));
lab_stlink_retained_calibration_t g_lab_stlink_calibration
    __attribute__((section(".noinit.lab_stlink_calibration"), used));
volatile lab_stlink_status_t g_lab_stlink_status;
volatile lab_stlink_trace_t g_lab_stlink_trace
    __attribute__((section(".noinit.lab_stlink_trace"), used));

_Static_assert(sizeof(lab_stlink_command_mailbox_t) == 28U,
               "ST-Link command mailbox ABI changed");
_Static_assert(sizeof(lab_stlink_status_t) == 176U,
               "ST-Link status ABI changed");
_Static_assert(sizeof(lab_stlink_trace_sample_t) == 48U,
               "ST-Link trace sample ABI changed");
_Static_assert(sizeof(lab_stlink_trace_t) == 18472U,
               "ST-Link trace ABI changed");
_Static_assert(sizeof(lab_stlink_retained_calibration_t) == 16U,
               "ST-Link retained calibration ABI changed");

static uint32_t float_bits(float value)
{
    uint32_t bits;

    memcpy(&bits, &value, sizeof(bits));
    return bits;
}

static float bits_float(uint32_t bits)
{
    float value;

    memcpy(&value, &bits, sizeof(value));
    return value;
}

uint32_t lab_stlink_command_guard(uint32_t command,
                                  uint32_t duration_ms,
                                  uint32_t value_bits,
                                  uint32_t voltage_limit_bits,
                                  uint32_t sequence)
{
    return LAB_STLINK_COMMAND_MAGIC ^ command ^ duration_ms ^ value_bits ^
           voltage_limit_bits ^ sequence ^ LAB_STLINK_GUARD_SALT;
}

bool lab_stlink_take_request(uint32_t *last_sequence,
                             lab_commissioning_request_t *request,
                             uint32_t *accepted_sequence)
{
    uint32_t sequence_before;
    uint32_t sequence_after;
    uint32_t magic;
    uint32_t command;
    uint32_t duration_ms;
    uint32_t value_bits;
    uint32_t voltage_limit_bits;
    uint32_t guard;

    if ((last_sequence == NULL) || (request == NULL) ||
        (accepted_sequence == NULL)) {
        return false;
    }

    sequence_before = g_lab_stlink_command.sequence;
    if ((sequence_before == 0U) || (sequence_before == *last_sequence)) {
        return false;
    }
    magic = g_lab_stlink_command.magic;
    command = g_lab_stlink_command.command;
    duration_ms = g_lab_stlink_command.duration_ms;
    value_bits = g_lab_stlink_command.value_bits;
    voltage_limit_bits = g_lab_stlink_command.voltage_limit_bits;
    guard = g_lab_stlink_command.guard;
    sequence_after = g_lab_stlink_command.sequence;

    if ((sequence_before != sequence_after) ||
        (magic != LAB_STLINK_COMMAND_MAGIC) ||
        (guard != lab_stlink_command_guard(command,
                                           duration_ms,
                                           value_bits,
                                           voltage_limit_bits,
                                           sequence_before)) ||
        (command > (uint32_t)LAB_COMMISSIONING_COMMAND_OPEN_LOOP)) {
        return false;
    }

    request->command = (lab_commissioning_command_t)command;
    request->duration_ms = duration_ms;
    memcpy(&request->value, &value_bits, sizeof(request->value));
    memcpy(&request->voltage_limit_v,
           &voltage_limit_bits,
           sizeof(request->voltage_limit_v));
    *last_sequence = sequence_before;
    *accepted_sequence = sequence_before;
    return true;
}

bool lab_stlink_consume_request(uint32_t sequence)
{
    if ((sequence == 0U) || (g_lab_stlink_command.sequence != sequence)) {
        return false;
    }
    /* Sequence is the commit word.  Clearing it makes a later watchdog or
     * debugger reset unable to replay a motion request. */
    g_lab_stlink_command.sequence = 0U;
    __asm volatile("dmb" ::: "memory");
    return true;
}

static bool retained_command_is_valid(void)
{
    uint32_t sequence_before;
    uint32_t sequence_after;
    uint32_t magic;
    uint32_t command;
    uint32_t duration_ms;
    uint32_t value_bits;
    uint32_t voltage_limit_bits;
    uint32_t guard;

    sequence_before = g_lab_stlink_command.sequence;
    if (sequence_before == 0U) {
        return false;
    }
    magic = g_lab_stlink_command.magic;
    command = g_lab_stlink_command.command;
    duration_ms = g_lab_stlink_command.duration_ms;
    value_bits = g_lab_stlink_command.value_bits;
    voltage_limit_bits = g_lab_stlink_command.voltage_limit_bits;
    guard = g_lab_stlink_command.guard;
    sequence_after = g_lab_stlink_command.sequence;

    return (sequence_before == sequence_after) &&
           (magic == LAB_STLINK_COMMAND_MAGIC) &&
           (command <= (uint32_t)LAB_COMMISSIONING_COMMAND_OPEN_LOOP) &&
           (guard == lab_stlink_command_guard(command,
                                              duration_ms,
                                              value_bits,
                                              voltage_limit_bits,
                                              sequence_before));
}

void lab_stlink_initialize_retention(bool allow_software_reset_retention)
{
    /* Debug PIN/SW resets retain a guarded calibration independently of the
     * mailbox because the legacy tool writes RAM after releasing reset. An
     * incomplete command is still discarded. Power/watchdog-class resets and
     * the flash script explicitly invalidate calibration. */
    if (allow_software_reset_retention) {
        if (!retained_command_is_valid()) {
            g_lab_stlink_command.sequence = 0U;
        }
        if ((g_lab_stlink_trace.magic != LAB_STLINK_TRACE_MAGIC) ||
            (g_lab_stlink_trace.version != LAB_STLINK_TRACE_ABI_VERSION) ||
            (g_lab_stlink_trace.capacity != LAB_STLINK_TRACE_CAPACITY) ||
            (g_lab_stlink_trace.sample_count >
             LAB_STLINK_TRACE_CAPACITY) ||
            ((g_lab_stlink_trace.sample_count ==
              LAB_STLINK_TRACE_CAPACITY) &&
             (g_lab_stlink_trace.oldest_sample_index >=
              LAB_STLINK_TRACE_CAPACITY))) {
            g_lab_stlink_trace.magic = 0U;
        }
        __asm volatile("dmb" ::: "memory");
        return;
    }
    g_lab_stlink_command.sequence = 0U;
    g_lab_stlink_calibration.magic = 0U;
    g_lab_stlink_trace.magic = 0U;
    __asm volatile("dmb" ::: "memory");
}

void lab_stlink_clear_calibration(void)
{
    g_lab_stlink_calibration.magic = 0U;
    __asm volatile("dmb" ::: "memory");
}

bool lab_stlink_store_calibration(float encoder_direction,
                                  float electrical_zero_rad)
{
    uint32_t direction_bits;
    uint32_t zero_bits;

    if (((encoder_direction != 1.0f) && (encoder_direction != -1.0f)) ||
        !isfinite(electrical_zero_rad) ||
        (fabsf(electrical_zero_rad) > 3.14159265358979323846f)) {
        return false;
    }
    direction_bits = float_bits(encoder_direction);
    zero_bits = float_bits(electrical_zero_rad);
    g_lab_stlink_calibration.magic = 0U;
    __asm volatile("dmb" ::: "memory");
    g_lab_stlink_calibration.encoder_direction_bits = direction_bits;
    g_lab_stlink_calibration.electrical_zero_bits = zero_bits;
    g_lab_stlink_calibration.guard = LAB_STLINK_CALIBRATION_MAGIC ^
                                     direction_bits ^ zero_bits ^
                                     LAB_STLINK_CALIBRATION_SALT;
    __asm volatile("dmb" ::: "memory");
    g_lab_stlink_calibration.magic = LAB_STLINK_CALIBRATION_MAGIC;
    return true;
}

bool lab_stlink_load_calibration(float *encoder_direction,
                                 float *electrical_zero_rad)
{
    uint32_t magic_before;
    uint32_t magic_after;
    uint32_t direction_bits;
    uint32_t zero_bits;
    uint32_t guard;
    float direction;
    float zero;

    if ((encoder_direction == NULL) || (electrical_zero_rad == NULL)) {
        return false;
    }
    magic_before = g_lab_stlink_calibration.magic;
    direction_bits = g_lab_stlink_calibration.encoder_direction_bits;
    zero_bits = g_lab_stlink_calibration.electrical_zero_bits;
    guard = g_lab_stlink_calibration.guard;
    magic_after = g_lab_stlink_calibration.magic;
    if ((magic_before != magic_after) ||
        (magic_before != LAB_STLINK_CALIBRATION_MAGIC) ||
        (guard != (LAB_STLINK_CALIBRATION_MAGIC ^ direction_bits ^
                   zero_bits ^ LAB_STLINK_CALIBRATION_SALT))) {
        return false;
    }
    direction = bits_float(direction_bits);
    zero = bits_float(zero_bits);
    if (((direction != 1.0f) && (direction != -1.0f)) || !isfinite(zero) ||
        (fabsf(zero) > 3.14159265358979323846f)) {
        return false;
    }
    *encoder_direction = direction;
    *electrical_zero_rad = zero;
    return true;
}

void lab_stlink_publish_status(const lab_stlink_status_t *status)
{
    uint32_t publish_sequence;

    if (status == NULL) {
        return;
    }
    publish_sequence = g_lab_stlink_status.publish_sequence;
    if ((publish_sequence & 1U) != 0U) {
        ++publish_sequence;
    }
    g_lab_stlink_status.publish_sequence = publish_sequence + 1U;
    __asm volatile("dmb" ::: "memory");

    g_lab_stlink_status.magic = LAB_STLINK_STATUS_MAGIC;
    g_lab_stlink_status.version = LAB_STLINK_ABI_VERSION;
    g_lab_stlink_status.applied_command_sequence =
        status->applied_command_sequence;
    g_lab_stlink_status.state = status->state;
    g_lab_stlink_status.result = status->result;
    g_lab_stlink_status.flags = status->flags;
    g_lab_stlink_status.timestamp_us = status->timestamp_us;
    g_lab_stlink_status.encoder_raw_count = status->encoder_raw_count;
    g_lab_stlink_status.encoder_health = status->encoder_health;
    g_lab_stlink_status.active_faults = status->active_faults;
    g_lab_stlink_status.latched_faults = status->latched_faults;
    g_lab_stlink_status.current_loop_count = status->current_loop_count;
    g_lab_stlink_status.supervisor_loop_count =
        status->supervisor_loop_count;
    g_lab_stlink_status.raw_mechanical_angle_rad =
        status->raw_mechanical_angle_rad;
    g_lab_stlink_status.directed_position_rad =
        status->directed_position_rad;
    g_lab_stlink_status.mechanical_velocity_rad_s =
        status->mechanical_velocity_rad_s;
    g_lab_stlink_status.encoder_direction = status->encoder_direction;
    g_lab_stlink_status.electrical_zero_rad = status->electrical_zero_rad;
    g_lab_stlink_status.alignment_delta_rad = status->alignment_delta_rad;
    g_lab_stlink_status.alpha_voltage_v = status->alpha_voltage_v;
    g_lab_stlink_status.beta_voltage_v = status->beta_voltage_v;
    g_lab_stlink_status.q_voltage_v = status->q_voltage_v;
    g_lab_stlink_status.duty_a = status->duty_a;
    g_lab_stlink_status.duty_b = status->duty_b;
    g_lab_stlink_status.duty_c = status->duty_c;
    g_lab_stlink_status.bus_voltage_v = status->bus_voltage_v;
    g_lab_stlink_status.encoder_status_raw = status->encoder_status_raw;
    g_lab_stlink_status.encoder_agc = status->encoder_agc;
    g_lab_stlink_status.encoder_magnitude = status->encoder_magnitude;
    g_lab_stlink_status.encoder_minimum_agc =
        status->encoder_minimum_agc;
    g_lab_stlink_status.encoder_maximum_agc =
        status->encoder_maximum_agc;
    g_lab_stlink_status.encoder_minimum_magnitude =
        status->encoder_minimum_magnitude;
    g_lab_stlink_status.encoder_maximum_magnitude =
        status->encoder_maximum_magnitude;
    g_lab_stlink_status.encoder_angle_read_count =
        status->encoder_angle_read_count;
    g_lab_stlink_status.encoder_diagnostic_read_count =
        status->encoder_diagnostic_read_count;
    g_lab_stlink_status.encoder_bus_error_count =
        status->encoder_bus_error_count;
    g_lab_stlink_status.encoder_bus_recovery_count =
        status->encoder_bus_recovery_count;
    g_lab_stlink_status.encoder_bus_failure_count =
        status->encoder_bus_failure_count;
    g_lab_stlink_status.encoder_diagnostic_failure_count =
        status->encoder_diagnostic_failure_count;
    g_lab_stlink_status.encoder_age_us = status->encoder_age_us;
    g_lab_stlink_status.encoder_maximum_age_us =
        status->encoder_maximum_age_us;
    g_lab_stlink_status.encoder_angle_glitch_count =
        status->encoder_angle_glitch_count;
    g_lab_stlink_status.encoder_angle_glitch_recovery_count =
        status->encoder_angle_glitch_recovery_count;

    __asm volatile("dmb" ::: "memory");
    g_lab_stlink_status.publish_sequence = publish_sequence + 2U;
}

void lab_stlink_trace_reset(uint32_t command_sequence)
{
    uint32_t publish_sequence = g_lab_stlink_trace.publish_sequence;

    if ((publish_sequence & 1U) != 0U) {
        ++publish_sequence;
    }
    g_lab_stlink_trace.publish_sequence = publish_sequence + 1U;
    __asm volatile("dmb" ::: "memory");

    g_lab_stlink_trace.magic = LAB_STLINK_TRACE_MAGIC;
    g_lab_stlink_trace.version = LAB_STLINK_TRACE_ABI_VERSION;
    g_lab_stlink_trace.command_sequence = command_sequence;
    g_lab_stlink_trace.sample_count = 0U;
    g_lab_stlink_trace.capacity = LAB_STLINK_TRACE_CAPACITY;
    g_lab_stlink_trace.sample_period_us = LAB_STLINK_TRACE_PERIOD_US;
    g_lab_stlink_trace.final_state = LAB_COMMISSIONING_STATE_IDLE;
    g_lab_stlink_trace.final_result = LAB_COMMISSIONING_RESULT_NONE;
    g_lab_stlink_trace.oldest_sample_index = 0U;

    __asm volatile("dmb" ::: "memory");
    g_lab_stlink_trace.publish_sequence = publish_sequence + 2U;
}

void lab_stlink_trace_append(const lab_stlink_trace_sample_t *sample)
{
    uint32_t count = g_lab_stlink_trace.sample_count;
    uint32_t index;

    if ((sample == NULL) ||
        (g_lab_stlink_trace.magic != LAB_STLINK_TRACE_MAGIC) ||
        (count > LAB_STLINK_TRACE_CAPACITY)) {
        return;
    }
    if (count < LAB_STLINK_TRACE_CAPACITY) {
        index = count;
        ++count;
    } else {
        index = g_lab_stlink_trace.oldest_sample_index;
        if (index >= LAB_STLINK_TRACE_CAPACITY) {
            return;
        }
    }
    g_lab_stlink_trace.samples[index] = *sample;
    __asm volatile("dmb" ::: "memory");
    if (count < LAB_STLINK_TRACE_CAPACITY) {
        g_lab_stlink_trace.sample_count = count;
    } else if (g_lab_stlink_trace.sample_count <
               LAB_STLINK_TRACE_CAPACITY) {
        g_lab_stlink_trace.sample_count = LAB_STLINK_TRACE_CAPACITY;
        g_lab_stlink_trace.oldest_sample_index = 0U;
    } else {
        ++index;
        if (index >= LAB_STLINK_TRACE_CAPACITY) {
            index = 0U;
        }
        g_lab_stlink_trace.oldest_sample_index = index;
    }
}

void lab_stlink_trace_finish(uint32_t state, uint32_t result)
{
    if (g_lab_stlink_trace.magic != LAB_STLINK_TRACE_MAGIC) {
        return;
    }
    g_lab_stlink_trace.final_state = state;
    g_lab_stlink_trace.final_result = result;
}
