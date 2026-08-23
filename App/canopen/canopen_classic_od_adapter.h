#ifndef BRUSHLESS_CANOPEN_CLASSIC_OD_ADAPTER_H
#define BRUSHLESS_CANOPEN_CLASSIC_OD_ADAPTER_H

#include <stdbool.h>
#include <stdint.h>

#include "301/CO_ODinterface.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Attach CANopenNode OD IO extensions to the application's validated CiA 402
 * byte-access API.  Must run before CO_CANopenInitPDO(). */
bool canopen_classic_od_bind(OD_t *od);

/* Bracket CO_process_RPDO().  While active, the fixed RPDO2 mapping
 * (0x607A + 0x60FF) is staged and committed atomically instead of exposing
 * CANopenNode's sequential per-object writes to the control application.
 * The end function returns the number of rejected/partial RPDO2 records. */
void canopen_classic_od_rpdo_process_begin(void);
uint32_t canopen_classic_od_rpdo_process_end(void);

/* Gate nonzero controlword writes.  The integration layer grants this only
 * while local NMT is Operational and the configured supervisor heartbeat is
 * active and reports remote NMT Operational. */
void canopen_classic_od_set_control_permission(bool permitted);

/* Clear 0x6040 through the same validated application path.  Used whenever
 * NMT leaves Operational so a stale controlword cannot keep torque enabled. */
bool canopen_classic_od_force_disable(void);

#ifdef __cplusplus
}
#endif

#endif /* BRUSHLESS_CANOPEN_CLASSIC_OD_ADAPTER_H */
