# CANopenNode classic-CAN integration

This directory contains a CANopenNode v4 **CiA 301 classic CAN** port for the
STM32G431 FDCAN peripheral.  Every CANopen frame uses an 11-bit identifier and
0..8 data bytes with `FDCAN_CLASSIC_CAN` and BRS disabled.  It is not a CiA 1301
CANopen FD implementation: it has no USDO service and must not be advertised or
tested as CANopen FD.

The same FDCAN instance is shared with the project's manufacturer-specific
extended-ID CAN-FD MIT data plane.  Consequently, every physical node on a bus
where those FD frames are transmitted must be CAN-FD tolerant.  A classical-CAN
only controller can disrupt that mixed bus even though the CANopen frames are
classical.

## Implemented services

- NMT slave and boot-up message
- EMCY producer and eight-entry error history
- producer heartbeat at 100 ms
- one heartbeat consumer entry (`0x1016:1`, disabled by default with value zero)
- one expedited/segmented SDO server
- two fixed asynchronous RPDOs and two fixed asynchronous TPDOs
- CiA 402 OD access delegated to `app_cia402_od_read()` and
  `app_cia402_od_write()` so the application's range and state validation is
  not bypassed

CANopen torque permission is fail-safe.  A nonzero `0x6040` write is accepted
only when all three conditions hold:

1. this node is NMT Operational;
2. `0x1016:1` contains a nonzero heartbeat-consumer configuration; and
3. the first monitored heartbeat has arrived, has not timed out, and reports
   the remote producer in NMT Operational.

The default `0x1016:1` value is zero, so an unconfigured CANopen master cannot
enable the drive.  Configure it at runtime with
`(producer_node_id << 16) | timeout_ms`; this runtime value is not claimed to be
nonvolatile.  `canopen_classic_status()` exposes `heartbeat_configured` and
`heartbeat_active`.  Losing a permission which had become active clears the
application controlword, and heartbeat timeout or remote reset revokes it in
the same CANopen processing pass.  CANopenNode also moves local NMT out of
Operational through `CO_NMT_ERR_ON_BUSOFF_HB`.

This gate is applied only to writes arriving through the classic-CAN CANopen OD
adapter.  The manufacturer CAN-FD MIT command path retains its independent
changed-sequence and 5/20 ms watchdog semantics; merely leaving `0x1016`
unconfigured does not periodically clear that vendor path.  Heartbeat
supervision does not replace the command watchdog or hardware break/current
protection.

## Default communication objects

Node ID is supplied to `canopen_classic_init()`; the board default is 1.  The
nominal bit rate is fixed at 500 kbit/s by the BSP.

| Object | COB-ID | Fixed mapping | Period |
|---|---:|---|---:|
| RPDO1 | `0x200 + node` | `6040:00` 16b | on reception |
| RPDO2 | `0x300 + node` | `607A:00` 32b, `60FF:00` 32b | on reception |
| TPDO1 | `0x180 + node` | `6041:00` 16b, `6061:00` 8b, `603F:00` 16b | 10 ms |
| TPDO2 | `0x280 + node` | `6064:00` 32b, `606C:00` 32b | 20 ms |

All multi-byte values are CANopen little-endian.  PDO communication and mapping
objects are intentionally read-only at runtime; change them in the released
OD/EDS and repeat integration and conformance testing instead of remapping a
running drive.

RPDO1 intentionally contains only the controlword.  CANopenNode invokes the OD
write callback for each mapped RPDO object separately and ignores a later
callback error, so placing mode/limit fields after `0x6040` could partially
apply an enable command.  Set `0x6060` by SDO while the drive is disabled;
`0x6071` is also SDO-only in this fixed OD and remains subject to the
application's range checks.  RPDO2 may contain position and velocity together
because the active mode consumes only its corresponding target and neither
field enables torque.

The manufacturer-specific MIT record is `0x2000`:

1. target position, Q16.16 rad (`i32`)
2. target velocity, Q16.16 rad/s (`i32`)
3. feed-forward torque, Q16.16 N m (`i32`)
4. Kp, UQ8.8 (`u16`)
5. Kd, UQ8.8 (`u16`)
6. current limit, UQ8.8 A (`u16`)
7. sequence (`u16`)

For SDO updates, write sub-index 7 last; the control application uses a changed
sequence as the atomic commit.  High-rate MIT operation belongs on the existing
vendor CAN-FD command frame, not on a sequence of SDO transfers.

## Drive fault EMCY bridge

The stack cannot infer FOC/PDS faults merely from the presence of EMCY objects.
After the application has published the same cycle's `0x603F`, it must call:

```c
canopen_classic_report_drive_faults(effective_fault_mask,
                                    cia402_error_code);
```

The bridge uses manufacturer error-status bit `0x40`.  A nonzero fault mask
sets CANopen error-register (`0x1001`) manufacturer bit `0x80`, adds an entry to
`0x1003`, and produces EMCY `0x080 + node`.  The EMCY error code is the supplied
CiA 402 `0x603F` code; if the caller supplies zero for a nonzero fault mask, the
fallback is device-specific `0xFF00`.  EMCY bytes 4..7 contain the full 32-bit
effective fault mask in CANopen little-endian order.

Clearing the final fault emits the corresponding EMCY reset.  If a nonzero
mask or its `0x603F` code changes, the aggregate bit is reset and re-reported so
the new EMCY carries current rather than first-fault-only information.  The
current bridged mask is available as `reported_drive_faults` in
`canopen_classic_status()`.  This is intentionally one aggregate manufacturer
condition because the compact target reserves no stable CANopen error bit per
application fault.

The manufacturer fault bit is intentionally not an automatic NMT-stop mask.
The CiA 402 PDS fault state already removes torque, while keeping NMT
Operational lets a supervised master read diagnostics and issue the CiA 402
fault-reset controlword.  Bus-off and heartbeat-consumer failures still force
the local node out of Operational and revoke control permission.

## Integration calls

After the BSP has initialized FDCAN and the application OD exists:

```c
canopen_classic_init(&hfdcan1,
                     BOARD_CANOPEN_NODE_ID,
                     BOARD_CAN_NOMINAL_BITRATE / 1000U);
```

FDCAN must be started before processing.  Call
`canopen_classic_background(elapsed_us)` from the main loop and
`canopen_classic_process_1ms()` from a deferred 1 kHz task or low-priority
PendSV bottom half.  Do not call the latter inline from the 20 kHz
ADC/current-control interrupt.  The existing FDCAN FIFO0 callback passes
classic frames to the strong
`canopen_classic_receive_isr()` entry; the vendor FD frames stay on their
separate decoder.  Call `canopen_classic_report_drive_faults()` after the
supervisor publishes `0x603F`; CANopenNode's EMCY lock makes this short bridge
safe from the supervisor context, while actual EMCY transmission remains in
`canopen_classic_background()`.

## CMake sources

Compile these project sources:

```text
App/canopen/CO_driver.c
App/canopen/OD.c
App/canopen/canopen_classic_od_adapter.c
App/canopen/canopen_classic.c
```

and these CANopenNode sources:

```text
Middlewares/Third_Party/CANopenNode/CANopen.c
Middlewares/Third_Party/CANopenNode/301/CO_ODinterface.c
Middlewares/Third_Party/CANopenNode/301/CO_Emergency.c
Middlewares/Third_Party/CANopenNode/301/CO_NMT_Heartbeat.c
Middlewares/Third_Party/CANopenNode/301/CO_HBconsumer.c
Middlewares/Third_Party/CANopenNode/301/CO_SDOserver.c
Middlewares/Third_Party/CANopenNode/301/CO_PDO.c
Middlewares/Third_Party/CANopenNode/301/CO_fifo.c
```

Place `App/canopen` before other candidate OD/driver directories in include
paths, then include the CANopenNode root and `App/protocol`.  The upstream
`CO_fifo.c` has an unused `crc` parameter in this reduced configuration, so a
project that promotes all warnings to errors should apply
`-Wno-error=unused-parameter` to third-party CANopenNode sources only.

The port and selected stack sources pass ARM GCC 14.2 C11 syntax/type checking.
This is implementation evidence, not CiA conformance evidence.  Release still
requires bus-load analysis, fault injection, EDS/OD consistency checks, and a
CANopen conformance test campaign on the exact hardware and transceiver.
