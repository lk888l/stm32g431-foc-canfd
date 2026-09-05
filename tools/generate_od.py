#!/usr/bin/env python3
"""Single source for CANopenNode OD, EDS and supported-object reference. Offline."""
from pathlib import Path
import argparse

ROOT = Path(__file__).resolve().parents[1]
TYPES = {
    "u8": ("uint8_t", 1, 5),
    "i8": ("int8_t", 1, 2),
    "u16": ("uint16_t", 2, 6),
    "i16": ("int16_t", 2, 3),
    "u32": ("uint32_t", 4, 7),
    "i32": ("int32_t", 4, 4),
    "f32": ("float", 4, 8),
}
objects = []


def item(sub, name, typ, default=0, access="rw", pdo=False):
    return dict(sub=sub, name=name, typ=typ, default=default, access=access, pdo=pdo)


def var(index, name, typ, default=0, access="rw", pdo=False):
    objects.append((index, name, [item(0, name, typ, default, access, pdo)], False))


def rec(index, name, entries):
    objects.append(
        (
            index,
            name,
            [item(0, "count", "u8", max(e["sub"] for e in entries), "ro")] + entries,
            True,
        )
    )


var(0x1000, "device_type", "u32", 0x20192, "ro")
var(0x1001, "error_register", "u8", 0, "ro")
rec(0x1003, "error_history", [item(i, f"error_{i}", "u32", 0, "ro") for i in range(1, 9)])
objects[-1][2][0].update(default=0, access="rw")
var(0x1005, "sync_cob_id", "u32", 0x80)
var(0x1006, "sync_period_us", "u32", 1000)
var(0x1007, "sync_window_us", "u32", 0)
rec(0x1010, "store", [item(1, "all", "u32", 1)])
rec(0x1011, "restore", [item(1, "all", "u32", 1)])
var(0x1014, "emcy_cob_id", "u32", 0x80)
var(0x1015, "emcy_inhibit_100us", "u16", 100)
rec(0x1016, "heartbeat_consumers", [item(1, "master", "u32", 0)])
var(0x1017, "heartbeat_ms", "u16", 1000)
rec(
    0x1018,
    "identity",
    [
        item(1, "vendor", "u32", 0, "ro"),
        item(2, "product", "u32", 0x431, "ro"),
        item(3, "revision", "u32", 0x10000, "ro"),
        item(4, "serial", "u32", 0, "ro"),
    ],
)
rec(0x1200, "sdo", [item(1, "rx", "u32", 0x601, "ro"), item(2, "tx", "u32", 0x581, "ro")])
rpdo = [
    [0x60FF0020, 0x60400010, 0x60600008],
    [0x607A0020, 0x60400010, 0x60600008],
    [0x60710010, 0x60400010, 0x60600008],
    [0x20040220, 0x20040320],
]
tpdo = [
    [0x603F0010, 0x60410010, 0x60610008],
    [0x60640020, 0x606C0020],
    [0x60770010, 0x22020020],
    [],
]
for i in range(4):
    rec(
        0x1400 + i,
        f"rpdo{i+1}",
        [
            item(1, "cob_id", "u32", 0x80000200 + i * 0x100),
            item(2, "type", "u8", 255),
            item(5, "timeout_ms", "u16", 0),
        ],
    )
for base, maps in [(0x1600, rpdo), (0x1A00, tpdo)]:
    for i, mapping in enumerate(maps):
        rec(
            base + i,
            f"map_{base+i:04x}",
            [
                item(j, f"entry_{j}", "u32", mapping[j - 1] if j <= len(mapping) else 0)
                for j in range(1, 9)
            ],
        )
        objects[-1][2][0].update(default=len(mapping), access="rw")
for i in range(4):
    rec(
        0x1800 + i,
        f"tpdo{i+1}",
        [
            item(1, "cob_id", "u32", 0xC0000180 + i * 0x100),
            item(2, "type", "u8", 1),
            item(3, "inhibit_100us", "u16", 0),
            item(4, "reserved", "u8", 0, "ro"),
            item(5, "event_ms", "u16", 0),
        ],
    )
rec(
    0x2001,
    "communication",
    [
        item(1, "node_id", "u8", 1),
        item(2, "bitrate", "u32", 1000000, "ro"),
        item(3, "data_bitrate", "u32", 1000000, "ro"),
        item(4, "brs", "u8", 0, "ro"),
    ],
)
rec(
    0x2003,
    "mit",
    [
        item(1, "position", "f32", 0, "rw", True),
        item(2, "velocity", "f32", 0, "rw", True),
        item(3, "torque", "f32", 0, "rw", True),
        item(4, "kp", "u16", 0, "rw", True),
        item(5, "kd", "u16", 0, "rw", True),
        item(6, "limit", "u16", 1000, "rw", True),
        item(7, "scale", "f32", 0.001, "ro", True),
    ],
)
rec(
    0x2004,
    "compressed_mit",
    [
        item(1, "enable", "u8"),
        item(2, "low", "u32", 0, "rw", True),
        item(3, "high", "u32", 0, "rw", True),
    ]
    + [
        item(i, n, "f32", v)
        for i, n, v in [
            (4, "p_min", -0.5),
            (5, "p_max", 0.5),
            (6, "v_min", -10),
            (7, "v_max", 10),
            (8, "kp_min", 0),
            (9, "kp_max", 100),
            (10, "kd_min", 0),
            (11, "kd_max", 20),
            (12, "t_min", -0.05),
            (13, "t_max", 0.05),
        ]
    ]
    + [item(14, "limit", "u16", 1000)],
)
rec(
    0x2100,
    "commissioning",
    [
        item(1, "request", "u32"),
        item(2, "state", "u8", 0, "ro", True),
        item(3, "pole_pairs", "u16", 7, "ro"),
        item(4, "direction", "i8", 1, "ro"),
        item(5, "offset_rad", "f32", 0, "ro"),
        item(6, "current_limit_a", "f32", 0.25),
        item(7, "torque_constant", "f32", 60 / (6.283185307179586 * 200)),
        item(8, "command_timeout_ms", "u16", 100, "ro"),
        item(9, "mcu_temperature", "i16", 0, "ro", True),
        item(10, "isr_max_cycles", "u32", 0, "ro"),
    ],
)
for idx, name, fields in [
    (0x2200, "phase_current", ["a", "b", "c"]),
    (0x2201, "phase_voltage", ["a", "b", "c"]),
    (0x2203, "dq_current", ["d", "q"]),
]:
    rec(idx, name, [item(i, n, "i32", 0, "ro", True) for i, n in enumerate(fields, 1)])
var(0x2202, "bus_mv", "i32", 0, "ro", True)
rec(
    0x3000,
    "padding",
    [
        item(1, "byte", "u8", 0, "rw", True),
        item(2, "word", "u16", 0, "rw", True),
        item(3, "dword", "u32", 0, "rw", True),
    ],
)
for idx, name, typ, default, access, pdo in [
    (0x6007, "abort_connection", "i16", 1, "ro", False),
    (0x603F, "fault", "u16", 0, "ro", True),
    (0x6040, "controlword", "u16", 0, "rw", True),
    (0x6041, "statusword", "u16", 0x240, "ro", True),
    (0x605A, "quick_stop_option", "i16", 2, "ro", False),
    (0x605B, "shutdown_option", "i16", 0, "ro", False),
    (0x605C, "disable_option", "i16", 0, "ro", False),
    (0x605E, "fault_option", "i16", 0, "ro", False),
    (0x6060, "mode", "i8", 0, "rw", True),
    (0x6061, "mode_display", "i8", 0, "ro", True),
    (0x6064, "position", "f32", 0, "ro", True),
    (0x6065, "following_window", "f32", 0.25, "rw", False),
    (0x6066, "following_ms", "u16", 1000, "rw", False),
    (0x606C, "velocity", "f32", 0, "ro", True),
    (0x6071, "target_torque", "i16", 0, "rw", True),
    (0x6072, "max_torque", "u16", 1000, "rw", True),
    (0x6076, "peak_torque", "f32", 0.011936621, "ro", False),
    (0x6077, "actual_torque", "i16", 0, "ro", True),
    (0x607A, "target_position", "f32", 0, "rw", True),
    (0x6081, "profile_velocity", "f32", 2, "rw", True),
    (0x6083, "acceleration", "f32", 2, "rw", True),
    (0x6084, "deceleration", "f32", 2, "rw", True),
    (0x6085, "quick_deceleration", "f32", 4, "rw", False),
    (0x60FF, "target_velocity", "f32", 0, "rw", True),
]:
    var(idx, name, typ, default, access, pdo)
objects.sort()


def field(idx, e):
    return f'x{idx:04x}_{e["sub"]:02x}'


def value(e):
    if e["typ"] == "f32":
        return str(float(e["default"])) + "F"
    return str(e["default"]) + ("U" if e["typ"].startswith("u") else "")


def generate():
    h = [
        "// Generated by tools/generate_od.py. Do not edit.",
        "#ifndef ROLLER_OD_H",
        "#define ROLLER_OD_H",
        '#include "301/CO_ODinterface.h"',
        "#ifdef __cplusplus",
        'extern "C" {',
        "#endif",
    ]
    counts = {
        "NMT": 1,
        "EM": 1,
        "EM_PROD": 1,
        "HB_PROD": 1,
        "HB_CONS": 1,
        "SDO_SRV": 1,
        "SYNC": 1,
        "RPDO": 4,
        "TPDO": 4,
        "ARR_1003": 8,
        "ARR_1016": 1,
    }
    h += [f"#define OD_CNT_{k} {v}" for k, v in counts.items()]
    h += (
        ["typedef struct {"]
        + [f'    {TYPES[e["typ"]][0]} {field(idx,e)};' for idx, _, es, _ in objects for e in es]
        + [
            "} RollerOD;",
            "extern RollerOD OD_RAM;",
            "extern OD_entry_t ODList[];",
            "extern OD_t* OD;",
            "void OD_reset_defaults(void);",
            "void OD_restore_parameters(const RollerOD* saved);",
        ]
    )
    h += [f"#define OD_ENTRY_H{idx:04X} (&ODList[{n}])" for n, (idx, *_) in enumerate(objects)]
    h += ["#ifdef __cplusplus", "}", "#endif", "#endif", ""]
    c = [
        "// Generated by tools/generate_od.py. Do not edit.",
        "#define OD_DEFINITION",
        '#include "OD.h"',
    ]
    init = ",\n".join(f"    .{field(idx,e)} = {value(e)}" for idx, _, es, _ in objects for e in es)
    c += [
        "static const RollerOD defaults = {",
        init,
        "};",
        "RollerOD OD_RAM = {",
        init,
        "};",
        "void OD_reset_defaults(void) { OD_RAM = defaults; }",
    ]
    persistent = {
        0x1005,
        0x1006,
        0x1007,
        0x1014,
        0x1015,
        0x1016,
        0x1017,
        0x2001,
        0x6065,
        0x6066,
        0x6072,
        0x6081,
        0x6083,
        0x6084,
        0x6085,
    }
    c += ["void OD_restore_parameters(const RollerOD* saved) {"]
    for idx, _, es, _ in objects:
        for e in es:
            if idx in persistent or 0x1400 <= idx <= 0x1A03:
                if e["access"] == "rw":
                    c += [f"    OD_RAM.{field(idx,e)} = saved->{field(idx,e)};"]
    c += ["}"]
    for idx, _, es, record in objects:
        if record:
            c += [f"static const OD_obj_record_t obj_{idx:04x}[] = {{"]
            for e in es:
                attr = "ODA_SDO_RW" if e["access"] == "rw" else "ODA_SDO_R"
                if TYPES[e["typ"]][1] > 1:
                    attr += " | ODA_MB"
                if e["pdo"]:
                    attr += " | " + ("ODA_TRPDO" if e["access"] == "rw" else "ODA_TPDO")
                c += [f'    {{&OD_RAM.{field(idx,e)}, {e["sub"]}, {attr}, {TYPES[e["typ"]][1]}}},']
            c += ["};"]
        else:
            e = es[0]
            attr = "ODA_SDO_RW" if e["access"] == "rw" else "ODA_SDO_R"
            if TYPES[e["typ"]][1] > 1:
                attr += " | ODA_MB"
            if e["pdo"]:
                attr += " | " + ("ODA_TRPDO" if e["access"] == "rw" else "ODA_TPDO")
            c += [
                f'static const OD_obj_var_t obj_{idx:04x} = {{&OD_RAM.{field(idx,e)}, {attr}, {TYPES[e["typ"]][1]}}};'
            ]
    c += (
        ["OD_entry_t ODList[] = {"]
        + [
            f'    {{0x{idx:04X}, {len(es)}, {"ODT_REC" if record else "ODT_VAR"}, &obj_{idx:04x}, NULL}},'
            for idx, _, es, record in objects
        ]
        + [
            "};",
            f"static OD_t dictionary = {{{len(objects)}, ODList}};",
            "OD_t* OD = &dictionary;",
            "",
        ]
    )
    eds = [
        "[FileInfo]",
        "FileName=rollercan.eds",
        "FileVersion=1",
        "FileRevision=0",
        "EDSVersion=4.0",
        "Description=RollerCAN C++23 CANopen classic / HexMeow OD subset",
        "CreationTime=12:00PM",
        "CreationDate=09-05-2026",
        "CreatedBy=RollerCAN contributors",
        "[DeviceInfo]",
        "VendorName=Community firmware",
        "VendorNumber=0",
        "ProductName=RollerCAN C++",
        "ProductNumber=1073",
        "RevisionNumber=65536",
        "OrderCode=STM32G431",
        "BaudRate_10=0",
        "BaudRate_20=0",
        "BaudRate_50=0",
        "BaudRate_125=0",
        "BaudRate_250=0",
        "BaudRate_500=0",
        "BaudRate_800=0",
        "BaudRate_1000=1",
        "SimpleBootUpMaster=0",
        "SimpleBootUpSlave=1",
        "Granularity=8",
        "DynamicChannelsSupported=0",
        "GroupMessaging=0",
        "NrOfRXPDO=4",
        "NrOfTXPDO=4",
        "LSS_Supported=0",
        "[DummyUsage]",
    ]
    eds += [f"Dummy{i:04d}=0" for i in range(1, 8)]
    for name, indices in [
        ("MandatoryObjects", [0x1000, 0x1001, 0x1018]),
        (
            "OptionalObjects",
            [
                i
                for i, *_ in objects
                if i not in (0x1000, 0x1001, 0x1018) and not 0x2000 <= i < 0x6000
            ],
        ),
        ("ManufacturerObjects", [i for i, *_ in objects if 0x2000 <= i < 0x6000]),
    ]:
        eds += [f"[{name}]", f"SupportedObjects={len(indices)}"] + [
            f"{n}=0x{i:04X}" for n, i in enumerate(indices, 1)
        ]
    md = [
        "# Implemented object dictionary",
        "",
        "Generated by `tools/generate_od.py`. Float32 motion units follow HexMeow OD 0x08; this is a CANopen CC subset, not a certified CiA 402 profile.",
        "",
        "| Index:sub | Name | Type | Access | PDO | Default |",
        "|---|---|---|---|---|---|",
    ]
    for idx, name, es, record in objects:
        if record:
            eds += [
                f"[{idx:04X}]",
                f"ParameterName={name}",
                "ObjectType=0x9",
                f"SubNumber={len(es)}",
            ]
        for e in es:
            default = str(e["default"])
            if e["sub"] == 1 and (
                idx in (0x1014, 0x1200) or 0x1400 <= idx <= 0x1403 or 0x1800 <= idx <= 0x1803
            ):
                if idx == 0x1200:
                    default = "$NODEID+0x600"
                elif idx != 0x1014:
                    default = f'$NODEID+0x{e["default"]:X}'
            if idx == 0x1200 and e["sub"] == 2:
                default = "$NODEID+0x580"
            if idx == 0x1014:
                default = "$NODEID+0x80"
            eds += [
                f'[{idx:04X}sub{e["sub"]:X}]' if record else f"[{idx:04X}]",
                f'ParameterName={e["name"]}',
                "ObjectType=0x7",
                f'DataType=0x{TYPES[e["typ"]][2]:04X}',
                f'AccessType={e["access"]}',
                f"DefaultValue={default}",
                f'PDOMapping={int(e["pdo"])}',
            ]
            md += [
                f'| {idx:04X}:{e["sub"]:02X} | {name}.{e["name"]} | {e["typ"]} | {e["access"]} | {int(e["pdo"])} | {e["default"]} |'
            ]
    return {
        "Protocol/CANopen/OD.h": "\n".join(h),
        "Protocol/CANopen/OD.c": "\n".join(c),
        "config/rollercan.eds": "\n".join(eds) + "\n",
        "docs/object-dictionary.md": "\n".join(md) + "\n",
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    for name, text in generate().items():
        path = ROOT / name
        if args.check:
            if not path.exists() or path.read_text(encoding="utf8") != text:
                raise SystemExit("Stale generated file: " + name)
        else:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(text, encoding="utf8", newline="\n")
