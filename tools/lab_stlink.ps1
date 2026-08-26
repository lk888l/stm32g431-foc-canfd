[CmdletBinding()]
param(
    [ValidateSet('Flash', 'Status', 'Stop', 'Clear', 'Calibrate',
                 'Voltage', 'Velocity', 'Position', 'OpenLoop', 'Trace')]
    [string]$Action = 'Status',
    [float]$Value = 0.0,
    [float]$VoltageLimit = 0.8,
    [ValidateRange(0, 10000)]
    [int]$DurationMs = 2000,
    [bool]$WaitForCompletion = $true,
    [switch]$PreserveCalibration,
    [string]$TraceCsv,
    [string]$ElfPath,
    [string]$StLinkCli =
        'C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\ST-LINK_CLI.exe'
)

$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
if ([string]::IsNullOrWhiteSpace($ElfPath)) {
    $ElfPath = Join-Path $projectRoot 'build/firmware-lab-debug/brushless_control.elf'
}
if (-not (Test-Path -LiteralPath $ElfPath -PathType Leaf)) {
    throw "Lab ELF not found: $ElfPath"
}
if (-not (Test-Path -LiteralPath $StLinkCli -PathType Leaf)) {
    throw "ST-LINK CLI not found: $StLinkCli"
}
$nm = (Get-Command arm-none-eabi-nm -ErrorAction Stop).Source

$commandMagic = [Convert]::ToUInt32('4C414243', 16)
$statusMagic = [Convert]::ToUInt32('4C414253', 16)
$guardSalt = [Convert]::ToUInt32('A5C35A3C', 16)
$wordMask = [Convert]::ToUInt64('FFFFFFFF', 16)
$commandIds = @{
    Stop = [uint32]0
    Clear = [uint32]1
    Calibrate = [uint32]2
    Voltage = [uint32]3
    Velocity = [uint32]4
    Position = [uint32]5
    OpenLoop = [uint32]6
}

function Invoke-Checked {
    param([string[]]$Arguments)
    # Keep vendor-tool chatter visible without leaking it into a caller's
    # return value (Send-LabCommand must return only the committed sequence).
    & $StLinkCli @Arguments | Out-Host
    if ($LASTEXITCODE -ne 0) {
        throw "ST-LINK CLI failed with exit code $LASTEXITCODE"
    }
}

function Get-SymbolAddress {
    param([Parameter(Mandatory)][string]$Name)
    $match = & $nm -g --defined-only $ElfPath |
        Where-Object { $_ -match "\s$([regex]::Escape($Name))$" } |
        Select-Object -First 1
    if ([string]::IsNullOrWhiteSpace($match)) {
        throw "ELF symbol not found: $Name"
    }
    $addressText = ($match.Trim() -split '\s+')[0]
    return [Convert]::ToUInt32($addressText, 16)
}

function Get-SymbolSize {
    param([Parameter(Mandatory)][string]$Name)
    $match = & $nm -S -g --defined-only $ElfPath |
        Where-Object { $_ -match "\s$([regex]::Escape($Name))$" } |
        Select-Object -First 1
    if ([string]::IsNullOrWhiteSpace($match)) {
        throw "ELF symbol not found: $Name"
    }
    $fields = $match.Trim() -split '\s+'
    if ($fields.Count -lt 4) {
        throw "Unexpected nm output for ${Name}: $match"
    }
    return [Convert]::ToUInt32($fields[1], 16)
}

function Format-Hex32 {
    param([uint32]$Value)
    return ('0x{0:X8}' -f $Value)
}

function Format-BitNames {
    param(
        [uint32]$Value,
        [System.Collections.IDictionary]$Map,
        [string]$ZeroName = 'None'
    )
    $names = [System.Collections.Generic.List[string]]::new()
    foreach ($entry in $Map.GetEnumerator()) {
        if (($Value -band ([uint32]1 -shl $entry.Value)) -ne 0) {
            $names.Add([string]$entry.Key)
        }
    }
    $description = if ($names.Count -eq 0) { $ZeroName } else { $names -join ', ' }
    return ('{0} (0x{1:X8})' -f $description, $Value)
}

function Format-EnumName {
    param(
        [uint32]$Value,
        [System.Collections.IDictionary]$Map
    )
    $name = $Map[[int]$Value]
    if ([string]::IsNullOrWhiteSpace($name)) { $name = 'Unknown' }
    return ('{0} ({1})' -f $name, $Value)
}

function Get-FloatBits {
    param([float]$Number)
    return [BitConverter]::ToUInt32([BitConverter]::GetBytes($Number), 0)
}

function Get-CommandGuard {
    param(
        [uint32]$Command,
        [uint32]$Duration,
        [uint32]$ValueBits,
        [uint32]$VoltageBits,
        [uint32]$Sequence
    )
    $guard = [uint64]$commandMagic -bxor [uint64]$Command
    $guard = $guard -bxor [uint64]$Duration
    $guard = $guard -bxor [uint64]$ValueBits
    $guard = $guard -bxor [uint64]$VoltageBits
    $guard = $guard -bxor [uint64]$Sequence
    $guard = $guard -bxor [uint64]$guardSalt
    return [uint32]($guard -band $wordMask)
}

function Send-LabCommand {
    param(
        [Parameter(Mandatory)][string]$Name,
        [float]$CommandValue = 0.0,
        [float]$CommandVoltageLimit = 0.8,
        [uint32]$CommandDurationMs = 0
    )
    $mailbox = Get-SymbolAddress 'g_lab_stlink_command'
    $command = $commandIds[$Name]
    $valueBits = Get-FloatBits $CommandValue
    $voltageBits = Get-FloatBits $CommandVoltageLimit
    $milliseconds = [uint64][DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds()
    $sequence = [uint32]($milliseconds -band $wordMask)
    if ($sequence -eq 0) { $sequence = 1 }
    $guard = Get-CommandGuard $command $CommandDurationMs $valueBits `
        $voltageBits $sequence

    # Hold the core at reset while writing the mailbox. HotPlug writes can
    # pause the GPIO-I2C backend halfway through a byte and leave the AS5600
    # driving SDA low. A complete guarded command is retained across this
    # narrow PINRST/SFTRST boot class and consumed once before any gate arm.
    $arguments = @('-c', 'SWD', 'Freq=1000', 'UR')
    $words = @(
        @([uint32]0, $commandMagic),
        @([uint32]4, $command),
        @([uint32]8, $CommandDurationMs),
        @([uint32]12, $valueBits),
        @([uint32]16, $voltageBits),
        @([uint32]20, $guard),
        @([uint32]24, $sequence)
    )
    foreach ($word in $words) {
        $arguments += '-w32'
        $arguments += Format-Hex32 ([uint32]($mailbox + $word[0]))
        $arguments += Format-Hex32 ([uint32]$word[1])
    }
    $arguments += '-Run'
    Invoke-Checked $arguments
    return $sequence
}

function Read-LabStatus {
    $statusAddress = Get-SymbolAddress 'g_lab_stlink_status'
    $statusSize = Get-SymbolSize 'g_lab_stlink_status'
    if ($statusSize -lt 168) {
        throw "Lab status ABI is too old: $statusSize bytes"
    }
    # ST-LINK_CLI prompts forever when the dump target already exists.  Use a
    # unique path without pre-creating the file so status reads stay unattended.
    $temporary = Join-Path ([System.IO.Path]::GetTempPath()) (
        'brushless-lab-status-{0}.bin' -f [System.Guid]::NewGuid().ToString('N'))
    try {
        Invoke-Checked @('-c', 'SWD', 'Freq=1000', 'HOTPLUG',
                         '-Dump', (Format-Hex32 $statusAddress),
                         ([string]$statusSize),
                         $temporary, '-Run')
        $bytes = [System.IO.File]::ReadAllBytes($temporary)
        if ($bytes.Length -lt 168) {
            throw "Short status read: $($bytes.Length) bytes"
        }
        $u32 = {
            param([int]$Offset)
            [BitConverter]::ToUInt32($bytes, $Offset)
        }
        $f32 = {
            param([int]$Offset)
            [BitConverter]::ToSingle($bytes, $Offset)
        }
        $magic = & $u32 0
        if ($magic -ne $statusMagic) {
            throw ('Invalid lab status magic 0x{0:X8}; firmware may not be running' -f $magic)
        }
        $version = & $u32 4
        if ($version -notin @(3, 4)) {
            throw "Unsupported lab status ABI version: $version"
        }
        $flags = & $u32 24
        $flagNames = [System.Collections.Generic.List[string]]::new()
        $flagMap = [ordered]@{
            EncoderValid = 0
            ElectricalCalibrated = 1
            GateArmed = 2
            OutputEnabled = 3
            GateFaultActive = 4
            CurrentFeedbackValid = 5
            ThreePwmProfile = 6
            CurrentLoopInterfaceAvailable = 7
            CanEnabled = 8
            WeakMagnetAccepted = 9
            EncoderBusQuiesced = 10
            OutputWasEnabled = 11
            EncoderSclHigh = 12
            EncoderSdaHigh = 13
            EncoderDriverInitialized = 14
        }
        foreach ($entry in $flagMap.GetEnumerator()) {
            if (($flags -band ([uint32]1 -shl $entry.Value)) -ne 0) {
                $flagNames.Add($entry.Key)
            }
        }
        $stateMap = [ordered]@{
            0 = 'Idle'
            1 = 'WaitEncoder'
            2 = 'AlignZero'
            3 = 'AlignQuadrature'
            4 = 'AlignReturn'
            5 = 'RunVoltage'
            6 = 'RunVelocity'
            7 = 'RunPosition'
            8 = 'Complete'
            9 = 'Fault'
            10 = 'RunOpenLoop'
        }
        $resultMap = [ordered]@{
            0 = 'None'
            1 = 'Running'
            2 = 'OK'
            3 = 'InvalidRequest'
            4 = 'EncoderInvalid'
            5 = 'AlignmentNoMotion'
            6 = 'SafetyFault'
            7 = 'Timeout'
            8 = 'AlignmentReturnError'
            9 = 'AlignmentInconsistent'
        }
        $encoderHealthMap = [ordered]@{
            NoMagnet = 0
            MagnetWeak = 1
            MagnetStrong = 2
            Overspeed = 3
            Crc = 4
            Bus = 5
            Stale = 6
            ReservedStatus = 7
        }
        $driveFaultMap = [ordered]@{
            PhaseOvercurrent = 0
            DcOvervoltage = 1
            DcUndervoltage = 2
            PowerStageOvertemp = 3
            MotorOvertemp = 4
            GateDriver = 5
            CurrentFeedback = 6
            PositionFeedback = 7
            Overspeed = 8
            ControlDeadline = 9
            GateInterlock = 10
            CanBusOff = 11
            CommandSoftTimeout = 12
            CommandHardTimeout = 13
            Protocol = 14
            StoActive = 15
        }
        $state = & $u32 16
        $result = & $u32 20
        $publishSequence = & $u32 8
        if (($publishSequence -band 1) -ne 0) {
            throw 'Status update was interrupted; retry the Status action'
        }
        $encoderHealth = & $u32 36
        $activeFaults = & $u32 40
        $latchedFaults = & $u32 44
        [pscustomobject]@{
            Version = $version
            PublishSequence = $publishSequence
            AppliedCommandSequence = & $u32 12
            State = Format-EnumName $state $stateMap
            Result = Format-EnumName $result $resultMap
            Flags = $flagNames -join ', '
            TimestampUs = & $u32 28
            EncoderRaw = & $u32 32
            EncoderHealth = Format-BitNames $encoderHealth $encoderHealthMap 'OK'
            ActiveFaults = Format-BitNames $activeFaults $driveFaultMap
            LatchedFaults = Format-BitNames $latchedFaults $driveFaultMap
            CurrentLoopCount = & $u32 48
            SupervisorLoopCount = & $u32 52
            RawAngleRad = & $f32 56
            PositionRad = & $f32 60
            VelocityRadS = & $f32 64
            EncoderDirection = & $f32 68
            ElectricalZeroRad = & $f32 72
            AlignmentDeltaRad = & $f32 76
            AlphaVoltageV = & $f32 80
            BetaVoltageV = & $f32 84
            QVoltageV = & $f32 88
            DutyA = & $f32 92
            DutyB = & $f32 96
            DutyC = & $f32 100
            BusVoltageV = & $f32 104
            EncoderStatusRaw = ('0x{0:X2}' -f (& $u32 108))
            EncoderAgc = & $u32 112
            EncoderMagnitude = & $u32 116
            EncoderAgcRange = ('{0}..{1}' -f (& $u32 120), (& $u32 124))
            EncoderMagnitudeRange = ('{0}..{1}' -f (& $u32 128), (& $u32 132))
            EncoderAngleReadCount = & $u32 136
            EncoderDiagnosticReadCount = & $u32 140
            EncoderBusErrorCount = & $u32 144
            EncoderBusRecoveryCount = & $u32 148
            EncoderBusFailureCount = & $u32 152
            EncoderDiagnosticFailureCount = & $u32 156
            EncoderAgeUs = & $u32 160
            EncoderMaximumAgeUs = & $u32 164
            EncoderAngleGlitchCount = if (($version -ge 4) -and
                                           ($bytes.Length -ge 176)) {
                & $u32 168
            } else { 0 }
            EncoderAngleGlitchRecoveryCount = if (($version -ge 4) -and
                                                   ($bytes.Length -ge 176)) {
                & $u32 172
            } else { 0 }
        }
    }
    finally {
        Remove-Item -LiteralPath $temporary -Force -ErrorAction SilentlyContinue
    }
}

function Read-LabTrace {
    $traceAddress = Get-SymbolAddress 'g_lab_stlink_trace'
    $traceSize = Get-SymbolSize 'g_lab_stlink_trace'
    $traceMagic = [Convert]::ToUInt32('4C545243', 16)
    $highBit = [Convert]::ToUInt32('80000000', 16)
    $temporary = Join-Path ([System.IO.Path]::GetTempPath()) (
        'brushless-lab-trace-{0}.bin' -f [System.Guid]::NewGuid().ToString('N'))
    try {
        Invoke-Checked @('-c', 'SWD', 'Freq=1000', 'HOTPLUG',
                         '-Dump', (Format-Hex32 $traceAddress),
                         ([string]$traceSize), $temporary, '-Run')
        $bytes = [System.IO.File]::ReadAllBytes($temporary)
        if ($bytes.Length -lt 40) {
            throw "Short trace read: $($bytes.Length) bytes"
        }
        $u32 = {
            param([int]$Offset)
            [BitConverter]::ToUInt32($bytes, $Offset)
        }
        $f32 = {
            param([int]$Offset)
            [BitConverter]::ToSingle($bytes, $Offset)
        }
        $magic = & $u32 0
        if ($magic -ne $traceMagic) {
            throw ('Invalid lab trace magic 0x{0:X8}; no command trace is available' -f $magic)
        }
        $version = & $u32 4
        if (($version -ne 1) -and ($version -ne 2) -and
            ($version -ne 3)) {
            throw "Unsupported lab trace ABI version: $version"
        }
        $sampleSize = if ($version -eq 1) { 32 } else { 48 }
        $publishSequence = & $u32 8
        if (($publishSequence -band 1) -ne 0) {
            throw 'Trace update was interrupted; retry the Trace action'
        }
        $count = & $u32 16
        $capacity = & $u32 20
        if (($count -gt $capacity) -or
            ((40 + ([uint64]$capacity * $sampleSize)) -gt $bytes.Length)) {
            throw "Invalid trace bounds: count=$count capacity=$capacity size=$($bytes.Length)"
        }
        $oldestIndex = if (($version -ge 3) -and
                           ($count -eq $capacity) -and ($count -gt 0)) {
            & $u32 36
        } else { 0 }
        if (($count -gt 0) -and ($oldestIndex -ge $capacity)) {
            throw "Invalid trace ring index: oldest=$oldestIndex capacity=$capacity"
        }
        $firstOffset = 40 + ($sampleSize * $oldestIndex)
        $firstTimestamp = if ($count -gt 0) { & $u32 $firstOffset } else { 0 }
        $samples = [System.Collections.Generic.List[object]]::new()
        for ($index = 0; $index -lt $count; ++$index) {
            $physicalIndex = if ($capacity -gt 0) {
                ($oldestIndex + $index) % $capacity
            } else { 0 }
            $offset = 40 + ($sampleSize * $physicalIndex)
            $timestamp = & $u32 $offset
            $stateResult = & $u32 ($offset + 4)
            $encoderSample = & $u32 ($offset + 8)
            $diagnostics = & $u32 ($offset + 12)
            $elapsedUs = if ($timestamp -ge $firstTimestamp) {
                [uint64]$timestamp - [uint64]$firstTimestamp
            } else {
                [uint64]$timestamp + [uint64]4294967296 - [uint64]$firstTimestamp
            }
            $samples.Add([pscustomobject]@{
                Index = $index
                ElapsedMs = [double]$elapsedUs / 1000.0
                TimestampUs = $timestamp
                State = $stateResult -band 0xFFFF
                Result = ($stateResult -shr 16) -band 0xFFFF
                EncoderRaw = $encoderSample -band 0xFFFF
                EncoderHealth = ($encoderSample -shr 16) -band 0xFF
                EncoderValid = ($encoderSample -band $highBit) -ne 0
                EncoderStatusRaw = $diagnostics -band 0xFF
                EncoderAgc = ($diagnostics -shr 8) -band 0xFF
                EncoderMagnitude = ($diagnostics -shr 16) -band 0xFFF
                EncoderDiagnosticsValid = ($diagnostics -band $highBit) -ne 0
                PositionRad = & $f32 ($offset + 16)
                VelocityRadS = & $f32 ($offset + 20)
                Reference = & $f32 ($offset + 24)
                VoltageV = & $f32 ($offset + 28)
                EncoderAgeUs = if ($version -ge 2) {
                    & $u32 ($offset + 32)
                } else { 0 }
                EncoderAngleReadCount = if ($version -ge 2) {
                    & $u32 ($offset + 36)
                } else { 0 }
                EncoderBusErrorCount = if ($version -ge 2) {
                    & $u32 ($offset + 40)
                } else { 0 }
                EncoderBusFailureCount = if ($version -ge 2) {
                    & $u32 ($offset + 44)
                } else { 0 }
            })
        }

        if (-not [string]::IsNullOrWhiteSpace($TraceCsv)) {
            $csvPath = if ([System.IO.Path]::IsPathRooted($TraceCsv)) {
                $TraceCsv
            } else {
                Join-Path $projectRoot $TraceCsv
            }
            $csvDirectory = Split-Path $csvPath -Parent
            if (-not [string]::IsNullOrWhiteSpace($csvDirectory)) {
                New-Item -ItemType Directory -Path $csvDirectory -Force | Out-Null
            }
            $samples | Export-Csv -LiteralPath $csvPath -NoTypeInformation -Encoding utf8
            Write-Host "Trace CSV: $csvPath"
        }

        $positionStats = $samples | Measure-Object PositionRad -Minimum -Maximum
        $velocityStats = $samples | Measure-Object VelocityRadS -Minimum -Maximum
        $agcStats = $samples | Where-Object EncoderDiagnosticsValid |
            Measure-Object EncoderAgc -Minimum -Maximum
        $magnitudeStats = $samples | Where-Object EncoderDiagnosticsValid |
            Measure-Object EncoderMagnitude -Minimum -Maximum
        $encoderAgeStats = $samples | Measure-Object EncoderAgeUs -Maximum
        $maximumAbsVelocity = ($samples | ForEach-Object {
            [Math]::Abs($_.VelocityRadS)
        } | Measure-Object -Maximum).Maximum
        $maximumAbsVoltage = ($samples | ForEach-Object {
            [Math]::Abs($_.VoltageV)
        } | Measure-Object -Maximum).Maximum
        $durationMs = if ($samples.Count -gt 0) {
            $samples[$samples.Count - 1].ElapsedMs
        } else { 0.0 }
        $startPosition = if ($samples.Count -gt 0) {
            $samples[0].PositionRad
        } else { 0.0 }
        $endPosition = if ($samples.Count -gt 0) {
            $samples[$samples.Count - 1].PositionRad
        } else { 0.0 }
        $angleReadDelta = if (($version -ge 2) -and
                              ($samples.Count -gt 1)) {
            [uint32]($samples[$samples.Count - 1].EncoderAngleReadCount -
                     $samples[0].EncoderAngleReadCount)
        } else { 0 }
        $busErrorDelta = if (($version -ge 2) -and
                             ($samples.Count -gt 1)) {
            [uint32]($samples[$samples.Count - 1].EncoderBusErrorCount -
                     $samples[0].EncoderBusErrorCount)
        } else { 0 }
        $busFailureDelta = if (($version -ge 2) -and
                               ($samples.Count -gt 1)) {
            [uint32]($samples[$samples.Count - 1].EncoderBusFailureCount -
                     $samples[0].EncoderBusFailureCount)
        } else { 0 }
        return [pscustomobject]@{
            Summary = [pscustomobject]@{
                Version = $version
                CommandSequence = & $u32 12
                Samples = $count
                Capacity = $capacity
                SamplePeriodUs = & $u32 24
                FinalState = & $u32 28
                FinalResult = & $u32 32
                DurationMs = $durationMs
                StartPositionRad = $startPosition
                EndPositionRad = $endPosition
                PositionChangeRad = $endPosition - $startPosition
                PositionRangeRad = ('{0:F6}..{1:F6}' -f $positionStats.Minimum, $positionStats.Maximum)
                VelocityRangeRadS = ('{0:F6}..{1:F6}' -f $velocityStats.Minimum, $velocityStats.Maximum)
                MaximumAbsVelocityRadS = $maximumAbsVelocity
                MaximumAbsVoltageV = $maximumAbsVoltage
                EncoderInvalidSamples = @($samples | Where-Object { -not $_.EncoderValid }).Count
                EncoderNonzeroHealthSamples = @($samples | Where-Object { $_.EncoderHealth -ne 0 }).Count
                MaximumEncoderAgeUs = $encoderAgeStats.Maximum
                EncoderAngleReadsDelta = $angleReadDelta
                EncoderBusErrorsDelta = $busErrorDelta
                EncoderBusFailuresDelta = $busFailureDelta
                EncoderAgcRange = ('{0}..{1}' -f $agcStats.Minimum, $agcStats.Maximum)
                EncoderMagnitudeRange = ('{0}..{1}' -f $magnitudeStats.Minimum, $magnitudeStats.Maximum)
            }
            Samples = $samples
        }
    }
    finally {
        Remove-Item -LiteralPath $temporary -Force -ErrorAction SilentlyContinue
    }
}

if ($Action -eq 'Flash') {
    $flashPath = [System.IO.Path]::ChangeExtension($ElfPath, '.hex')
    if (-not (Test-Path -LiteralPath $flashPath -PathType Leaf)) {
        throw "Lab HEX not found: $flashPath"
    }
    $mailboxSequenceAddress = [uint32](
        (Get-SymbolAddress 'g_lab_stlink_command') + [uint32]24)
    $calibrationMagicAddress = Get-SymbolAddress 'g_lab_stlink_calibration'
    $traceMagicAddress = Get-SymbolAddress 'g_lab_stlink_trace'
    # Reflashing always invalidates the one-shot command and old trace.
    # Calibration is cleared by default because a firmware change may alter
    # phase/angle conventions. Gain-only tuning can explicitly retain the
    # guarded NOLOAD record after its zero has already been verified.
    $flashArguments = @('-c', 'SWD', 'Freq=1000', 'UR',
                        '-P', $flashPath, '-V', 'after_programming',
                        '-w32', (Format-Hex32 $mailboxSequenceAddress),
                        '0x00000000')
    if (-not $PreserveCalibration) {
        $flashArguments += @('-w32',
                             (Format-Hex32 $calibrationMagicAddress),
                             '0x00000000')
    }
    $flashArguments += @('-w32', (Format-Hex32 $traceMagicAddress),
                         '0x00000000', '-Rst', '-Run')
    Invoke-Checked $flashArguments
    Start-Sleep -Milliseconds 500
    Read-LabStatus | Format-List
    exit
}

if ($Action -eq 'Status') {
    Read-LabStatus | Format-List
    exit
}

if ($Action -eq 'Trace') {
    $trace = Read-LabTrace
    $trace.Summary | Format-List
    exit
}

if ($Action -in @('Calibrate', 'Voltage', 'Velocity', 'Position', 'OpenLoop')) {
    [void](Send-LabCommand -Name Clear -CommandVoltageLimit $VoltageLimit)
    Start-Sleep -Milliseconds 200
}

$accepted = Send-LabCommand -Name $Action -CommandValue $Value `
    -CommandVoltageLimit $VoltageLimit -CommandDurationMs ([uint32]$DurationMs)
Write-Host ('Committed {0} command sequence 0x{1:X8}' -f $Action, $accepted)

if ($WaitForCompletion) {
    $waitMs = switch ($Action) {
        # 100 ms post-reset settle + 1 s encoder wait + two 3 s passes,
        # with margin. Never read status while the slowest valid alignment
        # path can still have the gate armed.
        'Calibrate' { 7700 }
        'Clear' { 250 }
        'Stop' { 150 }
        default { $DurationMs + 350 }
    }
    Start-Sleep -Milliseconds $waitMs
    Read-LabStatus | Format-List
    if ($Action -in @('Calibrate', 'Voltage', 'Velocity',
                      'Position', 'OpenLoop')) {
        $trace = Read-LabTrace
        $trace.Summary | Format-List
    }
}
