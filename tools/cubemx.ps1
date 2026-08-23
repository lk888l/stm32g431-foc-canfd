[CmdletBinding()]
param(
    [ValidateSet('create-ioc', 'configure-ioc', 'validate', 'open', 'generate', 'install-g4')]
    [string]$Action = 'open',
    [string]$CubeMxHome = 'C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeMX'
)

$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$jar = Join-Path $CubeMxHome 'STM32CubeMX.exe'
$java = Join-Path $CubeMxHome 'jre/bin/java.exe'
$ioc = Join-Path $projectRoot 'brushless_control.ioc'

if (-not (Test-Path -LiteralPath $jar -PathType Leaf)) {
    throw "STM32CubeMX was not found at: $jar"
}
if (-not (Test-Path -LiteralPath $java -PathType Leaf)) {
    throw "Bundled Java runtime was not found at: $java"
}

function ConvertTo-CubeMxPath {
    param([Parameter(Mandatory)][string]$Path)
    return $Path.Replace('\', '/')
}

function New-CubeMxScript {
    param(
        [Parameter(Mandatory)][string]$TemplatePath,
        [Parameter(Mandatory)][hashtable]$Replacements
    )

    $content = Get-Content -LiteralPath $TemplatePath -Raw
    foreach ($name in $Replacements.Keys) {
        $content = $content.Replace("@$name@", [string]$Replacements[$name])
    }

    $temporaryScript = Join-Path ([System.IO.Path]::GetTempPath()) (
        'brushless-cubemx-' + [System.Guid]::NewGuid().ToString('N') + '.txt')
    [System.IO.File]::WriteAllText($temporaryScript, $content,
        [System.Text.Encoding]::ASCII)
    return $temporaryScript
}

function Invoke-CubeMxScript {
    param(
        [Parameter(Mandatory)][string]$ScriptPath,
        [Parameter(Mandatory)][string]$Description
    )

    $token = [System.Guid]::NewGuid().ToString('N')
    $stdoutPath = Join-Path ([System.IO.Path]::GetTempPath()) "brushless-cubemx-$token.out"
    $stderrPath = Join-Path ([System.IO.Path]::GetTempPath()) "brushless-cubemx-$token.err"
    try {
        $arguments = @('-jar', ('"' + $jar + '"'), '-q', ('"' + $ScriptPath + '"'))
        $process = Start-Process -FilePath $java -ArgumentList $arguments `
            -Wait -PassThru -WindowStyle Hidden `
            -RedirectStandardOutput $stdoutPath -RedirectStandardError $stderrPath

        $stdout = if (Test-Path -LiteralPath $stdoutPath) {
            Get-Content -LiteralPath $stdoutPath -Raw
        } else { '' }
        $stderr = if (Test-Path -LiteralPath $stderrPath) {
            Get-Content -LiteralPath $stderrPath -Raw
        } else { '' }
        $combined = $stdout + [Environment]::NewLine + $stderr

        if ($process.ExitCode -ne 0) {
            throw "$Description failed with exit code $($process.ExitCode).`n$combined"
        }

        $lines = @($combined -split '\r?\n')
        $commandPattern = '^\s*(load|set|project|config|check|isPluginError|clock|waitclock|SetStructure|SetCopyLibrary)\b'
        $koIndexes = for ($index = 0; $index -lt $lines.Count; $index++) {
            if ($lines[$index] -notmatch '^\s*KO\s*$') { continue }
            $previous = $index - 1
            while (($previous -ge 0) -and [string]::IsNullOrWhiteSpace($lines[$previous])) {
                $previous--
            }
            # Third-party pack startup noise in this CubeMX RC emits a bare KO
            # before the first script command. Only reject KO paired with an
            # echoed command from our script.
            if (($previous -ge 0) -and ($lines[$previous] -match $commandPattern)) {
                $index
            }
        }
        if ($koIndexes.Count -gt 0) {
            $context = foreach ($index in $koIndexes) {
                $first = [Math]::Max(0, $index - 3)
                $last = [Math]::Min($lines.Count - 1, $index + 1)
                ($lines[$first..$last] -join [Environment]::NewLine)
            }
            throw "$Description contained a rejected CubeMX command:`n$($context -join [Environment]::NewLine)"
        }

        Write-Host "$Description completed with STM32CubeMX."
        return $combined
    }
    finally {
        Remove-Item -LiteralPath $stdoutPath, $stderrPath -Force -ErrorAction SilentlyContinue
    }
}

function Read-IocEntries {
    param([Parameter(Mandatory)][string]$Path)

    $entries = [System.Collections.Generic.SortedDictionary[string,string]]::new(
        [System.StringComparer]::Ordinal)
    foreach ($line in [System.IO.File]::ReadAllLines($Path)) {
        if ([string]::IsNullOrWhiteSpace($line) -or $line.StartsWith('#')) {
            continue
        }
        $separator = $line.IndexOf('=')
        if ($separator -le 0) {
            throw "Malformed .ioc entry: $line"
        }
        $entries[$line.Substring(0, $separator)] = $line.Substring($separator + 1)
    }
    return $entries
}

function Set-ProductionIocMetadata {
    param([Parameter(Mandatory)][string]$Path)

    $entries = Read-IocEntries $Path

    # Normalize indexed collections so repeated configuration is deterministic.
    foreach ($key in @($entries.Keys)) {
        if (($key -match '^Mcu\.IP\d+$') -or
            ($key -match '^Mcu\.Pin\d+$') -or
            ($key -match '^Dma\.')) {
            [void]$entries.Remove($key)
        }
    }

    $ipNames = @('ADC1', 'ADC2', 'DMA', 'FDCAN1', 'I2C1', 'IWDG',
                 'NVIC', 'RCC', 'SPI3', 'SYS', 'TIM1', 'TIM2')
    for ($index = 0; $index -lt $ipNames.Count; $index++) {
        $entries["Mcu.IP$index"] = $ipNames[$index]
    }
    $entries['Mcu.IPNb'] = [string]$ipNames.Count

    $pinNames = @(
        'PC13', 'PF0-OSC_IN', 'PF1-OSC_OUT', 'PA1', 'PA3', 'PA7',
        'PB10', 'PB11', 'PB12', 'PB13', 'PB14', 'PB15', 'PA8', 'PA9',
        'PA10', 'PA11', 'PA12', 'PA13', 'PA14', 'PA15', 'PC10', 'PC11',
        'PB3', 'PB4', 'PB5', 'PB6', 'PB7', 'VP_IWDG_VS_IWDG',
        'VP_SYS_VS_Systick', 'VP_SYS_VS_DBSignals',
        'VP_TIM1_VS_ClockSourceINT', 'VP_TIM1_VS_no_output4',
        'VP_TIM2_VS_ClockSourceINT'
    )
    for ($index = 0; $index -lt $pinNames.Count; $index++) {
        $entries["Mcu.Pin$index"] = $pinNames[$index]
    }
    $entries['Mcu.PinsNb'] = [string]$pinNames.Count

    $fixed = [ordered]@{
        # Exact production part and project metadata.
        'Mcu.CPN' = 'STM32G431CBU6'
        'Mcu.UserName' = 'STM32G431CBUx'
        'ProjectManager.DeviceId' = 'STM32G431CBUx'
        'ProjectManager.ProjectFileName' = 'brushless_control.ioc'
        'ProjectManager.ProjectName' = 'brushless_control'
        'ProjectManager.TargetToolchain' = 'CMake'
        'ProjectManager.CompilerLinker' = 'GCC'
        'ProjectManager.CoupleFile' = 'true'
        'ProjectManager.functionlistsort' = '1-SystemClock_Config-RCC-false-HAL-false,2-MX_GPIO_Init-GPIO-false-HAL-true,3-MX_DMA_Init-DMA-false-HAL-true,4-MX_ADC1_Init-ADC1-false-HAL-true,5-MX_ADC2_Init-ADC2-false-HAL-true,6-MX_FDCAN1_Init-FDCAN1-false-HAL-true,7-MX_I2C1_Init-I2C1-false-HAL-true,8-MX_IWDG_Init-IWDG-false-HAL-true,9-MX_SPI3_Init-SPI3-false-HAL-true,10-MX_TIM1_Init-TIM1-false-HAL-true,11-MX_TIM2_Init-TIM2-false-HAL-true'

        # 8 MHz HSE, PLLM=2, PLLN=85, PLLR=2: 170 MHz SYSCLK.
        'RCC.HSE_VALUE.Ext' = '8000000'
        'RCC.OscillatorType.Ext' = 'RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSI'
        'RCC.PLLSourceVirtual' = 'RCC_PLLSOURCE_HSE'
        'RCC.PLLM' = 'RCC_PLLM_DIV2'
        'RCC.PLLN' = '85'
        'RCC.PLLP.Ext' = 'RCC_PLLP_DIV2'
        'RCC.PLLQ.Ext' = 'RCC_PLLQ_DIV2'
        'RCC.PLLR.Ext' = 'RCC_PLLR_DIV2'
        'RCC.PWR_Regulator_Voltage_Scale' = 'PWR_REGULATOR_VOLTAGE_SCALE1_BOOST'
        'RCC.SYSCLKSource' = 'RCC_SYSCLKSOURCE_PLLCLK'
        'RCC.SYSCLKSource.Ext' = 'RCC_SYSCLKSOURCE_PLLCLK'
        'RCC.SYSCLKSourceARG.Ext' = 'RCC_SYSCLKSOURCE_PLLCLK'
        'RCC.AHBCLKDivider.Ext' = 'RCC_SYSCLK_DIV1'
        'RCC.APB1CLKDivider.Ext' = 'RCC_HCLK_DIV1'
        'RCC.APB2CLKDivider.Ext' = 'RCC_HCLK_DIV1'
        'RCC.FLatency.Ext' = 'FLASH_LATENCY_4'
        'RCC.EnbaleCSS.Ext' = 'true'
        'RCC.VCOInputFreq_Value' = '4000000'
        'RCC.VCOOutputFreq_Value' = '340000000'
        'RCC.PLLPoutputFreq_Value' = '170000000'
        'RCC.PLLQoutputFreq_Value' = '170000000'
        'RCC.PLLRCLKFreq_Value' = '170000000'
        'RCC.SYSCLKFreq_VALUE' = '170000000'
        'RCC.AHBFreq_Value' = '170000000'
        'RCC.HCLKFreq_Value' = '170000000'
        'RCC.FCLKCortexFreq_Value' = '170000000'
        'RCC.CortexFreq_Value' = '170000000'
        'RCC.PWRFreq_Value' = '170000000'
        'RCC.APB1Freq_Value' = '170000000'
        'RCC.APB1TimFreq_Value' = '170000000'
        'RCC.APB2Freq_Value' = '170000000'
        'RCC.APB2TimFreq_Value' = '170000000'
        'RCC.ADC12Freq_Value.Ext' = '170000000'
        'RCC.FDCANFreq_Value' = '170000000'
        'RCC.I2C1Freq_Value' = '170000000'

        # ADC clocks are derived from the 170 MHz synchronous source / 4.
        # ADC1 is the common dual-mode master; ADC2 remains the slave IP.
        'ADC1.FHzAsyncCLK.Ext' = '1.7E8'
        'ADC1.FHzSyncCLK.Ext' = '1.7E8'
        'ADC1.Mode' = 'ADC_DUALMODE_INJECSIMULT'
        'ADC1.DMAAccessMode' = 'ADC_DMAACCESSMODE_DISABLED'
        'ADC1.TwoSamplingDelay' = 'ADC_TWOSAMPLINGDELAY_1CYCLE'
        'ADC1.ScanConvMode.Ext' = 'ADC_SCAN_ENABLE'
        'ADC1.EOCSelection.Ext' = 'ADC_EOC_SEQ_CONV'
        'ADC1.ExternalTrigConv.Ext' = 'ADC_SOFTWARE_START'
        'ADC1.ExternalTrigConvEdge.Ext' = 'ADC_EXTERNALTRIGCONVEDGE_NONE'
        'ADC1.ContinuousConvMode.Ext' = 'DISABLE'
        'ADC1.Overrun.Ext' = 'ADC_OVR_DATA_PRESERVED'
        'ADC1.NbrOfConversion.Ext' = '2'
        'ADC1.NbrOfConversionFlag' = '2'
        'ADC1.Channel-0\#ChannelRegularConversion' = 'ADC_CHANNEL_4'
        'ADC1.Rank-0\#ChannelRegularConversion' = '1'
        'ADC1.SamplingTime-0\#ChannelRegularConversion' = 'ADC_SAMPLETIME_47CYCLES_5'
        'ADC1.SingleDiff-0\#ChannelRegularConversion' = 'ADC_SINGLE_ENDED'
        'ADC1.OffsetNumber-0\#ChannelRegularConversion' = 'ADC_OFFSET_NONE'
        'ADC1.Channel-1\#ChannelRegularConversion' = 'ADC_CHANNEL_4'
        'ADC1.Rank-1\#ChannelRegularConversion' = '2'
        'ADC1.SamplingTime-1\#ChannelRegularConversion' = 'ADC_SAMPLETIME_47CYCLES_5'
        'ADC1.SingleDiff-1\#ChannelRegularConversion' = 'ADC_SINGLE_ENDED'
        'ADC1.OffsetNumber-1\#ChannelRegularConversion' = 'ADC_OFFSET_NONE'
        'ADC2.FHzAsyncCLK.Ext' = '1.7E8'
        'ADC2.FHzSyncCLK.Ext' = '1.7E8'
        'ADC2.ScanConvMode.Ext' = 'ADC_SCAN_ENABLE'
        'ADC2.EOCSelection.Ext' = 'ADC_EOC_SEQ_CONV'

        # 20 kHz center-aligned PWM. ARR+1=4250; CH4 PWM2 at 4164 places
        # rank 2 around the carrier crest, clear of the maximum-duty phase
        # edge. 4164 = 4250 - (340 / 4) - 1.
        'TIM1.Prescaler' = '0'
        'TIM1.PeriodNoDither' = '4249'
        'TIM1.PulseNoDither_1' = '2125'
        'TIM1.PulseNoDither_2' = '2125'
        'TIM1.PulseNoDither_3' = '2125'
        'TIM1.PulseNoDither_4' = '4164'
        'TIM1.OCMode_PWM-PWM\ Generation4\ No\ Output' = 'TIM_OCMODE_PWM2'
        'TIM1.DeadTime' = '34'
        'TIM1.BreakState.Ext' = 'TIM_BREAK_ENABLE'
        'TIM1.BreakPolarity' = 'TIM_BREAKPOLARITY_LOW'
        'TIM1.BreakFilter' = '4'
        'TIM1.BreakAFMode.Ext' = 'TIM_BREAK_AFMODE_INPUT'
        'TIM1.AutomaticOutput.Ext' = 'TIM_AUTOMATICOUTPUT_DISABLE'
        'TIM1.IPParameters' = 'Prescaler,CounterMode,PeriodNoDither,ClockDivision,RepetitionCounter,AutoReloadPreload,Channel-PWM Generation1 CH1 CH1N,Channel-PWM Generation2 CH2 CH2N,Channel-PWM Generation3 CH3 CH3N,Channel-PWM Generation4 No Output,PulseNoDither_1,PulseNoDither_2,PulseNoDither_3,PulseNoDither_4,OCMode_PWM-PWM Generation4 No Output,DeadTime,OffStateRunMode,OffStateIDLEMode,BreakState,BreakPolarity,BreakFilter,BreakAFMode,AutomaticOutput'
        'TIM2.Prescaler' = '169'
        'TIM2.PeriodNoDither.Ext' = '4294967295'

        # CAN-FD: 500 kbit/s arbitration, 2 Mbit/s data phase, BRS enabled.
        'FDCAN1.CalculateBaudRateNominal' = '500000'
        'FDCAN1.CalculateTimeBitNominal' = '2000'
        'FDCAN1.CalculateTimeQuantumNominal' = '58.8235294117647'
        'FDCAN1.FrameFormat' = 'FDCAN_FRAME_FD_BRS'
        'FDCAN1.NominalPrescaler' = '10'
        'FDCAN1.NominalSyncJumpWidth' = '4'
        'FDCAN1.NominalTimeSeg1' = '27'
        'FDCAN1.NominalTimeSeg2' = '6'
        'FDCAN1.DataPrescaler' = '5'
        'FDCAN1.DataSyncJumpWidth' = '3'
        'FDCAN1.DataTimeSeg1' = '13'
        'FDCAN1.DataTimeSeg2' = '3'
        'FDCAN1.StdFiltersNbr' = '8'
        'FDCAN1.ExtFiltersNbr' = '4'

        # I2C1 encoder fallback and production MT6701 SPI/SSI interface.
        'I2C1.AddressingMode.Ext' = 'I2C_ADDRESSINGMODE_7BIT'
        'I2C1.Analog_Filter.Ext' = 'I2C_ANALOGFILTER_ENABLE'
        'I2C1.CustomTiming' = 'Enabled'
        'I2C1.DualAddressMode.Ext' = 'I2C_DUALADDRESS_DISABLE'
        'I2C1.GeneralCallMode.Ext' = 'I2C_GENERALCALL_DISABLE'
        'I2C1.I2C_Coeff_DF.Ext' = '0'
        'I2C1.I2C_Fall_Time.Ext' = '0'
        'I2C1.I2C_Rise_Time.Ext' = '0'
        'I2C1.I2C_Speed_Mode.Ext' = 'I2C_Fast'
        'I2C1.IPParameters' = 'CustomTiming,Timing'
        'I2C1.Instance.Ext' = 'I2C$Index'
        'I2C1.NoStretchMode.Ext' = 'I2C_NOSTRETCH_DISABLE'
        'I2C1.OwnAddress.Ext' = '0'
        'I2C1.OwnAddress2.Ext' = '0'
        'I2C1.OwnAddress2Masks.Ext' = 'I2C_OA2_NOMASK'
        'I2C1.Speed.Ext' = '400'
        'I2C1.Timing' = '0x00F02B86'
        'PB6.Locked' = 'true'
        'PB6.Mode' = 'I2C'
        'PB6.Signal' = 'I2C1_SCL'
        'PB7.Locked' = 'true'
        'PB7.Mode' = 'I2C'
        'PB7.Signal' = 'I2C1_SDA'

        'SPI3.CalculateBaudRate' = '5.3125 MBits/s'
        'SPI3.GPIO_SPEED' = 'GPIO_SPEED_FREQ_VERY_HIGH'

        # GPIO names and power-up-safe output levels.
        'PB10.GPIOParameters' = 'GPIO_Speed,GPIO_Label'
        'PB10.GPIO_Label' = 'GATE_ENABLE'
        'PB10.GPIO_Speed' = 'GPIO_SPEED_FREQ_HIGH'
        'PB10.PinState' = 'GPIO_PIN_RESET'
        'PB11.GPIOParameters' = 'GPIO_Speed,GPIO_Label'
        'PB11.GPIO_Label' = 'DRIVER_RESET'
        'PB11.GPIO_Speed' = 'GPIO_SPEED_FREQ_HIGH'
        'PB11.PinState' = 'GPIO_PIN_RESET'
        'PB12.GPIOParameters' = 'GPIO_Speed,GPIO_Label'
        'PB12.GPIO_Label' = 'DRIVER_SLEEP'
        'PB12.GPIO_Speed' = 'GPIO_SPEED_FREQ_HIGH'
        'PB12.PinState' = 'GPIO_PIN_RESET'
        'PB3.GPIOParameters' = 'GPIO_Speed,GPIO_Label'
        'PB3.GPIO_Label' = 'CAN_STANDBY'
        'PB3.GPIO_Speed' = 'GPIO_SPEED_FREQ_HIGH'
        'PB3.PinState' = 'GPIO_PIN_SET'
        'PB4.GPIOParameters' = 'GPIO_Speed,GPIO_Label'
        'PB4.GPIO_Label' = 'ENCODER_CS'
        'PB4.GPIO_Speed' = 'GPIO_SPEED_FREQ_HIGH'
        'PB4.PinState' = 'GPIO_PIN_SET'
        'PC13.GPIOParameters' = 'GPIO_Speed,GPIO_Label'
        'PC13.GPIO_Label' = 'STATUS_LED'
        'PC13.GPIO_Speed' = 'GPIO_SPEED_FREQ_LOW'
        'PC13.PinState' = 'GPIO_PIN_RESET'
        'PA15.GPIOParameters' = 'GPIO_Speed,GPIO_Label'
        'PA15.GPIO_Label' = 'GATE_FAULT'
        'PA15.GPIO_Speed' = 'GPIO_SPEED_FREQ_VERY_HIGH'

        # Realtime interrupt priorities (preempt priority : subpriority).
        'NVIC.TIM1_BRK_TIM15_IRQn' = 'true\:0\:0\:false\:false\:true\:true\:true\:true'
        'NVIC.ADC1_2_IRQn' = 'true\:1\:0\:false\:false\:true\:true\:true\:true'
        'NVIC.DMA1_Channel1_IRQn' = 'true\:2\:0\:false\:false\:true\:true\:true\:true'
        'NVIC.DMA1_Channel2_IRQn' = 'true\:2\:1\:false\:false\:true\:true\:true\:true'
        'NVIC.SPI3_IRQn' = 'true\:2\:0\:false\:false\:true\:true\:true\:true'
        'NVIC.FDCAN1_IT0_IRQn' = 'true\:3\:0\:false\:false\:true\:true\:true\:true'
        'NVIC.FDCAN1_IT1_IRQn' = 'true\:3\:1\:false\:false\:true\:true\:true\:true'
        'NVIC.I2C1_EV_IRQn' = 'true\:4\:0\:false\:false\:true\:true\:true\:true'
        'NVIC.I2C1_ER_IRQn' = 'true\:4\:1\:false\:false\:true\:true\:true\:true'
        'NVIC.PendSV_IRQn' = 'true\:5\:0\:false\:false\:true\:false\:false\:false'
    }

    foreach ($key in $fixed.Keys) {
        $entries[$key] = [string]$fixed[$key]
    }

    # CubeMX only generates HAL_ADCEx_MultiModeConfigChannel() when these
    # non-default common parameters are listed as explicit ADC1 parameters.
    $adc1Parameters = [System.Collections.Generic.List[string]]::new()
    foreach ($parameter in ($entries['ADC1.IPParameters'] -split ',')) {
        if (-not [string]::IsNullOrWhiteSpace($parameter)) {
            $adc1Parameters.Add($parameter)
        }
    }
    foreach ($parameter in @(
            'Mode', 'DMAAccessMode', 'TwoSamplingDelay',
            'NbrOfConversionFlag',
            'Rank-1\#ChannelRegularConversion',
            'Channel-1\#ChannelRegularConversion',
            'SamplingTime-1\#ChannelRegularConversion',
            'SingleDiff-1\#ChannelRegularConversion',
            'OffsetNumber-1\#ChannelRegularConversion')) {
        if (-not $adc1Parameters.Contains($parameter)) {
            $adc1Parameters.Add($parameter)
        }
    }
    $entries['ADC1.IPParameters'] = $adc1Parameters -join ','

    # An independent-mode default cached by CubeMX conflicts with the explicit
    # common dual mode above. It is a UI-derived value, not a codegen input.
    [void]$entries.Remove('ADC1.Mode.Ext')

    $dma = [ordered]@{
        'Dma.Request0' = 'SPI3_RX'
        'Dma.Request1' = 'SPI3_TX'
        'Dma.RequestsNb' = '2'
        'Dma.SPI3_RX.0.Direction' = 'DMA_PERIPH_TO_MEMORY'
        'Dma.SPI3_RX.0.EventEnable' = 'DISABLE'
        'Dma.SPI3_RX.0.Instance' = 'DMA1_Channel1'
        'Dma.SPI3_RX.0.MemDataAlignment' = 'DMA_MDATAALIGN_BYTE'
        'Dma.SPI3_RX.0.MemInc' = 'DMA_MINC_ENABLE'
        'Dma.SPI3_RX.0.Mode' = 'DMA_NORMAL'
        'Dma.SPI3_RX.0.PeriphDataAlignment' = 'DMA_PDATAALIGN_BYTE'
        'Dma.SPI3_RX.0.PeriphInc' = 'DMA_PINC_DISABLE'
        'Dma.SPI3_RX.0.Polarity' = 'HAL_DMAMUX_REQ_GEN_RISING'
        'Dma.SPI3_RX.0.Priority' = 'DMA_PRIORITY_VERY_HIGH'
        'Dma.SPI3_RX.0.RequestNumber' = '1'
        'Dma.SPI3_RX.0.RequestParameters' = 'Instance,Direction,PeriphInc,MemInc,PeriphDataAlignment,MemDataAlignment,Mode,Priority,SignalID,Polarity,RequestNumber,SyncSignalID,SyncPolarity,SyncEnable,EventEnable,SyncRequestNumber'
        'Dma.SPI3_RX.0.SignalID' = 'NONE'
        'Dma.SPI3_RX.0.SyncEnable' = 'DISABLE'
        'Dma.SPI3_RX.0.SyncPolarity' = 'HAL_DMAMUX_SYNC_NO_EVENT'
        'Dma.SPI3_RX.0.SyncRequestNumber' = '1'
        'Dma.SPI3_RX.0.SyncSignalID' = 'NONE'
        'Dma.SPI3_TX.1.Direction' = 'DMA_MEMORY_TO_PERIPH'
        'Dma.SPI3_TX.1.EventEnable' = 'DISABLE'
        'Dma.SPI3_TX.1.Instance' = 'DMA1_Channel2'
        'Dma.SPI3_TX.1.MemDataAlignment' = 'DMA_MDATAALIGN_BYTE'
        'Dma.SPI3_TX.1.MemInc' = 'DMA_MINC_ENABLE'
        'Dma.SPI3_TX.1.Mode' = 'DMA_NORMAL'
        'Dma.SPI3_TX.1.PeriphDataAlignment' = 'DMA_PDATAALIGN_BYTE'
        'Dma.SPI3_TX.1.PeriphInc' = 'DMA_PINC_DISABLE'
        'Dma.SPI3_TX.1.Polarity' = 'HAL_DMAMUX_REQ_GEN_RISING'
        'Dma.SPI3_TX.1.Priority' = 'DMA_PRIORITY_HIGH'
        'Dma.SPI3_TX.1.RequestNumber' = '1'
        'Dma.SPI3_TX.1.RequestParameters' = 'Instance,Direction,PeriphInc,MemInc,PeriphDataAlignment,MemDataAlignment,Mode,Priority,SignalID,Polarity,RequestNumber,SyncSignalID,SyncPolarity,SyncEnable,EventEnable,SyncRequestNumber'
        'Dma.SPI3_TX.1.SignalID' = 'NONE'
        'Dma.SPI3_TX.1.SyncEnable' = 'DISABLE'
        'Dma.SPI3_TX.1.SyncPolarity' = 'HAL_DMAMUX_SYNC_NO_EVENT'
        'Dma.SPI3_TX.1.SyncRequestNumber' = '1'
        'Dma.SPI3_TX.1.SyncSignalID' = 'NONE'
    }
    foreach ($key in $dma.Keys) {
        $entries[$key] = [string]$dma[$key]
    }

    # These stale defaults conflict with the explicit production clock keys.
    foreach ($key in @('RCC.HSE_VALUE', 'RCC.PLLM.Ext', 'RCC.PLLN.Ext')) {
        [void]$entries.Remove($key)
    }

    $output = [System.Collections.Generic.List[string]]::new()
    $output.Add('#MicroXplorer Configuration settings - do not modify')
    foreach ($entry in $entries.GetEnumerator()) {
        $output.Add("$($entry.Key)=$($entry.Value)")
    }
    [System.IO.File]::WriteAllLines($Path, $output,
        [System.Text.Encoding]::ASCII)
}

function Test-IocContract {
    param([Parameter(Mandatory)][string]$Path)

    if (-not (Test-Path -LiteralPath $Path -PathType Leaf)) {
        throw "Missing .ioc file: $Path"
    }
    $entries = Read-IocEntries $Path
    $required = [ordered]@{
        'Mcu.CPN' = 'STM32G431CBU6'
        'RCC.HSE_VALUE.Ext' = '8000000'
        'RCC.PLLSourceVirtual' = 'RCC_PLLSOURCE_HSE'
        'RCC.PLLM' = 'RCC_PLLM_DIV2'
        'RCC.PLLN' = '85'
        'RCC.SYSCLKSource' = 'RCC_SYSCLKSOURCE_PLLCLK'
        'RCC.SYSCLKFreq_VALUE' = '170000000'
        'RCC.FDCANFreq_Value' = '170000000'
        'PA1.Signal' = 'ADC1_IN2'
        'PA3.Signal' = 'ADC1_IN4'
        'PA7.Signal' = 'ADC2_IN4'
        'ADC1.Channel-0\#ChannelRegularConversion' = 'ADC_CHANNEL_4'
        'ADC1.Channel-1\#ChannelRegularConversion' = 'ADC_CHANNEL_4'
        'ADC1.Rank-0\#ChannelRegularConversion' = '1'
        'ADC1.Rank-1\#ChannelRegularConversion' = '2'
        'ADC1.SamplingTime-0\#ChannelRegularConversion' = 'ADC_SAMPLETIME_47CYCLES_5'
        'ADC1.SamplingTime-1\#ChannelRegularConversion' = 'ADC_SAMPLETIME_47CYCLES_5'
        'ADC1.SingleDiff-0\#ChannelRegularConversion' = 'ADC_SINGLE_ENDED'
        'ADC1.SingleDiff-1\#ChannelRegularConversion' = 'ADC_SINGLE_ENDED'
        'ADC1.OffsetNumber-0\#ChannelRegularConversion' = 'ADC_OFFSET_NONE'
        'ADC1.OffsetNumber-1\#ChannelRegularConversion' = 'ADC_OFFSET_NONE'
        'ADC1.NbrOfConversion.Ext' = '2'
        'ADC1.NbrOfConversionFlag' = '2'
        'ADC1.Overrun.Ext' = 'ADC_OVR_DATA_PRESERVED'
        'ADC1.Mode' = 'ADC_DUALMODE_INJECSIMULT'
        'ADC1.DMAAccessMode' = 'ADC_DMAACCESSMODE_DISABLED'
        'ADC1.TwoSamplingDelay' = 'ADC_TWOSAMPLINGDELAY_1CYCLE'
        'ADC1.ScanConvMode.Ext' = 'ADC_SCAN_ENABLE'
        'ADC1.EOCSelection.Ext' = 'ADC_EOC_SEQ_CONV'
        'ADC1.ExternalTrigConv.Ext' = 'ADC_SOFTWARE_START'
        'ADC1.ExternalTrigConvEdge.Ext' = 'ADC_EXTERNALTRIGCONVEDGE_NONE'
        'ADC1.ContinuousConvMode.Ext' = 'DISABLE'
        'ADC1.EnableInjectedConversion' = 'ENABLE'
        'ADC1.InjNumberOfConversion' = '2'
        'ADC1.InjectedChannel-0\#ChannelInjectedConversion' = 'ADC_CHANNEL_2'
        'ADC1.InjectedChannel-1\#ChannelInjectedConversion' = 'ADC_CHANNEL_2'
        'ADC1.InjectedRank-0\#ChannelInjectedConversion' = '1'
        'ADC1.InjectedRank-1\#ChannelInjectedConversion' = '2'
        'ADC1.InjectedSamplingTime-0\#ChannelInjectedConversion' = 'ADC_SAMPLETIME_12CYCLES_5'
        'ADC1.InjectedSamplingTime-1\#ChannelInjectedConversion' = 'ADC_SAMPLETIME_12CYCLES_5'
        'ADC1.ExternalTrigInjecConv' = 'ADC_EXTERNALTRIGINJEC_T1_CC4'
        'ADC2.InjNumberOfConversion' = '2'
        'ADC2.EnableInjectedConversion' = 'ENABLE'
        'ADC2.InjectedChannel-0\#ChannelInjectedConversion' = 'ADC_CHANNEL_4'
        'ADC2.InjectedChannel-1\#ChannelInjectedConversion' = 'ADC_CHANNEL_4'
        'ADC2.InjectedRank-0\#ChannelInjectedConversion' = '1'
        'ADC2.InjectedRank-1\#ChannelInjectedConversion' = '2'
        'ADC2.InjectedSamplingTime-0\#ChannelInjectedConversion' = 'ADC_SAMPLETIME_12CYCLES_5'
        'ADC2.InjectedSamplingTime-1\#ChannelInjectedConversion' = 'ADC_SAMPLETIME_12CYCLES_5'
        'ADC2.Mode' = 'ADC_DUALMODE_INJECSIMULT'
        'ADC2.ScanConvMode.Ext' = 'ADC_SCAN_ENABLE'
        'ADC2.EOCSelection.Ext' = 'ADC_EOC_SEQ_CONV'
        'ADC2.EnableRegularConversion' = 'DISABLE'
        'ADC2.NbrOfConversion.Ext' = '1'
        'ADC2.NbrOfConversionFlag.Ext' = '0'
        'TIM1.CounterMode' = 'TIM_COUNTERMODE_CENTERALIGNED1'
        'TIM1.PeriodNoDither' = '4249'
        'TIM1.PulseNoDither_4' = '4164'
        'TIM1.OCMode_PWM-PWM\ Generation4\ No\ Output' = 'TIM_OCMODE_PWM2'
        'TIM1.DeadTime' = '34'
        'PA15.Signal' = 'TIM1_BKIN'
        'TIM2.Prescaler' = '169'
        'TIM2.PeriodNoDither.Ext' = '4294967295'
        'FDCAN1.FrameFormat' = 'FDCAN_FRAME_FD_BRS'
        'FDCAN1.NominalPrescaler' = '10'
        'FDCAN1.NominalTimeSeg1' = '27'
        'FDCAN1.NominalTimeSeg2' = '6'
        'FDCAN1.DataPrescaler' = '5'
        'FDCAN1.DataTimeSeg1' = '13'
        'FDCAN1.DataTimeSeg2' = '3'
        'PB6.Signal' = 'I2C1_SCL'
        'PB7.Signal' = 'I2C1_SDA'
        'SPI3.BaudRatePrescaler' = 'SPI_BAUDRATEPRESCALER_32'
        'Dma.SPI3_RX.0.Instance' = 'DMA1_Channel1'
        'Dma.SPI3_TX.1.Instance' = 'DMA1_Channel2'
        'NVIC.TIM1_BRK_TIM15_IRQn' = 'true\:0\:0\:false\:false\:true\:true\:true\:true'
        'NVIC.ADC1_2_IRQn' = 'true\:1\:0\:false\:false\:true\:true\:true\:true'
        'NVIC.FDCAN1_IT0_IRQn' = 'true\:3\:0\:false\:false\:true\:true\:true\:true'
        'NVIC.PendSV_IRQn' = 'true\:5\:0\:false\:false\:true\:false\:false\:false'
        'PB3.PinState' = 'GPIO_PIN_SET'
        'PB4.PinState' = 'GPIO_PIN_SET'
        'PB10.PinState' = 'GPIO_PIN_RESET'
    }

    $errors = [System.Collections.Generic.List[string]]::new()
    foreach ($key in $required.Keys) {
        if (-not $entries.ContainsKey($key)) {
            $errors.Add("missing $key")
        } elseif ($entries[$key] -ne $required[$key]) {
            $errors.Add("$key is '$($entries[$key])', expected '$($required[$key])'")
        }
    }
    if ($entries.ContainsKey('PB0.Signal') -and
        $entries['PB0.Signal'].StartsWith('ADC')) {
        $errors.Add('PB0 must remain reserved/optional and may not enter an ADC sequence')
    }

    $sysclk = 170000000.0
    $nominalBitrate = $sysclk / (10.0 * (1.0 + 27.0 + 6.0))
    $dataBitrate = $sysclk / (5.0 * (1.0 + 13.0 + 3.0))
    $pwmFrequency = $sysclk / (2.0 * (4249.0 + 1.0))
    $microsecondTimer = $sysclk / (169.0 + 1.0)
    $minimumWindowTicks = 340
    $expectedSamplePulse = 4250 - ($minimumWindowTicks / 4) - 1
    if (($nominalBitrate -ne 500000.0) -or ($dataBitrate -ne 2000000.0) -or
        ($pwmFrequency -ne 20000.0) -or ($microsecondTimer -ne 1000000.0) -or
        ($expectedSamplePulse -ne 4164)) {
        $errors.Add('internal timing contract calculation failed')
    }

    if ($errors.Count -gt 0) {
        throw "The CubeMX hardware contract is invalid:`n - $($errors -join "`n - ")"
    }
    Write-Host 'Static .ioc contract: PASS (170 MHz, CAN-FD 500k/2M, 20 kHz PWM, dual injected ADC plus two-rank ADC1 VBUS, DMA and IRQ priorities).'
}

function Invoke-IocCliValidation {
    $templatePath = Join-Path $PSScriptRoot 'cubemx_validate.txt.in'
    $temporaryScript = New-CubeMxScript $templatePath @{
        IOC = ConvertTo-CubeMxPath $ioc
    }
    try {
        [void](Invoke-CubeMxScript $temporaryScript 'CubeMX reload/check validation')
    }
    finally {
        Remove-Item -LiteralPath $temporaryScript -Force -ErrorAction SilentlyContinue
    }
}

switch ($Action) {
    'open' {
        Test-IocContract $ioc
        Start-Process -FilePath $jar -ArgumentList @($ioc) -WindowStyle Hidden
    }
    { ($_ -eq 'create-ioc') -or ($_ -eq 'configure-ioc') } {
        $templatePath = Join-Path $PSScriptRoot 'cubemx_configure_ioc.txt.in'
        $temporaryScript = New-CubeMxScript $templatePath @{
            IOC = ConvertTo-CubeMxPath $ioc
        }
        try {
            [void](Invoke-CubeMxScript $temporaryScript 'CubeMX peripheral configuration')
            Set-ProductionIocMetadata $ioc
            Test-IocContract $ioc
            Invoke-IocCliValidation
        }
        finally {
            Remove-Item -LiteralPath $temporaryScript -Force -ErrorAction SilentlyContinue
        }
    }
    'validate' {
        Test-IocContract $ioc
        Invoke-IocCliValidation
    }
    'generate' {
        Test-IocContract $ioc
        Invoke-IocCliValidation
        $stagingPath = Join-Path $projectRoot 'staging/cubemx'
        [void](New-Item -ItemType Directory -Path $stagingPath -Force)
        Write-Warning 'CubeMX output is restricted to staging/cubemx and will not overwrite the hand-integrated Core/App firmware.'
        Write-Warning 'This script never accepts an ST firmware licence. If CubeMX requests one, review it interactively and rerun.'
        $templatePath = Join-Path $PSScriptRoot 'cubemx_generate.txt.in'
        $temporaryScript = New-CubeMxScript $templatePath @{
            IOC = ConvertTo-CubeMxPath $ioc
            PROJECT_ROOT = ConvertTo-CubeMxPath $projectRoot
        }
        try {
            [void](Invoke-CubeMxScript $temporaryScript 'CubeMX staging generation')
        }
        finally {
            Remove-Item -LiteralPath $temporaryScript -Force -ErrorAction SilentlyContinue
        }
    }
    'install-g4' {
        Write-Warning 'STM32CubeMX may open an ST authentication/licence prompt. This script will not accept it for you.'
        & $java -jar $jar -i (Join-Path $PSScriptRoot 'cubemx_install_g4.txt')
        if ($LASTEXITCODE -ne 0) {
            throw "STM32CubeG4 package installation did not complete (exit $LASTEXITCODE)."
        }
    }
}
