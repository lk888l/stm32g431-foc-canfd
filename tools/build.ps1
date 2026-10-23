param(
    [ValidateSet('Debug', 'Release', 'Host', 'Bench')][string]$Preset = 'Debug',
    [string]$Python = 'python',
    [ValidateRange(1, 256)][int]$Jobs = 8,
    [switch]$Fresh
)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
Push-Location -LiteralPath $projectRoot
try {
    $requiredTools = @($Python, 'cmake', 'ninja')
    if ($Preset -eq 'Host') {
        $requiredTools += 'ctest'
    } else {
        $requiredTools += @('arm-none-eabi-gcc', 'arm-none-eabi-g++', 'arm-none-eabi-objcopy', 'arm-none-eabi-size', 'arm-none-eabi-nm')
    }
    foreach ($tool in $requiredTools) {
        if (-not (Get-Command $tool -ErrorAction SilentlyContinue)) {
            throw "Required tool '$tool' was not found. Add its installation directory to PATH, then open a new terminal. See docs/build.md."
        }
    }
    & $Python tools/generate_od.py --check
    if ($LASTEXITCODE -ne 0) { throw 'Object dictionary is stale' }
    & $Python tools/vendor.py --check
    if ($LASTEXITCODE -ne 0) { throw 'Vendor integrity check failed' }
    & $Python tools/check_project.py
    if ($LASTEXITCODE -ne 0) { throw 'CubeMX contract check failed' }
    $configureArguments = @('--preset', $Preset)
    if ($Fresh) { $configureArguments += '--fresh' }
    cmake @configureArguments
    if ($LASTEXITCODE -ne 0) { throw "CMake configure failed; see the error above. For a copied build cache or a changed toolchain, retry with -Preset $Preset -Fresh." }
    cmake --build --preset $Preset --parallel $Jobs
    if ($LASTEXITCODE -ne 0) { throw 'Build failed' }
    if ($Preset -eq 'Host') {
        ctest --preset Host
        if ($LASTEXITCODE -ne 0) { throw 'Host tests failed' }
    } else {
        $benchArguments = @()
        if ($Preset -eq 'Bench') { $benchArguments = @('--bench') }
        & $Python tools/check_project.py --elf "build/$Preset/canopen_foc.elf" @benchArguments
        if ($LASTEXITCODE -ne 0) { throw 'ELF contract check failed' }
    }
} finally { Pop-Location }
