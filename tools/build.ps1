param(
    [ValidateSet('Debug', 'Release', 'Host', 'Bench')][string]$Preset = 'Debug',
    [string]$Python = 'python',
    [int]$Jobs = 8
)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
Push-Location -LiteralPath $projectRoot
try {
    & $Python tools/generate_od.py --check
    if ($LASTEXITCODE -ne 0) { throw 'Object dictionary is stale' }
    & $Python tools/vendor.py --check
    if ($LASTEXITCODE -ne 0) { throw 'Vendor integrity check failed' }
    & $Python tools/check_project.py
    if ($LASTEXITCODE -ne 0) { throw 'CubeMX contract check failed' }
    cmake --preset $Preset
    if ($LASTEXITCODE -ne 0) { throw 'CMake configure failed; for a copied old build directory use cmake --fresh --preset <name>' }
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
