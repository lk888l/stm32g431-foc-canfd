[CmdletBinding()]
param(
    [ValidateSet('all', 'host', 'firmware')]
    [string]$Target = 'all',
    [ValidateSet('Debug', 'Release')]
    [string]$Configuration = 'Debug',
    [ValidateSet('Production', 'Lab')]
    [string]$BoardProfile = 'Production',
    [string]$ArmToolchainRoot = $env:ARM_GNU_TOOLCHAIN_ROOT
)

$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$cmake = (Get-Command cmake -ErrorAction Stop).Source
$make = (Get-Command mingw32-make -ErrorAction Stop).Source

function Invoke-Checked {
    param([string]$Program, [string[]]$Arguments)
    & $Program @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed with exit code ${LASTEXITCODE}: $Program $Arguments"
    }
}

if (($Target -eq 'all') -or ($Target -eq 'host')) {
    $hostBuild = Join-Path $projectRoot 'build/host'
    Invoke-Checked $cmake @(
        '-S', $projectRoot, '-B', $hostBuild,
        '-G', 'MinGW Makefiles',
        "-DCMAKE_MAKE_PROGRAM=$make",
        "-DCMAKE_BUILD_TYPE=$Configuration",
        '-DBRUSHLESS_BUILD_FIRMWARE=OFF',
        '-DBRUSHLESS_BUILD_TESTS=ON'
    )
    Invoke-Checked $cmake @('--build', $hostBuild, '--parallel')
    Invoke-Checked $cmake @('--build', $hostBuild, '--target', 'test')
}

if (($Target -eq 'all') -or ($Target -eq 'firmware')) {
    if ([string]::IsNullOrWhiteSpace($ArmToolchainRoot)) {
        $armGcc = Get-Command arm-none-eabi-gcc -ErrorAction Stop
        $ArmToolchainRoot = Split-Path (Split-Path $armGcc.Source -Parent) -Parent
    }

    $profileSuffix = if ($BoardProfile -eq 'Lab') { '-lab' } else { '' }
    $firmwareBuild = Join-Path $projectRoot (
        "build/firmware$profileSuffix-" + $Configuration.ToLowerInvariant())
    $toolchain = Join-Path $projectRoot 'cmake/gcc-arm-none-eabi.cmake'
    $firmwareConfigureArguments = @(
        '-S', $projectRoot, '-B', $firmwareBuild,
        '-G', 'MinGW Makefiles',
        "-DCMAKE_MAKE_PROGRAM=$make",
        "-DCMAKE_BUILD_TYPE=$Configuration",
        "-DARM_GNU_TOOLCHAIN_ROOT=$ArmToolchainRoot",
        '-DBRUSHLESS_BUILD_FIRMWARE=ON',
        '-DBRUSHLESS_BUILD_TESTS=OFF',
        "-DBRUSHLESS_BOARD_PROFILE=$($BoardProfile.ToUpperInvariant())",
        '-DBRUSHLESS_USE_8MHZ_HSE=ON'
    )
    if (-not (Test-Path -LiteralPath (Join-Path $firmwareBuild 'CMakeCache.txt'))) {
        $firmwareConfigureArguments += "-DCMAKE_TOOLCHAIN_FILE=$toolchain"
    }
    Invoke-Checked $cmake $firmwareConfigureArguments
    Invoke-Checked $cmake @('--build', $firmwareBuild, '--parallel')
}
