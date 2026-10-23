param(
    [string]$CubeMxHome = $env:STM32CUBEMX_HOME,
    [string]$Python = 'python',
    [switch]$Check
)
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path -LiteralPath (Split-Path -Parent $PSScriptRoot)).Path
if (-not $CubeMxHome) {
    # Also see a newly configured user installation from an already-open terminal.
    $CubeMxHome = [Environment]::GetEnvironmentVariable('STM32CUBEMX_HOME', 'User')
}
if (-not $CubeMxHome) {
    $cubeCommand = Get-Command STM32CubeMX.exe -ErrorAction SilentlyContinue
    if ($cubeCommand) { $CubeMxHome = Split-Path -Parent $cubeCommand.Source }
    elseif ($env:ProgramFiles) { $CubeMxHome = Join-Path $env:ProgramFiles 'STMicroelectronics/STM32Cube/STM32CubeMX' }
}
if (-not $CubeMxHome) { throw 'Set STM32CUBEMX_HOME or pass -CubeMxHome with the CubeMX installation directory.' }
$java = Join-Path $CubeMxHome 'jre/bin/java.exe'
$cube = Join-Path $CubeMxHome 'STM32CubeMX.exe'
if (-not (Test-Path -LiteralPath $java) -or -not (Test-Path -LiteralPath $cube)) {
    throw "CubeMX with bundled JRE not found at '$CubeMxHome'. Set STM32CUBEMX_HOME or pass -CubeMxHome."
}
$ioc = Join-Path $projectRoot 'canopen_foc.ioc'
$iocSettings = Get-Content -LiteralPath $ioc
$requiredVersion = ($iocSettings | Select-String '^MxCube\.Version=(.+)$').Matches.Groups[1].Value
$requiredFirmware = ($iocSettings | Select-String '^ProjectManager\.FirmwarePackage=(.+)$').Matches.Groups[1].Value
# The uninstall registry version can remain stale after CubeMX's in-place updater.
$versionText = (Get-Item -LiteralPath $cube).VersionInfo.ProductVersion
$versionMatch = [regex]::Match($versionText, '\d+\.\d+\.\d+')
if (-not $versionMatch.Success) { throw "Cannot read the installed CubeMX version from '$cube'." }
$installedVersion = $versionMatch.Value
Write-Host "CubeMX: $installedVersion ($cube)"
Write-Host "Project requires CubeMX $requiredVersion or newer and $requiredFirmware."
if ([version]$installedVersion -lt [version]$requiredVersion) {
    throw "Installed CubeMX $installedVersion is older than this .ioc ($requiredVersion). Install CubeMX $requiredVersion or newer and select that executable in VS Code's STM32CubeMX path setting. Do not downgrade the .ioc version fields."
}
if ($Check) {
    Write-Host 'CubeMX version check passed. Install the required firmware package in CubeMX before generating code.'
    return
}
$scratch = Join-Path $projectRoot '.work'
New-Item -ItemType Directory -Path $scratch -Force | Out-Null
$script = Join-Path $scratch 'cubemx-generate.txt'
# Relative paths also work when the checkout path contains spaces; avoid a UTF-8
# BOM, which Windows PowerShell 5.1's Set-Content -Encoding utf8 would emit.
[IO.File]::WriteAllLines($script, @('config load canopen_foc.ioc', 'config save canopen_foc.ioc', 'project generate', 'exit'), [Text.UTF8Encoding]::new($false))
# Root files are custom-owned when CubeMX sees that they already exist.
$owned = @('CMakeLists.txt', 'CMakePresets.json', 'cmake/gcc-arm-none-eabi.cmake')
$before = @{}
foreach ($relative in $owned) { $before[$relative] = (Get-FileHash -LiteralPath (Join-Path $projectRoot $relative)).Hash }
Push-Location -LiteralPath $projectRoot
try {
    & $java -jar $cube -q $script *> (Join-Path $scratch 'cubemx-generate.log')
    if ($LASTEXITCODE -ne 0) { throw 'CubeMX generation failed; inspect .work/cubemx-generate.log' }
    foreach ($relative in $owned) {
        if ((Get-FileHash -LiteralPath $relative).Hash -ne $before[$relative]) { throw "CubeMX changed custom file: $relative" }
    }
    & $Python tools/check_project.py
    if ($LASTEXITCODE -ne 0) { throw 'Generated project violates the hardware contract' }
    Write-Host 'CubeMX generation passed. Rebuild firmware to validate the generated sources.'
} finally { Pop-Location }
