param(
    [Parameter(Mandatory = $true)][string]$CubeMxHome,
    [string]$Python = 'python'
)
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path -LiteralPath (Split-Path -Parent $PSScriptRoot)).Path
$java = Join-Path $CubeMxHome 'jre/bin/java.exe'
$cube = Join-Path $CubeMxHome 'STM32CubeMX.exe'
if (-not (Test-Path -LiteralPath $java) -or -not (Test-Path -LiteralPath $cube)) { throw 'CubeMX with bundled JRE not found' }
$scratch = Join-Path $projectRoot '.work'
New-Item -ItemType Directory -Path $scratch -Force | Out-Null
$script = Join-Path $scratch 'cubemx-generate.txt'
$ioc = Join-Path $projectRoot 'canopen_foc.ioc'
@("config load $ioc", "config save $ioc", 'project generate', 'exit') | Set-Content -LiteralPath $script -Encoding utf8
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
