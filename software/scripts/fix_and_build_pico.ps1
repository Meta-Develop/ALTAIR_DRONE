$ErrorActionPreference = "Stop"

$picoFirmwareDir = Join-Path $PSScriptRoot "..\pico_firmware"
$buildDir = Join-Path $picoFirmwareDir "build"

Write-Host "Cleaning build directory: $buildDir"
if (Test-Path $buildDir) {
    Remove-Item -Path $buildDir -Recurse -Force
}
New-Item -Path $buildDir -ItemType Directory | Out-Null

Set-Location $buildDir

Write-Host "Running CMake..."
cmake -G "MinGW Makefiles" ..
if ($LASTEXITCODE -ne 0) {
    Write-Error "CMake failed."
}

Write-Host "Running Make..."
mingw32-make -j4
if ($LASTEXITCODE -ne 0) {
    Write-Error "Make failed."
}

Write-Host "Build Successful. UF2 should be in $buildDir"
