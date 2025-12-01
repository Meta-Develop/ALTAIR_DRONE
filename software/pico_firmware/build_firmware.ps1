# Build Script for Pico Firmware

$PICO_SDK_PATH = "$PSScriptRoot\pico-sdk"
$BUILD_DIR = "$PSScriptRoot\build"

Write-Host "Initializing Submodules..."
Set-Location $PICO_SDK_PATH
git submodule update --init --recursive

Write-Host "Configuring CMake..."
Set-Location $PSScriptRoot
if (Test-Path $BUILD_DIR) {
    Remove-Item -Recurse -Force $BUILD_DIR
}
$env:PICO_SDK_PATH = $PICO_SDK_PATH
cmake -B build -S . -G "MinGW Makefiles"

Write-Host "Building Firmware..."
cmake --build build

Write-Host "Done. Firmware is at $BUILD_DIR\pico_node.uf2"
