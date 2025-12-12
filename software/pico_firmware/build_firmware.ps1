# Build Script for Pico Firmware
# Note: pico-sdk is fetched automatically by CMake (PICO_SDK_FETCH_FROM_GIT=ON in CMakeLists.txt)

$BUILD_DIR = "$PSScriptRoot\build"

Write-Host "Configuring CMake..."
Set-Location $PSScriptRoot
if (Test-Path $BUILD_DIR) {
    Remove-Item -Recurse -Force $BUILD_DIR
}
cmake -B build -S . -G "MinGW Makefiles"

Write-Host "Building Firmware..."
cmake --build build

Write-Host "Done. Firmware is at $BUILD_DIR\pico_node.uf2"
