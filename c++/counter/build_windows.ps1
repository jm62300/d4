# build_windows.ps1 — Windows equivalent of build.sh for C++ counter
# Usage: .\build_windows.ps1 [-c] [-d] [-s] [-p] [-j]
#
#   -c  Competition mode
#   -d  Debug mode
#   -s  Static mode
#   -p  Profiling mode
#   -j  Parallel build

param(
    [switch]$c,
    [switch]$d,
    [switch]$s,
    [switch]$p,
    [switch]$j
)

# Set error preference to Continue since Windows profiles might output noise
$ErrorActionPreference = "Continue"

# Resolve directories
$SCRIPT_DIR = Split-Path -Parent $MyInvocation.MyCommand.Definition
$D4_ROOT_DIR = Resolve-Path (Join-Path $SCRIPT_DIR "..\..")

# --- Parse option flags ---
$BUILD_TYPE = "Release"
$STATIC_FLAG = "OFF"
$PROFILE_FLAG = "OFF"
$COMPETITION_FLAG = "OFF"

$BUILD_MODE_ARGS = @()

if ($c) {
    $BUILD_TYPE = "Release"
    $STATIC_FLAG = "OFF"
    $PROFILE_FLAG = "OFF"
    $COMPETITION_FLAG = "ON"
    $BUILD_MODE_ARGS += "-c"
} elseif ($d) {
    $BUILD_TYPE = "Debug"
    $STATIC_FLAG = "OFF"
    $PROFILE_FLAG = "OFF"
    $BUILD_MODE_ARGS += "-d"
} elseif ($s) {
    $BUILD_TYPE = "Release"
    $STATIC_FLAG = "ON"
    $PROFILE_FLAG = "OFF"
    $BUILD_MODE_ARGS += "-s"
} elseif ($p) {
    $BUILD_TYPE = "RelWithDebInfo"
    $STATIC_FLAG = "OFF"
    $PROFILE_FLAG = "ON"
    $BUILD_MODE_ARGS += "-p"
}

# --- Parallel flag ---
$PARALLEL_FLAGS = @()
if ($j) {
    $PARALLEL_FLAGS = @("--parallel")
    $BUILD_MODE_ARGS += "-j"
}

# ==============================================================================
# Check and build dependencies (d4)
# ==============================================================================
Write-Host "c [BUILD] Parent library d4 build starting..." -ForegroundColor Cyan

Push-Location $D4_ROOT_DIR
powershell -ExecutionPolicy Bypass -File .\build_windows.ps1 @BUILD_MODE_ARGS
if ($LASTEXITCODE -ne 0) {
    Write-Error "Parent build of d4 failed!"
    Pop-Location
    exit $LASTEXITCODE
}
Pop-Location

# ==============================================================================
# Configure and build 'counter'
# ==============================================================================
Set-Location $SCRIPT_DIR
New-Item -ItemType Directory -Force -Path "build" | Out-Null
Set-Location "build"

Write-Host "c [BUILD] Configuring CMake (Type: $BUILD_TYPE, Static: $STATIC_FLAG, Profile: $PROFILE_FLAG, Competition: $COMPETITION_FLAG)..." -ForegroundColor Cyan

$CMAKE_FLAGS = @(
    "-DCMAKE_BUILD_TYPE=$BUILD_TYPE",
    "-DBUILD_STATIC=$STATIC_FLAG",
    "-DBUILD_PROFILE=$PROFILE_FLAG",
    "-DBUILD_COMPETITION=$COMPETITION_FLAG"
)

cmake .. @CMAKE_FLAGS
if ($LASTEXITCODE -ne 0) {
    Write-Error "CMake configuration failed (exit code $LASTEXITCODE)."
    exit $LASTEXITCODE
}

Write-Host "c [BUILD] Compiling executable..." -ForegroundColor Cyan
cmake --build . @PARALLEL_FLAGS
if ($LASTEXITCODE -ne 0) {
    Write-Error "CMake build failed (exit code $LASTEXITCODE)."
    exit $LASTEXITCODE
}

Write-Host "c [BUILD] Build complete! The 'counter' executable is ready." -ForegroundColor Green
