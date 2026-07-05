# build_windows.ps1 — Windows equivalent of build.sh
# Usage: .\build_windows.ps1 [-c] [-d] [-s] [-p] [-j]
#
#   -c  Competition mode  (BUILD_MODE=4 : -O3 -flto, via CMake)
#   -d  Debug mode        (BUILD_MODE=1)
#   -s  Static mode       (BUILD_MODE=2)
#   -p  Profiling mode    (BUILD_MODE=3)
#   -j  Parallel build    (passes --parallel to cmake --build)

param(
    [switch]$c,
    [switch]$d,
    [switch]$s,
    [switch]$p,
    [switch]$j
)

# Stop on first error (equivalent to set -e)
# Note: PowerShell profile errors (e.g. Chocolatey DLL) are non-fatal and ignored.
$ErrorActionPreference = "Continue"

# Resolve script directory (equivalent to SCRIPT_DIR in bash)
$SCRIPT_DIR = Split-Path -Parent $MyInvocation.MyCommand.Definition

# --- Determine BUILD_MODE (same priority as bash getopts) ---
$opt = 0
if ($c) { $opt = 4 }
elseif ($d) { $opt = 1 }
elseif ($s) { $opt = 2 }
elseif ($p) { $opt = 3 }

# --- Parallel flag ---
$PARALLEL_FLAGS = @()
if ($j) { $PARALLEL_FLAGS = @("--parallel") }

# --- CMake flags ---
$CMAKE_FLAGS = @("-DBUILD_MODE=$opt")
if ($env:CMAKE_TOOLCHAIN_FILE) {
    $CMAKE_FLAGS += "-DCMAKE_TOOLCHAIN_FILE=$env:CMAKE_TOOLCHAIN_FILE"
}

# --- GitLab token (same default as build.sh) ---
if (-not $env:GITLAB_TOKEN_LOGICAL) {
    $env:GITLAB_TOKEN_LOGICAL = "glpat-eHavaAslBimi87EABf6MIW86MQp1OjJpdgk.01.0z0dxf0ko"
}

# --- Create and enter build directory ---
Set-Location $SCRIPT_DIR
New-Item -ItemType Directory -Force -Path "build" | Out-Null
Set-Location "build"

# --- Configure ---
Write-Host "c [BUILD] Configuring CMake..." -ForegroundColor Cyan
cmake .. @CMAKE_FLAGS
if ($LASTEXITCODE -ne 0) {
    Write-Error "CMake configuration failed (exit code $LASTEXITCODE)."
    exit $LASTEXITCODE
}

# --- Build ---
Write-Host "c [BUILD] Compiling..." -ForegroundColor Cyan
cmake --build . @PARALLEL_FLAGS
if ($LASTEXITCODE -ne 0) {
    Write-Error "CMake build failed (exit code $LASTEXITCODE)."
    exit $LASTEXITCODE
}

Write-Host "c [BUILD] Build complete! A monolithic libd4.a has been created natively." -ForegroundColor Green
