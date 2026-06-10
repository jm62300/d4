/*
 * File:   CompilerDetection.hpp
 *
 * Hand-written replacement for the header htd generates with CMake's
 * write_compiler_detection_header (removed in CMake >= 3.19). d4 compiles
 * htd with a C++20 compiler, so all the detected features are available
 * unconditionally. Only the macros actually used by the htd sources are
 * defined here.
 */

#ifndef HTD_COMPILER_DETECTION_HPP
#define HTD_COMPILER_DETECTION_HPP

#if defined(_MSC_VER)
#define HTD_COMPILER_IS_MSVC 1
#else
#define HTD_COMPILER_IS_MSVC 0
#endif

#if defined(__clang__) && defined(__apple_build_version__)
#define HTD_COMPILER_IS_AppleClang 1
#define HTD_COMPILER_IS_Clang 0
#elif defined(__clang__)
#define HTD_COMPILER_IS_AppleClang 0
#define HTD_COMPILER_IS_Clang 1
#else
#define HTD_COMPILER_IS_AppleClang 0
#define HTD_COMPILER_IS_Clang 0
#endif

#if defined(__GNUC__) && !defined(__clang__)
#define HTD_COMPILER_IS_GNU 1
#else
#define HTD_COMPILER_IS_GNU 0
#endif

#define HTD_NOEXCEPT noexcept
#define HTD_OVERRIDE override

#endif /* HTD_COMPILER_DETECTION_HPP */
