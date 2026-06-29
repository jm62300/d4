#pragma once

// --- Include OS-specific headers ---
#if defined(_WIN32)
#include <windows.h>
///
#include <psapi.h>  // Required for GetProcessMemoryInfo

#elif defined(__linux__)
#include <fstream>
#include <sstream>
#include <string>
#elif defined(__APPLE__)
#include <mach/mach.h>     // Required for task_info (current RSS)
#include <sys/resource.h>  // Required for getrusage
#endif

namespace d4 {

class MemoryStat {
 public:
  /**
   * @brief Retrieves the peak memory used by the current process.
   *
   * Automatically adapts to Windows (Peak Virtual Memory via psapi),
   * Linux (Peak Virtual Memory via /proc/self/status), and
   * macOS (Peak Physical Memory/RSS via getrusage).
   *
   * @return The peak memory usage in Megabytes (MB), or 0.0 if unsupported.
   */
  static double memUsedPeak() {
#if defined(_WIN32)

    PROCESS_MEMORY_COUNTERS pmc;
    if (GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc))) {
      // PeakPagefileUsage represents the peak virtual memory in bytes.
      return static_cast<double>(pmc.PeakPagefileUsage) / (1024.0 * 1024.0);
    }
    return 0.0;

#elif defined(__linux__)
    std::ifstream file("/proc/self/status");
    std::string line;

    while (std::getline(file, line)) {
      std::istringstream iss(line);
      std::string key;

      if (iss >> key && key == "VmPeak:") {
        double value = 0.0;

        if (iss >> value) {
          return value / 1024.0;
        }
      }
    }
    return 0.0;
#elif defined(__APPLE__)

    struct rusage usage;
    if (getrusage(RUSAGE_SELF, &usage) == 0) {
      // On macOS, ru_maxrss is exactly the peak physical memory in BYTES.
      // (Note: On Linux, ru_maxrss is in kilobytes, which is why we don't
      // use this function for Linux).
      return static_cast<double>(usage.ru_maxrss) / (1024.0 * 1024.0);
    }
    return 0.0;

#else
    // Fallback for any other unknown operating systems
    return 0.0;
#endif
  }

  /**
   * @brief Retrieves the memory the OS currently has resident for this
   * process (the Resident Set Size, i.e. the physical RAM actually backing
   * the process right now).
   *
   * Unlike memUsedPeak(), this is the *current* footprint as seen by the
   * system, not the high-water mark, and it reflects physical pages rather
   * than the virtual address space.
   *
   * Adapts to Windows (WorkingSetSize via psapi), Linux (VmRSS via
   * /proc/self/status) and macOS (resident_size via task_info).
   *
   * @return The current resident memory in Megabytes (MB), or 0.0 if
   * unsupported.
   */
  static double memResident() {
#if defined(_WIN32)

    PROCESS_MEMORY_COUNTERS pmc;
    if (GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc))) {
      // WorkingSetSize is the current resident set in bytes.
      return static_cast<double>(pmc.WorkingSetSize) / (1024.0 * 1024.0);
    }
    return 0.0;

#elif defined(__linux__)
    std::ifstream file("/proc/self/status");
    std::string line;

    while (std::getline(file, line)) {
      std::istringstream iss(line);
      std::string key;

      if (iss >> key && key == "VmRSS:") {
        double value = 0.0;

        // VmRSS is reported in kilobytes.
        if (iss >> value) {
          return value / 1024.0;
        }
      }
    }
    return 0.0;
#elif defined(__APPLE__)

    struct mach_task_basic_info info;
    mach_msg_type_number_t count = MACH_TASK_BASIC_INFO_COUNT;
    if (task_info(mach_task_self(), MACH_TASK_BASIC_INFO,
                  reinterpret_cast<task_info_t>(&info), &count) == KERN_SUCCESS) {
      // resident_size is the current resident set in bytes.
      return static_cast<double>(info.resident_size) / (1024.0 * 1024.0);
    }
    return 0.0;

#else
    // Fallback for any other unknown operating systems
    return 0.0;
#endif
  }
};

}  // namespace d4