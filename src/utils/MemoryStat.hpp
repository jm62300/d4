#pragma once

// --- Include OS-specific headers ---
#if defined(_WIN32)
#include <psapi.h>  // Required for GetProcessMemoryInfo
#include <windows.h>
#elif defined(__linux__)
#include <fstream>
#include <string>
#elif defined(__APPLE__)
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
};

}  // namespace d4