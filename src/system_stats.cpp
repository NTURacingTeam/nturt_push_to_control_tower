#include "nturt_push_to_control_tower/system_stats.hpp"

// glibc include
#include <sys/statvfs.h>

// stl include
#include <fstream>
#include <sstream>
#include <string>

void CpuStats::update() {
  std::ifstream proc_stat("/proc/stat");

  if (proc_stat.good()) {
    std::string line;
    std::getline(proc_stat, line);

    std::stringstream iss(line);
    std::string cpu;

    // flush out "cpu"
    iss >> cpu;

    int* stats_ptr = &user;
    while (iss >> *stats_ptr) {
      stats_ptr++;
    };
  }

  proc_stat.close();
}

int CpuStats::get_idle_time() const { return idle + iowait; }

int CpuStats::get_active_time() const {
  return user + nice + system + irq + softirq + steal + guest + guest_nice;
}

void MemoryStats::update() {
  std::ifstream proc_meminfo("/proc/meminfo");

  if (proc_meminfo.good()) {
    std::string content((std::istreambuf_iterator<char>(proc_meminfo)),
                        std::istreambuf_iterator<char>());

    total_memory = get_memory_value("MemTotal:", content);
    total_swap = get_memory_value("SwapTotal:", content);
    free_swap = get_memory_value("SwapFree:", content);
    available_memory = get_memory_value("MemAvailable:", content);
  }

  proc_meminfo.close();
}

double MemoryStats::get_memory_usage() const {
  return static_cast<double>(total_memory - available_memory) / total_memory;
}

double MemoryStats::get_swap_usage() const {
  return static_cast<double>(total_swap - free_swap) / total_swap;
}

int MemoryStats::get_memory_value(const std::string& target,
                                  const std::string& content) {
  int result = -1;
  std::size_t start = content.find(target);
  if (start != std::string::npos) {
    int begin = start + target.length();
    std::size_t end = content.find("kB", start);
    std::string substr = content.substr(begin, end - begin);
    result = std::stoi(substr);
  }
  return result;
}

double get_cpu_usage(const CpuStats& t1, const CpuStats& t2) {
  const double active_time =
      static_cast<double>(t2.get_active_time() - t1.get_active_time());
  const double idle_time =
      static_cast<double>(t2.get_idle_time() - t1.get_idle_time());
  return active_time / (active_time + idle_time);
}

double get_disk_usage(const std::string& disk) {
  struct statvfs diskData;

  statvfs(disk.c_str(), &diskData);

  return static_cast<double>(diskData.f_blocks - diskData.f_bfree) /
         diskData.f_blocks;
}

double get_thermalzone_temperature(int thermal_index) {
  int result = -1;
  std::ifstream thermal_file("/sys/class/thermal/thermal_zone" +
                             std::to_string(thermal_index) + "/temp");

  if (thermal_file.good()) {
    std::string line;
    getline(thermal_file, line);

    std::stringstream iss(line);
    iss >> result;
  } else {
    throw std::invalid_argument(std::to_string(thermal_index) +
                                " doesn't refer to a valid thermal zone.");
  }

  thermal_file.close();

  return result / 1000.0;
}
