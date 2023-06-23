/**
 * @file system_stats.hpp
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief Utility classes and functions for monitoring system stats.
 *
 * Courtesy of https://github.com/doleron/cpp-linux-system-stats.
 */

#ifndef SYSTEM_STATS_HPP
#define SYSTEM_STATS_HPP

// stl include
#include <string>

/**
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief Class for monitoring cpu stats.
 *
 * Please checkout http://www.linuxhowtos.org/manpages/5/proc.htm for details.
 */
class CpuStats {
 public:
  /// @brief Function for updating cpu stats.
  void update();

  /**
   * @brief Function for getting the idel cpu time from boot to now.
   *
   * @return int Idel cpu time.
   */
  int get_idle_time() const;

  /**
   * @brief Function for getting the actitive cpu time from boot to now.
   *
   * @return int Active cpu time.
   */
  int get_active_time() const;

 private:
  int user;
  int nice;
  int system;
  int idle;
  int iowait;
  int irq;
  int softirq;
  int steal;
  int guest;
  int guest_nice;
};

/**
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief Class for monitoring memory stats.
 */
class MemoryStats {
 public:
  void update();

  /**
   * @brief Function for getting memory usage from 0 ~ 1.
   *
   * @return double Memory usage.
   */
  double get_memory_usage() const;

  /**
   * @brief Function for getting swap usage from 0 ~ 1.
   *
   * @return double Swap usage.
   */
  double get_swap_usage() const;

 private:
  static int get_memory_value(const std::string &target,
                              const std::string &content);

  int total_memory;
  int available_memory;
  int total_swap;
  int free_swap;
};

/**
 * @brief Function for getting the average cpu usage between two time point 1
 * and 2 from 0 ~ 1.
 *
 * @param t1 CPU stats at time point 1.
 * @param t2 CPU stats at time point 2.
 * @return double CPU usage.
 */
double get_cpu_usage(const CpuStats &t1, const CpuStats &t2);

/**
 * @brief Function for getting the disk usage from 0 ~ 1.
 *
 * @param disk The directory of the disk to check usage.
 * @return double Disk usage.
 */
double get_disk_usage(const std::string &disk);

/**
 * @brief Function for geting the thermalzone temperature in [C].
 *
 * @param thermal_index The index of the thermal zone.
 * @return double Thermalzoe temperature.
 */
double get_thermalzone_temperature(int thermal_index);

#endif  // SYSTEM_STATS_HPP
