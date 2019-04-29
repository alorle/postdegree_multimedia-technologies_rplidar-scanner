#ifndef SCANNER_H
#define SCANNER_H

#include "rplidar.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <memory>
#include <cmath>

#define SCAN_SAMPLES 8192

using namespace rp::standalone::rplidar;
class Scanner
{
  static uint32_t valid_baud_rates[2];

private:
  const std::string com_path;
  RPlidarDriver *rplidar;
  rplidar_response_device_info_t devinfo;
  rplidar_response_device_health_t healthinfo;

  pcl::PointCloud<pcl::PointXYZ> point_cloud;

public:
  explicit Scanner(const std::string &);

  ~Scanner();

  static std::shared_ptr<Scanner> make_shared(const std::string &);

  bool connect();

  bool isHealthy();

  void showInfo();

  void scan(const long int = 0, const long int = 50, const long int = 5);

  void save(const std::string &);

private:
  void initialize();

  void scanLevel(const long int, const long unsigned int = 5000);
};

#endif
