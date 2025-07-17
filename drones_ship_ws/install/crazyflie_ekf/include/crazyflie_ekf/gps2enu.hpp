#ifndef CRAZYFLIE_EKF__GPS2ENU_HPP_
#define CRAZYFLIE_EKF__GPS2ENU_HPP_

#include "geometry_msgs/msg/point.hpp"

namespace crazyflie_ekf
{

class Gps2Enu
{
public:
  Gps2Enu();  // <-- This was missing

  void initialize(double ref_lat, double ref_lon, double ref_alt);
  bool isInitialized() const;
  geometry_msgs::msg::Point convert(double lat, double lon, double alt) const;

private:
  bool initialized_;
  double ref_lat_, ref_lon_, ref_alt_;
  double ref_lat_rad_;
  double radius_lat_, radius_lon_;
};

}  // namespace crazyflie_ekf

#endif  // CRAZYFLIE_EKF__GPS2ENU_HPP_
