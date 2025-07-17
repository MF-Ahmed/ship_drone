#include "crazyflie_ekf/gps2enu.hpp"
#include <cmath>

namespace crazyflie_ekf
{

Gps2Enu::Gps2Enu()
: initialized_(false)
{
}

void Gps2Enu::initialize(double ref_lat, double ref_lon, double ref_alt)
{
  ref_lat_ = ref_lat;
  ref_lon_ = ref_lon;
  ref_alt_ = ref_alt;
  initialized_ = true;

  constexpr double deg2rad = M_PI / 180.0;
  ref_lat_rad_ = ref_lat * deg2rad;

  // Earth radius in meters
  constexpr double earth_radius = 6378137.0;
  radius_lat_ = earth_radius;
  radius_lon_ = earth_radius * std::cos(ref_lat_rad_);
}

bool Gps2Enu::isInitialized() const
{
  return initialized_;
}

geometry_msgs::msg::Point Gps2Enu::convert(double lat, double lon, double alt) const
{
  geometry_msgs::msg::Point enu;

  if (!initialized_) {
    return enu;
  }

  constexpr double deg2rad = M_PI / 180.0;
  double d_lat = (lat - ref_lat_) * deg2rad;
  double d_lon = (lon - ref_lon_) * deg2rad;

  enu.x = d_lon * radius_lon_;
  enu.y = d_lat * radius_lat_;
  enu.z = alt - ref_alt_;

  return enu;
}

}  // namespace crazyflie_ekf
