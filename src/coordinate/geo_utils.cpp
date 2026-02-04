#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "geo_utils.hpp"

static constexpr double WGS84_A  = 6378137.0; // 타원체 장반경 a
static constexpr double WGS84_F  = 1.0 / 298.257223563; // 편평률 f
static constexpr double WGS84_E2 = WGS84_F * (2.0 - WGS84_F); // 이심률 제곱 e^2

UTM GeoUtils::Wgs84ToUtm(const WGS& wgs) {
  UTM out{};
  double gamma = 0.0, k = 0.0;

  GeographicLib::UTMUPS::Forward(wgs.lat_deg, wgs.lon_deg,out.zone, out.northp,out.E, out.N,gamma, k);
  out.alt_m = wgs.alt_m;
  return out;
}

WGS GeoUtils::UtmToWgs84(const UTM& utm) {
  WGS out{};
  double gamma = 0.0, k = 0.0;

  GeographicLib::UTMUPS::Reverse(utm.zone, utm.northp,utm.E, utm.N,out.lat_deg, out.lon_deg,gamma, k);
  out.alt_m = utm.alt_m;
  return out;
}

Vec GeoUtils::Wgs84ToEcef(const WGS& wgs) {
  const double lat = DegToRad(wgs.lat_deg);
  const double lon = DegToRad(wgs.lon_deg);
  const double h   = wgs.alt_m;

  const double sphi = std::sin(lat);
  const double cphi = std::cos(lat);
  const double slam = std::sin(lon);
  const double clam = std::cos(lon);

  const double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * sphi * sphi);

  Vec ecef;
  ecef.x = (N + h) * cphi * clam;
  ecef.y = (N + h) * cphi * slam;
  ecef.z = (N * (1.0 - WGS84_E2) + h) * sphi;
  return ecef;
}

Vec GeoUtils::EcefToEnu(const Vec& p_ecef, const Vec& ref_ecef,double ref_lat_deg, double ref_lon_deg) {
  const double lat0 = DegToRad(ref_lat_deg);
  const double lon0 = DegToRad(ref_lon_deg);

  const double sphi = std::sin(lat0);
  const double cphi = std::cos(lat0);
  const double slam = std::sin(lon0);
  const double clam = std::cos(lon0);

  // 기준점 기준 상대벡터(평행이동)
  const double dx = p_ecef.x - ref_ecef.x;
  const double dy = p_ecef.y - ref_ecef.y;
  const double dz = p_ecef.z - ref_ecef.z;

  // ENU 회전(축 투영)
  Vec enu;
  enu.x = -slam * dx + clam * dy;                         // East
  enu.y = -sphi*clam*dx - sphi*slam*dy + cphi*dz;         // North
  enu.z =  cphi*clam*dx + cphi*slam*dy + sphi*dz;         // Up
  return enu;
}

Vec GeoUtils::Wgs84ToEnu(const WGS& p, const WGS& origin) {
  const Vec p_ecef   = Wgs84ToEcef(p);
  const Vec org_ecef = Wgs84ToEcef(origin);
  return EcefToEnu(p_ecef, org_ecef, origin.lat_deg, origin.lon_deg);
}

Vec GeoUtils::UtmToEnu(const UTM& ego_utm, const WGS& origin) {
  const WGS ego_wgs = UtmToWgs84(ego_utm);
  return Wgs84ToEnu(ego_wgs, origin);
}

static tf2::Matrix3x3 EnuToNed_C() {
  return tf2::Matrix3x3(
    0, 1,  0,
    1, 0,  0,
    0, 0, -1
  );
}

RPY GeoUtils::EnuToNed(const RPY& enu) {
  tf2::Quaternion q_enu;
  q_enu.setRPY(enu.roll, enu.pitch, enu.yaw);
  tf2::Matrix3x3 R_enu(q_enu);
  tf2::Matrix3x3 R_ned = EnuToNed_C()* R_enu * EnuToNed_C().transpose();
  double r, p, y;
  R_ned.getRPY(r, p, y);
  return RPY{r, p, y};
}

RPY GeoUtils::NedToEnu(const RPY& ned) {
  tf2::Quaternion q_ned;
  q_ned.setRPY(ned.roll, ned.pitch, ned.yaw);
  tf2::Matrix3x3 R_ned(q_ned);
  tf2::Matrix3x3 R_enu = EnuToNed_C().transpose() * R_ned * EnuToNed_C();
  double r, p, y;
  R_enu.getRPY(r, p, y);
  return RPY{r, p, y};
}