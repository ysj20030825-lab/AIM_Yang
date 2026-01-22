#pragma once //헤더 중복 include 방지

#include <GeographicLib/UTMUPS.hpp> 
#include <cmath>           
#include <string>

struct WGS {
  double lat_deg; // 위도 
  double lon_deg; // 경도 
  double alt_m;   // 고도 
};

struct UTM {
  double E;       // East
  double N;       // North
  int zone;       // UTM zone
  bool northp;    // 북반구면 true
  double alt_m;   // 고도
};

struct RPY { double roll, pitch, yaw; };

struct Vec {
  double x, y, z; // 일반 벡터 (ENU 결과 등)
};

class GeoUtils {
public:
  static UTM Wgs84ToUtm(const WGS& wgs);

  static WGS UtmToWgs84(const UTM& utm);

  static Vec Wgs84ToEcef(const WGS& wgs);

  static Vec EcefToEnu(const Vec& p_ecef, const Vec& ref_ecef, double ref_lat_deg, double ref_lon_deg);

  static Vec Wgs84ToEnu(const WGS& p, const WGS& origin);

  static Vec UtmToEnu(const UTM& ego_utm, const WGS& origin);

  static RPY EnuToNed(const RPY& enu);

  static RPY NedToEnu(const RPY& ned);

private:
  static double DegToRad(double d) { return d * M_PI / 180.0; }
};
