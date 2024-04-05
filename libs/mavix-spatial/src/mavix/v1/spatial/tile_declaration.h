#pragma once

#include <mavix/v1/core/core.h>

#include <cmath>
#include <cstdint>
#include "nvm/macro.h"

namespace mavix {
namespace v1 {
namespace spatial {

constexpr static double EARTH_CIRCUMFERENCE = 40075016.6855785;
constexpr static double PI = 3.14159265358979323846;
constexpr static double EARTH_RADIUS = EARTH_CIRCUMFERENCE / 2.0 / PI;
const static uint8_t MAX_ZOOM = 30;

enum class TileFormat {
  Unknown = 0,
  Png = 1,
  Jpeg = 2,
  Webp = 3,
  Mvt = 4,
  Json = 5
};

// cppcheck-suppress unknownMacro
NVM_ENUM_CLASS_DISPLAY_TRAIT(TileFormat)

enum class KnownProjectionType {
  Unknown = 0,
  WGS84 = 1,
  WebMercator = 2,
  Others = 3
};

NVM_ENUM_CLASS_DISPLAY_TRAIT(KnownProjectionType)

template <typename T>
struct Point {
  T x;  //< Equivalent with Lon in WGS84/WebMercator
  T y;  //< Equivalent with Lat in WGS84/WebMercator
  T z;  //< Equivalent with Elevation in WGS84/WebMercator

  Point() : x(), y(), z() {}
  explicit Point(T x, T y, T z = 0) : x(x), y(y), z(z) {}
};

template <typename T>
struct Rect {
  T left;
  T top;
  T right;
  T bottom;
  uint16_t epsg;  //< Have value only if KnownProjectionType::Custom
  KnownProjectionType projection;

  Rect()
      : left(),
        top(),
        right(),
        bottom(),
        epsg(),
        projection(KnownProjectionType::Unknown){};

  explicit Rect(T left, T top, T right, T bottom,
                KnownProjectionType projection = KnownProjectionType::Unknown,
                uint16_t epsg = 0)
      : left(left),
        top(top),
        right(right),
        bottom(bottom),
        epsg(epsg),
        projection(projection){};
};

struct Coord {
  double lat;        //< Equivalent with Y in carthesian
  double lon;        //< Equivalent with X in carthesian
  double elevation;  //< Equivalent with Z in carthesian
  uint16_t epsg;     //< Have value only if KnownProjectionType::Custom
  KnownProjectionType projection;

  Coord()
      : lat(),
        lon(),
        elevation(),
        epsg(),
        projection(KnownProjectionType::Unknown) {}

  explicit Coord(double lat, double lon, double z,
                 KnownProjectionType projection, uint16_t epsg = 0)
      : lat(lat), lon(lon), elevation(z), epsg(epsg), projection(projection) {}
};

}  // namespace spatial
}  // namespace v1
}  // namespace mavix