#pragma once

#include <mavix/v1/core/core.h>

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "mavix/v1/spatial/tile_declaration.h"

namespace mavix {
namespace v1 {
namespace spatial {

class TileUtility {
 private:
 protected:
  TileUtility(){};
  ~TileUtility(){};

 public:
  static Point<uint32_t> GetTileIndex(const double &lat, const double &lon,
                                      const uint8_t &zoom_level) {
    auto zoom = uint32_t(zoom_level);
    uint32_t shifted = ((1U << zoom) - 1U);

    auto tile_size = EARTH_CIRCUMFERENCE / static_cast<double>(zoom);
    auto mercator_coord = TransformWgs84ToWebMercator(lat, lon);

    auto col_calc = static_cast<uint32_t>(std::abs(
        (mercator_coord.lon - (EARTH_CIRCUMFERENCE * -0.5)) / tile_size));
    auto col = std::min(col_calc, shifted);

    auto row_calc = static_cast<uint32_t>(std::abs(
        ((EARTH_CIRCUMFERENCE * 0.5) - mercator_coord.lat) / tile_size));
    auto row = std::min(row_calc, shifted);

    return Point(col, row);
  }

  static Coord TransformWgs84ToWebMercator(const double &lat, const double &lon,
                                           const double &elevation = 0) {
    double mercator_lon = lon * PI / 180.0 * EARTH_RADIUS;
    double rad = lat * PI / 180.0;
    double sin = std::sin(rad);
    double mercator_lat =
        EARTH_RADIUS / 2.0 * std::log((1.0 + sin) / (1.0 - sin));

    return Coord(mercator_lat, mercator_lon, double(elevation),
                 KnownProjectionType::WebMercator);
  }

  static Coord TransformWebMercatorToWgs84(const double &lat, const double &lon,
                                           const double &elevation = 0) {
    double wgs_lon = ToDegrees(lon / EARTH_RADIUS);
    double wgs_lat = ToDegrees(std::atan(std::sinh(lat / EARTH_RADIUS)));

    return Coord(wgs_lat, wgs_lon, double(elevation),
                 KnownProjectionType::WGS84);
  }

  static double ToDegrees(const double &radians) {
    return radians * (180.0 / PI);
  }

  // static Rect<double> XyzWebMercatorToBboxWgs84(const uint32_t &min_x, const
  // uint32_t &min_y,
  //                                               const uint32_t &max_x, const
  //                                               uint32_t &max_y, uint8_t
  //                                               zoom_level) {
  //   auto lmin_X = uint32_t(min_x);
  //   auto lmin_y = uint32_t(min_y);
  //   auto lmax_X = uint32_t(max_x);
  //   auto lmax_y = uint32_t(max_y);

  //   auto zoom_divider = static_cast<double>(1U << zoom_level);
  //   double tile_length = EARTH_CIRCUMFERENCE / zoom_divider;

  //   auto left_down_bbox = TileBbox(lmin_X, lmax_y, tile_length);
  //   auto right_top_bbox = TileBbox(lmax_X, lmin_y, tile_length);

  //   Coord ldown_coord = TransformWebMercatorToWgs84();
  //   Coord rtop_coord = TransformWebMercatorToWgs84();

  //   return Rect<double>(ldown_coord.lon,
  //                       rtop_coord.lon,
  //                       ldown_coord.lat,
  //                       rtop_coord.lat,
  //                       KnownProjectionType::WGS84);
  // }
};

}  // namespace spatial
}  // namespace v1
}  // namespace mavix
