// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/point_cloud_io.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <istream>
#include <optional>
#include <ostream>
#include <span>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace bagwiz::core::slam
{
namespace
{
void write_f32(std::ostream & os, float value)
{
  std::array<char, sizeof(float)> bytes{};
  std::memcpy(bytes.data(), &value, sizeof(float));
  os.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
}
}  // namespace

void write_pcd(
  std::ostream & os, std::span<const std::array<float, 3>> points,
  std::span<const float> intensities)
{
  const bool with_intensity = !intensities.empty() && intensities.size() == points.size();
  const char * const intensity_field = with_intensity ? " intensity" : "";
  const char * const intensity_size = with_intensity ? " 4" : "";
  const char * const intensity_type = with_intensity ? " F" : "";
  const char * const intensity_count = with_intensity ? " 1" : "";

  // Standard PCD v0.7 binary header. WIDTH/POINTS carry the count, HEIGHT 1
  // marks an unorganized cloud, and the identity VIEWPOINT keeps points in the
  // world frame. The body that follows is tightly packed little-endian float32.
  os << "# .PCD v0.7 - Point Cloud Data file format\n"
     << "VERSION 0.7\n"
     << "FIELDS x y z" << intensity_field << '\n'
     << "SIZE 4 4 4" << intensity_size << '\n'
     << "TYPE F F F" << intensity_type << '\n'
     << "COUNT 1 1 1" << intensity_count << '\n'
     << "WIDTH " << points.size() << '\n'
     << "HEIGHT 1\n"
     << "VIEWPOINT 0 0 0 1 0 0 0\n"
     << "POINTS " << points.size() << '\n'
     << "DATA binary\n";

  for (std::size_t i = 0; i < points.size(); ++i) {
    write_f32(os, points[i][0]);
    write_f32(os, points[i][1]);
    write_f32(os, points[i][2]);
    if (with_intensity) {
      write_f32(os, intensities[i]);
    }
  }
}

PcdReadResult read_pcd(std::istream & is)
{
  // Parse the PCD v0.7 header line by line until "DATA binary".
  std::unordered_map<std::string, std::string> header;
  std::string line;
  while (std::getline(is, line)) {
    if (line.empty()) {
      continue;
    }
    if (line[0] == '#') {
      continue;
    }
    const auto space = line.find(' ');
    const std::string key = (space == std::string::npos) ? line : line.substr(0, space);
    const std::string value = (space == std::string::npos) ? std::string{} : line.substr(space + 1);
    header[key] = value;
    if (key == "DATA") {
      break;
    }
  }

  auto get = [&](const char * key) -> std::optional<std::string> {
    const auto it = header.find(key);
    if (it == header.end()) {
      return std::nullopt;
    }
    return it->second;
  };

  const auto data = get("DATA");
  if (!data || *data != "binary") {
    return {false, {}, "PCD must use DATA binary"};
  }
  const auto version = get("VERSION");
  if (!version || *version != "0.7") {
    return {false, {}, "PCD must be version 0.7"};
  }
  const auto fields_str = get("FIELDS");
  if (!fields_str) {
    return {false, {}, "PCD header missing FIELDS"};
  }
  const auto size_str = get("SIZE");
  const auto type_str = get("TYPE");
  const auto count_str = get("COUNT");
  if (!size_str || !type_str || !count_str) {
    return {false, {}, "PCD header missing SIZE/TYPE/COUNT"};
  }

  std::istringstream fields_ss(*fields_str);
  std::istringstream size_ss(*size_str);
  std::istringstream type_ss(*type_str);
  std::istringstream count_ss(*count_str);
  std::vector<std::string> fields;
  std::vector<std::size_t> sizes;
  std::vector<char> types;
  std::vector<std::size_t> counts;
  std::string token;
  while (fields_ss >> token) {
    fields.push_back(token);
    std::size_t sz = 0;
    if (!(size_ss >> sz)) {
      return {false, {}, "PCD SIZE list too short"};
    }
    sizes.push_back(sz);
    char ty = 0;
    if (!(type_ss >> ty)) {
      return {false, {}, "PCD TYPE list too short"};
    }
    types.push_back(ty);
    std::size_t ct = 0;
    if (!(count_ss >> ct)) {
      return {false, {}, "PCD COUNT list too short"};
    }
    counts.push_back(ct);
  }

  if (fields.size() != 3 && fields.size() != 4) {
    return {false, {}, "PCD FIELDS must be x y z [intensity]"};
  }
  for (std::size_t i = 0; i < fields.size(); ++i) {
    if (sizes[i] != 4 || types[i] != 'F' || counts[i] != 1) {
      return {false, {}, "PCD fields must be float32 (F, size 4, count 1)"};
    }
  }
  const bool with_intensity = fields.size() == 4;
  if (with_intensity && fields[3] != "intensity") {
    return {false, {}, "PCD fourth field must be intensity"};
  }

  const auto width_str = get("WIDTH");
  const auto height_str = get("HEIGHT");
  const auto points_str = get("POINTS");
  if (!width_str || !height_str || !points_str) {
    return {false, {}, "PCD header missing WIDTH/HEIGHT/POINTS"};
  }
  std::size_t width = 0;
  std::size_t height = 0;
  std::size_t num_points = 0;
  {
    std::istringstream ss(*width_str);
    if (!(ss >> width)) {
      return {false, {}, "PCD WIDTH is not an integer"};
    }
  }
  {
    std::istringstream ss(*height_str);
    if (!(ss >> height)) {
      return {false, {}, "PCD HEIGHT is not an integer"};
    }
  }
  {
    std::istringstream ss(*points_str);
    if (!(ss >> num_points)) {
      return {false, {}, "PCD POINTS is not an integer"};
    }
  }
  if (width * height != num_points) {
    return {false, {}, "PCD WIDTH * HEIGHT does not match POINTS"};
  }

  PcdCloud cloud;
  cloud.points.reserve(num_points);
  if (with_intensity) {
    cloud.intensities.reserve(num_points);
  }
  constexpr std::size_t kFloatBytes = 4;
  const std::size_t point_bytes = fields.size() * kFloatBytes;
  std::vector<char> buffer(point_bytes);
  for (std::size_t i = 0; i < num_points; ++i) {
    if (!is.read(buffer.data(), static_cast<std::streamsize>(point_bytes))) {
      return {false, {}, "PCD body truncated"};
    }
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
    std::memcpy(&x, buffer.data() + 0 * kFloatBytes, kFloatBytes);
    std::memcpy(&y, buffer.data() + 1 * kFloatBytes, kFloatBytes);
    std::memcpy(&z, buffer.data() + 2 * kFloatBytes, kFloatBytes);
    cloud.points.push_back({x, y, z});
    if (with_intensity) {
      float intensity = 0.0F;
      std::memcpy(&intensity, buffer.data() + 3 * kFloatBytes, kFloatBytes);
      cloud.intensities.push_back(intensity);
    }
  }

  return {true, std::move(cloud), {}};
}

}  // namespace bagwiz::core::slam
