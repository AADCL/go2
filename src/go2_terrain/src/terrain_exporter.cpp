#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include <pcl/common/point_tests.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <iostream>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <boost/filesystem.hpp>

#include "go2_terrain/terrain_algorithms.hpp"
#include "go2_terrain/terrain_grid.hpp"

namespace go2_terrain
{
namespace
{

constexpr std::uint8_t kUnknownImage = 205;
constexpr std::uint8_t kFreeImage = 254;
constexpr std::uint8_t kOccupiedImage = 0;
constexpr double kPi = 3.14159265358979323846;

bool validExportId(const std::string& value)
{
  if (value.empty() || value.size() > 128 ||
      !std::isalnum(static_cast<unsigned char>(value.front())))
  {
    return false;
  }
  return std::all_of(value.begin(), value.end(), [](char character) {
    const unsigned char byte = static_cast<unsigned char>(character);
    return std::isalnum(byte) || character == '_' || character == '-';
  });
}

struct MapDefinition
{
  GridGeometry geometry;
  std::string input_image_path;
  double occupied_threshold = 0.65;
  double free_threshold = 0.196;
  int negate = 0;
};

struct ExportParameters
{
  double required_resolution = 0.05;
  double map_padding_m = 0.50;
  double base_search_radius = 1.0;
  double base_min_height = 0.20;
  double base_max_height = 0.55;
  int base_min_points = 20;
  double candidate_quantile = 0.05;
  double candidate_support_band = 0.12;
  double candidate_max_vertical_span = 0.25;
  double trajectory_seed_radius = 0.25;
  double seed_height_tolerance = 0.15;
  double seed_cell_height_tolerance = 0.04;
  double maximum_trajectory_gap = 0.50;
  double maximum_reanchor_height_from_initial = 0.65;
  double growth_height_margin = 0.005;
  int growth_fill_iterations = 3;
  double max_ground_slope_deg = 35.0;
  double plane_radius = 0.30;
  int plane_min_cells = 4;
  double plane_huber_m = 0.03;
  double max_plane_rmse = 0.08;
  double obstacle_min_height = 0.05;
  double obstacle_max_height = 1.50;
  double obstacle_ground_search = 0.30;
  int min_obstacle_points = 1;
  double trajectory_free_radius = 0.18;
  double obstacle_inflation_m = 0.03;
  bool preserve_existing_map = true;
  int minimum_ground_cells = 100;
  double minimum_traced_trajectory_ratio = 0.80;
  double minimum_ground_observation_ratio = 0.30;
  double minimum_largest_ground_component_ratio = 0.60;
  double minimum_ground_to_baseline_free_ratio = 0.08;
  double minimum_trajectory_corridor_known_ratio = 0.95;
  CostParameters cost;
  int pmf_max_window_size = 20;
  double pmf_slope = 0.7;
  double pmf_initial_distance = 0.08;
  double pmf_max_distance = 0.30;
};

std::size_t cellIndex(int x, int y, const GridGeometry& geometry)
{
  return static_cast<std::size_t>(y) * geometry.width +
         static_cast<std::size_t>(x);
}

bool inside(int x, int y, const GridGeometry& geometry)
{
  return x >= 0 && y >= 0 && x < static_cast<int>(geometry.width) &&
         y < static_cast<int>(geometry.height);
}

bool pointToCell(double x,
                 double y,
                 const GridGeometry& geometry,
                 int* grid_x,
                 int* grid_y)
{
  const int gx = static_cast<int>(
      std::floor((x - geometry.origin_x) / geometry.resolution));
  const int gy = static_cast<int>(
      std::floor((y - geometry.origin_y) / geometry.resolution));
  if (!inside(gx, gy, geometry))
  {
    return false;
  }
  *grid_x = gx;
  *grid_y = gy;
  return true;
}

std::string readPgmToken(std::istream* input)
{
  std::string token;
  char c = '\0';
  while (input->get(c))
  {
    if (c == '#')
    {
      input->ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      continue;
    }
    if (!std::isspace(static_cast<unsigned char>(c)))
    {
      token.push_back(c);
      break;
    }
  }
  while (input->get(c))
  {
    if (std::isspace(static_cast<unsigned char>(c)))
    {
      if (c == '\r' && input->peek() == '\n')
      {
        input->get(c);
      }
      break;
    }
    token.push_back(c);
  }
  return token;
}

std::pair<std::size_t, std::size_t> readPgmDimensions(
    const std::string& path)
{
  std::ifstream input(path.c_str(), std::ios::binary);
  if (!input)
  {
    throw std::runtime_error("Cannot open map image: " + path);
  }
  const std::string magic = readPgmToken(&input);
  if (magic != "P5" && magic != "P2")
  {
    throw std::runtime_error("Only P5/P2 PGM map images are supported: " + path);
  }
  const long width = std::stol(readPgmToken(&input));
  const long height = std::stol(readPgmToken(&input));
  const long maximum = std::stol(readPgmToken(&input));
  if (width <= 0 || height <= 0 || maximum <= 0 || maximum > 255)
  {
    throw std::runtime_error("Invalid PGM header: " + path);
  }
  return {static_cast<std::size_t>(width),
          static_cast<std::size_t>(height)};
}

MapDefinition loadMapDefinition(const std::string& map_yaml)
{
  const YAML::Node root = YAML::LoadFile(map_yaml);
  MapDefinition result;
  result.geometry.resolution = root["resolution"].as<double>();
  const YAML::Node origin = root["origin"];
  if (!origin || !origin.IsSequence() || origin.size() != 3)
  {
    throw std::runtime_error("map.yaml origin must have three values");
  }
  result.geometry.origin_x = origin[0].as<double>();
  result.geometry.origin_y = origin[1].as<double>();
  result.geometry.origin_yaw = origin[2].as<double>();
  if (std::fabs(result.geometry.origin_yaw) > 1e-9)
  {
    throw std::runtime_error("Rotated map.yaml origins are not supported");
  }
  result.negate = root["negate"] ? root["negate"].as<int>() : 0;
  if (result.negate != 0)
  {
    throw std::runtime_error(
        "map.yaml negate must be 0; terrain export writes standard occupancy pixels");
  }
  result.occupied_threshold =
      root["occupied_thresh"] ? root["occupied_thresh"].as<double>() : 0.65;
  result.free_threshold =
      root["free_thresh"] ? root["free_thresh"].as<double>() : 0.196;

  const boost::filesystem::path yaml_path(map_yaml);
  const boost::filesystem::path image(root["image"].as<std::string>());
  result.input_image_path = image.is_absolute()
                                ? image.string()
                                : (yaml_path.parent_path() / image).string();
  const auto dimensions = readPgmDimensions(result.input_image_path);
  result.geometry.width = dimensions.first;
  result.geometry.height = dimensions.second;
  if (!result.geometry.valid())
  {
    throw std::runtime_error("Invalid geometry in map.yaml");
  }
  return result;
}

std::vector<std::uint8_t> readMapOccupancy(const MapDefinition& map)
{
  std::ifstream input(map.input_image_path.c_str(), std::ios::binary);
  if (!input)
  {
    throw std::runtime_error("Cannot open map image: " + map.input_image_path);
  }
  const std::string magic = readPgmToken(&input);
  if (magic != "P5" && magic != "P2")
  {
    throw std::runtime_error("Only P5/P2 PGM map images are supported: " +
                             map.input_image_path);
  }
  const long width = std::stol(readPgmToken(&input));
  const long height = std::stol(readPgmToken(&input));
  const long maximum = std::stol(readPgmToken(&input));
  if (width != static_cast<long>(map.geometry.width) ||
      height != static_cast<long>(map.geometry.height) ||
      maximum <= 0 || maximum > 255)
  {
    throw std::runtime_error("Map image geometry or range is invalid: " +
                             map.input_image_path);
  }

  const std::size_t count = map.geometry.cellCount();
  std::vector<std::uint8_t> pixels(count, 0U);
  if (magic == "P5")
  {
    input.read(reinterpret_cast<char*>(pixels.data()),
               static_cast<std::streamsize>(pixels.size()));
    if (input.gcount() != static_cast<std::streamsize>(pixels.size()))
    {
      throw std::runtime_error("Map image is truncated: " +
                               map.input_image_path);
    }
  }
  else
  {
    for (std::size_t i = 0; i < count; ++i)
    {
      const long value = std::stol(readPgmToken(&input));
      if (value < 0 || value > maximum)
      {
        throw std::runtime_error("Map image pixel is outside its range: " +
                                 map.input_image_path);
      }
      pixels[i] = static_cast<std::uint8_t>(value);
    }
  }

  std::vector<std::uint8_t> occupancy(count, kUnknownImage);
  for (std::size_t image_y = 0; image_y < map.geometry.height; ++image_y)
  {
    const std::size_t grid_y = map.geometry.height - image_y - 1U;
    for (std::size_t x = 0; x < map.geometry.width; ++x)
    {
      const std::uint8_t pixel =
          pixels[image_y * map.geometry.width + x];
      const double normalized = static_cast<double>(pixel) /
                                static_cast<double>(maximum);
      const double occupied_probability =
          map.negate != 0 ? normalized : 1.0 - normalized;
      const std::size_t destination = grid_y * map.geometry.width + x;
      if (occupied_probability > map.occupied_threshold)
      {
        occupancy[destination] = kOccupiedImage;
      }
      else if (occupied_probability < map.free_threshold)
      {
        occupancy[destination] = kFreeImage;
      }
    }
  }
  return occupancy;
}

MapDefinition deriveMapDefinition(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                  const ExportParameters& parameters)
{
  double minimum_x = std::numeric_limits<double>::infinity();
  double minimum_y = std::numeric_limits<double>::infinity();
  double maximum_x = -std::numeric_limits<double>::infinity();
  double maximum_y = -std::numeric_limits<double>::infinity();
  for (const auto& point : cloud.points)
  {
    if (!pcl::isFinite(point))
    {
      continue;
    }
    minimum_x = std::min(minimum_x, static_cast<double>(point.x));
    minimum_y = std::min(minimum_y, static_cast<double>(point.y));
    maximum_x = std::max(maximum_x, static_cast<double>(point.x));
    maximum_y = std::max(maximum_y, static_cast<double>(point.y));
  }
  if (!std::isfinite(minimum_x) || !std::isfinite(minimum_y) ||
      !std::isfinite(maximum_x) || !std::isfinite(maximum_y))
  {
    throw std::runtime_error("public_map.pcd contains no finite XYZ points");
  }

  MapDefinition result;
  result.geometry.resolution = parameters.required_resolution;
  result.geometry.origin_x = minimum_x - parameters.map_padding_m;
  result.geometry.origin_y = minimum_y - parameters.map_padding_m;
  result.geometry.origin_yaw = 0.0;
  const double padded_maximum_x = maximum_x + parameters.map_padding_m;
  const double padded_maximum_y = maximum_y + parameters.map_padding_m;
  result.geometry.width = static_cast<std::size_t>(std::ceil(
                              (padded_maximum_x - result.geometry.origin_x) /
                              result.geometry.resolution)) +
                          1U;
  result.geometry.height = static_cast<std::size_t>(std::ceil(
                               (padded_maximum_y - result.geometry.origin_y) /
                               result.geometry.resolution)) +
                           1U;
  if (!result.geometry.valid())
  {
    throw std::runtime_error("Failed to derive valid map geometry from public map");
  }
  return result;
}

float median(std::vector<float> values)
{
  if (values.empty())
  {
    return std::numeric_limits<float>::quiet_NaN();
  }
  const std::size_t middle = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  float result = values[middle];
  if (values.size() % 2 == 0)
  {
    const auto lower = std::max_element(values.begin(), values.begin() + middle);
    result = 0.5F * (result + *lower);
  }
  return result;
}

double estimateBaseToFloor(
    const pcl::PointCloud<pcl::PointXYZ>& cloud,
    const pcl::PointCloud<pcl::PointXYZ>& trajectory,
    const ExportParameters& parameters)
{
  if (trajectory.empty())
  {
    throw std::runtime_error(
        "traversed_path_map.pcd is empty; base-to-floor cannot be estimated safely");
  }
  const pcl::PointXYZ& start = trajectory.front();
  const double radius_squared =
      parameters.base_search_radius * parameters.base_search_radius;
  const double bin_width = 0.01;
  const int bin_count = static_cast<int>(std::ceil(
      (parameters.base_max_height - parameters.base_min_height) / bin_width)) + 1;
  std::vector<std::vector<float>> bins(static_cast<std::size_t>(bin_count));

  for (const auto& point : cloud.points)
  {
    if (!pcl::isFinite(point))
    {
      continue;
    }
    const double dx = point.x - start.x;
    const double dy = point.y - start.y;
    if (dx * dx + dy * dy > radius_squared)
    {
      continue;
    }
    // /odom_nav deliberately flattens Z. FAST-LIO starts with base_link at
    // map Z=0, so only trajectory XY is used and initial floor is measured
    // below that zero reference.
    const double height = -static_cast<double>(point.z);
    if (height < parameters.base_min_height ||
        height > parameters.base_max_height)
    {
      continue;
    }
    const int bin = std::max(
        0,
        std::min(bin_count - 1,
                 static_cast<int>((height - parameters.base_min_height) /
                                  bin_width)));
    bins[static_cast<std::size_t>(bin)].push_back(static_cast<float>(height));
  }

  std::size_t best = 0;
  for (std::size_t i = 1; i < bins.size(); ++i)
  {
    if (bins[i].size() > bins[best].size())
    {
      best = i;
    }
  }
  if (bins[best].size() < static_cast<std::size_t>(parameters.base_min_points))
  {
    std::ostringstream message;
    message << "Only " << bins[best].size()
            << " consistent floor samples near trajectory start; need "
            << parameters.base_min_points;
    throw std::runtime_error(message.str());
  }

  std::vector<float> support;
  for (int offset = -1; offset <= 1; ++offset)
  {
    const int index = static_cast<int>(best) + offset;
    if (index >= 0 && index < bin_count)
    {
      support.insert(support.end(), bins[static_cast<std::size_t>(index)].begin(),
                     bins[static_cast<std::size_t>(index)].end());
    }
  }
  const double result = median(std::move(support));
  if (!std::isfinite(result) || result < parameters.base_min_height ||
      result > parameters.base_max_height)
  {
    throw std::runtime_error("Estimated base-to-floor height is outside safe bounds");
  }
  return result;
}

void markDisk(double world_x,
              double world_y,
              double radius_m,
              const GridGeometry& geometry,
              std::vector<std::uint8_t>* mask)
{
  int center_x = 0;
  int center_y = 0;
  if (!pointToCell(world_x, world_y, geometry, &center_x, &center_y))
  {
    return;
  }
  const int radius_cells =
      std::max(0, static_cast<int>(std::ceil(radius_m / geometry.resolution)));
  const double radius_sq = radius_m * radius_m +
                           geometry.resolution * geometry.resolution * 0.5;
  for (int dy = -radius_cells; dy <= radius_cells; ++dy)
  {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx)
    {
      const int x = center_x + dx;
      const int y = center_y + dy;
      if (!inside(x, y, geometry))
      {
        continue;
      }
      const double metric_x = dx * geometry.resolution;
      const double metric_y = dy * geometry.resolution;
      if (metric_x * metric_x + metric_y * metric_y <= radius_sq)
      {
        (*mask)[cellIndex(x, y, geometry)] = 1;
      }
    }
  }
}

void markSegment(double start_x,
                 double start_y,
                 double end_x,
                 double end_y,
                 double radius_m,
                 const GridGeometry& geometry,
                 std::vector<std::uint8_t>* mask)
{
  const double distance = std::hypot(end_x - start_x, end_y - start_y);
  const double spacing = std::max(geometry.resolution * 0.5, 0.01);
  const int steps = std::max(1, static_cast<int>(std::ceil(distance / spacing)));
  for (int step = 0; step <= steps; ++step)
  {
    const double ratio = static_cast<double>(step) /
                         static_cast<double>(steps);
    markDisk(start_x + ratio * (end_x - start_x),
             start_y + ratio * (end_y - start_y),
             radius_m,
             geometry,
             mask);
  }
}

std::vector<std::uint8_t> buildTrajectoryMask(
    const std::vector<PlanarPoint>& trajectory,
    double radius_m,
    double maximum_gap_m,
    const GridGeometry& geometry)
{
  std::vector<std::uint8_t> mask(geometry.cellCount(), 0U);
  for (std::size_t i = 0; i < trajectory.size(); ++i)
  {
    markDisk(trajectory[i].x, trajectory[i].y, radius_m, geometry, &mask);
    if (i == 0)
    {
      continue;
    }
    const double segment_length = std::hypot(
        trajectory[i].x - trajectory[i - 1].x,
        trajectory[i].y - trajectory[i - 1].y);
    if (segment_length <= maximum_gap_m)
    {
      markSegment(trajectory[i - 1].x,
                  trajectory[i - 1].y,
                  trajectory[i].x,
                  trajectory[i].y,
                  radius_m,
                  geometry,
                  &mask);
    }
  }
  return mask;
}

class Sha256
{
public:
  Sha256() { reset(); }

  void update(const std::uint8_t* data, std::size_t length)
  {
    for (std::size_t i = 0; i < length; ++i)
    {
      block_[block_size_++] = data[i];
      bit_length_ += 8;
      if (block_size_ == 64)
      {
        transform();
        block_size_ = 0;
      }
    }
  }

  std::string finish()
  {
    block_[block_size_++] = 0x80;
    if (block_size_ > 56)
    {
      while (block_size_ < 64)
      {
        block_[block_size_++] = 0;
      }
      transform();
      block_size_ = 0;
    }
    while (block_size_ < 56)
    {
      block_[block_size_++] = 0;
    }
    for (int i = 7; i >= 0; --i)
    {
      block_[block_size_++] =
          static_cast<std::uint8_t>((bit_length_ >> (i * 8)) & 0xffU);
    }
    transform();

    std::ostringstream output;
    output << std::hex << std::setfill('0');
    for (const std::uint32_t value : state_)
    {
      output << std::setw(8) << value;
    }
    return output.str();
  }

private:
  static std::uint32_t rotateRight(std::uint32_t value, std::uint32_t count)
  {
    return (value >> count) | (value << (32U - count));
  }

  void reset()
  {
    state_ = {{0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U, 0xa54ff53aU,
               0x510e527fU, 0x9b05688cU, 0x1f83d9abU, 0x5be0cd19U}};
    block_.fill(0);
    block_size_ = 0;
    bit_length_ = 0;
  }

  void transform()
  {
    static const std::uint32_t constants[64] = {
        0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U,
        0x3956c25bU, 0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U,
        0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U,
        0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U, 0xc19bf174U,
        0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU,
        0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU,
        0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U,
        0xc6e00bf3U, 0xd5a79147U, 0x06ca6351U, 0x14292967U,
        0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU, 0x53380d13U,
        0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
        0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U,
        0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U,
        0x19a4c116U, 0x1e376c08U, 0x2748774cU, 0x34b0bcb5U,
        0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU, 0x682e6ff3U,
        0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U,
        0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U};
    std::uint32_t words[64] = {};
    for (int i = 0; i < 16; ++i)
    {
      words[i] = (static_cast<std::uint32_t>(block_[i * 4]) << 24U) |
                 (static_cast<std::uint32_t>(block_[i * 4 + 1]) << 16U) |
                 (static_cast<std::uint32_t>(block_[i * 4 + 2]) << 8U) |
                 static_cast<std::uint32_t>(block_[i * 4 + 3]);
    }
    for (int i = 16; i < 64; ++i)
    {
      const std::uint32_t s0 = rotateRight(words[i - 15], 7) ^
                               rotateRight(words[i - 15], 18) ^
                               (words[i - 15] >> 3U);
      const std::uint32_t s1 = rotateRight(words[i - 2], 17) ^
                               rotateRight(words[i - 2], 19) ^
                               (words[i - 2] >> 10U);
      words[i] = words[i - 16] + s0 + words[i - 7] + s1;
    }

    std::uint32_t a = state_[0];
    std::uint32_t b = state_[1];
    std::uint32_t c = state_[2];
    std::uint32_t d = state_[3];
    std::uint32_t e = state_[4];
    std::uint32_t f = state_[5];
    std::uint32_t g = state_[6];
    std::uint32_t h = state_[7];
    for (int i = 0; i < 64; ++i)
    {
      const std::uint32_t sum1 = rotateRight(e, 6) ^ rotateRight(e, 11) ^
                                 rotateRight(e, 25);
      const std::uint32_t choice = (e & f) ^ ((~e) & g);
      const std::uint32_t temp1 = h + sum1 + choice + constants[i] + words[i];
      const std::uint32_t sum0 = rotateRight(a, 2) ^ rotateRight(a, 13) ^
                                 rotateRight(a, 22);
      const std::uint32_t majority = (a & b) ^ (a & c) ^ (b & c);
      const std::uint32_t temp2 = sum0 + majority;
      h = g;
      g = f;
      f = e;
      e = d + temp1;
      d = c;
      c = b;
      b = a;
      a = temp1 + temp2;
    }
    state_[0] += a;
    state_[1] += b;
    state_[2] += c;
    state_[3] += d;
    state_[4] += e;
    state_[5] += f;
    state_[6] += g;
    state_[7] += h;
  }

  std::array<std::uint32_t, 8> state_;
  std::array<std::uint8_t, 64> block_;
  std::size_t block_size_ = 0;
  std::uint64_t bit_length_ = 0;
};

std::string sha256File(const std::string& path)
{
  std::ifstream input(path.c_str(), std::ios::binary);
  if (!input)
  {
    throw std::runtime_error("Cannot checksum file: " + path);
  }
  Sha256 sha;
  std::array<char, 1024 * 1024> buffer;
  while (input)
  {
    input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const std::streamsize count = input.gcount();
    if (count > 0)
    {
      sha.update(reinterpret_cast<const std::uint8_t*>(buffer.data()),
                 static_cast<std::size_t>(count));
    }
  }
  if (!input.eof())
  {
    throw std::runtime_error("Failed while checksumming file: " + path);
  }
  return sha.finish();
}

std::map<std::string, std::string> readChecksumFile(const std::string& path)
{
  std::ifstream input(path.c_str());
  if (!input)
  {
    throw std::runtime_error("Cannot read checksum file: " + path);
  }
  std::map<std::string, std::string> result;
  std::string hash;
  std::string file;
  while (input >> hash >> file)
  {
    if (hash.size() != 64 || file.empty() || file.find('/') != std::string::npos ||
        file.find('\\') != std::string::npos || file == "." || file == "..")
    {
      throw std::runtime_error("Malformed checksum entry in: " + path);
    }
    if (!std::all_of(hash.begin(), hash.end(), [](char c) {
          return std::isxdigit(static_cast<unsigned char>(c)) != 0;
        }))
    {
      throw std::runtime_error("Checksum is not hexadecimal in: " + path);
    }
    std::transform(hash.begin(), hash.end(), hash.begin(), [](char c) {
      return static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    });
    if (!result.emplace(file, hash).second)
    {
      throw std::runtime_error("Duplicate checksum entry for " + file);
    }
  }
  return result;
}

void verifyMappingSnapshot(const std::string& marker,
                           const std::string& public_map,
                           const std::string& trajectory)
{
  const std::map<std::string, std::string> checksums = readChecksumFile(marker);
  const std::set<std::string> expected = {
      "public_map.pcd", "traversed_path_map.pcd"};
  if (checksums.size() != expected.size())
  {
    throw std::runtime_error(
        "mapping_snapshot.sha256 must contain exactly the two mapping inputs");
  }
  for (const auto& file : expected)
  {
    const auto found = checksums.find(file);
    if (found == checksums.end())
    {
      throw std::runtime_error("Mapping snapshot is missing: " + file);
    }
    const std::string& path =
        file == "public_map.pcd" ? public_map : trajectory;
    if (boost::filesystem::path(path).filename().string() != file)
    {
      throw std::runtime_error("Mapping input basename does not match marker: " +
                               path);
    }
    if (sha256File(path) != found->second)
    {
      throw std::runtime_error("Mapping snapshot checksum mismatch: " + file);
    }
  }
}

std::string utcNow()
{
  const std::time_t now = std::time(nullptr);
  std::tm utc = {};
  gmtime_r(&now, &utc);
  char text[32] = {};
  std::strftime(text, sizeof(text), "%Y-%m-%dT%H:%M:%SZ", &utc);
  return text;
}

void atomicReplace(const boost::filesystem::path& source,
                   const boost::filesystem::path& destination)
{
  if (std::rename(source.string().c_str(), destination.string().c_str()) != 0)
  {
    throw std::runtime_error("Atomic rename failed: " + source.string() +
                             " -> " + destination.string() + ": " +
                             std::strerror(errno));
  }
}

void writePgm(const std::string& path,
              const std::vector<std::uint8_t>& grid,
              const GridGeometry& geometry)
{
  std::ofstream output(path.c_str(), std::ios::binary | std::ios::trunc);
  if (!output)
  {
    throw std::runtime_error("Cannot create map image: " + path);
  }
  output << "P5\n" << geometry.width << " " << geometry.height << "\n255\n";
  for (int y = static_cast<int>(geometry.height) - 1; y >= 0; --y)
  {
    output.write(
        reinterpret_cast<const char*>(grid.data() + cellIndex(0, y, geometry)),
        static_cast<std::streamsize>(geometry.width));
  }
  if (!output)
  {
    throw std::runtime_error("Failed writing map image: " + path);
  }
}

void writeMapYaml(const std::string& path,
                  const MapDefinition& map,
                  const std::string& image_file)
{
  std::ofstream output(path.c_str(), std::ios::trunc);
  if (!output)
  {
    throw std::runtime_error("Cannot create map YAML: " + path);
  }
  output << "image: " << image_file << "\n";
  output << std::fixed << std::setprecision(9);
  output << "resolution: " << map.geometry.resolution << "\n";
  output << "origin: [" << map.geometry.origin_x << ", "
         << map.geometry.origin_y << ", " << map.geometry.origin_yaw << "]\n";
  output << "negate: " << map.negate << "\n";
  output << "occupied_thresh: " << map.occupied_threshold << "\n";
  output << "free_thresh: " << map.free_threshold << "\n";
}

void writePreview(const std::string& path,
                  const TerrainGrid& terrain,
                  const std::vector<std::uint8_t>& occupancy)
{
  std::ofstream output(path.c_str(), std::ios::binary | std::ios::trunc);
  if (!output)
  {
    throw std::runtime_error("Cannot create terrain preview: " + path);
  }
  output << "P6\n" << terrain.geometry.width << " "
         << terrain.geometry.height << "\n255\n";
  for (int y = static_cast<int>(terrain.geometry.height) - 1; y >= 0; --y)
  {
    for (int x = 0; x < static_cast<int>(terrain.geometry.width); ++x)
    {
      const std::size_t index = cellIndex(x, y, terrain.geometry);
      std::array<std::uint8_t, 3> pixel = {{90, 90, 90}};
      if (occupancy[index] == kOccupiedImage)
      {
        pixel = {{0, 0, 0}};
      }
      else if (terrain.cost[index] == 254)
      {
        pixel = {{220, 35, 35}};
      }
      else if (terrain.cost[index] != 255)
      {
        const double ratio = std::min(1.0, terrain.cost[index] / 80.0);
        pixel = {{static_cast<std::uint8_t>(40 + 200 * ratio),
                  static_cast<std::uint8_t>(190 - 100 * ratio), 45}};
      }
      else if (occupancy[index] == kFreeImage)
      {
        pixel = {{220, 220, 220}};
      }
      output.write(reinterpret_cast<const char*>(pixel.data()), 3);
    }
  }
}

}  // namespace

class TerrainExporter
{
public:
  TerrainExporter() : private_nh_("~")
  {
    loadParameters();
  }

  void run()
  {
    const boost::filesystem::path map_dir(map_dir_);
    if (!boost::filesystem::is_directory(map_dir))
    {
      throw std::runtime_error("Map directory does not exist: " + map_dir_);
    }
    if (!boost::filesystem::is_regular_file(input_pcd_) ||
        !boost::filesystem::is_regular_file(trajectory_pcd_) ||
        !boost::filesystem::is_regular_file(mapping_snapshot_))
    {
      throw std::runtime_error(
          "public_map.pcd, traversed_path_map.pcd, and mapping_snapshot.sha256 are required");
    }
    verifyMappingSnapshot(mapping_snapshot_, input_pcd_, trajectory_pcd_);
    const std::string initial_snapshot_digest =
        sha256File(mapping_snapshot_);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr trajectory(
        new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile(input_pcd_, *cloud) != 0 || cloud->empty())
    {
      throw std::runtime_error("Cannot load nonempty public map: " + input_pcd_);
    }
    if (pcl::io::loadPCDFile(trajectory_pcd_, *trajectory) != 0 ||
        trajectory->empty())
    {
      throw std::runtime_error("Cannot load nonempty trajectory: " +
                               trajectory_pcd_);
    }
    ROS_INFO("Terrain input: map_points=%zu trajectory_points=%zu",
             cloud->size(), trajectory->size());

    MapDefinition map;
    const bool has_input_map =
        boost::filesystem::is_regular_file(input_map_yaml_);
    if (has_input_map)
    {
      map = loadMapDefinition(input_map_yaml_);
      if (!nearlyEqual(map.geometry.resolution,
                       parameters_.required_resolution,
                       1e-9))
      {
        std::ostringstream error;
        error << "Map resolution must be " << parameters_.required_resolution
              << " m, got " << map.geometry.resolution;
        throw std::runtime_error(error.str());
      }
      ROS_INFO("Using existing map.yaml geometry: %zux%zu",
               map.geometry.width, map.geometry.height);
    }
    else
    {
      map = deriveMapDefinition(*cloud, parameters_);
      ROS_INFO("Derived map geometry from public_map.pcd: %zux%zu",
               map.geometry.width, map.geometry.height);
    }

    std::vector<std::uint8_t> baseline_occupancy;
    if (parameters_.preserve_existing_map)
    {
      if (!has_input_map)
      {
        throw std::runtime_error(
            "occupancy/preserve_existing_map requires input_map_yaml");
      }
      baseline_occupancy = readMapOccupancy(map);
      ROS_INFO("Loaded existing occupancy as atomic export baseline: %s",
               map.input_image_path.c_str());
    }

    const double base_to_floor =
        estimateBaseToFloor(*cloud, *trajectory, parameters_);
    ROS_INFO("Estimated base_link-to-floor height: %.3f m", base_to_floor);

    TerrainGrid terrain;
    std::vector<std::uint8_t> occupancy;
    pcl::PointCloud<pcl::PointXYZI> ground_diagnostic;
    pcl::PointCloud<pcl::PointXYZI> obstacle_diagnostic;
    reconstruct(*cloud,
                *trajectory,
                map.geometry,
                base_to_floor,
                baseline_occupancy,
                &terrain,
                &occupancy,
                &ground_diagnostic,
                &obstacle_diagnostic);

    const boost::filesystem::path stage =
        map_dir / boost::filesystem::unique_path(".go2_terrain_stage_%%%%-%%%%");
    boost::filesystem::create_directory(stage);
    try
    {
      verifyMappingSnapshot(mapping_snapshot_, input_pcd_, trajectory_pcd_);
      if (sha256File(mapping_snapshot_) != initial_snapshot_digest)
      {
        throw std::runtime_error(
            "Mapping snapshot changed while terrain reconstruction was running");
      }
      writeOutputs(stage,
                   map,
                   terrain,
                   occupancy,
                   ground_diagnostic,
                   obstacle_diagnostic,
                   base_to_floor);
      validateStage(stage, map.geometry);
      verifyMappingSnapshot(mapping_snapshot_, input_pcd_, trajectory_pcd_);
      if (sha256File(mapping_snapshot_) != initial_snapshot_digest)
      {
        throw std::runtime_error(
            "Mapping snapshot changed before terrain assets could be committed");
      }
      commitOutputs(stage, map_dir);
      boost::filesystem::remove_all(stage);
    }
    catch (...)
    {
      boost::filesystem::remove_all(stage);
      throw;
    }

    ROS_INFO("Terrain export committed: %s", map_dir_.c_str());
  }

private:
  void loadParameters()
  {
    private_nh_.param<std::string>("map_dir", map_dir_, std::string());
    if (map_dir_.empty())
    {
      throw std::runtime_error("~map_dir is required");
    }
    private_nh_.param<std::string>("export_id", export_id_, std::string());
    if (!validExportId(export_id_))
    {
      throw std::runtime_error(
          "~export_id is required and must contain only letters, numbers, '_' or '-'");
    }
    const boost::filesystem::path root(map_dir_);
    private_nh_.param<std::string>(
        "input_pcd", input_pcd_, (root / "public_map.pcd").string());
    private_nh_.param<std::string>(
        "trajectory_pcd",
        trajectory_pcd_,
        (root / "traversed_path_map.pcd").string());
    private_nh_.param<std::string>(
        "input_map_yaml", input_map_yaml_, (root / "map.yaml").string());
    private_nh_.param<std::string>(
        "mapping_snapshot",
        mapping_snapshot_,
        (root / "mapping_snapshot.sha256").string());

    auto param = [this](const std::string& name, double* value) {
      private_nh_.param(name, *value, *value);
    };
    auto int_param = [this](const std::string& name, int* value) {
      private_nh_.param(name, *value, *value);
    };
    param("resolution", &parameters_.required_resolution);
    param("map_padding_m", &parameters_.map_padding_m);
    param("base_height/search_radius", &parameters_.base_search_radius);
    param("base_height/min", &parameters_.base_min_height);
    param("base_height/max", &parameters_.base_max_height);
    int_param("base_height/min_points", &parameters_.base_min_points);
    param("ground/candidate_quantile", &parameters_.candidate_quantile);
    param("ground/candidate_support_band",
          &parameters_.candidate_support_band);
    param("ground/candidate_max_vertical_span",
          &parameters_.candidate_max_vertical_span);
    param("ground/trajectory_seed_radius",
          &parameters_.trajectory_seed_radius);
    param("ground/seed_height_tolerance",
          &parameters_.seed_height_tolerance);
    param("ground/seed_cell_height_tolerance",
          &parameters_.seed_cell_height_tolerance);
    param("ground/max_trajectory_gap_m",
          &parameters_.maximum_trajectory_gap);
    param("ground/max_reanchor_height_from_initial_m",
          &parameters_.maximum_reanchor_height_from_initial);
    param("ground/growth_height_margin",
          &parameters_.growth_height_margin);
    int_param("ground/growth_fill_iterations",
              &parameters_.growth_fill_iterations);
    param("ground/max_slope_deg", &parameters_.max_ground_slope_deg);
    param("ground/plane_radius", &parameters_.plane_radius);
    int_param("ground/plane_min_cells", &parameters_.plane_min_cells);
    param("ground/plane_huber_m", &parameters_.plane_huber_m);
    param("ground/max_plane_rmse", &parameters_.max_plane_rmse);
    param("obstacle/min_height", &parameters_.obstacle_min_height);
    param("obstacle/max_height", &parameters_.obstacle_max_height);
    param("obstacle/ground_search", &parameters_.obstacle_ground_search);
    int_param("obstacle/min_points", &parameters_.min_obstacle_points);
    param("occupancy/trajectory_free_radius",
          &parameters_.trajectory_free_radius);
    param("occupancy/obstacle_inflation_m",
          &parameters_.obstacle_inflation_m);
    private_nh_.param("occupancy/preserve_existing_map",
                      parameters_.preserve_existing_map,
                      parameters_.preserve_existing_map);
    int_param("minimum_ground_cells", &parameters_.minimum_ground_cells);
    param("quality/minimum_traced_trajectory_ratio",
          &parameters_.minimum_traced_trajectory_ratio);
    param("quality/minimum_ground_observation_ratio",
          &parameters_.minimum_ground_observation_ratio);
    param("quality/minimum_largest_ground_component_ratio",
          &parameters_.minimum_largest_ground_component_ratio);
    param("quality/minimum_ground_to_baseline_free_ratio",
          &parameters_.minimum_ground_to_baseline_free_ratio);
    param("quality/minimum_trajectory_corridor_known_ratio",
          &parameters_.minimum_trajectory_corridor_known_ratio);
    param("cost/flat_slope_deg", &parameters_.cost.flat_slope_deg);
    param("cost/lethal_slope_deg", &parameters_.cost.lethal_slope_deg);
    double minimum_cost = parameters_.cost.minimum_cost;
    double maximum_soft_cost = parameters_.cost.maximum_soft_cost;
    param("cost/minimum_cost", &minimum_cost);
    param("cost/maximum_soft_cost", &maximum_soft_cost);
    parameters_.cost.minimum_cost = static_cast<float>(minimum_cost);
    parameters_.cost.maximum_soft_cost = static_cast<float>(maximum_soft_cost);
    int lethal_cluster =
        static_cast<int>(parameters_.cost.minimum_lethal_cluster_cells);
    int_param("cost/minimum_lethal_cluster_cells", &lethal_cluster);
    parameters_.cost.minimum_lethal_cluster_cells =
        static_cast<std::size_t>(std::max(1, lethal_cluster));
    param("cost/dilation_m", &parameters_.cost.dilation_m);
    int_param("pmf/max_window_size", &parameters_.pmf_max_window_size);
    param("pmf/slope", &parameters_.pmf_slope);
    param("pmf/initial_distance", &parameters_.pmf_initial_distance);
    param("pmf/max_distance", &parameters_.pmf_max_distance);

    if (parameters_.base_min_height < 0.20 ||
        parameters_.base_max_height > 0.55 ||
        parameters_.base_min_height >= parameters_.base_max_height)
    {
      throw std::runtime_error(
          "base_height limits must remain within the approved [0.20, 0.55] m range");
    }
    if (parameters_.minimum_traced_trajectory_ratio <= 0.0 ||
        parameters_.minimum_traced_trajectory_ratio > 1.0 ||
        parameters_.minimum_ground_observation_ratio <= 0.0 ||
        parameters_.minimum_ground_observation_ratio > 1.0 ||
        parameters_.minimum_largest_ground_component_ratio <= 0.0 ||
        parameters_.minimum_largest_ground_component_ratio > 1.0 ||
        parameters_.minimum_ground_to_baseline_free_ratio <= 0.0 ||
        parameters_.minimum_ground_to_baseline_free_ratio > 1.0 ||
        parameters_.minimum_trajectory_corridor_known_ratio <= 0.0 ||
        parameters_.minimum_trajectory_corridor_known_ratio > 1.0 ||
        parameters_.maximum_reanchor_height_from_initial <= 0.0)
    {
      throw std::runtime_error("Terrain quality ratios must be in (0, 1]");
    }
    const double maximum_adjacent_rise =
        std::tan(parameters_.max_ground_slope_deg * kPi / 180.0) *
            parameters_.required_resolution +
        parameters_.growth_height_margin;
    if (!nearlyEqual(parameters_.required_resolution, 0.05, 1e-9) ||
        maximum_adjacent_rise >= 0.05 ||
        parameters_.seed_cell_height_tolerance >= 0.05)
    {
      throw std::runtime_error(
          "Ground continuity settings must preserve a strict barrier at a 0.05 m step");
    }
    if (parameters_.max_ground_slope_deg != 35.0 ||
        parameters_.cost.flat_slope_deg != 8.0 ||
        parameters_.cost.lethal_slope_deg != 30.0 ||
        parameters_.cost.minimum_lethal_cluster_cells != 4 ||
        !nearlyEqual(parameters_.obstacle_min_height, 0.05, 1e-9) ||
        !nearlyEqual(parameters_.obstacle_max_height, 1.50, 1e-9) ||
        !parameters_.preserve_existing_map ||
        !nearlyEqual(parameters_.maximum_reanchor_height_from_initial,
                     0.65, 1e-9) ||
        !nearlyEqual(parameters_.minimum_traced_trajectory_ratio,
                     0.80, 1e-9) ||
        !nearlyEqual(parameters_.minimum_ground_observation_ratio,
                     0.30, 1e-9) ||
        !nearlyEqual(parameters_.minimum_largest_ground_component_ratio,
                     0.60, 1e-9) ||
        !nearlyEqual(parameters_.minimum_ground_to_baseline_free_ratio,
                     0.08, 1e-9) ||
        !nearlyEqual(parameters_.minimum_trajectory_corridor_known_ratio,
                     0.95, 1e-9) ||
        !nearlyEqual(parameters_.trajectory_free_radius, 0.18, 1e-9) ||
        !nearlyEqual(parameters_.obstacle_inflation_m, 0.03, 1e-9) ||
        !nearlyEqual(parameters_.cost.minimum_cost, 15.0, 1e-9) ||
        !nearlyEqual(parameters_.cost.maximum_soft_cost, 80.0, 1e-9) ||
        !nearlyEqual(parameters_.cost.dilation_m, 0.20, 1e-9))
    {
      throw std::runtime_error(
          "Terrain safety thresholds differ from the approved GO2 profile");
    }
  }

  void reconstruct(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                   const pcl::PointCloud<pcl::PointXYZ>& trajectory,
                   const GridGeometry& geometry,
                   double base_to_floor,
                   const std::vector<std::uint8_t>& baseline_occupancy,
                   TerrainGrid* terrain,
                   std::vector<std::uint8_t>* occupancy,
                   pcl::PointCloud<pcl::PointXYZI>* ground_diagnostic,
                   pcl::PointCloud<pcl::PointXYZI>* obstacle_diagnostic) const
  {
    terrain->resize(geometry);
    const std::size_t count = geometry.cellCount();
    if (!baseline_occupancy.empty() && baseline_occupancy.size() != count)
    {
      throw std::runtime_error("Baseline occupancy geometry does not match map");
    }
    std::vector<std::vector<float>> samples(count);
    for (const auto& point : cloud.points)
    {
      if (!pcl::isFinite(point))
      {
        continue;
      }
      int x = 0;
      int y = 0;
      if (pointToCell(point.x, point.y, geometry, &x, &y))
      {
        samples[cellIndex(x, y, geometry)].push_back(point.z);
      }
    }

    std::vector<float> candidate(count,
                                 std::numeric_limits<float>::quiet_NaN());
    std::vector<float> vertical_span(count,
                                     std::numeric_limits<float>::quiet_NaN());
    for (std::size_t i = 0; i < count; ++i)
    {
      if (samples[i].empty())
      {
        continue;
      }
      const GroundColumnSummary summary = summarizeGroundColumn(
          samples[i],
          parameters_.candidate_quantile,
          parameters_.candidate_support_band);
      candidate[i] = summary.candidate;
      vertical_span[i] = summary.support_span;
    }

    pcl::PointIndices pmf_indices;
    pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_ptr = cloud.makeShared();
    pmf.setInputCloud(cloud_ptr);
    pmf.setMaxWindowSize(parameters_.pmf_max_window_size);
    pmf.setSlope(parameters_.pmf_slope);
    pmf.setInitialDistance(parameters_.pmf_initial_distance);
    pmf.setMaxDistance(parameters_.pmf_max_distance);
    pmf.extract(pmf_indices.indices);
    std::vector<std::uint8_t> pmf_seed(count, 0);
    for (const int index : pmf_indices.indices)
    {
      const auto& point = cloud.points[static_cast<std::size_t>(index)];
      int x = 0;
      int y = 0;
      if (pointToCell(point.x, point.y, geometry, &x, &y))
      {
        pmf_seed[cellIndex(x, y, geometry)] = 1;
      }
    }
    const std::size_t candidate_cells = static_cast<std::size_t>(std::count_if(
        candidate.begin(), candidate.end(),
        [](float value) { return isKnown(value); }));
    const std::size_t pmf_cells = static_cast<std::size_t>(std::count(
        pmf_seed.begin(), pmf_seed.end(), static_cast<std::uint8_t>(1U)));
    std::size_t admissible_candidate_cells = 0U;
    for (std::size_t i = 0; i < count; ++i)
    {
      if (pmf_seed[i] != 0U && isKnown(candidate[i]) &&
          isKnown(vertical_span[i]) &&
          vertical_span[i] <= parameters_.candidate_max_vertical_span)
      {
        ++admissible_candidate_cells;
      }
    }
    ROS_INFO("Terrain ground inputs: candidates=%zu pmf_cells=%zu "
             "admissible_candidates=%zu",
             candidate_cells, pmf_cells, admissible_candidate_cells);

    std::vector<PlanarPoint> trajectory_xy;
    trajectory_xy.reserve(trajectory.size());
    for (const auto& point : trajectory.points)
    {
      if (pcl::isFinite(point))
      {
        trajectory_xy.push_back({point.x, point.y});
      }
    }
    if (trajectory_xy.empty())
    {
      throw std::runtime_error("Trajectory contains no finite points");
    }
    const std::vector<float> traced_ground = traceGroundAlongTrajectory(
        trajectory_xy,
        candidate,
        vertical_span,
        pmf_seed,
        geometry,
        parameters_.trajectory_seed_radius,
        parameters_.candidate_max_vertical_span,
        -base_to_floor,
        parameters_.seed_height_tolerance,
        parameters_.max_ground_slope_deg,
        parameters_.growth_height_margin,
        parameters_.maximum_reanchor_height_from_initial,
        parameters_.maximum_trajectory_gap);

    const std::size_t traced_points = static_cast<std::size_t>(std::count_if(
        traced_ground.begin(), traced_ground.end(),
        [](float value) { return isKnown(value); }));
    const double traced_ratio = static_cast<double>(traced_points) /
                                static_cast<double>(traced_ground.size());
    ROS_INFO("Terrain trajectory trace: %zu/%zu (%.1f%%)",
             traced_points, traced_ground.size(), 100.0 * traced_ratio);
    if (traced_ratio < parameters_.minimum_traced_trajectory_ratio)
    {
      std::ostringstream error;
      error << "Ground trace covers " << std::fixed << std::setprecision(1)
            << 100.0 * traced_ratio << "% of trajectory; require at least "
            << 100.0 * parameters_.minimum_traced_trajectory_ratio << "%";
      throw std::runtime_error(error.str());
    }
    const std::vector<std::uint8_t> initial_seed = buildTrajectorySeedMask(
        trajectory_xy,
        traced_ground,
        candidate,
        vertical_span,
        geometry,
        parameters_.trajectory_seed_radius,
        parameters_.candidate_max_vertical_span,
        parameters_.seed_cell_height_tolerance);
    const std::size_t initial_seed_cells = static_cast<std::size_t>(std::count(
        initial_seed.begin(), initial_seed.end(), static_cast<std::uint8_t>(1U)));

    const std::vector<std::uint8_t> accepted = growConnectedGround(
        candidate,
        vertical_span,
        initial_seed,
        geometry,
        parameters_.candidate_max_vertical_span,
        parameters_.max_ground_slope_deg,
        parameters_.growth_height_margin,
        parameters_.growth_fill_iterations);
    const std::size_t accepted_cells = static_cast<std::size_t>(std::count(
        accepted.begin(), accepted.end(), static_cast<std::uint8_t>(1U)));
    ROS_INFO("Terrain ground propagation: trajectory_seeds=%zu accepted=%zu",
             initial_seed_cells, accepted_cells);

    std::vector<float> raw_elevation(count,
                                     std::numeric_limits<float>::quiet_NaN());
    for (std::size_t i = 0; i < count; ++i)
    {
      if (accepted[i])
      {
        raw_elevation[i] = candidate[i];
      }
    }
    fitTerrainPlanes(raw_elevation, samples, geometry, terrain);

    std::size_t ground_cells = 0;
    std::vector<std::uint8_t> reconstructed_ground(count, 0U);
    for (std::size_t i = 0; i < terrain->elevation.size(); ++i)
    {
      if (isKnown(terrain->elevation[i]))
      {
        reconstructed_ground[i] = 1U;
        ++ground_cells;
      }
    }
    if (ground_cells < static_cast<std::size_t>(parameters_.minimum_ground_cells))
    {
      std::ostringstream error;
      error << "Robust ground reconstruction produced " << ground_cells
            << " cells (trajectory_seeds=" << initial_seed_cells
            << ", accepted=" << accepted_cells << "); require at least "
            << parameters_.minimum_ground_cells;
      throw std::runtime_error(error.str());
    }
    // Every reconstructed cell originated from a trajectory-anchored,
    // height- and slope-bounded candidate. Evaluate plane-fit retention
    // against those candidates. A raw PMF overlap is not a reliable truth
    // metric for indoor maps with walls, ceilings, and multiple floor levels.
    const std::size_t eligible_observations = accepted_cells;
    std::size_t reconstructed_observations = 0U;
    for (std::size_t i = 0; i < count; ++i)
    {
      if (accepted[i] != 0U)
      {
        reconstructed_observations += reconstructed_ground[i] != 0U ? 1U : 0U;
      }
    }
    if (eligible_observations == 0U)
    {
      throw std::runtime_error("Ground propagation produced no observations");
    }
    const double observation_ratio =
        static_cast<double>(reconstructed_observations) /
        static_cast<double>(eligible_observations);
    // Sparse LiDAR samples can split fitted floor pixels even though the robot
    // physically traversed the space between them. Add the gap-bounded robot
    // corridor only to the connectivity metric; it never creates elevation,
    // slope, free-space, or obstacle evidence.
    std::vector<std::uint8_t> connectivity_mask = reconstructed_ground;
    const std::vector<std::uint8_t> trajectory_connectivity =
        buildTrajectoryMask(trajectory_xy,
                            parameters_.trajectory_seed_radius,
                            parameters_.maximum_trajectory_gap,
                            geometry);
    for (std::size_t i = 0; i < count; ++i)
    {
      connectivity_mask[i] =
          connectivity_mask[i] != 0U || trajectory_connectivity[i] != 0U
              ? 1U
              : 0U;
    }
    const BinaryMaskMetrics ground_quality =
        measureBinaryMaskQuality(connectivity_mask, geometry);
    ROS_INFO("Terrain ground quality: bbox_coverage=%.1f%% "
             "fit_retention=%.1f%% trajectory_anchored_component=%.1f%% "
             "(%zu/%zu cells)",
             100.0 * ground_quality.coverage_ratio,
             100.0 * observation_ratio,
             100.0 * ground_quality.largest_component_ratio,
             ground_quality.largest_component_cells,
             ground_quality.active_cells);
    if (observation_ratio < parameters_.minimum_ground_observation_ratio ||
        ground_quality.largest_component_ratio <
            parameters_.minimum_largest_ground_component_ratio)
    {
      std::ostringstream error;
      error << "Terrain ground quality failed: fit_retention="
            << std::fixed
            << std::setprecision(1)
            << 100.0 * observation_ratio << "% (need "
            << 100.0 * parameters_.minimum_ground_observation_ratio
            << "%), trajectory_anchored_component="
            << 100.0 * ground_quality.largest_component_ratio << "% (need "
            << 100.0 * parameters_.minimum_largest_ground_component_ratio
            << "%)";
      throw std::runtime_error(error.str());
    }
    terrain->cost = buildSlopeCostLayer(
        terrain->slope_deg, geometry, parameters_.cost);

    // Plane fitting intentionally refuses weak cells. Obstacle extraction can
    // still use a nearby value from the accepted, step-safe ground surface so
    // isolated 5 cm fit holes do not suppress an entire obstacle column.
    std::vector<float> obstacle_ground_reference = terrain->elevation;
    for (std::size_t i = 0; i < count; ++i)
    {
      if (!isKnown(obstacle_ground_reference[i]) &&
          isKnown(raw_elevation[i]))
      {
        obstacle_ground_reference[i] = raw_elevation[i];
      }
    }

    std::vector<int> obstacle_count(count, 0);
    for (const auto& point : cloud.points)
    {
      if (!pcl::isFinite(point))
      {
        continue;
      }
      int x = 0;
      int y = 0;
      if (!pointToCell(point.x, point.y, geometry, &x, &y))
      {
        continue;
      }
      const float ground = nearestGround(
          x, y, obstacle_ground_reference, geometry);
      if (!isKnown(ground))
      {
        continue;
      }
      const double relative_height = point.z - ground;
      if (isObstacleHeight(relative_height,
                           parameters_.obstacle_min_height,
                           parameters_.obstacle_max_height))
      {
        ++obstacle_count[cellIndex(x, y, geometry)];
      }
    }

    std::vector<std::uint8_t> free_mask(count, 0);
    std::vector<std::uint8_t> obstacle_mask(count, 0);
    for (std::size_t i = 0; i < count; ++i)
    {
      free_mask[i] = isKnown(terrain->elevation[i]) ? 1 : 0;
      obstacle_mask[i] = obstacle_count[i] >= parameters_.min_obstacle_points ? 1 : 0;
    }
    // Unknown terrain is fail-closed. Free-space evidence must never dilate
    // across a wall edge, rejected step, or >35 degree surface.
    std::vector<std::uint8_t> trajectory_free_evidence(count, 0U);
    std::vector<std::uint8_t> obstacle_inflated;
    const std::vector<std::uint8_t> all_trajectory_mask =
        buildTrajectoryMask(trajectory_xy,
                            parameters_.trajectory_free_radius,
                            parameters_.maximum_trajectory_gap,
                            geometry);
    // The complete recorded path is direct free-space evidence because the
    // robot physically occupied it. Long odometry jumps are not connected.
    // This evidence can only fill unknown cells: baseline obstacles survive,
    // and measured terrain obstacles are applied afterward.
    applyTrajectoryFreeEvidence(all_trajectory_mask,
                                &trajectory_free_evidence);
    dilateBinaryMaskMetric(obstacle_mask,
                           &obstacle_inflated,
                           geometry,
                           parameters_.obstacle_inflation_m);

    *occupancy = mergeOccupancyEvidence(
        baseline_occupancy,
        free_mask,
        trajectory_free_evidence,
        obstacle_inflated,
        kUnknownImage,
        kFreeImage,
        kOccupiedImage);

    std::size_t baseline_known = 0U;
    std::size_t baseline_known_preserved = 0U;
    std::size_t baseline_free = 0U;
    std::size_t ground_on_baseline_free = 0U;
    for (std::size_t i = 0; i < count; ++i)
    {
      if (!baseline_occupancy.empty() &&
          baseline_occupancy[i] != kUnknownImage)
      {
        ++baseline_known;
        baseline_known_preserved +=
            (*occupancy)[i] != kUnknownImage ? 1U : 0U;
      }
      if (!baseline_occupancy.empty() &&
          baseline_occupancy[i] == kFreeImage)
      {
        ++baseline_free;
        ground_on_baseline_free += reconstructed_ground[i] != 0U ? 1U : 0U;
      }
    }
    if (!baseline_occupancy.empty() &&
        (baseline_known == 0U || baseline_free == 0U ||
         baseline_known_preserved != baseline_known))
    {
      throw std::runtime_error(
          "Legacy occupancy baseline is empty or was not preserved completely");
    }
    if (!baseline_occupancy.empty())
    {
      const double baseline_ground_ratio =
          static_cast<double>(ground_on_baseline_free) /
          static_cast<double>(baseline_free);
      ROS_INFO("Terrain baseline quality: reconstructed_ground/free=%.1f%% "
               "known_preserved=%zu/%zu",
               100.0 * baseline_ground_ratio,
               baseline_known_preserved,
               baseline_known);
      if (baseline_ground_ratio <
          parameters_.minimum_ground_to_baseline_free_ratio)
      {
        std::ostringstream error;
        error << "Terrain ground covers only " << std::fixed
              << std::setprecision(1) << 100.0 * baseline_ground_ratio
              << "% of legacy free cells; require at least "
              << 100.0 * parameters_.minimum_ground_to_baseline_free_ratio
              << "%";
        throw std::runtime_error(error.str());
      }
    }

    std::size_t trajectory_corridor_cells = 0U;
    std::size_t known_trajectory_corridor_cells = 0U;
    for (std::size_t i = 0; i < count; ++i)
    {
      if (all_trajectory_mask[i] != 0U)
      {
        ++trajectory_corridor_cells;
        known_trajectory_corridor_cells +=
            (*occupancy)[i] != kUnknownImage ? 1U : 0U;
      }
    }
    if (trajectory_corridor_cells == 0U)
    {
      throw std::runtime_error("Trajectory corridor does not intersect map geometry");
    }
    const double trajectory_corridor_ratio =
        static_cast<double>(known_trajectory_corridor_cells) /
        static_cast<double>(trajectory_corridor_cells);
    ROS_INFO("Terrain trajectory corridor known coverage: %.1f%% (%zu/%zu)",
             100.0 * trajectory_corridor_ratio,
             known_trajectory_corridor_cells,
             trajectory_corridor_cells);
    if (trajectory_corridor_ratio <
        parameters_.minimum_trajectory_corridor_known_ratio)
    {
      std::ostringstream error;
      error << "Known occupancy covers " << std::fixed << std::setprecision(1)
            << 100.0 * trajectory_corridor_ratio
            << "% of driven corridor; require at least "
            << 100.0 * parameters_.minimum_trajectory_corridor_known_ratio
            << "%";
      throw std::runtime_error(error.str());
    }

    ground_diagnostic->header.frame_id = "map";
    obstacle_diagnostic->header.frame_id = "map";
    for (int y = 0; y < static_cast<int>(geometry.height); ++y)
    {
      for (int x = 0; x < static_cast<int>(geometry.width); ++x)
      {
        const std::size_t index = cellIndex(x, y, geometry);
        const float world_x = static_cast<float>(
            geometry.origin_x + (x + 0.5) * geometry.resolution);
        const float world_y = static_cast<float>(
            geometry.origin_y + (y + 0.5) * geometry.resolution);
        if (isKnown(terrain->elevation[index]))
        {
          pcl::PointXYZI point;
          point.x = world_x;
          point.y = world_y;
          point.z = terrain->elevation[index];
          point.intensity = isKnown(terrain->slope_deg[index])
                                ? terrain->slope_deg[index]
                                : -1.0F;
          ground_diagnostic->push_back(point);
        }
        if (obstacle_mask[index])
        {
          pcl::PointXYZI point;
          point.x = world_x;
          point.y = world_y;
          point.z = isKnown(terrain->elevation[index])
                        ? terrain->elevation[index] +
                              static_cast<float>(parameters_.obstacle_min_height)
                        : 0.0F;
          point.intensity = static_cast<float>(obstacle_count[index]);
          obstacle_diagnostic->push_back(point);
        }
      }
    }
    ROS_INFO("Terrain reconstruction: ground_cells=%zu obstacle_cells=%zu",
             ground_cells, obstacle_diagnostic->size());
  }

  void fitTerrainPlanes(const std::vector<float>& raw_elevation,
                        const std::vector<std::vector<float>>& samples,
                        const GridGeometry& geometry,
                        TerrainGrid* terrain) const
  {
    const int radius = static_cast<int>(
        std::ceil(parameters_.plane_radius / geometry.resolution));
    const double max_slope = parameters_.max_ground_slope_deg;
    for (int center_y = 0; center_y < static_cast<int>(geometry.height); ++center_y)
    {
      for (int center_x = 0; center_x < static_cast<int>(geometry.width); ++center_x)
      {
        const std::size_t center = cellIndex(center_x, center_y, geometry);
        if (!isKnown(raw_elevation[center]))
        {
          continue;
        }

        std::vector<Eigen::Vector3d> observations;
        for (int dy = -radius; dy <= radius; ++dy)
        {
          for (int dx = -radius; dx <= radius; ++dx)
          {
            if (dx * dx + dy * dy > radius * radius)
            {
              continue;
            }
            const int x = center_x + dx;
            const int y = center_y + dy;
            if (!inside(x, y, geometry))
            {
              continue;
            }
            const float z = raw_elevation[cellIndex(x, y, geometry)];
            if (isKnown(z))
            {
              observations.emplace_back(dx * geometry.resolution,
                                        dy * geometry.resolution,
                                        z);
            }
          }
        }
        if (observations.size() <
            static_cast<std::size_t>(parameters_.plane_min_cells))
        {
          continue;
        }

        Eigen::Vector3d model = Eigen::Vector3d::Zero();
        std::vector<double> weights(observations.size(), 1.0);
        bool solved = true;
        for (int iteration = 0; iteration < 3; ++iteration)
        {
          Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
          Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
          for (std::size_t i = 0; i < observations.size(); ++i)
          {
            const Eigen::Vector3d row(
                observations[i].x(), observations[i].y(), 1.0);
            normal += weights[i] * row * row.transpose();
            rhs += weights[i] * row * observations[i].z();
          }
          Eigen::LDLT<Eigen::Matrix3d> decomposition(normal);
          if (decomposition.info() != Eigen::Success)
          {
            solved = false;
            break;
          }
          model = decomposition.solve(rhs);
          if (!model.allFinite())
          {
            solved = false;
            break;
          }
          for (std::size_t i = 0; i < observations.size(); ++i)
          {
            const double predicted = model.x() * observations[i].x() +
                                     model.y() * observations[i].y() + model.z();
            const double residual = std::fabs(observations[i].z() - predicted);
            weights[i] = residual <= parameters_.plane_huber_m
                             ? 1.0
                             : parameters_.plane_huber_m / residual;
          }
        }
        if (!solved)
        {
          continue;
        }

        double squared_error = 0.0;
        for (const auto& observation : observations)
        {
          const double predicted = model.x() * observation.x() +
                                   model.y() * observation.y() + model.z();
          const double residual = observation.z() - predicted;
          squared_error += residual * residual;
        }
        const double rmse =
            std::sqrt(squared_error / static_cast<double>(observations.size()));
        const double slope =
            std::atan(std::hypot(model.x(), model.y())) * 180.0 / kPi;
        if (!std::isfinite(rmse) || !std::isfinite(slope) ||
            rmse > parameters_.max_plane_rmse || slope > max_slope)
        {
          continue;
        }

        double maximum_step = 0.0;
        for (int dy = -1; dy <= 1; ++dy)
        {
          for (int dx = -1; dx <= 1; ++dx)
          {
            if (dx == 0 && dy == 0)
            {
              continue;
            }
            const int x = center_x + dx;
            const int y = center_y + dy;
            if (inside(x, y, geometry))
            {
              const float neighbor = raw_elevation[cellIndex(x, y, geometry)];
              if (isKnown(neighbor))
              {
                maximum_step = std::max(
                    maximum_step,
                    std::fabs(static_cast<double>(raw_elevation[center] - neighbor)));
              }
            }
          }
        }

        terrain->elevation[center] = static_cast<float>(model.z());
        terrain->slope_deg[center] = static_cast<float>(slope);
        terrain->roughness[center] = static_cast<float>(rmse);
        terrain->step[center] = static_cast<float>(maximum_step);
        const double support = std::min(
            1.0,
            static_cast<double>(observations.size()) /
                std::max(4.0,
                         kPi * parameters_.plane_radius *
                             parameters_.plane_radius /
                             (geometry.resolution * geometry.resolution) * 0.5));
        const double sample_support =
            std::min(1.0, static_cast<double>(samples[center].size()) / 3.0);
        const double confidence =
            std::max(0.0,
                     std::min(1.0,
                              support * sample_support *
                                  std::exp(-rmse / 0.05)));
        terrain->confidence[center] = static_cast<std::uint8_t>(
            std::lround(confidence * 100.0));
      }
    }
  }

  float nearestGround(int center_x,
                      int center_y,
                      const std::vector<float>& ground_reference,
                      const GridGeometry& geometry) const
  {
    if (ground_reference.size() != geometry.cellCount())
    {
      throw std::invalid_argument("Ground-reference geometry mismatch");
    }
    const std::size_t center = cellIndex(center_x, center_y, geometry);
    if (isKnown(ground_reference[center]))
    {
      return ground_reference[center];
    }
    const int radius = static_cast<int>(
        std::ceil(parameters_.obstacle_ground_search /
                  geometry.resolution));
    float nearest = std::numeric_limits<float>::quiet_NaN();
    int best_squared = std::numeric_limits<int>::max();
    std::vector<Eigen::Vector3d> observations;
    for (int dy = -radius; dy <= radius; ++dy)
    {
      for (int dx = -radius; dx <= radius; ++dx)
      {
        const int squared = dx * dx + dy * dy;
        if (std::sqrt(static_cast<double>(squared)) *
                    geometry.resolution >
                parameters_.obstacle_ground_search)
        {
          continue;
        }
        const int x = center_x + dx;
        const int y = center_y + dy;
        if (!inside(x, y, geometry))
        {
          continue;
        }
        const float value = ground_reference[cellIndex(x, y, geometry)];
        if (isKnown(value))
        {
          observations.emplace_back(dx * geometry.resolution,
                                    dy * geometry.resolution,
                                    value);
          if (squared < best_squared)
          {
            nearest = value;
            best_squared = squared;
          }
        }
      }
    }
    // One-cell holes need no extrapolation and remain bounded by the same
    // adjacent-rise rule used for accepted ground growth.
    if (best_squared <= 1)
    {
      return nearest;
    }
    if (observations.size() <
        static_cast<std::size_t>(parameters_.plane_min_cells))
    {
      return std::numeric_limits<float>::quiet_NaN();
    }

    Eigen::Vector3d model = Eigen::Vector3d::Zero();
    std::vector<double> weights(observations.size(), 1.0);
    for (int iteration = 0; iteration < 3; ++iteration)
    {
      Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
      Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
      for (std::size_t i = 0; i < observations.size(); ++i)
      {
        const Eigen::Vector3d row(
            observations[i].x(), observations[i].y(), 1.0);
        normal += weights[i] * row * row.transpose();
        rhs += weights[i] * row * observations[i].z();
      }
      Eigen::LDLT<Eigen::Matrix3d> decomposition(normal);
      if (decomposition.info() != Eigen::Success)
      {
        return std::numeric_limits<float>::quiet_NaN();
      }
      model = decomposition.solve(rhs);
      if (!model.allFinite())
      {
        return std::numeric_limits<float>::quiet_NaN();
      }
      for (std::size_t i = 0; i < observations.size(); ++i)
      {
        const double predicted = model.x() * observations[i].x() +
                                 model.y() * observations[i].y() + model.z();
        const double residual = std::fabs(observations[i].z() - predicted);
        weights[i] = residual <= parameters_.plane_huber_m
                         ? 1.0
                         : parameters_.plane_huber_m / residual;
      }
    }
    double squared_error = 0.0;
    for (const auto& observation : observations)
    {
      const double predicted = model.x() * observation.x() +
                               model.y() * observation.y() + model.z();
      const double residual = observation.z() - predicted;
      squared_error += residual * residual;
    }
    const double rmse =
        std::sqrt(squared_error / static_cast<double>(observations.size()));
    const double slope =
        std::atan(std::hypot(model.x(), model.y())) * 180.0 / kPi;
    if (!std::isfinite(rmse) || !std::isfinite(slope) ||
        rmse > parameters_.max_plane_rmse ||
        slope > parameters_.max_ground_slope_deg)
    {
      return std::numeric_limits<float>::quiet_NaN();
    }
    return static_cast<float>(model.z());
  }

  void writeOutputs(
      const boost::filesystem::path& stage,
      const MapDefinition& map,
      const TerrainGrid& terrain,
      const std::vector<std::uint8_t>& occupancy,
      const pcl::PointCloud<pcl::PointXYZI>& ground_diagnostic,
      const pcl::PointCloud<pcl::PointXYZI>& obstacle_diagnostic,
      double base_to_floor) const
  {
    const std::string map_pgm = "map.pgm";
    const std::string map_yaml = "map.yaml";
    const std::string elevation = "terrain_elevation.f32";
    const std::string slope = "terrain_slope.f32";
    const std::string roughness = "terrain_roughness.f32";
    const std::string step = "terrain_step.f32";
    const std::string cost = "terrain_cost.u8";
    const std::string confidence = "terrain_confidence.u8";
    const std::string ground_pcd = "terrain_ground.pcd";
    const std::string obstacle_pcd = "terrain_obstacles.pcd";
    const std::string preview = "terrain_preview.ppm";
    const std::string checksums = "terrain_checksums.sha256";

    writePgm((stage / map_pgm).string(), occupancy, terrain.geometry);
    writeMapYaml((stage / map_yaml).string(), map, map_pgm);
    writeFloatLayer((stage / elevation).string(), terrain.elevation);
    writeFloatLayer((stage / slope).string(), terrain.slope_deg);
    writeFloatLayer((stage / roughness).string(), terrain.roughness);
    writeFloatLayer((stage / step).string(), terrain.step);
    writeUint8Layer((stage / cost).string(), terrain.cost);
    writeUint8Layer((stage / confidence).string(), terrain.confidence);
    if (pcl::io::savePCDFileBinaryCompressed((stage / ground_pcd).string(),
                                             ground_diagnostic) != 0 ||
        pcl::io::savePCDFileBinaryCompressed((stage / obstacle_pcd).string(),
                                             obstacle_diagnostic) != 0)
    {
      throw std::runtime_error("Failed writing terrain diagnostic PCD files");
    }
    writePreview((stage / preview).string(), terrain, occupancy);

    TerrainMetadata metadata;
    metadata.format = "go2_terrain_2p5d";
    metadata.version = 1;
    metadata.export_id = export_id_;
    metadata.frame_id = "map";
    metadata.geometry = terrain.geometry;
    metadata.image_file = map_pgm;
    metadata.base_to_floor_m = base_to_floor;
    metadata.elevation = {elevation, "float32_le", "m"};
    metadata.slope = {slope, "float32_le", "deg"};
    metadata.roughness = {roughness, "float32_le", "m"};
    metadata.step = {step, "float32_le", "m"};
    metadata.cost = {cost, "uint8", "cost"};
    metadata.confidence = {confidence, "uint8", "percent"};
    const std::vector<std::pair<std::string, double>> exported_parameters = {
        {"ground_max_slope_deg", parameters_.max_ground_slope_deg},
        {"obstacle_min_height_m", parameters_.obstacle_min_height},
        {"obstacle_max_height_m", parameters_.obstacle_max_height},
        {"trajectory_free_radius_m", parameters_.trajectory_free_radius},
        {"obstacle_inflation_m", parameters_.obstacle_inflation_m},
        {"preserve_existing_map",
         parameters_.preserve_existing_map ? 1.0 : 0.0},
        {"maximum_reanchor_height_from_initial_m",
         parameters_.maximum_reanchor_height_from_initial},
        {"minimum_traced_trajectory_ratio",
         parameters_.minimum_traced_trajectory_ratio},
        {"minimum_ground_observation_ratio",
         parameters_.minimum_ground_observation_ratio},
        {"minimum_largest_ground_component_ratio",
         parameters_.minimum_largest_ground_component_ratio},
        {"minimum_ground_to_baseline_free_ratio",
         parameters_.minimum_ground_to_baseline_free_ratio},
        {"minimum_trajectory_corridor_known_ratio",
         parameters_.minimum_trajectory_corridor_known_ratio},
        {"flat_slope_deg", parameters_.cost.flat_slope_deg},
        {"lethal_slope_deg", parameters_.cost.lethal_slope_deg},
        {"minimum_slope_cost", parameters_.cost.minimum_cost},
        {"maximum_soft_slope_cost", parameters_.cost.maximum_soft_cost},
        {"slope_cost_dilation_m", parameters_.cost.dilation_m},
        {"minimum_lethal_cluster_cells",
         static_cast<double>(parameters_.cost.minimum_lethal_cluster_cells)}};
    writeTerrainMetadata(
        (stage / "terrain_2p5d.yaml").string(),
        metadata,
        boost::filesystem::path(input_pcd_).filename().string(),
        boost::filesystem::path(trajectory_pcd_).filename().string(),
        utcNow(),
        checksums,
        exported_parameters);

    const std::vector<std::string> checksummed = {
        map_yaml, map_pgm, elevation, slope, roughness, step, cost, confidence,
        ground_pcd, obstacle_pcd, preview, "terrain_2p5d.yaml"};
    std::ofstream checksum_output((stage / checksums).string(), std::ios::trunc);
    if (!checksum_output)
    {
      throw std::runtime_error("Cannot create terrain checksum file");
    }
    for (const auto& file : checksummed)
    {
      checksum_output << sha256File((stage / file).string()) << "  " << file
                      << "\n";
    }
    const std::vector<std::pair<std::string, std::string>> source_files = {
        {input_pcd_, boost::filesystem::path(input_pcd_).filename().string()},
        {trajectory_pcd_,
         boost::filesystem::path(trajectory_pcd_).filename().string()},
        {mapping_snapshot_, "mapping_snapshot.sha256"}};
    for (const auto& source : source_files)
    {
      checksum_output << sha256File(source.first) << "  " << source.second
                      << "\n";
    }
    checksum_output.close();
    if (!checksum_output)
    {
      throw std::runtime_error("Failed writing terrain checksum file");
    }
  }

  void validateStage(const boost::filesystem::path& stage,
                     const GridGeometry& expected_geometry) const
  {
    const TerrainMetadata metadata =
        loadTerrainMetadata((stage / "terrain_2p5d.yaml").string());
    if (metadata.export_id != export_id_)
    {
      throw std::runtime_error("Staged terrain export_id does not match request");
    }
    std::string geometry_error;
    if (!geometryMatches(metadata.geometry,
                         expected_geometry,
                         1e-9,
                         &geometry_error))
    {
      throw std::runtime_error("Staged metadata geometry mismatch: " +
                               geometry_error);
    }
    if (metadata.base_to_floor_m < parameters_.base_min_height ||
        metadata.base_to_floor_m > parameters_.base_max_height)
    {
      throw std::runtime_error("Staged base-to-floor estimate is outside limits");
    }

    const MapDefinition staged_map =
        loadMapDefinition((stage / "map.yaml").string());
    if (!geometryMatches(staged_map.geometry,
                         expected_geometry,
                         1e-9,
                         &geometry_error))
    {
      throw std::runtime_error("Staged map geometry mismatch: " + geometry_error);
    }

    const std::vector<LayerDescriptor> float_layers = {
        metadata.elevation,
        metadata.slope,
        metadata.roughness,
        metadata.step};
    for (const auto& layer : float_layers)
    {
      const std::vector<float> values = readFloatLayer(
          (stage / layer.file).string(), metadata.geometry.cellCount());
      for (const float value : values)
      {
        if (std::isinf(value))
        {
          throw std::runtime_error("Staged float layer contains infinity: " +
                                   layer.file);
        }
      }
    }
    const std::vector<std::uint8_t> costs = readUint8Layer(
        (stage / metadata.cost.file).string(), metadata.geometry.cellCount());
    const std::vector<std::uint8_t> confidence = readUint8Layer(
        (stage / metadata.confidence.file).string(),
        metadata.geometry.cellCount());
    for (const std::uint8_t value : confidence)
    {
      if (value != 255 && value > 100)
      {
        throw std::runtime_error(
            "Staged confidence layer contains a value above 100");
      }
    }

    const std::set<std::string> required = {
        "map.yaml",
        "map.pgm",
        metadata.elevation.file,
        metadata.slope.file,
        metadata.roughness.file,
        metadata.step.file,
        metadata.cost.file,
        metadata.confidence.file,
        "terrain_ground.pcd",
        "terrain_obstacles.pcd",
        "terrain_preview.ppm",
        "terrain_2p5d.yaml",
        boost::filesystem::path(input_pcd_).filename().string(),
        boost::filesystem::path(trajectory_pcd_).filename().string(),
        "mapping_snapshot.sha256"};
    const std::map<std::string, std::string> checksums = readChecksumFile(
        (stage / "terrain_checksums.sha256").string());
    for (const auto& file_name : required)
    {
      const auto found = checksums.find(file_name);
      if (found == checksums.end())
      {
        throw std::runtime_error("Missing staged checksum entry: " + file_name);
      }
      boost::filesystem::path file_path = stage / file_name;
      if (file_name == boost::filesystem::path(input_pcd_).filename().string())
      {
        file_path = input_pcd_;
      }
      else if (file_name ==
               boost::filesystem::path(trajectory_pcd_).filename().string())
      {
        file_path = trajectory_pcd_;
      }
      else if (file_name == "mapping_snapshot.sha256")
      {
        file_path = mapping_snapshot_;
      }
      if (!boost::filesystem::is_regular_file(file_path) ||
          sha256File(file_path.string()) != found->second)
      {
        throw std::runtime_error("Staged checksum mismatch: " + file_name);
      }
    }
    if (boost::filesystem::file_size(stage / "terrain_ground.pcd") < 100 ||
        boost::filesystem::file_size(stage / "terrain_obstacles.pcd") < 100 ||
        boost::filesystem::file_size(stage / "terrain_preview.ppm") < 100)
    {
      throw std::runtime_error("One or more staged diagnostic assets are empty");
    }
    (void)costs;
    ROS_INFO("Validated all staged map, terrain, diagnostic, and checksum assets");
  }

  void commitOutputs(const boost::filesystem::path& stage,
                     const boost::filesystem::path& destination) const
  {
    // Metadata is the commit marker and is therefore always installed last.
    const std::vector<std::string> files = {
        "map.pgm",
        "map.yaml",
        "terrain_elevation.f32",
        "terrain_slope.f32",
        "terrain_roughness.f32",
        "terrain_step.f32",
        "terrain_cost.u8",
        "terrain_confidence.u8",
        "terrain_ground.pcd",
        "terrain_obstacles.pcd",
        "terrain_preview.ppm",
        "terrain_checksums.sha256",
        "terrain_2p5d.yaml"};
    for (const auto& file : files)
    {
      atomicReplace(stage / file, destination / file);
    }
  }

  ros::NodeHandle private_nh_;
  std::string map_dir_;
  std::string export_id_;
  std::string input_pcd_;
  std::string trajectory_pcd_;
  std::string input_map_yaml_;
  std::string mapping_snapshot_;
  ExportParameters parameters_;
};

}  // namespace go2_terrain

int main(int argc, char** argv)
{
  ros::init(argc, argv, "go2_terrain_exporter");
  try
  {
    go2_terrain::TerrainExporter exporter;
    exporter.run();
  }
  catch (const std::exception& error)
  {
    // A one-shot required roslaunch node may terminate before rosconsole has
    // flushed its final line. stderr keeps the fail-closed reason visible to
    // both operators and deployment logs.
    std::cerr << "Terrain export failed: " << error.what() << std::endl;
    ROS_FATAL("Terrain export failed: %s", error.what());
    return 1;
  }
  return 0;
}
