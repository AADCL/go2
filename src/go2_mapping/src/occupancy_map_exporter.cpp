#include <ros/ros.h>



#include <pcl/io/pcd_io.h>

#include <pcl/point_cloud.h>

#include <pcl/point_types.h>



#include <algorithm>

#include <cmath>

#include <cstdint>

#include <fstream>

#include <iomanip>

#include <limits>

#include <string>

#include <stdexcept>

#include <vector>



class PcdToPgm

{

public:

  PcdToPgm()

      : pnh_("~")

  {

    pnh_.param<std::string>(

        "input_pcd",

        input_pcd_,

        std::string());

    pnh_.param<std::string>(

        "output_pgm",

        output_pgm_,

        std::string());



    pnh_.param<std::string>(

        "output_yaml",

        output_yaml_,

        std::string());

    if (input_pcd_.empty() || output_pgm_.empty() || output_yaml_.empty())
    {
      throw std::runtime_error(
          "input_pcd, output_pgm, and output_yaml parameters are required");
    }



    pnh_.param<double>("resolution", resolution_, 0.05);

    pnh_.param<double>("padding_m", padding_m_, 0.50);



    // Ground evidence.

    pnh_.param<double>("floor_min_z", floor_min_z_, -0.15);

    pnh_.param<double>("floor_max_z", floor_max_z_, 0.08);



    // Obstacles relevant to GO2 body collision.

    pnh_.param<double>("obstacle_min_z", obstacle_min_z_, 0.08);

    pnh_.param<double>("obstacle_max_z", obstacle_max_z_, 0.70);



    // Fill small holes in floor observations.

    pnh_.param<double>("free_dilation_m", free_dilation_m_, 0.15);



    // Robot safety inflation.

    pnh_.param<double>("obstacle_inflation_m", obstacle_inflation_m_, 0.30);



    pnh_.param<int>("min_floor_points", min_floor_points_, 1);

    pnh_.param<int>("min_obstacle_points", min_obstacle_points_, 1);



    pnh_.param<int>("unknown_value", unknown_value_, 205);

    pnh_.param<int>("free_value", free_value_, 254);

    pnh_.param<int>("occupied_value", occupied_value_, 0);



    run();

  }



private:

  static bool finitePoint(const pcl::PointXYZI& p)

  {

    return std::isfinite(p.x) &&

           std::isfinite(p.y) &&

           std::isfinite(p.z);

  }



  int index(int x, int y) const

  {

    return y * width_ + x;

  }



  bool inside(int x, int y) const

  {

    return x >= 0 && x < width_ &&

           y >= 0 && y < height_;

  }



  void dilateMask(const std::vector<uint8_t>& input,

                  std::vector<uint8_t>& output,

                  int radius_cells)

  {

    output = input;



    if (radius_cells <= 0)

      return;



    for (int y = 0; y < height_; ++y)

    {

      for (int x = 0; x < width_; ++x)

      {

        if (!input[index(x, y)])

          continue;



        for (int dy = -radius_cells; dy <= radius_cells; ++dy)

        {

          for (int dx = -radius_cells; dx <= radius_cells; ++dx)

          {

            if (dx * dx + dy * dy >

                radius_cells * radius_cells)

              continue;



            const int nx = x + dx;

            const int ny = y + dy;



            if (inside(nx, ny))

              output[index(nx, ny)] = 1;

          }

        }

      }

    }

  }



  void writePgm(const std::vector<uint8_t>& grid)

  {

    std::ofstream out(output_pgm_.c_str(),

                      std::ios::out | std::ios::binary);



    if (!out)

      throw std::runtime_error(

          "Cannot open output PGM: " + output_pgm_);



    out << "P5\n";

    out << width_ << " " << height_ << "\n";

    out << "255\n";



    // PGM first row is image top.

    // ROS occupancy coordinates use origin at lower-left.

    // Therefore flip Y while writing.

    for (int image_y = height_ - 1;

         image_y >= 0;

         --image_y)

    {

      for (int x = 0; x < width_; ++x)

      {

        const uint8_t v = grid[index(x, image_y)];

        out.write(reinterpret_cast<const char*>(&v), 1);

      }

    }



    out.close();

  }



  void writeYaml()

  {

    std::ofstream out(output_yaml_.c_str());



    if (!out)

      throw std::runtime_error(

          "Cannot open output YAML: " + output_yaml_);



    // map_server resolves a relative image path against YAML directory.

    const std::size_t slash = output_pgm_.find_last_of('/');

    std::string image_name = output_pgm_;



    if (slash != std::string::npos)

      image_name = output_pgm_.substr(slash + 1);



    out << "image: " << image_name << "\n";



    out << std::fixed << std::setprecision(6);



    out << "resolution: " << resolution_ << "\n";



    out << "origin: ["

        << min_x_ << ", "

        << min_y_ << ", 0.0]\n";



    out << "negate: 0\n";

    out << "occupied_thresh: 0.65\n";

    out << "free_thresh: 0.196\n";



    out.close();

  }



  void run()

  {

    ROS_INFO("Loading PCD: %s", input_pcd_.c_str());



    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(

        new pcl::PointCloud<pcl::PointXYZI>());



    if (pcl::io::loadPCDFile<pcl::PointXYZI>(

            input_pcd_, *cloud) != 0)

    {

      ROS_FATAL("Failed to load PCD: %s",

                input_pcd_.c_str());

      ros::shutdown();

      return;

    }



    ROS_INFO("Loaded PCD points=%zu", cloud->size());



    if (cloud->empty())

    {

      ROS_FATAL("Input PCD is empty.");

      ros::shutdown();

      return;

    }



    double xmin = std::numeric_limits<double>::max();

    double ymin = std::numeric_limits<double>::max();



    double xmax = -std::numeric_limits<double>::max();

    double ymax = -std::numeric_limits<double>::max();



    size_t finite_count = 0;



    for (const auto& p : cloud->points)

    {

      if (!finitePoint(p))

        continue;



      xmin = std::min(xmin, static_cast<double>(p.x));

      xmax = std::max(xmax, static_cast<double>(p.x));



      ymin = std::min(ymin, static_cast<double>(p.y));

      ymax = std::max(ymax, static_cast<double>(p.y));



      ++finite_count;

    }



    if (finite_count == 0)

    {

      ROS_FATAL("No finite points in PCD.");

      ros::shutdown();

      return;

    }



    min_x_ = xmin - padding_m_;

    min_y_ = ymin - padding_m_;



    const double max_x = xmax + padding_m_;

    const double max_y = ymax + padding_m_;



    width_ =

        static_cast<int>(

            std::ceil((max_x - min_x_) / resolution_)) + 1;



    height_ =

        static_cast<int>(

            std::ceil((max_y - min_y_) / resolution_)) + 1;



    ROS_INFO(

        "Map bounds x=[%.3f %.3f] y=[%.3f %.3f]",

        min_x_, max_x, min_y_, max_y);



    ROS_INFO(

        "Map size=%d x %d resolution=%.3f",

        width_, height_, resolution_);



    const size_t n =

        static_cast<size_t>(width_) *

        static_cast<size_t>(height_);



    std::vector<int> floor_count(n, 0);

    std::vector<int> obstacle_count(n, 0);



    for (const auto& p : cloud->points)

    {

      if (!finitePoint(p))

        continue;



      const int gx =

          static_cast<int>(

              std::floor((p.x - min_x_) / resolution_));



      const int gy =

          static_cast<int>(

              std::floor((p.y - min_y_) / resolution_));



      if (!inside(gx, gy))

        continue;



      const int id = index(gx, gy);



      if (p.z >= floor_min_z_ &&

          p.z <= floor_max_z_)

      {

        floor_count[id]++;

      }



      if (p.z >= obstacle_min_z_ &&

          p.z <= obstacle_max_z_)

      {

        obstacle_count[id]++;

      }

    }



    std::vector<uint8_t> floor_mask(n, 0);

    std::vector<uint8_t> obstacle_mask(n, 0);



    for (size_t i = 0; i < n; ++i)

    {

      if (floor_count[i] >= min_floor_points_)

        floor_mask[i] = 1;



      if (obstacle_count[i] >= min_obstacle_points_)

        obstacle_mask[i] = 1;

    }



    const int free_radius =

        static_cast<int>(

            std::round(free_dilation_m_ / resolution_));



    const int obstacle_radius =

        static_cast<int>(

            std::round(obstacle_inflation_m_ / resolution_));



    std::vector<uint8_t> free_dilated;

    std::vector<uint8_t> obstacle_inflated;



    dilateMask(

        floor_mask,

        free_dilated,

        free_radius);



    dilateMask(

        obstacle_mask,

        obstacle_inflated,

        obstacle_radius);



    std::vector<uint8_t> image(

        n,

        static_cast<uint8_t>(unknown_value_));



    // Free evidence first.

    for (size_t i = 0; i < n; ++i)

    {

      if (free_dilated[i])

        image[i] =

            static_cast<uint8_t>(free_value_);

    }



    // Obstacles always override free evidence.

    for (size_t i = 0; i < n; ++i)

    {

      if (obstacle_inflated[i])

        image[i] =

            static_cast<uint8_t>(occupied_value_);

    }



    writePgm(image);

    writeYaml();



    size_t free_cells = 0;

    size_t occ_cells = 0;

    size_t unknown_cells = 0;



    for (const auto& v : image)

    {

      if (v == static_cast<uint8_t>(free_value_))

        ++free_cells;

      else if (v == static_cast<uint8_t>(occupied_value_))

        ++occ_cells;

      else

        ++unknown_cells;

    }



    ROS_INFO("PGM saved: %s", output_pgm_.c_str());

    ROS_INFO("YAML saved: %s", output_yaml_.c_str());



    ROS_INFO(

        "cells free=%zu occupied=%zu unknown=%zu",

        free_cells,

        occ_cells,

        unknown_cells);



    ros::shutdown();

  }



private:

  ros::NodeHandle pnh_;



  std::string input_pcd_;

  std::string output_pgm_;

  std::string output_yaml_;



  double resolution_;

  double padding_m_;



  double floor_min_z_;

  double floor_max_z_;



  double obstacle_min_z_;

  double obstacle_max_z_;



  double free_dilation_m_;

  double obstacle_inflation_m_;



  int min_floor_points_;

  int min_obstacle_points_;



  int unknown_value_;

  int free_value_;

  int occupied_value_;



  int width_;

  int height_;



  double min_x_;

  double min_y_;

};



int main(int argc, char** argv)

{

  ros::init(argc, argv, "pcd_to_pgm");



  try

  {

    PcdToPgm converter;

  }

  catch (const std::exception& e)

  {

    ROS_FATAL("pcd_to_pgm exception: %s", e.what());

    return 1;

  }



  return 0;

}
