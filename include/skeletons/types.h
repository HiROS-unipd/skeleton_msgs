#ifndef hiros_skeletons_types_h
#define hiros_skeletons_types_h

// Standard dependencies
#include <iostream>
#include <limits>
#include <map>
#include <vector>

namespace hiros {
  namespace skeletons {
    namespace types {

      // Vector
      struct Vector
      {
        Vector(const double& t_x = std::numeric_limits<double>::quiet_NaN(),
               const double& t_y = std::numeric_limits<double>::quiet_NaN(),
               const double& t_z = std::numeric_limits<double>::quiet_NaN());

        Vector& operator+=(const Vector& t_vec);
        friend Vector operator+(const Vector& t_v1, const Vector& t_v2);

        Vector& operator-=(const Vector& t_vec);
        friend Vector operator-(const Vector& t_v1, const Vector& t_v2);
        friend Vector operator-(const Vector& t_vec);

        Vector& operator*=(const double& t_val);
        friend Vector operator*(const Vector& t_vec, const double& t_val);
        friend Vector operator*(const double& t_val, const Vector& t_vec);

        Vector& operator/=(const double& t_val);
        friend Vector operator/(const Vector& t_vec, const double& t_val);

        friend std::ostream& operator<<(std::ostream& t_os, const Vector& t_v);

        double x, y, z;
      };

      typedef Vector Position;
      typedef Vector Velocity;
      typedef Vector Acceleration;

      // Point
      struct Point
      {
        Point(const Position& t_position = Position(),
              const Velocity& t_velocity = Velocity(),
              const Acceleration& t_acceleration = Acceleration());

        friend std::ostream& operator<<(std::ostream& t_os, const Point& t_p);

        Position position;
        Velocity velocity;
        Acceleration acceleration;
      };

      // Quaternion
      struct Quaternion
      {
        Quaternion(const double& t_x = 0.0,
                   const double& t_y = 0.0,
                   const double& t_z = 0.0,
                   const double& t_w = 1.0);

        friend std::ostream& operator<<(std::ostream& t_os, const Quaternion& t_q);

        double x, y, z, w;
      };

      // Box
      struct Box
      {
        Box(const Point& t_center = Point(),
            const double& t_length = std::numeric_limits<double>::quiet_NaN(),
            const double& t_height = std::numeric_limits<double>::quiet_NaN(),
            const double& t_width = std::numeric_limits<double>::quiet_NaN(),
            const Quaternion& t_orientation = Quaternion());

        friend std::ostream& operator<<(std::ostream& t_os, const Box& t_b);

        Point center;
        double length, height, width;
        Quaternion orientation;
      };

      // Marker
      struct Marker
      {
        Marker(const int& t_id = -1,
               const double& t_confidence = std::numeric_limits<double>::quiet_NaN(),
               const Point& t_point = Point());

        friend std::ostream& operator<<(std::ostream& t_os, const Marker& t_m);

        int id;
        double confidence;
        Point point;
      };

      // MarkerGroup
      struct MarkerGroup
      {
        MarkerGroup(const int& t_id = -1,
                    const unsigned int& t_max_markers = 0,
                    const double& t_confidence = std::numeric_limits<double>::quiet_NaN(),
                    const Box& t_bounding_box = Box(),
                    const std::vector<Marker>& t_markers = std::vector<Marker>());

        friend std::ostream& operator<<(std::ostream& t_os, const MarkerGroup& t_mg);

        int id;
        unsigned int max_markers;
        double confidence;
        Box bounding_box;
        // map<marker_id, marker>
        std::map<int, Marker> markers;
      };

      // MarkerSkeleton
      struct MarkerSkeleton
      {
        MarkerSkeleton(
          const int& t_id = -1,
          const double& t_confidence = std::numeric_limits<double>::quiet_NaN(),
          const std::vector<MarkerGroup>& t_marker_groups = std::vector<MarkerGroup>());

        friend std::ostream& operator<<(std::ostream& t_os, const MarkerSkeleton& t_ms);

        int id;
        double confidence;
        // map<marker_group_id, marker_group>
        std::map<int, MarkerGroup> marker_groups;
      };

      // MarkerSkeletonGroup
      struct MarkerSkeletonGroup
      {
        MarkerSkeletonGroup(
          const double& t_src_time = std::numeric_limits<double>::quiet_NaN(),
          const std::string& t_src_frame = "",
          const std::vector<MarkerSkeleton>& t_marker_skeletons = std::vector<MarkerSkeleton>());

        friend std::ostream& operator<<(std::ostream& t_os, const MarkerSkeletonGroup& t_msg);

        double src_time;
        std::string src_frame;
        std::vector<MarkerSkeleton> marker_skeletons;
      };

    } // namespace types
  } // namespace skeletons
} // namespace hiros

#endif
