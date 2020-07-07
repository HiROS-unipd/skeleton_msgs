#ifndef hiros_skeletons_types_h
#define hiros_skeletons_types_h

// Standard dependencies
#include <iostream>
#include <limits>
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

      // Keypoint
      struct Keypoint
      {
        Keypoint(const int& t_id = -1,
                 const double& t_confidence = std::numeric_limits<double>::quiet_NaN(),
                 const Point& t_point = Point());

        friend std::ostream& operator<<(std::ostream& t_os, const Keypoint& t_k);

        int id;
        double confidence;
        Point point;
      };

      // KeypointGroup
      struct KeypointGroup
      {
        KeypointGroup(const int& t_id = -1,
                      const unsigned int& t_max_keypoints = 0,
                      const double& t_confidence = std::numeric_limits<double>::quiet_NaN(),
                      const Box& t_bounding_box = Box(),
                      const std::vector<Keypoint>& t_keypoints = std::vector<Keypoint>());

        friend std::ostream& operator<<(std::ostream& t_os, const KeypointGroup& t_kg);

        int id;
        unsigned int max_keypoints;
        double confidence;
        Box bounding_box;
        std::vector<Keypoint> keypoints;
      };

      // Skeleton
      struct Skeleton
      {
        Skeleton(const int& t_id = -1,
                 const std::vector<KeypointGroup>& t_skeleton_parts = std::vector<KeypointGroup>());

        friend std::ostream& operator<<(std::ostream& t_os, const Skeleton& t_s);

        int id;
        std::vector<KeypointGroup> skeleton_parts;
      };

      // SkeletonGroup
      struct SkeletonGroup
      {
        SkeletonGroup(const std::vector<Skeleton>& t_skeletons = std::vector<Skeleton>());

        friend std::ostream& operator<<(std::ostream& t_os, const SkeletonGroup& t_sg);

        std::vector<Skeleton> skeletons;
      };

    } // namespace types
  } // namespace skeletons
} // namespace hiros

#endif
