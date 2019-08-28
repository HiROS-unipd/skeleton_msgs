#ifndef hiros_skeletons_types_h
#define hiros_skeletons_types_h

// Standard dependencies
#include <iostream>
#include <limits>
#include <vector>

namespace hiros {
  namespace skeletons {
    namespace types {

      // Point
      struct Point
      {
        Point(const double& t_x = std::numeric_limits<double>::quiet_NaN(),
              const double& t_y = std::numeric_limits<double>::quiet_NaN(),
              const double& t_z = std::numeric_limits<double>::quiet_NaN());

        friend std::ostream& operator<<(std::ostream& t_os, const Point& t_p);

        double x, y, z;
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
        Keypoint(const unsigned int& t_id = std::numeric_limits<unsigned int>::quiet_NaN(),
                 const double& t_confidence = std::numeric_limits<double>::quiet_NaN(),
                 const Point& t_point = Point());

        friend std::ostream& operator<<(std::ostream& t_os, const Keypoint& t_k);

        unsigned int id;
        double confidence;
        Point point;
      };

      // KeypointGroup
      struct KeypointGroup
      {
        KeypointGroup(
          const unsigned int& t_id = std::numeric_limits<unsigned int>::quiet_NaN(),
          const unsigned int& t_max_keypoints = std::numeric_limits<unsigned int>::quiet_NaN(),
          const double& t_confidence = std::numeric_limits<double>::quiet_NaN(),
          const Box& t_bounding_box = Box(),
          const std::vector<Keypoint>& t_keypoints = std::vector<Keypoint>());

        friend std::ostream& operator<<(std::ostream& t_os, const KeypointGroup& t_kg);

        unsigned int id, max_keypoints;
        double confidence;
        Box bounding_box;
        std::vector<Keypoint> keypoints;
      };

      // Skeleton
      struct Skeleton
      {
        Skeleton(const unsigned int& t_id = std::numeric_limits<unsigned int>::quiet_NaN(),
                 const std::vector<KeypointGroup>& t_skeleton_parts = std::vector<KeypointGroup>());

        friend std::ostream& operator<<(std::ostream& t_os, const Skeleton& t_s);

        unsigned int id;
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
