#ifndef hiros_skeletons_types_h
#define hiros_skeletons_types_h

// ROS dependencies
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"

// Standard dependencies
#include <iostream>
#include <limits>
#include <map>
#include <vector>

namespace hiros {
  namespace skeletons {
    namespace types {

      // Vector
      typedef tf2::Vector3 Position;
      typedef tf2::Vector3 Velocity;
      typedef tf2::Vector3 Acceleration;

      std::ostream& operator<<(std::ostream& t_os, const tf2::Vector3& t_v);

      // Point
      struct Point
      {
        Point(const Position& t_position = Position(std::numeric_limits<double>::quiet_NaN(),
                                                    std::numeric_limits<double>::quiet_NaN(),
                                                    std::numeric_limits<double>::quiet_NaN()),
              const Velocity& t_velocity = Velocity(std::numeric_limits<double>::quiet_NaN(),
                                                    std::numeric_limits<double>::quiet_NaN(),
                                                    std::numeric_limits<double>::quiet_NaN()),
              const Acceleration& t_acceleration =
                Acceleration(std::numeric_limits<double>::quiet_NaN(),
                             std::numeric_limits<double>::quiet_NaN(),
                             std::numeric_limits<double>::quiet_NaN()));

        friend std::ostream& operator<<(std::ostream& t_os, const Point& t_p);

        Position position;
        Velocity velocity;
        Acceleration acceleration;
      };

      // Quaternion
      std::ostream& operator<<(std::ostream& t_os, const tf2::Quaternion& t_q);

      // Box
      struct Box
      {
        Box(const Point& t_center = Point(),
            const double& t_length = std::numeric_limits<double>::quiet_NaN(),
            const double& t_height = std::numeric_limits<double>::quiet_NaN(),
            const double& t_width = std::numeric_limits<double>::quiet_NaN(),
            const tf2::Quaternion& t_orientation =
              tf2::Quaternion(std::numeric_limits<double>::quiet_NaN(),
                              std::numeric_limits<double>::quiet_NaN(),
                              std::numeric_limits<double>::quiet_NaN(),
                              std::numeric_limits<double>::quiet_NaN()));

        friend std::ostream& operator<<(std::ostream& t_os, const Box& t_b);

        Point center;
        double length, height, width;
        tf2::Quaternion orientation;
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

        bool addMarker(const Marker& t_marker);

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

        bool addMarkerGroup(const MarkerGroup& t_marker_group);

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

        bool addMarkerSkeleton(const MarkerSkeleton& t_marker_skeleton);

        friend std::ostream& operator<<(std::ostream& t_os, const MarkerSkeletonGroup& t_msg);

        double src_time;
        std::string src_frame;
        std::vector<MarkerSkeleton> marker_skeletons;
      };

      // Orientation
      struct Orientation
      {
        Orientation(const int& t_id = -1,
                    const std::string& t_frame_id = "",
                    const double& t_confidence = std::numeric_limits<double>::quiet_NaN(),
                    const tf2::Quaternion& t_orientation =
                      tf2::Quaternion(std::numeric_limits<double>::quiet_NaN(),
                                      std::numeric_limits<double>::quiet_NaN(),
                                      std::numeric_limits<double>::quiet_NaN(),
                                      std::numeric_limits<double>::quiet_NaN()),
                    const tf2::Vector3& t_angular_velocity =
                      tf2::Vector3(std::numeric_limits<double>::quiet_NaN(),
                                   std::numeric_limits<double>::quiet_NaN(),
                                   std::numeric_limits<double>::quiet_NaN()),
                    const tf2::Vector3& t_linear_acceleration =
                      tf2::Vector3(std::numeric_limits<double>::quiet_NaN(),
                                   std::numeric_limits<double>::quiet_NaN(),
                                   std::numeric_limits<double>::quiet_NaN()));

        friend std::ostream& operator<<(std::ostream& t_os, const Orientation& t_o);

        int id;
        std::string frame_id;
        double confidence;
        tf2::Quaternion orientation;
        tf2::Vector3 angular_velocity;
        tf2::Vector3 linear_acceleration;
      };

      // OrientationGroup
      struct OrientationGroup
      {
        OrientationGroup(
          const int& t_id = -1,
          const unsigned int& t_max_orientations = 0,
          const double& t_confidence = std::numeric_limits<double>::quiet_NaN(),
          const std::vector<Orientation>& t_orientations = std::vector<Orientation>());

        bool addOrientation(const Orientation& t_orientation);

        friend std::ostream& operator<<(std::ostream& t_os, const OrientationGroup& t_og);

        int id;
        unsigned int max_orientations;
        double confidence;
        // map<orientation_id, orientation>
        std::map<int, Orientation> orientations;
      };

      // OrientationSkeleton
      struct OrientationSkeleton
      {
        OrientationSkeleton(const int& t_id = -1,
                            const double& t_confidence = std::numeric_limits<double>::quiet_NaN(),
                            const std::vector<OrientationGroup>& t_orientation_groups =
                              std::vector<OrientationGroup>());

        bool addOrientationGroup(const OrientationGroup& t_orientation_group);

        friend std::ostream& operator<<(std::ostream& t_os, const MarkerSkeleton& t_osk);

        int id;
        double confidence;
        // map<orientation_group_id, orientation_group>
        std::map<int, OrientationGroup> orientation_groups;
      };

      // OrientationSkeletonGroup
      struct OrientationSkeletonGroup
      {
        OrientationSkeletonGroup(
          const double& t_src_time = std::numeric_limits<double>::quiet_NaN(),
          const std::string& t_src_frame = "",
          const std::vector<OrientationSkeleton>& t_orientation_skeletons =
            std::vector<OrientationSkeleton>());

        bool addOrientationSkeleton(const OrientationSkeleton& t_orientation_skeleton);

        friend std::ostream& operator<<(std::ostream& t_os, const MarkerSkeletonGroup& t_osg);

        double src_time;
        std::string src_frame;
        std::vector<OrientationSkeleton> orientation_skeletons;
      };

    } // namespace types
  } // namespace skeletons
} // namespace hiros

#endif
