#ifndef hiros_skeletons_types_h
#define hiros_skeletons_types_h

// ROS dependencies
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"

// Standard dependencies
#include <iostream>
#include <limits>
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
      typedef tf2::Quaternion Quaternion;

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

        const Marker& getMarker(const int& t_id) const;
        Marker& getMarker(const int& t_id);
        bool hasMarker(const int& t_id) const;
        bool addMarker(const Marker& t_marker);
        bool removeMarker(const int& t_id);

        friend std::ostream& operator<<(std::ostream& t_os, const MarkerGroup& t_mg);

        int id;
        unsigned int max_markers;
        double confidence;
        Box bounding_box;
        std::vector<Marker> markers;
      };

      // MIMU
      struct MIMU
      {
        MIMU(const std::string& t_frame_id = "",
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
                            std::numeric_limits<double>::quiet_NaN()),
             const tf2::Vector3& t_magnetic_field =
               tf2::Vector3(std::numeric_limits<double>::quiet_NaN(),
                            std::numeric_limits<double>::quiet_NaN(),
                            std::numeric_limits<double>::quiet_NaN()));

        friend std::ostream& operator<<(std::ostream& t_os, const MIMU& t_m);

        std::string frame_id;
        tf2::Quaternion orientation;
        tf2::Vector3 angular_velocity;
        tf2::Vector3 linear_acceleration;
        tf2::Vector3 magnetic_field;
      };

      // Orientation
      struct Orientation
      {
        Orientation(const int& t_id = -1,
                    const double& t_confidence = std::numeric_limits<double>::quiet_NaN(),
                    const MIMU& t_mimu = MIMU());

        friend std::ostream& operator<<(std::ostream& t_os, const Orientation& t_o);

        int id;
        double confidence;
        MIMU mimu;
      };

      // OrientationGroup
      struct OrientationGroup
      {
        OrientationGroup(
          const int& t_id = -1,
          const unsigned int& t_max_orientations = 0,
          const double& t_confidence = std::numeric_limits<double>::quiet_NaN(),
          const std::vector<Orientation>& t_orientations = std::vector<Orientation>());

        const Orientation& getOrientation(const int& t_id) const;
        Orientation& getOrientation(const int& t_id);
        bool hasOrientation(const int& t_id) const;
        bool addOrientation(const Orientation& t_orientation);
        bool removeOrientation(const int& t_id);

        friend std::ostream& operator<<(std::ostream& t_os, const OrientationGroup& t_og);

        int id;
        unsigned int max_orientations;
        double confidence;
        std::vector<Orientation> orientations;
      };

      // Skeleton
      struct Skeleton
      {
        Skeleton(const int& t_id = -1,
                 const double& t_src_time = std::numeric_limits<double>::quiet_NaN(),
                 const std::string t_src_frame = "",
                 const double& t_confidence = std::numeric_limits<double>::quiet_NaN(),
                 const std::vector<MarkerGroup>& t_marker_groups = std::vector<MarkerGroup>(),
                 const std::vector<OrientationGroup>& t_orientation_groups =
                   std::vector<OrientationGroup>());

        const MarkerGroup& getMarkerGroup(const int& t_id) const;
        MarkerGroup& getMarkerGroup(const int& t_id);
        bool hasMarkerGroup(const int& t_id) const;
        bool addMarkerGroup(const MarkerGroup& t_marker_group);
        bool removeMarkerGroup(const int& t_id);

        const OrientationGroup& getOrientationGroup(const int& t_id) const;
        OrientationGroup& getOrientationGroup(const int& t_id);
        bool hasOrientationGroup(const int& t_id) const;
        bool addOrientationGroup(const OrientationGroup& t_orientation_group);
        bool removeOrientationGroup(const int& t_id);

        friend std::ostream& operator<<(std::ostream& t_os, const Skeleton& t_s);

        int id;
        double src_time;
        std::string src_frame;
        double confidence;
        std::vector<MarkerGroup> marker_groups;
        std::vector<OrientationGroup> orientation_groups;
      };

      // SkeletonGroup
      struct SkeletonGroup
      {
        SkeletonGroup(const double& t_time = std::numeric_limits<double>::quiet_NaN(),
                      const std::string& t_frame = "",
                      const std::vector<Skeleton>& t_skeletons = std::vector<Skeleton>());

        const Skeleton& getSkeleton(const int& t_id) const;
        Skeleton& getSkeleton(const int& t_id);
        bool hasSkeleton(const int& t_id) const;
        bool addSkeleton(const Skeleton& t_skeleton);
        bool removeSkeleton(const int& t_id);

        friend std::ostream& operator<<(std::ostream& t_os, const SkeletonGroup& t_sg);

        double time;
        std::string frame;
        std::vector<Skeleton> skeletons;
      };

    } // namespace types
  } // namespace skeletons
} // namespace hiros

#endif
