// Standard dependencies
#include <cmath>

// Internal dependencies
#include "skeletons/types.h"
#include "skeletons/utils.h"

namespace hiros {
  namespace skeletons {
    namespace types {

      // Vector
      std::ostream& operator<<(std::ostream& t_os, const tf2::Vector3& t_v)
      {
        return t_os << utils::toString(t_v);
      }

      // Point
      Point::Point(const Position& t_position,
                   const Velocity& t_velocity,
                   const Acceleration& t_acceleration)
        : position(t_position)
        , velocity(t_velocity)
        , acceleration(t_acceleration)
      {}

      std::ostream& operator<<(std::ostream& t_os, const Point& t_p)
      {
        return t_os << utils::toString(t_p);
      }

      // Quaternion
      std::ostream& operator<<(std::ostream& t_os, const tf2::Quaternion& t_q)
      {
        return t_os << utils::toString(t_q);
      }

      // Box
      Box::Box(const Point& t_center,
               const double& t_length,
               const double& t_height,
               const double& t_width,
               const tf2::Quaternion& t_orientation)
        : center(t_center)
        , length(t_length)
        , height(t_height)
        , width(t_width)
        , orientation(t_orientation)
      {}

      std::ostream& operator<<(std::ostream& t_os, const Box& t_b)
      {
        return t_os << utils::toString(t_b);
      }

      // Marker
      Marker::Marker(const int& t_id, const double& t_confidence, const Point& t_point)
        : id(t_id)
        , confidence(t_confidence)
        , point(t_point)
      {}

      std::ostream& operator<<(std::ostream& t_os, const Marker& t_m)
      {
        return t_os << utils::toString(t_m);
      }

      // MarkerGroup
      MarkerGroup::MarkerGroup(const int& t_id,
                               const unsigned int& t_max_markers,
                               const double& t_confidence,
                               const Box& t_bounding_box,
                               const std::vector<Marker>& t_markers)
        : id(t_id)
        , max_markers(t_max_markers)
        , confidence(t_confidence)
        , bounding_box(t_bounding_box)
      {
        for (auto& m : t_markers) {
          markers.emplace(m.id, m);
        }
      }

      bool MarkerGroup::addMarker(const Marker& t_marker)
      {
        if (markers.count(t_marker.id) > 0) {
          return false;
        }

        markers.emplace(t_marker.id, t_marker);
        return true;
      }

      std::ostream& operator<<(std::ostream& t_os, const MarkerGroup& t_mg)
      {
        return t_os << utils::toString(t_mg);
      }

      // MarkerSkeleton
      MarkerSkeleton::MarkerSkeleton(const int& t_id,
                                     const double& t_confidence,
                                     const std::vector<MarkerGroup>& t_marker_groups)
        : id(t_id)
        , confidence(t_confidence)
      {
        for (auto& mg : t_marker_groups) {
          marker_groups.emplace(mg.id, mg);
        }
      }

      bool MarkerSkeleton::addMarkerGroup(const MarkerGroup& t_marker_group)
      {
        if (marker_groups.count(t_marker_group.id) > 0) {
          return false;
        }

        marker_groups.emplace(t_marker_group.id, t_marker_group);
        return true;
      }

      std::ostream& operator<<(std::ostream& t_os, const MarkerSkeleton& t_ms)
      {
        return t_os << utils::toString(t_ms);
      }

      // MarkerSkeletonGroup
      MarkerSkeletonGroup::MarkerSkeletonGroup(
        const double& t_src_time,
        const std::string& t_src_frame,
        const std::vector<MarkerSkeleton>& t_marker_skeletons)
        : src_time(t_src_time)
        , src_frame(t_src_frame)
        , marker_skeletons(t_marker_skeletons)
      {}

      bool MarkerSkeletonGroup::addMarkerSkeleton(const MarkerSkeleton& t_marker_skeleton)
      {
        if (std::find_if(
              marker_skeletons.begin(),
              marker_skeletons.end(),
              [&t_marker_skeleton](const auto& s) { return s.id == t_marker_skeleton.id; })
            != marker_skeletons.end()) {
          return false;
        }

        marker_skeletons.emplace_back(t_marker_skeleton);
        return true;
      }

      std::ostream& operator<<(std::ostream& t_os, const MarkerSkeletonGroup& t_msg)
      {
        return t_os << utils::toString(t_msg);
      }

      // MIMU
      MIMU::MIMU(const std::string& t_frame_id,
                 const tf2::Quaternion& t_orientation,
                 const tf2::Vector3& t_angular_velocity,
                 const tf2::Vector3& t_linear_acceleration,
                 const tf2::Vector3& t_magnetic_field)
        : frame_id(t_frame_id)
        , orientation(t_orientation)
        , angular_velocity(t_angular_velocity)
        , linear_acceleration(t_linear_acceleration)
        , magnetic_field(t_magnetic_field)
      {}

      std::ostream& operator<<(std::ostream& t_os, const MIMU& t_m)
      {
        return t_os << utils::toString(t_m);
      }

      // Orientation
      Orientation::Orientation(const int& t_id, const double& t_confidence, const MIMU& t_mimu)
        : id(t_id)
        , confidence(t_confidence)
        , mimu(t_mimu)
      {}

      std::ostream& operator<<(std::ostream& t_os, const Orientation& t_o)
      {
        return t_os << utils::toString(t_o);
      }

      // OrientationGroup
      OrientationGroup::OrientationGroup(const int& t_id,
                                         const unsigned int& t_max_orientations,
                                         const double& t_confidence,
                                         const std::vector<Orientation>& t_orientations)
        : id(t_id)
        , max_orientations(t_max_orientations)
        , confidence(t_confidence)
      {
        for (auto& o : t_orientations) {
          orientations.emplace(o.id, o);
        }
      }

      bool OrientationGroup::addOrientation(const Orientation& t_orientation)
      {
        if (orientations.count(t_orientation.id) > 0) {
          return false;
        }

        orientations.emplace(t_orientation.id, t_orientation);
        return true;
      }

      std::ostream& operator<<(std::ostream& t_os, const OrientationGroup& t_og)
      {
        return t_os << utils::toString(t_og);
      }

      // OrientationSkeleton
      OrientationSkeleton::OrientationSkeleton(
        const int& t_id,
        const double& t_confidence,
        const std::vector<OrientationGroup>& t_orientation_groups)
        : id(t_id)
        , confidence(t_confidence)
      {
        for (auto& og : t_orientation_groups) {
          orientation_groups.emplace(og.id, og);
        }
      }

      bool OrientationSkeleton::addOrientationGroup(const OrientationGroup& t_orientation_group)
      {
        if (orientation_groups.count(t_orientation_group.id) > 0) {
          return false;
        }

        orientation_groups.emplace(t_orientation_group.id, t_orientation_group);
        return true;
      }

      std::ostream& operator<<(std::ostream& t_os, const OrientationSkeleton& t_osk)
      {
        return t_os << utils::toString(t_osk);
      }

      // OrientationSkeletonGroup
      OrientationSkeletonGroup::OrientationSkeletonGroup(
        const double& t_src_time,
        const std::string& t_src_frame,
        const std::vector<OrientationSkeleton>& t_orientation_skeletons)
        : src_time(t_src_time)
        , src_frame(t_src_frame)
        , orientation_skeletons(t_orientation_skeletons)
      {}

      bool OrientationSkeletonGroup::addOrientationSkeleton(
        const OrientationSkeleton& t_orientation_skeleton)
      {
        if (std::find_if(orientation_skeletons.begin(),
                         orientation_skeletons.end(),
                         [&t_orientation_skeleton](const auto& s) {
                           return s.id == t_orientation_skeleton.id;
                         })
            != orientation_skeletons.end()) {
          return false;
        }

        orientation_skeletons.emplace_back(t_orientation_skeleton);
        return true;
      }

      std::ostream& operator<<(std::ostream& t_os, const OrientationSkeletonGroup& t_osg)
      {
        return t_os << utils::toString(t_osg);
      }

    } // namespace types
  } // namespace skeletons
} // namespace hiros
