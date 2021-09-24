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
        , markers(t_markers)
      {
        markers.reserve(max_markers);
      }

      const Marker& MarkerGroup::getMarker(const int& t_id) const
      {
        for (const auto& marker : markers) {
          if (marker.id == t_id) {
            return marker;
          }
        }
        throw std::out_of_range("Marker " + std::to_string(t_id) + " not present in MarkerGroup "
                                + std::to_string(id));
      }

      Marker& MarkerGroup::getMarker(const int& t_id)
      {
        for (auto& marker : markers) {
          if (marker.id == t_id) {
            return marker;
          }
        }
        throw std::out_of_range("Marker " + std::to_string(t_id) + " not present in MarkerGroup "
                                + std::to_string(id));
      }

      bool MarkerGroup::hasMarker(const int& t_id) const
      {
        for (const auto& marker : markers) {
          if (marker.id == t_id) {
            return true;
          }
        }
        return false;
      }

      bool MarkerGroup::addMarker(const Marker& t_marker)
      {
        if (hasMarker(t_marker.id)) {
          return false;
        }

        markers.push_back(t_marker);
        return true;
      }

      bool MarkerGroup::removeMarker(const int& t_id)
      {
        if (!hasMarker(t_id)) {
          return false;
        }

        markers.erase(std::remove_if(
          markers.begin(), markers.end(), [&](const Marker& m) { return m.id == t_id; }));
        return true;
      }

      std::ostream& operator<<(std::ostream& t_os, const MarkerGroup& t_mg)
      {
        return t_os << utils::toString(t_mg);
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
        , orientations(t_orientations)
      {
        orientations.reserve(max_orientations);
      }

      const Orientation& OrientationGroup::getOrientation(const int& t_id) const
      {
        for (const auto& orientation : orientations) {
          if (orientation.id == t_id) {
            return orientation;
          }
        }
        throw std::out_of_range("Orientation " + std::to_string(t_id)
                                + " not present in OrientationGroup " + std::to_string(id));
      }

      Orientation& OrientationGroup::getOrientation(const int& t_id)
      {
        for (auto& orientation : orientations) {
          if (orientation.id == t_id) {
            return orientation;
          }
        }
        throw std::out_of_range("Orientation " + std::to_string(t_id)
                                + " not present in OrientationGroup " + std::to_string(id));
      }

      bool OrientationGroup::hasOrientation(const int& t_id) const
      {
        for (const auto& orientation : orientations) {
          if (orientation.id == t_id) {
            return true;
          }
        }
        return false;
      }

      bool OrientationGroup::addOrientation(const Orientation& t_orientation)
      {
        if (hasOrientation(t_orientation.id)) {
          return false;
        }

        orientations.push_back(t_orientation);
        return true;
      }

      bool OrientationGroup::removeOrientation(const int& t_id)
      {
        if (!hasOrientation(t_id)) {
          return false;
        }

        orientations.erase(std::remove_if(orientations.begin(),
                                          orientations.end(),
                                          [&](const Orientation& o) { return o.id == t_id; }));
        return true;
      }

      std::ostream& operator<<(std::ostream& t_os, const OrientationGroup& t_og)
      {
        return t_os << utils::toString(t_og);
      }

      // Skeleton
      Skeleton::Skeleton(const int& t_id,
                         const double& t_src_time,
                         const std::string t_src_frame,
                         const double& t_confidence,
                         const std::vector<MarkerGroup>& t_marker_groups,
                         const std::vector<OrientationGroup>& t_orientation_groups)
        : id(t_id)
        , src_time(t_src_time)
        , src_frame(t_src_frame)
        , confidence(t_confidence)
        , marker_groups(t_marker_groups)
        , orientation_groups(t_orientation_groups)
      {}

      const MarkerGroup& Skeleton::getMarkerGroup(const int& t_id) const
      {
        for (const auto& marker_group : marker_groups) {
          if (marker_group.id == t_id) {
            return marker_group;
          }
        }
        throw std::out_of_range("MarkerGroup " + std::to_string(t_id) + " not present in Skeleton "
                                + std::to_string(id));
      }

      MarkerGroup& Skeleton::getMarkerGroup(const int& t_id)
      {
        for (auto& marker_group : marker_groups) {
          if (marker_group.id == t_id) {
            return marker_group;
          }
        }
        throw std::out_of_range("MarkerGroup " + std::to_string(t_id) + " not present in Skeleton "
                                + std::to_string(id));
      }

      bool Skeleton::hasMarkerGroup(const int& t_id) const
      {
        for (const auto& marker_group : marker_groups) {
          if (marker_group.id == t_id) {
            return true;
          }
        }
        return false;
      }

      bool Skeleton::addMarkerGroup(const MarkerGroup& t_marker_group)
      {
        if (hasMarkerGroup(t_marker_group.id)) {
          return false;
        }

        marker_groups.push_back(t_marker_group);
        return true;
      }

      bool Skeleton::removeMarkerGroup(const int& t_id)
      {
        if (!hasMarkerGroup(t_id)) {
          return false;
        }

        marker_groups.erase(std::remove_if(marker_groups.begin(),
                                           marker_groups.end(),
                                           [&](const MarkerGroup& mg) { return mg.id == t_id; }));
        return true;
      }

      const OrientationGroup& Skeleton::getOrientationGroup(const int& t_id) const
      {
        for (const auto& orientation_group : orientation_groups) {
          if (orientation_group.id == t_id) {
            return orientation_group;
          }
        }
        throw std::out_of_range("OrientationGroup " + std::to_string(t_id)
                                + " not present in Skeleton " + std::to_string(id));
      }

      OrientationGroup& Skeleton::getOrientationGroup(const int& t_id)
      {
        for (auto& orientation_group : orientation_groups) {
          if (orientation_group.id == t_id) {
            return orientation_group;
          }
        }
        throw std::out_of_range("OrientationGroup " + std::to_string(t_id)
                                + " not present in Skeleton " + std::to_string(id));
      }

      bool Skeleton::hasOrientationGroup(const int& t_id) const
      {
        for (const auto& orientation_group : orientation_groups) {
          if (orientation_group.id == t_id) {
            return true;
          }
        }
        return false;
      }

      bool Skeleton::addOrientationGroup(const OrientationGroup& t_orientation_group)
      {
        if (hasOrientationGroup(t_orientation_group.id)) {
          return false;
        }

        orientation_groups.push_back(t_orientation_group);
        return true;
      }

      bool Skeleton::removeOrientationGroup(const int& t_id)
      {
        if (!hasOrientationGroup(t_id)) {
          return false;
        }

        orientation_groups.erase(std::remove_if(
          orientation_groups.begin(), orientation_groups.end(), [&](const OrientationGroup& og) {
            return og.id == t_id;
          }));
        return true;
      }

      std::ostream& operator<<(std::ostream& t_os, const Skeleton& t_s)
      {
        return t_os << utils::toString(t_s);
      }

      // SkeletonGroup
      SkeletonGroup::SkeletonGroup(const double& t_time,
                                   const std::string& t_frame,
                                   const std::vector<Skeleton>& t_skeletons)
        : time(t_time)
        , frame(t_frame)
        , skeletons(t_skeletons)
      {}

      const Skeleton& SkeletonGroup::getSkeleton(const int& t_id) const
      {
        for (const auto& skeleton : skeletons) {
          if (skeleton.id == t_id) {
            return skeleton;
          }
        }
        throw std::out_of_range("Skeleton " + std::to_string(t_id)
                                + " not present in SkeletonGroup");
      }

      Skeleton& SkeletonGroup::getSkeleton(const int& t_id)
      {
        for (auto& skeleton : skeletons) {
          if (skeleton.id == t_id) {
            return skeleton;
          }
        }
        throw std::out_of_range("Skeleton " + std::to_string(t_id)
                                + " not present in SkeletonGroup");
      }

      bool SkeletonGroup::hasSkeleton(const int& t_id) const
      {
        for (const auto& skeleton : skeletons) {
          if (skeleton.id == t_id) {
            return true;
          }
        }
        return false;
      }

      bool SkeletonGroup::addSkeleton(const Skeleton& t_skeleton)
      {
        if (hasSkeleton(t_skeleton.id)) {
          return false;
        }

        skeletons.push_back(t_skeleton);
        return true;
      }

      bool SkeletonGroup::removeSkeleton(const int& t_id)
      {
        if (!hasSkeleton(t_id)) {
          return false;
        }

        skeletons.erase(std::remove_if(skeletons.begin(),
                                       skeletons.end(),
                                       [&](const Skeleton& s) { return (s.id == t_id); }),
                        skeletons.end());
        return true;
      }

      std::ostream& operator<<(std::ostream& t_os, const SkeletonGroup& t_sg)
      {
        return t_os << utils::toString(t_sg);
      }

    } // namespace types
  } // namespace skeletons
} // namespace hiros
