// Standard dependencies
#include <cmath>

// Internal dependencies
#include "skeletons/types.h"
#include "skeletons/utils.h"

namespace hiros {
  namespace skeletons {
    namespace types {

      // Vector
      Vector::Vector(const double& t_x, const double& t_y, const double& t_z)
        : x(t_x)
        , y(t_y)
        , z(t_z)
      {}

      Vector& Vector::operator+=(const Vector& t_vec)
      {
        x += t_vec.x;
        y += t_vec.y;
        z += t_vec.z;
        return *this;
      }

      Vector operator+(const Vector& t_v1, const Vector& t_v2)
      {
        Vector res = t_v1;
        return (res += t_v2);
      }

      Vector& Vector::operator-=(const Vector& t_vec)
      {
        x -= t_vec.x;
        y -= t_vec.y;
        z -= t_vec.z;
        return *this;
      }

      Vector operator-(const Vector& t_v1, const Vector& t_v2)
      {
        Vector res = t_v1;
        return (res -= t_v2);
      }

      Vector operator-(const Vector& t_vec)
      {
        Vector res;
        res.x = -t_vec.x;
        res.y = -t_vec.y;
        res.z = -t_vec.z;
        return res;
      }

      Vector& Vector::operator*=(const double& t_val)
      {
        x *= t_val;
        y *= t_val;
        z *= t_val;
        return *this;
      }

      Vector operator*(const Vector& t_vec, const double& t_val)
      {
        Vector res = t_vec;
        return (res *= t_val);
      }

      Vector operator*(const double& t_val, const Vector& t_vec)
      {
        Vector res = t_vec;
        return (res *= t_val);
      }

      Vector& Vector::operator/=(const double& t_val)
      {
        x /= t_val;
        y /= t_val;
        z /= t_val;
        return *this;
      }

      Vector operator/(const Vector& t_vec, const double& t_val)
      {
        Vector res = t_vec;
        return (res /= t_val);
      }

      std::ostream& operator<<(std::ostream& t_os, const Vector& t_v)
      {
        t_os << utils::padding(5) << "- x: " << t_v.x << std::endl
             << utils::padding(6) << "y: " << t_v.y;
        if (!std::isnan(t_v.z)) {
          t_os << std::endl << utils::padding(6) << "z: " << t_v.z;
        }
        return t_os;
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
        t_os << utils::padding(4) << "- position: " << std::endl << t_p.position;
        if (!std::isnan(t_p.velocity.x)) {
          t_os << std::endl << utils::padding(5) << "velocity: " << std::endl << t_p.velocity;
        }
        if (!std::isnan(t_p.acceleration.x)) {
          t_os << std::endl
               << utils::padding(5) << "acceleration: " << std::endl
               << t_p.acceleration;
        }
        return t_os;
      }

      // Quaternion
      Quaternion::Quaternion(const double& t_x,
                             const double& t_y,
                             const double& t_z,
                             const double& t_w)
        : x(t_x)
        , y(t_y)
        , z(t_z)
        , w(t_w)
      {}

      std::ostream& operator<<(std::ostream& t_os, const Quaternion& t_p)
      {
        t_os << utils::padding(4) << "- x: " << t_p.x << std::endl
             << utils::padding(5) << "y: " << t_p.y << std::endl
             << utils::padding(5) << "z: " << t_p.z << std::endl
             << utils::padding(5) << "w: " << t_p.w;
        return t_os;
      }

      // Box
      Box::Box(const Point& t_center,
               const double& t_length,
               const double& t_height,
               const double& t_width,
               const Quaternion& t_orientation)
        : center(t_center)
        , length(t_length)
        , height(t_height)
        , width(t_width)
        , orientation(t_orientation)
      {}

      std::ostream& operator<<(std::ostream& t_os, const Box& t_b)
      {
        t_os << utils::padding(3) << "- center: " << std::endl
             << t_b.center << std::endl
             << utils::padding(4) << "length: " << t_b.length << std::endl
             << utils::padding(4) << "height: " << t_b.height;
        if (!std::isnan(t_b.width)) {
          t_os << std::endl << utils::padding(4) << "width: " << t_b.width;
        }
        if (t_b.orientation.w != 1.0) {
          t_os << std::endl << utils::padding(4) << "orientation: " << std::endl << t_b.orientation;
        }
        return t_os;
      }

      // Marker
      Marker::Marker(const int& t_id, const double& t_confidence, const Point& t_point)
        : id(t_id)
        , confidence(t_confidence)
        , point(t_point)
      {}

      std::ostream& operator<<(std::ostream& t_os, const Marker& t_m)
      {
        t_os << utils::padding(3) << "- id: " << t_m.id << std::endl
             << utils::padding(4) << "confidence: " << t_m.confidence << std::endl
             << utils::padding(4) << "point: " << std::endl
             << t_m.point;
        return t_os;
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
        t_os << utils::padding(2) << "- id: " << t_mg.id << std::endl
             << utils::padding(3) << "max_markers: " << t_mg.max_markers << std::endl
             << utils::padding(3) << "confidence: " << t_mg.confidence << std::endl
             << utils::padding(3) << "bounding_box: " << std::endl
             << t_mg.bounding_box << std::endl
             << utils::padding(3) << "markers: ";
        if (t_mg.markers.empty()) {
          t_os << "[]";
        }
        else {
          for (auto m : t_mg.markers) {
            t_os << std::endl << m.second;
          }
        }
        return t_os;
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
        t_os << utils::padding(1) << "- id: " << t_ms.id << std::endl
             << utils::padding(2) << "confidence: " << t_ms.confidence << std::endl
             << utils::padding(2) << "marker_groups: ";
        if (t_ms.marker_groups.empty()) {
          t_os << "[]";
        }
        else {
          for (auto mg : t_ms.marker_groups) {
            t_os << std::endl << mg.second;
          }
        }
        return t_os;
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
        long src_time_sec = static_cast<long>(t_msg.src_time);
        long src_time_nsec = static_cast<long>((t_msg.src_time - src_time_sec) * 1e9);

        t_os << "- src_time: " << src_time_sec << "." << src_time_nsec << std::endl
             << utils::padding(1) << "src_frame: " << t_msg.src_frame << std::endl
             << utils::padding(1) << "marker_skeletons: ";
        if (t_msg.marker_skeletons.empty()) {
          t_os << "[]";
        }
        else {
          for (auto ms : t_msg.marker_skeletons) {
            t_os << std::endl << ms;
          }
        }
        return t_os;
      }

      // Orientation
      Orientation::Orientation(const int& t_id,
                               const double& t_confidence,
                               const Quaternion& t_orientation,
                               const Vector& t_angular_velocity,
                               const Vector& t_linear_acceleration)
        : id(t_id)
        , confidence(t_confidence)
        , orientation(t_orientation)
        , angular_velocity(t_angular_velocity)
        , linear_acceleration(t_linear_acceleration)
      {}

      std::ostream& operator<<(std::ostream& t_os, const Orientation& t_o)
      {
        t_os << utils::padding(3) << "- id: " << t_o.id << std::endl
             << utils::padding(4) << "confidence: " << t_o.confidence << std::endl
             << utils::padding(4) << "orientation: " << std::endl
             << t_o.orientation << utils::padding(4) << "angular_velocity: " << std::endl
             << t_o.angular_velocity << utils::padding(4) << "linear_acceleration: " << std::endl
             << t_o.linear_acceleration;
        return t_os;
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
        t_os << utils::padding(2) << "- id: " << t_og.id << std::endl
             << utils::padding(3) << "max_orientations: " << t_og.max_orientations << std::endl
             << utils::padding(3) << "confidence: " << t_og.confidence << std::endl
             << utils::padding(3) << "orientations: ";
        if (t_og.orientations.empty()) {
          t_os << "[]";
        }
        else {
          for (auto o : t_og.orientations) {
            t_os << std::endl << o.second;
          }
        }
        return t_os;
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
        t_os << utils::padding(1) << "- id: " << t_osk.id << std::endl
             << utils::padding(2) << "confidence: " << t_osk.confidence << std::endl
             << utils::padding(2) << "orientation_groups: ";
        if (t_osk.orientation_groups.empty()) {
          t_os << "[]";
        }
        else {
          for (auto og : t_osk.orientation_groups) {
            t_os << std::endl << og.second;
          }
        }
        return t_os;
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
        long src_time_sec = static_cast<long>(t_osg.src_time);
        long src_time_nsec = static_cast<long>((t_osg.src_time - src_time_sec) * 1e9);

        t_os << "- src_time: " << src_time_sec << "." << src_time_nsec << std::endl
             << utils::padding(1) << "src_frame: " << t_osg.src_frame << std::endl
             << utils::padding(1) << "orientation_skeletons: ";
        if (t_osg.orientation_skeletons.empty()) {
          t_os << "[]";
        }
        else {
          for (auto os : t_osg.orientation_skeletons) {
            t_os << std::endl << os;
          }
        }
        return t_os;
      }

    } // namespace types
  } // namespace skeletons
} // namespace hiros
