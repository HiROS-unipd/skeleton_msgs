// Standard dependencies
#include <cmath>

// Internal dependencies
#include "skeletons/types.h"
#include "skeletons/utils.h"

namespace hiros {
  namespace skeletons {
    namespace types {

      // Point
      Point::Point(const double& t_x, const double& t_y, const double& t_z)
        : x(t_x)
        , y(t_y)
        , z(t_z)
      {}

      std::ostream& operator<<(std::ostream& t_os, const Point& t_p)
      {
        t_os << utils::padding(4) << "- x: " << t_p.x << std::endl
             << utils::padding(5) << "y: " << t_p.y;
        if (!std::isnan(t_p.z)) {
          t_os << std::endl << utils::padding(5) << "z: " << t_p.z;
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

      // Keypoint
      Keypoint::Keypoint(const unsigned int& t_id, const double& t_confidence, const Point& t_point)
        : id(t_id)
        , confidence(t_confidence)
        , point(t_point)
      {}

      std::ostream& operator<<(std::ostream& t_os, const Keypoint& t_k)
      {
        t_os << utils::padding(3) << "- id: " << t_k.id << std::endl
             << utils::padding(4) << "confidence: " << t_k.confidence << std::endl
             << utils::padding(4) << "point: " << std::endl
             << t_k.point;
        return t_os;
      }

      // KeypointGroup
      KeypointGroup::KeypointGroup(const unsigned int& t_id,
                                   const double& t_confidence,
                                   const Box& t_bounding_box,
                                   const std::vector<Keypoint>& t_keypoints)
        : id(t_id)
        , confidence(t_confidence)
        , bounding_box(t_bounding_box)
        , keypoints(t_keypoints)
      {}

      std::ostream& operator<<(std::ostream& t_os, const KeypointGroup& t_kg)
      {
        t_os << utils::padding(2) << "- id: " << t_kg.id << std::endl
             << utils::padding(3) << "confidence: " << t_kg.confidence << std::endl
             << utils::padding(3) << "bounding_box: " << std::endl
             << t_kg.bounding_box << std::endl
             << utils::padding(3) << "keypoints: ";
        if (t_kg.keypoints.empty()) {
          t_os << "[]";
        }
        else {
          for (auto k : t_kg.keypoints) {
            t_os << std::endl << k;
          }
        }
        return t_os;
      }

      // Skeleton
      Skeleton::Skeleton(const unsigned int& t_id,
                         const std::vector<KeypointGroup>& t_skeleton_parts)
        : id(t_id)
        , skeleton_parts(t_skeleton_parts)
      {}

      std::ostream& operator<<(std::ostream& t_os, const Skeleton& t_s)
      {
        t_os << utils::padding(1) << "- id: " << t_s.id << std::endl
             << utils::padding(2) << "skeleton_parts: ";
        if (t_s.skeleton_parts.empty()) {
          t_os << "[]";
        }
        else {
          for (auto sp : t_s.skeleton_parts) {
            t_os << std::endl << sp;
          }
        }
        return t_os;
      }

      // SkeletonGroup
      SkeletonGroup::SkeletonGroup(const std::vector<Skeleton>& t_skeletons)
        : skeletons(t_skeletons)
      {}

      std::ostream& operator<<(std::ostream& t_os, const SkeletonGroup& t_sg)
      {
        t_os << "skeletons: ";
        if (t_sg.skeletons.empty()) {
          t_os << "[]";
        }
        else {
          for (auto s : t_sg.skeletons) {
            t_os << std::endl << s;
          }
        }
        return t_os;
      }

    } // namespace types
  } // namespace skeletons
} // namespace hiros
