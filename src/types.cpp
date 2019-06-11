#include "skeletons/types.h"

#include <cmath>

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
        t_os << "        - x: " << t_p.x << std::endl << "          y: " << t_p.y;
        if (!std::isnan(t_p.z)) {
          t_os << std::endl << "          z: " << t_p.z;
        }
        return t_os;
      }

      // Rectangle
      Rectangle::Rectangle(const double& t_x,
                           const double& t_y,
                           const double& t_width,
                           const double& t_height)
        : x(t_x)
        , y(t_y)
        , width(t_width)
        , height(t_height)
      {}

      std::ostream& operator<<(std::ostream& t_os, const Rectangle& t_r)
      {
        t_os << "      - x: " << t_r.x << std::endl
             << "        y: " << t_r.y << std::endl
             << "        width: " << t_r.width << std::endl
             << "        height: " << t_r.height;
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
        t_os << "      - id: " << t_k.id << std::endl
             << "        confidence: " << t_k.confidence << std::endl
             << "        point: " << std::endl
             << t_k.point;
        return t_os;
      }

      // KeypointGroup
      KeypointGroup::KeypointGroup(const unsigned int& t_id,
                                   const double& t_confidence,
                                   const std::vector<Keypoint>& t_keypoints,
                                   const std::vector<Rectangle>& t_bounding_boxes)
        : id(t_id)
        , confidence(t_confidence)
        , bounding_boxes(t_bounding_boxes)
        , keypoints(t_keypoints)
      {}

      std::ostream& operator<<(std::ostream& t_os, const KeypointGroup& t_kg)
      {
        t_os << "    - id: " << t_kg.id << std::endl << "      confidence: " << t_kg.confidence;
        t_os << std::endl << "      bounding_boxes: ";
        if (t_kg.bounding_boxes.empty()) {
          t_os << "[]";
        }
        else {
          for (auto bb : t_kg.bounding_boxes) {
            t_os << std::endl << bb;
          }
        }
        t_os << std::endl << "      keypoints: ";
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
        t_os << "  - id: " << t_s.id << std::endl << "    skeleton_parts: ";
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
