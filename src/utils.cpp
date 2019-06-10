#include "skeletons/utils.h"

namespace hiros {
  namespace skeletons {
    namespace utils {

      // Point
      hiros::skeletons::types::Point
      toStruct(const double& t_x, const double& t_y, const double& t_z)
      {
        hiros::skeletons::types::Point p;
        p.x = t_x;
        p.y = t_y;
        p.z = t_z;
        return p;
      }

      hiros::skeletons::types::Point toStruct(const geometry_msgs::Point& t_p)
      {
        hiros::skeletons::types::Point p;
        p.x = t_p.x;
        p.y = t_p.y;
        p.z = t_p.z;
        return p;
      }

      geometry_msgs::Point toMsg(const hiros::skeletons::types::Point& t_p)
      {
        geometry_msgs::Point p;
        p.x = t_p.x;
        p.y = t_p.y;
        p.z = t_p.z;
        return p;
      }

      // Rectangle
      hiros::skeletons::types::Rectangle
      toStruct(const double& t_x, const double& t_y, const double& t_width, const double& t_height)
      {
        hiros::skeletons::types::Rectangle r;
        r.x = t_x;
        r.y = t_y;
        r.width = t_width;
        r.height = t_height;
        return r;
      }

      hiros::skeletons::types::Rectangle toStruct(const skeleton_msgs::Rectangle& t_r)
      {
        hiros::skeletons::types::Rectangle r;
        r.x = t_r.x;
        r.y = t_r.y;
        r.width = t_r.width;
        r.height = t_r.height;
        return r;
      }

      skeleton_msgs::Rectangle toMsg(const hiros::skeletons::types::Rectangle& t_r)
      {
        skeleton_msgs::Rectangle r;
        r.x = t_r.x;
        r.y = t_r.y;
        r.width = t_r.width;
        r.height = t_r.height;
        return r;
      }

      // Keypoint
      hiros::skeletons::types::Keypoint toStruct(const unsigned& t_id,
                                                 const double& t_confidence,
                                                 const double& t_x,
                                                 const double& t_y,
                                                 const double& t_z)
      {
        hiros::skeletons::types::Keypoint k;
        k.id = t_id;
        k.confidence = t_confidence;
        k.point = toStruct(t_x, t_y, t_z);
        return k;
      }

      hiros::skeletons::types::Keypoint toStruct(const unsigned& t_id,
                                                 const double& t_confidence,
                                                 const hiros::skeletons::types::Point& t_point)
      {
        hiros::skeletons::types::Keypoint k;
        k.id = t_id;
        k.confidence = t_confidence;
        k.point = t_point;
        return k;
      }

      hiros::skeletons::types::Keypoint toStruct(const skeleton_msgs::Keypoint& t_k)
      {
        hiros::skeletons::types::Keypoint k;
        k.id = static_cast<unsigned>(t_k.id);
        k.confidence = t_k.confidence;
        k.point = toStruct(t_k.point);
        return k;
      }

      skeleton_msgs::Keypoint toMsg(const hiros::skeletons::types::Keypoint& t_k)
      {
        skeleton_msgs::Keypoint k;
        k.id = t_k.id;
        k.confidence = t_k.confidence;
        k.point = toMsg(t_k.point);
        return k;
      }

      // keypointGroup
      hiros::skeletons::types::KeypointGroup
      toStruct(const unsigned& t_id,
               const double& t_confidence,
               const std::vector<hiros::skeletons::types::Rectangle>& t_boundingBoxes,
               const std::vector<hiros::skeletons::types::Keypoint> t_keypoints)
      {
        hiros::skeletons::types::KeypointGroup kg;
        kg.id = t_id;
        kg.confidence = t_confidence;
        kg.boundingBoxes = t_boundingBoxes;
        kg.keypoints = t_keypoints;
        return kg;
      }

      hiros::skeletons::types::KeypointGroup toStruct(const skeleton_msgs::KeypointGroup& t_kg)
      {
        hiros::skeletons::types::KeypointGroup kg;
        kg.id = static_cast<unsigned>(t_kg.id);
        kg.confidence = t_kg.confidence;
        kg.boundingBoxes.reserve(t_kg.boundingBoxes.size());
        for (auto& bb : t_kg.boundingBoxes) {
          kg.boundingBoxes.push_back(toStruct(bb));
        }
        kg.keypoints.reserve(t_kg.keypoints.size());
        for (auto& k : t_kg.keypoints) {
          kg.keypoints.push_back(toStruct(k));
        }
        return kg;
      }

      skeleton_msgs::KeypointGroup toMsg(const hiros::skeletons::types::KeypointGroup& t_kg)
      {
        skeleton_msgs::KeypointGroup kg;
        kg.id = t_kg.id;
        kg.confidence = t_kg.confidence;
        kg.boundingBoxes.reserve(t_kg.boundingBoxes.size());
        for (auto& bb : t_kg.boundingBoxes) {
          kg.boundingBoxes.push_back(toMsg(bb));
        }
        kg.keypoints.reserve(t_kg.keypoints.size());
        for (auto& k : t_kg.keypoints) {
          kg.keypoints.push_back(toMsg(k));
        }
        return kg;
      }

      // Skeleton
      hiros::skeletons::types::Skeleton
      toStruct(const unsigned& t_id,
               const std::vector<hiros::skeletons::types::KeypointGroup>& t_skeletonParts)
      {
        hiros::skeletons::types::Skeleton s;
        s.id = t_id;
        s.skeletonParts = t_skeletonParts;
        return s;
      }

      hiros::skeletons::types::Skeleton toStruct(const skeleton_msgs::Skeleton& t_s)
      {
        hiros::skeletons::types::Skeleton s;
        s.id = static_cast<unsigned>(t_s.id);
        s.skeletonParts.reserve(t_s.skeletonParts.size());
        for (auto& sp : t_s.skeletonParts) {
          s.skeletonParts.push_back(toStruct(sp));
        }
        return s;
      }

      skeleton_msgs::Skeleton toMsg(const hiros::skeletons::types::Skeleton& t_s)
      {
        skeleton_msgs::Skeleton s;
        s.id = t_s.id;
        s.skeletonParts.reserve(t_s.skeletonParts.size());
        for (auto& sp : t_s.skeletonParts) {
          s.skeletonParts.push_back(toMsg(sp));
        }
        return s;
      }

      // SkeletonGroup
      hiros::skeletons::types::SkeletonGroup
      toStruct(const std::vector<hiros::skeletons::types::Skeleton>& t_skeletons)
      {
        hiros::skeletons::types::SkeletonGroup sg;
        sg.skeletons = t_skeletons;
        return sg;
      }

      hiros::skeletons::types::SkeletonGroup toStruct(const skeleton_msgs::SkeletonGroup& t_sg)
      {
        hiros::skeletons::types::SkeletonGroup sg;
        sg.skeletons.reserve(t_sg.skeletons.size());
        for (auto& s : t_sg.skeletons) {
          sg.skeletons.push_back(toStruct(s));
        }
        return sg;
      }

      skeleton_msgs::SkeletonGroup toMsg(const hiros::skeletons::types::SkeletonGroup& t_sg)
      {
        skeleton_msgs::SkeletonGroup sg;
        sg.skeletons.reserve(t_sg.skeletons.size());
        for (auto& s : t_sg.skeletons) {
          sg.skeletons.push_back(toMsg(s));
        }
        return sg;
      }

      skeleton_msgs::SkeletonGroup toMsg(const unsigned& t_seq,
                                         const ros::Time& t_stamp,
                                         const std::string& t_frame_id,
                                         const ros::Time& t_srcTime,
                                         const hiros::skeletons::types::SkeletonGroup& t_sg)
      {
        skeleton_msgs::SkeletonGroup sg;
        sg.header.seq = t_seq;
        sg.header.stamp = t_stamp;
        sg.header.frame_id = t_frame_id;
        sg.srcTime = t_srcTime;
        sg.skeletons.reserve(t_sg.skeletons.size());
        for (auto& s : t_sg.skeletons) {
          sg.skeletons.push_back(toMsg(s));
        }
        return sg;
      }

    } // namespace utils
  } // namespace skeletons
} // namespace hiros
