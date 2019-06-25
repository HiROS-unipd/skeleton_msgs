// Internal dependencies
#include "skeletons/utils.h"

namespace hiros {
  namespace skeletons {
    namespace utils {

      const std::string PAD = "  ";

      const std::string padding(int t_n_pads)
      {
        std::string ret_str;
        while (--t_n_pads >= 0) {
          ret_str += PAD;
        }
        return ret_str;
      }

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

      // Box
      hiros::skeletons::types::Box toStruct(const double& t_x,
                                            const double& t_y,
                                            const double& t_z,
                                            const double& t_length,
                                            const double& t_height,
                                            const double& t_width)
      {
        hiros::skeletons::types::Box b;
        b.x = t_x;
        b.y = t_y;
        b.z = t_z;
        b.length = t_length;
        b.height = t_height;
        b.width = t_width;
        return b;
      }

      hiros::skeletons::types::Box toStruct(const skeleton_msgs::Box& t_b)
      {
        hiros::skeletons::types::Box b;
        b.x = t_b.x;
        b.y = t_b.y;
        b.z = t_b.z;
        b.length = t_b.length;
        b.height = t_b.height;
        b.width = t_b.width;
        return b;
      }

      skeleton_msgs::Box toMsg(const hiros::skeletons::types::Box& t_b)
      {
        skeleton_msgs::Box b;
        b.x = t_b.x;
        b.y = t_b.y;
        b.z = t_b.z;
        b.length = t_b.length;
        b.height = t_b.height;
        b.width = t_b.width;
        return b;
      }

      // Keypoint
      hiros::skeletons::types::Keypoint toStruct(const unsigned int& t_id,
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
        k.id = static_cast<unsigned int>(t_k.id);
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
      toStruct(const unsigned int& t_id,
               const double& t_confidence,
               const hiros::skeletons::types::Box& t_bounding_box,
               const std::vector<hiros::skeletons::types::Keypoint> t_keypoints)
      {
        hiros::skeletons::types::KeypointGroup kg;
        kg.id = t_id;
        kg.confidence = t_confidence;
        kg.bounding_box = t_bounding_box;
        kg.keypoints = t_keypoints;
        return kg;
      }

      hiros::skeletons::types::KeypointGroup toStruct(const skeleton_msgs::KeypointGroup& t_kg)
      {
        hiros::skeletons::types::KeypointGroup kg;
        kg.id = static_cast<unsigned int>(t_kg.id);
        kg.confidence = t_kg.confidence;
        kg.bounding_box = toStruct(t_kg.bounding_box);
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
        kg.bounding_box = toMsg(t_kg.bounding_box);
        kg.keypoints.reserve(t_kg.keypoints.size());
        for (auto& k : t_kg.keypoints) {
          kg.keypoints.push_back(toMsg(k));
        }
        return kg;
      }

      // Skeleton
      hiros::skeletons::types::Skeleton
      toStruct(const unsigned int& t_id,
               const std::vector<hiros::skeletons::types::KeypointGroup>& t_skeleton_parts)
      {
        hiros::skeletons::types::Skeleton s;
        s.id = t_id;
        s.skeleton_parts = t_skeleton_parts;
        return s;
      }

      hiros::skeletons::types::Skeleton toStruct(const skeleton_msgs::Skeleton& t_s)
      {
        hiros::skeletons::types::Skeleton s;
        s.id = static_cast<unsigned int>(t_s.id);
        s.skeleton_parts.reserve(t_s.skeleton_parts.size());
        for (auto& sp : t_s.skeleton_parts) {
          s.skeleton_parts.push_back(toStruct(sp));
        }
        return s;
      }

      skeleton_msgs::Skeleton toMsg(const hiros::skeletons::types::Skeleton& t_s)
      {
        skeleton_msgs::Skeleton s;
        s.id = t_s.id;
        s.skeleton_parts.reserve(t_s.skeleton_parts.size());
        for (auto& sp : t_s.skeleton_parts) {
          s.skeleton_parts.push_back(toMsg(sp));
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

      skeleton_msgs::SkeletonGroup toMsg(const std_msgs::Header& t_header,
                                         const ros::Time& t_src_time,
                                         const hiros::skeletons::types::SkeletonGroup& t_sg)
      {
        skeleton_msgs::SkeletonGroup sg;
        sg.header = t_header;
        sg.src_time = t_src_time;
        sg.skeletons.reserve(t_sg.skeletons.size());
        for (auto& s : t_sg.skeletons) {
          sg.skeletons.push_back(toMsg(s));
        }
        return sg;
      }

      skeleton_msgs::SkeletonGroup toMsg(const unsigned int& t_seq,
                                         const ros::Time& t_stamp,
                                         const std::string& t_frame_id,
                                         const ros::Time& t_src_time,
                                         const hiros::skeletons::types::SkeletonGroup& t_sg)
      {
        skeleton_msgs::SkeletonGroup sg;
        sg.header.seq = t_seq;
        sg.header.stamp = t_stamp;
        sg.header.frame_id = t_frame_id;
        sg.src_time = t_src_time;
        sg.skeletons.reserve(t_sg.skeletons.size());
        for (auto& s : t_sg.skeletons) {
          sg.skeletons.push_back(toMsg(s));
        }
        return sg;
      }

    } // namespace utils
  } // namespace skeletons
} // namespace hiros
