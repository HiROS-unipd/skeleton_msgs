#ifndef hiros_skeletons_utils_h
#define hiros_skeletons_utils_h

// Ros Distributed Message dependencies
#include <geometry_msgs/Point.h>
#include <std_msgs/Header.h>

// Custom Ros Message dependencies
#include "skeleton_msgs/Box.h"
#include "skeleton_msgs/Keypoint.h"
#include "skeleton_msgs/KeypointGroup.h"
#include "skeleton_msgs/Skeleton.h"
#include "skeleton_msgs/SkeletonGroup.h"

// Internal dependencies
#include "skeletons/types.h"

namespace hiros {
  namespace skeletons {
    namespace utils {

      const std::string padding(int t_n_pads);

      // Point
      hiros::skeletons::types::Point
      toStruct(const double& t_x,
               const double& t_y,
               const double& t_z = std::numeric_limits<double>::quiet_NaN());

      hiros::skeletons::types::Point toStruct(const geometry_msgs::Point& t_p);

      geometry_msgs::Point toMsg(const hiros::skeletons::types::Point& t_p);

      // Box
      hiros::skeletons::types::Box toStruct(const double& t_x,
                                            const double& t_y,
                                            const double& t_z,
                                            const double& t_length,
                                            const double& t_height,
                                            const double& t_width);

      hiros::skeletons::types::Box toStruct(const skeleton_msgs::Box& t_b);

      skeleton_msgs::Box toMsg(const hiros::skeletons::types::Box& t_b);

      // Keypoint
      hiros::skeletons::types::Keypoint toStruct(const unsigned int& t_id,
                                                 const double& t_confidence,
                                                 const hiros::skeletons::types::Point& t_point);

      hiros::skeletons::types::Keypoint toStruct(const skeleton_msgs::Keypoint& t_k);

      skeleton_msgs::Keypoint toMsg(const hiros::skeletons::types::Keypoint& t_k);

      // keypointGroup
      hiros::skeletons::types::KeypointGroup
      toStruct(const unsigned int& t_id,
               const double& t_confidence,
               const hiros::skeletons::types::Box& t_bounding_box,
               const std::vector<hiros::skeletons::types::Keypoint> t_keypoints);

      hiros::skeletons::types::KeypointGroup toStruct(const skeleton_msgs::KeypointGroup& t_kg);

      skeleton_msgs::KeypointGroup toMsg(const hiros::skeletons::types::KeypointGroup& t_kg);

      // Skeleton
      hiros::skeletons::types::Skeleton
      toStruct(const unsigned int& t_id,
               const std::vector<hiros::skeletons::types::KeypointGroup>& t_skeleton_parts);

      hiros::skeletons::types::Skeleton toStruct(const skeleton_msgs::Skeleton& t_s);

      skeleton_msgs::Skeleton toMsg(const hiros::skeletons::types::Skeleton& t_s);

      // SkeletonGroup
      hiros::skeletons::types::SkeletonGroup
      toStruct(const std::vector<hiros::skeletons::types::Skeleton>& t_skeletons);

      hiros::skeletons::types::SkeletonGroup toStruct(const skeleton_msgs::SkeletonGroup& t_sg);

      skeleton_msgs::SkeletonGroup toMsg(const hiros::skeletons::types::SkeletonGroup& t_sg);

      skeleton_msgs::SkeletonGroup toMsg(const std_msgs::Header& t_header,
                                         const ros::Time& t_src_time,
                                         const hiros::skeletons::types::SkeletonGroup& t_sg);

      skeleton_msgs::SkeletonGroup toMsg(const ros::Time& t_stamp,
                                         const std::string& t_frame_id,
                                         const ros::Time& t_src_time,
                                         const hiros::skeletons::types::SkeletonGroup& t_sg);

      skeleton_msgs::SkeletonGroup toMsg(const unsigned int& t_seq,
                                         const ros::Time& t_stamp,
                                         const std::string& t_frame_id,
                                         const ros::Time& t_src_time,
                                         const hiros::skeletons::types::SkeletonGroup& t_sg);

    } // namespace utils
  } // namespace skeletons
} // namespace hiros
#endif
