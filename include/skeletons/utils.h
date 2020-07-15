#ifndef hiros_skeletons_utils_h
#define hiros_skeletons_utils_h

// Ros Distributed Message dependencies
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Header.h>

// Custom Ros Message dependencies
#include "skeleton_msgs/Box.h"
#include "skeleton_msgs/Keypoint.h"
#include "skeleton_msgs/KeypointGroup.h"
#include "skeleton_msgs/Point.h"
#include "skeleton_msgs/Skeleton.h"
#include "skeleton_msgs/SkeletonGroup.h"

// Internal dependencies
#include "skeletons/types.h"

namespace hiros {
  namespace skeletons {
    namespace utils {

      const std::string padding(int t_n_pads);

      // Vector
      hiros::skeletons::types::Vector
      toStruct(const double& t_x,
               const double& t_y,
               const double& t_z = std::numeric_limits<double>::quiet_NaN());

      hiros::skeletons::types::Vector toStruct(const geometry_msgs::Point& t_p);
      hiros::skeletons::types::Vector toStruct(const geometry_msgs::Vector3& t_v);

      geometry_msgs::Point toPointMsg(const hiros::skeletons::types::Vector& t_v);
      geometry_msgs::Vector3 toVector3Msg(const hiros::skeletons::types::Vector& t_v);

      // Point
      hiros::skeletons::types::Point toStruct(
        const hiros::skeletons::types::Position& t_p,
        const hiros::skeletons::types::Velocity& t_v = hiros::skeletons::types::Velocity(),
        const hiros::skeletons::types::Acceleration& t_a = hiros::skeletons::types::Acceleration());

      hiros::skeletons::types::Point toStruct(const skeleton_msgs::Point& t_p);

      skeleton_msgs::Point toMsg(const hiros::skeletons::types::Point& t_p);

      // Quaternion
      hiros::skeletons::types::Quaternion
      toStruct(const double& t_x, const double& t_y, const double& t_z, const double& t_w);

      hiros::skeletons::types::Quaternion toStruct(const geometry_msgs::Quaternion& t_q);

      geometry_msgs::Quaternion toMsg(const hiros::skeletons::types::Quaternion& t_q);

      // Box
      hiros::skeletons::types::Box
      toStruct(const hiros::skeletons::types::Point& t_center,
               const double& t_length,
               const double& t_height,
               const double& t_width = std::numeric_limits<double>::quiet_NaN(),
               const hiros::skeletons::types::Quaternion& t_orientation =
                 hiros::skeletons::types::Quaternion());

      hiros::skeletons::types::Box toStruct(const skeleton_msgs::Box& t_b);

      skeleton_msgs::Box toMsg(const hiros::skeletons::types::Box& t_b);

      // Keypoint
      hiros::skeletons::types::Keypoint toStruct(const int& t_id,
                                                 const double& t_confidence,
                                                 const hiros::skeletons::types::Point& t_point);

      hiros::skeletons::types::Keypoint toStruct(
        const int& t_id,
        const double& t_confidence,
        const hiros::skeletons::types::Position& t_position,
        const hiros::skeletons::types::Velocity& t_velocity = hiros::skeletons::types::Velocity(),
        const hiros::skeletons::types::Acceleration& t_acceleration =
          hiros::skeletons::types::Acceleration());

      hiros::skeletons::types::Keypoint toStruct(const skeleton_msgs::Keypoint& t_k);

      skeleton_msgs::Keypoint toMsg(const hiros::skeletons::types::Keypoint& t_k);

      // keypointGroup
      hiros::skeletons::types::KeypointGroup
      toStruct(const int& t_id,
               const unsigned int& t_max_keypoints,
               const double& t_confidence,
               const std::vector<hiros::skeletons::types::Keypoint> t_keypoints);

      hiros::skeletons::types::KeypointGroup
      toStruct(const int& t_id,
               const unsigned int& t_max_keypoints,
               const double& t_confidence,
               const hiros::skeletons::types::Box& t_bounding_box,
               const std::vector<hiros::skeletons::types::Keypoint> t_keypoints);

      hiros::skeletons::types::KeypointGroup toStruct(const skeleton_msgs::KeypointGroup& t_kg);

      skeleton_msgs::KeypointGroup toMsg(const hiros::skeletons::types::KeypointGroup& t_kg);

      // Skeleton
      hiros::skeletons::types::Skeleton
      toStruct(const int& t_id,
               const std::vector<hiros::skeletons::types::KeypointGroup>& t_skeleton_parts);

      hiros::skeletons::types::Skeleton toStruct(const skeleton_msgs::Skeleton& t_s);

      skeleton_msgs::Skeleton toMsg(const hiros::skeletons::types::Skeleton& t_s);

      // SkeletonGroup
      hiros::skeletons::types::SkeletonGroup
      toStruct(const double& t_src_time,
               const std::string& t_src_frame,
               const std::vector<hiros::skeletons::types::Skeleton>& t_skeletons);

      hiros::skeletons::types::SkeletonGroup toStruct(const skeleton_msgs::SkeletonGroup& t_sg);

      skeleton_msgs::SkeletonGroup toMsg(const hiros::skeletons::types::SkeletonGroup& t_sg);

      skeleton_msgs::SkeletonGroup toMsg(const std_msgs::Header& t_header,
                                         const ros::Time& t_src_time,
                                         const std::string& t_src_frame,
                                         const hiros::skeletons::types::SkeletonGroup& t_sg);

      skeleton_msgs::SkeletonGroup toMsg(const std_msgs::Header& t_header,
                                         const ros::Time& t_src_time,
                                         const hiros::skeletons::types::SkeletonGroup& t_sg);

      skeleton_msgs::SkeletonGroup toMsg(const ros::Time& t_stamp,
                                         const std::string& t_frame_id,
                                         const ros::Time& t_src_time,
                                         const std::string& t_src_frame,
                                         const hiros::skeletons::types::SkeletonGroup& t_sg);

      skeleton_msgs::SkeletonGroup toMsg(const ros::Time& t_stamp,
                                         const std::string& t_frame_id,
                                         const ros::Time& t_src_time,
                                         const hiros::skeletons::types::SkeletonGroup& t_sg);

      skeleton_msgs::SkeletonGroup toMsg(const unsigned int& t_seq,
                                         const ros::Time& t_stamp,
                                         const std::string& t_frame_id,
                                         const ros::Time& t_src_time,
                                         const std::string& t_src_frame,
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
