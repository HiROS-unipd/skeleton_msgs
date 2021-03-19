#ifndef hiros_skeletons_utils_h
#define hiros_skeletons_utils_h

// Ros Distributed Message dependencies
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Header.h>

// Custom Ros Message dependencies
#include "skeleton_msgs/Box.h"
#include "skeleton_msgs/Marker.h"
#include "skeleton_msgs/MarkerGroup.h"
#include "skeleton_msgs/MarkerSkeleton.h"
#include "skeleton_msgs/MarkerSkeletonGroup.h"
#include "skeleton_msgs/Orientation.h"
#include "skeleton_msgs/OrientationGroup.h"
#include "skeleton_msgs/OrientationSkeleton.h"
#include "skeleton_msgs/OrientationSkeletonGroup.h"
#include "skeleton_msgs/Point.h"

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

      double magnitude(const hiros::skeletons::types::Vector& t_v);
      double distance(const hiros::skeletons::types::Vector& t_v1,
                      const hiros::skeletons::types::Vector& t_v2);

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

      // Marker
      hiros::skeletons::types::Marker toStruct(const int& t_id,
                                               const double& t_confidence,
                                               const hiros::skeletons::types::Point& t_point);

      hiros::skeletons::types::Marker toStruct(
        const int& t_id,
        const double& t_confidence,
        const hiros::skeletons::types::Position& t_position,
        const hiros::skeletons::types::Velocity& t_velocity = hiros::skeletons::types::Velocity(),
        const hiros::skeletons::types::Acceleration& t_acceleration =
          hiros::skeletons::types::Acceleration());

      hiros::skeletons::types::Marker toStruct(const skeleton_msgs::Marker& t_m);

      skeleton_msgs::Marker toMsg(const hiros::skeletons::types::Marker& t_m);

      bool hasMarker(const hiros::skeletons::types::MarkerGroup& t_marker_group,
                     const int& t_marker_id);
      bool hasMarker(const hiros::skeletons::types::MarkerSkeleton& t_marker_skeleton,
                     const int& t_marker_group_id,
                     const int& t_marker_id);

      // MarkerGroup
      hiros::skeletons::types::MarkerGroup
      toStruct(const int& t_id,
               const unsigned int& t_max_markers,
               const double& t_confidence,
               const std::vector<hiros::skeletons::types::Marker> t_markers);

      hiros::skeletons::types::MarkerGroup
      toStruct(const int& t_id,
               const unsigned int& t_max_markers,
               const double& t_confidence,
               const hiros::skeletons::types::Box& t_bounding_box,
               const std::vector<hiros::skeletons::types::Marker> t_markers);

      hiros::skeletons::types::MarkerGroup toStruct(const skeleton_msgs::MarkerGroup& t_mg);

      skeleton_msgs::MarkerGroup toMsg(const hiros::skeletons::types::MarkerGroup& t_mg);

      bool hasMarkerGroup(const hiros::skeletons::types::MarkerSkeleton& t_marker_skeleton,
                          const int& t_marker_group_id);

      // MarkerSkeleton
      hiros::skeletons::types::MarkerSkeleton
      toStruct(const int& t_id,
               const double& t_confidence,
               const std::vector<hiros::skeletons::types::MarkerGroup>& t_marker_groups);

      hiros::skeletons::types::MarkerSkeleton toStruct(const skeleton_msgs::MarkerSkeleton& t_ms);

      skeleton_msgs::MarkerSkeleton toMsg(const hiros::skeletons::types::MarkerSkeleton& t_ms);

      std::shared_ptr<hiros::skeletons::types::MarkerSkeleton>
      getMarkerSkeleton(hiros::skeletons::types::MarkerSkeletonGroup& t_marker_skeleton_group,
                        const int& t_marker_skeleton_id);

      // MarkerSkeletonGroup
      hiros::skeletons::types::MarkerSkeletonGroup
      toStruct(const double& t_src_time,
               const std::string& t_src_frame,
               const std::vector<hiros::skeletons::types::MarkerSkeleton>& t_marker_skeletons);

      hiros::skeletons::types::MarkerSkeletonGroup
      toStruct(const skeleton_msgs::MarkerSkeletonGroup& t_msg);

      skeleton_msgs::MarkerSkeletonGroup
      toMsg(const unsigned int& t_seq,
            const ros::Time& t_stamp,
            const std::string& t_frame_id,
            const ros::Time& t_src_time,
            const std::string& t_src_frame,
            const hiros::skeletons::types::MarkerSkeletonGroup& t_msg);

      skeleton_msgs::MarkerSkeletonGroup
      toMsg(const ros::Time& t_stamp,
            const std::string& t_frame_id,
            const ros::Time& t_src_time,
            const std::string& t_src_frame,
            const hiros::skeletons::types::MarkerSkeletonGroup& t_msg);

      skeleton_msgs::MarkerSkeletonGroup
      toMsg(const unsigned int& t_seq,
            const ros::Time& t_stamp,
            const std::string& t_frame_id,
            const hiros::skeletons::types::MarkerSkeletonGroup& t_msg);

      skeleton_msgs::MarkerSkeletonGroup
      toMsg(const std_msgs::Header& t_header,
            const ros::Time& t_src_time,
            const std::string& t_src_frame,
            const hiros::skeletons::types::MarkerSkeletonGroup& t_msg);

      skeleton_msgs::MarkerSkeletonGroup
      toMsg(const ros::Time& t_stamp,
            const std::string& t_frame_id,
            const hiros::skeletons::types::MarkerSkeletonGroup& t_msg);

      skeleton_msgs::MarkerSkeletonGroup
      toMsg(const std_msgs::Header& t_header,
            const hiros::skeletons::types::MarkerSkeletonGroup& t_msg);

      skeleton_msgs::MarkerSkeletonGroup
      toMsg(const hiros::skeletons::types::MarkerSkeletonGroup& t_msg);

      // Orientation
      hiros::skeletons::types::Orientation
      toStruct(const int& t_id,
               const double& t_confidence,
               const hiros::skeletons::types::Quaternion& t_orientation,
               const hiros::skeletons::types::Vector& t_angular_velocity =
                 hiros::skeletons::types::Vector(),
               const hiros::skeletons::types::Vector& t_linear_acceleration =
                 hiros::skeletons::types::Vector());

      hiros::skeletons::types::Orientation toStruct(const skeleton_msgs::Orientation& t_o);

      skeleton_msgs::Orientation toMsg(const std_msgs::Header& t_header,
                                       const hiros::skeletons::types::Orientation& t_o);

      skeleton_msgs::Orientation toMsg(const hiros::skeletons::types::Orientation& t_o);

      bool hasOrientation(const hiros::skeletons::types::OrientationGroup& t_orientation_group,
                          const int& t_orientation_id);
      bool
      hasOrientation(const hiros::skeletons::types::OrientationSkeleton& t_orientation_skeleton,
                     const int& t_orientation_group_id,
                     const int& t_orientation_id);

      // OrientationGroup
      hiros::skeletons::types::OrientationGroup
      toStruct(const int& t_id,
               const unsigned int& t_max_orientations,
               const double& t_confidence,
               const std::vector<hiros::skeletons::types::Orientation> t_orientations);

      hiros::skeletons::types::OrientationGroup
      toStruct(const skeleton_msgs::OrientationGroup& t_og);

      skeleton_msgs::OrientationGroup toMsg(const hiros::skeletons::types::OrientationGroup& t_og);

      bool hasOrientationGroup(
        const hiros::skeletons::types::OrientationSkeleton& t_orientation_skeleton,
        const int& t_orientation_group_id);

      // OrientationSkeleton
      hiros::skeletons::types::OrientationSkeleton
      toStruct(const int& t_id,
               const double& t_confidence,
               const std::vector<hiros::skeletons::types::OrientationGroup>& t_orientation_groups);

      hiros::skeletons::types::OrientationSkeleton
      toStruct(const skeleton_msgs::OrientationSkeleton& t_os);

      skeleton_msgs::OrientationSkeleton
      toMsg(const hiros::skeletons::types::OrientationSkeleton& t_os);

      std::shared_ptr<hiros::skeletons::types::OrientationSkeleton> getOrientationSkeleton(
        hiros::skeletons::types::OrientationSkeletonGroup& t_orientation_skeleton_group,
        const int& t_orientation_skeleton_id);

      // OrientationSkeletonGroup
      hiros::skeletons::types::OrientationSkeletonGroup toStruct(
        const double& t_src_time,
        const std::string& t_src_frame,
        const std::vector<hiros::skeletons::types::OrientationSkeleton>& t_orientation_skeletons);

      hiros::skeletons::types::OrientationSkeletonGroup
      toStruct(const skeleton_msgs::OrientationSkeletonGroup& t_osg);

      skeleton_msgs::OrientationSkeletonGroup
      toMsg(const unsigned int& t_seq,
            const ros::Time& t_stamp,
            const std::string& t_frame_id,
            const ros::Time& t_src_time,
            const std::string& t_src_frame,
            const hiros::skeletons::types::OrientationSkeletonGroup& t_osg);

      skeleton_msgs::OrientationSkeletonGroup
      toMsg(const ros::Time& t_stamp,
            const std::string& t_frame_id,
            const ros::Time& t_src_time,
            const std::string& t_src_frame,
            const hiros::skeletons::types::OrientationSkeletonGroup& t_osg);

      skeleton_msgs::OrientationSkeletonGroup
      toMsg(const unsigned int& t_seq,
            const ros::Time& t_stamp,
            const std::string& t_frame_id,
            const hiros::skeletons::types::OrientationSkeletonGroup& t_osg);

      skeleton_msgs::OrientationSkeletonGroup
      toMsg(const std_msgs::Header& t_header,
            const ros::Time& t_src_time,
            const std::string& t_src_frame,
            const hiros::skeletons::types::OrientationSkeletonGroup& t_osg);

      skeleton_msgs::OrientationSkeletonGroup
      toMsg(const ros::Time& t_stamp,
            const std::string& t_frame_id,
            const hiros::skeletons::types::OrientationSkeletonGroup& t_osg);

      skeleton_msgs::OrientationSkeletonGroup
      toMsg(const std_msgs::Header& t_header,
            const hiros::skeletons::types::OrientationSkeletonGroup& t_osg);

      skeleton_msgs::OrientationSkeletonGroup
      toMsg(const hiros::skeletons::types::OrientationSkeletonGroup& t_osg);

    } // namespace utils
  } // namespace skeletons
} // namespace hiros
#endif
