#ifndef hiros_skeletons_utils_h
#define hiros_skeletons_utils_h

// Ros Distributed Message dependencies
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Header.h>

// Custom Ros Message dependencies
#include "hiros_skeleton_msgs/Box.h"
#include "hiros_skeleton_msgs/MIMU.h"
#include "hiros_skeleton_msgs/Marker.h"
#include "hiros_skeleton_msgs/MarkerGroup.h"
#include "hiros_skeleton_msgs/Orientation.h"
#include "hiros_skeleton_msgs/OrientationGroup.h"
#include "hiros_skeleton_msgs/Point.h"
#include "hiros_skeleton_msgs/Skeleton.h"
#include "hiros_skeleton_msgs/SkeletonGroup.h"

// Internal dependencies
#include "skeletons/types.h"

namespace hiros {
  namespace skeletons {
    namespace utils {

      const std::string padding(int t_n_pads);

      // Vector
      tf2::Vector3 toStruct(const double& t_x,
                            const double& t_y,
                            const double& t_z = std::numeric_limits<double>::quiet_NaN());

      tf2::Vector3 toStruct(const geometry_msgs::Point& t_p);
      tf2::Vector3 toStruct(const geometry_msgs::Vector3& t_v);

      geometry_msgs::Point toPointMsg(const tf2::Vector3& t_v);
      geometry_msgs::Vector3 toVector3Msg(const tf2::Vector3& t_v);

      double magnitude(const tf2::Vector3& t_v);
      double distance(const types::Position& t_p1, const types::Position& t_p2);

      std::string toString(const tf2::Vector3& t_v, int t_pad_lv = 0);

      // Point
      hiros::skeletons::types::Point
      toStruct(const hiros::skeletons::types::Position& t_p,
               const hiros::skeletons::types::Velocity& t_v =
                 hiros::skeletons::types::Velocity(std::numeric_limits<double>::quiet_NaN(),
                                                   std::numeric_limits<double>::quiet_NaN(),
                                                   std::numeric_limits<double>::quiet_NaN()),
               const hiros::skeletons::types::Acceleration& t_a =
                 hiros::skeletons::types::Acceleration(std::numeric_limits<double>::quiet_NaN(),
                                                       std::numeric_limits<double>::quiet_NaN(),
                                                       std::numeric_limits<double>::quiet_NaN()));

      hiros::skeletons::types::Point toStruct(const hiros_skeleton_msgs::Point& t_p);

      hiros_skeleton_msgs::Point toMsg(const hiros::skeletons::types::Point& t_p);

      std::string toString(const hiros::skeletons::types::Point& t_p, int t_pad_lv = 0);

      // Quaternion
      tf2::Quaternion
      toStruct(const double& t_x, const double& t_y, const double& t_z, const double& t_w);

      tf2::Quaternion toStruct(const geometry_msgs::Quaternion& t_q);

      geometry_msgs::Quaternion toMsg(const tf2::Quaternion& t_q);

      double distance(const tf2::Quaternion& t_q1, const tf2::Quaternion& t_q2);

      std::string toString(const tf2::Quaternion& t_q, int t_pad_lv = 0);

      // Box
      hiros::skeletons::types::Box
      toStruct(const hiros::skeletons::types::Point& t_center,
               const double& t_length,
               const double& t_height,
               const double& t_width = std::numeric_limits<double>::quiet_NaN(),
               const tf2::Quaternion& t_orientation =
                 tf2::Quaternion(std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::quiet_NaN()));

      hiros::skeletons::types::Box toStruct(const hiros_skeleton_msgs::Box& t_b);

      hiros_skeleton_msgs::Box toMsg(const hiros::skeletons::types::Box& t_b);

      std::string toString(const hiros::skeletons::types::Box& t_b, int t_pad_lv = 0);

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

      hiros::skeletons::types::Marker toStruct(const hiros_skeleton_msgs::Marker& t_m);

      hiros_skeleton_msgs::Marker toMsg(const hiros::skeletons::types::Marker& t_m);

      std::string toString(const hiros::skeletons::types::Marker& t_m, int t_pad_lv = 0);

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

      hiros::skeletons::types::MarkerGroup toStruct(const hiros_skeleton_msgs::MarkerGroup& t_mg);

      hiros_skeleton_msgs::MarkerGroup toMsg(const hiros::skeletons::types::MarkerGroup& t_mg);

      std::string toString(const hiros::skeletons::types::MarkerGroup& t_mg, int t_pad_lv = 0);

      // MIMU
      hiros::skeletons::types::MIMU
      toStruct(const std::string& t_frame_id = "",
               const tf2::Quaternion& t_orientation =
                 tf2::Quaternion(std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::quiet_NaN()),
               const tf2::Vector3& t_angular_velocity =
                 tf2::Vector3(std::numeric_limits<double>::quiet_NaN(),
                              std::numeric_limits<double>::quiet_NaN(),
                              std::numeric_limits<double>::quiet_NaN()),
               const tf2::Vector3& t_linear_acceleration =
                 tf2::Vector3(std::numeric_limits<double>::quiet_NaN(),
                              std::numeric_limits<double>::quiet_NaN(),
                              std::numeric_limits<double>::quiet_NaN()),
               const tf2::Vector3& t_magnetic_field =
                 tf2::Vector3(std::numeric_limits<double>::quiet_NaN(),
                              std::numeric_limits<double>::quiet_NaN(),
                              std::numeric_limits<double>::quiet_NaN()));

      hiros::skeletons::types::MIMU toStruct(const hiros_skeleton_msgs::MIMU& t_m);

      hiros_skeleton_msgs::MIMU toMsg(const std_msgs::Header& t_header,
                                      const hiros::skeletons::types::MIMU& t_m);

      hiros_skeleton_msgs::MIMU toMsg(const hiros::skeletons::types::MIMU& t_m);

      std::string toString(const hiros::skeletons::types::MIMU& t_m, int t_pad_lv = 0);

      // Orientation
      hiros::skeletons::types::Orientation
      toStruct(const int& t_id,
               const double& t_confidence,
               const hiros::skeletons::types::MIMU& t_mimu = hiros::skeletons::types::MIMU());

      hiros::skeletons::types::Orientation toStruct(const hiros_skeleton_msgs::Orientation& t_o);

      hiros_skeleton_msgs::Orientation toMsg(const std_msgs::Header& t_header,
                                             const hiros::skeletons::types::Orientation& t_o);

      hiros_skeleton_msgs::Orientation toMsg(const hiros::skeletons::types::Orientation& t_o);

      std::string toString(const hiros::skeletons::types::Orientation& t_o, int t_pad_lv = 0);

      // OrientationGroup
      hiros::skeletons::types::OrientationGroup
      toStruct(const int& t_id,
               const unsigned int& t_max_orientations,
               const double& t_confidence,
               const std::vector<hiros::skeletons::types::Orientation> t_orientations);

      hiros::skeletons::types::OrientationGroup
      toStruct(const hiros_skeleton_msgs::OrientationGroup& t_og);

      hiros_skeleton_msgs::OrientationGroup
      toMsg(const std_msgs::Header& t_header,
            const hiros::skeletons::types::OrientationGroup& t_og);

      hiros_skeleton_msgs::OrientationGroup
      toMsg(const hiros::skeletons::types::OrientationGroup& t_og);

      std::string toString(const hiros::skeletons::types::OrientationGroup& t_og, int t_pad_lv = 0);

      // Skeleton
      hiros::skeletons::types::Skeleton
      toStruct(const int& t_id,
               const double& t_src_time,
               const std::string& t_src_frame,
               const double& t_confidence,
               const std::vector<hiros::skeletons::types::MarkerGroup>& t_marker_groups,
               const std::vector<hiros::skeletons::types::OrientationGroup>& t_orientation_groups);

      hiros::skeletons::types::Skeleton toStruct(const hiros_skeleton_msgs::Skeleton& t_s);

      hiros_skeleton_msgs::Skeleton toMsg(const std_msgs::Header& t_header,
                                          const hiros::skeletons::types::Skeleton& t_s);

      hiros_skeleton_msgs::Skeleton toMsg(const hiros::skeletons::types::Skeleton& t_s);

      unsigned int numberOfMarkers(const hiros::skeletons::types::Skeleton& t_s);
      unsigned int numberOfOrientations(const hiros::skeletons::types::Skeleton& t_s);

      std::string toString(const hiros::skeletons::types::Skeleton& t_s, int t_pad_lv = 0);

      // SkeletonGroup
      hiros::skeletons::types::SkeletonGroup
      toStruct(const double& t_time,
               const std::string& t_frame,
               const std::vector<hiros::skeletons::types::Skeleton>& t_skeletons);

      hiros::skeletons::types::SkeletonGroup
      toStruct(const hiros_skeleton_msgs::SkeletonGroup& t_sg);

      hiros_skeleton_msgs::SkeletonGroup toMsg(const hiros::skeletons::types::SkeletonGroup& t_sg);

      std::string toString(const hiros::skeletons::types::SkeletonGroup& t_sg, int t_pad_lv = 0);

    } // namespace utils
  } // namespace skeletons
} // namespace hiros
#endif
