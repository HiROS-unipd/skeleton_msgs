#ifndef hiros_skeleton_msgs_utils_h
#define hiros_skeleton_msgs_utils_h

// ROS dependencies
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/time.hpp>

// Custom ROS Message dependencies
#include "hiros_skeleton_msgs/msg/box.hpp"
#include "hiros_skeleton_msgs/msg/kinematic_state.hpp"
#include "hiros_skeleton_msgs/msg/link.hpp"
#include "hiros_skeleton_msgs/msg/marker.hpp"
#include "hiros_skeleton_msgs/msg/skeleton.hpp"
#include "hiros_skeleton_msgs/msg/skeleton_group.hpp"

// Internal dependencies
#include "skeletons/types.h"

namespace hiros {
namespace skeletons {
namespace utils {

const std::string padding(int t_n_pads);

// Vector3
hiros::skeletons::types::Vector3 toStruct(
    const double &t_x, const double &t_y,
    const double &t_z = std::numeric_limits<double>::quiet_NaN());

hiros::skeletons::types::Vector3 toStruct(
    const geometry_msgs::msg::Vector3 &t_v);

geometry_msgs::msg::Vector3 toVector3Msg(
    const hiros::skeletons::types::Vector3 &t_v);

bool isNaN(const hiros::skeletons::types::Vector3 &t_v);

double magnitude(const hiros::skeletons::types::Vector3 &t_v);

std::string toString(const hiros::skeletons::types::Vector3 &t_v,
                     int t_pad_lv = 0);

// Point
hiros::skeletons::types::Point toStruct(const geometry_msgs::msg::Point &t_p);

geometry_msgs::msg::Point toPointMsg(const hiros::skeletons::types::Point &t_p);

double distance(const hiros::skeletons::types::Point &t_p1,
                const hiros::skeletons::types::Point &t_p2);

// Quaternion
hiros::skeletons::types::Quaternion toStruct(const double &t_x,
                                             const double &t_y,
                                             const double &t_z,
                                             const double &t_w);

hiros::skeletons::types::Quaternion toStruct(
    const geometry_msgs::msg::Quaternion &t_q);

geometry_msgs::msg::Quaternion toMsg(
    const hiros::skeletons::types::Quaternion &t_q);

bool isNaN(const hiros::skeletons::types::Quaternion &t_q);

double distance(const hiros::skeletons::types::Quaternion &t_q1,
                const hiros::skeletons::types::Quaternion &t_q2);

std::string toString(const hiros::skeletons::types::Quaternion &t_q,
                     int t_pad_lv = 0);

// Pose
hiros::skeletons::types::Pose toStruct(
    const hiros::skeletons::types::Point &t_position,
    const hiros::skeletons::types::Quaternion &t_orientation =
        hiros::skeletons::types::Quaternion(
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN()));
hiros::skeletons::types::Pose toStruct(
    const hiros::skeletons::types::Quaternion &t_orientation);

hiros::skeletons::types::Pose toStruct(const geometry_msgs::msg::Pose &t_p);

geometry_msgs::msg::Pose toMsg(const hiros::skeletons::types::Pose &t_p);

bool isNaN(const hiros::skeletons::types::Pose &t_p);

std::string toString(const hiros::skeletons::types::Pose &t_p,
                     int t_pad_lv = 0);

// Velocity
hiros::skeletons::types::Velocity toStruct(
    const hiros::skeletons::types::Vector3 &t_linear,
    const hiros::skeletons::types::Vector3 &t_angular =
        hiros::skeletons::types::Vector3(
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN()));

hiros::skeletons::types::Velocity toStruct(
    const geometry_msgs::msg::Twist &t_v);

geometry_msgs::msg::Twist toTwistMsg(
    const hiros::skeletons::types::Velocity &t_v);

bool isNaN(const hiros::skeletons::types::Velocity &t_v);

std::string toString(const hiros::skeletons::types::Velocity &t_v,
                     int t_pad_lv = 0);

// Acceleration
hiros::skeletons::types::Acceleration toStruct(
    const geometry_msgs::msg::Accel &t_a);

geometry_msgs::msg::Accel toAccelMsg(
    const hiros::skeletons::types::Acceleration &t_a);

// KinematicState
hiros::skeletons::types::KinematicState toStruct(
    const hiros::skeletons::types::Pose &t_pose,
    const hiros::skeletons::types::Velocity &t_velocity =
        hiros::skeletons::types::Velocity(),
    const hiros::skeletons::types::Acceleration &t_acceleration =
        hiros::skeletons::types::Acceleration());

hiros::skeletons::types::KinematicState toStruct(
    const hiros_skeleton_msgs::msg::KinematicState &t_ks);

hiros_skeleton_msgs::msg::KinematicState toMsg(
    const hiros::skeletons::types::KinematicState &t_ks);

std::string toString(const hiros::skeletons::types::KinematicState &t_ks,
                     int t_pad_lv = 0);

// Box
hiros::skeletons::types::Box toStruct(
    const hiros::skeletons::types::KinematicState &t_center,
    const double &t_height, const double &t_length,
    const double &t_width = std::numeric_limits<double>::quiet_NaN());

hiros::skeletons::types::Box toStruct(const hiros_skeleton_msgs::msg::Box &t_b);

hiros_skeleton_msgs::msg::Box toMsg(const hiros::skeletons::types::Box &t_b);

std::string toString(const hiros::skeletons::types::Box &t_b, int t_pad_lv = 0);

// Marker
hiros::skeletons::types::Marker toStruct(
    const int &t_id, const std::string &t_name, const double &t_confidence,
    const hiros::skeletons::types::KinematicState &t_center);

hiros::skeletons::types::Marker toStruct(
    const hiros_skeleton_msgs::msg::Marker &t_m);

hiros_skeleton_msgs::msg::Marker toMsg(
    const hiros::skeletons::types::Marker &t_m);

std::string toString(const hiros::skeletons::types::Marker &t_m,
                     int t_pad_lv = 0);

// Link
hiros::skeletons::types::Link toStruct(
    const int &t_id, const std::string &t_name, const int &t_parent_marker,
    const int &t_child_marker, const double &t_confidence,
    const hiros::skeletons::types::KinematicState &t_center);

hiros::skeletons::types::Link toStruct(
    const hiros_skeleton_msgs::msg::Link &t_l);

hiros_skeleton_msgs::msg::Link toMsg(const hiros::skeletons::types::Link &t_l);

double linkLength(const hiros::skeletons::types::Skeleton &t_skeleton,
                  const int &t_link_id);

std::string toString(const hiros::skeletons::types::Link &t_l,
                     int t_pad_lv = 0);

// Skeleton
hiros::skeletons::types::Skeleton toStruct(
    const int &t_id, const double &t_src_time, const std::string &t_src_frame,
    const unsigned int &t_max_markers = 0, const unsigned int &t_max_links = 0,
    const double &t_confidence = std::numeric_limits<double>::quiet_NaN(),
    const hiros::skeletons::types::Box &t_bounding_box =
        hiros::skeletons::types::Box(),
    const std::vector<hiros::skeletons::types::Marker> &t_markers =
        std::vector<hiros::skeletons::types::Marker>(),
    const std::vector<hiros::skeletons::types::Link> &t_links =
        std::vector<hiros::skeletons::types::Link>());

hiros::skeletons::types::Skeleton toStruct(
    const hiros_skeleton_msgs::msg::Skeleton &t_s);

hiros_skeleton_msgs::msg::Skeleton toMsg(
    const hiros::skeletons::types::Skeleton &t_s);

hiros::skeletons::types::Box computeBoundingBox(
    const hiros::skeletons::types::Skeleton &t_s);

hiros::skeletons::types::KinematicState centroid(
    const hiros::skeletons::types::Skeleton &t_s);

std::string toString(const hiros::skeletons::types::Skeleton &t_s,
                     int t_pad_lv = 0);

// SkeletonGroup
hiros::skeletons::types::SkeletonGroup toStruct(
    const double &t_time, const std::string &t_frame,
    const std::vector<hiros::skeletons::types::Skeleton> &t_skeletons);

hiros::skeletons::types::SkeletonGroup toStruct(
    const hiros_skeleton_msgs::msg::SkeletonGroup &t_sg);

hiros_skeleton_msgs::msg::SkeletonGroup toMsg(
    const hiros::skeletons::types::SkeletonGroup &t_sg);

std::string toString(const hiros::skeletons::types::SkeletonGroup &t_sg,
                     int t_pad_lv = 0);

}  // namespace utils
}  // namespace skeletons
}  // namespace hiros

#endif
