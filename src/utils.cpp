// Standard dependencies
#include <memory>

// Internal dependencies
#include "skeletons/utils.h"

const std::string PAD = "  ";

const std::string hiros::skeletons::utils::padding(int t_n_pads)
{
  std::string ret_str;
  while (--t_n_pads >= 0) {
    ret_str += PAD;
  }
  return ret_str;
}

// Vector
hiros::skeletons::types::Vector
hiros::skeletons::utils::toStruct(const double& t_x, const double& t_y, const double& t_z)
{
  return hiros::skeletons::types::Vector(t_x, t_y, t_z);
}

hiros::skeletons::types::Vector hiros::skeletons::utils::toStruct(const geometry_msgs::Point& t_p)
{
  return hiros::skeletons::types::Vector(t_p.x, t_p.y, t_p.z);
}

hiros::skeletons::types::Vector hiros::skeletons::utils::toStruct(const geometry_msgs::Vector3& t_v)
{
  return hiros::skeletons::types::Vector(t_v.x, t_v.y, t_v.z);
}

geometry_msgs::Point hiros::skeletons::utils::toPointMsg(const hiros::skeletons::types::Vector& t_v)
{
  geometry_msgs::Point p;
  p.x = t_v.x;
  p.y = t_v.y;
  p.z = t_v.z;
  return p;
}

geometry_msgs::Vector3
hiros::skeletons::utils::toVector3Msg(const hiros::skeletons::types::Vector& t_v)
{
  geometry_msgs::Vector3 v;
  v.x = t_v.x;
  v.y = t_v.y;
  v.z = t_v.z;
  return v;
}

double hiros::skeletons::utils::magnitude(const hiros::skeletons::types::Vector& t_v)
{
  return distance(t_v, hiros::skeletons::types::Vector(0, 0, 0));
}

double hiros::skeletons::utils::distance(const hiros::skeletons::types::Vector& t_v1,
                                         const hiros::skeletons::types::Vector& t_v2)
{
  double squared_dist = std::pow((t_v1.x - t_v2.x), 2) + std::pow((t_v1.y - t_v2.y), 2);

  if (!std::isnan(t_v1.z) && !std::isnan(t_v2.z)) {
    squared_dist += std::pow((t_v1.z - t_v2.z), 2);
  }

  return std::sqrt(squared_dist);
}

// Point
hiros::skeletons::types::Point
hiros::skeletons::utils::toStruct(const hiros::skeletons::types::Position& t_p,
                                  const hiros::skeletons::types::Velocity& t_v,
                                  const hiros::skeletons::types::Acceleration& t_a)
{
  return hiros::skeletons::types::Point(t_p, t_v, t_a);
}

hiros::skeletons::types::Point hiros::skeletons::utils::toStruct(const skeleton_msgs::Point& t_p)
{
  return hiros::skeletons::types::Point(
    toStruct(t_p.position), toStruct(t_p.velocity), toStruct(t_p.acceleration));
}

skeleton_msgs::Point hiros::skeletons::utils::toMsg(const hiros::skeletons::types::Point& t_p)
{
  skeleton_msgs::Point p;
  p.position = toPointMsg(t_p.position);
  p.velocity = toVector3Msg(t_p.velocity);
  p.acceleration = toVector3Msg(t_p.acceleration);
  return p;
}

// Quaternion
hiros::skeletons::types::Quaternion hiros::skeletons::utils::toStruct(const double& t_x,
                                                                      const double& t_y,
                                                                      const double& t_z,
                                                                      const double& t_w)
{
  return hiros::skeletons::types::Quaternion(t_x, t_y, t_z, t_w);
}

hiros::skeletons::types::Quaternion
hiros::skeletons::utils::toStruct(const geometry_msgs::Quaternion& t_q)
{
  return hiros::skeletons::types::Quaternion(t_q.x, t_q.y, t_q.z, t_q.w);
}

geometry_msgs::Quaternion
hiros::skeletons::utils::toMsg(const hiros::skeletons::types::Quaternion& t_q)
{
  geometry_msgs::Quaternion q;
  q.x = t_q.x;
  q.y = t_q.y;
  q.z = t_q.z;
  q.w = t_q.w;
  return q;
}

// Box
hiros::skeletons::types::Box
hiros::skeletons::utils::toStruct(const hiros::skeletons::types::Point& t_center,
                                  const double& t_length,
                                  const double& t_height,
                                  const double& t_width,
                                  const hiros::skeletons::types::Quaternion& t_orientation)
{
  return hiros::skeletons::types::Box(t_center, t_length, t_height, t_width, t_orientation);
}

hiros::skeletons::types::Box hiros::skeletons::utils::toStruct(const skeleton_msgs::Box& t_b)
{
  return hiros::skeletons::types::Box(
    toStruct(t_b.center), t_b.length, t_b.height, t_b.width, toStruct(t_b.orientation));
}

skeleton_msgs::Box hiros::skeletons::utils::toMsg(const hiros::skeletons::types::Box& t_b)
{
  skeleton_msgs::Box b;
  b.center = toMsg(t_b.center);
  b.length = t_b.length;
  b.height = t_b.height;
  b.width = t_b.width;
  b.orientation = toMsg(t_b.orientation);
  return b;
}

// Marker
hiros::skeletons::types::Marker
hiros::skeletons::utils::toStruct(const int& t_id,
                                  const double& t_confidence,
                                  const hiros::skeletons::types::Point& t_point)
{
  return hiros::skeletons::types::Marker(t_id, t_confidence, t_point);
}

hiros::skeletons::types::Marker
hiros::skeletons::utils::toStruct(const int& t_id,
                                  const double& t_confidence,
                                  const hiros::skeletons::types::Position& t_position,
                                  const hiros::skeletons::types::Velocity& t_velocity,
                                  const hiros::skeletons::types::Acceleration& t_acceleration)
{
  return hiros::skeletons::types::Marker(
    t_id, t_confidence, hiros::skeletons::types::Point(t_position, t_velocity, t_acceleration));
}

hiros::skeletons::types::Marker hiros::skeletons::utils::toStruct(const skeleton_msgs::Marker& t_m)
{
  return hiros::skeletons::types::Marker(t_m.id, t_m.confidence, toStruct(t_m.point));
}

skeleton_msgs::Marker hiros::skeletons::utils::toMsg(const hiros::skeletons::types::Marker& t_m)
{
  skeleton_msgs::Marker m;
  m.id = t_m.id;
  m.confidence = t_m.confidence;
  m.point = toMsg(t_m.point);
  return m;
}

bool hiros::skeletons::utils::hasMarker(const hiros::skeletons::types::MarkerGroup& t_marker_group,
                                        const int& t_marker_id)
{
  return (t_marker_group.markers.count(t_marker_id) > 0);
}

bool hiros::skeletons::utils::hasMarker(
  const hiros::skeletons::types::MarkerSkeleton& t_marker_skeleton,
  const int& t_marker_group_id,
  const int& t_marker_id)
{
  return (hasMarkerGroup(t_marker_skeleton, t_marker_group_id)
          && hasMarker(t_marker_skeleton.marker_groups.at(t_marker_group_id), t_marker_id));
}

// MarkerGroup
hiros::skeletons::types::MarkerGroup
hiros::skeletons::utils::toStruct(const int& t_id,
                                  const unsigned int& t_max_markers,
                                  const double& t_confidence,
                                  const std::vector<hiros::skeletons::types::Marker> t_markers)
{
  return hiros::skeletons::types::MarkerGroup(
    t_id, t_max_markers, t_confidence, hiros::skeletons::types::Box(), t_markers);
}

hiros::skeletons::types::MarkerGroup
hiros::skeletons::utils::toStruct(const int& t_id,
                                  const unsigned int& t_max_markers,
                                  const double& t_confidence,
                                  const hiros::skeletons::types::Box& t_bounding_box,
                                  const std::vector<hiros::skeletons::types::Marker> t_markers)
{
  return hiros::skeletons::types::MarkerGroup(
    t_id, t_max_markers, t_confidence, t_bounding_box, t_markers);
}

hiros::skeletons::types::MarkerGroup
hiros::skeletons::utils::toStruct(const skeleton_msgs::MarkerGroup& t_mg)
{
  hiros::skeletons::types::MarkerGroup mg(
    t_mg.id, t_mg.max_markers, t_mg.confidence, toStruct(t_mg.bounding_box));
  for (auto& m : t_mg.markers) {
    mg.markers.emplace(m.id, toStruct(m));
  }
  return mg;
}

skeleton_msgs::MarkerGroup
hiros::skeletons::utils::toMsg(const hiros::skeletons::types::MarkerGroup& t_mg)
{
  skeleton_msgs::MarkerGroup mg;
  mg.id = t_mg.id;
  mg.max_markers = t_mg.max_markers;
  mg.confidence = t_mg.confidence;
  mg.bounding_box = toMsg(t_mg.bounding_box);
  mg.markers.reserve(t_mg.markers.size());
  for (auto& m : t_mg.markers) {
    mg.markers.push_back(toMsg(m.second));
  }
  return mg;
}

bool hiros::skeletons::utils::hasMarkerGroup(
  const hiros::skeletons::types::MarkerSkeleton& t_marker_skeleton,
  const int& t_marker_group_id)
{
  return (t_marker_skeleton.marker_groups.count(t_marker_group_id) > 0);
}

// MarkerSkeleton
hiros::skeletons::types::MarkerSkeleton hiros::skeletons::utils::toStruct(
  const int& t_id,
  const double& t_confidence,
  const std::vector<hiros::skeletons::types::MarkerGroup>& t_marker_groups)
{
  return hiros::skeletons::types::MarkerSkeleton(t_id, t_confidence, t_marker_groups);
}

hiros::skeletons::types::MarkerSkeleton
hiros::skeletons::utils::toStruct(const skeleton_msgs::MarkerSkeleton& t_ms)
{
  hiros::skeletons::types::MarkerSkeleton ms(t_ms.id, t_ms.confidence);
  for (auto& mg : t_ms.marker_groups) {
    ms.marker_groups.emplace(mg.id, toStruct(mg));
  }
  return ms;
}

skeleton_msgs::MarkerSkeleton
hiros::skeletons::utils::toMsg(const hiros::skeletons::types::MarkerSkeleton& t_ms)
{
  skeleton_msgs::MarkerSkeleton ms;
  ms.id = t_ms.id;
  ms.confidence = t_ms.confidence;
  ms.marker_groups.reserve(t_ms.marker_groups.size());
  for (auto& mg : t_ms.marker_groups) {
    ms.marker_groups.push_back(toMsg(mg.second));
  }
  return ms;
}

std::shared_ptr<hiros::skeletons::types::MarkerSkeleton> hiros::skeletons::utils::getMarkerSkeleton(
  hiros::skeletons::types::MarkerSkeletonGroup& t_marker_skeleton_group,
  const int& t_marker_skeleton_id)
{
  auto marker_skeleton_it =
    std::find_if(t_marker_skeleton_group.marker_skeletons.begin(),
                 t_marker_skeleton_group.marker_skeletons.end(),
                 [t_marker_skeleton_id](const hiros::skeletons::types::MarkerSkeleton& ms) {
                   return ms.id == t_marker_skeleton_id;
                 });

  return marker_skeleton_it != t_marker_skeleton_group.marker_skeletons.end()
           ? std::shared_ptr<hiros::skeletons::types::MarkerSkeleton>(&*marker_skeleton_it)
           : nullptr;
}

// MarkerSkeletonGroup
hiros::skeletons::types::MarkerSkeletonGroup hiros::skeletons::utils::toStruct(
  const double& t_src_time,
  const std::string& t_src_frame,
  const std::vector<hiros::skeletons::types::MarkerSkeleton>& t_marker_skeletons)
{
  return hiros::skeletons::types::MarkerSkeletonGroup(t_src_time, t_src_frame, t_marker_skeletons);
}

hiros::skeletons::types::MarkerSkeletonGroup
hiros::skeletons::utils::toStruct(const skeleton_msgs::MarkerSkeletonGroup& t_msg)
{
  hiros::skeletons::types::MarkerSkeletonGroup msg;
  msg.src_time = t_msg.src_time.toSec();
  msg.src_frame = t_msg.src_frame;
  msg.marker_skeletons.reserve(t_msg.marker_skeletons.size());
  for (auto& ms : t_msg.marker_skeletons) {
    msg.marker_skeletons.push_back(toStruct(ms));
  }
  return msg;
}

skeleton_msgs::MarkerSkeletonGroup
hiros::skeletons::utils::toMsg(const unsigned int& t_seq,
                               const ros::Time& t_stamp,
                               const std::string& t_frame_id,
                               const ros::Time& t_src_time,
                               const std::string& t_src_frame,
                               const hiros::skeletons::types::MarkerSkeletonGroup& t_msg)
{
  skeleton_msgs::MarkerSkeletonGroup msg;
  msg.header.seq = t_seq;
  msg.header.stamp = t_stamp;
  msg.header.frame_id = t_frame_id;
  msg.src_time = t_src_time;
  msg.src_frame = t_src_frame;
  msg.marker_skeletons.reserve(t_msg.marker_skeletons.size());
  for (auto& ms : t_msg.marker_skeletons) {
    msg.marker_skeletons.push_back(toMsg(ms));
  }
  return msg;
}

skeleton_msgs::MarkerSkeletonGroup
hiros::skeletons::utils::toMsg(const ros::Time& t_stamp,
                               const std::string& t_frame_id,
                               const ros::Time& t_src_time,
                               const std::string& t_src_frame,
                               const hiros::skeletons::types::MarkerSkeletonGroup& t_msg)
{
  return toMsg(0, t_stamp, t_frame_id, t_src_time, t_src_frame, t_msg);
}

skeleton_msgs::MarkerSkeletonGroup
hiros::skeletons::utils::toMsg(const unsigned int& t_seq,
                               const ros::Time& t_stamp,
                               const std::string& t_frame_id,
                               const hiros::skeletons::types::MarkerSkeletonGroup& t_msg)
{
  return toMsg(t_seq, t_stamp, t_frame_id, ros::Time(t_msg.src_time), t_msg.src_frame, t_msg);
}

skeleton_msgs::MarkerSkeletonGroup
hiros::skeletons::utils::toMsg(const std_msgs::Header& t_header,
                               const ros::Time& t_src_time,
                               const std::string& t_src_frame,
                               const hiros::skeletons::types::MarkerSkeletonGroup& t_msg)
{
  return toMsg(t_header.seq, t_header.stamp, t_header.frame_id, t_src_time, t_src_frame, t_msg);
}

skeleton_msgs::MarkerSkeletonGroup
hiros::skeletons::utils::toMsg(const ros::Time& t_stamp,
                               const std::string& t_frame_id,
                               const hiros::skeletons::types::MarkerSkeletonGroup& t_msg)
{
  return toMsg(0, t_stamp, t_frame_id, t_msg);
}

skeleton_msgs::MarkerSkeletonGroup
hiros::skeletons::utils::toMsg(const std_msgs::Header& t_header,
                               const hiros::skeletons::types::MarkerSkeletonGroup& t_msg)
{
  return toMsg(t_header, ros::Time(t_msg.src_time), t_msg.src_frame, t_msg);
}

skeleton_msgs::MarkerSkeletonGroup
hiros::skeletons::utils::toMsg(const hiros::skeletons::types::MarkerSkeletonGroup& t_msg)
{
  return toMsg(std_msgs::Header(), t_msg);
}
