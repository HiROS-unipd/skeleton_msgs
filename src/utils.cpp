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
tf2::Vector3
hiros::skeletons::utils::toStruct(const double& t_x, const double& t_y, const double& t_z)
{
  return tf2::Vector3(t_x, t_y, t_z);
}

tf2::Vector3 hiros::skeletons::utils::toStruct(const geometry_msgs::Point& t_p)
{
  return tf2::Vector3(t_p.x, t_p.y, t_p.z);
}

tf2::Vector3 hiros::skeletons::utils::toStruct(const geometry_msgs::Vector3& t_v)
{
  return tf2::Vector3(t_v.x, t_v.y, t_v.z);
}

geometry_msgs::Point hiros::skeletons::utils::toPointMsg(const tf2::Vector3& t_v)
{
  geometry_msgs::Point p;
  p.x = t_v.x();
  p.y = t_v.y();
  p.z = t_v.z();
  return p;
}

geometry_msgs::Vector3 hiros::skeletons::utils::toVector3Msg(const tf2::Vector3& t_v)
{
  geometry_msgs::Vector3 v;
  v.x = t_v.x();
  v.y = t_v.y();
  v.z = t_v.z();
  return v;
}

double hiros::skeletons::utils::magnitude(const tf2::Vector3& t_v)
{
  return t_v.length();
}

double hiros::skeletons::utils::distance(const types::Position& t_p1, const types::Position& t_p2)
{
  return (!std::isnan(t_p1.z()) && !std::isnan(t_p2.z()))
           ? t_p1.distance(t_p2)
           : types::Position(t_p1.x(), t_p1.y(), 0)
               .distance(types::Position(t_p2.x(), t_p2.y(), 0));
}

std::string hiros::skeletons::utils::toString(const tf2::Vector3& t_v, int t_pad_lv)
{
  std::stringstream ss;
  ss << padding(t_pad_lv) << "- x: " << t_v.x() << std::endl
     << padding(t_pad_lv + 1) << "y: " << t_v.y();
  if (!std::isnan(t_v.z())) {
    ss << std::endl << padding(t_pad_lv + 1) << "z: " << t_v.z();
  }
  return ss.str();
}

// Point
hiros::skeletons::types::Point
hiros::skeletons::utils::toStruct(const hiros::skeletons::types::Position& t_p,
                                  const hiros::skeletons::types::Velocity& t_v,
                                  const hiros::skeletons::types::Acceleration& t_a)
{
  return hiros::skeletons::types::Point(t_p, t_v, t_a);
}

hiros::skeletons::types::Point
hiros::skeletons::utils::toStruct(const hiros_skeleton_msgs::Point& t_p)
{
  return hiros::skeletons::types::Point(
    toStruct(t_p.position), toStruct(t_p.velocity), toStruct(t_p.acceleration));
}

hiros_skeleton_msgs::Point hiros::skeletons::utils::toMsg(const hiros::skeletons::types::Point& t_p)
{
  hiros_skeleton_msgs::Point p;
  p.position = toPointMsg(t_p.position);
  p.velocity = toVector3Msg(t_p.velocity);
  p.acceleration = toVector3Msg(t_p.acceleration);
  return p;
}

std::string hiros::skeletons::utils::toString(const hiros::skeletons::types::Point& t_p,
                                              int t_pad_lv)
{
  std::stringstream ss;
  ss << padding(t_pad_lv) << "- position: " << std::endl << toString(t_p.position, t_pad_lv + 1);
  if (!std::isnan(t_p.velocity.x())) {
    ss << std::endl
       << padding(t_pad_lv + 1) << "velocity: " << std::endl
       << toString(t_p.velocity, t_pad_lv + 1);
  }
  if (!std::isnan(t_p.acceleration.x())) {
    ss << std::endl
       << padding(t_pad_lv + 1) << "acceleration: " << std::endl
       << toString(t_p.acceleration, t_pad_lv + 1);
  }
  return ss.str();
}

// Quaternion
tf2::Quaternion hiros::skeletons::utils::toStruct(const double& t_x,
                                                  const double& t_y,
                                                  const double& t_z,
                                                  const double& t_w)
{
  return tf2::Quaternion(t_x, t_y, t_z, t_w);
}

tf2::Quaternion hiros::skeletons::utils::toStruct(const geometry_msgs::Quaternion& t_q)
{
  return tf2::Quaternion(t_q.x, t_q.y, t_q.z, t_q.w);
}

geometry_msgs::Quaternion hiros::skeletons::utils::toMsg(const tf2::Quaternion& t_q)
{
  geometry_msgs::Quaternion q;
  q.x = t_q.x();
  q.y = t_q.y();
  q.z = t_q.z();
  q.w = t_q.w();
  return q;
}

double hiros::skeletons::utils::distance(const tf2::Quaternion& t_q1, const tf2::Quaternion& t_q2)
{
  return t_q1.normalized().angleShortestPath(t_q2.normalized());
}

std::string hiros::skeletons::utils::toString(const tf2::Quaternion& t_q, int t_pad_lv)
{
  std::stringstream ss;
  ss << padding(t_pad_lv) << "- x: " << t_q.x() << std::endl
     << padding(t_pad_lv + 1) << "y: " << t_q.y() << std::endl
     << padding(t_pad_lv + 1) << "z: " << t_q.z() << std::endl
     << padding(t_pad_lv + 1) << "w: " << t_q.w();
  return ss.str();
}

// Box
hiros::skeletons::types::Box
hiros::skeletons::utils::toStruct(const hiros::skeletons::types::Point& t_center,
                                  const double& t_length,
                                  const double& t_height,
                                  const double& t_width,
                                  const tf2::Quaternion& t_orientation)
{
  return hiros::skeletons::types::Box(t_center, t_length, t_height, t_width, t_orientation);
}

hiros::skeletons::types::Box hiros::skeletons::utils::toStruct(const hiros_skeleton_msgs::Box& t_b)
{
  return hiros::skeletons::types::Box(
    toStruct(t_b.center), t_b.length, t_b.height, t_b.width, toStruct(t_b.orientation));
}

hiros_skeleton_msgs::Box hiros::skeletons::utils::toMsg(const hiros::skeletons::types::Box& t_b)
{
  hiros_skeleton_msgs::Box b;
  b.center = toMsg(t_b.center);
  b.length = t_b.length;
  b.height = t_b.height;
  b.width = t_b.width;
  b.orientation = toMsg(t_b.orientation);
  return b;
}

std::string hiros::skeletons::utils::toString(const hiros::skeletons::types::Box& t_b, int t_pad_lv)
{
  std::stringstream ss;
  ss << padding(t_pad_lv) << "- center: " << std::endl
     << toString(t_b.center, t_pad_lv + 1) << std::endl
     << padding(t_pad_lv + 1) << "length: " << t_b.length << std::endl
     << padding(t_pad_lv + 1) << "height: " << t_b.height;
  if (!std::isnan(t_b.width)) {
    ss << std::endl << padding(t_pad_lv + 1) << "width: " << t_b.width;
  }
  if (!std::isnan(t_b.orientation.w())) {
    ss << std::endl
       << padding(t_pad_lv + 1) << "orientation: " << std::endl
       << toString(t_b.orientation, t_pad_lv + 1);
  }
  return ss.str();
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

hiros::skeletons::types::Marker
hiros::skeletons::utils::toStruct(const hiros_skeleton_msgs::Marker& t_m)
{
  return hiros::skeletons::types::Marker(t_m.id, t_m.confidence, toStruct(t_m.point));
}

hiros_skeleton_msgs::Marker
hiros::skeletons::utils::toMsg(const hiros::skeletons::types::Marker& t_m)
{
  hiros_skeleton_msgs::Marker m;
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

std::string hiros::skeletons::utils::toString(const hiros::skeletons::types::Marker& t_m,
                                              int t_pad_lv)
{
  std::stringstream ss;
  ss << padding(t_pad_lv) << "- id: " << t_m.id << std::endl
     << padding(t_pad_lv + 1) << "confidence: " << t_m.confidence << std::endl
     << padding(t_pad_lv + 1) << "point: " << std::endl
     << toString(t_m.point, t_pad_lv + 1);
  return ss.str();
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
hiros::skeletons::utils::toStruct(const hiros_skeleton_msgs::MarkerGroup& t_mg)
{
  hiros::skeletons::types::MarkerGroup mg(
    t_mg.id, t_mg.max_markers, t_mg.confidence, toStruct(t_mg.bounding_box));
  for (auto& m : t_mg.markers) {
    mg.markers.emplace(m.id, toStruct(m));
  }
  return mg;
}

hiros_skeleton_msgs::MarkerGroup
hiros::skeletons::utils::toMsg(const hiros::skeletons::types::MarkerGroup& t_mg)
{
  hiros_skeleton_msgs::MarkerGroup mg;
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

std::string hiros::skeletons::utils::toString(const hiros::skeletons::types::MarkerGroup& t_mg,
                                              int t_pad_lv)
{
  std::stringstream ss;
  ss << padding(t_pad_lv) << "- id: " << t_mg.id << std::endl
     << padding(t_pad_lv + 1) << "max_markers: " << t_mg.max_markers << std::endl
     << padding(t_pad_lv + 1) << "confidence: " << t_mg.confidence << std::endl
     << padding(t_pad_lv + 1) << "bounding_box: " << std::endl
     << toString(t_mg.bounding_box, t_pad_lv + 1) << std::endl
     << padding(t_pad_lv + 1) << "markers: ";
  if (t_mg.markers.empty()) {
    ss << "[]";
  }
  else {
    for (auto m : t_mg.markers) {
      ss << std::endl << toString(m.second, t_pad_lv + 1);
    }
  }
  return ss.str();
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
hiros::skeletons::utils::toStruct(const hiros_skeleton_msgs::MarkerSkeleton& t_ms)
{
  hiros::skeletons::types::MarkerSkeleton ms(t_ms.id, t_ms.confidence);
  for (auto& mg : t_ms.marker_groups) {
    ms.marker_groups.emplace(mg.id, toStruct(mg));
  }
  return ms;
}

hiros_skeleton_msgs::MarkerSkeleton
hiros::skeletons::utils::toMsg(const hiros::skeletons::types::MarkerSkeleton& t_ms)
{
  hiros_skeleton_msgs::MarkerSkeleton ms;
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
  auto marker_skeleton_it = std::find_if(t_marker_skeleton_group.marker_skeletons.begin(),
                                         t_marker_skeleton_group.marker_skeletons.end(),
                                         [&](const hiros::skeletons::types::MarkerSkeleton& ms) {
                                           return ms.id == t_marker_skeleton_id;
                                         });

  return marker_skeleton_it != t_marker_skeleton_group.marker_skeletons.end()
           ? std::make_shared<hiros::skeletons::types::MarkerSkeleton>(*marker_skeleton_it)
           : nullptr;
}

bool hiros::skeletons::utils::hasMarkerSkeleton(
  const hiros::skeletons::types::MarkerSkeletonGroup& t_marker_skeleton_group,
  const int& t_marker_skeleton_id)
{
  return std::find_if(t_marker_skeleton_group.marker_skeletons.begin(),
                      t_marker_skeleton_group.marker_skeletons.end(),
                      [&](const hiros::skeletons::types::MarkerSkeleton& ms) {
                        return ms.id == t_marker_skeleton_id;
                      })
         != t_marker_skeleton_group.marker_skeletons.end();
}

std::string hiros::skeletons::utils::toString(const hiros::skeletons::types::MarkerSkeleton& t_ms,
                                              int t_pad_lv)
{
  std::stringstream ss;
  ss << padding(t_pad_lv) << "- id: " << t_ms.id << std::endl
     << padding(t_pad_lv + 1) << "confidence: " << t_ms.confidence << std::endl
     << padding(t_pad_lv + 1) << "marker_groups: ";
  if (t_ms.marker_groups.empty()) {
    ss << "[]";
  }
  else {
    for (auto mg : t_ms.marker_groups) {
      ss << std::endl << toString(mg.second, t_pad_lv + 1);
    }
  }
  return ss.str();
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
hiros::skeletons::utils::toStruct(const hiros_skeleton_msgs::MarkerSkeletonGroup& t_msg)
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

hiros_skeleton_msgs::MarkerSkeletonGroup
hiros::skeletons::utils::toMsg(const unsigned int& t_seq,
                               const ros::Time& t_stamp,
                               const std::string& t_frame_id,
                               const ros::Time& t_src_time,
                               const std::string& t_src_frame,
                               const hiros::skeletons::types::MarkerSkeletonGroup& t_msg)
{
  hiros_skeleton_msgs::MarkerSkeletonGroup msg;
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

hiros_skeleton_msgs::MarkerSkeletonGroup
hiros::skeletons::utils::toMsg(const ros::Time& t_stamp,
                               const std::string& t_frame_id,
                               const ros::Time& t_src_time,
                               const std::string& t_src_frame,
                               const hiros::skeletons::types::MarkerSkeletonGroup& t_msg)
{
  return toMsg(0, t_stamp, t_frame_id, t_src_time, t_src_frame, t_msg);
}

hiros_skeleton_msgs::MarkerSkeletonGroup
hiros::skeletons::utils::toMsg(const unsigned int& t_seq,
                               const ros::Time& t_stamp,
                               const std::string& t_frame_id,
                               const hiros::skeletons::types::MarkerSkeletonGroup& t_msg)
{
  return toMsg(t_seq, t_stamp, t_frame_id, ros::Time(t_msg.src_time), t_msg.src_frame, t_msg);
}

hiros_skeleton_msgs::MarkerSkeletonGroup
hiros::skeletons::utils::toMsg(const std_msgs::Header& t_header,
                               const ros::Time& t_src_time,
                               const std::string& t_src_frame,
                               const hiros::skeletons::types::MarkerSkeletonGroup& t_msg)
{
  return toMsg(t_header.seq, t_header.stamp, t_header.frame_id, t_src_time, t_src_frame, t_msg);
}

hiros_skeleton_msgs::MarkerSkeletonGroup
hiros::skeletons::utils::toMsg(const ros::Time& t_stamp,
                               const std::string& t_frame_id,
                               const hiros::skeletons::types::MarkerSkeletonGroup& t_msg)
{
  return toMsg(0, t_stamp, t_frame_id, t_msg);
}

hiros_skeleton_msgs::MarkerSkeletonGroup
hiros::skeletons::utils::toMsg(const std_msgs::Header& t_header,
                               const hiros::skeletons::types::MarkerSkeletonGroup& t_msg)
{
  return std::isnan(t_msg.src_time)
           ? toMsg(t_header, t_header.stamp, t_msg.src_frame, t_msg)
           : toMsg(t_header, ros::Time(t_msg.src_time), t_msg.src_frame, t_msg);
}

hiros_skeleton_msgs::MarkerSkeletonGroup
hiros::skeletons::utils::toMsg(const hiros::skeletons::types::MarkerSkeletonGroup& t_msg)
{
  return toMsg(std_msgs::Header(), t_msg);
}

std::string
hiros::skeletons::utils::toString(const hiros::skeletons::types::MarkerSkeletonGroup& t_msg,
                                  int t_pad_lv)
{
  std::stringstream ss;
  ss << padding(t_pad_lv) << "- src_time: ";
  if (!std::isnan(t_msg.src_time)) {
    long src_time_sec = static_cast<long>(t_msg.src_time);
    long src_time_nsec = static_cast<long>((t_msg.src_time - src_time_sec) * 1e9);
    ss << src_time_sec << "." << src_time_nsec;
  }
  else {
    ss << "nan";
  }
  ss << std::endl
     << padding(t_pad_lv + 1) << "src_frame: " << t_msg.src_frame << std::endl
     << padding(t_pad_lv + 1) << "marker_skeletons: ";
  if (t_msg.marker_skeletons.empty()) {
    ss << "[]";
  }
  else {
    for (auto ms : t_msg.marker_skeletons) {
      ss << std::endl << toString(ms, t_pad_lv + 1);
    }
  }
  return ss.str();
}

// MIMU
hiros::skeletons::types::MIMU
hiros::skeletons::utils::toStruct(const std::string& t_frame_id,
                                  const tf2::Quaternion& t_orientation,
                                  const tf2::Vector3& t_angular_velocity,
                                  const tf2::Vector3& t_linear_acceleration,
                                  const tf2::Vector3& t_magnetic_field)
{
  return hiros::skeletons::types::MIMU(
    t_frame_id, t_orientation, t_angular_velocity, t_linear_acceleration, t_magnetic_field);
}

hiros::skeletons::types::MIMU
hiros::skeletons::utils::toStruct(const hiros_skeleton_msgs::MIMU& t_m)
{
  return hiros::skeletons::types::MIMU(t_m.imu.header.frame_id,
                                       toStruct(t_m.imu.orientation),
                                       toStruct(t_m.imu.angular_velocity),
                                       toStruct(t_m.imu.linear_acceleration),
                                       toStruct(t_m.mag.magnetic_field));
}

hiros_skeleton_msgs::MIMU hiros::skeletons::utils::toMsg(const std_msgs::Header& t_header,
                                                         const hiros::skeletons::types::MIMU& t_m)
{
  hiros_skeleton_msgs::MIMU m;
  m.imu.header = t_header;
  m.imu.header.frame_id = t_m.frame_id;
  m.imu.orientation = toMsg(t_m.orientation);
  m.imu.angular_velocity = toVector3Msg(t_m.angular_velocity);
  m.imu.linear_acceleration = toVector3Msg(t_m.linear_acceleration);
  m.imu.orientation_covariance.front() = -1;
  m.imu.angular_velocity_covariance.front() = -1;
  m.imu.linear_acceleration_covariance.front() = -1;
  m.mag.header = t_header;
  m.mag.header.frame_id = t_m.frame_id;
  m.mag.magnetic_field = toVector3Msg(t_m.magnetic_field);
  m.mag.magnetic_field_covariance.front() = -1;
  return m;
}

hiros_skeleton_msgs::MIMU hiros::skeletons::utils::toMsg(const hiros::skeletons::types::MIMU& t_m)
{
  std_msgs::Header h;
  h.frame_id = t_m.frame_id;
  return toMsg(h, t_m);
}

std::string hiros::skeletons::utils::toString(const hiros::skeletons::types::MIMU& t_m,
                                              int t_pad_lv)
{
  std::stringstream ss;
  ss << padding(t_pad_lv) << "- frame_id: " << t_m.frame_id << std::endl
     << padding(t_pad_lv + 1) << "orientation: " << std::endl
     << utils::toString(t_m.orientation, t_pad_lv + 1);
  if (!std::isnan(t_m.angular_velocity.x())) {
    ss << std::endl
       << padding(t_pad_lv + 1) << "angular_velocity: " << std::endl
       << utils::toString(t_m.angular_velocity, t_pad_lv + 1);
  }
  if (!std::isnan(t_m.linear_acceleration.x())) {
    ss << std::endl
       << padding(t_pad_lv + 1) << "linear_acceleration: " << std::endl
       << utils::toString(t_m.linear_acceleration, t_pad_lv + 1);
  }
  if (!std::isnan(t_m.magnetic_field.x())) {
    ss << std::endl
       << padding(t_pad_lv + 1) << "magnetic_field: " << std::endl
       << utils::toString(t_m.magnetic_field, t_pad_lv + 1);
  }
  return ss.str();
}

// Orientation
hiros::skeletons::types::Orientation
hiros::skeletons::utils::toStruct(const int& t_id,
                                  const double& t_confidence,
                                  const hiros::skeletons::types::MIMU& t_mimu)
{
  return hiros::skeletons::types::Orientation(t_id, t_confidence, t_mimu);
}

hiros::skeletons::types::Orientation
hiros::skeletons::utils::toStruct(const hiros_skeleton_msgs::Orientation& t_o)
{
  return hiros::skeletons::types::Orientation(t_o.id, t_o.confidence, toStruct(t_o.mimu));
}

hiros_skeleton_msgs::Orientation
hiros::skeletons::utils::toMsg(const std_msgs::Header& t_header,
                               const hiros::skeletons::types::Orientation& t_o)
{
  hiros_skeleton_msgs::Orientation o;
  o.id = t_o.id;
  o.confidence = t_o.confidence;
  o.mimu = toMsg(t_header, t_o.mimu);
  return o;
}

hiros_skeleton_msgs::Orientation
hiros::skeletons::utils::toMsg(const hiros::skeletons::types::Orientation& t_o)
{
  std_msgs::Header h;
  h.frame_id = t_o.mimu.frame_id;
  return toMsg(h, t_o);
}

bool hiros::skeletons::utils::hasOrientation(
  const hiros::skeletons::types::OrientationGroup& t_orientation_group,
  const int& t_orientation_id)
{
  return (t_orientation_group.orientations.count(t_orientation_id) > 0);
}

bool hiros::skeletons::utils::hasOrientation(
  const hiros::skeletons::types::OrientationSkeleton& t_orientation_skeleton,
  const int& t_orientation_group_id,
  const int& t_orientation_id)
{
  return (hasOrientationGroup(t_orientation_skeleton, t_orientation_group_id)
          && hasOrientation(t_orientation_skeleton.orientation_groups.at(t_orientation_group_id),
                            t_orientation_id));
}

std::string hiros::skeletons::utils::toString(const hiros::skeletons::types::Orientation& t_o,
                                              int t_pad_lv)
{
  std::stringstream ss;
  ss << padding(t_pad_lv) << "- id: " << t_o.id << std::endl
     << padding(t_pad_lv + 1) << "confidence: " << t_o.confidence << std::endl
     << padding(t_pad_lv + 1) << "mimu: " << std::endl
     << utils::toString(t_o.mimu, t_pad_lv + 1);
  return ss.str();
}

// OrientationGroup
hiros::skeletons::types::OrientationGroup hiros::skeletons::utils::toStruct(
  const int& t_id,
  const unsigned int& t_max_orientations,
  const double& t_confidence,
  const std::vector<hiros::skeletons::types::Orientation> t_orientations)
{
  return hiros::skeletons::types::OrientationGroup(
    t_id, t_max_orientations, t_confidence, t_orientations);
}

hiros::skeletons::types::OrientationGroup
hiros::skeletons::utils::toStruct(const hiros_skeleton_msgs::OrientationGroup& t_og)
{
  hiros::skeletons::types::OrientationGroup og(t_og.id, t_og.max_orientations, t_og.confidence);
  for (auto& o : t_og.orientations) {
    og.orientations.emplace(o.id, toStruct(o));
  }
  return og;
}

hiros_skeleton_msgs::OrientationGroup
hiros::skeletons::utils::toMsg(const std_msgs::Header& t_header,
                               const hiros::skeletons::types::OrientationGroup& t_og)
{
  hiros_skeleton_msgs::OrientationGroup og;
  og.id = t_og.id;
  og.max_orientations = t_og.max_orientations;
  og.confidence = t_og.confidence;
  og.orientations.reserve(t_og.orientations.size());
  for (auto& o : t_og.orientations) {
    og.orientations.push_back(toMsg(t_header, o.second));
  }
  return og;
}

hiros_skeleton_msgs::OrientationGroup
hiros::skeletons::utils::toMsg(const hiros::skeletons::types::OrientationGroup& t_og)
{
  return toMsg({}, t_og);
}

bool hiros::skeletons::utils::hasOrientationGroup(
  const hiros::skeletons::types::OrientationSkeleton& t_orientation_skeleton,
  const int& t_orientation_group_id)
{
  return (t_orientation_skeleton.orientation_groups.count(t_orientation_group_id) > 0);
}

std::string hiros::skeletons::utils::toString(const hiros::skeletons::types::OrientationGroup& t_og,
                                              int t_pad_lv)
{
  std::stringstream ss;
  ss << padding(t_pad_lv) << "- id: " << t_og.id << std::endl
     << padding(t_pad_lv + 1) << "max_orientations: " << t_og.max_orientations << std::endl
     << padding(t_pad_lv + 1) << "confidence: " << t_og.confidence << std::endl
     << padding(t_pad_lv + 1) << "orientations: ";
  if (t_og.orientations.empty()) {
    ss << "[]";
  }
  else {
    for (auto o : t_og.orientations) {
      ss << std::endl << utils::toString(o.second, t_pad_lv + 1);
    }
  }
  return ss.str();
}

// OrientationSkeleton
hiros::skeletons::types::OrientationSkeleton hiros::skeletons::utils::toStruct(
  const int& t_id,
  const double& t_confidence,
  const std::vector<hiros::skeletons::types::OrientationGroup>& t_orientation_groups)
{
  return hiros::skeletons::types::OrientationSkeleton(t_id, t_confidence, t_orientation_groups);
}

hiros::skeletons::types::OrientationSkeleton
hiros::skeletons::utils::toStruct(const hiros_skeleton_msgs::OrientationSkeleton& t_os)
{
  hiros::skeletons::types::OrientationSkeleton os(t_os.id, t_os.confidence);
  for (auto& og : t_os.orientation_groups) {
    os.orientation_groups.emplace(og.id, toStruct(og));
  }
  return os;
}

hiros_skeleton_msgs::OrientationSkeleton
hiros::skeletons::utils::toMsg(const std_msgs::Header& t_header,
                               const hiros::skeletons::types::OrientationSkeleton& t_os)
{
  hiros_skeleton_msgs::OrientationSkeleton os;
  os.id = t_os.id;
  os.confidence = t_os.confidence;
  os.orientation_groups.reserve(t_os.orientation_groups.size());
  for (auto& og : t_os.orientation_groups) {
    os.orientation_groups.push_back(toMsg(t_header, og.second));
  }
  return os;
}

hiros_skeleton_msgs::OrientationSkeleton
hiros::skeletons::utils::toMsg(const hiros::skeletons::types::OrientationSkeleton& t_os)
{
  return toMsg({}, t_os);
}

std::shared_ptr<hiros::skeletons::types::OrientationSkeleton>
hiros::skeletons::utils::getOrientationSkeleton(
  hiros::skeletons::types::OrientationSkeletonGroup& t_orientation_skeleton_group,
  const int& t_orientation_skeleton_id)
{
  auto orientation_skeleton_it =
    std::find_if(t_orientation_skeleton_group.orientation_skeletons.begin(),
                 t_orientation_skeleton_group.orientation_skeletons.end(),
                 [&](const hiros::skeletons::types::OrientationSkeleton& os) {
                   return os.id == t_orientation_skeleton_id;
                 });

  return orientation_skeleton_it != t_orientation_skeleton_group.orientation_skeletons.end()
           ? std::make_shared<hiros::skeletons::types::OrientationSkeleton>(
             *orientation_skeleton_it)
           : nullptr;
}

bool hiros::skeletons::utils::hasOrientationSkeleton(
  const hiros::skeletons::types::OrientationSkeletonGroup& t_orientation_skeleton_group,
  const int& t_orientation_skeleton_id)
{
  return std::find_if(t_orientation_skeleton_group.orientation_skeletons.begin(),
                      t_orientation_skeleton_group.orientation_skeletons.end(),
                      [&](const hiros::skeletons::types::OrientationSkeleton& os) {
                        return os.id == t_orientation_skeleton_id;
                      })
         != t_orientation_skeleton_group.orientation_skeletons.end();
}

std::string
hiros::skeletons::utils::toString(const hiros::skeletons::types::OrientationSkeleton& t_os,
                                  int t_pad_lv)
{
  std::stringstream ss;
  ss << padding(t_pad_lv) << "- id: " << t_os.id << std::endl
     << padding(t_pad_lv + 1) << "confidence: " << t_os.confidence << std::endl
     << padding(t_pad_lv + 1) << "orientation_groups: ";
  if (t_os.orientation_groups.empty()) {
    ss << "[]";
  }
  else {
    for (auto og : t_os.orientation_groups) {
      ss << std::endl << toString(og.second, t_pad_lv + 1);
    }
  }
  return ss.str();
}

// OrientationSkeletonGroup
hiros::skeletons::types::OrientationSkeletonGroup hiros::skeletons::utils::toStruct(
  const double& t_src_time,
  const std::string& t_src_frame,
  const std::vector<hiros::skeletons::types::OrientationSkeleton>& t_orientation_skeletons)
{
  return hiros::skeletons::types::OrientationSkeletonGroup(
    t_src_time, t_src_frame, t_orientation_skeletons);
}

hiros::skeletons::types::OrientationSkeletonGroup
hiros::skeletons::utils::toStruct(const hiros_skeleton_msgs::OrientationSkeletonGroup& t_osg)
{
  hiros::skeletons::types::OrientationSkeletonGroup osg;
  osg.src_time = t_osg.src_time.toSec();
  osg.src_frame = t_osg.src_frame;
  osg.orientation_skeletons.reserve(t_osg.orientation_skeletons.size());
  for (auto& os : t_osg.orientation_skeletons) {
    osg.orientation_skeletons.push_back(toStruct(os));
  }
  return osg;
}

hiros_skeleton_msgs::OrientationSkeletonGroup
hiros::skeletons::utils::toMsg(const unsigned int& t_seq,
                               const ros::Time& t_stamp,
                               const std::string& t_frame_id,
                               const ros::Time& t_src_time,
                               const std::string& t_src_frame,
                               const hiros::skeletons::types::OrientationSkeletonGroup& t_osg)
{
  hiros_skeleton_msgs::OrientationSkeletonGroup osg;
  osg.header.seq = t_seq;
  osg.header.stamp = t_stamp;
  osg.header.frame_id = t_frame_id;
  osg.src_time = t_src_time;
  osg.src_frame = t_src_frame;
  osg.orientation_skeletons.reserve(t_osg.orientation_skeletons.size());
  for (auto& os : t_osg.orientation_skeletons) {
    osg.orientation_skeletons.push_back(toMsg(osg.header, os));
  }
  return osg;
}

hiros_skeleton_msgs::OrientationSkeletonGroup
hiros::skeletons::utils::toMsg(const ros::Time& t_stamp,
                               const std::string& t_frame_id,
                               const ros::Time& t_src_time,
                               const std::string& t_src_frame,
                               const hiros::skeletons::types::OrientationSkeletonGroup& t_osg)
{
  return toMsg(0, t_stamp, t_frame_id, t_src_time, t_src_frame, t_osg);
}

hiros_skeleton_msgs::OrientationSkeletonGroup
hiros::skeletons::utils::toMsg(const unsigned int& t_seq,
                               const ros::Time& t_stamp,
                               const std::string& t_frame_id,
                               const hiros::skeletons::types::OrientationSkeletonGroup& t_osg)
{
  return toMsg(t_seq, t_stamp, t_frame_id, ros::Time(t_osg.src_time), t_osg.src_frame, t_osg);
}

hiros_skeleton_msgs::OrientationSkeletonGroup
hiros::skeletons::utils::toMsg(const std_msgs::Header& t_header,
                               const ros::Time& t_src_time,
                               const std::string& t_src_frame,
                               const hiros::skeletons::types::OrientationSkeletonGroup& t_osg)
{
  return toMsg(t_header.seq, t_header.stamp, t_header.frame_id, t_src_time, t_src_frame, t_osg);
}

hiros_skeleton_msgs::OrientationSkeletonGroup
hiros::skeletons::utils::toMsg(const ros::Time& t_stamp,
                               const std::string& t_frame_id,
                               const hiros::skeletons::types::OrientationSkeletonGroup& t_osg)
{
  return toMsg(0, t_stamp, t_frame_id, t_osg);
}

hiros_skeleton_msgs::OrientationSkeletonGroup
hiros::skeletons::utils::toMsg(const std_msgs::Header& t_header,
                               const hiros::skeletons::types::OrientationSkeletonGroup& t_osg)
{
  return std::isnan(t_osg.src_time)
           ? toMsg(t_header, t_header.stamp, t_osg.src_frame, t_osg)
           : toMsg(t_header, ros::Time(t_osg.src_time), t_osg.src_frame, t_osg);
}

hiros_skeleton_msgs::OrientationSkeletonGroup
hiros::skeletons::utils::toMsg(const hiros::skeletons::types::OrientationSkeletonGroup& t_osg)
{
  return toMsg(std_msgs::Header(), t_osg);
}

std::string
hiros::skeletons::utils::toString(const hiros::skeletons::types::OrientationSkeletonGroup& t_osg,
                                  int t_pad_lv)
{
  std::stringstream ss;
  ss << padding(t_pad_lv) << "- src_time: ";
  if (!std::isnan(t_osg.src_time)) {
    long src_time_sec = static_cast<long>(t_osg.src_time);
    long src_time_nsec = static_cast<long>((t_osg.src_time - src_time_sec) * 1e9);
    ss << std::to_string(src_time_sec) << "." << std::to_string(src_time_nsec);
  }
  else {
    ss << "nan";
  }
  ss << std::endl
     << padding(t_pad_lv + 1) << "src_frame: " << t_osg.src_frame << std::endl
     << padding(t_pad_lv + 1) << "orientation_skeletons: ";
  if (t_osg.orientation_skeletons.empty()) {
    ss << "[]";
  }
  else {
    for (auto os : t_osg.orientation_skeletons) {
      ss << std::endl << toString(os, t_pad_lv + 1);
    }
  }
  return ss.str();
}
