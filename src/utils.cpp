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

// Keypoint
hiros::skeletons::types::Keypoint
hiros::skeletons::utils::toStruct(const int& t_id,
                                  const double& t_confidence,
                                  const hiros::skeletons::types::Point& t_point)
{
  return hiros::skeletons::types::Keypoint(t_id, t_confidence, t_point);
}

hiros::skeletons::types::Keypoint
hiros::skeletons::utils::toStruct(const int& t_id,
                                  const double& t_confidence,
                                  const hiros::skeletons::types::Position& t_position,
                                  const hiros::skeletons::types::Velocity& t_velocity,
                                  const hiros::skeletons::types::Acceleration& t_acceleration)
{
  return hiros::skeletons::types::Keypoint(
    t_id, t_confidence, hiros::skeletons::types::Point(t_position, t_velocity, t_acceleration));
}

hiros::skeletons::types::Keypoint
hiros::skeletons::utils::toStruct(const skeleton_msgs::Keypoint& t_k)
{
  return hiros::skeletons::types::Keypoint(t_k.id, t_k.confidence, toStruct(t_k.point));
}

skeleton_msgs::Keypoint hiros::skeletons::utils::toMsg(const hiros::skeletons::types::Keypoint& t_k)
{
  skeleton_msgs::Keypoint k;
  k.id = t_k.id;
  k.confidence = t_k.confidence;
  k.point = toMsg(t_k.point);
  return k;
}

// keypointGroup
hiros::skeletons::types::KeypointGroup
hiros::skeletons::utils::toStruct(const int& t_id,
                                  const unsigned int& t_max_keypoints,
                                  const double& t_confidence,
                                  const std::vector<hiros::skeletons::types::Keypoint> t_keypoints)
{
  return hiros::skeletons::types::KeypointGroup(
    t_id, t_max_keypoints, t_confidence, hiros::skeletons::types::Box(), t_keypoints);
}

hiros::skeletons::types::KeypointGroup
hiros::skeletons::utils::toStruct(const int& t_id,
                                  const unsigned int& t_max_keypoints,
                                  const double& t_confidence,
                                  const hiros::skeletons::types::Box& t_bounding_box,
                                  const std::vector<hiros::skeletons::types::Keypoint> t_keypoints)
{
  return hiros::skeletons::types::KeypointGroup(
    t_id, t_max_keypoints, t_confidence, t_bounding_box, t_keypoints);
}

hiros::skeletons::types::KeypointGroup
hiros::skeletons::utils::toStruct(const skeleton_msgs::KeypointGroup& t_kg)
{
  hiros::skeletons::types::KeypointGroup kg(
    t_kg.id, t_kg.max_keypoints, t_kg.confidence, toStruct(t_kg.bounding_box));
  kg.keypoints.reserve(t_kg.keypoints.size());
  for (auto& k : t_kg.keypoints) {
    kg.keypoints.push_back(toStruct(k));
  }
  return kg;
}

skeleton_msgs::KeypointGroup
hiros::skeletons::utils::toMsg(const hiros::skeletons::types::KeypointGroup& t_kg)
{
  skeleton_msgs::KeypointGroup kg;
  kg.id = t_kg.id;
  kg.max_keypoints = t_kg.max_keypoints;
  kg.confidence = t_kg.confidence;
  kg.bounding_box = toMsg(t_kg.bounding_box);
  kg.keypoints.reserve(t_kg.keypoints.size());
  for (auto& k : t_kg.keypoints) {
    kg.keypoints.push_back(toMsg(k));
  }
  return kg;
}

// Skeleton
hiros::skeletons::types::Skeleton hiros::skeletons::utils::toStruct(
  const int& t_id,
  const std::vector<hiros::skeletons::types::KeypointGroup>& t_skeleton_parts)
{
  return hiros::skeletons::types::Skeleton(t_id, t_skeleton_parts);
}

hiros::skeletons::types::Skeleton
hiros::skeletons::utils::toStruct(const skeleton_msgs::Skeleton& t_s)
{
  hiros::skeletons::types::Skeleton s(t_s.id);
  s.skeleton_parts.reserve(t_s.skeleton_parts.size());
  for (auto& sp : t_s.skeleton_parts) {
    s.skeleton_parts.push_back(toStruct(sp));
  }
  return s;
}

skeleton_msgs::Skeleton hiros::skeletons::utils::toMsg(const hiros::skeletons::types::Skeleton& t_s)
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
hiros::skeletons::utils::toStruct(const double& t_src_time,
                                  const std::string& t_src_frame,
                                  const std::vector<hiros::skeletons::types::Skeleton>& t_skeletons)
{
  return hiros::skeletons::types::SkeletonGroup(t_src_time, t_src_frame, t_skeletons);
}

hiros::skeletons::types::SkeletonGroup
hiros::skeletons::utils::toStruct(const skeleton_msgs::SkeletonGroup& t_sg)
{
  hiros::skeletons::types::SkeletonGroup sg;
  sg.src_time = t_sg.src_time.toSec();
  sg.src_frame = t_sg.src_frame;
  sg.skeletons.reserve(t_sg.skeletons.size());
  for (auto& s : t_sg.skeletons) {
    sg.skeletons.push_back(toStruct(s));
  }
  return sg;
}

skeleton_msgs::SkeletonGroup
hiros::skeletons::utils::toMsg(const hiros::skeletons::types::SkeletonGroup& t_sg)
{
  skeleton_msgs::SkeletonGroup sg;
  sg.src_time = ros::Time(t_sg.src_time);
  sg.src_frame = t_sg.src_frame;
  sg.skeletons.reserve(t_sg.skeletons.size());
  for (auto& s : t_sg.skeletons) {
    sg.skeletons.push_back(toMsg(s));
  }
  return sg;
}

skeleton_msgs::SkeletonGroup
hiros::skeletons::utils::toMsg(const std_msgs::Header& t_header,
                               const ros::Time& t_src_time,
                               const std::string& t_src_frame,
                               const hiros::skeletons::types::SkeletonGroup& t_sg)
{
  skeleton_msgs::SkeletonGroup sg;
  sg.header = t_header;
  sg.src_time = t_src_time;
  sg.src_frame = t_src_frame;
  sg.skeletons.reserve(t_sg.skeletons.size());
  for (auto& s : t_sg.skeletons) {
    sg.skeletons.push_back(toMsg(s));
  }
  return sg;
}

skeleton_msgs::SkeletonGroup
hiros::skeletons::utils::toMsg(const std_msgs::Header& t_header,
                               const ros::Time& t_src_time,
                               const hiros::skeletons::types::SkeletonGroup& t_sg)
{
  return toMsg(t_header, t_src_time, "", t_sg);
}

skeleton_msgs::SkeletonGroup
hiros::skeletons::utils::toMsg(const ros::Time& t_stamp,
                               const std::string& t_frame_id,
                               const ros::Time& t_src_time,
                               const std::string& t_src_frame,
                               const hiros::skeletons::types::SkeletonGroup& t_sg)
{
  skeleton_msgs::SkeletonGroup sg;
  sg.header.stamp = t_stamp;
  sg.header.frame_id = t_frame_id;
  sg.src_time = t_src_time;
  sg.src_frame = t_src_frame;
  sg.skeletons.reserve(t_sg.skeletons.size());
  for (auto& s : t_sg.skeletons) {
    sg.skeletons.push_back(toMsg(s));
  }
  return sg;
}

skeleton_msgs::SkeletonGroup
hiros::skeletons::utils::toMsg(const ros::Time& t_stamp,
                               const std::string& t_frame_id,
                               const ros::Time& t_src_time,
                               const hiros::skeletons::types::SkeletonGroup& t_sg)
{
  return toMsg(t_stamp, t_frame_id, t_src_time, "", t_sg);
}

skeleton_msgs::SkeletonGroup
hiros::skeletons::utils::toMsg(const unsigned int& t_seq,
                               const ros::Time& t_stamp,
                               const std::string& t_frame_id,
                               const ros::Time& t_src_time,
                               const std::string& t_src_frame,
                               const hiros::skeletons::types::SkeletonGroup& t_sg)
{
  skeleton_msgs::SkeletonGroup sg;
  sg.header.seq = t_seq;
  sg.header.stamp = t_stamp;
  sg.header.frame_id = t_frame_id;
  sg.src_time = t_src_time;
  sg.src_frame = t_src_frame;
  sg.skeletons.reserve(t_sg.skeletons.size());
  for (auto& s : t_sg.skeletons) {
    sg.skeletons.push_back(toMsg(s));
  }
  return sg;
}

skeleton_msgs::SkeletonGroup
hiros::skeletons::utils::toMsg(const unsigned int& t_seq,
                               const ros::Time& t_stamp,
                               const std::string& t_frame_id,
                               const ros::Time& t_src_time,
                               const hiros::skeletons::types::SkeletonGroup& t_sg)
{
  return toMsg(t_seq, t_stamp, t_frame_id, t_src_time, "", t_sg);
}
