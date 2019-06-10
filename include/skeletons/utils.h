#ifndef hiros_skeletons_utils_h
#define hiros_skeletons_utils_h

#include <geometry_msgs/Point.h>

#include "skeleton_msgs/Keypoint.h"
#include "skeleton_msgs/KeypointGroup.h"
#include "skeleton_msgs/Rectangle.h"
#include "skeleton_msgs/Skeleton.h"
#include "skeleton_msgs/SkeletonGroup.h"

#include "skeletons/types.h"

namespace hiros {
    namespace skeletons {
        namespace utils {

            // Point
            hiros::skeletons::types::Point
            toStruct(const double& t_x,
                     const double& t_y,
                     const double& t_z = std::numeric_limits<double>::quiet_NaN());

            hiros::skeletons::types::Point toStruct(const geometry_msgs::Point& t_p);

            geometry_msgs::Point toMsg(const hiros::skeletons::types::Point& t_p);

            // Rectangle
            hiros::skeletons::types::Rectangle toStruct(const double& t_x,
                                                        const double& t_y,
                                                        const double& t_width,
                                                        const double& t_height);

            hiros::skeletons::types::Rectangle toStruct(const skeleton_msgs::Rectangle& t_r);

            skeleton_msgs::Rectangle toMsg(const hiros::skeletons::types::Rectangle& t_r);

            // Keypoint
            hiros::skeletons::types::Keypoint
            toStruct(const unsigned& t_id,
                     const double& t_confidence,
                     const double& t_x,
                     const double& t_y,
                     const double& t_z = std::numeric_limits<double>::quiet_NaN());

            hiros::skeletons::types::Keypoint
            toStruct(const unsigned& t_id,
                     const double& t_confidence,
                     const hiros::skeletons::types::Point& t_point);

            hiros::skeletons::types::Keypoint toStruct(const skeleton_msgs::Keypoint& t_k);

            skeleton_msgs::Keypoint toMsg(const hiros::skeletons::types::Keypoint& t_k);

            // keypointGroup
            hiros::skeletons::types::KeypointGroup
            toStruct(const unsigned& t_id,
                     const double& t_confidence,
                     const std::vector<hiros::skeletons::types::Rectangle>& t_boundingBoxes,
                     const std::vector<hiros::skeletons::types::Keypoint> t_keypoints);

            hiros::skeletons::types::KeypointGroup
            toStruct(const skeleton_msgs::KeypointGroup& t_kg);

            skeleton_msgs::KeypointGroup toMsg(const hiros::skeletons::types::KeypointGroup& t_kg);

            // Skeleton
            hiros::skeletons::types::Skeleton
            toStruct(const unsigned& t_id,
                     const std::vector<hiros::skeletons::types::KeypointGroup>& t_skeletonParts);

            hiros::skeletons::types::Skeleton toStruct(const skeleton_msgs::Skeleton& t_s);

            skeleton_msgs::Skeleton toMsg(const hiros::skeletons::types::Skeleton& t_s);

            // SkeletonGroup
            hiros::skeletons::types::SkeletonGroup
            toStruct(const std::vector<hiros::skeletons::types::Skeleton>& t_skeletons);

            hiros::skeletons::types::SkeletonGroup
            toStruct(const skeleton_msgs::SkeletonGroup& t_sg);

            skeleton_msgs::SkeletonGroup toMsg(const hiros::skeletons::types::SkeletonGroup& t_sg);

            skeleton_msgs::SkeletonGroup toMsg(const unsigned& t_seq,
                                               const ros::Time& t_stamp,
                                               const std::string& t_frame_id,
                                               const ros::Time& t_srcTime,
                                               const hiros::skeletons::types::SkeletonGroup& t_sg);

        } // namespace utils
    } // namespace skeletons
} // namespace hiros
#endif
