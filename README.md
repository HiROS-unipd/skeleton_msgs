# Hi-ROS Skeleton Messages

ROS messages and related convenience utilities to handle human (or not) skeletons

```
SkeletonGroup
│
└── Header header (publication time, actual frame_id)
│
└── Skeleton[] skeletons
    │
    └── int32 id
    └── time src_time (timestamp of the input data)
    └── string src_frame (frame_id of the input data)
    └── uint32 max_markers
    └── uint32 max_links
    └── float64 confidence
    │
    └── Box bounding_box
    │   │
    │   └── KinematicState center
    │   │   │
    │   │   └── geometry_msgs/Pose pose
    │   │   └── geometry_msgs/Twist velocity
    │   │   └── geometry_msgs/Accel acceleration
    │   │
    │   └── float64 length
    │   └── float64 height
    │   └── float64 width
    │
    └── Marker[] markers
    │   │
    │   └── int32 id
    │   └── string name
    │   └── float64 confidence
    │   └── KinematicState center
    │
    └── Link[] links
        │
        └── int32 id
        └── string name
        └── int32 parent_marker
        └── int32 child_marker
        └── float64 confidence
        └── KinematicState center
```
