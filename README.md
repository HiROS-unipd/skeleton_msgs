# Hi-ROS Skeleton Messages

ROS messages and related convenience utilities to handle human (or not) skeletons

```
MarkerSkeletonGroup
│
└── Header header
└── time src_time
└── string src_frame
│
└── MarkerSkeleton[] marker_skeletons
    │
    └── int32 id
    └── float64 confidence
    │
    └── MarkerGroup[] marker_groups
        │
        └── int32 id
        └── uint32 max_markers
        └── float64 confidence
        │
        └── Box bounding_box
        │   │
        │   └── Point center
        │   │   │
        │   │   └── geometry_msgs/Point position
        │   │   └── geometry_msgs/Vector3 velocity
        │   │   └── geometry_msgs/Vector3 acceleration
        │   │
        │   └── float64 length
        │   └── float64 height
        │   └── float64 width
        │   └── geometry_msgs/Quaternion orientation
        │
        └── Marker[] markers
            │
            └── int32 id
            └── float64 confidence
            │
            └── Point point
                │
                └── geometry_msgs/Point position
                └── geometry_msgs/Vector3 velocity
                └── geometry_msgs/Vector3 acceleration
```

```
OrientationSkeletonGroup
│
└── Header header
└── time src_time
└── string src_frame
│
└── OrientationSkeleton[] orientation_skeletons
    │
    └── int32 id
    └── float64 confidence
    │
    └── OrientationGroup[] orientation_groups
        │
        └── int32 id
        └── uint32 max_orientations
        └── float64 confidence
        │
        └── Orientation[] orientations
            │
            └── int32 id
            └── float64 confidence
            └── sensor_msgs/Imu orientation
```
