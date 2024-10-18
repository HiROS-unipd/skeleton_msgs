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


## Citation
Please cite the following paper:
```
Guidolin, M., Tagliapietra, L., Menegatti, E., & Reggiani, M. (2023). Hi-ROS: Open-source multi-camera sensor fusion for real-time people tracking. Computer Vision and Image Understanding, 232, 103694.
```

Bib citation source:
```bibtex
@article{GUIDOLIN2023103694,
  title = {Hi-ROS: Open-source multi-camera sensor fusion for real-time people tracking},
  journal = {Computer Vision and Image Understanding},
  volume = {232},
  pages = {103694},
  year = {2023},
  issn = {1077-3142},
  doi = {https://doi.org/10.1016/j.cviu.2023.103694},
  url = {https://www.sciencedirect.com/science/article/pii/S1077314223000747},
  author = {Mattia Guidolin and Luca Tagliapietra and Emanuele Menegatti and Monica Reggiani},
  keywords = {Markerless motion capture, Multi-view body tracking, Real-time, ROS}
}
```
