ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {frame_id: 'map'},
  pose: {
    pose: {
      position: {x: 1.0, y: 1.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    },
    covariance: [
      0,0,0,0,0,0,
      0,0,0,0,0,0,
      0,0,0.0,0,0,0,
      0,0,0,0.0,0,0,
      0,0,0,0,0.0,0,
      0,0,0,0,0,0
    ]
  }
}