ros2 service call /make_plan nav_msgs/srv/GetPlan "{
  start: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {w: 1.0}
    }
  },
  goal: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: 15.0, y: 3.0, z: 0.0},
      orientation: {w: 1.0}
    }
  }
}"
