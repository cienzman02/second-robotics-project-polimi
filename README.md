SECOND ROBOTICS PROJECT – Mapping & Navigation (Politecnico di Milano)

Author: Vincenzo Del Grosso
Package: second-robotics-project-polimi

---

TASK 1 – MAPPING
-----------------
- Use `mapping.launch` to:
  - Convert odometry to TF
  - Merge front & back laser scans
  - Run gmapping SLAM
  - Visualize in RViz

- Save final map with:
  rosrun map_server map_saver -f map

Files:
- map/map.pgm + map.yaml
- launch/mapping.launch
- src/odom_to_tf.cpp
- src/scan_merger.cpp
- rviz/mapping.rviz

---

TASK 2 – NAVIGATION
-------------------
- Use `navigation.launch` to:
  - Launch Stage simulation
  - Load saved map
  - Run AMCL for localization
  - Launch move_base
  - Start goal controller using ROS actions
  - Launch RViz for full visualization

Files:
- launch/navigation.launch
- launch/amcl.launch
- launch/move_base.launch
- config/*.yaml (navigation configs)
- worlds/navigation.world (Stage world)
- csv/goals.csv (x, y, theta goals)
- src/goal_publisher.cpp
- rviz/navigation.rviz

To test:
  roslaunch second-robotics-project-polimi navigation.launch

---

NOTES
-----
- Robot kinematics: Skid-steering
- Laser topic: /base_scan
- Simulated time enabled: use_sim_time=true
