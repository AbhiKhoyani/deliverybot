#!/usr/bin/bash
positions=('0.236 0.03 0.182 0 1.57 0' '0.8422 0.03 0.182 0 1.57 0' '0.199 0.030 0.157 0 1.57 0' '0.199 0.030 0.232 0 1.57 0' '0.791 0.52 0.222 0 1.57 0' '0.791 -0.52 0.222 0 1.57 0' '0.791 0.53 0.573 0 1.57 0' '0.671 -0.355 0.573 0 1.57 0' '0.236 0.03 0.182 0 1.01 0' '0.236 0.03 0.182 3.14 1.01 3.14') 
for p in "${positions[@]}"
do
   read -r x y z R P Y <<< "$p"
   # pause gazebo before calling set_entity_state service .  Apparently setting entity state doesn't work reliablity with gazebo in running mode
   ros2 service call /pause_physics 'std_srvs/Empty' {""}
   ros2 service call /set_entity_state 'gazebo_msgs/SetEntityState' "{state: { name: 'checkerboard_left', pose: {position: {x: $x, y: $y, z: $z}, orientation: {x: $R, y: $P, z: $Y}}, reference_frame: world}}"
   ros2 service call /unpause_physics 'std_srvs/Empty' {""}
   sleep .5
done