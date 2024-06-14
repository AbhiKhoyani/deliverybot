# start Gazebo with plugins
gazebo -s libgazebo_ros_factory.so -s libgazebo_ros_init.so -s libgazebo_ros_state.so ../worlds/empty.world &
sleep 3

# spawn deliveryBot 
ros2 run gazebo_ros spawn_entity.py -file ../models/model.sdf -entity deliveryBot  -Y 0.733 -unpause
sleep 1

#spawn checkerboard
ros2 run gazebo_ros spawn_entity.py -file ../models/checkerboard/checkerboard_8x6_003.sdf  -entity checkerboard_left  -x 0.236 -y 0.03 -z 0.182 -P 1.57 -unpause
sleep 15

#run cameracalibrator
ros2 run camera_calibration cameracalibrator --no-service-check -p checkerboard --size 8x6 --square 0.03 --ros-args -r image:=/camera_left/image_raw -p camera:=/camera_left