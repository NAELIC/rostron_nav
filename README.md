Publish odom base_link
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link

Publish map odom
ros2 run rostron_nav localisation

Launch all 
ros2 launch rostron_bringup yellow_sim.launch.py

grsim

launch conv files ! 
ros2 run rostron_nav conv

ros2 run tf2_ros tf2_echo map odom