# boeing

ray casting:

roslaunch touch_optimization particle_with_rays.launch'
rosrun gazebo_ray_trace gazebo_rayTrace_grid'

coverage path planning:

roslaunch coverage gazebo_wingbox.launch
roslaunch robot_control robot_control.launch
rosrun g2_control g2_control.node
rosrun rviz rviz
