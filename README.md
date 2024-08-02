# HUBO-PERCEPTION HRI LAB
### Running and use
To start, it is good to build and source, (colcon build and source install/setup.bash for ros2, catkin_make and source devel/setup.bash for ros-noetic)

##### To use with the openni features
launch the pointcloud_practice node with the test.launch.py on the main branch.

##### To use with the Multisense SLB: 
* run the Docker container from [https://github.com/MacksCohn/Docker_for_Multisense]
* make sure to run the ifconfig in the docker (changing the protocol to whatever the multisense shows up as) to change the ethernet port to have an ip the program is looking for
* Then, run the ros-noetic multisense bring up in multisense_ws and run the ros-to-ros2-pipe node with "ros2 run ros-to-ros2-pipe ros-to-ros2-pipe"
* Last, use ros2 launch pointcloud_practice multisense.launch.py

**MAKE SURE TO SET THE ROS DOMAIN FOR THE DOCKER AND NORMAL WORKSPACE TO BE THE SAME SO TOPICS ALL SHOW UP**

Presentation [https://docs.google.com/presentation/d/1UAIuAUh4PQTTA3J4ly86R8wnkd2VFnEHoXXQqt9aI5I]
