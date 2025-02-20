cd /home/rva_container/rva_exchange
mkdir -p rva_ws/src
cd rva_ws
catkin_make
source devel/setup.bash
cd src
catkin_create_pkg epd2 rospy tf geometry_msgs tf2_ros sensor_msgs
cd ..
catkin_make
source devel/setup.bash
cd src/epd2
mkdir scripts
mkdir launch
cd ../..
