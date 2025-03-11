# Launch localization
roslaunch localization gmapping.launch

# Go to maps folder
cd /home/rva_container/rva_exchange/rva_ws/src/localization/maps
# -f defines the name
rosrun map_server map_saver -f map


# Task 4
roslaunch localization amcl.launch
