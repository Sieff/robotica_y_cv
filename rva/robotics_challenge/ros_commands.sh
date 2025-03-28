# Simulated challenge
rosrun robotics_challenge robotics_challenge.bash

# Go to maps folder
cd /home/rva_container/rva_exchange/rva_ws/src/robotics_challenge/maps
# -f defines the name
rosrun map_server map_saver -f map


# Evaluation
roslaunch robotics_challenge evaluation.launch
