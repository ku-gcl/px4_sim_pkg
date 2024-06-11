#!/bin/bash

# chmod +x launch_sitl.sh
# ./launch_sitl.sh


# ROSの環境設定を読み込む
source /opt/ros/noetic/setup.bash
source ~/ros-tutorial-ws/devel/setup.bash

# roscoreを別のターミナルで実行
gnome-terminal --tab -- bash -c "killall -9 gzserver; exec bash"
gnome-terminal --tab -- bash -c "gazebo --verbose worlds/iris_arducopter_runway.world; exec bash"
gnome-terminal --tab -- bash -c "cd ~/ardupilot/ArduCopter && sim_vehicle.py -f gazebo-iris --custom-location=33.595270,130.215496,13,353; exec bash"
sleep 10
gnome-terminal --tab -- bash -c "source ~/catkin_ws/devel/setup.bash && roslaunch px4_sim_pkg sitl.launch; exec bash"
gnome-terminal --tab -- bash -c "cd && source ~/catkin_ws/devel/setup.bash; exec bash"
