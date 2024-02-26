# Install

```
cd ~/catkin_ws/src
git clone https://github.com/ku-gcl/px4_sim_pkg.git
cd ~/catkin_ws
catkin_make
```

# 使用方法

## CMakeLists.txt の編集

px4_sim_pkg/CMakeLists.txt の最下行に以下を追加する．

```
add_executable(ardu_guided src/ardu_guided.cpp)
target_link_libraries(ardu_guided ${catkin_LIBRARIES})
```

## ビルド

```
cd ~/catkin_ws
catkin_make
```

## ノードの起動

```
roslaunch apm.launch fcu_url:=/dev/ttyS0:921600 gcs_url:=udp://:14550@192.168.xxx.xxx:14557
rosrun px4_sim_pkg ardu_guided
```

## Rosbag の開始

```
cd Tools
chmod 755 record_experiment.sh
source ./record_experiment.sh
```

# Reference

- https://docs.px4.io/main/en/ros/mavros_offboard_cpp.html
- [自作ノードを実行する（C++）](https://uenota.github.io/dronedoc/ja/runnode/runnodecpp.html)
- [ROS 講座 131 Ardupilot と ROS 経由で接続する](https://qiita.com/srs/items/09d217c8b9f9e21d2f1d)
