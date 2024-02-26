#!/bin/bash
# Starting rosbag recording
# chmod +x record_experiment.sh

cd ~/rosbag
rosbag record -a -O test
