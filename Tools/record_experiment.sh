#!/bin/bash
# Starting rosbag recording
# chmod +x record_experiment.sh

timestamp=$(date +"%Y-%m-%d-%H-%M-%S")
filename="experiment_$timestamp.bag"

dir=~/rosbag
mkdir -p $dir

cd $dir
rosbag record -a -O $filename
