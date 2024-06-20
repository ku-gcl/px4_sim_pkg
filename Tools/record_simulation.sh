#!/bin/bash
# Starting rosbag recording
# chmod +x record_experiment.sh

timestamp=$(date +"%Y-%m-%d-%H-%M-%S")
filename="simulation_$timestamp.bag"

folder=$(date +"%Y-%m-%d")

dir=~/rosbag/$folder
mkdir -p $dir

cd $dir
rosbag record -a -O $filename
