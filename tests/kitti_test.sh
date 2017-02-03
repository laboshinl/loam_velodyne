#! /bin/bash

DATA_DIR=/media/files/kitti/dataset_odometry_velodyne/sequences/train
OUTPUT_DIR=/media/files/velodyne-odometry-results/47-LOAM-results

roscore &
pid_core=$!

sleep 5s

mkdir -p $OUTPUT_DIR

for seq in $(ls $DATA_DIR)
do
	pushd $DATA_DIR/$seq/velodyne/
		roslaunch loam_velodyne loam_velodyne_noreg.launch > $OUTPUT_DIR/$seq.txt &
		pid_mapping=$!
		sleep 5s
		rosrun loam_velodyne scanRegistration $(ls *.bin | sort)
		kill $pid_mapping
		sleep 5s
	popd
done

kill $pid_core
