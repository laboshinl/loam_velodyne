#! /bin/bash

DATA_DIR=${1:-/media/files/kitti/dataset_odometry_velodyne/sequences/train}
OUTPUT_DIR=${2:-/media/files/velodyne-odometry-results/55-LOAM-results-ref-5e9396}
POSES_DIR=${3:-/media/kitti/dataset_odometry_poses/poses/}
EMPTY_EVAL=${4:-/media/files/velodyne-odometry-results/empty-evaluation}
EVAL_TOOL=${5:-~/workspace/tmp-ivelas/sw/kitti-odometry-devkit/evaluate_odometry}



function filter_output {
	file=$1
	expected_lines=$2

	tmp_file=$(mktemp)
	grep '^\([[:digit:]e.+-]\+\s*\)\{12\}$' $file | tee $tmp_file
	found_lines=$(wc -l < $tmp_file)
	while [ $found_lines -lt $expected_lines ]
	do
		tail -n1 $tmp_file
		found_lines=$(($found_lines+1))
	done
	rm $tmp_file
}



roscore &
pid_core=$!
sleep 5s

mkdir -p $OUTPUT_DIR
for seq in $(ls $DATA_DIR)
do
	pushd $DATA_DIR/$seq/velodyne/
		roslaunch loam_velodyne loam_velodyne_noreg.launch rviz:=false > $OUTPUT_DIR/$seq.txt &
		pid_mapping=$!
		sleep 5s
		rosrun loam_velodyne scanRegistration $(ls *.bin | sort)
		kill $pid_mapping
		sleep 5s
	popd
done
kill $pid_core


cp -r $EMPTY_EVAL $OUTPUT_DIR/evaluation
for i in $OUTPUT_DIR/*.txt
do
	lines=$(wc -l < $POSES_DIR/$(basename $i))
	filter_output $i $lines > $OUTPUT_DIR/evaluation/results/ivelas/data/$(basename $i)
done
pushd $OUTPUT_DIR/evaluation
	$EVAL_TOOL ivelas
popd
