#!/bin/bash

set -x

cp ~/ORB_SLAM3/f_dataset-corridor1_512_mono.txt .
cp ~/ORB_SLAM3/process_monitor.csv .
cp ~/ORB_SLAM3/ExecMean.txt .
cp ~/ORB_SLAM3/ttrack.csv .
evo_traj tum f_dataset-corridor1_512_mono.txt --save_as_tum
evo_ape tum ~/ORB_SLAM3/Datasets/tum/dataset-corridor1_512_16/mav0/mocap0/data_nocomma_clean.tum f_dataset-corridor1_512_mono.tum -vas > ape.txt
evo_rpe tum ~/ORB_SLAM3/Datasets/tum/dataset-corridor1_512_16/mav0/mocap0/data_nocomma_clean.tum f_dataset-corridor1_512_mono.tum -vas > rpe.txt

set +x
