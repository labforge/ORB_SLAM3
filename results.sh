#!/bin/sh
evo_ape tum Datasets/euroc/MH_01_easy/mav0/state_groundtruth_estimate0/data.tum logs/euroc/orb/traj/f_dataset-MH01_stereo_orb_clean.tum -a --save_results results/ape/euroc_orb.zip
evo_ape tum Datasets/euroc/MH_01_easy/mav0/state_groundtruth_estimate0/data.tum logs/euroc/akaze/traj/f_dataset-MH01_stereo_akaze_clean.tum -a --save_results results/ape/euroc_akaze.zip
evo_ape tum Datasets/tum/dataset-room1_512_16/mav0/mocap0/data_clean.tum logs/tum/orb/traj/f_dataset-room1_512_stereo_orb_clean.tum -a --save_results results/ape/tum_orb.zip
evo_ape tum Datasets/tum/dataset-room1_512_16/mav0/mocap0/data_clean.tum logs/tum/akaze/traj/f_dataset-room1_512_stereo_akaze_clean.tum -a --save_results results/ape/tum_akaze.zip
evo_ape kitti Datasets/kitti/data_odometry_poses/dataset/poses/00.txt logs/kitti/orb/traj/CameraTrajectory_orb.txt -a --save_results results/ape/kitti_orb.zip
evo_ape kitti Datasets/kitti/data_odometry_poses/dataset/poses/00.txt logs/kitti/akaze/traj/CameraTrajectory_akaze.txt -a --save_results results/ape/kitti_akaze.zip
evo_rpe tum Datasets/euroc/MH_01_easy/mav0/state_groundtruth_estimate0/data.tum logs/euroc/orb/traj/f_dataset-MH01_stereo_orb_clean.tum -a --pose_relation angle_deg --delta 1 --delta_unit m --save_results results/rpe/euroc_orb.zip
evo_rpe tum Datasets/euroc/MH_01_easy/mav0/state_groundtruth_estimate0/data.tum logs/euroc/akaze/traj/f_dataset-MH01_stereo_akaze_clean.tum -a --pose_relation angle_deg --delta 1 --delta_unit m --save_results results/rpe/euroc_akaze.zip
evo_rpe tum Datasets/tum/dataset-room1_512_16/mav0/mocap0/data_clean.tum logs/tum/orb/traj/f_dataset-room1_512_stereo_orb_clean.tum -a --pose_relation angle_deg --delta 1 --delta_unit m --save_results results/rpe/tum_orb.zip
evo_rpe tum Datasets/tum/dataset-room1_512_16/mav0/mocap0/data_clean.tum logs/tum/akaze/traj/f_dataset-room1_512_stereo_akaze_clean.tum -a --pose_relation angle_deg --delta 1 --delta_unit m --save_results results/rpe/tum_akaze.zip
evo_rpe kitti Datasets/kitti/data_odometry_poses/dataset/poses/00.txt logs/kitti/orb/traj/CameraTrajectory_orb.txt -a --pose_relation angle_deg --delta 1 --delta_unit m --save_results results/rpe/kitti_orb.zip
evo_rpe kitti Datasets/kitti/data_odometry_poses/dataset/poses/00.txt logs/kitti/akaze/traj/CameraTrajectory_akaze.txt -a --pose_relation angle_deg --delta 1 --delta_unit m --save_results results/rpe/kitti_akaze.zip


evo_res results/ape/euroc_orb.zip results/ape/euroc_akaze.zip --save_table results/ape/euroc_table.csv --save_plot results/ape/euroc_plot.pdf
evo_res results/ape/tum_orb.zip results/ape/tum_akaze.zip --save_table results/ape/tum_table.csv --save_plot results/ape/tum_plot.pdf
evo_res results/ape/kitti_orb.zip results/ape/kitti_akaze.zip --save_table results/ape/kitti_table.csv --save_plot results/ape/kitti_plot.pdf
evo_res results/rpe/euroc_orb.zip results/rpe/euroc_akaze.zip --save_table results/rpe/euroc_table.csv --save_plot results/rpe/euroc_plot.pdf
evo_res results/rpe/tum_orb.zip results/rpe/tum_akaze.zip --save_table results/rpe/tum_table.csv --save_plot results/rpe/tum_plot.pdf
evo_res results/rpe/kitti_orb.zip results/rpe/kitti_akaze.zip --save_table results/rpe/kitti_table.csv --save_plot results/rpe/kitti_plot.pdf
