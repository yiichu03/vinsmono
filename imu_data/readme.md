
docker start vins_mono_kinetic
docker exec -it --user "$(id -u)":"$(id -g)" vins_mono_kinetic bash


source /opt/ros/kinetic/setup.bash
cd /catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source /catkin_ws/devel/setup.bash

rosrun vins_estimator export_vins_preint_pack --imu_txt /catkin_ws/src/VINS-Mono/imu_data/imu_data_Tangent_0.txt --config_yaml /catkin_ws/src/VINS-Mono/imu_data/cpc_config_Tangent_0.yaml --out_txt /catkin_ws/src/VINS-Mono/imu_data/vins_preint_pack.txt

```
root@liuyi:/ws# rosrun sliding_window_estimator compare_vinsmono_gtsam   --vins_all  /ws/src/swift_vio/imu_data/vins_preint_pack.txt   --gtsam_all /ws/src/swift_vio/imu_data/gtsam_ref_out/gtsam_ref_preint_all.txt
[ OK ] dR
[ OK ] dP
[ OK ] dV
[ OK ] DT
[ OK ] Sigma_z (z=[dphi,dp,dv,dba,dbg])
[ OK ] JincBias_ba_bg (rows=[dphi,dp,dv])
```