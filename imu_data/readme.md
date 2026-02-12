VINS-Mono 的 exporter（vins_preint_pack.txt）里，compare_vinsmono_gtsam 实际只用这 6 个 block 名称：

dR_vins
dP_vins
dV_vins
DT_vins
Sigma_z_vins_gtsam
JincBias_ba_bg_vins
其它像 Sigma_vins_raw、dq_dbg_vins_raw、dphi_dbg_vins_* 都只是 debug，不参与 compare。

Sigma_z（15×15，z=[dphi,dp,dv,dba,dbg]）
JincBias（9×6，rows=[dphi,dp,dv]，cols=[dba,dbg]）




docker start vins_mono_kinetic
docker exec -it --user "$(id -u)":"$(id -g)" vins_mono_kinetic bash


source /opt/ros/kinetic/setup.bash
cd /catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source /catkin_ws/devel/setup.bash

rosrun vins_estimator export_vins_preint_pack --imu_txt /catkin_ws/src/VINS-Mono/imu_data/imu_data_Tangent_0.txt --config_yaml /catkin_ws/src/VINS-Mono/imu_data/cpc_config_Tangent_0.yaml --out_txt /catkin_ws/src/VINS-Mono/imu_data/vins_preint_pack.txt

[ OK ] Sigma_z (z=[dphi,dp,dv,dba,dbg])
[FAIL] JincBias_ba_bg (rows=[dphi,dp,dv]): max violation at (0,4)
  a=-0.0420799110401397503 b=-0.0483458147461485396 |a-b|=0.00626590370600878938 tol=0.000583458147461485429 (abs=0.000100000000000000005, rel=0.0100000000000000002)
  failing_entries_count=6
  (0,4) a=-0.0420799110401397503 b=-0.0483458147461485396 |a-b|=0.00626590370600878938 tol=0.000583458147461485429
  (0,5) a=0.103053291443817585 b=0.106660481471105462 |a-b|=0.00360719002728787652 tol=0.0011666048147110546
  (1,3) a=0.10241240813626673 b=0.105553449402744887 |a-b|=0.00314104126647815618 tol=0.00115553449402744889
  (1,5) a=-0.0433616337114337558 b=-0.0487368862597678393 |a-b|=0.0053752525483340835 tol=0.000587368862597678404
  (2,3) a=-0.105027047001499341 b=-0.110213665627921328 |a-b|=0.00518661862642198768 tol=0.0012021366562792134
  (2,4) a=0.0261059531097564967 b=0.0306810620263506077 |a-b|=0.00457510891659411098 tol=0.000406810620263506101
  to_pass_by_tuning_tolerance:
    rel_needed_if_abs_fixed=0.145858996430783178 (current_rel=0.0100000000000000002, abs_fixed=0.000100000000000000005)
    abs_needed_if_rel_fixed=0.00578244555854730367 (current_abs=0.000100000000000000005, rel_fixed=0.0100000000000000002)
export_vins_preint_pack failed: comparison failed

失败只在 JincBias_ba_bg 的 6 个元素，而且全在 (rows 0..2, cols 3..5)，即 dphi/dbg 这一个 3x3 子块。

```
root@liuyi:/ws# rosrun sliding_window_estimator compare_vinsmono_gtsam   --vins_all  /ws/src/swift_vio/imu_data/vins_preint_pack.txt   --gtsam_all /ws/src/swift_vio/imu_data/gtsam_ref_out/gtsam_ref_preint_all.txt
[ OK ] dR
[ OK ] dP
[ OK ] dV
[ OK ] DT
[ OK ] Sigma_z (z=[dphi,dp,dv,dba,dbg])
[ OK ] JincBias_ba_bg (rows=[dphi,dp,dv])
```

写代码过程中有过两次错误，修改后才可以，这里记录一下:
```
[ OK ] dR
[ OK ] dP
[ OK ] dV
[ OK ] DT
[FAIL] Sigma_z (z=[dphi,dp,dv,dba,dbg]): max violation at (3,3)
  a=3.28873077542375425 b=1157.9529903601383 |a-b|=1154.6642595847145 tol=17.3693948554020743 (abs=0.000100000000000000005, rel=0.0149999999999999994)
[FAIL] JincBias_ba_bg (rows=[dphi,dp,dv]): max violation at (0,4)
  a=-0.0420799110401397503 b=-0.0483458147461485396 |a-b|=0.00626590370600878938 tol=0.000825187221192228066 (abs=0.000100000000000000005, rel=0.0149999999999999994)
compare_vinsmono_gtsam failed: comparison failed
```
1. 均值是对的、噪声建模（协方差）是错的：

dR/dP/dV/DT 全部 [ OK ]：说明 IMU 积分流程、重力方向、bias 的使用方式基本没问题。
Sigma_z 差了 2~3 个数量级：典型是 VINS 的 IntegrationBase 用的是“离散步长的噪声注入形式”，而 cpc_config_Tangent_0.yaml 里的 sigma_*_c / sigma_*w_c 是“连续时间噪声密度”；如果不把密度换算到离散步长，协方差会按一个 dt 的因子缩小。
这个数据 dt=0.005，所以会出现接近 1/dt = 200 的倍率差（你报错里 Sigma(3,3) 的倍率更大，是因为测量噪声还叠加了 trapezoidal 两端点注入的因素）。
我已经把 exporter 按这个结论改了：

测量噪声（ACC_N/GYR_N）：用 sigma * sqrt(2/dt)（trapezoidal 两端点注入需要额外 sqrt(2)）
bias 随机游走（ACC_W/GYR_W）：用 sigma_rw * 1/sqrt(dt)（每步只注入一次，不乘 sqrt(2)）

```
[ OK ] dR
[ OK ] dP
[ OK ] dV
[ OK ] DT
[FAIL] Sigma_z (z=[dphi,dp,dv,dba,dbg]): max violation at (3,13)
  a=-0.150972577659511942 b=0.150583048542513948 |a-b|=0.301555626202025917 tol=0.00236458866489267904 (abs=0.000100000000000000005, rel=0.0149999999999999994)
[FAIL] JincBias_ba_bg (rows=[dphi,dp,dv]): max violation at (0,4)
  a=-0.0420799110401397503 b=-0.0483458147461485396 |a-b|=0.00626590370600878938 tol=0.000825187221192228066 (abs=0.000100000000000000005, rel=0.0149999999999999994)
compare_vinsmono_gtsam failed: comparison failed
```
2. Sigma_z 符号翻转（dp/dv/dphi 与 bias 的 cross-cov）
现在的失败点 (3,13)（dp_x vs dbg_y）是典型特征：数值量级对了、但符号反了。这说明 VINS IntegrationBase::covariance 里最后 6 维 bias 部分对应的是 bias increment = (b_j - b_i)；而我们统一的 GTSAM tangent z=[dphi,dp,dv,dba,dbg] 里 bias residual 用的是 (b_i - b_j)（也就是前面写的 r_b = b_s - b_e）。
所以把 dba, dbg 这 6 维在从 VINS cov 映射到 GTSAM Sigma_z 时乘上 -I，就会只翻转 cross-cov 的符号，bias-bias 的对角块不变，正好对齐。

JincBias 小的 off-diagonal 不够准（dphi/dbg）
失败的是 JincBias 的 (0,4) 这种小量（0.05 量级），abs+rel 容差会变得很严格。VINS 内部传播出来的 preint.jacobian(O_R,O_BG) 经过 Jr^{-1} 转成 dphi/dbg 后，对这些小 off-diagonal 会有系统性偏差（长时间段更明显）。工程上最稳的做法是：直接对 phi = Log(deltaR(bg)) 在 bg 处做中心差分，得到 dphi/dbg，这样就和 GTSAM tangent 的定义对齐。

已经把这两点改进落到 exporter 里了：

export_vins_preint_pack.cpp (line 305)：Sigma_z 映射里对 dba/dbg 加了 -I（修正 cross-cov 符号）。
export_vins_preint_pack.cpp (line 395)：JincBias 的 dphi/dbg 改为对 phi=Log(deltaR(bg)) 做中心差分（并额外输出了 dphi_dbg_vins_analytic / dphi_dbg_vins_fd 便于你看差异）。