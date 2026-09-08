# VINS 内部预积分修复与核对（2026-09-08）

## 修改范围

本版修改 `vins_estimator/src/factor/integration_base.h` 中实际被 VINS 主程序使用的 `midPointIntegration`。
exporter 直接读取 `preint.jacobian` 和 `preint.covariance`，只保留映射到 GTSAM Tangent 的坐标变换。前一轮原型中的外部旋转导数、输出子块替换及 `exact/native` 开关已经移除。

1. 四元数增量和下一时刻四元数都归一化，并在用它旋转加速度之前完成归一化。
2. 旋转状态转移和陀螺 bias 注入采用同一离散更新的精确解析导数。
3. 位置、速度对旋转和 bias 的耦合项按链式法则同步更新。
4. 两端陀螺噪声对旋转、位置、速度的注入同步更新，协方差继续用内部 `F P F^T + V Q V^T` 传播。
5. 输入数据、噪声密度转换、GTSAM reference 和原有 GTSAM 比较容差均不调整。

因此本版会影响位置/速度均值、完整 Jacobian 和协方差，不能再表述为“核心未改”。这是修改后的 VINS 实现通过验证，不代表未经修改的上游实现本身通过。

## 内部公式

```text
w = (gyro0 + gyro1)/2 - bg
a = w*dt/2
s = normalize([1, a])
q1 = normalize(q0*s)
D = R(s)^T
B = -dt/(1 + a^T*a) * (I - skew(a))
N = -B/2
H0 = -R(q0)*skew(acc0-ba)
H1 = -R(q1)*skew(acc1-ba)
```

旋转的 `F(R,R)=D`、`F(R,bg)=B`；每端陀螺噪声 `V(R,ng)=N`。
位置的相关导数为 `dt^2/4 * (H0 + H1*D)`、`dt^2/4 * H1*B`；
速度的相关导数为 `dt/2 * (H0 + H1*D)`、`dt/2 * H1*B`。
噪声对位置、速度的影响同样由 `H1*N` 传递。

状态误差顺序保持 `[dp,dtheta,dv,dba,dbg]`，旋转误差为右扰动。噪声顺序保持 `[na0,ng0,na1,ng1,nba,nbg]`。bias random walk 仍在每步末端注入，本次未改噪声独立性或连续到离散转换约定。

## 核对方法

`test_imu_preintegration.cpp` 对实际核心均值积分函数做独立数值扰动，检查：

- 3种单步运动 × 3种扰动步长，逐元素检查15x15状态 Jacobian `F` 和15x18噪声 Jacobian `V`。
- 使用数值 `F/V` 和非零初始协方差重建输出协方差，再与内部传播结果比较。
- 6种序列（零转动、bias抵消、极小角速度、大单步、10秒变轴、不等间隔）× 3种步长，检查全部9x6 bias Jacobian，包括位置、旋转、速度和两种 bias。
- 所有序列的协方差对称性、半正定性，以及 `repropagate` 重放的一致性。
- 现有 IMU 输入的完整9x6 Tangent bias Jacobian 数值检查；exporter 默认检查协方差对称性/半正定性。
- 原有内置及独立 GTSAM 比较器检查直接由内部导出的 `Sigma_z`、`JincBias`。

GTSAM 对比使用原来的 `abs_tol=1e-4`、`rel_tol=1.5e-2`。有限差分使用3种步长 `1e-5/1e-6/1e-7`，测试容差与 GTSAM 对比独立：单步 `F/V` 为 `2e-7 + 1e-6*scale`，完整积分 bias Jacobian 为 `1e-5 + 2e-6*scale`。有限差分只核验，不参与正式导出或核心计算。

## 已核对结果与版本

2026-09-08：完整 catkin 编译、内部单步/序列测试、固定 IMU 的有限差分与协方差检查、内置及独立 GTSAM 比较全部通过。测试日志保存在 [本次结果目录](validation_20260908_core)。例如原失败元素 `(0,4)` 的差值从约 `0.0062659` 降为 `0.0003141`，小于未修改的 GTSAM 比较阈值 `0.0008252`。

本文、核心修复和测试证据作为同一次交接修订保存。[GitHub](https://github.com/yiichu03/vinsmono) 的旧快照 `3962798` 仍是旧版超差结果，需使用后续核心修复提交复现本次结果。本次不包含 `J_s/J_e` 验收或完整 VIO 轨迹回归。

## 复现

```bash
source /opt/ros/kinetic/setup.bash
cd /catkin_ws
catkin_make -j2 -l2 -DCATKIN_WHITELIST_PACKAGES= -DCMAKE_BUILD_TYPE=Release
bash /catkin_ws/src/VINS-Mono/imu_data/run_validation.sh \
  /catkin_ws/devel/lib/vins_estimator
```

一键脚本会打印结果目录。单独运行 exporter 的参数仍为 `--imu_txt`、`--config_yaml`、`--out_txt`，可加 `--check_jacobian_fd`。

本次日志与内部导出矩阵保存在 `validation_20260908_core/`。独立 GTSAM 环境中，以 `DATA` 指向本目录：

```bash
rosrun sliding_window_estimator compare_vinsmono_gtsam \
  --vins_all "$DATA/validation_20260908_core/vins_preint_core.txt" \
  --gtsam_all "$DATA/gtsam_ref_out/gtsam_ref_preint_all.txt" \
  --abs_tol 1e-4 --rel_tol 1.5e-2
```

结论范围为内部预积分与上述数值测试；尚未进行完整相机/IMU数据集的 VIO 轨迹回归。不等采样间隔测试仅证明内部导数与离散传播一致，不等于重新验证了该情况下连续噪声离散化模型。
