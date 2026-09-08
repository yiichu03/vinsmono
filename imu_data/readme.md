# VINS-Mono IMU 预积分验证交接

本目录提供同一段 IMU 输入下的 VINS-Mono 预积分结果，并按 GTSAM Tangent 误差状态定义检查协方差和 bias Jacobian。2026-09-08 的最新修复已落实到核心 `IntegrationBase`，导出工具直接读取内部 `jacobian` 和 `covariance`，只做必要的误差坐标转换。

## 导出与比较的数据块

`vins_preint_pack.txt` 导出：

- `dR_vins`、`dP_vins`、`dV_vins`、`DT_vins`
- `Sigma_z_vins_gtsam`：15x15，顺序为 `[dphi,dp,dv,dba,dbg]`
- `JincBias_ba_bg_vins`：9x6，行顺序为 `[dphi,dp,dv]`，列顺序为 `[dba,dbg]`

当前内置及独立比较器只检查 `Sigma_z_vins_gtsam` 和 `JincBias_ba_bg_vins`。均值增量也会导出，但不在这两个比较器的当前通过结论内。
文件头 `jacobian_source: IntegrationBase::jacobian` 标明结果来源。当前已移除 exporter 另算旋转导数和替换子块的实现。

## 当前可复现结论

2026-09-08 核心修复版：内置比较器与独立 `compare_vinsmono_gtsam` 对同一 IMU 输入和原有 GTSAM reference 均通过：

```text
[ OK ] Sigma_z (z=[dphi,dp,dv,dba,dbg])
[ OK ] JincBias_ba_bg (rows=[dphi,dp,dv])
```

- 容差保持为 `abs_tol=1e-4`、`rel_tol=1.5e-2`；通过只针对本目录固定输入与该容差。
- 原失败位置 `(0,4)` 的绝对差由 `0.0062659037` 降至 `0.0003140755`，小于容差 `0.0008251872`。
- 内部旋转递推采用归一化四元数的精确导数，并同步更新位置/速度耦合项和噪声注入矩阵。
- 下一时刻的四元数在旋转加速度之前归一化，使均值、Jacobian、协方差使用一致的旋转；因此位置/速度均值和协方差也会相应变化。
- 不再提供 `exact/native` 切换，也不在 exporter 中替换内部结果。旧版本 `3962798` 的6项超差和前一轮 exporter 原型结果仅作历史对照。

`--check_jacobian_fd` 用3种步长 `1e-5/1e-6/1e-7` 检查完整9x6 bias Jacobian，差分值仅用于核验。单元测试另检查15x15状态转移矩阵、15x18噪声矩阵、由数值导数组合得到的协方差、6种合成运动和重传播一致性。当前说明见 [内部修复记录](core_preintegration_fix.md)，日志见 [validation_20260908_core](validation_20260908_core)。[上一轮原型](OUTDATED_rotation_bias_fix.md) 及 `validation_20260908/` 为历史记录。

版本说明（2026-09-08）：本次交接修订包含核心修复、回归测试和新结果。[GitHub 仓库](https://github.com/yiichu03/vinsmono) 中旧快照 `3962798` 仍是原生 bias Jacobian 超差版本；复现本次通过结果须使用该快照之后的核心修复提交，不能只下载旧快照。

## 复现命令

```bash
docker start vins_mono_kinetic
docker exec -it --user "$(id -u)":"$(id -g)" vins_mono_kinetic bash

source /opt/ros/kinetic/setup.bash
cd /catkin_ws
catkin_make -j2 -l2 -DCATKIN_WHITELIST_PACKAGES= -DCMAKE_BUILD_TYPE=Release
source /catkin_ws/devel/setup.bash

rosrun vins_estimator export_vins_preint_pack \
  --imu_txt /catkin_ws/src/VINS-Mono/imu_data/imu_data_Tangent_0.txt \
  --config_yaml /catkin_ws/src/VINS-Mono/imu_data/cpc_config_Tangent_0.yaml \
  --out_txt /catkin_ws/src/VINS-Mono/imu_data/vins_preint_pack.txt \
  --check_jacobian_fd

# 内部矩阵检查 + 合成运动回归 + 直接导出的 GTSAM 比较。
bash /catkin_ws/src/VINS-Mono/imu_data/run_validation.sh /catkin_ws/devel/lib/vins_estimator
```

独立比较器位于 `vio_imu_process` 仓库的 `compare_vinsmono_gtsam.cpp`。本次使用同一 IMU 与配置的 GTSAM reference，检查内部结果直接导出的矩阵，得到 `Sigma_z` 和 `JincBias` 双 PASS。

## 已确认的现象

原版的旋转 Jacobian 使用一阶近似，且加速度旋转使用了尚未归一化的四元数。修复版使均值与完整内部误差传播一致。当前结论限于预积分层面的上述测试，不等于完整 VIO 轨迹精度已验证，也不保证所有输入与时长都满足同一 GTSAM 容差。
