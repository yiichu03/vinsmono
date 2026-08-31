# VINS-Mono IMU 预积分验证交接

本目录提供同一段 IMU 输入下的 VINS-Mono 预积分结果，并按 GTSAM Tangent 误差状态定义检查协方差和 bias Jacobian。VINS-Mono 核心预积分实现未修改，新增内容仅位于验证 exporter、输入、参考数据和结果文件。

## 比较器使用的数据块

`vins_preint_pack.txt` 中参与比较的是：

- `dR_vins`、`dP_vins`、`dV_vins`、`DT_vins`
- `Sigma_z_vins_gtsam`：15x15，顺序为 `[dphi,dp,dv,dba,dbg]`
- `JincBias_ba_bg_vins`：9x6，行顺序为 `[dphi,dp,dv]`，列顺序为 `[dba,dbg]`

其他 `Sigma_vins_raw`、`dq_dbg_vins_raw` 等数据块仅用于调试。

## 当前可复现结论

2026-08-31 从当前源码强制重新编译并重新生成结果后，exporter 内置比较器与独立 `compare_vinsmono_gtsam` 得到一致结论：

```text
[ OK ] Sigma_z (z=[dphi,dp,dv,dba,dbg])
[FAIL] JincBias_ba_bg (rows=[dphi,dp,dv]): max violation at (0,4)
```

- 当前容差：`abs_tol=1e-4`、`rel_tol=1.5e-2`。
- 超差集中在 `dphi/dbg` 一个 3x3 子块，共 6 个元素。
- 最大超差位置 `(0,4)`：VINS 为 `-0.0420799110`，GTSAM 为 `-0.0483458147`，绝对差为 `0.0062659037`。
- exporter 会先正常写出结果文件，再因比较未全部通过返回非零退出码；这是当前验证现象，不是数据导出失败。

当前提交保留了解析 Jacobian 结果。源码中有限差分辅助函数仅供后续排查，默认未启用；本次交接不通过切换有限差分或继续放宽容差制造 PASS。

## 复现命令

```bash
docker start vins_mono_kinetic
docker exec -it --user "$(id -u)":"$(id -g)" vins_mono_kinetic bash

source /opt/ros/kinetic/setup.bash
cd /catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source /catkin_ws/devel/setup.bash

rosrun vins_estimator export_vins_preint_pack \
  --imu_txt /catkin_ws/src/VINS-Mono/imu_data/imu_data_Tangent_0.txt \
  --config_yaml /catkin_ws/src/VINS-Mono/imu_data/cpc_config_Tangent_0.yaml \
  --out_txt /catkin_ws/src/VINS-Mono/imu_data/vins_preint_pack.txt
```

独立比较器位于 `vio_imu_process` 仓库的 `compare_vinsmono_gtsam.cpp`。使用同一份 GTSAM reference 再运行一次，可复现相同的 `Sigma_z` PASS 与 `JincBias` FAIL。

## 已确认的现象

VINS 的离散噪声传播需要先把配置中的连续时间噪声密度换算到采样步长；协方差还需要把 VINS 的旋转误差和 bias 增量约定映射到统一的 GTSAM Tangent 定义。完成这些转换后，协方差已经对齐。当前保留的问题仅是长时间积分下，VINS 解析旋转 bias Jacobian 与 GTSAM 参考在若干小的非对角元素上存在系统偏差。
