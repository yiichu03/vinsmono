# VINS-Mono 预积分导出与对比逻辑说明

本文说明两部分代码：

1. `vins_mono_kinetic/catkin_ws/src/VINS-Mono/vins_estimator/src/tools/export_vins_preint_pack.cpp`
2. `/home/liuyi/projects/project3_imu/JzHuai0108-swift_vio-2aa4b7d13701/sliding_window_estimator/src/apps/compare_vinsmono_gtsam.cpp`

目标是把 VINS 的预积分结果导出为可和 GTSAM 参考值直接对比的矩阵包，并能明确区分 `dphi/dbg` 采用“有限差分”还是“VINS 解析雅可比映射”。

---

## 1. `export_vins_preint_pack.cpp` 的完整流程

### 1.1 输入与配置读取

程序命令行参数：

- `--imu_txt`：IMU 序列文本，单行格式 `t wx wy wz ax ay az`
- `--config_yaml`：噪声、重力、bias、dt 等配置
- `--out_txt`：输出基准文件名（程序会基于它派生两个输出文件）

读取后会完成：

- IMU 文本解析为 `rows[k] = {t, w, a}`
- YAML 中读取：
  - `gravity`
  - `dt`（可选）
  - `sigma_g_c, sigma_a_c, sigma_gw_c, sigma_aw_c`
  - `biases.gyro, biases.accel`（可选）

### 1.2 噪声映射到 VINS 全局参数

核心映射（代码变量）：

- `G = -cfg.gravity`
- `dt_nominal = (cfg.dt > 0) ? cfg.dt : estimate_nominal_dt(rows)`
- `ACC_N, GYR_N, ACC_W, GYR_W`

其中：

- 测量噪声使用 `sqrt(2)/sqrt(dt)` 缩放（对应 VINS 梯形注入噪声的实现）
- bias random walk 使用 `1/sqrt(dt)` 缩放（每步注入一次）

### 1.3 VINS 预积分本体

使用 `IntegrationBase preint(...)`，逐段调用：

- `preint.push_back(dt, a_{k+1}, w_{k+1})`

最终得到 VINS 原生结果：

- `dR = preint.delta_q.toRotationMatrix()`
- `dP = preint.delta_p`
- `dV = preint.delta_v`
- `DT = preint.sum_dt`
- `preint.covariance`
- `preint.jacobian`

### 1.4 从 VINS 误差定义映射到 GTSAM Tangent 顺序

VINS 原生误差顺序：

- `[dp, dtheta, dv, dba, dbg]`

目标顺序：

- `z = [dphi, dp, dv, dba, dbg]`

步骤：

1. `phi_hat = Log(dR)`
2. 用 `Jr^{-1}(phi_hat)` 将旋转误差由 `dtheta -> dphi`
3. 通过矩阵 `A` 完成重排和 bias 增量号约定修正
4. 协方差映射：`Sigma_z_gtsam = A * preint.covariance * A^T`

其中 `A` 对 bias 部分使用 `-I`，用于把 VINS 的 bias increment 约定转成当前对比约定。

### 1.5 `dphi/dbg` 的两种计算路径

程序同时计算两种 `dphi/dbg`：

1. 解析映射（VINS 自身雅可比）：
   - `dq_dbg = preint.jacobian(O_R, O_BG)`（VINS 的 `dtheta/dbg`）
   - `dphi_dbg_analytic = Jr^{-1}(phi_hat) * dq_dbg`

2. 有限差分定义法：
   - 固定同一段 IMU 数据
   - 对 `bg` 三个分量做 `±eps` 扰动
   - 每次用 `repropagate(ba, bg)` 重传播
   - 计算 `phi = Log(deltaR(bg))`
   - 中心差分得到 `dphi_dbg_fd`

### 1.6 双文件输出（本次改动）

程序现在一次运行会输出两个 pack 文件：

- `<out_txt 基名>_fd.txt`：`JincBias` 中 `dphi/dbg` 使用有限差分 `dphi_dbg_fd`
- `<out_txt 基名>_analytic.txt`：`JincBias` 中 `dphi/dbg` 使用解析映射 `dphi_dbg_analytic`

两个文件都包含以下关键 block：

- `dR_vins, dP_vins, dV_vins, DT_vins`
- `Sigma_z_vins_gtsam`
- `JincBias_ba_bg_vins`

并且都保留 debug block：

- `Sigma_vins_raw`
- `dq_dbg_vins_raw`
- `dphi_dbg_vins_analytic`
- `dphi_dbg_vins_fd`
- `dphi_dbg_vins_used`（当前文件实际采用的版本）

文件头里增加了：

- `# dphi_dbg_source: finite_difference` 或 `analytic_from_vins_dtheta_dbg`

---

## 2. `compare_vinsmono_gtsam.cpp` 的对比逻辑

该程序的职责是：

- 读取 VINS pack 文件与 GTSAM 参考 pack 文件
- 逐矩阵做元素级绝对/相对容差比较
- 输出 `[ OK ]` 或 `[FAIL]` 并指出最大超差位置

### 2.1 读取机制

`readMatrixBlocks()` 解析格式：

- 头行：`name (RxC)`
- 后续 `R` 行每行 `C` 个数字
- 支持跳过空行和 `#` 注释行

读取后存到：

- `unordered_map<string, MatrixXd>`

### 2.2 强制比较的 6 个 block

程序固定取以下名字（名字必须匹配）：

1. `dR_vins` vs `dR_gtsam`
2. `dP_vins` vs `dP_gtsam`
3. `dV_vins` vs `dV_gtsam`
4. `DT_vins` vs `DT_gtsam`
5. `Sigma_z_vins_gtsam` vs `Sigma_z_gtsam`
6. `JincBias_ba_bg_vins` vs `JincBias_ba_bg_gtsam`

注意：`dphi_dbg_vins_analytic`、`dphi_dbg_vins_fd` 等 debug block 不参与 pass/fail 判定。

### 2.3 判定规则

每个元素比较阈值：

- `tol = absTol + relTol * max(|a|, |b|)`

默认参数：

- `absTol = 1e-4`
- `relTol = 1.5e-2`

若任一 block 超差，程序返回失败并打印最大超差元素坐标与值。

---

## 3. 如何用两份输出给老师提供证据

推荐同一组输入，分别比较两份 VINS pack 对 GTSAM 参考：

```bash
# 1) 生成两份 VINS pack
rosrun vins_estimator export_vins_preint_pack \
  --imu_txt /catkin_ws/src/VINS-Mono/imu_data/imu_data_Tangent_0.txt \
  --config_yaml /catkin_ws/src/VINS-Mono/imu_data/cpc_config_Tangent_0.yaml \
  --out_txt /catkin_ws/src/VINS-Mono/imu_data/vins_preint_pack.txt

# 实际会生成：
# /catkin_ws/src/VINS-Mono/imu_data/vins_preint_pack_fd.txt
# /catkin_ws/src/VINS-Mono/imu_data/vins_preint_pack_analytic.txt
```

```bash
# 2) 对比有限差分版
rosrun sliding_window_estimator compare_vinsmono_gtsam \
  --vins_all /catkin_ws/src/VINS-Mono/imu_data/vins_preint_pack_fd.txt \
  --gtsam_all /ws/src/swift_vio/imu_data/gtsam_ref_out/gtsam_ref_preint_all.txt

# 3) 对比解析版
rosrun sliding_window_estimator compare_vinsmono_gtsam \
  --vins_all /catkin_ws/src/VINS-Mono/imu_data/vins_preint_pack_analytic.txt \
  --gtsam_all /ws/src/swift_vio/imu_data/gtsam_ref_out/gtsam_ref_preint_all.txt
```

如果两次对比在 `JincBias_ba_bg` 上表现不同，而其余项（如 `dR/dP/dV/DT`、`Sigma_z`）一致，那么可以直接说明差异来自 `dphi/dbg` 计算路径，而不是积分主流程或噪声映射。

---

## 4. 你在答辩时可以直接用的结论框架

1. 我没有改 VINS 预积分主流程，`dR/dP/dV/DT` 都来自同一个 `IntegrationBase`。
2. 我只在导出阶段提供了两种 `dphi/dbg` 版本，用于验证“解析雅可比映射”与“定义法有限差分”的一致性。
3. 对比程序只看 6 个核心 block，因此结论可以精确归因到 `JincBias_ba_bg` 是否由 `dphi_dbg_fd` 或 `dphi_dbg_analytic` 驱动。
4. 这样可以把“实现正确性”和“坐标定义一致性”分离开来给证据，而不是混在一个结果里解释。
