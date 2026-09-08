> OUTDATED（2026-09-08）：仅修 exporter 的第一轮原型，已由 [内部修复](core_preintegration_fix.md) 取代；不能据此声称当前核心未修改。

# 旋转 bias Jacobian 修复记录（2026-09-08）

> 历史记录：本文描述第一轮“仅修 exporter”的原型，已被 [核心内部修复](core_preintegration_fix.md) 取代。当前程序直接导出内部结果，本文的 `exact/native` 模式、辅助头文件和脚本行为不再适用于当前源码。旧运行日志保留在 `validation_20260908/`。

## 范围与结论

修复前版本 `3962798` 中，GTSAM 比较的协方差通过，bias Jacobian 有6个元素超差。
本次仅在验证 exporter 中补充 VINS 旋转积分的精确解析 bias 导数，并替换主输出 Jacobian 的 `dphi/dbg` 3x3 子块。
原生 `IntegrationBase` 不变；它的近似 Jacobian 通过 `native` 模式继续保留。
协方差、位置/速度的 bias Jacobian 仍取自原生实现，因此此处不是全量15维误差传播的重写。

## 原因及解析公式

`IntegrationBase::midPointIntegration` 的旋转更新为

```text
a = dt/2 * ((gyro_prev + gyro_next)/2 - bg)
s = normalize(Quaternion(1, a))
q_next = normalize(q * s)
```

令 `J` 为旋转关于陀螺 bias 的右扰动 Jacobian，即
`R(bg + db) ≈ R(bg) Exp(J db)`。
归一化四元数的微分满足

```text
2 vec(s^{-1} ds) = 2 (I - skew(a)) da / (1 + a^T a)
da/dbg = -dt/2 I
```

对两次旋转复合求导得到精确递推：

```text
J_next = R(s)^T J - dt/(1 + a^T a) * (I - skew(a))
J_initial = 0
```

原生递推用 `I - skew(omega)*dt` 近似 `R(s)^T`，bias 注入项用 `-dt*I`。
这种截断产生的偏差在当前约10秒积分窗口可见。
最终仍通过已有的 `Jr_inverse(Log(deltaR)) * J` 映射到 GTSAM Tangent 的 `dphi/dbg`。
实现位于 `vins_estimator/src/tools/rotation_bias_jacobian.h`。

这是对原始 VINS 离散旋转函数求导，不读取 GTSAM 数值参与计算，不使用有限差分作为正式输出，也未调整对比容差。

## 核验记录

- 6种合成运动：零旋转、bias抵消后的零旋转、极小旋转、较大单步旋转、10秒变轴旋转、不等采样间隔。全部通过。
- 每种运动用3个 bias 扰动步长 `1e-5/1e-6/1e-7`，重新调用实际 `IntegrationBase::repropagate` 做独立有限差分。所有合成样例的最大绝对差小于 `3e-8`。
- 原始 IMU 段的 Tangent `dphi/dbg` 也通过上述3步长核验，使用 `abs=1e-6, rel=1e-6`。
- 内置比较：`exact` 模式的 `Sigma_z`、`JincBias` 均通过。
- 从同一输入重新生成 GTSAM reference，独立 `compare_vinsmono_gtsam` 再次双通过。
- `native` 模式仍报告原始6项超差、退出码1，证明原始行为可复现。
- GTSAM 比较容差始终为逐元素 `1e-4 + 0.015 * max(abs(a), abs(b))`。

原最大超差位置（零基索引）`JincBias(0,4)`：

| 项目 | 数值 |
| --- | ---: |
| 原生近似 | -0.04207991104013975 |
| exporter 精确解析导数 | -0.04803173921676196 |
| GTSAM reference | -0.04834581474614854 |
| 修复前绝对差 | 0.00626590370600879 |
| 修复后绝对差 | 0.00031407552938658 |
| 容差 | 0.00082518722119223 |

当前独立比较器只检查协方差和 bias Jacobian；以上结果不等于完整 VIO 轨迹评测，也不保证任意积分时长和任意旋转都通过同一 GTSAM 容差。SO(3) Log 在主值分支边界附近的可微性不在本次范围内。

## 复现与文件

VINS 容器内完成编译后：

```bash
bash /catkin_ws/src/VINS-Mono/imu_data/run_validation.sh \
  /catkin_ws/devel/lib/vins_estimator
```

可选第二个参数指定输出目录；不传时创建临时目录。
脚本返回0表示精确导数检查通过，并且原生模式按预期复现6项超差。

独立 GTSAM 环境中（设置 `DATA` 为该环境能访问的 VINS `imu_data` 目录）：

```bash
rosrun sliding_window_estimator gtsam_ref_preint_from_txt \
  "$DATA/imu_data_Tangent_0.txt" "$DATA/cpc_config_Tangent_0.yaml" "$DATA/validation_20260908/gtsam_ref_out"
rosrun sliding_window_estimator compare_vinsmono_gtsam \
  --vins_all "$DATA/validation_20260908/vins_preint_exact.txt" \
  --gtsam_all "$DATA/validation_20260908/gtsam_ref_out/gtsam_ref_preint_all.txt" \
  --abs_tol 1e-4 --rel_tol 1.5e-2
```

本次运行保存在 `validation_20260908/`：`rotation_cases.log`、`exact.log`、`native.log`、`independent_compare.log`、两个模式的矩阵包和重新生成的 GTSAM reference。
