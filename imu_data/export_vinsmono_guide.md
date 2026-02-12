# VINS-Mono `export_vins_preint_pack.cpp` 提炼说明

## 1) 导出逻辑（第一性原理）
- 目标：把 VINS 内部预积分结果映射到统一的 GTSAM 切空间残差定义。
- 核心等式：先得到均值增量 `dR,dP,dV,DT`，再映射协方差/雅可比到 `z=[dphi,dp,dv,dba,dbg]`。
- `dphi/dbg` 提供两种来源：  
  1) `analytic = Jr^{-1} * dtheta/dbg`（VINS内部解析）  
  2) `fd = d/d(bg) Log(deltaR(bg))`（有限差分）

### 核心代码
```cpp
IntegrationBase preint(rows.front().a, rows.front().w, cfg.bias_accel, cfg.bias_gyro);
for (size_t k = 0; k + 1 < rows.size(); ++k) {
  const double dt = rows[k + 1].t - rows[k].t;
  if (dt > 0.0) preint.push_back(dt, rows[k + 1].a, rows[k + 1].w);
}
const Eigen::Matrix3d dR = preint.delta_q.toRotationMatrix();
const Eigen::Vector3d dP = preint.delta_p, dV = preint.delta_v;
const Eigen::Matrix<double,15,15> Sigma_z_gtsam = A * preint.covariance * A.transpose();
```

```cpp
const Eigen::Matrix3d dphi_dbg_analytic = Jr_inv * dq_dbg;
for (int k = 0; k < 3; ++k) {
  bg_p(k) += eps_bg; bg_m(k) -= eps_bg;
  dphi_dbg_fd.col(k) = (phi_from_bias(ba, bg_p) - phi_from_bias(ba, bg_m)) / (2.0 * eps_bg);
}
```

```cpp
write_pack_txt(out_txt_fd, ..., dphi_dbg_fd, ..., JincBias_ba_bg_fd, jac_fd.J_e, jac_fd.J_s, "finite_difference");
write_pack_txt(out_txt_analytic, ..., dphi_dbg_analytic, ..., JincBias_ba_bg_analytic, jac_analytic.J_e, jac_analytic.J_s,
               "analytic_from_vins_dtheta_dbg");
```

## 2) 生成文件内容（`txt`）
- 输出文件：  
  - `*_fd.txt`：`dphi/dbg` 使用有限差分。  
  - `*_analytic.txt`：`dphi/dbg` 使用 VINS 解析传播。
- 两个文件都含核心块：  
  - `dR_vins`, `dP_vins`, `dV_vins`, `DT_vins`  
  - `Sigma_z_vins_gtsam`（15x15）  
  - `JincBias_ba_bg_vins`（9x6）  
  - `J_e_preint_vins`（15x15）  
  - `J_s_preint_vins`（15x15）
- 说明：为保持输出简洁，exporter 不再把 VINS 内部 raw/debug 块写入 `txt`，对比逻辑只依赖上述核心块与文件头 `# dphi_dbg_source:`。

---

## 3) 详细版（步骤-代码-公式，一步一块）

### Step 1：读取配置并做噪声离散化
```cpp
const double dt_nominal = (cfg.dt > 0.0) ? cfg.dt : estimate_nominal_dt(rows);
const double inv_sqrt_dt = 1.0 / std::sqrt(dt_nominal);
const double meas_scale = std::sqrt(2.0) * inv_sqrt_dt;
ACC_N = cfg.sigma_a_c * meas_scale;
GYR_N = cfg.sigma_g_c * meas_scale;
ACC_W = cfg.sigma_aw_c * inv_sqrt_dt;
GYR_W = cfg.sigma_gw_c * inv_sqrt_dt;
```
对应数学（连续噪声密度 \(\sigma_c\) 到离散标准差）：
$$
\sigma_{\text{meas,disc}} \approx \sqrt{2}\,\frac{\sigma_c}{\sqrt{\Delta t}},\qquad
\sigma_{\text{rw,disc}} \approx \frac{\sigma_{\text{rw},c}}{\sqrt{\Delta t}}.
$$

补充说明（你在 `readme.md` 里的那条记录，对应这里的代码动机）：
- 现象：`dR/dP/dV/DT` 全部对，但 `Sigma_z` 可能差 2 到 3 个数量级。
- 原因：`cpc_config_Tangent_0.yaml` 里的 `sigma_*_c` / `sigma_*w_c` 是连续时间白噪声密度；而 VINS `IntegrationBase` 需要你提供“离散步长上的噪声注入标准差”（也就是 `ACC_N/GYR_N/ACC_W/GYR_W` 这四个全局量）。
- 量级估算：若漏做密度到离散的换算，离散噪声会少一个约 $1/\sqrt{\Delta t}$ 因子，协方差量级随之少约 $1/\Delta t$。例如数据 `dt=0.005`，$1/\Delta t \approx 200$；测量噪声又叠加了梯形（两端点）注入的因素，因此某些对角线项会更夸张。
- 对应实现结论（和上面代码一致）：
  - 测量噪声（`ACC_N/GYR_N`）：`sigma * sqrt(2/dt)`
  - bias 随机游走（`ACC_W/GYR_W`）：`sigma_rw * 1/sqrt(dt)`

### Step 2：用 VINS 原生预积分得到均值与原生协方差
```cpp
IntegrationBase preint(rows.front().a, rows.front().w, cfg.bias_accel, cfg.bias_gyro);
for (...) preint.push_back(dt, rows[k + 1].a, rows[k + 1].w);
const Eigen::Matrix3d dR = preint.delta_q.toRotationMatrix();
const Eigen::Vector3d dP = preint.delta_p;
const Eigen::Vector3d dV = preint.delta_v;
```
对应数学：
$$
z_{\text{mean}} = \{\Delta R,\Delta p,\Delta v,\Delta t\},\qquad
\Sigma_{\text{vins}} = \mathrm{preint.covariance}.
$$

### Step 3：把 VINS 误差坐标映射到 GTSAM 切空间残差坐标
```cpp
const Eigen::Matrix<double, 15, 15> A = jac_vins_error_to_gtsam_tangent_z(phi_hat);
const Eigen::Matrix<double, 15, 15> Sigma_z_gtsam = A * preint.covariance * A.transpose();
```
对应关系：
$$
\delta\theta \approx J_r(\phi)\,\delta\phi
\;\Rightarrow\;
\delta\phi \approx J_r^{-1}(\phi)\,\delta\theta.
$$
并且 bias 残差符号统一为 \(r_b=b_s-b_e\)，因此有符号翻转块：
$$
[\delta b_a,\delta b_g]_{\text{target}} = -[\delta b_a,\delta b_g]_{\text{vins-increment}}.
$$
协方差映射：
$$
\Sigma_z = A\Sigma_{\text{vins}}A^\top.
$$

### Step 4：两种 \(d\phi/db_g\) 路径（解析 vs 有限差分）
```cpp
const Eigen::Matrix3d dphi_dbg_analytic = Jr_inv * dq_dbg;
for (int k = 0; k < 3; ++k) {
  dphi_dbg_fd.col(k) = (phi_from_bias(ba, bg_p) - phi_from_bias(ba, bg_m)) / (2.0 * eps_bg);
}
```
对应数学：
$$
\left.\frac{\partial \phi}{\partial b_g}\right|_{\text{analytic}}
\approx
J_r^{-1}(\phi)\left.\frac{\partial \theta}{\partial b_g}\right|_{\text{vins}},
$$
$$
\left.\frac{\partial \phi}{\partial b_{g,k}}\right|_{\text{FD}}
\approx
\frac{\log(\Delta R(b_g+\epsilon e_k))-\log(\Delta R(b_g-\epsilon e_k))}{2\epsilon}.
$$

### Step 5：构建 \(J_{\text{inc,bias}}\) 与因子雅可比 \(J_s,J_e\)
```cpp
const Eigen::Matrix<double, 9, 6> JincBias_ba_bg = build_jinc_bias(dphi_dbg);
const PreintFactorJacobians jac = build_preint_factor_jacobians_local(dR, dP, dV, DT, JincBias_ba_bg);
```
代码线性化模型：
$$
\delta x_e \approx F\delta x_s + G_9J_{\text{inc,bias}}\delta b + Gn.
$$
因此：
$$
J = F + \begin{bmatrix}G_9J_{\text{inc,bias}}\\0\end{bmatrix},\quad
J_e = G^{-1},\quad
J_s = -G^{-1}J.
$$

### Step 6：一次运行输出两份 `txt`
```cpp
write_pack_txt(out_txt_fd, ..., JincBias_ba_bg_fd, jac_fd.J_e, jac_fd.J_s, "finite_difference");
write_pack_txt(out_txt_analytic, ..., JincBias_ba_bg_analytic, jac_analytic.J_e, jac_analytic.J_s,
               "analytic_from_vins_dtheta_dbg");
```
含义：同一段 IMU 的其余量保持一致，仅 \(d\phi/db_g\) 路径不同，便于做“解析传播 vs 按定义差分”的可证据比较。

---

## 4) 通俗理解版本
- 这个 exporter 做的事可以理解成：先让 VINS 按它最擅长的方式把预积分算完，再把结果“翻译”到你统一的 GTSAM 语言里。
- `Sigma_z` 的映射就是“坐标系和符号对齐”；如果这一步不做，数值量级可能对了但正负号会错。
- `dphi/dbg` 你现在保留两条路：  
  - `analytic`：相信 VINS 内部传播。  
  - `fd`：不信内部推导，直接按定义扰动后做差分。  
  这样你能用实验结果向老师说明哪条更贴近目标定义。
- 最后一次运行出两份文件（`_fd` 和 `_analytic`），其他量保持一致，只改这一处，证据最干净。
