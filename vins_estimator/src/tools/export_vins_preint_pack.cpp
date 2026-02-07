#include <Eigen/Dense>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "../factor/integration_base.h"
#include "../parameters.h"

namespace {

struct ImuRow {
  double t = 0.0;
  Eigen::Vector3d w = Eigen::Vector3d::Zero();
  Eigen::Vector3d a = Eigen::Vector3d::Zero();
};

struct Config {
  Eigen::Vector3d gravity = Eigen::Vector3d(0, 0, -9.81);
  double dt = 0.0;
  double sigma_g_c = 0.0;
  double sigma_a_c = 0.0;
  double sigma_gw_c = 0.0;
  double sigma_aw_c = 0.0;
  Eigen::Vector3d bias_gyro = Eigen::Vector3d::Zero();
  Eigen::Vector3d bias_accel = Eigen::Vector3d::Zero();
};

static inline std::string trim(const std::string &s) {
  size_t b = 0;
  while (b < s.size() && std::isspace(static_cast<unsigned char>(s[b])))
    b++;
  size_t e = s.size();
  while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1])))
    e--;
  return s.substr(b, e - b);
}

static std::string slurp_file(const std::string &path) {
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    throw std::runtime_error("unable to open file: " + path);
  }
  std::stringstream buffer;
  buffer << ifs.rdbuf();
  return buffer.str();
}

static bool find_yaml_scalar_double(const std::string &content, const std::string &key, double &out) {
  const std::string needle = key + ":";
  size_t pos = content.find(needle);
  if (pos == std::string::npos)
    return false;
  pos += needle.size();
  while (pos < content.size() && std::isspace(static_cast<unsigned char>(content[pos])))
    pos++;
  size_t end = pos;
  while (end < content.size()) {
    const char c = content[end];
    if (std::isdigit(static_cast<unsigned char>(c)) || c == '+' || c == '-' || c == '.' || c == 'e' || c == 'E') {
      end++;
      continue;
    }
    break;
  }
  if (end == pos)
    return false;
  out = std::stod(content.substr(pos, end - pos));
  return true;
}

static bool find_yaml_inline_list(const std::string &content, const std::string &key, std::vector<double> &out) {
  const std::string needle = key + ":";
  size_t pos = content.find(needle);
  if (pos == std::string::npos)
    return false;
  pos = content.find('[', pos + needle.size());
  if (pos == std::string::npos)
    return false;
  size_t end = content.find(']', pos + 1);
  if (end == std::string::npos)
    return false;
  std::string inside = content.substr(pos + 1, end - (pos + 1));
  std::vector<double> vals;
  std::stringstream ss(inside);
  std::string tok;
  while (std::getline(ss, tok, ',')) {
    const std::string t = trim(tok);
    if (t.empty())
      continue;
    vals.push_back(std::stod(t));
  }
  out = std::move(vals);
  return !out.empty();
}

static bool find_yaml_inline_list_under_section(const std::string &content, const std::string &section, const std::string &key,
                                                std::vector<double> &out) {
  std::istringstream iss(content);
  std::string line;
  bool in_section = false;
  const std::string section_hdr = section + ":";
  const std::string key_hdr = key + ":";

  while (std::getline(iss, line)) {
    if (!line.empty() && line.back() == '\r')
      line.pop_back();

    size_t first = 0;
    while (first < line.size() && (line[first] == ' ' || line[first] == '\t'))
      first++;
    if (first == line.size())
      continue;

    if (!in_section) {
      if (first == 0 && line.compare(0, section_hdr.size(), section_hdr) == 0) {
        in_section = true;
      }
      continue;
    }

    if (first == 0) {
      break;
    }

    if (line.compare(first, key_hdr.size(), key_hdr) != 0) {
      continue;
    }

    size_t lb = line.find('[', first + key_hdr.size());
    if (lb == std::string::npos)
      return false;
    size_t rb = line.find(']', lb + 1);
    if (rb == std::string::npos)
      return false;
    std::string inside = line.substr(lb + 1, rb - (lb + 1));

    std::vector<double> vals;
    std::stringstream ss(inside);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
      const std::string t = trim(tok);
      if (t.empty())
        continue;
      vals.push_back(std::stod(t));
    }
    out = std::move(vals);
    return !out.empty();
  }
  return false;
}

static Eigen::Vector3d to_vec3(const std::vector<double> &v, const std::string &what) {
  if (v.size() != 3) {
    throw std::runtime_error("expected 3 elements for " + what);
  }
  return Eigen::Vector3d(v[0], v[1], v[2]);
}

static Config load_config_yaml(const std::string &path) {
  const std::string content = slurp_file(path);
  Config cfg;

  std::vector<double> g;
  if (!find_yaml_inline_list(content, "gravity", g) || g.size() != 3) {
    throw std::runtime_error("config_yaml missing 'gravity: [x,y,z]': " + path);
  }
  cfg.gravity = to_vec3(g, "gravity");

  // nominal dt (optional)
  (void)find_yaml_scalar_double(content, "dt", cfg.dt);

  if (!find_yaml_scalar_double(content, "sigma_g_c", cfg.sigma_g_c) || !find_yaml_scalar_double(content, "sigma_a_c", cfg.sigma_a_c) ||
      !find_yaml_scalar_double(content, "sigma_gw_c", cfg.sigma_gw_c) || !find_yaml_scalar_double(content, "sigma_aw_c", cfg.sigma_aw_c)) {
    throw std::runtime_error("config_yaml missing one of imu_params.sigma_*: " + path);
  }

  std::vector<double> bg, ba;
  if (find_yaml_inline_list_under_section(content, "biases", "gyro", bg) && bg.size() == 3) {
    cfg.bias_gyro = to_vec3(bg, "biases.gyro");
  }
  if (find_yaml_inline_list_under_section(content, "biases", "accel", ba) && ba.size() == 3) {
    cfg.bias_accel = to_vec3(ba, "biases.accel");
  }

  return cfg;
}

static std::vector<ImuRow> read_imu_txt(const std::string &path) {
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    throw std::runtime_error("unable to open imu_txt: " + path);
  }

  std::vector<ImuRow> rows;
  std::string line;
  while (std::getline(ifs, line)) {
    size_t i = 0;
    while (i < line.size() && std::isspace(static_cast<unsigned char>(line[i])))
      i++;
    if (i == line.size() || line[i] == '#')
      continue;

    std::istringstream iss(line);
    ImuRow r;
    double wx = 0.0, wy = 0.0, wz = 0.0, ax = 0.0, ay = 0.0, az = 0.0;
    if (!(iss >> r.t >> wx >> wy >> wz >> ax >> ay >> az)) {
      throw std::runtime_error("failed to parse imu row: " + line);
    }
    r.w = Eigen::Vector3d(wx, wy, wz);
    r.a = Eigen::Vector3d(ax, ay, az);
    rows.push_back(r);
  }

  if (rows.size() < 2) {
    throw std::runtime_error("need >=2 imu samples");
  }
  return rows;
}

static double estimate_nominal_dt(const std::vector<ImuRow> &rows) {
  std::vector<double> dts;
  dts.reserve(rows.size() > 1 ? (rows.size() - 1) : 0);
  for (size_t k = 0; k + 1 < rows.size(); ++k) {
    const double dt = rows[k + 1].t - rows[k].t;
    if (dt > 0.0) {
      dts.push_back(dt);
    }
  }
  if (dts.empty()) {
    throw std::runtime_error("unable to estimate dt (no positive time deltas)");
  }
  std::sort(dts.begin(), dts.end());
  return dts[dts.size() / 2];
}

template <int R, int C>
static void appendMatrixBlock(std::ostream &os, const std::string &name, const Eigen::Matrix<double, R, C> &mat) {
  os << name << " (" << R << "x" << C << ")\n";
  os.setf(std::ios::fixed);
  os << std::setprecision(18);
  for (int r = 0; r < R; ++r) {
    for (int c = 0; c < C; ++c) {
      os << mat(r, c);
      if (c < C - 1) {
        os << ' ';
      }
    }
    os << '\n';
  }
  os << '\n';
}

static inline Eigen::Matrix3d skew(const Eigen::Vector3d &v) {
  Eigen::Matrix3d S;
  S << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return S;
}

static inline double clamp(double x, double lo, double hi) {
  return std::max(lo, std::min(hi, x));
}

static Eigen::Vector3d so3_log(const Eigen::Matrix3d &R) {
  const double cos_theta = clamp((R.trace() - 1.0) * 0.5, -1.0, 1.0);
  const double theta = std::acos(cos_theta);
  Eigen::Vector3d vee;
  vee << (R(2, 1) - R(1, 2)), (R(0, 2) - R(2, 0)), (R(1, 0) - R(0, 1));
  if (theta < 1e-9) {
    return 0.5 * vee;
  }
  const double sin_theta = std::sin(theta);
  if (std::abs(sin_theta) < 1e-12) {
    return 0.5 * vee;
  }
  return (theta / (2.0 * sin_theta)) * vee;
}

static Eigen::Matrix3d so3_right_jacobian_inverse(const Eigen::Vector3d &phi) {
  const double theta = phi.norm();
  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d Phi = skew(phi);
  const Eigen::Matrix3d Phi2 = Phi * Phi;

  if (theta < 1e-8) {
    // Series: Jr^{-1} ≈ I + 0.5*Phi + 1/12*Phi^2
    return I + 0.5 * Phi + (1.0 / 12.0) * Phi2;
  }

  const double theta2 = theta * theta;
  const double half_theta = 0.5 * theta;
  const double cot_half_theta = std::cos(half_theta) / std::sin(half_theta);
  const double a = (1.0 / theta2) - (0.5 / theta) * cot_half_theta;
  return I + 0.5 * Phi + a * Phi2;
}

static Eigen::Matrix<double, 15, 15> jac_vins_error_to_gtsam_tangent_z(const Eigen::Vector3d &phi_hat) {
  Eigen::Matrix<double, 15, 15> A = Eigen::Matrix<double, 15, 15>::Zero();
  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  // VINS error:  [dp, dtheta, dv, dba, dbg]
  // GTSAM z:     [dphi, dp, dv, dba, dbg]  (TangentPreintegration uses additive dphi on phi_hat)
  // Relationship: dtheta ≈ Jr(phi_hat) * dphi  =>  dphi ≈ Jr^{-1}(phi_hat) * dtheta
  const Eigen::Matrix3d Jr_inv = so3_right_jacobian_inverse(phi_hat);
  A.block<3, 3>(0, 3) = Jr_inv; // dphi <- dtheta
  A.block<3, 3>(3, 0) = I;      // dp   <- dp
  A.block<3, 3>(6, 6) = I;      // dv   <- dv
  // IMPORTANT: VINS IntegrationBase covariance last 6 dims correspond to bias increments (b_j - b_i),
  // while our GTSAM tangent z uses (b_i - b_j) for the factor residual convention.
  // This only flips cross-covariance signs; bias-bias covariance stays the same.
  A.block<3, 3>(9, 9) = -I;      // dba  <- -dba
  A.block<3, 3>(12, 12) = -I;    // dbg  <- -dbg
  return A;
}

} // namespace

int main(int argc, char **argv) {
  try {
    std::string imu_txt;
    std::string config_yaml;
    std::string out_txt;

    for (int i = 1; i < argc; i++) {
      const std::string a = argv[i];
      auto next = [&]() -> std::string {
        if (i + 1 >= argc)
          throw std::runtime_error("missing value after " + a);
        return std::string(argv[++i]);
      };
      if (a == "--imu_txt") {
        imu_txt = next();
      } else if (a == "--config_yaml") {
        config_yaml = next();
      } else if (a == "--out_txt") {
        out_txt = next();
      } else if (a == "--help" || a == "-h") {
        std::cout << "usage: " << argv[0] << " --imu_txt <imu_data_Tangent_0.txt> --config_yaml <cpc_config_Tangent_0.yaml> --out_txt <out.txt>\n";
        return EXIT_SUCCESS;
      } else {
        throw std::runtime_error("unknown argument: " + a);
      }
    }

    if (imu_txt.empty() || config_yaml.empty() || out_txt.empty()) {
      std::cerr << "usage: " << argv[0] << " --imu_txt <imu_data_Tangent_0.txt> --config_yaml <cpc_config_Tangent_0.yaml> --out_txt <out.txt>\n";
      return EXIT_FAILURE;
    }

    const Config cfg = load_config_yaml(config_yaml);
    const std::vector<ImuRow> rows = read_imu_txt(imu_txt);

    // ---- Map swift_vio config into VINS globals ----
    // VINS uses G as "specific-force gravity" (i.e. -gravity accel), so that a stationary IMU measures +G.
    G = -cfg.gravity;
    const double dt_nominal = (cfg.dt > 0.0) ? cfg.dt : estimate_nominal_dt(rows);
    const double inv_sqrt_dt = 1.0 / std::sqrt(dt_nominal);
    // IntegrationBase uses trapezoidal (two endpoint) noise injections for accel/gyro meas noise.
    // For continuous-time white noise density (sigma_*_c), the discrete endpoint sample std is ~sigma/sqrt(dt),
    // and the trapezoidal rule underestimates the integral variance by ~2, so we apply an extra sqrt(2).
    const double meas_scale = std::sqrt(2.0) * inv_sqrt_dt;
    ACC_N = cfg.sigma_a_c * meas_scale;
    GYR_N = cfg.sigma_g_c * meas_scale;
    // Bias random-walk is injected once per step (no trapezoidal duplication), so no sqrt(2) factor.
    ACC_W = cfg.sigma_aw_c * inv_sqrt_dt;
    GYR_W = cfg.sigma_gw_c * inv_sqrt_dt;

    // ---- Preintegrate ----
    IntegrationBase preint(rows.front().a, rows.front().w, cfg.bias_accel, cfg.bias_gyro);
    for (size_t k = 0; k + 1 < rows.size(); ++k) {
      const double dt = rows[k + 1].t - rows[k].t;
      if (!(dt > 0.0)) {
        continue;
      }
      preint.push_back(dt, rows[k + 1].a, rows[k + 1].w);
    }

    const Eigen::Matrix3d dR = preint.delta_q.toRotationMatrix();
    const Eigen::Vector3d dP = preint.delta_p;
    const Eigen::Vector3d dV = preint.delta_v;
    const double DT = preint.sum_dt;

    // ---- Convert to GTSAM Tangent z order: [dphi, dp, dv, dba, dbg] ----
    const Eigen::Vector3d phi_hat = so3_log(dR);
    const Eigen::Matrix<double, 15, 15> A = jac_vins_error_to_gtsam_tangent_z(phi_hat);
    const Eigen::Matrix<double, 15, 15> Sigma_z_gtsam = A * preint.covariance * A.transpose();

    // Bias Jacobian: rows [dphi, dp, dv], cols [dba, dbg]
    Eigen::Matrix<double, 9, 6> JincBias_ba_bg;
    JincBias_ba_bg.setZero();
    const Eigen::Matrix3d dq_dbg = preint.jacobian.block<3, 3>(O_R, O_BG);
    const Eigen::Matrix3d dp_dba = preint.jacobian.block<3, 3>(O_P, O_BA);
    const Eigen::Matrix3d dp_dbg = preint.jacobian.block<3, 3>(O_P, O_BG);
    const Eigen::Matrix3d dv_dba = preint.jacobian.block<3, 3>(O_V, O_BA);
    const Eigen::Matrix3d dv_dbg = preint.jacobian.block<3, 3>(O_V, O_BG);
    const Eigen::Matrix3d Jr_inv = so3_right_jacobian_inverse(phi_hat);
    // VINS provides dtheta/dbg (right-multiplicative small angle). Convert to GTSAM Tangent dphi/dbg.
    const Eigen::Matrix3d dphi_dbg_analytic = Jr_inv * dq_dbg;

    // Empirically, VINS' closed-form propagation of dq_dbg can deviate from GTSAM Tangent's H_bg on small
    // off-diagonal terms over long intervals. To make the exporter match the GTSAM Tangent definition, we
    // compute dphi/dbg directly by finite-differencing phi = Log(deltaR(bg)) around the linearization bias.
    const double eps_bg = 1e-6;
    Eigen::Matrix3d dphi_dbg_fd = Eigen::Matrix3d::Zero();
    auto phi_from_bias = [&](const Eigen::Vector3d& ba, const Eigen::Vector3d& bg) -> Eigen::Vector3d {
      IntegrationBase tmp = preint;
      tmp.repropagate(ba, bg);
      return so3_log(tmp.delta_q.toRotationMatrix());
    };
    for (int k = 0; k < 3; ++k) {
      Eigen::Vector3d bg_p = cfg.bias_gyro;
      Eigen::Vector3d bg_m = cfg.bias_gyro;
      bg_p(k) += eps_bg;
      bg_m(k) -= eps_bg;
      const Eigen::Vector3d phi_p = phi_from_bias(cfg.bias_accel, bg_p);
      const Eigen::Vector3d phi_m = phi_from_bias(cfg.bias_accel, bg_m);
      dphi_dbg_fd.col(k) = (phi_p - phi_m) / (2.0 * eps_bg);
    }

    // Use finite-difference Jacobian for better agreement with GTSAM Tangent reference.
    JincBias_ba_bg.block<3, 3>(0, 3) = dphi_dbg_fd;
    JincBias_ba_bg.block<3, 3>(3, 0) = dp_dba;
    JincBias_ba_bg.block<3, 3>(3, 3) = dp_dbg;
    JincBias_ba_bg.block<3, 3>(6, 0) = dv_dba;
    JincBias_ba_bg.block<3, 3>(6, 3) = dv_dbg;

    // ---- Write pack txt ----
    std::ofstream ofs(out_txt, std::ios::trunc);
    if (!ofs.is_open()) {
      throw std::runtime_error("failed to open for writing: " + out_txt);
    }

    ofs << "# export_vins_preint_pack\n";
    ofs << "# imu_txt: " << imu_txt << "\n";
    ofs << "# config_yaml: " << config_yaml << "\n";
    ofs << std::setprecision(17) << "# interval: ts=" << rows.front().t << " te=" << rows.back().t << " DT=" << DT << "\n";
    ofs << std::setprecision(17) << "# dt_nominal: " << dt_nominal << "\n";
    ofs << std::setprecision(17) << "# vins_noise: ACC_N=" << ACC_N << " GYR_N=" << GYR_N << " ACC_W=" << ACC_W << " GYR_W=" << GYR_W << "\n";
    ofs << "# z_order: [dphi, dp, dv, dba, dbg]\n\n";

    appendMatrixBlock(ofs, "dR_vins", dR);
    appendMatrixBlock(ofs, "dP_vins", dP);
    appendMatrixBlock(ofs, "dV_vins", dV);
    Eigen::Matrix<double, 1, 1> DT_mat;
    DT_mat(0, 0) = DT;
    appendMatrixBlock(ofs, "DT_vins", DT_mat);

    // Raw (VINS native) blocks for debugging:
    // - state/error order: [dp, dtheta, dv, dba, dbg]
    appendMatrixBlock(ofs, "Sigma_vins_raw", preint.covariance);
    appendMatrixBlock(ofs, "dq_dbg_vins_raw", dq_dbg);
    appendMatrixBlock(ofs, "dphi_dbg_vins_analytic", dphi_dbg_analytic);
    appendMatrixBlock(ofs, "dphi_dbg_vins_fd", dphi_dbg_fd);

    appendMatrixBlock(ofs, "Sigma_z_vins_gtsam", Sigma_z_gtsam);
    appendMatrixBlock(ofs, "JincBias_ba_bg_vins", JincBias_ba_bg);

    std::cout << std::setprecision(17) << "integrated interval: ts=" << rows.front().t << " te=" << rows.back().t << " DT=" << DT << "\n";
    std::cout << "wrote: " << out_txt << "\n";
    return EXIT_SUCCESS;

  } catch (const std::exception &e) {
    std::cerr << "export_vins_preint_pack failed: " << e.what() << "\n";
    return EXIT_FAILURE;
  }
}
