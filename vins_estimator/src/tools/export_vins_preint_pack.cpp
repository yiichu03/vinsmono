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
#include <unordered_map>
#include <utility>
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

static bool read_next_data_line(std::istream &is, std::string &out) {
  std::string line;
  while (std::getline(is, line)) {
    if (!line.empty() && line.back() == '\r')
      line.pop_back();
    const std::string t = trim(line);
    if (t.empty() || t[0] == '#')
      continue;
    out = t;
    return true;
  }
  return false;
}

static bool parse_header_line(const std::string &line, std::string &name, int &rows, int &cols) {
  const size_t lb = line.find('(');
  const size_t rb = line.find(')', lb == std::string::npos ? 0 : lb + 1);
  if (lb == std::string::npos || rb == std::string::npos || rb <= lb + 1)
    return false;
  name = trim(line.substr(0, lb));
  const std::string inside = trim(line.substr(lb + 1, rb - (lb + 1)));
  const size_t x = inside.find('x');
  if (x == std::string::npos)
    return false;
  rows = std::stoi(trim(inside.substr(0, x)));
  cols = std::stoi(trim(inside.substr(x + 1)));
  return !name.empty() && rows > 0 && cols > 0;
}

static std::unordered_map<std::string, Eigen::MatrixXd> read_matrix_blocks(const std::string &path) {
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    throw std::runtime_error("unable to open: " + path);
  }
  std::unordered_map<std::string, Eigen::MatrixXd> blocks;
  std::string line;
  while (read_next_data_line(ifs, line)) {
    std::string name;
    int rows = 0;
    int cols = 0;
    if (!parse_header_line(line, name, rows, cols)) {
      throw std::runtime_error("failed to parse matrix header line: '" + line + "' in " + path);
    }
    Eigen::MatrixXd mat(rows, cols);
    for (int r = 0; r < rows; ++r) {
      std::string row_line;
      if (!read_next_data_line(ifs, row_line)) {
        throw std::runtime_error("unexpected EOF reading matrix '" + name + "' from " + path);
      }
      std::istringstream iss(row_line);
      for (int c = 0; c < cols; ++c) {
        double v = 0.0;
        if (!(iss >> v)) {
          throw std::runtime_error("failed to parse matrix '" + name + "' row " + std::to_string(r) + " from " + path);
        }
        mat(r, c) = v;
      }
    }
    blocks[name] = std::move(mat);
  }
  return blocks;
}

static const Eigen::MatrixXd &get_block_or_throw(const std::unordered_map<std::string, Eigen::MatrixXd> &blocks, const std::string &name,
                                                 const std::string &path) {
  const auto it = blocks.find(name);
  if (it == blocks.end()) {
    throw std::runtime_error("missing block '" + name + "' in " + path);
  }
  return it->second;
}

static bool expect_near_abs_rel(const Eigen::MatrixXd &a, const Eigen::MatrixXd &b, double abs_tol, double rel_tol, const std::string &what) {
  if (a.rows() != b.rows() || a.cols() != b.cols()) {
    std::cerr << "[FAIL] " << what << ": shape mismatch: " << a.rows() << "x" << a.cols() << " vs " << b.rows() << "x" << b.cols() << "\n";
    return false;
  }

  double max_violation = -1.0;
  int max_r = 0;
  int max_c = 0;
  double max_diff = 0.0;
  double max_tol = 0.0;
  double max_a = 0.0;
  double max_b = 0.0;
  std::vector<std::pair<int, int>> fail_indices;
  std::vector<double> fail_a;
  std::vector<double> fail_b;
  std::vector<double> fail_diff;
  std::vector<double> fail_tol;

  double rel_needed_if_abs_fixed = 0.0;
  double abs_needed_if_rel_fixed = 0.0;

  for (int r = 0; r < a.rows(); ++r) {
    for (int c = 0; c < a.cols(); ++c) {
      const double va = a(r, c);
      const double vb = b(r, c);
      const double diff = std::abs(va - vb);
      const double scale = std::max(std::abs(va), std::abs(vb));
      const double tol = abs_tol + rel_tol * scale;
      const double violation = diff - tol;
      const double rel_needed_local = (diff > abs_tol && scale > 0.0) ? ((diff - abs_tol) / scale) : 0.0;
      const double abs_needed_local = std::max(0.0, diff - rel_tol * scale);
      rel_needed_if_abs_fixed = std::max(rel_needed_if_abs_fixed, rel_needed_local);
      abs_needed_if_rel_fixed = std::max(abs_needed_if_rel_fixed, abs_needed_local);
      if (violation > max_violation) {
        max_violation = violation;
        max_r = r;
        max_c = c;
        max_diff = diff;
        max_tol = tol;
        max_a = va;
        max_b = vb;
      }
      if (violation > 0.0) {
        fail_indices.emplace_back(r, c);
        fail_a.push_back(va);
        fail_b.push_back(vb);
        fail_diff.push_back(diff);
        fail_tol.push_back(tol);
      }
    }
  }

  if (max_violation > 0.0) {
    std::cerr << std::setprecision(18);
    std::cerr << "[FAIL] " << what << ": max violation at (" << max_r << "," << max_c << ")\n";
    std::cerr << "  a=" << max_a << " b=" << max_b << " |a-b|=" << max_diff << " tol=" << max_tol << " (abs=" << abs_tol
              << ", rel=" << rel_tol << ")\n";
    std::cerr << "  failing_entries_count=" << fail_indices.size() << "\n";
    for (size_t i = 0; i < fail_indices.size(); ++i) {
      std::cerr << "  (" << fail_indices[i].first << "," << fail_indices[i].second << ")"
                << " a=" << fail_a[i] << " b=" << fail_b[i] << " |a-b|=" << fail_diff[i] << " tol=" << fail_tol[i] << "\n";
    }
    std::cerr << "  to_pass_by_tuning_tolerance:\n";
    std::cerr << "    rel_needed_if_abs_fixed=" << rel_needed_if_abs_fixed << " (current_rel=" << rel_tol << ", abs_fixed=" << abs_tol
              << ")\n";
    std::cerr << "    abs_needed_if_rel_fixed=" << abs_needed_if_rel_fixed << " (current_abs=" << abs_tol << ", rel_fixed=" << rel_tol
              << ")\n";
    return false;
  }

  std::cout << "[ OK ] " << what << "\n";
  return true;
}

static std::string dirname_of(const std::string &path) {
  const size_t slash = path.find_last_of("/\\");
  if (slash == std::string::npos)
    return ".";
  if (slash == 0)
    return path.substr(0, 1);
  return path.substr(0, slash);
}

static std::string join_path(const std::string &dir, const std::string &name) {
  if (dir.empty())
    return name;
  const char last = dir.back();
  if (last == '/' || last == '\\')
    return dir + name;
  return dir + "/" + name;
}

static void compare_vinsmono_against_gtsam_ref(const std::string &config_yaml, const Eigen::Matrix<double, 15, 15> &Sigma_z_vins,
                                               const Eigen::Matrix<double, 9, 6> &JincBias_ba_bg_vins) {
  const std::string imu_data_dir = dirname_of(config_yaml);
  const std::string gtsam_all = join_path(join_path(imu_data_dir, "gtsam_ref_out"), "gtsam_ref_preint_all.txt");
  const auto gtsam = read_matrix_blocks(gtsam_all);
  const Eigen::MatrixXd &Sigma_z_gtsam = get_block_or_throw(gtsam, "Sigma_z_gtsam", gtsam_all);
  const Eigen::MatrixXd &JincBias_gtsam = get_block_or_throw(gtsam, "JincBias_ba_bg_gtsam", gtsam_all);

  constexpr double abs_tol = 1e-4; // 绝对误差底线。主要管接近 0 的元素（比如很多小的非对角项）。
  constexpr double rel_tol = 1e-2; // 相对误差比例。主要管大数值元素（比如大对角项），相当于允许百分比误差。
  bool ok = true;
  ok &= expect_near_abs_rel(Eigen::MatrixXd(Sigma_z_vins), Sigma_z_gtsam, abs_tol, rel_tol, "Sigma_z (z=[dphi,dp,dv,dba,dbg])");
  ok &= expect_near_abs_rel(Eigen::MatrixXd(JincBias_ba_bg_vins), JincBias_gtsam, abs_tol, rel_tol, "JincBias_ba_bg (rows=[dphi,dp,dv])");
  if (!ok) {
    throw std::runtime_error("comparison failed");
  }
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

static void write_pack_txt(const std::string &out_txt, const std::string &imu_txt, const std::string &config_yaml, const double ts,
                           const double te, const double DT, const double dt_nominal, const Eigen::Matrix3d &dR, const Eigen::Vector3d &dP,
                           const Eigen::Vector3d &dV, const Eigen::Matrix<double, 15, 15> &Sigma_z_gtsam,
                           const Eigen::Matrix<double, 9, 6> &JincBias_ba_bg) {
  std::ofstream ofs(out_txt, std::ios::trunc);
  if (!ofs.is_open()) {
    throw std::runtime_error("failed to open for writing: " + out_txt);
  }

  ofs << "# export_vins_preint_pack\n";
  ofs << "# imu_txt: " << imu_txt << "\n";
  ofs << "# config_yaml: " << config_yaml << "\n";
  ofs << std::setprecision(17) << "# interval: ts=" << ts << " te=" << te << " DT=" << DT << "\n";
  ofs << std::setprecision(17) << "# dt_nominal: " << dt_nominal << "\n";
  ofs << std::setprecision(17) << "# vins_noise: ACC_N=" << ACC_N << " GYR_N=" << GYR_N << " ACC_W=" << ACC_W << " GYR_W=" << GYR_W << "\n";
  ofs << "# z_order: [dphi, dp, dv, dba, dbg]\n\n";

  appendMatrixBlock(ofs, "dR_vins", dR);
  appendMatrixBlock(ofs, "dP_vins", dP);
  appendMatrixBlock(ofs, "dV_vins", dV);
  Eigen::Matrix<double, 1, 1> DT_mat;
  DT_mat(0, 0) = DT;
  appendMatrixBlock(ofs, "DT_vins", DT_mat);

  appendMatrixBlock(ofs, "Sigma_z_vins_gtsam", Sigma_z_gtsam);
  appendMatrixBlock(ofs, "JincBias_ba_bg_vins", JincBias_ba_bg);
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
    return I + 0.5 * Phi + (1.0 / 12.0) * Phi2;
  }

  const double theta2 = theta * theta;
  const double half_theta = 0.5 * theta;
  const double cot_half_theta = std::cos(half_theta) / std::sin(half_theta);
  const double a = (1.0 / theta2) - (0.5 / theta) * cot_half_theta;
  return I + 0.5 * Phi + a * Phi2;
}

static Eigen::Matrix3d theta_to_phi_jacobian_from_dR(const Eigen::Matrix3d &dR) {
  const Eigen::Vector3d phi_hat = so3_log(dR);
  return so3_right_jacobian_inverse(phi_hat);
}

static Eigen::Matrix<double, 15, 15> jac_vins_error_to_gtsam_tangent_z(const Eigen::Matrix3d &T_theta_to_phi) {
  Eigen::Matrix<double, 15, 15> A = Eigen::Matrix<double, 15, 15>::Zero();
  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  A.block<3, 3>(0, 3) = T_theta_to_phi; // dphi <- T_theta_to_phi * dtheta
  A.block<3, 3>(3, 0) = I;      // dp <- dp
  A.block<3, 3>(6, 6) = I;      // dv <- dv
  A.block<3, 3>(9, 9) = -I;     // dba <- -dba
  A.block<3, 3>(12, 12) = -I;   // dbg <- -dbg
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

    G = -cfg.gravity;
    const double dt_nominal = (cfg.dt > 0.0) ? cfg.dt : estimate_nominal_dt(rows);
    const double inv_sqrt_dt = 1.0 / std::sqrt(dt_nominal);

    ACC_N = cfg.sigma_a_c * inv_sqrt_dt;
    GYR_N = cfg.sigma_g_c * inv_sqrt_dt;
    const double acc_n_no_sqrt2 = ACC_N;
    const double gyr_n_no_sqrt2 = GYR_N;
    std::cout << std::setprecision(17) << "[debug] ACC_N no_sqrt2=" << acc_n_no_sqrt2 << "\n";
    std::cout << std::setprecision(17) << "[debug] GYR_N no_sqrt2=" << gyr_n_no_sqrt2 << "\n";
    ACC_N *= std::sqrt(2.0); // Comment these two lines to reproduce no-sqrt(2) behavior.
    GYR_N *= std::sqrt(2.0);
    ACC_W = cfg.sigma_aw_c * inv_sqrt_dt;
    GYR_W = cfg.sigma_gw_c * inv_sqrt_dt;
    std::cout << std::setprecision(17) << "[debug] ACC_N with_sqrt2=" << ACC_N << " delta=" << (ACC_N - acc_n_no_sqrt2) << "\n";
    std::cout << std::setprecision(17) << "[debug] GYR_N with_sqrt2=" << GYR_N << " delta=" << (GYR_N - gyr_n_no_sqrt2) << "\n";

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

    Eigen::Matrix3d T_theta_to_phi = Eigen::Matrix3d::Identity();
    // T_theta_to_phi = theta_to_phi_jacobian_from_dR(dR); // Comment this line to reproduce dphi == dtheta behavior.
    const Eigen::Matrix<double, 15, 15> A = jac_vins_error_to_gtsam_tangent_z(T_theta_to_phi);
    const Eigen::Matrix<double, 15, 15> Sigma_z_gtsam = A * preint.covariance * A.transpose();

    const Eigen::Matrix3d dq_dbg = preint.jacobian.block<3, 3>(O_R, O_BG);
    const Eigen::Matrix3d dp_dba = preint.jacobian.block<3, 3>(O_P, O_BA);
    const Eigen::Matrix3d dp_dbg = preint.jacobian.block<3, 3>(O_P, O_BG);
    const Eigen::Matrix3d dv_dba = preint.jacobian.block<3, 3>(O_V, O_BA);
    const Eigen::Matrix3d dv_dbg = preint.jacobian.block<3, 3>(O_V, O_BG);

    Eigen::Matrix<double, 9, 6> JincBias_ba_bg = Eigen::Matrix<double, 9, 6>::Zero();
    JincBias_ba_bg.block<3, 3>(0, 3) = T_theta_to_phi * dq_dbg;
    JincBias_ba_bg.block<3, 3>(3, 0) = dp_dba;
    JincBias_ba_bg.block<3, 3>(3, 3) = dp_dbg;
    JincBias_ba_bg.block<3, 3>(6, 0) = dv_dba;
    JincBias_ba_bg.block<3, 3>(6, 3) = dv_dbg;
    std::cout << "[debug] T_theta_to_phi * dq_dbg:\n" << (T_theta_to_phi * dq_dbg) << "\n[debug] dq_dbg:\n" << dq_dbg << "\n";

    write_pack_txt(out_txt, imu_txt, config_yaml, rows.front().t, rows.back().t, DT, dt_nominal, dR, dP, dV, Sigma_z_gtsam, JincBias_ba_bg);
    compare_vinsmono_against_gtsam_ref(config_yaml, Sigma_z_gtsam, JincBias_ba_bg);

    std::cout << std::setprecision(17) << "integrated interval: ts=" << rows.front().t << " te=" << rows.back().t << " DT=" << DT << "\n";
    std::cout << "wrote: " << out_txt << "\n";
    return EXIT_SUCCESS;

  } catch (const std::exception &e) {
    std::cerr << "export_vins_preint_pack failed: " << e.what() << "\n";
    return EXIT_FAILURE;
  }
}
