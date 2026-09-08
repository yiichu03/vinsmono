#include "../factor/integration_base.h"
#include <Eigen/Eigenvalues>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace {
using V15 = Eigen::Matrix<double, 15, 1>;
using V18 = Eigen::Matrix<double, 18, 1>;
using M15 = Eigen::Matrix<double, 15, 15>;
using M18 = Eigen::Matrix<double, 18, 18>;
using M15x18 = Eigen::Matrix<double, 15, 18>;
using M9x6 = Eigen::Matrix<double, 9, 6>;

struct State {
  Eigen::Vector3d p = Eigen::Vector3d::Zero(), v = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  Eigen::Vector3d ba = Eigen::Vector3d::Zero(), bg = Eigen::Vector3d::Zero();
};

Eigen::Vector3d logQuaternion(Eigen::Quaterniond q) {
  q.normalize();
  if (q.w() < 0.0) q.coeffs() *= -1.0;
  const double n = q.vec().norm();
  if (n < 1e-12) return 2.0 * q.vec();
  return (2.0 * std::atan2(n, q.w()) / n) * q.vec();
}

State perturb(State s, int k, double eps) {
  if (k < 3) s.p[k] += eps;
  else if (k < 6) s.q = s.q * Eigen::Quaterniond(Eigen::AngleAxisd(eps, Eigen::Vector3d::Unit(k - 3)));
  else if (k < 9) s.v[k - 6] += eps;
  else if (k < 12) s.ba[k - 9] += eps;
  else s.bg[k - 12] += eps;
  return s;
}

V15 difference(const State &s, const State &base) {
  V15 d;
  d << s.p - base.p, logQuaternion(base.q.conjugate() * s.q), s.v - base.v, s.ba - base.ba, s.bg - base.bg;
  return d;
}

bool near(const Eigen::MatrixXd &a, const Eigen::MatrixXd &b, double abs, double rel, const std::string &label) {
  if (a.rows() != b.rows() || a.cols() != b.cols() || !a.allFinite() || !b.allFinite()) {
    std::cerr << "[FAIL] " << label << ": shape/non-finite\n";
    return false;
  }
  for (int r = 0; r < a.rows(); ++r) for (int c = 0; c < a.cols(); ++c) {
    if (std::abs(a(r,c) - b(r,c)) > abs + rel * std::max(std::abs(a(r,c)), std::abs(b(r,c)))) {
      std::cerr << "[FAIL] " << label << " at (" << r << "," << c << ") analytic=" << a(r,c) << " fd=" << b(r,c) << "\n";
      return false;
    }
  }
  std::cout << "[ OK ] " << label << " max_abs=" << (a - b).cwiseAbs().maxCoeff() << "\n";
  return true;
}

bool covarianceValid(const M15 &cov, const std::string &label) {
  if (!cov.allFinite() || (cov - cov.transpose()).cwiseAbs().maxCoeff() > 1e-8) return false;
  Eigen::SelfAdjointEigenSolver<M15> eig(0.5 * (cov + cov.transpose()));
  if (eig.info() != Eigen::Success || eig.eigenvalues().minCoeff() < -1e-10) {
    std::cerr << "[FAIL] covariance PSD " << label << "\n";
    return false;
  }
  std::cout << "[ OK ] covariance symmetric/PSD " << label << "\n";
  return true;
}

State step(const State &s, double dt, const Eigen::Vector3d &w0, const Eigen::Vector3d &w1,
           const V18 &noise, IntegrationBase *derivatives = nullptr) {
  const Eigen::Vector3d a0(0.7, -0.3, 9.6), a1(-0.2, 0.8, 10.0);
  IntegrationBase scratch(a0, w0, s.ba, s.bg);
  IntegrationBase &p = derivatives ? *derivatives : scratch;
  State out;
  // Finite differences call the actual mean implementation with updates off.
  p.midPointIntegration(dt, a0 + noise.segment<3>(0), w0 + noise.segment<3>(3),
                        a1 + noise.segment<3>(6), w1 + noise.segment<3>(9),
                        s.p, s.q, s.v, s.ba, s.bg,
                        out.p, out.q, out.v, out.ba, out.bg, derivatives != nullptr);
  // Existing discrete random-walk model: bias noise is injected at step end.
  out.ba += dt * noise.segment<3>(12);
  out.bg += dt * noise.segment<3>(15);
  return out;
}

bool stepCase(const std::string &name, double dt, const Eigen::Vector3d &w0, const Eigen::Vector3d &w1) {
  State s;
  s.p << 1.0, -0.2, 0.4;
  s.v << -0.3, 0.5, 1.1;
  s.q = Eigen::Quaterniond(Eigen::AngleAxisd(0.6, Eigen::Vector3d(1,2,3).normalized()));
  s.ba << 0.03, -0.02, 0.01;
  s.bg << 0.1, -0.2, 0.05;
  IntegrationBase p(Eigen::Vector3d::Zero(), w0, s.ba, s.bg);
  const M15 prior = 0.03 * M15::Identity() + 0.001 * M15::Ones();
  p.covariance = prior;
  const M18 Q = p.noise;
  const State base = step(s, dt, w0, w1, V18::Zero(), &p);
  bool ok = true;
  for (double eps : {1e-5, 1e-6, 1e-7}) {
    M15 F;
    M15x18 V;
    for (int k = 0; k < 15; ++k)
      F.col(k) = (difference(step(perturb(s,k,eps), dt,w0,w1,V18::Zero()),base) -
                  difference(step(perturb(s,k,-eps),dt,w0,w1,V18::Zero()),base)) / (2.0*eps);
    for (int k = 0; k < 18; ++k) {
      V18 noise = V18::Zero(); noise[k] = eps;
      V.col(k) = (difference(step(s,dt,w0,w1,noise),base) - difference(step(s,dt,w0,w1,-noise),base)) / (2.0*eps);
    }
    ok &= near(p.step_jacobian, F, 2e-7, 1e-6, name + " F(15x15)");
    ok &= near(p.step_V, V, 2e-7, 1e-6, name + " V(15x18)");
    ok &= near(p.covariance, F*prior*F.transpose() + V*Q*V.transpose(), 1e-8, 1e-6, name + " covariance from numeric F/V");
  }
  ok &= covarianceValid(p.covariance, name);
  return ok;
}

M9x6 biasFiniteDifference(const IntegrationBase &p, double eps) {
  M9x6 fd;
  for (int k = 0; k < 6; ++k) {
    Eigen::Vector3d bap = p.linearized_ba, bam = bap, bgp = p.linearized_bg, bgm = bgp;
    if (k < 3) { bap[k] += eps; bam[k] -= eps; }
    else { bgp[k-3] += eps; bgm[k-3] -= eps; }
    IntegrationBase pp = p, pm = p;
    pp.repropagate(bap,bgp); pm.repropagate(bam,bgm);
    fd.block<3,1>(0,k) = (pp.delta_p - pm.delta_p)/(2.0*eps);
    fd.block<3,1>(3,k) = (logQuaternion(p.delta_q.conjugate()*pp.delta_q) - logQuaternion(p.delta_q.conjugate()*pm.delta_q))/(2.0*eps);
    fd.block<3,1>(6,k) = (pp.delta_v - pm.delta_v)/(2.0*eps);
  }
  return fd;
}

bool sequenceCase(const std::string &name, const std::vector<double> &dts,
                  const std::vector<Eigen::Vector3d> &gyros, const Eigen::Vector3d &bg) {
  const Eigen::Vector3d ba(0.03, -0.01, 0.02);
  IntegrationBase p(Eigen::Vector3d(0.3,-0.2,9.81), gyros.front(), ba, bg);
  for (size_t k = 0; k < dts.size(); ++k) {
    const double phase = 0.005 * k;
    p.push_back(dts[k], Eigen::Vector3d(0.3+std::sin(phase),-0.2+std::cos(phase),9.81), gyros[k+1]);
  }
  bool ok = true;
  for (double eps : {1e-5,1e-6,1e-7})
    ok &= near(p.jacobian.block<9,6>(0,9), biasFiniteDifference(p,eps), 1e-5, 2e-6, name + " internal bias J(9x6)");
  IntegrationBase replay = p;
  replay.repropagate(ba,bg);
  ok &= near(p.jacobian, replay.jacobian, 1e-12, 1e-12, name + " repropagate Jacobian");
  ok &= near(p.covariance, replay.covariance, 1e-12, 1e-12, name + " repropagate covariance");
  ok &= covarianceValid(p.covariance, name);
  return ok;
}
}  // namespace

int main() {
  ACC_N = GYR_N = 0.01;
  ACC_W = GYR_W = 0.001;
  bool ok = true;
  const Eigen::Vector3d bias(0.1,-0.2,0.05);
  ok &= stepCase("zero rotation step",0.005,bias,bias);
  ok &= stepCase("changing gyro step",0.005,Eigen::Vector3d(0.2,0.4,-0.1),Eigen::Vector3d(-0.3,0.7,0.2));
  ok &= stepCase("large rotation step",0.15,Eigen::Vector3d(4,-3,2),Eigen::Vector3d(5,1,-2));
  ok &= sequenceCase("zero gyro",std::vector<double>(100,0.005),std::vector<Eigen::Vector3d>(101,Eigen::Vector3d::Zero()),Eigen::Vector3d::Zero());
  ok &= sequenceCase("bias-cancelled",std::vector<double>(100,0.005),std::vector<Eigen::Vector3d>(101,bias),bias);
  ok &= sequenceCase("tiny gyro",std::vector<double>(100,0.005),std::vector<Eigen::Vector3d>(101,Eigen::Vector3d(1e-10,-2e-10,3e-10)),Eigen::Vector3d::Zero());
  ok &= sequenceCase("large single step",{0.15},{Eigen::Vector3d(4,-3,2),Eigen::Vector3d(5,1,-2)},bias);
  std::vector<double> dts;
  std::vector<Eigen::Vector3d> gyros;
  for (int k = 0; k <= 2000; ++k) {
    const double t = 0.005*k;
    gyros.emplace_back(0.7*std::sin(0.8*t)+bias.x(),0.6*std::cos(0.6*t)+bias.y(),0.3*std::sin(0.3*t)+bias.z());
    if (k < 2000) dts.push_back(0.005);
  }
  ok &= sequenceCase("10s changing axes",dts,gyros,bias);
  for (size_t k = 0; k < dts.size(); ++k) dts[k] = 0.002+0.001*(k%7);
  ok &= sequenceCase("irregular intervals",dts,gyros,bias);
  return ok ? 0 : 1;
}
