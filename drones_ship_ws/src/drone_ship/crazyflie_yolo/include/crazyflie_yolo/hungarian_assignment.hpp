#pragma once

#include <vector>
#include <limits>
#include <optional>
#include <Eigen/Dense>

/**
 * Hungarian assignment tailored to Sec. "Decentralized Fusion and Task Allocation"
 * of the paper: cost combines D-optimality (log det P), travel distance, and separation penalty.
 *
 * We solve a rectangular linear assignment: M drones -> N obstacles (M <= N allowed, or vice-versa).
 * Unassigned rows/cols are handled by padding with dummy columns/rows with cost = UNASSIGNED_PENALTY.
 *
 * Author: ChatGPT (GPT-5 Thinking)
 * License: MIT
 */

namespace mr_assign {

// ---------------- Data Types ----------------

struct Drone {
  Eigen::Vector3d position; // r_j in ENU
};

struct Track {
  Eigen::Vector3d mean;     // \hat{p}_i in ENU
  Eigen::Matrix3d covariance; // P_i (3x3, SPD)
  // Optionally store R for gating; if not set, Mahalanobis gate is skipped.
  std::optional<Eigen::Matrix3d> measR;
};

struct Params {
  double alpha = 1.0;     // weight on logdet(P)
  double beta  = 1.0;     // weight on travel distance
  double gamma = 0.2;     // weight on separation penalty
  double r_safe = 0.5;    // separation buffer [m]
  double d_max = 25.0;    // max travel allowable [m] (soft gate)
  double maha_gate_sq = 9.0; // chi^2 gate for 3D (approx 0.997 for 3 dof)
  double kappa = 1e3;     // penalty added when a gate fails (effectively blocks assignment)
  double unassigned_penalty = 25.0; // cost to leave a drone or target unassigned (tune as needed)
};

/**
 * Soft barrier function phi(Delta):
 *   0                    , Delta >= 0
 *   (1 + Delta/r_safe)^-2 - 1 , Delta < 0
 * Delta = (min inter-drone separation - r_safe)
 */
inline double separation_barrier(double delta, double r_safe) {
  if (delta >= 0.0) return 0.0;
  double t = 1.0 + (delta / r_safe);
  // prevent division by zero if delta ~ -r_safe
  if (t <= 1e-6) t = 1e-6;
  return 1.0 / (t * t) - 1.0;
}

// Utility: log(det(P)) robustly for SPD matrix.
inline double logdet_spd(const Eigen::Matrix3d& P) {
  Eigen::LLT<Eigen::Matrix3d> llt(P);
  if (llt.info() != Eigen::Success) {
    // Fallback: add small jitter and retry
    Eigen::Matrix3d Pj = P + 1e-9 * Eigen::Matrix3d::Identity();
    Eigen::LLT<Eigen::Matrix3d> llt2(Pj);
    if (llt2.info() != Eigen::Success) return 1e6; // huge if not SPD
    const auto& L = llt2.matrixL();
    double logdet = 0.0;
    for (int i = 0; i < 3; ++i) logdet += std::log(std::max(1e-12, (double)L(i,i)));
    return 2.0 * logdet;
  }
  const auto& L = llt.matrixL();
  double logdet = 0.0;
  for (int i = 0; i < 3; ++i) logdet += std::log(std::max(1e-12, (double)L(i,i)));
  return 2.0 * logdet;
}

// Mahalanobis distance squared: (z - Hx)^T S^{-1} (z - Hx), here H = I, S = P + R
inline double mahalanobis_sq(const Eigen::Vector3d& z, const Eigen::Vector3d& x,
                             const Eigen::Matrix3d& P, const std::optional<Eigen::Matrix3d>& R_opt) {
  Eigen::Matrix3d S = P;
  if (R_opt.has_value()) S += *R_opt;
  Eigen::LLT<Eigen::Matrix3d> llt(S);
  if (llt.info() != Eigen::Success) {
    // add jitter
    Eigen::Matrix3d Sj = S + 1e-9 * Eigen::Matrix3d::Identity();
    Eigen::LLT<Eigen::Matrix3d> llt2(Sj);
    if (llt2.info() != Eigen::Success) return std::numeric_limits<double>::infinity();
    Eigen::Vector3d y = llt2.matrixL().solve(z - x);
    return y.squaredNorm();
  }
  Eigen::Vector3d y = llt.matrixL().solve(z - x);
  return y.squaredNorm();
}

// Build rectangular cost matrix with gating penalties.
// Also handles pairwise separation term by penalizing being near other drones' current positions.
inline Eigen::MatrixXd build_cost_matrix(const std::vector<Drone>& drones,
                                         const std::vector<Track>& tracks,
                                         const Params& params) {
  const int M = (int)drones.size();
  const int N = (int)tracks.size();
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(M, N);

  // Precompute D-optimality term per track: log det P_i
  std::vector<double> logdetP(N, 0.0);
  for (int i = 0; i < N; ++i) {
    logdetP[i] = logdet_spd(tracks[i].covariance);
  }

  // For each drone j and track i, compute cost
  for (int j = 0; j < M; ++j) {
    const Eigen::Vector3d rj = drones[j].position;
    // precompute min separation to other drones (for the barrier)
    double min_sep = std::numeric_limits<double>::infinity();
    for (int k = 0; k < M; ++k) {
      if (k == j) continue;
      double d = (rj - drones[k].position).norm();
      if (d < min_sep) min_sep = d;
    }
    if (M == 1) min_sep = std::numeric_limits<double>::infinity(); // no separation penalty with single drone
    double delta = min_sep - params.r_safe;
    double sep_pen = separation_barrier(delta, params.r_safe); // >= 0

    for (int i = 0; i < N; ++i) {
      const Eigen::Vector3d pi = tracks[i].mean;
      const Eigen::Matrix3d& Pi = tracks[i].covariance;

      double travel = (pi - rj).norm();
      double base_cost = params.alpha * logdetP[i] + params.beta * travel + params.gamma * sep_pen;

      // Gates
      double cost = base_cost;
      if (travel > params.d_max) {
        cost += params.kappa; // too far
      }
      double d2 = mahalanobis_sq(pi, pi, Pi, tracks[i].measR); // here z=x (consistency gate on P and R)
      // If you want gate w.r.t. expected measurement from this drone, replace the second 'pi' with a predicted Hx.
      if (std::isfinite(d2) && d2 > params.maha_gate_sq) {
        cost += params.kappa;
      }

      C(j, i) = cost;
    }
  }
  return C;
}

// ---------------- Hungarian (Munkres) ----------------
// This is a classic O(n^3) implementation adapted to rectangular matrices by padding.

class Hungarian {
public:
  // Solve the assignment for cost matrix costMx (MxN). Returns pairs (row -> col).
  // If rows != cols, the matrix is padded with dummy rows/cols with UNASSIGNED_PENALTY.
  // The solution returns only assignments where row < M and col < N. Others are dummy.
  static std::vector<std::pair<int,int>> solve(const Eigen::MatrixXd& costMx, double unassigned_penalty) {
    int M = (int)costMx.rows();
    int N = (int)costMx.cols();
    int n = std::max(M, N);

    // Build square matrix padded with unassigned_penalty
    Eigen::MatrixXd A = Eigen::MatrixXd::Constant(n, n, unassigned_penalty);
    A.block(0, 0, M, N) = costMx;

    // Munkres algorithm
    // Step 1: Row reduction
    for (int i = 0; i < n; ++i) {
      double minv = A.row(i).minCoeff();
      A.row(i).array() -= minv;
    }
    // Step 2: Column reduction
    for (int j = 0; j < n; ++j) {
      double minv = A.col(j).minCoeff();
      A.col(j).array() -= minv;
    }

    // Masks and covers
    Eigen::MatrixXi mask = Eigen::MatrixXi::Zero(n, n); // 1=star, 2=prime
    std::vector<bool> rowCover(n,false), colCover(n,false);

    // Step 3: Star zeros greedily
    for (int i = 0; i < n; ++i) {
      for (int j = 0; j < n; ++j) {
        if (A(i,j) == 0.0 && !rowCover[i] && !colCover[j]) {
          mask(i,j) = 1; // star
          rowCover[i] = true;
          colCover[j] = true;
        }
      }
    }
    std::fill(rowCover.begin(), rowCover.end(), false);
    std::fill(colCover.begin(), colCover.end(), false);

    auto cover_columns_with_stars = [&]() {
      int count = 0;
      for (int j = 0; j < n; ++j) {
        for (int i = 0; i < n; ++i) {
          if (mask(i,j) == 1) { colCover[j] = true; break; }
        }
        if (colCover[j]) count++;
      }
      return count;
    };

    int step = 4;
    int coveredCols = cover_columns_with_stars();
    if (coveredCols >= n) step = 7;

    int z0_r = -1, z0_c = -1; // position of last primed zero

    while (true) {
      if (step == 4) {
        // Step 4: Find a noncovered zero and prime it. If no star in its row, go to Step 5.
        bool done = false;
        while (!done) {
          int row = -1, col = -1;
          // Find a noncovered zero
          for (int i = 0; i < n; ++i) {
            if (rowCover[i]) continue;
            for (int j = 0; j < n; ++j) {
              if (!colCover[j] && A(i,j) == 0.0) { row = i; col = j; break; }
            }
            if (row != -1) break;
          }
          if (row == -1) { step = 6; done = true; break; } // no uncovered zero
          mask(row,col) = 2; // prime
          // If there is a star in the row, cover row and uncover the column containing the star
          int star_col = -1;
          for (int j = 0; j < n; ++j) {
            if (mask(row,j) == 1) { star_col = j; break; }
          }
          if (star_col != -1) {
            rowCover[row] = true;
            colCover[star_col] = false;
          } else {
            // Go to Step 5
            step = 5;
            z0_r = row; z0_c = col;
            done = true;
          }
        }
      } else if (step == 5) {
        // Step 5: Construct a series of alternating primed and starred zeros
        std::vector<std::pair<int,int>> path;
        path.emplace_back(z0_r, z0_c);
        bool done = false;
        while (!done) {
          // Find starred zero in column of last element
          int r = -1;
          for (int i = 0; i < n; ++i) {
            if (mask(i, path.back().second) == 1) { r = i; break; }
          }
          if (r == -1) {
            done = true;
          } else {
            path.emplace_back(r, path.back().second);
            // Find primed zero in row r
            int c = -1;
            for (int j = 0; j < n; ++j) {
              if (mask(r,j) == 2) { c = j; break; }
            }
            path.emplace_back(r, c);
          }
        }
        // Unstar stars and star primes along the path
        for (auto& rc : path) {
          if (mask(rc.first, rc.second) == 1) mask(rc.first, rc.second) = 0;
          else if (mask(rc.first, rc.second) == 2) mask(rc.first, rc.second) = 1;
        }
        // Clear covers and erase all primes
        std::fill(rowCover.begin(), rowCover.end(), false);
        std::fill(colCover.begin(), colCover.end(), false);
        for (int i = 0; i < n; ++i) for (int j = 0; j < n; ++j) if (mask(i,j) == 2) mask(i,j) = 0;
        // Cover columns with stars
        coveredCols = cover_columns_with_stars();
        if (coveredCols >= n) step = 7; else step = 4;
      } else if (step == 6) {
        // Step 6: Add the smallest uncovered value to every covered row, subtract it from every uncovered column
        double minval = std::numeric_limits<double>::infinity();
        for (int i = 0; i < n; ++i) if (!rowCover[i])
          for (int j = 0; j < n; ++j) if (!colCover[j])
            if (A(i,j) < minval) minval = A(i,j);
        if (!std::isfinite(minval)) minval = 0.0;
        for (int i = 0; i < n; ++i) {
          for (int j = 0; j < n; ++j) {
            if (rowCover[i]) A(i,j) += minval;
            if (!colCover[j]) A(i,j) -= minval;
          }
        }
        step = 4;
      } else if (step == 7) {
        break;
      }
    }

    // Build solution: starred zeros indicate assignments
    std::vector<std::pair<int,int>> assignments;
    assignments.reserve(n);
    for (int i = 0; i < n; ++i) {
      for (int j = 0; j < n; ++j) {
        if (mask(i,j) == 1) {
          if (i < M && j < N) assignments.emplace_back(i, j);
        }
      }
    }
    return assignments;
  }
};

// Convenience wrapper that returns (drone_index -> track_index), with -1 if unassigned.
inline std::vector<int> assign_drones_to_tracks(const std::vector<Drone>& drones,
                                                const std::vector<Track>& tracks,
                                                const Params& params,
                                                Eigen::MatrixXd* out_cost_matrix = nullptr) {
  Eigen::MatrixXd C = build_cost_matrix(drones, tracks, params);
  if (out_cost_matrix) *out_cost_matrix = C;
  auto pairs = Hungarian::solve(C, params.unassigned_penalty);

  // Build mapping vector size M (drones) -> track index or -1
  std::vector<int> result(drones.size(), -1);
  for (auto& pr : pairs) {
    int r = pr.first;
    int c = pr.second;
    if (r >= 0 && r < (int)drones.size() && c >= 0 && c < (int)tracks.size()) {
      result[r] = c;
    }
  }
  return result;
}

} // namespace mr_assign
