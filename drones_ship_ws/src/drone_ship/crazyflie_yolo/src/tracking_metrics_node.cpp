// tracking_metrics_node.cpp
//
// FINAL tracking metrics node (ROS 2)
// ----------------------------------
// Goal: Fair comparison of OURS vs AB3DMOT vs SORT in 3D (world/ENU).
//
// Key fixes vs your current version:
// 1) Adds per-tick DEBUG that explains NaNs (pred_count / matches / min distances).
// 2) Optionally require NON-EMPTY predictions to evaluate a method on a tick.
// 3) Computes a fair "tick stamp" as NOW (or GT min-stamp) selectable.
// 4) Adds distance diagnostics: min distance from each GT to any pred + overall min.
// 5) Writes CSV with stable columns and debug columns.
//
// Important notes:
// - NaN in Med/RMSE/P95 is correct if there are zero matched pairs so far.
// - This node will tell you *why* matches are zero (no preds, too far, stale, etc.).
//
// Assumptions:
// - All tracker outputs are in the same frame as GT (world/ENU).
// - GT is /container{i}/odometry (i=1..gt_container_count).
// - Tracker outputs are crazyflie_yolo/msg/TrackedObstacleArray.

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>

#include "crazyflie_yolo/msg/tracked_obstacle_array.hpp"

#include <unordered_map>
#include <map>
#include <vector>
#include <string>
#include <limits>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <filesystem>
#include <chrono>

using crazyflie_yolo::msg::TrackedObstacleArray;

// ============================================================
// Math helpers
// ============================================================
struct Vec3 {
  double x{0.0}, y{0.0}, z{0.0};
};

static inline double dist3(const Vec3& a, const Vec3& b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx*dx + dy*dy + dz*dz);
}

// ============================================================
// Hungarian (minimum-cost assignment) O(n^3)
// ============================================================
static std::vector<int> hungarianMinCost(const std::vector<std::vector<double>>& cost)
{
  const int n = static_cast<int>(cost.size());
  const double INF = 1e18;

  std::vector<double> u(n + 1, 0.0), v(n + 1, 0.0);
  std::vector<int> p(n + 1, 0), way(n + 1, 0);

  for (int i = 1; i <= n; ++i) {
    p[0] = i;
    int j0 = 0;
    std::vector<double> minv(n + 1, INF);
    std::vector<char> used(n + 1, false);

    do {
      used[j0] = true;
      const int i0 = p[j0];
      int j1 = 0;
      double delta = INF;

      for (int j = 1; j <= n; ++j) {
        if (used[j]) continue;
        const double cur = cost[i0 - 1][j - 1] - u[i0] - v[j];
        if (cur < minv[j]) {
          minv[j] = cur;
          way[j] = j0;
        }
        if (minv[j] < delta) {
          delta = minv[j];
          j1 = j;
        }
      }

      for (int j = 0; j <= n; ++j) {
        if (used[j]) {
          u[p[j]] += delta;
          v[j] -= delta;
        } else {
          minv[j] -= delta;
        }
      }
      j0 = j1;
    } while (p[j0] != 0);

    do {
      const int j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0 != 0);
  }

  std::vector<int> ans(n, -1);
  for (int j = 1; j <= n; ++j) {
    if (p[j] >= 1 && p[j] <= n) {
      ans[p[j] - 1] = j - 1;
    }
  }
  return ans;
}

static std::vector<std::vector<double>> makeSquareCost(
  const std::vector<std::vector<double>>& C, double pad_cost)
{
  const int R = static_cast<int>(C.size());
  const int K = (R > 0) ? static_cast<int>(C[0].size()) : 0;
  const int n = std::max(R, K);

  std::vector<std::vector<double>> S(n, std::vector<double>(n, pad_cost));
  for (int i = 0; i < R; ++i) {
    for (int j = 0; j < K; ++j) {
      S[i][j] = C[i][j];
    }
  }
  return S;
}

// ============================================================
// Per-method cumulative state
// ============================================================
struct MethodState
{
  std::unordered_map<int, int>  last_pred_id;   // gt_id -> last matched track id
  std::unordered_map<int, bool> was_matched;    // gt_id -> matched last tick
  std::unordered_map<int, bool> seen_before;    // gt_id -> ever seen

  long long total_gt{0};
  long long total_fp{0};
  long long total_fn{0};
  long long idsw{0};
  long long frag{0};

  std::vector<double> errors;

  std::map<int, std::map<int, long long>> id_counts;
  long long total_pred_matched{0};
  long long total_gt_matched{0};
};

// Latest predictions cache
struct PredFrame
{
  rclcpp::Time stamp;
  std::vector<int> ids;
  std::vector<Vec3> pos;
  std::string frame_id;
};

// One CSV row per method per tick
struct TickRow
{
  double stamp_sec{0.0};
  std::string method;

  double idf1{std::numeric_limits<double>::quiet_NaN()};
  long long idsw{0};
  long long frag{0};
  double mota{std::numeric_limits<double>::quiet_NaN()};

  double med{std::numeric_limits<double>::quiet_NaN()};
  double rmse{std::numeric_limits<double>::quiet_NaN()};
  double p95{std::numeric_limits<double>::quiet_NaN()};

  long long fp{0};
  long long fn{0};
  long long gt{0};

  long long frame_gt{0};
  long long frame_pred{0};
  long long frame_matches{0};

  bool fresh{false};
  bool nonempty{false};

  // Distance diagnostics
  double min_gt_to_pred{std::numeric_limits<double>::quiet_NaN()}; // min over all GT
  double mean_gt_to_pred{std::numeric_limits<double>::quiet_NaN()};
  double p95_gt_to_pred{std::numeric_limits<double>::quiet_NaN()};

  double pred_age_sec{std::numeric_limits<double>::quiet_NaN()};
  std::string pred_frame;
};

// ============================================================
// TrackingMetricsNode
// ============================================================
class TrackingMetricsNode : public rclcpp::Node
{
public:
  TrackingMetricsNode()
  : Node("tracking_metrics_node")
  {
    // ---------------------------
    // Parameters
    // ---------------------------
    eval_rate_hz_ = declare_parameter<double>("eval_rate_hz", 5.0);
    dist_thresh_ = declare_parameter<double>("dist_thresh", 8.0);
    gt_container_count_ = declare_parameter<int>("gt_container_count", 5);

    pred_timeout_sec_ = declare_parameter<double>("pred_timeout_sec", 1.0);
    gt_timeout_sec_ = declare_parameter<double>("gt_timeout_sec", 1.0);
    warmup_ticks_ = declare_parameter<int>("warmup_ticks", 10);

    require_all_methods_fresh_ = declare_parameter<bool>("require_all_methods_fresh", false);

    // New knobs:
    require_nonempty_pred_ = declare_parameter<bool>("require_nonempty_pred", false);
    debug_ = declare_parameter<bool>("debug", true);
    debug_every_n_ticks_ = declare_parameter<int>("debug_every_n_ticks", 5);

    // Tick timestamp source:
    // "gt_min" (oldest GT stamp), "now" (this->now()).
    tick_time_source_ = declare_parameter<std::string>("tick_time_source", "gt_min");

    methods_ = declare_parameter<std::vector<std::string>>(
      "methods", std::vector<std::string>{"ours", "sort", "ab3dmot"});

    csv_path_   = declare_parameter<std::string>("csv_path", "/tmp/tracking_metrics.csv");
    csv_append_ = declare_parameter<bool>("csv_append", false);
    write_every_n_ = declare_parameter<int>("write_every_n", 1);

    // ---------------------------
    // Tracker subscriptions
    // ---------------------------
    for (const auto& m : methods_) {
      const std::string key = "method_topics." + m;
      std::string default_topic;

      if (m == "ours")      default_topic = "tracked_obstacles_array";
      else if (m == "sort") default_topic = "sort/tracked_obstacles_array";
      else if (m == "ab3dmot") default_topic = "ab3dmot/tracked_obstacles_array";
      else default_topic = m + "/tracked_obstacles_array";

      const std::string topic = declare_parameter<std::string>(key, default_topic);
      method_topics_[m] = topic;

      auto sub = create_subscription<TrackedObstacleArray>(
        topic, rclcpp::QoS(20),
        [this, m](TrackedObstacleArray::ConstSharedPtr msg) {
          this->predCb(m, msg);
        });

      pred_subs_.push_back(sub);
      method_state_[m] = MethodState{};

      RCLCPP_INFO(get_logger(), "Method '%s' subscribed to: %s", m.c_str(), topic.c_str());
    }

    // ---------------------------
    // GT subscriptions
    // ---------------------------
    for (int i = 1; i <= gt_container_count_; ++i) {
      const std::string topic = "/container" + std::to_string(i) + "/odometry";
      auto sub = create_subscription<nav_msgs::msg::Odometry>(
        topic, rclcpp::SensorDataQoS(),
        [this, i](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          gt_pos_[i] = Vec3{
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z
          };
          gt_stamps_[i] = rclcpp::Time(msg->header.stamp);
          newest_gt_stamp_ = rclcpp::Time(msg->header.stamp);
        });
      gt_subs_.push_back(sub);
      RCLCPP_INFO(get_logger(), "GT subscribed to: %s", topic.c_str());
    }

    report_pub_ = create_publisher<std_msgs::msg::String>("tracking_metrics/report", rclcpp::QoS(10));

    openCsv();

    const auto period = std::chrono::duration<double>(1.0 / std::max(0.1, eval_rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&TrackingMetricsNode::onTimer, this));

    RCLCPP_INFO(get_logger(),
      "TrackingMetricsNode FINAL | eval_rate=%.2fHz dist_thresh=%.2f gt_count=%d warmup=%d "
      "pred_timeout=%.2f gt_timeout=%.2f require_nonempty=%d tick_time_source=%s",
      eval_rate_hz_, dist_thresh_, gt_container_count_, warmup_ticks_,
      pred_timeout_sec_, gt_timeout_sec_, (int)require_nonempty_pred_, tick_time_source_.c_str());
  }

  ~TrackingMetricsNode() override
  {
    if (csv_.is_open()) {
      csv_.flush();
      csv_.close();
    }
  }

private:
  // ============================================================
  // CSV
  // ============================================================
  void openCsv()
  {
    namespace fs = std::filesystem;
    try {
      fs::path p(csv_path_);
      if (p.has_parent_path()) {
        fs::create_directories(p.parent_path());
      }
    } catch (...) {
      RCLCPP_WARN(get_logger(), "Failed to create CSV directory for %s", csv_path_.c_str());
    }

    std::ios_base::openmode mode = std::ios::out;
    mode |= (csv_append_ ? std::ios::app : std::ios::trunc);
    csv_.open(csv_path_, mode);

    if (!csv_.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open CSV: %s", csv_path_.c_str());
      return;
    }

    if (csv_append_) {
      try {
        csv_header_written_ = (std::filesystem::file_size(csv_path_) > 0);
      } catch (...) {
        csv_header_written_ = false;
      }
    } else {
      csv_header_written_ = false;
    }

    RCLCPP_INFO(get_logger(), "CSV logging: %s (append=%s)",
                csv_path_.c_str(), csv_append_ ? "true" : "false");
  }

  void writeCsvHeaderIfNeeded()
  {
    if (!csv_.is_open() || csv_header_written_) return;

    csv_ << "stamp_sec,method,IDF1,IDSW,Frag,MOTA,MedErr,RMSE,P95,FP,FN,GT,"
         << "frame_GT,frame_pred,frame_matches,fresh,nonempty,pred_age_sec,pred_frame,"
         << "min_gt_to_pred,mean_gt_to_pred,p95_gt_to_pred\n";
    csv_.flush();
    csv_header_written_ = true;
  }

  static std::string fmtDouble(double v)
  {
    if (!std::isfinite(v)) return "nan";
    std::ostringstream os;
    os << std::setprecision(10) << v;
    return os.str();
  }

  void writeCsvRowsForTick(const std::vector<TickRow>& rows)
  {
    if (!csv_.is_open() || rows.empty()) return;

    const long long tick_key_us = static_cast<long long>(std::llround(rows.front().stamp_sec * 1e6));
    if (last_written_tick_key_us_ == tick_key_us) {
      return;
    }

    writeCsvHeaderIfNeeded();

    for (const auto& r : rows) {
      csv_ << fmtDouble(r.stamp_sec) << ","
           << r.method << ","
           << fmtDouble(r.idf1) << ","
           << r.idsw << ","
           << r.frag << ","
           << fmtDouble(r.mota) << ","
           << fmtDouble(r.med) << ","
           << fmtDouble(r.rmse) << ","
           << fmtDouble(r.p95) << ","
           << r.fp << ","
           << r.fn << ","
           << r.gt << ","
           << r.frame_gt << ","
           << r.frame_pred << ","
           << r.frame_matches << ","
           << (r.fresh ? 1 : 0) << ","
           << (r.nonempty ? 1 : 0) << ","
           << fmtDouble(r.pred_age_sec) << ","
           << r.pred_frame << ","
           << fmtDouble(r.min_gt_to_pred) << ","
           << fmtDouble(r.mean_gt_to_pred) << ","
           << fmtDouble(r.p95_gt_to_pred)
           << "\n";
    }

    csv_.flush();
    last_written_tick_key_us_ = tick_key_us;
  }

  // ============================================================
  // Callbacks
  // ============================================================
  void predCb(const std::string& method, TrackedObstacleArray::ConstSharedPtr msg)
  {
    PredFrame pf;
    pf.stamp = rclcpp::Time(msg->header.stamp);
    pf.frame_id = msg->header.frame_id;
    pf.ids.reserve(msg->obstacles.size());
    pf.pos.reserve(msg->obstacles.size());

    for (const auto& o : msg->obstacles) {
      pf.ids.push_back(static_cast<int>(o.id));
      pf.pos.push_back(Vec3{o.position.x, o.position.y, o.position.z});
    }

    latest_pred_[method] = std::move(pf);
  }

  // ============================================================
  // Timer
  // ============================================================
  void onTimer()
  {
    ++tick_count_total_;
    if (tick_count_total_ <= warmup_ticks_) return;

    // Build GT
    std::vector<int> gt_ids;
    std::vector<Vec3> gt_vec;
    rclcpp::Time gt_min_stamp;
    if (!buildCommonGTSnapshot(gt_ids, gt_vec, gt_min_stamp)) return;

    rclcpp::Time tick_stamp = (tick_time_source_ == "now") ? now() : gt_min_stamp;
    const double tick_stamp_sec = tick_stamp.seconds();

    // freshness per method
    std::map<std::string, bool> fresh_map;
    bool all_fresh = true;
    for (const auto& m : methods_) {
      const bool f = isPredFresh(m, tick_stamp);
      fresh_map[m] = f;
      if (!f) all_fresh = false;
    }

    if (require_all_methods_fresh_ && !all_fresh) {
      if (debug_ && (tick_count_total_ % std::max(1, debug_every_n_ticks_) == 0)) {
        RCLCPP_WARN(get_logger(), "[METRICS] skipping tick %.3f (not all methods fresh)", tick_stamp_sec);
      }
      return;
    }

    std::vector<TickRow> rows;
    rows.reserve(methods_.size());

    for (const auto& m : methods_) {
      TickRow row;
      row.stamp_sec = tick_stamp_sec;
      row.method = m;
      row.frame_gt = static_cast<long long>(gt_ids.size());
      row.fresh = fresh_map[m];

      auto pit = latest_pred_.find(m);
      if (pit == latest_pred_.end()) {
        row.frame_pred = 0;
        row.nonempty = false;
        row.pred_age_sec = std::numeric_limits<double>::quiet_NaN();
        row.pred_frame = "";
        // still log row? we skip by default if not fresh
        if (!row.fresh) continue;
        rows.push_back(row);
        continue;
      }

      const auto& pred = pit->second;
      row.frame_pred = static_cast<long long>(pred.ids.size());
      row.nonempty = (row.frame_pred > 0);
      row.pred_age_sec = (now() - pred.stamp).seconds();
      row.pred_frame = pred.frame_id;

      // If user requests non-empty to evaluate, skip empty preds
      if (require_nonempty_pred_ && row.frame_pred == 0) {
        if (debug_ && (tick_count_total_ % std::max(1, debug_every_n_ticks_) == 0)) {
          RCLCPP_WARN(get_logger(), "[METRICS] method=%s skipped (empty predictions)", m.c_str());
        }
        continue;
      }

      // Distance diagnostics (even if no matches)
      computeGtToPredDistanceStats(gt_vec, pred.pos,
                                  row.min_gt_to_pred, row.mean_gt_to_pred, row.p95_gt_to_pred);

      long long frame_matches = 0;
      evalOneMethod(m, gt_ids, gt_vec, pred.ids, pred.pos, frame_matches);

      const auto& S = method_state_[m];
      row.frame_matches = frame_matches;

      row.idf1 = computeIDF1(S);
      row.idsw = S.idsw;
      row.frag = S.frag;
      row.mota = computeMOTA(S);

      row.med  = percentile(S.errors, 50.0);
      row.rmse = rmse(S.errors);
      row.p95  = percentile(S.errors, 95.0);

      row.fp = S.total_fp;
      row.fn = S.total_fn;
      row.gt = S.total_gt;

      rows.push_back(row);

      if (debug_ && (tick_count_total_ % std::max(1, debug_every_n_ticks_) == 0)) {
        RCLCPP_INFO(get_logger(),
          "[METRICS] t=%.3f m=%s fresh=%d pred=%lld gt=%lld matches=%lld "
          "minGT2P=%.2f meanGT2P=%.2f p95GT2P=%.2f | cumGT=%lld cumFN=%lld cumFP=%lld errors=%zu",
          tick_stamp_sec, m.c_str(), (int)row.fresh,
          row.frame_pred, row.frame_gt, row.frame_matches,
          row.min_gt_to_pred, row.mean_gt_to_pred, row.p95_gt_to_pred,
          row.gt, row.fn, row.fp, method_state_[m].errors.size());
      }
    }

    if (rows.empty()) return;

    publishReport();

    ++eval_write_counter_;
    if (write_every_n_ > 0 && (eval_write_counter_ % write_every_n_) == 0) {
      writeCsvRowsForTick(rows);
    }
  }

  bool buildCommonGTSnapshot(std::vector<int>& gt_ids,
                            std::vector<Vec3>& gt_vec,
                            rclcpp::Time& min_stamp)
  {
    gt_ids.clear();
    gt_vec.clear();

    if ((int)gt_pos_.size() < gt_container_count_) return false;
    if ((int)gt_stamps_.size() < gt_container_count_) return false;

    const rclcpp::Time now_t = now();

    bool first = true;
    for (int i = 1; i <= gt_container_count_; ++i) {
      auto itp = gt_pos_.find(i);
      auto its = gt_stamps_.find(i);
      if (itp == gt_pos_.end() || its == gt_stamps_.end()) return false;

      const rclcpp::Time s = its->second;
      if (s.nanoseconds() == 0) return false;

      const double age = (now_t - s).seconds();
      if (age > gt_timeout_sec_) {
        return false;
      }

      if (first) { min_stamp = s; first = false; }
      else if (s < min_stamp) { min_stamp = s; }
    }

    for (int i = 1; i <= gt_container_count_; ++i) {
      gt_ids.push_back(i);
      gt_vec.push_back(gt_pos_[i]);
    }

    return true;
  }

  bool isPredFresh(const std::string& method, const rclcpp::Time& tick_ref) const
  {
    auto it = latest_pred_.find(method);
    if (it == latest_pred_.end()) return false;
    const auto& pf = it->second;
    if (pf.stamp.nanoseconds() == 0) return false;

    const double age_now = (now() - pf.stamp).seconds();
    if (age_now > pred_timeout_sec_) return false;

    // guard against very old relative to tick
    const double lag = std::abs((tick_ref - pf.stamp).seconds());
    if (lag > pred_timeout_sec_) return false;

    return true;
  }

  // ============================================================
  // Distance diagnostics: for each GT, compute nearest pred distance
  // ============================================================
  static void computeGtToPredDistanceStats(const std::vector<Vec3>& gt,
                                          const std::vector<Vec3>& pr,
                                          double& out_min,
                                          double& out_mean,
                                          double& out_p95)
  {
    out_min = std::numeric_limits<double>::quiet_NaN();
    out_mean = std::numeric_limits<double>::quiet_NaN();
    out_p95 = std::numeric_limits<double>::quiet_NaN();

    if (gt.empty() || pr.empty()) return;

    std::vector<double> dists;
    dists.reserve(gt.size());

    for (const auto& g : gt) {
      double best = std::numeric_limits<double>::infinity();
      for (const auto& p : pr) {
        best = std::min(best, dist3(g, p));
      }
      dists.push_back(best);
    }

    out_min = *std::min_element(dists.begin(), dists.end());
    const double sum = std::accumulate(dists.begin(), dists.end(), 0.0);
    out_mean = sum / std::max<size_t>(1, dists.size());
    std::sort(dists.begin(), dists.end());
    const size_t idx = (size_t)std::floor(0.95 * (double)(dists.size() - 1));
    out_p95 = dists[idx];
  }

  // ============================================================
  // Core eval per method (cumulative)
  // ============================================================
  void evalOneMethod(const std::string& method,
                     const std::vector<int>& gt_ids,
                     const std::vector<Vec3>& gt_pos,
                     const std::vector<int>& pr_ids,
                     const std::vector<Vec3>& pr_pos,
                     long long& frame_matches_out)
  {
    frame_matches_out = 0;
    auto& S = method_state_[method];

    const int G = (int)gt_ids.size();
    const int K = (int)pr_ids.size();

    S.total_gt += G;

    if (G == 0 && K == 0) return;
    if (G == 0) { S.total_fp += K; return; }
    if (K == 0) {
      S.total_fn += G;
      for (int i = 0; i < G; ++i) {
        const int g = gt_ids[i];
        if (S.seen_before.find(g) == S.seen_before.end()) S.seen_before[g] = true;
        S.was_matched[g] = false;
      }
      return;
    }

    std::vector<std::vector<double>> C(G, std::vector<double>(K, 0.0));
    for (int i = 0; i < G; ++i)
      for (int j = 0; j < K; ++j)
        C[i][j] = dist3(gt_pos[i], pr_pos[j]);

    constexpr double PAD = 1e6;
    auto Sq = makeSquareCost(C, PAD);
    auto assign = hungarianMinCost(Sq);

    std::vector<int> gt_to_pr(G, -1);
    int matched = 0;
    for (int i = 0; i < G; ++i) {
      const int j = assign[i];
      if (j < 0 || j >= K) continue;
      if (C[i][j] <= dist_thresh_) {
        gt_to_pr[i] = j;
        matched++;
      }
    }

    frame_matches_out = matched;

    const int fn = G - matched;
    const int fp = K - matched;

    S.total_fn += fn;
    S.total_fp += fp;
    S.total_gt_matched += matched;
    S.total_pred_matched += matched;

    for (int i = 0; i < G; ++i) {
      const int g_id = gt_ids[i];
      const bool now_matched = (gt_to_pr[i] >= 0);
      const int now_pred_id = now_matched ? pr_ids[gt_to_pr[i]] : -1;

      const bool seen = (S.seen_before.find(g_id) != S.seen_before.end()) ? S.seen_before[g_id] : false;
      const bool had_prev = (S.was_matched.find(g_id) != S.was_matched.end());
      const bool prev_matched = had_prev ? S.was_matched[g_id] : false;
      const int prev_pred_id = (S.last_pred_id.find(g_id) != S.last_pred_id.end()) ? S.last_pred_id[g_id] : -1;

      if (!seen) {
        S.seen_before[g_id] = true;
      } else {
        if (had_prev && !prev_matched && now_matched) S.frag++;
        if (prev_matched && now_matched && prev_pred_id >= 0 && now_pred_id >= 0 && prev_pred_id != now_pred_id)
          S.idsw++;
      }

      S.was_matched[g_id] = now_matched;

      if (now_matched) {
        S.last_pred_id[g_id] = now_pred_id;
        const double e = C[i][gt_to_pr[i]];
        S.errors.push_back(e);
        S.id_counts[g_id][now_pred_id] += 1;
      }
    }
  }

  // ============================================================
  // Metric helpers
  // ============================================================
  static double computeMOTA(const MethodState& S)
  {
    if (S.total_gt <= 0) return std::numeric_limits<double>::quiet_NaN();
    return 1.0 - (double)(S.total_fn + S.total_fp + S.idsw) / (double)S.total_gt;
  }

  double computeIDF1(const MethodState& S) const
  {
    std::vector<int> g_ids;
    std::vector<int> p_ids;

    for (const auto& gkv : S.id_counts) {
      g_ids.push_back(gkv.first);
      for (const auto& pkv : gkv.second) p_ids.push_back(pkv.first);
    }
    if (g_ids.empty()) return 0.0;
    std::sort(p_ids.begin(), p_ids.end());
    p_ids.erase(std::unique(p_ids.begin(), p_ids.end()), p_ids.end());
    if (p_ids.empty()) return 0.0;

    const int G = (int)g_ids.size();
    const int P = (int)p_ids.size();
    const int n = std::max(G, P);

    long long maxC = 0;
    for (const auto& gkv : S.id_counts)
      for (const auto& pkv : gkv.second)
        maxC = std::max(maxC, pkv.second);

    std::vector<std::vector<double>> cost(n, std::vector<double>(n, (double)maxC));
    for (int i = 0; i < G; ++i) {
      const int g = g_ids[i];
      for (int j = 0; j < P; ++j) {
        const int p = p_ids[j];
        long long c = 0;
        auto itg = S.id_counts.find(g);
        if (itg != S.id_counts.end()) {
          auto itp = itg->second.find(p);
          if (itp != itg->second.end()) c = itp->second;
        }
        cost[i][j] = (double)(maxC - c);
      }
    }

    auto assign = hungarianMinCost(cost);

    long long IDTP = 0;
    for (int i = 0; i < G; ++i) {
      const int j = assign[i];
      if (j < 0 || j >= P) continue;
      const int g = g_ids[i];
      const int p = p_ids[j];
      auto itg = S.id_counts.find(g);
      if (itg == S.id_counts.end()) continue;
      auto itp = itg->second.find(p);
      if (itp == itg->second.end()) continue;
      IDTP += itp->second;
    }

    const long long IDFN = S.total_gt_matched - IDTP;
    const long long IDFP = S.total_pred_matched - IDTP;

    const double denom = 2.0 * (double)IDTP + (double)IDFP + (double)IDFN;
    if (denom <= 1e-12) return 0.0;
    return (2.0 * (double)IDTP) / denom;
  }

  static double rmse(const std::vector<double>& e)
  {
    if (e.empty()) return std::numeric_limits<double>::quiet_NaN();
    long double s = 0.0L;
    for (double v : e) s += (long double)v * (long double)v;
    return std::sqrt((double)(s / (long double)e.size()));
  }

  static double percentile(std::vector<double> e, double p)
  {
    if (e.empty()) return std::numeric_limits<double>::quiet_NaN();
    std::sort(e.begin(), e.end());
    const double idx = (p / 100.0) * (double)(e.size() - 1);
    const size_t i0 = (size_t)std::floor(idx);
    const size_t i1 = std::min(i0 + 1, e.size() - 1);
    const double a = idx - (double)i0;
    return (1.0 - a) * e[i0] + a * e[i1];
  }

  // ============================================================
  // Report publisher
  // ============================================================
  void publishReport()
  {
    std_msgs::msg::String msg;
    std::ostringstream os;
    os << std::fixed << std::setprecision(3);
    os << "Tracking Metrics FINAL (d_thresh=" << dist_thresh_ << ", require_nonempty=" << (int)require_nonempty_pred_
       << ", tick_time=" << tick_time_source_ << ")\n";

    for (const auto& m : methods_) {
      const auto it = method_state_.find(m);
      if (it == method_state_.end()) continue;
      const auto& S = it->second;

      os << "  [" << m << "] "
         << "IDF1=" << computeIDF1(S)
         << " IDSW=" << S.idsw
         << " Frag=" << S.frag
         << " MOTA=" << computeMOTA(S)
         << " | Med=" << percentile(S.errors, 50.0)
         << " RMSE=" << rmse(S.errors)
         << " P95=" << percentile(S.errors, 95.0)
         << " | FP=" << S.total_fp
         << " FN=" << S.total_fn
         << " GT=" << S.total_gt
         << "\n";
    }

    msg.data = os.str();
    report_pub_->publish(msg);
  }

private:
  // Parameters
  double eval_rate_hz_{5.0};
  double dist_thresh_{8.0};
  int gt_container_count_{5};

  double pred_timeout_sec_{1.0};
  double gt_timeout_sec_{1.0};
  int warmup_ticks_{10};
  bool require_all_methods_fresh_{false};

  bool require_nonempty_pred_{false};
  bool debug_{true};
  int debug_every_n_ticks_{5};
  std::string tick_time_source_{"gt_min"};

  std::vector<std::string> methods_;
  std::map<std::string, std::string> method_topics_;

  // GT cache
  std::unordered_map<int, Vec3> gt_pos_;
  std::unordered_map<int, rclcpp::Time> gt_stamps_;
  rclcpp::Time newest_gt_stamp_;

  // Predictions cache
  std::unordered_map<std::string, PredFrame> latest_pred_;

  // Per-method state
  std::unordered_map<std::string, MethodState> method_state_;

  // ROS
  std::vector<rclcpp::Subscription<TrackedObstacleArray>::SharedPtr> pred_subs_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> gt_subs_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr report_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // CSV
  std::string csv_path_;
  bool csv_append_{false};
  int write_every_n_{1};
  std::ofstream csv_;
  bool csv_header_written_{false};
  long long last_written_tick_key_us_{-1};

  // Counters
  int tick_count_total_{0};
  int eval_write_counter_{0};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrackingMetricsNode>());
  rclcpp::shutdown();
  return 0;
}