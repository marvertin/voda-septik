#include "filter_monitor.h"
#include <algorithm>
#include <cmath>

void FilterMonitor::reset_window() { count_ = 0; start_ts_ = -1; }
void FilterMonitor::flow(float value, int64_t ts) {
    if (ts <= flow_ts_) return;
    if (flow_ts_ >= 0 && ts - flow_ts_ > MAX_AGE_US) reset_window();
    flow_ts_ = ts;
    flow_ = value;
    if (!std::isfinite(value) || value < config_.min_flow) reset_window();
}
void FilterMonitor::tick(int64_t now) {
    if (flow_ts_ < 0 || pressure_ts_ < 0 || now - flow_ts_ > MAX_AGE_US ||
        now - pressure_ts_ > MAX_AGE_US) reset_window();
}
bool FilterMonitor::pressure(float before, float after, int64_t ts) {
    if (ts <= pressure_ts_) return false;
    if (pressure_ts_ >= 0 && ts - pressure_ts_ > MAX_AGE_US) reset_window();
    pressure_ts_ = ts;
    if (!std::isfinite(before) || !std::isfinite(after) || !std::isfinite(flow_) ||
        flow_ < config_.min_flow || flow_ts_ < 0 || ts < flow_ts_ ||
        ts - flow_ts_ > MAX_AGE_US) { reset_window(); return false; }
    if (start_ts_ < 0) start_ts_ = ts;
    if (count_ == 64) { reset_window(); return false; }
    dp_[count_] = before - after - config_.offset_bar;
    flows_[count_] = flow_;
    before_[count_] = before;
    after_[count_++] = after;
    if (ts - start_ts_ < WINDOW_US) return false;
    bool accepted = false;
    if (count_ >= 30) {
        float flow_sum = 0;
        for (size_t i = 0; i < count_; ++i) flow_sum += flows_[i];
        const float flow_mean = flow_sum / count_;
        auto flow_range = std::minmax_element(flows_, flows_ + count_);
        // Reject common-mode pressure transients as well as unstable differential pressure.
        std::sort(before_, before_ + count_);
        std::sort(after_, after_ + count_);
        std::sort(dp_, dp_ + count_);
        const size_t trim = count_ / 10, n = count_ - 2 * trim;
        double sum = 0, variance = 0;
        for (size_t i = trim; i < count_ - trim; ++i) sum += dp_[i];
        const float mean = sum / n;
        for (size_t i = trim; i < count_ - trim; ++i) {
            const double d = dp_[i] - mean; variance += d * d;
        }
        const float stddev = std::sqrt(variance / n);
        accepted = *flow_range.second - *flow_range.first <= config_.max_flow_range_fraction * flow_mean &&
            before_[count_ - trim - 1] - before_[trim] <= config_.max_pressure_range_bar &&
            after_[count_ - trim - 1] - after_[trim] <= config_.max_pressure_range_bar &&
            stddev <= config_.max_stddev_bar;
        if (accepted) {
            result_ = {true, result_.measurement_id + 1, mean, flow_mean, stddev, n};
        }
    }
    reset_window();
    // Next window starts at this boundary; its first sample arrives in the next cycle.
    start_ts_ = ts;
    return accepted;
}
