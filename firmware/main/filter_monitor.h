#pragma once
#include <cstddef>
#include <cstdint>

struct FilterConfig {
    float min_flow = 25.0f;
    float offset_bar = 0.0f;
    float max_stddev_bar = 0.05f;
    float max_flow_range_fraction = 0.20f;
    float max_pressure_range_bar = 0.30f;
};
struct FilterResult {
    bool valid = false;
    uint32_t measurement_id = 0;
    float pressure_diff_bar = 0;
    float flow_l_min = 0;
    float pressure_diff_stddev_bar = 0;
    size_t sample_count = 0; // retained samples after trimming
};
class FilterMonitor {
public:
    explicit FilterMonitor(FilterConfig config = {}) : config_(config) {}
    void flow(float value, int64_t timestamp_us);
    bool pressure(float before, float after, int64_t timestamp_us);
    void tick(int64_t now_us);
    const FilterResult &result() const { return result_; }
private:
    static constexpr int64_t WINDOW_US = 10000000;
    static constexpr int64_t MAX_AGE_US = 1000000;
    void reset_window();
    FilterConfig config_;
    FilterResult result_;
    float flow_ = 0;
    int64_t flow_ts_ = -1, pressure_ts_ = -1, start_ts_ = -1;
    float dp_[64] = {}, flows_[64] = {}, before_[64] = {}, after_[64] = {};
    size_t count_ = 0;
};
void filter_monitor_register_config_items();
FilterConfig filter_monitor_load_config();
