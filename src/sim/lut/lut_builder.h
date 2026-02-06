#ifndef SIM_LUT_BUILDER_H
#define SIM_LUT_BUILDER_H

#include <QString>
#include <limits>
#include <vector>

#include "sim/strategy/strategy_spec.h"

namespace sim
{
struct LutConfig
{
    int phi_bins = 12;
    double phi_min_deg = -180.0;
    double phi_max_deg = 180.0;
    QString phi_source = "phi_idiq_deg";
    bool smooth_islands = true;
    int island_min_neighbors = 4;
};

struct LutConstraints
{
    double thd_max_pct = std::numeric_limits<double>::quiet_NaN();
    double i_ripple_rms_max_a = std::numeric_limits<double>::quiet_NaN();
    double min_pulse_margin_min_s = std::numeric_limits<double>::quiet_NaN();
    double min_pulse_s_effective = std::numeric_limits<double>::quiet_NaN();
};

struct LutBuildInfo
{
    QString timestamp_utc;
    QString git_hash;
    std::vector<QString> warnings;
};

struct LutCell
{
    StrategyId strategy = StrategyId::SVPWM;
    bool valid = false;
    double loss_w = 0.0;
    double best_loss_w = 0.0;
};

struct LutResult
{
    std::vector<double> speed_rpm;
    std::vector<double> iq_A;
    int phi_bins = 0;
    double phi_min_deg = -180.0;
    double phi_max_deg = 180.0;
    QString phi_source;
    std::vector<double> phi_bins_deg;
    std::vector<LutCell> cells;
    std::vector<uint8_t> valid_mask;
    double regret_mean_w = 0.0;
    double regret_max_w = 0.0;
    int invalid_cells = 0;
    int constraint_violations = 0;
    double phi_bin_fill_ratio = 0.0;
    double avg_filled_phi_bins_per_speediq = 0.0;
    LutConstraints constraints;
    LutBuildInfo build_info;
    QString run_manifest_path;
};

bool BuildLutFromSummaryCsv(const QString& summary_csv_path,
                            const LutConfig& config,
                            LutResult* out,
                            QString* error);

bool WriteLutJson(const QString& path, const LutResult& lut, QString* error = nullptr);
bool WriteLutHeader(const QString& path, const LutResult& lut, QString* error = nullptr);
bool WriteLutRuntimeParams(const QString& path, const LutResult& lut, QString* error = nullptr);
} // namespace sim

#endif // SIM_LUT_BUILDER_H
