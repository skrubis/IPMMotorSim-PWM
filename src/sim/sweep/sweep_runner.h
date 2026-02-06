#ifndef SIM_SWEEP_RUNNER_H
#define SIM_SWEEP_RUNNER_H

#include <QString>
#include <vector>
#include <functional>
#include <limits>

#include "sim/metrics/metric_pack.h"
#include "sim/sim_runner.h"
#include "sim/sweep/sweep_config.h"

namespace sim
{
struct SweepPoint
{
    double speed_rpm = 0.0;
    double iq_A = 0.0;
    double id_A = 0.0;
    double vdc_V = 0.0;
    double temp_C = 0.0;
    double f_sw_Hz = 0.0;
};

struct SweepPointResult
{
    SweepPoint point;
    StrategyId strategy = StrategyId::SVPWM;
    MetricPack metrics;
    double avg_igbt_cond_w = 0.0;
    double avg_diode_cond_w = 0.0;
    double avg_igbt_sw_w = 0.0;
    double avg_diode_rr_w = 0.0;
    double avg_total_w = 0.0;
    double avg_inv_eff_pct = 0.0;
    bool constraint_thd_ok = true;
    bool constraint_ripple_ok = true;
    bool constraint_pulse_ok = true;
    bool constraint_ok = true;
    int samples = 0;
};

struct SweepResult
{
    std::vector<SweepPointResult> points;
};

struct SweepContext
{
    SimRunner* runner = nullptr;
    MotorPlant* motor = nullptr;
    SimInputs base_inputs;
};

struct SweepProgress
{
    int point_index = 0;
    int point_count = 0;
    int strategy_index = 0;
    int strategy_count = 0;
    SweepPoint point;
    StrategyId strategy = StrategyId::SVPWM;
    int step_index = 0;
    int measure_step_index = 0;
    int settle_steps = 0;
    int measure_steps = 0;
    bool in_measure = false;
    ModulationMode current_mode = ModulationMode::SVPWM;
    double total_loss_w = std::numeric_limits<double>::quiet_NaN();
    double switching_loss_w = std::numeric_limits<double>::quiet_NaN();
    double thd_proxy_pct = std::numeric_limits<double>::quiet_NaN();
    double min_pulse_margin_s = std::numeric_limits<double>::quiet_NaN();
    int auto_switch_count = 0;
    ModulationMode auto_primary_mode = ModulationMode::SVPWM;
    double auto_primary_frac = 0.0;
};

struct SweepCallbacks
{
    std::function<void(const SweepProgress&)> on_progress;
    std::function<bool()> should_abort;
};

std::vector<SweepPoint> BuildOperatingMapPoints(const SweepConfig& cfg);
std::vector<SweepPoint> BuildFixedPointFreqPoints(const SweepConfig& cfg);
bool ResolveRunSteps(const SweepConfig& cfg, double timestep_s, double elec_freq_hz, int* settle_steps, int* measure_steps);

bool RunSweep(const SweepConfig& cfg, const SweepContext& ctx, SweepResult* out, QString* error,
              const SweepCallbacks* callbacks = nullptr);
} // namespace sim

#endif // SIM_SWEEP_RUNNER_H
