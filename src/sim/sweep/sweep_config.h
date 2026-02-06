#ifndef SIM_SWEEP_CONFIG_H
#define SIM_SWEEP_CONFIG_H

#include <QString>
#include <vector>

#include "sim/metrics/metric_pack.h"
#include "sim/strategy/strategy_spec.h"

namespace sim
{
enum class SweepMode
{
    OperatingMapSweep,
    FixedPointFreqSweep
};

struct SweepAxis
{
    std::vector<double> values;
};

struct SweepConfig
{
    int schema_version = 1;
    SweepMode mode = SweepMode::OperatingMapSweep;

    SweepAxis speed_rpm;
    SweepAxis iq_A;
    SweepAxis id_A;
    SweepAxis vdc_V;
    SweepAxis temp_C;
    SweepAxis f_sw_Hz;

    double settle_ms = 0.0;
    double measure_ms = 0.0;
    double settle_cycles = 0.0;
    double measure_cycles = 0.0;

    int pole_pairs = 0;

    StrategyId baseline = StrategyId::SVPWM;
    std::vector<StrategyId> candidates;

    double thd_max_pct = -1.0;
    double i_ripple_rms_max_a = -1.0;
    double min_pulse_margin_min_s = 0.0;

    double min_pulse_s = -1.0;
    QString powerstage_preset;

    ThdMode thd_mode = ThdMode::ControlStepProxy;
    int thd_samples = 2048;

    QString output_dir;
    bool write_point_json = true;
    bool write_summary_csv = true;
};

bool LoadSweepConfig(const QString& path, SweepConfig* out, QString* error);
} // namespace sim

#endif // SIM_SWEEP_CONFIG_H
