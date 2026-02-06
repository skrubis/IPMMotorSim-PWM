#ifndef SIM_METRIC_PACK_H
#define SIM_METRIC_PACK_H

#include <array>
#include <limits>
#include <vector>

#include "sim/inverter_switching_model.h"
#include "sim/sim_runner.h"

namespace sim
{
enum class ThdMode
{
    Disabled,
    ControlStepProxy
};

struct ThdConfig
{
    ThdMode mode = ThdMode::Disabled;
    int samples = 2048;
    double fundamental_hz = 0.0;
};

struct MetricConfig
{
    ThdConfig thd;
    double clamp_eps = 1e-4;
};

struct MetricPack
{
    double phi_pf_deg = std::numeric_limits<double>::quiet_NaN();
    double phi_idiq_deg = std::numeric_limits<double>::quiet_NaN();
    double thd_phase_a_pct = std::numeric_limits<double>::quiet_NaN();
    double i_ripple_rms = std::numeric_limits<double>::quiet_NaN();
    double min_pulse_margin_s = std::numeric_limits<double>::quiet_NaN();
    std::array<double, 3> clamp_fraction{{0.0, 0.0, 0.0}};
    std::array<double, 3> clamp_fraction_high{{0.0, 0.0, 0.0}};
    std::array<double, 3> clamp_fraction_low{{0.0, 0.0, 0.0}};
};

class MetricAccumulator
{
public:
    MetricAccumulator(const MetricConfig& config, const InverterParams& inv_params, double timestep_s);

    void Reset();
    void OnStep(const StepSnapshot& snap);
    MetricPack Finalize() const;

private:
    MetricConfig m_config{};
    InverterParams m_inv_params{};
    double m_timestep_s = 0.0;

    int m_count = 0;
    double m_sum_p = 0.0;
    double m_sum_q = 0.0;
    double m_sum_id = 0.0;
    double m_sum_iq = 0.0;

    std::array<double, 3> m_sum_i{{0.0, 0.0, 0.0}};
    std::array<double, 3> m_sum_i2{{0.0, 0.0, 0.0}};

    std::array<int, 3> m_clamp_count{{0, 0, 0}};
    std::array<int, 3> m_clamp_high_count{{0, 0, 0}};
    std::array<int, 3> m_clamp_low_count{{0, 0, 0}};

    double m_min_pulse_margin_s = std::numeric_limits<double>::infinity();

    std::vector<double> m_thd_samples_a;
};
} // namespace sim

#endif // SIM_METRIC_PACK_H
