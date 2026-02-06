#include "sim/metrics/metric_pack.h"

#include <algorithm>
#include <cmath>

#include "pwm_timeline.h"

namespace sim
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kSqrt3 = 1.7320508075688772;

double WrapDeg(double deg)
{
    while (deg > 180.0)
        deg -= 360.0;
    while (deg < -180.0)
        deg += 360.0;
    return deg;
}

double ComputeThdPct(const std::vector<double>& samples, double sample_rate_hz, double fundamental_hz)
{
    if (samples.size() < 8 || sample_rate_hz <= 0.0 || fundamental_hz <= 0.0)
        return std::numeric_limits<double>::quiet_NaN();

    const size_t n = samples.size();
    double mean_sq = 0.0;
    for (double x : samples)
        mean_sq += x * x;
    mean_sq /= static_cast<double>(n);
    if (mean_sq <= 0.0)
        return 0.0;

    const double k_real = (fundamental_hz * static_cast<double>(n)) / sample_rate_hz;
    const int k = std::max(1, static_cast<int>(std::lround(k_real)));
    const double w = (2.0 * kPi * static_cast<double>(k)) / static_cast<double>(n);
    const double cos_w = std::cos(w);
    const double sin_w = std::sin(w);

    double s_prev = 0.0;
    double s_prev2 = 0.0;
    for (double x : samples)
    {
        const double s = x + (2.0 * cos_w * s_prev) - s_prev2;
        s_prev2 = s_prev;
        s_prev = s;
    }
    const double real = s_prev - (s_prev2 * cos_w);
    const double imag = s_prev2 * sin_w;
    const double mag = std::sqrt((real * real) + (imag * imag));

    const double rms_total = std::sqrt(mean_sq);
    const double rms_fundamental = (mag / static_cast<double>(n)) * std::sqrt(2.0);
    if (rms_fundamental <= 1e-12)
        return 0.0;

    const double rms_harm = std::sqrt(std::max(0.0, (rms_total * rms_total) - (rms_fundamental * rms_fundamental)));
    return 100.0 * (rms_harm / rms_fundamental);
}
} // namespace

MetricAccumulator::MetricAccumulator(const MetricConfig& config, const InverterParams& inv_params, double timestep_s)
    : m_config(config)
    , m_inv_params(inv_params)
    , m_timestep_s(timestep_s)
{
    Reset();
}

void MetricAccumulator::Reset()
{
    m_count = 0;
    m_sum_p = 0.0;
    m_sum_q = 0.0;
    m_sum_id = 0.0;
    m_sum_iq = 0.0;
    m_sum_i = {0.0, 0.0, 0.0};
    m_sum_i2 = {0.0, 0.0, 0.0};
    m_clamp_count = {0, 0, 0};
    m_clamp_high_count = {0, 0, 0};
    m_clamp_low_count = {0, 0, 0};
    m_min_pulse_margin_s = std::numeric_limits<double>::infinity();
    m_thd_samples_a.clear();
    if (m_config.thd.mode == ThdMode::ControlStepProxy && m_config.thd.samples > 0)
        m_thd_samples_a.reserve(static_cast<size_t>(m_config.thd.samples));
}

void MetricAccumulator::OnStep(const StepSnapshot& snap)
{
    const double va = snap.voltages_ln.a;
    const double vb = snap.voltages_ln.b;
    const double ia = snap.phase_currents.a;
    const double ib = snap.phase_currents.b;

    const double v_alpha = va;
    const double v_beta = (va + 2.0 * vb) / kSqrt3;
    const double i_alpha = ia;
    const double i_beta = (ia + 2.0 * ib) / kSqrt3;

    const double p_inst = 1.5 * ((v_alpha * i_alpha) + (v_beta * i_beta));
    const double q_inst = 1.5 * ((v_beta * i_alpha) - (v_alpha * i_beta));

    m_sum_p += p_inst;
    m_sum_q += q_inst;
    m_sum_id += snap.motor.id;
    m_sum_iq += snap.motor.iq;

    m_sum_i[0] += snap.phase_currents.a;
    m_sum_i[1] += snap.phase_currents.b;
    m_sum_i[2] += snap.phase_currents.c;
    m_sum_i2[0] += snap.phase_currents.a * snap.phase_currents.a;
    m_sum_i2[1] += snap.phase_currents.b * snap.phase_currents.b;
    m_sum_i2[2] += snap.phase_currents.c * snap.phase_currents.c;

    const double eps = m_config.clamp_eps;
    const double duties[3] = {snap.duty.a_norm, snap.duty.b_norm, snap.duty.c_norm};
    for (int idx = 0; idx < 3; ++idx)
    {
        if (duties[idx] <= eps)
        {
            ++m_clamp_count[idx];
            ++m_clamp_low_count[idx];
        }
        else if (duties[idx] >= (1.0 - eps))
        {
            ++m_clamp_count[idx];
            ++m_clamp_high_count[idx];
        }
    }

    if (snap.pwm_enabled && m_inv_params.pwm_frequency_hz > 0.0)
    {
        const double period_s = 1.0 / m_inv_params.pwm_frequency_hz;
        for (int idx = 0; idx < 3; ++idx)
        {
            const double ton = ClampDutyToTon(duties[idx], period_s,
                                              m_inv_params.deadtime_s,
                                              m_inv_params.min_on_s,
                                              m_inv_params.min_off_s);
            const double toff = period_s - ton;
            const double on_req = m_inv_params.deadtime_s + m_inv_params.min_on_s;
            const double off_req = m_inv_params.deadtime_s + m_inv_params.min_off_s;
            const double margin = std::min(ton - on_req, toff - off_req);
            m_min_pulse_margin_s = std::min(m_min_pulse_margin_s, margin);
        }
    }

    if (m_config.thd.mode == ThdMode::ControlStepProxy &&
        static_cast<int>(m_thd_samples_a.size()) < m_config.thd.samples)
    {
        m_thd_samples_a.push_back(snap.phase_currents.a);
    }

    ++m_count;
}

MetricPack MetricAccumulator::Finalize() const
{
    MetricPack out{};
    if (m_count <= 0)
        return out;

    const double inv_count = 1.0 / static_cast<double>(m_count);
    const double p = m_sum_p * inv_count;
    const double q = m_sum_q * inv_count;
    out.phi_pf_deg = WrapDeg(std::atan2(q, p) * (180.0 / kPi));

    const double id = m_sum_id * inv_count;
    const double iq = m_sum_iq * inv_count;
    out.phi_idiq_deg = WrapDeg(std::atan2(id, iq) * (180.0 / kPi));

    double ripple_rms_sum = 0.0;
    for (int idx = 0; idx < 3; ++idx)
    {
        const double mean = m_sum_i[idx] * inv_count;
        const double mean_sq = m_sum_i2[idx] * inv_count;
        const double var = std::max(0.0, mean_sq - (mean * mean));
        ripple_rms_sum += std::sqrt(var);
        out.clamp_fraction[idx] = static_cast<double>(m_clamp_count[idx]) * inv_count;
        out.clamp_fraction_high[idx] = static_cast<double>(m_clamp_high_count[idx]) * inv_count;
        out.clamp_fraction_low[idx] = static_cast<double>(m_clamp_low_count[idx]) * inv_count;
    }
    out.i_ripple_rms = ripple_rms_sum / 3.0;

    if (std::isfinite(m_min_pulse_margin_s))
        out.min_pulse_margin_s = m_min_pulse_margin_s;

    if (m_config.thd.mode == ThdMode::ControlStepProxy && !m_thd_samples_a.empty())
    {
        const double fs = m_timestep_s > 0.0 ? (1.0 / m_timestep_s) : 0.0;
        out.thd_phase_a_pct = ComputeThdPct(m_thd_samples_a, fs, m_config.thd.fundamental_hz);
    }

    return out;
}
} // namespace sim
