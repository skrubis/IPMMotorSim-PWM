#include "modulator_diagnostics.h"

#include <cmath>
#include <fstream>

namespace sim
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
} // namespace

std::vector<ClampSample> SampleClampPattern(ModulationMode mode,
                                            double modulation_index,
                                            double vdc,
                                            int samples)
{
    std::vector<ClampSample> out;
    if (samples <= 0 || vdc <= 0.0)
        return out;

    Modulator modulator;
    out.reserve(static_cast<size_t>(samples));

    const double m = std::max(0.0, std::min(modulation_index, 1.0));
    const double vref = 0.5 * vdc * m;

    // Sample mid-bin angles to avoid exact sector-boundary ties (which can cause clamp-leg jitter
    // due to deterministic tie-breaking).
    for (int i = 0; i < samples; ++i)
    {
        const double theta = (2.0 * kPi) * ((static_cast<double>(i) + 0.5) / static_cast<double>(samples));
        const double v_alpha = vref * std::cos(theta);
        const double v_beta = vref * std::sin(theta);
        ModulatorDiag diag{};
        DutyCycles duty = modulator.ComputeFromAlphaBeta(v_alpha, v_beta, vdc, mode, 1.0, &diag);

        ClampSample sample{};
        sample.theta_rad = theta;
        sample.clamp_leg = diag.clamp_leg;
        sample.clamp_polarity = diag.clamp_polarity;
        sample.duty_a = duty.a_norm;
        sample.duty_b = duty.b_norm;
        sample.duty_c = duty.c_norm;
        out.push_back(sample);
    }

    return out;
}

ClampStats ComputeClampStats(const std::vector<ClampSample>& samples)
{
    ClampStats stats;
    if (samples.empty())
        return stats;

    const double inv_count = 1.0 / static_cast<double>(samples.size());
    auto effectivePol = [](const ClampSample& s) -> int
    {
        if (s.clamp_leg < 0 || s.clamp_leg >= 3)
            return 0;
        return s.clamp_polarity;
    };

    int last_pol = effectivePol(samples.front());
    int seg_len = 1;

    for (size_t i = 0; i < samples.size(); ++i)
    {
        const ClampSample& s = samples[i];
        if (s.clamp_leg >= 0 && s.clamp_leg < 3)
        {
            stats.clamp_fraction[static_cast<size_t>(s.clamp_leg)] += inv_count;
            if (s.clamp_polarity > 0)
                stats.clamp_fraction_high[static_cast<size_t>(s.clamp_leg)] += inv_count;
            else if (s.clamp_polarity < 0)
                stats.clamp_fraction_low[static_cast<size_t>(s.clamp_leg)] += inv_count;
        }

        if (i == 0)
            continue;

        const int pol = effectivePol(s);
        if (pol == last_pol)
        {
            ++seg_len;
        }
        else
        {
            if (last_pol != 0)
            {
                const double seg_deg = 360.0 * (static_cast<double>(seg_len) / static_cast<double>(samples.size()));
                // Ignore tiny boundary-jitter segments that can occur exactly at sector boundaries.
                if (seg_deg >= 0.5)
                    stats.segment_lengths_deg.push_back(seg_deg);
            }
            seg_len = 1;
            last_pol = pol;
        }
    }

    if (seg_len > 0)
    {
        if (last_pol != 0)
        {
            const double seg_deg = 360.0 * (static_cast<double>(seg_len) / static_cast<double>(samples.size()));
            if (seg_deg >= 0.5)
                stats.segment_lengths_deg.push_back(seg_deg);
        }
    }

    return stats;
}

bool WriteClampSamplesCsv(const std::string& path, const std::vector<ClampSample>& samples)
{
    std::ofstream file(path);
    if (!file.is_open())
        return false;

    file << "theta_rad,clamp_leg,clamp_polarity,duty_a,duty_b,duty_c\n";
    for (const auto& s : samples)
    {
        file << s.theta_rad << "," << s.clamp_leg << "," << s.clamp_polarity << ","
             << s.duty_a << "," << s.duty_b << "," << s.duty_c << "\n";
    }
    return true;
}
} // namespace sim
