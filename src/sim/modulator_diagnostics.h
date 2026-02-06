#ifndef SIM_MODULATOR_DIAGNOSTICS_H
#define SIM_MODULATOR_DIAGNOSTICS_H

#include <array>
#include <string>
#include <vector>

#include "modulator.h"

namespace sim
{
struct ClampSample
{
    double theta_rad = 0.0;
    int clamp_leg = -1;      // 0=A,1=B,2=C, -1 = none
    int clamp_polarity = 0;  // -1 low, +1 high, 0 none
    double duty_a = 0.0;
    double duty_b = 0.0;
    double duty_c = 0.0;
};

struct ClampStats
{
    std::array<double, 3> clamp_fraction{{0.0, 0.0, 0.0}};
    std::array<double, 3> clamp_fraction_high{{0.0, 0.0, 0.0}};
    std::array<double, 3> clamp_fraction_low{{0.0, 0.0, 0.0}};
    std::vector<double> segment_lengths_deg;
};

std::vector<ClampSample> SampleClampPattern(ModulationMode mode,
                                            double modulation_index,
                                            double vdc,
                                            int samples);

ClampStats ComputeClampStats(const std::vector<ClampSample>& samples);

bool WriteClampSamplesCsv(const std::string& path, const std::vector<ClampSample>& samples);
} // namespace sim

#endif // SIM_MODULATOR_DIAGNOSTICS_H
