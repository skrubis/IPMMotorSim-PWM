#include "modulator.h"

#include <algorithm>
#include <cmath>

#include "foc.h"

namespace sim
{
DutyCycles Modulator::GetDutyCycles() const
{
    DutyCycles duty;
    duty.a = FOC::DutyCycles[0];
    duty.b = FOC::DutyCycles[1];
    duty.c = FOC::DutyCycles[2];
    duty.a_norm = duty.a / 65535.0;
    duty.b_norm = duty.b / 65535.0;
    duty.c_norm = duty.c / 65535.0;
    return duty;
}

DutyCycles Modulator::ComputeFromAlphaBeta(double v_alpha, double v_beta, double vdc,
                                           ModulationMode mode, double blend,
                                           ModulatorDiag* diag) const
{
    DutyCycles duty;
    if (vdc <= 0.0)
        return duty;

    constexpr double sqrt3 = 1.7320508075688772;
    const double va = v_alpha;
    const double vb = (-0.5 * v_alpha) + ((sqrt3 / 2.0) * v_beta);
    const double vc = (-0.5 * v_alpha) - ((sqrt3 / 2.0) * v_beta);

    const double v_max = std::max({va, vb, vc});
    const double v_min = std::min({va, vb, vc});

    constexpr double pi = 3.14159265358979323846;
    const double angle = std::atan2(v_beta, v_alpha);
    double theta = angle < 0.0 ? (angle + 2.0 * pi) : angle;
    int sector = static_cast<int>(theta / (pi / 3.0)) + 1;
    if (sector > 6)
        sector = 6;

    double v_offset = 0.0;
    const double sv_offset = -0.5 * (v_max + v_min);
    bool clampMin = false;
    bool clampMax = false;
    switch (mode)
    {
        case ModulationMode::DPWMMIN:
            clampMin = true;
            break;
        case ModulationMode::DPWMMAX:
            clampMax = true;
            break;
        case ModulationMode::DPWM0:
            clampMax = (sector % 2) == 1;
            clampMin = !clampMax;
            break;
        case ModulationMode::DPWM1:
            clampMin = (sector % 2) == 1;
            clampMax = !clampMin;
            break;
        case ModulationMode::DPWM2:
        case ModulationMode::DPWM3:
        {
            const double shift = (mode == ModulationMode::DPWM2) ? (-pi / 6.0) : 0.0;
            const double harmonic = (mode == ModulationMode::DPWM3) ? 6.0 : 3.0;
            const double sel = std::sin(harmonic * (theta - shift));
            clampMax = (sel >= 0.0);
            clampMin = !clampMax;
            break;
        }
        case ModulationMode::SVPWM:
        case ModulationMode::Firmware:
        default:
            v_offset = sv_offset;
            break;
    }

    if (clampMin || clampMax)
    {
        const double clamp_offset = clampMin ? ((-0.5 * vdc) - v_min) : ((0.5 * vdc) - v_max);
        const double lambda = std::clamp(blend, 0.0, 1.0);
        v_offset = (1.0 - lambda) * sv_offset + (lambda * clamp_offset);
    }

    auto toDuty = [vdc, v_offset](double v_phase)
    {
        const double d = (v_phase + v_offset) / vdc + 0.5;
        return std::clamp(d, 0.0, 1.0);
    };

    duty.a_norm = toDuty(va);
    duty.b_norm = toDuty(vb);
    duty.c_norm = toDuty(vc);
    duty.a = static_cast<uint16_t>(std::lround(duty.a_norm * 65535.0));
    duty.b = static_cast<uint16_t>(std::lround(duty.b_norm * 65535.0));
    duty.c = static_cast<uint16_t>(std::lround(duty.c_norm * 65535.0));

    if (diag)
    {
        const double theta_s = theta - (sector - 1) * (pi / 3.0);
        const double vref = std::sqrt((v_alpha * v_alpha) + (v_beta * v_beta));
        double t1 = (sqrt3 / vdc) * vref * std::sin((pi / 3.0) - theta_s);
        double t2 = (sqrt3 / vdc) * vref * std::sin(theta_s);
        double t0 = 1.0 - t1 - t2;
        if (t1 < 0.0) t1 = 0.0;
        if (t2 < 0.0) t2 = 0.0;
        if (t0 < 0.0) t0 = 0.0;

        diag->zero_seq = v_offset / vdc;
        diag->sector = sector;
        diag->t1 = t1;
        diag->t2 = t2;
        diag->t0 = t0;
        diag->clamp_leg = -1;
        diag->clamp_polarity = 0;

        if (clampMin)
        {
            diag->clamp_leg = (va <= vb && va <= vc) ? 0 : ((vb <= vc) ? 1 : 2);
            diag->clamp_polarity = -1;
        }
        else if (clampMax)
        {
            diag->clamp_leg = (va >= vb && va >= vc) ? 0 : ((vb >= vc) ? 1 : 2);
            diag->clamp_polarity = 1;
        }
    }

    return duty;
}
} // namespace sim
