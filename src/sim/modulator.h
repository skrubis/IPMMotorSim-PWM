#ifndef SIM_MODULATOR_H
#define SIM_MODULATOR_H

#include <cstdint>

namespace sim
{
struct DutyCycles
{
    uint16_t a = 0;
    uint16_t b = 0;
    uint16_t c = 0;
    double a_norm = 0.0;
    double b_norm = 0.0;
    double c_norm = 0.0;
};

enum class ModulationMode
{
    Firmware,
    SVPWM,
    DPWMMIN,
    DPWMMAX,
    DPWM0,
    DPWM1
};

struct ModulatorDiag
{
    double zero_seq = 0.0;
    int clamp_leg = -1;      // 0=A, 1=B, 2=C
    int clamp_polarity = 0;  // -1 low, +1 high
    int sector = -1;
    double t1 = 0.0;
    double t2 = 0.0;
    double t0 = 0.0;
};

class Modulator
{
public:
    DutyCycles GetDutyCycles() const;
    DutyCycles ComputeFromAlphaBeta(double v_alpha, double v_beta, double vdc,
                                    ModulationMode mode, double blend = 1.0,
                                    ModulatorDiag* diag = nullptr) const;
};
} // namespace sim

#endif // SIM_MODULATOR_H
