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

class Modulator
{
public:
    DutyCycles GetDutyCycles() const;
};
} // namespace sim

#endif // SIM_MODULATOR_H
