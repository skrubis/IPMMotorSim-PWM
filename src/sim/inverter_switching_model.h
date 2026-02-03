#ifndef SIM_INVERTER_SWITCHING_MODEL_H
#define SIM_INVERTER_SWITCHING_MODEL_H

#include "modulator.h"

namespace sim
{
struct PhaseVoltages
{
    double a = 0.0;
    double b = 0.0;
    double c = 0.0;
};

class InverterSwitchingModel
{
public:
    PhaseVoltages FromDuty(double vdc, const DutyCycles& duty) const;
    void RemoveCommonMode(PhaseVoltages& voltages) const;
};
} // namespace sim

#endif // SIM_INVERTER_SWITCHING_MODEL_H
