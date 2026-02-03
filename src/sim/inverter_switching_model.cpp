#include "inverter_switching_model.h"

namespace sim
{
PhaseVoltages InverterSwitchingModel::FromDuty(double vdc, const DutyCycles& duty) const
{
    PhaseVoltages voltages;
    const double scale = vdc / 65536.0;
    voltages.a = scale * (static_cast<int>(duty.a) - 32768);
    voltages.b = scale * (static_cast<int>(duty.b) - 32768);
    voltages.c = scale * (static_cast<int>(duty.c) - 32768);
    return voltages;
}

void InverterSwitchingModel::RemoveCommonMode(PhaseVoltages& voltages) const
{
    const double offset = (voltages.a + voltages.b + voltages.c) / 3.0;
    voltages.a -= offset;
    voltages.b -= offset;
    voltages.c -= offset;
}
} // namespace sim
