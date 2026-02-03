#ifndef SIM_INVERTER_SWITCHING_MODEL_H
#define SIM_INVERTER_SWITCHING_MODEL_H

#include "modulator.h"
#include "power_module.h"
#include <array>

namespace sim
{
struct PhaseVoltages
{
    double a = 0.0;
    double b = 0.0;
    double c = 0.0;
};

struct PhaseCurrents
{
    double a = 0.0;
    double b = 0.0;
    double c = 0.0;
};

struct DeviceLosses
{
    double igbt_cond_W = 0.0;
    double diode_cond_W = 0.0;
    double igbt_sw_W = 0.0;
    double diode_rr_W = 0.0;
};

struct LossBreakdown
{
    std::array<DeviceLosses, 3> phase{};
    double total_cond_W = 0.0;
    double total_sw_W = 0.0;
    double total_rr_W = 0.0;
    double total_W = 0.0;
};

struct ThermalState
{
    double case_C = 25.0;
    std::array<double, 3> igbt_C{{25.0, 25.0, 25.0}};
    std::array<double, 3> diode_C{{25.0, 25.0, 25.0}};
};

struct InverterParams
{
    double deadtime_s = 2e-6;
    double pwm_frequency_hz = 8800.0;
    double sink_temp_C = 25.0;
    double thermal_tau_s = 1.0;
    bool enable_losses = true;
};

class InverterSwitchingModel
{
public:
    InverterSwitchingModel();
    PhaseVoltages FromDuty(double vdc, const DutyCycles& duty,
                           const PhaseCurrents& currents,
                           double dt,
                           const InverterParams& params,
                           LossBreakdown* losses = nullptr,
                           ThermalState* thermal = nullptr);
    void RemoveCommonMode(PhaseVoltages& voltages) const;
    void ResetThermals(double sink_temp_C);
    void SetModuleParams(const PowerModuleParams& params);

private:
    PowerModuleParams m_module;
    ThermalState m_thermal;
};
} // namespace sim

#endif // SIM_INVERTER_SWITCHING_MODEL_H
