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

struct PwmRippleDiag
{
    bool valid = false;
    std::array<double, 3> i_start_A{{0.0, 0.0, 0.0}};
    std::array<double, 3> i_end_A{{0.0, 0.0, 0.0}};
    std::array<double, 3> i_min_A{{0.0, 0.0, 0.0}};
    std::array<double, 3> i_max_A{{0.0, 0.0, 0.0}};

    bool torque_valid = false;
    double id_start_A = 0.0;
    double iq_start_A = 0.0;
    double id_end_A = 0.0;
    double iq_end_A = 0.0;
    double id_min_A = 0.0;
    double id_max_A = 0.0;
    double iq_min_A = 0.0;
    double iq_max_A = 0.0;
    double torque_start_Nm = 0.0;
    double torque_end_Nm = 0.0;
    double torque_min_Nm = 0.0;
    double torque_max_Nm = 0.0;
};

struct InverterParams
{
    double deadtime_s = 2e-6;
    double min_on_s = 0.0;
    double min_off_s = 0.0;
    double pwm_frequency_hz = 8800.0;
    double sink_temp_C = 25.0;
    double thermal_tau_s = 1.0;
    bool enable_losses = true;

    // Phase-B (optional): intra-PWM current integration for loss inputs only.
    // Uses a simple per-phase RL model: di/dt = (v_phase_ln - bemf_phase_ln - R*i)/L
    bool integrate_currents_in_pwm = false;
    double phase_R_ohm = 0.0;
    double phase_L_H = 0.0;
    PhaseVoltages bemf_phase_ln_V{};

    // Optional: compute Id/Iq and torque ripple metrics from the intra-PWM phase current waveform.
    // This does not feed back into the motor plant yet; it is purely diagnostic.
    bool compute_torque_ripple = false;
    double elec_angle_rad = 0.0;
    double pole_pairs = 0.0;
    double flux_Wb = 0.0;
    double ld_H = 0.0;
    double lq_H = 0.0;
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
                           ThermalState* thermal = nullptr,
                           PwmRippleDiag* ripple = nullptr);
    void RemoveCommonMode(PhaseVoltages& voltages) const;
    void ResetThermals(double sink_temp_C);
    void SetModuleParams(const PowerModuleParams& params);

private:
    PowerModuleParams m_module;
    ThermalState m_thermal;
};
} // namespace sim

#endif // SIM_INVERTER_SWITCHING_MODEL_H
