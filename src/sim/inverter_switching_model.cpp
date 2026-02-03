#include "inverter_switching_model.h"

#include <algorithm>
#include <cmath>

namespace sim
{
namespace
{
struct PhaseResult
{
    double v_phase = 0.0;
    DeviceLosses losses;
};

PhaseResult ComputePhase(double vdc, double duty, double current_A, double pwm_period_s,
                         double deadtime_s, const PowerModuleParams& module,
                         double tj_igbt_C, double tj_diode_C)
{
    PhaseResult result;
    if (vdc <= 0.0 || pwm_period_s <= 0.0)
        return result;

    const double d = std::clamp(duty, 0.0, 1.0);
    const double dead = std::clamp(deadtime_s, 0.0, pwm_period_s * 0.49);

    const double t_hs = std::max(0.0, d * pwm_period_s - dead);
    const double t_ls = std::max(0.0, (1.0 - d) * pwm_period_s - dead);
    const double t_dt = pwm_period_s - t_hs - t_ls;

    const double absI = std::abs(current_A);
    const double vce = IgbtVceSat(module, absI, tj_igbt_C);
    const double vf = DiodeVf(module, absI, tj_diode_C);

    if (current_A >= 0.0)
    {
        const double v_hs = (0.5 * vdc) - vce;
        const double v_ls = (-0.5 * vdc) - vf;
        result.v_phase = ((t_hs * v_hs) + ((t_ls + t_dt) * v_ls)) / pwm_period_s;

        result.losses.igbt_cond_W = absI * vce * (t_hs / pwm_period_s);
        result.losses.diode_cond_W = absI * vf * ((t_ls + t_dt) / pwm_period_s);
    }
    else
    {
        const double v_hs = (0.5 * vdc) + vf;
        const double v_ls = (-0.5 * vdc) + vce;
        result.v_phase = (((t_hs + t_dt) * v_hs) + (t_ls * v_ls)) / pwm_period_s;

        result.losses.igbt_cond_W = absI * vce * (t_ls / pwm_period_s);
        result.losses.diode_cond_W = absI * vf * ((t_hs + t_dt) / pwm_period_s);
    }

    if (d > 0.0 && d < 1.0)
    {
        const double eon = IgbtEon(module, absI, vdc, tj_igbt_C);
        const double eoff = IgbtEoff(module, absI, vdc, tj_igbt_C);
        const double err = DiodeErr(module, absI, vdc, tj_diode_C);

        result.losses.igbt_sw_W = (eon + eoff) * 2.0; // two IGBTs switch per leg
        result.losses.diode_rr_W = err;
    }

    return result;
}
} // namespace

InverterSwitchingModel::InverterSwitchingModel()
    : m_module(PM300CLA060())
{
    ResetThermals(25.0);
}

PhaseVoltages InverterSwitchingModel::FromDuty(double vdc, const DutyCycles& duty,
                                               const PhaseCurrents& currents,
                                               double dt,
                                               const InverterParams& params,
                                               LossBreakdown* losses,
                                               ThermalState* thermal)
{
    PhaseVoltages voltages;
    LossBreakdown localLoss{};

    if (vdc <= 0.0 || params.pwm_frequency_hz <= 0.0)
    {
        if (losses)
            *losses = localLoss;
        if (thermal)
            *thermal = m_thermal;
        return voltages;
    }

    const double pwm_period_s = 1.0 / params.pwm_frequency_hz;

    const PhaseResult resA = ComputePhase(vdc, duty.a_norm, currents.a, pwm_period_s,
                                          params.deadtime_s, m_module,
                                          m_thermal.igbt_C[0], m_thermal.diode_C[0]);
    const PhaseResult resB = ComputePhase(vdc, duty.b_norm, currents.b, pwm_period_s,
                                          params.deadtime_s, m_module,
                                          m_thermal.igbt_C[1], m_thermal.diode_C[1]);
    const PhaseResult resC = ComputePhase(vdc, duty.c_norm, currents.c, pwm_period_s,
                                          params.deadtime_s, m_module,
                                          m_thermal.igbt_C[2], m_thermal.diode_C[2]);

    voltages.a = resA.v_phase;
    voltages.b = resB.v_phase;
    voltages.c = resC.v_phase;

    const double p_scale = params.pwm_frequency_hz;
    localLoss.phase[0].igbt_cond_W = resA.losses.igbt_cond_W;
    localLoss.phase[0].diode_cond_W = resA.losses.diode_cond_W;
    localLoss.phase[0].igbt_sw_W = resA.losses.igbt_sw_W * p_scale;
    localLoss.phase[0].diode_rr_W = resA.losses.diode_rr_W * p_scale;

    localLoss.phase[1].igbt_cond_W = resB.losses.igbt_cond_W;
    localLoss.phase[1].diode_cond_W = resB.losses.diode_cond_W;
    localLoss.phase[1].igbt_sw_W = resB.losses.igbt_sw_W * p_scale;
    localLoss.phase[1].diode_rr_W = resB.losses.diode_rr_W * p_scale;

    localLoss.phase[2].igbt_cond_W = resC.losses.igbt_cond_W;
    localLoss.phase[2].diode_cond_W = resC.losses.diode_cond_W;
    localLoss.phase[2].igbt_sw_W = resC.losses.igbt_sw_W * p_scale;
    localLoss.phase[2].diode_rr_W = resC.losses.diode_rr_W * p_scale;

    for (const auto& phase : localLoss.phase)
    {
        localLoss.total_cond_W += (phase.igbt_cond_W + phase.diode_cond_W);
        localLoss.total_sw_W += phase.igbt_sw_W;
        localLoss.total_rr_W += phase.diode_rr_W;
    }
    localLoss.total_W = localLoss.total_cond_W + localLoss.total_sw_W + localLoss.total_rr_W;

    if (params.enable_losses)
    {
        const double alpha = params.thermal_tau_s > 0.0
                                 ? std::clamp(dt / params.thermal_tau_s, 0.0, 1.0)
                                 : 1.0;
        const double tcase_target = params.sink_temp_C + localLoss.total_W * m_module.rth_cs_C_per_W;
        m_thermal.case_C += (tcase_target - m_thermal.case_C) * alpha;

        for (size_t idx = 0; idx < localLoss.phase.size(); ++idx)
        {
            const double p_igbt = localLoss.phase[idx].igbt_cond_W + localLoss.phase[idx].igbt_sw_W;
            const double p_diode = localLoss.phase[idx].diode_cond_W + localLoss.phase[idx].diode_rr_W;

            const double p_igbt_per_switch = p_igbt * 0.5;
            const double p_diode_per_switch = p_diode * 0.5;

            const double t_igbt_target = m_thermal.case_C + p_igbt_per_switch * m_module.rth_jc_igbt_C_per_W;
            const double t_diode_target = m_thermal.case_C + p_diode_per_switch * m_module.rth_jc_diode_C_per_W;

            m_thermal.igbt_C[idx] += (t_igbt_target - m_thermal.igbt_C[idx]) * alpha;
            m_thermal.diode_C[idx] += (t_diode_target - m_thermal.diode_C[idx]) * alpha;
        }
    }

    if (losses)
        *losses = localLoss;
    if (thermal)
        *thermal = m_thermal;

    return voltages;
}

void InverterSwitchingModel::RemoveCommonMode(PhaseVoltages& voltages) const
{
    const double offset = (voltages.a + voltages.b + voltages.c) / 3.0;
    voltages.a -= offset;
    voltages.b -= offset;
    voltages.c -= offset;
}

void InverterSwitchingModel::ResetThermals(double sink_temp_C)
{
    m_thermal.case_C = sink_temp_C;
    for (double& temp : m_thermal.igbt_C)
        temp = sink_temp_C;
    for (double& temp : m_thermal.diode_C)
        temp = sink_temp_C;
}

void InverterSwitchingModel::SetModuleParams(const PowerModuleParams& params)
{
    m_module = params;
}
} // namespace sim
