#include "inverter_switching_model.h"
#include "pwm_timeline.h"

#include <algorithm>
#include <cmath>

namespace sim
{
namespace
{
enum class ConductionElement
{
    HsIgbt,
    HsDiode,
    LsIgbt,
    LsDiode
};

static ConductionElement ConductionFor(LegGateState state, double current_A)
{
    if (state == LegGateState::HsOn)
        return current_A >= 0.0 ? ConductionElement::HsIgbt : ConductionElement::HsDiode;
    if (state == LegGateState::LsOn)
        return current_A >= 0.0 ? ConductionElement::LsDiode : ConductionElement::LsIgbt;
    return current_A >= 0.0 ? ConductionElement::LsDiode : ConductionElement::HsDiode;
}

static void GateBools(LegGateState state, bool& hs_on, bool& ls_on)
{
    hs_on = (state == LegGateState::HsOn);
    ls_on = (state == LegGateState::LsOn);
}

static double IntegrateCurrentRL(double i0_A, double v_phase_ln_V, double bemf_ln_V,
                                 double r_ohm, double l_H, double dt_s)
{
    if (dt_s <= 0.0 || l_H <= 0.0)
        return i0_A;

    if (r_ohm <= 0.0)
    {
        const double di = (v_phase_ln_V - bemf_ln_V) * (dt_s / l_H);
        return i0_A + di;
    }

    const double a = r_ohm / l_H;
    const double v = v_phase_ln_V - bemf_ln_V;
    const double i_ss = v / r_ohm;
    const double e = std::exp(-a * dt_s);
    return i_ss + (i0_A - i_ss) * e;
}

static void CurrentsToDq(double ia, double ib, double theta_rad, double& id_A, double& iq_A)
{
    constexpr double sqrt3 = 1.7320508075688772;
    const double i_alpha = ia;
    const double i_beta = (ia + 2.0 * ib) / sqrt3;
    const double c = std::cos(theta_rad);
    const double s = std::sin(theta_rad);
    id_A = (i_alpha * c) + (i_beta * s);
    iq_A = (-i_alpha * s) + (i_beta * c);
}

static double TorqueNm(double pole_pairs, double flux_Wb, double ld_H, double lq_H, double id_A, double iq_A)
{
    return 1.5 * pole_pairs * ((flux_Wb * iq_A) + ((ld_H - lq_H) * id_A * iq_A));
}

static double PhaseVoltageFor(double vdc, ConductionElement elem, double absI,
                              const PowerModuleParams& module, double tj_igbt_C, double tj_diode_C,
                              double& e_igbt_cond_J, double& e_diode_cond_J, double dt_s)
{
    const double vce = IgbtVceSat(module, absI, tj_igbt_C);
    const double vf = DiodeVf(module, absI, tj_diode_C);

    switch (elem)
    {
        case ConductionElement::HsIgbt:
            e_igbt_cond_J += absI * vce * dt_s;
            return (0.5 * vdc) - vce;
        case ConductionElement::HsDiode:
            e_diode_cond_J += absI * vf * dt_s;
            return (0.5 * vdc) + vf;
        case ConductionElement::LsIgbt:
            e_igbt_cond_J += absI * vce * dt_s;
            return (-0.5 * vdc) + vce;
        case ConductionElement::LsDiode:
        default:
            e_diode_cond_J += absI * vf * dt_s;
            return (-0.5 * vdc) - vf;
    }
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
                                               ThermalState* thermal,
                                               PwmRippleDiag* ripple)
{
    PhaseVoltages voltages;
    LossBreakdown localLoss{};
    PwmRippleDiag localRipple{};

    if (vdc <= 0.0 || params.pwm_frequency_hz <= 0.0)
    {
        if (losses)
            *losses = localLoss;
        if (thermal)
            *thermal = m_thermal;
        if (ripple)
            *ripple = localRipple;
        return voltages;
    }

    const double pwm_period_s = 1.0 / params.pwm_frequency_hz;

    const std::vector<TimelineSegment> timeline =
        BuildCenterAlignedTimeline(duty, pwm_period_s, params.deadtime_s,
                                   params.min_on_s, params.min_off_s);

    std::array<double, 3> v_int{{0.0, 0.0, 0.0}};
    std::array<double, 3> e_igbt_cond_J{{0.0, 0.0, 0.0}};
    std::array<double, 3> e_diode_cond_J{{0.0, 0.0, 0.0}};
    std::array<double, 3> e_igbt_sw_J{{0.0, 0.0, 0.0}};
    std::array<double, 3> e_diode_rr_J{{0.0, 0.0, 0.0}};

    const std::array<double, 3> i_start{{currents.a, currents.b, currents.c}};
    localRipple.i_start_A = i_start;
    localRipple.i_end_A = i_start;
    localRipple.i_min_A = i_start;
    localRipple.i_max_A = i_start;
    std::array<double, 3> i_seg = i_start;
    const bool doRipple = params.integrate_currents_in_pwm &&
                          params.phase_L_H > 0.0 &&
                          pwm_period_s > 0.0;
    const std::array<double, 3> bemf_ln{{params.bemf_phase_ln_V.a,
                                         params.bemf_phase_ln_V.b,
                                         params.bemf_phase_ln_V.c}};

    const bool doTorque = doRipple &&
                          params.compute_torque_ripple &&
                          std::isfinite(params.elec_angle_rad) &&
                          params.pole_pairs > 0.0 &&
                          params.ld_H > 0.0 &&
                          params.lq_H > 0.0;

    if (doTorque)
    {
        CurrentsToDq(i_start[0], i_start[1], params.elec_angle_rad, localRipple.id_start_A, localRipple.iq_start_A);
        localRipple.id_end_A = localRipple.id_start_A;
        localRipple.iq_end_A = localRipple.iq_start_A;
        localRipple.id_min_A = localRipple.id_start_A;
        localRipple.id_max_A = localRipple.id_start_A;
        localRipple.iq_min_A = localRipple.iq_start_A;
        localRipple.iq_max_A = localRipple.iq_start_A;
        localRipple.torque_start_Nm = TorqueNm(params.pole_pairs, params.flux_Wb, params.ld_H, params.lq_H,
                                              localRipple.id_start_A, localRipple.iq_start_A);
        localRipple.torque_end_Nm = localRipple.torque_start_Nm;
        localRipple.torque_min_Nm = localRipple.torque_start_Nm;
        localRipple.torque_max_Nm = localRipple.torque_start_Nm;
    }

    std::array<bool, 3> prev_hs{{false, false, false}};
    std::array<bool, 3> prev_ls{{true, true, true}};
    std::array<ConductionElement, 3> prev_elem{{ConductionElement::LsDiode, ConductionElement::LsDiode, ConductionElement::LsDiode}};
    bool have_prev = false;

    for (const TimelineSegment& seg : timeline)
    {
        const double dt_seg = seg.t1_s - seg.t0_s;
        if (dt_seg <= 0.0)
            continue;

        std::array<double, 3> v_raw{{0.0, 0.0, 0.0}};
        std::array<double, 3> i0{{0.0, 0.0, 0.0}};
        std::array<double, 3> i1{{0.0, 0.0, 0.0}};
        std::array<ConductionElement, 3> elem{{ConductionElement::LsDiode, ConductionElement::LsDiode, ConductionElement::LsDiode}};
        std::array<bool, 3> hs_on{{false, false, false}};
        std::array<bool, 3> ls_on{{true, true, true}};

        for (size_t idx = 0; idx < 3; ++idx)
        {
            const double iA = doRipple ? i_seg[idx] : i_start[idx];
            i0[idx] = iA;
            const double absI = std::abs(iA);
            const double tj_igbt = m_thermal.igbt_C[idx];
            const double tj_diode = m_thermal.diode_C[idx];

            const LegGateState state = seg.leg[idx];
            elem[idx] = ConductionFor(state, iA);

            v_raw[idx] = PhaseVoltageFor(vdc, elem[idx], absI, m_module, tj_igbt, tj_diode,
                                         e_igbt_cond_J[idx], e_diode_cond_J[idx], dt_seg);

            GateBools(state, hs_on[idx], ls_on[idx]);

            if (have_prev)
            {
                if (prev_hs[idx] != hs_on[idx])
                {
                    const double e = hs_on[idx] ? IgbtEon(m_module, absI, vdc, tj_igbt)
                                                : IgbtEoff(m_module, absI, vdc, tj_igbt);
                    e_igbt_sw_J[idx] += e;
                }
                if (prev_ls[idx] != ls_on[idx])
                {
                    const double e = ls_on[idx] ? IgbtEon(m_module, absI, vdc, tj_igbt)
                                                : IgbtEoff(m_module, absI, vdc, tj_igbt);
                    e_igbt_sw_J[idx] += e;
                }

                const bool prev_diode = (prev_elem[idx] == ConductionElement::HsDiode) ||
                                        (prev_elem[idx] == ConductionElement::LsDiode);
                const bool now_igbt = (elem[idx] == ConductionElement::HsIgbt) || (elem[idx] == ConductionElement::LsIgbt);
                if (prev_diode && now_igbt)
                    e_diode_rr_J[idx] += DiodeErr(m_module, absI, vdc, tj_diode);
            }

            prev_hs[idx] = hs_on[idx];
            prev_ls[idx] = ls_on[idx];
            prev_elem[idx] = elem[idx];
        }

        for (size_t idx = 0; idx < 3; ++idx)
            v_int[idx] += v_raw[idx] * dt_seg;

        if (doRipple)
        {
            const double v_cm = (v_raw[0] + v_raw[1] + v_raw[2]) / 3.0;
            for (size_t idx = 0; idx < 3; ++idx)
            {
                const double v_ln = v_raw[idx] - v_cm;
                i1[idx] = IntegrateCurrentRL(i0[idx], v_ln, bemf_ln[idx],
                                             params.phase_R_ohm, params.phase_L_H, dt_seg);
                i_seg[idx] = i1[idx];

                localRipple.i_min_A[idx] = std::min(localRipple.i_min_A[idx], std::min(i0[idx], i1[idx]));
                localRipple.i_max_A[idx] = std::max(localRipple.i_max_A[idx], std::max(i0[idx], i1[idx]));
                localRipple.i_end_A[idx] = i1[idx];

                const double abs0 = std::abs(i0[idx]);
                const double abs1 = std::abs(i1[idx]);
                const double absAvg = 0.5 * (abs0 + abs1);
                if (absAvg != abs0)
                {
                    const double tj_igbt = m_thermal.igbt_C[idx];
                    const double tj_diode = m_thermal.diode_C[idx];
                    const double vce = IgbtVceSat(m_module, absAvg, tj_igbt);
                    const double vf = DiodeVf(m_module, absAvg, tj_diode);
                    const bool isIgbt = (elem[idx] == ConductionElement::HsIgbt) || (elem[idx] == ConductionElement::LsIgbt);
                    const double drop = isIgbt ? vce : vf;
                    const double deltaAbs = absAvg - abs0;
                    if (isIgbt)
                        e_igbt_cond_J[idx] += deltaAbs * drop * dt_seg;
                    else
                        e_diode_cond_J[idx] += deltaAbs * drop * dt_seg;
                }
            }

            if (doTorque)
            {
                double id0 = 0.0, iq0 = 0.0, id1 = 0.0, iq1 = 0.0;
                CurrentsToDq(i0[0], i0[1], params.elec_angle_rad, id0, iq0);
                CurrentsToDq(i1[0], i1[1], params.elec_angle_rad, id1, iq1);

                localRipple.id_min_A = std::min(localRipple.id_min_A, std::min(id0, id1));
                localRipple.id_max_A = std::max(localRipple.id_max_A, std::max(id0, id1));
                localRipple.iq_min_A = std::min(localRipple.iq_min_A, std::min(iq0, iq1));
                localRipple.iq_max_A = std::max(localRipple.iq_max_A, std::max(iq0, iq1));
                localRipple.id_end_A = id1;
                localRipple.iq_end_A = iq1;

                const double t0 = TorqueNm(params.pole_pairs, params.flux_Wb, params.ld_H, params.lq_H, id0, iq0);
                const double t1 = TorqueNm(params.pole_pairs, params.flux_Wb, params.ld_H, params.lq_H, id1, iq1);
                localRipple.torque_min_Nm = std::min(localRipple.torque_min_Nm, std::min(t0, t1));
                localRipple.torque_max_Nm = std::max(localRipple.torque_max_Nm, std::max(t0, t1));
                localRipple.torque_end_Nm = t1;
            }
        }

        have_prev = true;
    }

    localRipple.valid = doRipple;
    localRipple.torque_valid = doTorque;

    voltages.a = v_int[0] / pwm_period_s;
    voltages.b = v_int[1] / pwm_period_s;
    voltages.c = v_int[2] / pwm_period_s;

    const double p_scale = params.pwm_frequency_hz;
    for (size_t idx = 0; idx < 3; ++idx)
    {
        localLoss.phase[idx].igbt_cond_W = e_igbt_cond_J[idx] * p_scale;
        localLoss.phase[idx].diode_cond_W = e_diode_cond_J[idx] * p_scale;
        localLoss.phase[idx].igbt_sw_W = e_igbt_sw_J[idx] * p_scale;
        localLoss.phase[idx].diode_rr_W = e_diode_rr_J[idx] * p_scale;
    }

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
    if (ripple)
        *ripple = localRipple;

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
