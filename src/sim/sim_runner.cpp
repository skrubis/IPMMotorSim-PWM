#include "sim_runner.h"

#include <algorithm>
#include <cmath>

#include "inc_encoder.h"
#include "my_math.h"
#include "params.h"

namespace sim
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPiCont = 65536.0;
constexpr double kSqrt3 = 1.7320508075688772;
} // namespace

void SimRunner::Reset(const SimInit& init)
{
    m_state.motor = init.motor;
    m_state.time_s = init.time_s;
    m_state.old_va = init.old_va;
    m_state.old_vb = init.old_vb;
    m_state.old_vc = init.old_vc;
    m_state.old_time_10ms = init.old_time_10ms;
    m_state.old_time_ms = init.old_time_ms;
    m_state.last_torque_demand = init.last_torque_demand;
}

const SimState& SimRunner::state() const
{
    return m_state;
}

RunResult SimRunner::Run(const SimInputs& inputs, const RunSpec& settle, const RunSpec& measure, const RunHooks& hooks)
{
    RunResult result{};
    if (!m_state.motor)
    {
        result.ok = false;
        result.error = "SimRunner: motor is null";
        return result;
    }

    Controller controller;
    Modulator modulator;
    InverterSwitchingModel inverter;

    InverterParams inv_params = inputs.inv_params;
    inverter.SetModuleParams(inputs.module_params);
    inverter.ResetThermals(inv_params.sink_temp_C);

    int measure_steps_seen = 0;
    int v_sat_high_count = 0;

    auto updateValidity = [&](const StepSnapshot& snapshot)
    {
        if (!inputs.check_validity || !result.validity.valid)
            return;

        const double ia = snapshot.phase_currents.a;
        const double ib = snapshot.phase_currents.b;
        const double ic = snapshot.phase_currents.c;
        const double id = snapshot.motor.id;
        const double iq = snapshot.motor.iq;
        const double power = snapshot.elec_power_w;
        const double loss = snapshot.inv_loss.total_W;

        if (!std::isfinite(ia) || !std::isfinite(ib) || !std::isfinite(ic) ||
            !std::isfinite(id) || !std::isfinite(iq) ||
            !std::isfinite(power) || !std::isfinite(loss))
        {
            result.validity.valid = false;
            result.validity.reason = "NAN_INF";
            return;
        }

        const double max_abs_i = std::max({std::abs(ia), std::abs(ib), std::abs(ic)});
        const double max_abs_idq = std::max(std::abs(id), std::abs(iq));
        result.validity.max_abs_i_abc = std::max(result.validity.max_abs_i_abc, max_abs_i);
        result.validity.max_abs_idq = std::max(result.validity.max_abs_idq, max_abs_idq);
        result.validity.max_abs_power_w = std::max(result.validity.max_abs_power_w,
                                                   std::max(std::abs(power), std::abs(loss)));

        const double vdc = inputs.vdc_V;
        if (vdc > 0.0)
        {
            const double vd = snapshot.controller.vd_ctrl;
            const double vq = snapshot.controller.vq_ctrl;
            const double v_mag = std::sqrt(vd * vd + vq * vq);
            const double v_lim = vdc / 1.7320508075688772;
            if (v_lim > 0.0)
            {
                const double v_sat_frac = v_mag / v_lim;
                result.validity.v_sat_frac_max = std::max(result.validity.v_sat_frac_max, v_sat_frac);
                if (v_sat_frac >= inputs.validity_limits.v_sat_frac_limit)
                    ++v_sat_high_count;
            }
        }

        if (inputs.validity_limits.i_hard_max_A > 0.0 &&
            result.validity.max_abs_i_abc > inputs.validity_limits.i_hard_max_A)
        {
            result.validity.valid = false;
            result.validity.reason = "CURRENT_BLOWUP";
            return;
        }

        if (inputs.validity_limits.p_hard_max_W > 0.0 &&
            result.validity.max_abs_power_w > inputs.validity_limits.p_hard_max_W)
        {
            result.validity.valid = false;
            result.validity.reason = "POWER_BLOWUP";
            return;
        }
    };

    auto runSteps = [&](int steps, bool emitHooks)
    {
        for (int i = 0; i < steps; ++i)
        {
            if (hooks.should_abort && hooks.should_abort())
            {
                result.ok = false;
                result.error = "Cancelled";
                return false;
            }
            const int step_index = result.steps_total;
            StepSnapshot snapshot = StepOnce(step_index, inputs, controller, modulator, inverter, inv_params);
            ++result.steps_total;
            if (emitHooks)
            {
                ++measure_steps_seen;
                updateValidity(snapshot);
                if (inputs.check_validity && !result.validity.valid)
                    return false;
            }
            if (emitHooks && hooks.on_step)
                hooks.on_step(snapshot);
        }
        return true;
    };

    if (!runSteps(std::max(0, settle.steps), false))
        return result;
    if (!runSteps(std::max(0, measure.steps), true))
    {
        if (inputs.check_validity && !result.validity.valid)
        {
            if (measure_steps_seen > 0)
            {
                result.validity.v_sat_frac_pct =
                    100.0 * static_cast<double>(v_sat_high_count) / static_cast<double>(measure_steps_seen);
                if (result.validity.v_sat_frac_pct >= inputs.validity_limits.v_sat_frac_pct &&
                    result.validity.max_abs_i_abc > 0.5 * inputs.validity_limits.i_hard_max_A)
                {
                    result.validity.reason = "V_SAT_TOO_HIGH";
                }
            }
            return result;
        }
        return result;
    }

    if (measure_steps_seen > 0)
    {
        result.validity.v_sat_frac_pct =
            100.0 * static_cast<double>(v_sat_high_count) / static_cast<double>(measure_steps_seen);
        if (result.validity.v_sat_frac_pct >= inputs.validity_limits.v_sat_frac_pct &&
            result.validity.max_abs_i_abc > 0.5 * inputs.validity_limits.i_hard_max_A)
        {
            result.validity.valid = false;
            result.validity.reason = "V_SAT_TOO_HIGH";
        }
    }

    return result;
}

StepSnapshot SimRunner::StepOnce(int step_index,
                                 const SimInputs& inputs,
                                 Controller& controller,
                                 Modulator& modulator,
                                 InverterSwitchingModel& inverter,
                                 InverterParams& inv_params)
{
    StepSnapshot snap{};
    snap.step_index = step_index;
    snap.time_s = m_state.time_s;
    snap.vdc_V = inputs.vdc_V;

    // 10 ms tasks.
    if (static_cast<uint32_t>(m_state.time_s * 100.0) != m_state.old_time_10ms)
    {
        m_state.old_time_10ms = static_cast<uint32_t>(m_state.time_s * 100.0);
        Encoder::UpdateRotorFrequency(100);

        int requestedTorque = inputs.torque_demand_pct * 100;
        if (inputs.throttle_ramps)
        {
            if (m_state.last_torque_demand != requestedTorque)
            {
                if (requestedTorque > m_state.last_torque_demand)
                    requestedTorque = RAMPUP(m_state.last_torque_demand, requestedTorque,
                                             ((m_state.last_torque_demand >= 0) ? 500 : 50));
                else
                    requestedTorque = RAMPDOWN(m_state.last_torque_demand, requestedTorque,
                                               ((m_state.last_torque_demand >= 0) ? 500 : 50));
                m_state.last_torque_demand = requestedTorque;
            }
            controller.SetTorquePercent(((static_cast<float>(requestedTorque + 50)) / 100.0f));
        }
        else
        {
            controller.SetTorquePercent(static_cast<float>(inputs.torque_demand_pct));
        }
    }

    // 1 ms tasks (placeholder preserved for parity with prior behavior).
    if (static_cast<uint32_t>(m_state.time_s * 1000.0) != m_state.old_time_ms)
    {
        m_state.old_time_ms = static_cast<uint32_t>(m_state.time_s * 1000.0);
    }

    if (!m_state.motor)
        return snap;

    if (inputs.operating_mode == MotorModel::OperatingMode::ClampedSpeed)
        m_state.motor->setClampedSpeedRpm(inputs.clamped_speed_rpm);
    m_state.motor->setOperatingMode(inputs.operating_mode);

    controller.SetRotorAngle(static_cast<uint16_t>((m_state.motor->getElecPosition() * kTwoPiCont) / 360.0));
    bool pwmEnabled = controller.PwmEnabled();
    double il1_input = 0.0;
    double il2_input = 0.0;
    if (pwmEnabled)
    {
        il1_input = Param::GetFloat(Param::il1gain) * m_state.motor->getIaSamp();
        il2_input = Param::GetFloat(Param::il2gain) * m_state.motor->getIbSamp();
    }

    if (inputs.add_noise && inputs.noise_fn)
    {
        il1_input += inputs.noise_fn(inputs.noise_amp);
        il2_input += inputs.noise_fn(inputs.noise_amp);
    }

    controller.SetCurrentInputs(il1_input, il2_input);
    controller.Run();
    pwmEnabled = controller.PwmEnabled();

    DutyCycles duty{};
    ModulatorDiag modDiag{};
    PhaseVoltages voltages{};
    LossBreakdown invLoss{};
    ThermalState invThermal{};
    PwmRippleDiag invRipple{};

    const PhaseCurrents phaseCurrents{
        m_state.motor->getIaSamp(),
        m_state.motor->getIbSamp(),
        m_state.motor->getIcSamp()
    };

    const ModulationMode step_mode = inputs.mod_mode_fn ? inputs.mod_mode_fn() : inputs.mod_mode;

    if (!pwmEnabled)
    {
        voltages = {};
    }
    else
    {
        const double theta_rad = (m_state.motor->getElecPosition() * kPi) / 180.0;
        const double vd_ctrl = controller.UdVolts(inputs.vdc_V);
        const double vq_ctrl = controller.UqVolts(inputs.vdc_V);
        const double v_alpha = (vd_ctrl * std::cos(theta_rad)) - (vq_ctrl * std::sin(theta_rad));
        const double v_beta = (vd_ctrl * std::sin(theta_rad)) + (vq_ctrl * std::cos(theta_rad));

        if (inv_params.integrate_currents_in_pwm)
        {
            inv_params.elec_angle_rad = theta_rad;
            const double vq_bemf = m_state.motor->getVq_bemf();
            const double e_alpha = -vq_bemf * std::sin(theta_rad);
            const double e_beta = vq_bemf * std::cos(theta_rad);
            inv_params.bemf_phase_ln_V.a = e_alpha;
            inv_params.bemf_phase_ln_V.b = (-0.5 * e_alpha) + ((kSqrt3 / 2.0) * e_beta);
            inv_params.bemf_phase_ln_V.c = (-0.5 * e_alpha) - ((kSqrt3 / 2.0) * e_beta);
        }

        if (step_mode == ModulationMode::Firmware)
        {
            duty = modulator.GetDutyCycles();
            voltages = inverter.FromDuty(inputs.vdc_V, duty, phaseCurrents, inputs.timestep_s,
                                         inv_params, &invLoss, &invThermal, &invRipple);
            modulator.ComputeFromAlphaBeta(v_alpha, v_beta, inputs.vdc_V, ModulationMode::SVPWM,
                                           inputs.mod_blend, &modDiag);
        }
        else
        {
            duty = modulator.ComputeFromAlphaBeta(v_alpha, v_beta, inputs.vdc_V, step_mode,
                                                  inputs.mod_blend, &modDiag);
            voltages = inverter.FromDuty(inputs.vdc_V, duty, phaseCurrents, inputs.timestep_s,
                                         inv_params, &invLoss, &invThermal, &invRipple);
        }
    }

    PhaseVoltages voltages_cmd = voltages;
    inverter.RemoveCommonMode(voltages);

    const double va_ln = voltages.a;
    const double vb_ln = voltages.b;
    const double vc_ln = voltages.c;

    if (inputs.extra_cycle_delay)
        m_state.motor->Step(m_state.old_va, m_state.old_vb, m_state.old_vc);
    else
        m_state.motor->Step(va_ln, vb_ln, vc_ln);

    m_state.old_va = va_ln;
    m_state.old_vb = vb_ln;
    m_state.old_vc = vc_ln;

    snap.pwm_enabled = pwmEnabled;
    snap.duty = duty;
    snap.mod_diag = modDiag;
    snap.voltages_cmd = voltages_cmd;
    snap.voltages_ln = voltages;
    snap.phase_currents = phaseCurrents;
    snap.inv_loss = invLoss;
    snap.inv_thermal = invThermal;
    snap.inv_ripple = invRipple;

    snap.motor.ia_samp = m_state.motor->getIaSamp();
    snap.motor.ib_samp = m_state.motor->getIbSamp();
    snap.motor.ic_samp = m_state.motor->getIcSamp();
    snap.motor.id = m_state.motor->getId();
    snap.motor.iq = m_state.motor->getIq();
    snap.motor.motor_freq_hz = m_state.motor->getMotorFreq();
    snap.motor.motor_pos_deg = m_state.motor->getMotorPosition();
    snap.motor.elec_pos_deg = m_state.motor->getElecPosition();
    snap.motor.torque_nm = m_state.motor->getTorque();
    snap.motor.power_w = m_state.motor->getPower();
    snap.motor.vd = m_state.motor->getVd();
    snap.motor.vq = m_state.motor->getVq();
    snap.motor.vq_bemf = m_state.motor->getVq_bemf();
    snap.motor.vq_dueto_id = m_state.motor->getVq_dueto_id();
    snap.motor.vd_dueto_iq = m_state.motor->getVd_dueto_iq();
    snap.motor.vq_dueto_rq = m_state.motor->getVq_dueto_Rq();
    snap.motor.vd_dueto_rd = m_state.motor->getVd_dueto_Rd();
    snap.motor.vld = m_state.motor->getVLd();
    snap.motor.vlq = m_state.motor->getVLq();

    snap.controller.id = controller.Id();
    snap.controller.iq = controller.Iq();
    snap.controller.ifw = controller.Ifw();
    snap.controller.vd_ctrl = controller.UdVolts(inputs.vdc_V);
    snap.controller.vq_ctrl = controller.UqVolts(inputs.vdc_V);

    snap.elec_power_w = (va_ln * snap.motor.ia_samp) +
                        (vb_ln * snap.motor.ib_samp) +
                        (vc_ln * snap.motor.ic_samp);

    m_state.time_s += inputs.timestep_s;

    return snap;
}
} // namespace sim
