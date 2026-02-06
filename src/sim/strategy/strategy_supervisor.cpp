#include "strategy/strategy_supervisor.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sim
{
namespace
{
static ModulationMode ToModMode(StrategyId id)
{
    switch (id)
    {
        case StrategyId::SVPWM: return ModulationMode::SVPWM;
        case StrategyId::DPWMMIN: return ModulationMode::DPWMMIN;
        case StrategyId::DPWMMAX: return ModulationMode::DPWMMAX;
        case StrategyId::DPWM0: return ModulationMode::DPWM0;
        case StrategyId::DPWM1: return ModulationMode::DPWM1;
        case StrategyId::DPWM2: return ModulationMode::DPWM2;
        case StrategyId::DPWM3: return ModulationMode::DPWM3;
        case StrategyId::Firmware: return ModulationMode::Firmware;
        default: return ModulationMode::SVPWM;
    }
}
} // namespace

StrategySupervisor::StrategySupervisor(const SupervisorConfig& config)
    : m_config(config)
{
}

void StrategySupervisor::Reset(StrategyId initial)
{
    m_current = initial;
    m_last_switch_time_s = -1.0;
    m_last_sector = -1;
}

bool StrategySupervisor::AllowSwitch(double now_s, int sector) const
{
    if (m_last_switch_time_s < 0.0)
        return true;
    if (m_config.sector_lock && sector >= 0 && m_last_sector >= 0 && sector == m_last_sector)
        return false;
    return (now_s - m_last_switch_time_s) >= m_config.min_dwell_s;
}

StrategyId StrategySupervisor::ApplyHysteresis(StrategyId proposed, double phi_deg) const
{
    if (m_current == proposed)
        return proposed;

    const double deadband = m_config.hysteresis_deg;
    if (m_current == StrategyId::DPWM1)
    {
        if (std::abs(phi_deg) <= (m_config.phi_deadband_deg + deadband))
            return m_current;
    }
    if (m_current == StrategyId::DPWM2 && phi_deg > 0.0)
    {
        if (phi_deg <= (m_config.phi_deadband_deg + deadband))
            return m_current;
    }
    if (m_current == StrategyId::DPWM0 && phi_deg < 0.0)
    {
        if (phi_deg >= (-m_config.phi_deadband_deg - deadband))
            return m_current;
    }

    return proposed;
}

StrategyDecision StrategySupervisor::SelectAutoRule(const SupervisorInputs& in)
{
    const double phi = in.has_phi_pf ? in.phi_pf_deg : in.phi_idiq_deg;
    StrategyId proposed = StrategyId::DPWM1;

    if (std::abs(phi) <= m_config.phi_deadband_deg)
    {
        proposed = StrategyId::DPWM1;
    }
    else if (phi > 0.0 && phi <= m_config.phi_outer_deg)
    {
        proposed = StrategyId::DPWM2;
    }
    else if (phi < 0.0 && phi >= -m_config.phi_outer_deg)
    {
        proposed = StrategyId::DPWM0;
    }
    else
    {
        proposed = StrategyId::DPWM3;
    }

    proposed = ApplyHysteresis(proposed, phi);
    if (AllowSwitch(in.time_s, in.sector))
    {
        if (proposed != m_current)
        {
            m_current = proposed;
            m_last_switch_time_s = in.time_s;
            m_last_sector = in.sector;
        }
    }

    StrategyDecision decision{};
    decision.selected = m_current;
    return decision;
}

StrategyPrediction StrategySupervisor::PredictLoss(StrategyId id, const SupervisorInputs& in) const
{
    StrategyPrediction pred{};
    pred.id = id;
    const ModulationMode mod = ToModMode(id);
    if (mod == ModulationMode::Firmware)
    {
        pred.predicted_total_w = std::numeric_limits<double>::infinity();
        return pred;
    }

    Modulator modulator;
    ModulatorDiag diag{};
    DutyCycles duty = modulator.ComputeFromAlphaBeta(in.v_alpha, in.v_beta, in.vdc_V, mod, in.mod_blend, &diag);
    InverterSwitchingModel inverter;
    inverter.SetModuleParams(in.module_params);
    inverter.ResetThermals(in.inv_params.sink_temp_C);
    LossBreakdown loss{};
    inverter.FromDuty(in.vdc_V, duty, in.currents, 0.0, in.inv_params, &loss, nullptr, nullptr);
    pred.predicted_total_w = loss.total_W;
    return pred;
}

StrategyDecision StrategySupervisor::SelectAutoPred(const SupervisorInputs& in, const std::vector<StrategyId>& candidates)
{
    StrategyDecision decision{};
    if (candidates.empty())
    {
        decision.selected = StrategyId::SVPWM;
        return decision;
    }

    StrategyPrediction best{};
    best.predicted_total_w = std::numeric_limits<double>::infinity();
    for (StrategyId id : candidates)
    {
        if (StrategyIsAuto(id))
            continue;
        const StrategyPrediction pred = PredictLoss(id, in);
        if (pred.predicted_total_w < best.predicted_total_w)
            best = pred;
    }

    if (!std::isfinite(best.predicted_total_w))
    {
        decision.selected = m_current;
        decision.predicted_total_w = best.predicted_total_w;
        decision.benefit_w = 0.0;
        return decision;
    }

    const StrategyPrediction currentPred = PredictLoss(m_current, in);
    const double benefit = currentPred.predicted_total_w - best.predicted_total_w;

    StrategyId proposed = (benefit >= m_config.min_benefit_w) ? best.id : m_current;
    if (AllowSwitch(in.time_s, in.sector))
    {
        if (proposed != m_current)
        {
            m_current = proposed;
            m_last_switch_time_s = in.time_s;
            m_last_sector = in.sector;
        }
    }

    decision.selected = m_current;
    decision.predicted_total_w = best.predicted_total_w;
    decision.benefit_w = benefit;
    return decision;
}
} // namespace sim
