#ifndef SIM_STRATEGY_SUPERVISOR_H
#define SIM_STRATEGY_SUPERVISOR_H

#include <vector>

#include "sim/inverter_switching_model.h"
#include "sim/modulator.h"
#include "strategy_spec.h"

namespace sim
{
struct SupervisorConfig
{
    double phi_deadband_deg = 15.0;
    double phi_outer_deg = 75.0;
    double hysteresis_deg = 3.0;
    double min_dwell_s = 0.02;
    double min_benefit_w = 1.0;
    bool sector_lock = true;
};

struct SupervisorInputs
{
    double time_s = 0.0;
    double phi_pf_deg = 0.0;
    double phi_idiq_deg = 0.0;
    bool has_phi_pf = true;
    double v_alpha = 0.0;
    double v_beta = 0.0;
    double vdc_V = 0.0;
    PhaseCurrents currents;
    InverterParams inv_params;
    PowerModuleParams module_params;
    double mod_blend = 1.0;
    int sector = -1;
};

struct StrategyPrediction
{
    StrategyId id = StrategyId::SVPWM;
    double predicted_total_w = 0.0;
};

struct StrategyDecision
{
    StrategyId selected = StrategyId::SVPWM;
    double predicted_total_w = 0.0;
    double benefit_w = 0.0;
};

class StrategySupervisor
{
public:
    explicit StrategySupervisor(const SupervisorConfig& config = {});

    StrategyDecision SelectAutoRule(const SupervisorInputs& in);
    StrategyDecision SelectAutoPred(const SupervisorInputs& in, const std::vector<StrategyId>& candidates);

    StrategyId current() const { return m_current; }
    void Reset(StrategyId initial = StrategyId::SVPWM);

private:
    SupervisorConfig m_config{};
    StrategyId m_current = StrategyId::SVPWM;
    double m_last_switch_time_s = -1.0;
    int m_last_sector = -1;

    StrategyId ApplyHysteresis(StrategyId proposed, double phi_deg) const;
    bool AllowSwitch(double now_s, int sector) const;
    StrategyPrediction PredictLoss(StrategyId id, const SupervisorInputs& in) const;
};
} // namespace sim

#endif // SIM_STRATEGY_SUPERVISOR_H
