#include "strategy/strategy_spec.h"

namespace sim
{
const char* StrategyLabel(StrategyId id)
{
    switch (id)
    {
        case StrategyId::SVPWM: return "SVPWM";
        case StrategyId::DPWMMIN: return "DPWMMIN";
        case StrategyId::DPWMMAX: return "DPWMMAX";
        case StrategyId::DPWM0: return "DPWM0";
        case StrategyId::DPWM1: return "DPWM1";
        case StrategyId::DPWM2: return "DPWM2";
        case StrategyId::DPWM3: return "DPWM3";
        case StrategyId::Firmware: return "Firmware";
        case StrategyId::AUTO_RULE: return "AUTO_RULE";
        case StrategyId::AUTO_PRED: return "AUTO_PRED";
        default: return "Unknown";
    }
}

bool StrategyIsAuto(StrategyId id)
{
    return (id == StrategyId::AUTO_RULE) || (id == StrategyId::AUTO_PRED);
}
} // namespace sim
