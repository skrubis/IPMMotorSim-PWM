#ifndef SIM_STRATEGY_SPEC_H
#define SIM_STRATEGY_SPEC_H

#include <string>

namespace sim
{
enum class StrategyId
{
    SVPWM,
    DPWMMIN,
    DPWMMAX,
    DPWM0,
    DPWM1,
    DPWM2,
    DPWM3,
    Firmware,
    AUTO_RULE,
    AUTO_PRED
};

struct StrategySpec
{
    StrategyId id = StrategyId::SVPWM;
    const char* label = "";
    bool is_auto = false;
};

const char* StrategyLabel(StrategyId id);
bool StrategyIsAuto(StrategyId id);
} // namespace sim

#endif // SIM_STRATEGY_SPEC_H
