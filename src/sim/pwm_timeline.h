#ifndef SIM_PWM_TIMELINE_H
#define SIM_PWM_TIMELINE_H

#include "modulator.h"

#include <array>
#include <vector>

namespace sim
{
enum class LegGateState
{
    HsOn,
    LsOn,
    BothOff
};

struct TimelineSegment
{
    double t0_s = 0.0;
    double t1_s = 0.0;
    std::array<LegGateState, 3> leg{{LegGateState::LsOn, LegGateState::LsOn, LegGateState::LsOn}};
};

double ClampDutyToTon(double duty_norm,
                      double period_s,
                      double dead_s,
                      double min_on_s = 0.0,
                      double min_off_s = 0.0);

std::vector<TimelineSegment> BuildCenterAlignedTimeline(const DutyCycles& duty,
                                                        double pwm_period_s,
                                                        double deadtime_s,
                                                        double min_on_s = 0.0,
                                                        double min_off_s = 0.0);
} // namespace sim

#endif // SIM_PWM_TIMELINE_H
