#include "pwm_timeline.h"

#include <algorithm>
#include <cmath>

namespace sim
{
namespace
{
struct LegWaveform
{
    bool always_hs = false;
    bool always_ls = false;
    double t_rise = 0.0;
    double t_fall = 0.0;
    double dead = 0.0;

    LegGateState StateAt(double t) const
    {
        if (always_hs)
            return LegGateState::HsOn;
        if (always_ls)
            return LegGateState::LsOn;

        if (t < t_rise)
            return LegGateState::LsOn;
        if (t < (t_rise + dead))
            return LegGateState::BothOff;
        if (t < t_fall)
            return LegGateState::HsOn;
        if (t < (t_fall + dead))
            return LegGateState::BothOff;
        return LegGateState::LsOn;
    }
};

static double ClampDutyToTonInternal(double duty_norm, double period_s, double dead_s, double min_on_s, double min_off_s)
{
    const double d = std::clamp(duty_norm, 0.0, 1.0);
    double ton = std::clamp(d * period_s, 0.0, period_s);

    if (min_on_s > 0.0 && ton > 0.0 && ton < min_on_s)
        ton = 0.0;
    if (min_off_s > 0.0 && ton < period_s && (period_s - ton) < min_off_s)
        ton = period_s;

    const double dead = std::clamp(dead_s, 0.0, period_s * 0.49);
    if (ton > 0.0 && ton < period_s && ton <= (2.0 * dead))
        ton = 0.0;
    if (ton > 0.0 && ton < period_s && (period_s - ton) <= (2.0 * dead))
        ton = period_s;

    return ton;
}

static LegWaveform BuildLeg(double duty_norm, double period_s, double dead_s, double min_on_s, double min_off_s)
{
    LegWaveform wf;
    if (period_s <= 0.0)
    {
        wf.always_ls = true;
        return wf;
    }

    const double dead = std::clamp(dead_s, 0.0, period_s * 0.49);
    const double ton = ClampDutyToTonInternal(duty_norm, period_s, dead, min_on_s, min_off_s);
    wf.dead = dead;

    if (ton <= 0.0)
    {
        wf.always_ls = true;
        return wf;
    }
    if (ton >= period_s)
    {
        wf.always_hs = true;
        return wf;
    }

    wf.t_rise = 0.5 * period_s - 0.5 * ton;
    wf.t_fall = 0.5 * period_s + 0.5 * ton;
    wf.t_rise = std::clamp(wf.t_rise, 0.0, period_s);
    wf.t_fall = std::clamp(wf.t_fall, 0.0, period_s);
    if (wf.t_fall < wf.t_rise)
        std::swap(wf.t_fall, wf.t_rise);

    return wf;
}

static void AddTime(std::vector<double>& times, double t, double period_s)
{
    if (t <= 0.0)
        return;
    if (t >= period_s)
        return;
    times.push_back(t);
}
} // namespace

double ClampDutyToTon(double duty_norm, double period_s, double dead_s, double min_on_s, double min_off_s)
{
    return ClampDutyToTonInternal(duty_norm, period_s, dead_s, min_on_s, min_off_s);
}

std::vector<TimelineSegment> BuildCenterAlignedTimeline(const DutyCycles& duty,
                                                        double pwm_period_s,
                                                        double deadtime_s,
                                                        double min_on_s,
                                                        double min_off_s)
{
    std::vector<TimelineSegment> segments;
    if (pwm_period_s <= 0.0)
        return segments;

    const LegWaveform wa = BuildLeg(duty.a_norm, pwm_period_s, deadtime_s, min_on_s, min_off_s);
    const LegWaveform wb = BuildLeg(duty.b_norm, pwm_period_s, deadtime_s, min_on_s, min_off_s);
    const LegWaveform wc = BuildLeg(duty.c_norm, pwm_period_s, deadtime_s, min_on_s, min_off_s);

    std::vector<double> times;
    times.reserve(2 + 4 * 3);
    times.push_back(0.0);
    times.push_back(pwm_period_s);

    auto addLeg = [&](const LegWaveform& w)
    {
        if (w.always_hs || w.always_ls)
            return;
        AddTime(times, w.t_rise, pwm_period_s);
        AddTime(times, w.t_rise + w.dead, pwm_period_s);
        AddTime(times, w.t_fall, pwm_period_s);
        AddTime(times, w.t_fall + w.dead, pwm_period_s);
    };

    addLeg(wa);
    addLeg(wb);
    addLeg(wc);

    std::sort(times.begin(), times.end());
    times.erase(std::unique(times.begin(), times.end(), [](double a, double b)
                            { return std::abs(a - b) <= 1e-12; }),
                times.end());

    for (size_t idx = 0; idx + 1 < times.size(); ++idx)
    {
        const double t0 = times[idx];
        const double t1 = times[idx + 1];
        if (t1 <= t0)
            continue;

        const double tm = 0.5 * (t0 + t1);
        TimelineSegment seg;
        seg.t0_s = t0;
        seg.t1_s = t1;
        seg.leg[0] = wa.StateAt(tm);
        seg.leg[1] = wb.StateAt(tm);
        seg.leg[2] = wc.StateAt(tm);
        segments.push_back(seg);
    }

    return segments;
}
} // namespace sim
