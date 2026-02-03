#include "power_module.h"

#include <algorithm>
#include <cmath>

namespace sim
{
PowerModuleParams PM300CLA060()
{
    PowerModuleParams params;
    params.vref_V = 300.0;
    params.kv = 1.0;

    params.igbt_vce_sat = {{
        {50.0, 1.29, 1.11},
        {100.0, 1.37, 1.24},
        {200.0, 1.50, 1.41},
        {300.0, 1.60, 1.54},
    }};

    params.diode_vf_25C_V = 2.2;
    params.diode_vf_125C_V = 2.2;

    params.eon_mJ = {{
        {100.0, 3.72, 4.64},
        {200.0, 6.37, 7.22},
        {300.0, 7.80, 8.80},
    }};

    params.eoff_mJ = {{
        {100.0, 1.04, 2.37},
        {200.0, 3.91, 6.13},
        {300.0, 8.33, 11.44},
    }};

    params.irr_A = {{
        {100.0, 31.6, 85.6},
        {200.0, 72.5, 109.4},
        {300.0, 82.5, 128.5},
    }};

    params.trr_us = {{
        {100.0, 0.120, 0.245},
        {200.0, 0.115, 0.195},
        {300.0, 0.115, 0.195},
    }};

    params.rth_jc_igbt_C_per_W = 0.16;
    params.rth_jc_diode_C_per_W = 0.25;
    params.rth_cs_C_per_W = 0.023;
    return params;
}

double InterpI(const CurvePoint* points, size_t count, double current_A)
{
    if (!points || count == 0)
        return 0.0;

    const double x = std::abs(current_A);
    if (x <= points[0].current_A)
        return points[0].val_25C;
    if (x >= points[count - 1].current_A)
        return points[count - 1].val_25C;

    for (size_t idx = 1; idx < count; ++idx)
    {
        const CurvePoint& p0 = points[idx - 1];
        const CurvePoint& p1 = points[idx];
        if (x <= p1.current_A)
        {
            const double t = (x - p0.current_A) / (p1.current_A - p0.current_A);
            return p0.val_25C + t * (p1.val_25C - p0.val_25C);
        }
    }
    return points[count - 1].val_25C;
}

static double InterpI_T(const CurvePoint* points, size_t count, double current_A, bool use_125C)
{
    if (!points || count == 0)
        return 0.0;

    const double x = std::abs(current_A);
    auto valAt = [use_125C](const CurvePoint& p) { return use_125C ? p.val_125C : p.val_25C; };

    if (x <= points[0].current_A)
        return valAt(points[0]);
    if (x >= points[count - 1].current_A)
        return valAt(points[count - 1]);

    for (size_t idx = 1; idx < count; ++idx)
    {
        const CurvePoint& p0 = points[idx - 1];
        const CurvePoint& p1 = points[idx];
        if (x <= p1.current_A)
        {
            const double t = (x - p0.current_A) / (p1.current_A - p0.current_A);
            return valAt(p0) + t * (valAt(p1) - valAt(p0));
        }
    }
    return valAt(points[count - 1]);
}

double InterpT(double val_25C, double val_125C, double tj_C)
{
    const double t = std::clamp(tj_C, 25.0, 125.0);
    const double alpha = (t - 25.0) / 100.0;
    return val_25C + alpha * (val_125C - val_25C);
}

double IgbtVceSat(const PowerModuleParams& params, double current_A, double tj_C)
{
    const double v25 = InterpI_T(params.igbt_vce_sat.data(), params.igbt_vce_sat.size(), current_A, false);
    const double v125 = InterpI_T(params.igbt_vce_sat.data(), params.igbt_vce_sat.size(), current_A, true);
    return InterpT(v25, v125, tj_C);
}

double DiodeVf(const PowerModuleParams& params, double current_A, double tj_C)
{
    const double v25 = params.diode_vf_25C_V;
    const double v125 = params.diode_vf_125C_V;
    const double vf = InterpT(v25, v125, tj_C);
    (void)current_A;
    return vf;
}

double IgbtEon(const PowerModuleParams& params, double current_A, double vdc_V, double tj_C)
{
    const double e25_mJ = InterpI_T(params.eon_mJ.data(), params.eon_mJ.size(), current_A, false);
    const double e125_mJ = InterpI_T(params.eon_mJ.data(), params.eon_mJ.size(), current_A, true);
    const double e_mJ = InterpT(e25_mJ, e125_mJ, tj_C);
    const double scale = params.vref_V > 0.0 ? std::pow(vdc_V / params.vref_V, params.kv) : 1.0;
    return (e_mJ * 1e-3) * scale;
}

double IgbtEoff(const PowerModuleParams& params, double current_A, double vdc_V, double tj_C)
{
    const double e25_mJ = InterpI_T(params.eoff_mJ.data(), params.eoff_mJ.size(), current_A, false);
    const double e125_mJ = InterpI_T(params.eoff_mJ.data(), params.eoff_mJ.size(), current_A, true);
    const double e_mJ = InterpT(e25_mJ, e125_mJ, tj_C);
    const double scale = params.vref_V > 0.0 ? std::pow(vdc_V / params.vref_V, params.kv) : 1.0;
    return (e_mJ * 1e-3) * scale;
}

double DiodeErr(const PowerModuleParams& params, double current_A, double vdc_V, double tj_C)
{
    const double irr_25 = InterpI_T(params.irr_A.data(), params.irr_A.size(), current_A, false);
    const double irr_125 = InterpI_T(params.irr_A.data(), params.irr_A.size(), current_A, true);
    const double irr_A = InterpT(irr_25, irr_125, tj_C);

    const double trr_25 = InterpI_T(params.trr_us.data(), params.trr_us.size(), current_A, false);
    const double trr_125 = InterpI_T(params.trr_us.data(), params.trr_us.size(), current_A, true);
    const double trr_us = InterpT(trr_25, trr_125, tj_C);

    const double trr_s = trr_us * 1e-6;
    const double qrr_C = 0.5 * irr_A * trr_s;
    return vdc_V * qrr_C;
}
} // namespace sim
