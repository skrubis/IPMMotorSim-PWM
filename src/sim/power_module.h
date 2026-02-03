#ifndef SIM_POWER_MODULE_H
#define SIM_POWER_MODULE_H

#include <array>
#include <cstddef>

namespace sim
{
struct CurvePoint
{
    double current_A = 0.0;
    double val_25C = 0.0;
    double val_125C = 0.0;
};

struct PowerModuleParams
{
    double vref_V = 300.0;
    double kv = 1.0;

    std::array<CurvePoint, 4> igbt_vce_sat;
    double diode_vf_25C_V = 2.2;
    double diode_vf_125C_V = 2.2;

    std::array<CurvePoint, 3> eon_mJ;
    std::array<CurvePoint, 3> eoff_mJ;

    std::array<CurvePoint, 3> irr_A;
    std::array<CurvePoint, 3> trr_us;

    double rth_jc_igbt_C_per_W = 0.16;
    double rth_jc_diode_C_per_W = 0.25;
    double rth_cs_C_per_W = 0.023;
};

PowerModuleParams PM300CLA060();

double InterpI(const CurvePoint* points, size_t count, double current_A);
double InterpT(double val_25C, double val_125C, double tj_C);

double IgbtVceSat(const PowerModuleParams& params, double current_A, double tj_C);
double DiodeVf(const PowerModuleParams& params, double current_A, double tj_C);
double IgbtEon(const PowerModuleParams& params, double current_A, double vdc_V, double tj_C);
double IgbtEoff(const PowerModuleParams& params, double current_A, double vdc_V, double tj_C);
double DiodeErr(const PowerModuleParams& params, double current_A, double vdc_V, double tj_C);
} // namespace sim

#endif // SIM_POWER_MODULE_H
