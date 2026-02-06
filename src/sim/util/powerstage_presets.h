#ifndef SIM_UTIL_POWERSTAGE_PRESETS_H
#define SIM_UTIL_POWERSTAGE_PRESETS_H

#include <QHash>
#include <QVector>
#include <QString>

#include "sim/power_module.h"
#include "sim/inverter_switching_model.h"

namespace sim
{
struct PowerStagePreset
{
    QString key;
    QString label;
    bool has_parallel_devices_per_switch = false;
    int parallel_devices_per_switch = 1;
    bool has_deadtime_us = false;
    double deadtime_us = 0.0;
    bool has_vge_on_v = false;
    double vge_on_v = 0.0;
    bool has_vge_off_v = false;
    double vge_off_v = 0.0;
    bool has_rg_on_ohm = false;
    double rg_on_ohm = 0.0;
    bool has_rg_off_ohm = false;
    double rg_off_ohm = 0.0;
    bool has_vref_v = false;
    double vref_v = 0.0;
    bool has_kv = false;
    double kv = 0.0;
    bool has_diode_vf_25 = false;
    double diode_vf_25 = 0.0;
    bool has_diode_vf_125 = false;
    double diode_vf_125 = 0.0;
    bool has_rth_jc_igbt = false;
    double rth_jc_igbt = 0.0;
    bool has_rth_jc_diode = false;
    double rth_jc_diode = 0.0;
    bool has_rth_cs = false;
    double rth_cs = 0.0;
    QVector<sim::CurvePoint> vce_points;
    QVector<sim::CurvePoint> eon_points;
    QVector<sim::CurvePoint> eoff_points;
    QVector<sim::CurvePoint> irr_points;
    QVector<sim::CurvePoint> trr_points;
};

QString ResolvePowerStageYamlPath(const QString& app_dir, const QString& cwd);

bool LoadPowerStagePresets(const QString& yaml_path,
                           QVector<PowerStagePreset>* presets,
                           QHash<QString, int>* index_by_key,
                           QString* error);

const PowerStagePreset* FindPowerStagePreset(const QVector<PowerStagePreset>& presets,
                                             const QString& key);

void ApplyPowerStagePresetToParams(const PowerStagePreset& preset,
                                   InverterParams* inv_params,
                                   PowerModuleParams* module_params);
} // namespace sim

#endif // SIM_UTIL_POWERSTAGE_PRESETS_H
