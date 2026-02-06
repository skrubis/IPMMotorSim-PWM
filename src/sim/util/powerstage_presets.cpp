#include "sim/util/powerstage_presets.h"

#include <QDir>
#include <QFile>
#include <QMap>
#include <QStringList>

#include <array>
#include <algorithm>
#include <cmath>
#include <limits>

namespace sim
{
namespace
{
struct YamlFrame
{
    int indent = 0;
    QString key;
};

static QString StripComments(const QString& line)
{
    const int idx = line.indexOf('#');
    if (idx >= 0)
        return line.left(idx);
    return line;
}

static int LeadingSpaces(const QString& line)
{
    int count = 0;
    for (int i = 0; i < line.size(); ++i)
    {
        if (line[i] == ' ')
            ++count;
        else if (line[i] == '\t')
            count += 2;
        else
            break;
    }
    return count;
}

static QVector<double> ParseInlineList(QString text)
{
    text = text.trimmed();
    if (text.startsWith('['))
        text = text.mid(1);
    if (text.endsWith(']'))
        text.chop(1);
    QVector<double> values;
    const auto parts = text.split(',', Qt::SkipEmptyParts);
    for (const QString& part : parts)
    {
        bool ok = false;
        const double val = part.trimmed().toDouble(&ok);
        if (ok)
            values.append(val);
    }
    return values;
}

static double PickTempValue(const QMap<double, double>& values, double target, double fallback)
{
    if (values.isEmpty())
        return fallback;
    if (values.contains(target))
        return values.value(target);
    double bestTemp = values.firstKey();
    double bestDiff = std::abs(bestTemp - target);
    for (auto it = values.begin(); it != values.end(); ++it)
    {
        const double diff = std::abs(it.key() - target);
        if (diff < bestDiff)
        {
            bestDiff = diff;
            bestTemp = it.key();
        }
    }
    return values.value(bestTemp, fallback);
}

static QVector<sim::CurvePoint> NormalizeCurvePoints(QVector<sim::CurvePoint> points)
{
    for (auto& point : points)
    {
        if (std::isnan(point.val_25C) && !std::isnan(point.val_125C))
            point.val_25C = point.val_125C;
        if (std::isnan(point.val_125C) && !std::isnan(point.val_25C))
            point.val_125C = point.val_25C;
        if (std::isnan(point.val_25C))
            point.val_25C = 0.0;
        if (std::isnan(point.val_125C))
            point.val_125C = point.val_25C;
    }

    std::sort(points.begin(), points.end(),
              [](const sim::CurvePoint& a, const sim::CurvePoint& b)
              {
                  return a.current_A < b.current_A;
              });

    QVector<sim::CurvePoint> merged;
    const double eps = 1e-6;
    for (const auto& point : points)
    {
        if (!merged.isEmpty() && std::abs(point.current_A - merged.last().current_A) < eps)
        {
            merged.last().val_25C = point.val_25C;
            merged.last().val_125C = point.val_125C;
        }
        else
        {
            merged.append(point);
        }
    }
    points = merged;

    if (points.isEmpty())
        return points;

    if (points.front().current_A > 0.0)
        points.prepend({0.0, 0.0, 0.0});

    if (points.size() == 1)
    {
        const double i0 = points[0].current_A;
        const double i1 = (i0 > 0.0) ? (i0 * 2.0) : 1.0;
        const double scale = (i0 > 0.0) ? (i1 / i0) : 1.0;
        points.append({i1, points[0].val_25C * scale, points[0].val_125C * scale});
    }

    return points;
}

template <size_t N>
static std::array<CurvePoint, N> CopyCurvePoints(const QVector<CurvePoint>& points,
                                                 const std::array<CurvePoint, N>& defaults)
{
    std::array<CurvePoint, N> out = defaults;
    const int count = std::min<int>(static_cast<int>(N), static_cast<int>(points.size()));
    for (int i = 0; i < count; ++i)
        out[static_cast<size_t>(i)] = points[static_cast<size_t>(i)];
    return out;
}

static bool ParsePowerStagePresetsText(const QString& yamlContent,
                                       QVector<PowerStagePreset>* presets,
                                       QHash<QString, int>* index_by_key,
                                       QString* error)
{
    if (presets)
        presets->clear();
    if (index_by_key)
        index_by_key->clear();
    if (error)
        error->clear();

    if (!yamlContent.contains("power_stages:"))
    {
        if (error)
            *error = "Unsupported powerstages.yml schema (expected power_stages)";
        return false;
    }

    struct CurveBuckets
    {
        QMap<double, QMap<double, double>> vce_by_current;
        QMap<double, QMap<double, double>> eon_by_current;
        QMap<double, QMap<double, double>> eoff_by_current;
        QMap<double, QMap<double, double>> irr_by_current;
        QMap<double, QMap<double, double>> trr_by_current;
    };

    struct StageBuilderV1
    {
        PowerStagePreset preset;
        CurveBuckets curves;

        double diode_vf_current_A = 0.0;
        QMap<double, double> diode_vf_by_temp;

        double vref_V = 0.0;
        double kv = std::numeric_limits<double>::quiet_NaN();
    };

    auto unquote = [](QString text)
    {
        text = text.trimmed();
        if (text.size() >= 2)
        {
            const QChar first = text.front();
            const QChar last = text.back();
            if ((first == '"' && last == '"') || (first == '\'' && last == '\''))
                return text.mid(1, text.size() - 2);
        }
        return text;
    };

    auto tempBucket = [](double tj_C)
    {
        return (tj_C >= 100.0) ? 125.0 : 25.0;
    };

    auto pickTemp = [](const QMap<double, double>& values, double target, double fallback)
    {
        return PickTempValue(values, target, fallback);
    };

    auto addPoint3 = [&](QMap<double, QMap<double, double>>& bucket,
                         double current_A, double tj_C, double value)
    {
        if (current_A < 0.0)
            return;
        const double temp = tempBucket(tj_C);
        bucket[current_A][temp] = value;
    };

    QVector<PowerStagePreset> parsed;
    QHash<QString, int> parsedIndex;

    const QStringList lines = yamlContent.split('\n');
    QVector<YamlFrame> stack;
    bool inPowerStages = false;
    StageBuilderV1 current;
    bool haveStage = false;
    double globalKv = std::numeric_limits<double>::quiet_NaN();

    auto finalizeStage = [&]()
    {
        if (!haveStage || current.preset.key.trimmed().isEmpty())
            return;

        if (current.vref_V > 0.0)
        {
            current.preset.has_vref_v = true;
            current.preset.vref_v = current.vref_V;
        }
        const double kv = std::isfinite(current.kv) ? current.kv : (std::isfinite(globalKv) ? globalKv : std::numeric_limits<double>::quiet_NaN());
        if (std::isfinite(kv))
        {
            current.preset.has_kv = true;
            current.preset.kv = kv;
        }

        if (!current.diode_vf_by_temp.isEmpty())
        {
            current.preset.has_diode_vf_25 = true;
            current.preset.diode_vf_25 = pickTemp(current.diode_vf_by_temp, 25.0, 0.0);
            current.preset.has_diode_vf_125 = true;
            current.preset.diode_vf_125 = pickTemp(current.diode_vf_by_temp, 125.0, current.preset.diode_vf_25);
        }

        auto buildCurve = [&](const QMap<double, QMap<double, double>>& byCurrent)
        {
            QVector<sim::CurvePoint> out;
            out.reserve(byCurrent.size());
            for (auto it = byCurrent.begin(); it != byCurrent.end(); ++it)
            {
                const double current_A = it.key();
                const QMap<double, double>& temps = it.value();
                sim::CurvePoint point;
                point.current_A = current_A;
                point.val_25C = pickTemp(temps, 25.0, std::numeric_limits<double>::quiet_NaN());
                point.val_125C = pickTemp(temps, 125.0, point.val_25C);
                out.append(point);
            }
            return NormalizeCurvePoints(out);
        };

        current.preset.vce_points = buildCurve(current.curves.vce_by_current);
        current.preset.eon_points = buildCurve(current.curves.eon_by_current);
        current.preset.eoff_points = buildCurve(current.curves.eoff_by_current);
        current.preset.irr_points = buildCurve(current.curves.irr_by_current);
        current.preset.trr_points = buildCurve(current.curves.trr_by_current);

        if (current.preset.label.trimmed().isEmpty())
            current.preset.label = current.preset.key;

        parsedIndex.insert(current.preset.key, parsed.size());
        parsed.append(current.preset);
    };

    auto pathFromStack = [](const QVector<YamlFrame>& stack)
    {
        QStringList parts;
        parts.reserve(stack.size());
        for (const auto& frame : stack)
            parts.append(frame.key);
        return parts.join('.');
    };

    for (const QString& rawLine : lines)
    {
        const QString stripped = StripComments(rawLine);
        if (stripped.trimmed().isEmpty())
            continue;

        const int indent = LeadingSpaces(stripped);
        const QString trimmed = stripped.trimmed();

        if (trimmed.startsWith('-'))
        {
            const QString item = trimmed.mid(1).trimmed();

            if (inPowerStages && indent == 2 && item.startsWith("id:"))
            {
                finalizeStage();
                current = StageBuilderV1{};
                haveStage = true;

                const QString idValue = unquote(item.mid(item.indexOf(':') + 1).trimmed());
                current.preset.key = idValue;
                current.preset.label = idValue;

                stack.clear();
                stack.append({0, "power_stages"});
                continue;
            }

            if (!haveStage)
                continue;

            if (item.startsWith('['))
            {
                const auto values = ParseInlineList(item);
                const QString path = pathFromStack(stack);

                if (path.endsWith("device.conduction.igbt_vce_sat_points_V") && values.size() >= 3)
                {
                    addPoint3(current.curves.vce_by_current, values[0], values[1], values[2]);
                }
                else if (path.endsWith("device.conduction.diode_vf_points_V") && values.size() >= 3)
                {
                    const double current_A = values[0];
                    const double tj = values[1];
                    const double vf = values[2];
                    if (vf > 0.0 && current_A >= current.diode_vf_current_A)
                    {
                        current.diode_vf_current_A = current_A;
                        current.diode_vf_by_temp[tempBucket(tj)] = vf;
                    }
                }
                else if (path.endsWith("device.switching.igbt_eon_points_mJ") && values.size() >= 3)
                {
                    addPoint3(current.curves.eon_by_current, values[0], values[1], values[2]);
                }
                else if (path.endsWith("device.switching.igbt_eoff_points_mJ") && values.size() >= 3)
                {
                    addPoint3(current.curves.eoff_by_current, values[0], values[1], values[2]);
                }
                else if (path.endsWith("device.switching.diode_reverse_recovery.irr_points_A") && values.size() >= 3)
                {
                    addPoint3(current.curves.irr_by_current, values[0], values[1], values[2]);
                }
                else if (path.endsWith("device.switching.diode_reverse_recovery.trr_points_us") && values.size() >= 3)
                {
                    addPoint3(current.curves.trr_by_current, values[0], values[1], values[2]);
                }
                else if (path.endsWith("device.switching.igbt_eon_curve_mJ_vs_Ic_A") && values.size() >= 2)
                {
                    addPoint3(current.curves.eon_by_current, values[0], 125.0, values[1]);
                }
                else if (path.endsWith("device.switching.igbt_eoff_curve_mJ_vs_Ic_A") && values.size() >= 2)
                {
                    addPoint3(current.curves.eoff_by_current, values[0], 125.0, values[1]);
                }
                else if (path.endsWith("device.switching.diode_erec_points_mJ") && values.size() >= 3)
                {
                    const double current_A = values[0];
                    const double tj = values[1];
                    const double erec_mJ = values[2];
                    if (erec_mJ > 0.0 && current.vref_V > 0.0)
                    {
                        const double qrr_C = (erec_mJ * 1e-3) / current.vref_V;
                        const double trr_s = 0.2e-6;
                        const double irr_A = (trr_s > 0.0) ? (2.0 * qrr_C / trr_s) : 0.0;
                        addPoint3(current.curves.trr_by_current, current_A, tj, trr_s * 1e6);
                        addPoint3(current.curves.irr_by_current, current_A, tj, irr_A);
                    }
                }
            }

            continue;
        }

        const int colon = trimmed.indexOf(':');
        if (colon < 0)
            continue;

        const QString key = trimmed.left(colon).trimmed();
        const QString value = trimmed.mid(colon + 1).trimmed();

        while (!stack.isEmpty() && indent <= stack.last().indent)
            stack.removeLast();
        stack.append({indent, key});

        if (key == "power_stages")
        {
            inPowerStages = true;
            continue;
        }

        const QString path = pathFromStack(stack);

        if (path == "loss_model_defaults.switching_energy_vdc_exponent_kV")
        {
            bool ok = false;
            const double kv = value.toDouble(&ok);
            if (ok)
                globalKv = kv;
            continue;
        }

        if (!haveStage)
            continue;

        if (path.endsWith("display_name") && !value.isEmpty())
        {
            current.preset.label = unquote(value);
            continue;
        }

        if (path.endsWith("gate_drive.deadtime_min_us"))
        {
            bool ok = false;
            const double deadtime = value.toDouble(&ok);
            if (ok && deadtime >= 0.0)
            {
                current.preset.has_deadtime_us = true;
                current.preset.deadtime_us = deadtime;
            }
            continue;
        }
        if (path.endsWith("gate_drive.vge_on_v"))
        {
            bool ok = false;
            const double v = value.toDouble(&ok);
            if (ok)
            {
                current.preset.has_vge_on_v = true;
                current.preset.vge_on_v = v;
            }
            continue;
        }
        if (path.endsWith("gate_drive.vge_off_v"))
        {
            bool ok = false;
            const double v = value.toDouble(&ok);
            if (ok)
            {
                current.preset.has_vge_off_v = true;
                current.preset.vge_off_v = v;
            }
            continue;
        }
        if (path.endsWith("gate_drive.per_device_gate_resistors.rg_on_ohm") ||
            path.endsWith("gate_drive.effective_per_switch_seen_by_driver.rg_on_ohm"))
        {
            bool ok = false;
            const double r = value.toDouble(&ok);
            if (ok && r > 0.0)
            {
                current.preset.has_rg_on_ohm = true;
                current.preset.rg_on_ohm = r;
            }
            continue;
        }
        if (path.endsWith("gate_drive.per_device_gate_resistors.rg_off_ohm") ||
            path.endsWith("gate_drive.effective_per_switch_seen_by_driver.rg_off_ohm"))
        {
            bool ok = false;
            const double r = value.toDouble(&ok);
            if (ok && r > 0.0)
            {
                current.preset.has_rg_off_ohm = true;
                current.preset.rg_off_ohm = r;
            }
            continue;
        }

        if (path.endsWith("inverter.parallel_devices_per_switch"))
        {
            bool ok = false;
            const int par = value.toInt(&ok);
            if (ok && par >= 1)
            {
                current.preset.has_parallel_devices_per_switch = true;
                current.preset.parallel_devices_per_switch = par;
            }
            continue;
        }

        if (path.endsWith("device.switching.reference.vdc_V") || path.endsWith("device.switching.reference.vce_V"))
        {
            bool ok = false;
            const double vref = value.toDouble(&ok);
            if (ok && vref > 0.0)
                current.vref_V = vref;
            continue;
        }

        if (path.endsWith("device.thermal.rth_jc_K_per_W_per_device.igbt_max") ||
            path.endsWith("device.thermal.rth_jc_K_per_W_per_die.igbt_max"))
        {
            bool ok = false;
            const double val = value.toDouble(&ok);
            if (ok && val > 0.0)
            {
                current.preset.has_rth_jc_igbt = true;
                current.preset.rth_jc_igbt = val;
            }
            continue;
        }
        if (path.endsWith("device.thermal.rth_jc_K_per_W_per_device.diode_max") ||
            path.endsWith("device.thermal.rth_jc_K_per_W_per_die.diode_max"))
        {
            bool ok = false;
            const double val = value.toDouble(&ok);
            if (ok && val > 0.0)
            {
                current.preset.has_rth_jc_diode = true;
                current.preset.rth_jc_diode = val;
            }
            continue;
        }
        if (path.endsWith("device.thermal.rth_cs_K_per_W.typ") ||
            path.endsWith("device.thermal.rth_case_to_cooler_K_per_W_module.value"))
        {
            bool ok = false;
            const double val = value.toDouble(&ok);
            if (ok && val > 0.0)
            {
                current.preset.has_rth_cs = true;
                current.preset.rth_cs = val;
            }
            continue;
        }
    }

    finalizeStage();

    if (presets)
        *presets = parsed;
    if (index_by_key)
        *index_by_key = parsedIndex;
    return true;
}
} // namespace

QString ResolvePowerStageYamlPath(const QString& app_dir, const QString& cwd)
{
    const QString appDir = app_dir.isEmpty() ? QDir::currentPath() : app_dir;
    const QStringList candidates{
        QDir(cwd).filePath("powerstages.yml"),
        QDir(appDir).filePath("powerstages.yml"),
        QDir(appDir).filePath("../powerstages.yml"),
        QDir(appDir).filePath("../../powerstages.yml"),
        QDir(cwd).filePath("powerstages.yaml"),
        QDir(appDir).filePath("powerstages.yaml"),
        QDir(appDir).filePath("../powerstages.yaml"),
        QDir(appDir).filePath("../../powerstages.yaml")
    };
    for (const QString& path : candidates)
    {
        if (QFile::exists(path))
            return path;
    }
    return QString();
}

bool LoadPowerStagePresets(const QString& yaml_path,
                           QVector<PowerStagePreset>* presets,
                           QHash<QString, int>* index_by_key,
                           QString* error)
{
    if (presets)
        presets->clear();
    if (index_by_key)
        index_by_key->clear();
    if (error)
        error->clear();

    QFile file(yaml_path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        if (error)
            *error = QString("Unable to read powerstages.yml at '%1'").arg(yaml_path);
        return false;
    }

    const QString yamlContent = QString::fromUtf8(file.readAll());
    return ParsePowerStagePresetsText(yamlContent, presets, index_by_key, error);
}

const PowerStagePreset* FindPowerStagePreset(const QVector<PowerStagePreset>& presets,
                                             const QString& key)
{
    for (const auto& preset : presets)
    {
        if (preset.key == key)
            return &preset;
    }
    return nullptr;
}

void ApplyPowerStagePresetToParams(const PowerStagePreset& preset,
                                   InverterParams* inv_params,
                                   PowerModuleParams* module_params)
{
    if (inv_params)
    {
        if (preset.has_deadtime_us)
            inv_params->deadtime_s = preset.deadtime_us * 1e-6;
        if (preset.has_parallel_devices_per_switch)
        {
            inv_params->has_parallel_devices_per_switch = true;
            inv_params->parallel_devices_per_switch = std::max(1, preset.parallel_devices_per_switch);
        }
    }

    if (!module_params)
        return;

    if (preset.has_vref_v)
        module_params->vref_V = preset.vref_v;
    if (preset.has_kv)
        module_params->kv = preset.kv;
    if (preset.has_diode_vf_25)
        module_params->diode_vf_25C_V = preset.diode_vf_25;
    if (preset.has_diode_vf_125)
        module_params->diode_vf_125C_V = preset.diode_vf_125;
    if (preset.has_rth_jc_igbt)
        module_params->rth_jc_igbt_C_per_W = preset.rth_jc_igbt;
    if (preset.has_rth_jc_diode)
        module_params->rth_jc_diode_C_per_W = preset.rth_jc_diode;
    if (preset.has_rth_cs)
        module_params->rth_cs_C_per_W = preset.rth_cs;

    if (!preset.vce_points.isEmpty())
        module_params->igbt_vce_sat = CopyCurvePoints(preset.vce_points, module_params->igbt_vce_sat);
    if (!preset.eon_points.isEmpty())
        module_params->eon_mJ = CopyCurvePoints(preset.eon_points, module_params->eon_mJ);
    if (!preset.eoff_points.isEmpty())
        module_params->eoff_mJ = CopyCurvePoints(preset.eoff_points, module_params->eoff_mJ);
    if (!preset.irr_points.isEmpty())
        module_params->irr_A = CopyCurvePoints(preset.irr_points, module_params->irr_A);
    if (!preset.trr_points.isEmpty())
        module_params->trr_us = CopyCurvePoints(preset.trr_points, module_params->trr_us);
}
} // namespace sim
