#include "sweep/sweep_config.h"

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include <algorithm>
#include <cmath>

namespace sim
{
namespace
{
static bool ReadJsonFile(const QString& path, QJsonDocument* outDoc, QString* error)
{
    if (outDoc)
        *outDoc = {};
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
    {
        if (error)
            *error = QString("Failed to open %1").arg(path);
        return false;
    }

    QJsonParseError parseError{};
    const QByteArray bytes = file.readAll();
    const QJsonDocument doc = QJsonDocument::fromJson(bytes, &parseError);
    if (parseError.error != QJsonParseError::NoError)
    {
        if (error)
            *error = QString("JSON parse error at %1: %2").arg(parseError.offset).arg(parseError.errorString());
        return false;
    }

    if (!doc.isObject())
    {
        if (error)
            *error = "JSON root must be an object";
        return false;
    }

    if (outDoc)
        *outDoc = doc;
    return true;
}

static std::vector<double> ParseAxis(const QJsonValue& value)
{
    std::vector<double> out;
    if (value.isArray())
    {
        const QJsonArray arr = value.toArray();
        out.reserve(arr.size());
        for (const QJsonValue& v : arr)
        {
            if (v.isDouble())
                out.push_back(v.toDouble());
        }
        return out;
    }

    if (value.isObject())
    {
        const QJsonObject obj = value.toObject();
        const double start = obj.value("start").toDouble(0.0);
        const double stop = obj.value("stop").toDouble(0.0);
        const int steps = std::max(1, obj.value("steps").toInt(1));
        out.reserve(static_cast<size_t>(steps));
        if (steps == 1)
        {
            out.push_back(start);
        }
        else
        {
            const double step = (stop - start) / static_cast<double>(steps - 1);
            for (int i = 0; i < steps; ++i)
                out.push_back(start + (step * i));
        }
        return out;
    }

    if (value.isDouble())
    {
        out.push_back(value.toDouble());
    }
    return out;
}

static SweepMode ParseMode(const QString& text)
{
    const QString t = text.trimmed().toLower();
    if (t == "fixedpointfreqsweep" || t == "fixedpoint")
        return SweepMode::FixedPointFreqSweep;
    return SweepMode::OperatingMapSweep;
}

static StrategyId ParseStrategyId(const QString& text)
{
    const QString t = text.trimmed().toLower();
    if (t == "dpwmmin") return StrategyId::DPWMMIN;
    if (t == "dpwmmax") return StrategyId::DPWMMAX;
    if (t == "dpwm0") return StrategyId::DPWM0;
    if (t == "dpwm1") return StrategyId::DPWM1;
    if (t == "dpwm2") return StrategyId::DPWM2;
    if (t == "dpwm3") return StrategyId::DPWM3;
    if (t == "firmware") return StrategyId::Firmware;
    if (t == "auto_rule" || t == "autorule") return StrategyId::AUTO_RULE;
    if (t == "auto_pred" || t == "autopred") return StrategyId::AUTO_PRED;
    return StrategyId::SVPWM;
}

static ThdMode ParseThdMode(const QString& text)
{
    const QString t = text.trimmed().toLower();
    if (t == "disabled")
        return ThdMode::Disabled;
    if (t == "control_step_proxy" || t == "controlstepproxy")
        return ThdMode::ControlStepProxy;
    return ThdMode::ControlStepProxy;
}

static double ReadConstraintDouble(const QJsonObject& obj, const QStringList& keys, double fallback)
{
    for (const QString& key : keys)
    {
        if (obj.contains(key))
            return obj.value(key).toDouble(fallback);
    }
    return fallback;
}

static bool IsAllowedFsw(double value_hz)
{
    if (value_hz <= 0.0)
        return true;
    constexpr double kAllowed[] = {4400.0, 8800.0, 17600.0};
    for (double allowed : kAllowed)
    {
        if (std::abs(value_hz - allowed) <= 1e-6)
            return true;
    }
    return false;
}

static bool ValidateFswAxis(const std::vector<double>& values, QString* error)
{
    QStringList invalid;
    for (double v : values)
    {
        if (v <= 0.0)
            continue;
        if (!IsAllowedFsw(v))
            invalid.push_back(QString::number(v, 'f', 3));
    }
    if (!invalid.isEmpty())
    {
        if (error)
            *error = QString("Unsupported PWM frequency. Allowed: 4400, 8800, 17600 Hz. Invalid: %1.")
                         .arg(invalid.join(", "));
        return false;
    }
    return true;
}
} // namespace

bool LoadSweepConfig(const QString& path, SweepConfig* out, QString* error)
{
    if (out)
        *out = SweepConfig{};

    QJsonDocument doc;
    if (!ReadJsonFile(path, &doc, error))
        return false;

    const QJsonObject root = doc.object();
    SweepConfig cfg;
    cfg.schema_version = root.value("schema_version").toInt(cfg.schema_version);
    cfg.mode = ParseMode(root.value("mode").toString("OperatingMapSweep"));

    const QJsonObject axes = root.value("axes").toObject();
    cfg.speed_rpm.values = ParseAxis(axes.value("speed_rpm"));
    cfg.iq_A.values = ParseAxis(axes.value("iq_A"));
    cfg.id_A.values = ParseAxis(axes.value("id_A"));
    cfg.vdc_V.values = ParseAxis(axes.value("Vdc_V"));
    cfg.temp_C.values = ParseAxis(axes.value("temp_C"));
    cfg.f_sw_Hz.values = ParseAxis(axes.value("f_sw_Hz"));

    const QJsonObject point = root.value("point").toObject();
    if (cfg.speed_rpm.values.empty() && point.contains("speed_rpm"))
        cfg.speed_rpm.values = {point.value("speed_rpm").toDouble(0.0)};
    if (cfg.iq_A.values.empty() && point.contains("iq_A"))
        cfg.iq_A.values = {point.value("iq_A").toDouble(0.0)};
    if (cfg.id_A.values.empty() && point.contains("id_A"))
        cfg.id_A.values = {point.value("id_A").toDouble(0.0)};
    if (cfg.vdc_V.values.empty() && point.contains("Vdc_V"))
        cfg.vdc_V.values = {point.value("Vdc_V").toDouble(0.0)};
    if (cfg.temp_C.values.empty() && point.contains("temp_C"))
        cfg.temp_C.values = {point.value("temp_C").toDouble(0.0)};
    if (cfg.f_sw_Hz.values.empty() && root.contains("f_sw_Hz"))
        cfg.f_sw_Hz.values = ParseAxis(root.value("f_sw_Hz"));

    const QJsonObject run = root.value("run").toObject();
    auto readRun = [&](const QString& key, double fallback)
    {
        if (run.contains(key))
            return run.value(key).toDouble(fallback);
        if (root.contains(key))
            return root.value(key).toDouble(fallback);
        return fallback;
    };
    cfg.settle_ms = readRun("settle_ms", 0.0);
    cfg.measure_ms = readRun("measure_ms", 0.0);
    cfg.settle_cycles = readRun("settle_cycles", 0.0);
    cfg.measure_cycles = readRun("measure_cycles", 0.0);

    cfg.pole_pairs = root.value("pole_pairs").toInt(0);

    cfg.baseline = ParseStrategyId(root.value("baseline").toString("SVPWM"));
    const QJsonArray candidates = root.value("candidates").toArray();
    for (const QJsonValue& v : candidates)
    {
        if (!v.isString())
            continue;
        cfg.candidates.push_back(ParseStrategyId(v.toString()));
    }

    const QJsonObject constraints = root.value("constraints").toObject();
    cfg.thd_max_pct = ReadConstraintDouble(constraints,
                                           {"thd_max_pct", "THD_max", "thd_max"},
                                           cfg.thd_max_pct);
    cfg.i_ripple_rms_max_a = ReadConstraintDouble(constraints,
                                                  {"i_ripple_rms_max_a", "I_ripple_rms_max", "i_ripple_rms_max"},
                                                  cfg.i_ripple_rms_max_a);
    cfg.min_pulse_margin_min_s = ReadConstraintDouble(constraints,
                                                      {"min_pulse_margin_min_s", "min_pulse_margin_s", "min_pulse_margin_min"},
                                                      cfg.min_pulse_margin_min_s);

    const QJsonObject inverter = root.value("inverter").toObject();
    if (root.contains("min_pulse_s"))
        cfg.min_pulse_s = root.value("min_pulse_s").toDouble(cfg.min_pulse_s);
    else if (inverter.contains("min_pulse_s"))
        cfg.min_pulse_s = inverter.value("min_pulse_s").toDouble(cfg.min_pulse_s);

    if (root.contains("powerstage_preset"))
        cfg.powerstage_preset = root.value("powerstage_preset").toString().trimmed();
    else if (root.contains("power_stage_preset"))
        cfg.powerstage_preset = root.value("power_stage_preset").toString().trimmed();

    cfg.thd_mode = ParseThdMode(root.value("thd_mode").toString("control_step_proxy"));
    cfg.thd_samples = root.value("thd_samples").toInt(cfg.thd_samples);
    cfg.output_dir = root.value("output_dir").toString();
    cfg.write_point_json = root.value("write_point_json").toBool(cfg.write_point_json);
    cfg.write_summary_csv = root.value("write_summary_csv").toBool(cfg.write_summary_csv);

    if (!ValidateFswAxis(cfg.f_sw_Hz.values, error))
        return false;

    if (out)
        *out = cfg;
    return true;
}
} // namespace sim
