#include <QCoreApplication>
#include <QCommandLineParser>
#include <QCryptographicHash>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QProcess>
#include <QTextStream>
#include <memory>
#include <algorithm>

#include "sim/lut/lut_builder.h"
#include "sim/power_module.h"
#include "sim/sim_runner.h"
#include "sim/sweep/sweep_config.h"
#include "sim/sweep/sweep_runner.h"
#include "sim/util/powerstage_presets.h"
#include "sim/util/process_utils.h"
#include "sim/motor_plant.h"

#include "params.h"
#include "pwmgeneration.h"

namespace
{
struct MotorDefaults
{
    double wheel_radius_m = 0.3;
    double gear_ratio = 6.0;
    double road_gradient = 0.0;
    double vehicle_mass_kg = 500.0;
    double lq_H = 4e-3;
    double ld_H = 2e-3;
    double rs_ohm = 0.075;
    double poles = 4.0;
    double flux_Wb = 0.2;
    double timestep_s = 1.0 / 8800.0;
    double sync_delay = 16.0;
    double sampling_point = 50.0;
};

static bool LoadJsonDocument(const QString& path, QJsonDocument* doc, QString* error)
{
    if (doc)
        *doc = {};
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        if (error)
            *error = QString("Failed to open %1").arg(path);
        return false;
    }
    QJsonParseError parseError{};
    const QJsonDocument parsed = QJsonDocument::fromJson(file.readAll(), &parseError);
    if (parseError.error != QJsonParseError::NoError)
    {
        if (error)
            *error = QString("JSON parse error at %1: %2").arg(parseError.offset).arg(parseError.errorString());
        return false;
    }
    if (!parsed.isObject())
    {
        if (error)
            *error = "JSON root must be an object";
        return false;
    }
    if (doc)
        *doc = parsed;
    return true;
}

static QString Sha256File(const QString& path)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
        return QString();
    QCryptographicHash hash(QCryptographicHash::Sha256);
    if (!hash.addData(&file))
        return QString();
    return QString::fromUtf8(hash.result().toHex());
}

static QString ReadGitHash(const QString& workdir)
{
    QProcess process;
    process.setProgram("git");
    process.setArguments({"rev-parse", "HEAD"});
    if (!workdir.isEmpty())
        process.setWorkingDirectory(workdir);
    process.start();
    if (!process.waitForFinished(2000))
        return QString();
    if (process.exitStatus() != QProcess::NormalExit || process.exitCode() != 0)
        return QString();
    return QString::fromUtf8(process.readAllStandardOutput()).trimmed();
}

static double ReadDouble(const QJsonObject& obj, const QString& key, double fallback)
{
    if (!obj.contains(key))
        return fallback;
    const QJsonValue v = obj.value(key);
    if (v.isDouble())
        return v.toDouble(fallback);
    if (v.isString())
    {
        bool ok = false;
        const double parsed = v.toString().trimmed().toDouble(&ok);
        return ok ? parsed : fallback;
    }
    return fallback;
}

static QString ReadPhiSource(const QJsonObject& root)
{
    const QJsonObject lutObj = root.value("lut").toObject();
    const QString lutPhi = lutObj.value("phi_source").toString().trimmed();
    if (!lutPhi.isEmpty())
        return lutPhi;
    return root.value("phi_source").toString().trimmed();
}

static void ApplyLutConfigFromJson(const QJsonObject& root, sim::LutConfig* config)
{
    if (!config)
        return;
    const QJsonObject lutObj = root.value("lut").toObject();
    const QString phiSource = ReadPhiSource(root);
    if (!phiSource.isEmpty())
        config->phi_source = phiSource;
    if (lutObj.contains("phi_min_deg"))
        config->phi_min_deg = ReadDouble(lutObj, "phi_min_deg", config->phi_min_deg);
    if (lutObj.contains("phi_max_deg"))
        config->phi_max_deg = ReadDouble(lutObj, "phi_max_deg", config->phi_max_deg);
    if (lutObj.contains("phi_bins"))
        config->phi_bins = std::max(1, lutObj.value("phi_bins").toInt(config->phi_bins));
    if (lutObj.contains("smooth_islands"))
        config->smooth_islands = lutObj.value("smooth_islands").toBool(config->smooth_islands);
    if (lutObj.contains("island_min_neighbors"))
        config->island_min_neighbors = std::max(0, lutObj.value("island_min_neighbors").toInt(config->island_min_neighbors));
}

static bool BuildSweepContext(const QJsonObject& root,
                              const sim::SweepConfig& cfg,
                              const QString& preset_override,
                              sim::SweepContext* ctx,
                              sim::PowerModuleParams* module_params_out,
                              QString* preset_used,
                              QString* error)
{
    if (!ctx || !module_params_out)
        return false;

    MotorDefaults defaults;
    const QJsonObject motorObj = root.value("motor").toObject();
    defaults.wheel_radius_m = ReadDouble(motorObj, "wheel_radius_m", defaults.wheel_radius_m);
    defaults.gear_ratio = ReadDouble(motorObj, "gear_ratio", defaults.gear_ratio);
    defaults.road_gradient = ReadDouble(motorObj, "road_gradient", defaults.road_gradient);
    defaults.vehicle_mass_kg = ReadDouble(motorObj, "vehicle_mass_kg", defaults.vehicle_mass_kg);
    defaults.lq_H = ReadDouble(motorObj, "lq_mH", defaults.lq_H * 1e3) * 1e-3;
    defaults.ld_H = ReadDouble(motorObj, "ld_mH", defaults.ld_H * 1e3) * 1e-3;
    defaults.rs_ohm = ReadDouble(motorObj, "rs_ohm", defaults.rs_ohm);
    defaults.flux_Wb = ReadDouble(motorObj, "flux_mWb", defaults.flux_Wb * 1e3) * 1e-3;
    defaults.poles = ReadDouble(motorObj, "poles", defaults.poles);

    const QJsonObject simObj = root.value("sim").toObject();
    const double loop_freq = ReadDouble(simObj, "loop_freq_hz", 0.0);
    defaults.timestep_s = ReadDouble(simObj, "timestep_s", defaults.timestep_s);
    if (loop_freq > 0.0)
        defaults.timestep_s = 1.0 / loop_freq;
    defaults.sync_delay = ReadDouble(simObj, "sync_delay", defaults.sync_delay);
    defaults.sampling_point = ReadDouble(simObj, "sampling_point", defaults.sampling_point);

    const double poles = cfg.pole_pairs > 0 ? static_cast<double>(cfg.pole_pairs) : defaults.poles;

    auto motor = std::make_unique<sim::MotorPlant>(defaults.wheel_radius_m,
                                                   defaults.gear_ratio,
                                                   defaults.road_gradient,
                                                   defaults.vehicle_mass_kg,
                                                   defaults.lq_H,
                                                   defaults.ld_H,
                                                   defaults.rs_ohm,
                                                   poles,
                                                   defaults.flux_Wb,
                                                   defaults.timestep_s,
                                                   defaults.sync_delay,
                                                   defaults.sampling_point);

    auto runner = std::make_unique<sim::SimRunner>();

    sim::InverterParams invParams;
    invParams.pwm_frequency_hz = cfg.f_sw_Hz.values.empty() ? 8800.0 : cfg.f_sw_Hz.values.front();
    invParams.deadtime_s = 2e-6;
    invParams.min_on_s = 0.0;
    invParams.min_off_s = 0.0;
    invParams.integrate_currents_in_pwm = ReadDouble(simObj, "integrate_currents_in_pwm", 0.0) > 0.0;
    invParams.phase_R_ohm = std::max(0.0, defaults.rs_ohm);
    invParams.phase_L_H = std::max(0.0, 0.5 * (defaults.ld_H + defaults.lq_H));
    invParams.compute_torque_ripple = invParams.integrate_currents_in_pwm;
    invParams.pole_pairs = poles;
    invParams.flux_Wb = defaults.flux_Wb;
    invParams.ld_H = defaults.ld_H;
    invParams.lq_H = defaults.lq_H;

    const QJsonObject inverterObj = root.value("inverter").toObject();
    if (inverterObj.contains("deadtime_us"))
        invParams.deadtime_s = ReadDouble(inverterObj, "deadtime_us", invParams.deadtime_s * 1e6) * 1e-6;
    if (inverterObj.contains("min_on_us"))
        invParams.min_on_s = ReadDouble(inverterObj, "min_on_us", invParams.min_on_s * 1e6) * 1e-6;
    if (inverterObj.contains("min_off_us"))
        invParams.min_off_s = ReadDouble(inverterObj, "min_off_us", invParams.min_off_s * 1e6) * 1e-6;
    if (inverterObj.contains("min_pulse_s"))
    {
        const double minPulse = ReadDouble(inverterObj, "min_pulse_s", invParams.min_on_s);
        invParams.min_on_s = minPulse;
        invParams.min_off_s = minPulse;
    }

    if (!cfg.temp_C.values.empty())
        invParams.sink_temp_C = cfg.temp_C.values.front();
    else
        invParams.sink_temp_C = ReadDouble(simObj, "sink_temp_C", 25.0);

    invParams.thermal_tau_s = ReadDouble(simObj, "thermal_tau_s", 1.0);

    sim::PowerModuleParams moduleParams = sim::PM300CLA060();

    QString presetKey = cfg.powerstage_preset;
    if (!preset_override.isEmpty())
        presetKey = preset_override;

    if (!presetKey.isEmpty())
    {
        const QString yamlPath = sim::ResolvePowerStageYamlPath(QCoreApplication::applicationDirPath(), QDir::currentPath());
        if (yamlPath.isEmpty())
        {
            if (error)
                *error = "powerstages.yml not found";
            return false;
        }
        QVector<sim::PowerStagePreset> presets;
        QHash<QString, int> presetIndex;
        QString presetError;
        if (!sim::LoadPowerStagePresets(yamlPath, &presets, &presetIndex, &presetError))
        {
            if (error)
                *error = presetError;
            return false;
        }
        const sim::PowerStagePreset* preset = sim::FindPowerStagePreset(presets, presetKey);
        if (!preset)
        {
            if (error)
                *error = QString("Preset '%1' not found in %2").arg(presetKey, yamlPath);
            return false;
        }
        sim::ApplyPowerStagePresetToParams(*preset, &invParams, &moduleParams);
        if (preset_used)
            *preset_used = presetKey;
    }
    else if (preset_used)
    {
        preset_used->clear();
    }

    if (cfg.min_pulse_s >= 0.0)
    {
        invParams.min_on_s = cfg.min_pulse_s;
        invParams.min_off_s = cfg.min_pulse_s;
    }

    sim::SimInputs inputs;
    inputs.timestep_s = defaults.timestep_s;
    inputs.vdc_V = cfg.vdc_V.values.empty() ? 360.0 : cfg.vdc_V.values.front();
    inputs.mod_mode = sim::ModulationMode::SVPWM;
    inputs.mod_blend = ReadDouble(simObj, "mod_blend", 1.0);
    inputs.inv_params = invParams;
    inputs.module_params = moduleParams;

    ctx->runner = runner.get();
    ctx->motor = motor.get();
    ctx->base_inputs = inputs;

    *module_params_out = moduleParams;

    static std::vector<std::unique_ptr<sim::MotorPlant>> motors;
    static std::vector<std::unique_ptr<sim::SimRunner>> runners;
    motors.push_back(std::move(motor));
    runners.push_back(std::move(runner));
    return true;
}

static bool WriteRunManifest(const QString& out_dir,
                             const QString& config_path,
                             const sim::SweepConfig& cfg,
                             const sim::SimInputs& inputs,
                             const sim::PowerModuleParams& module_params,
                             const QString& preset_used,
                             const QString& phi_source,
                             const QString& python_version,
                             QString* error)
{
    QJsonObject root;
    root.insert("timestamp_utc", QDateTime::currentDateTimeUtc().toString(Qt::ISODate));
    root.insert("config_path", config_path);
    root.insert("config_sha256", Sha256File(config_path));
    if (!preset_used.isEmpty())
        root.insert("powerstage_preset", preset_used);

    QJsonObject inverterObj;
    inverterObj.insert("deadtime_s", inputs.inv_params.deadtime_s);
    inverterObj.insert("min_on_s", inputs.inv_params.min_on_s);
    inverterObj.insert("min_off_s", inputs.inv_params.min_off_s);
    inverterObj.insert("pwm_frequency_hz", inputs.inv_params.pwm_frequency_hz);
    if (inputs.inv_params.has_parallel_devices_per_switch)
        inverterObj.insert("parallel_devices_per_switch", inputs.inv_params.parallel_devices_per_switch);
    root.insert("inverter", inverterObj);

    QJsonObject moduleObj;
    moduleObj.insert("vref_V", module_params.vref_V);
    moduleObj.insert("kv", module_params.kv);
    root.insert("module", moduleObj);

    QJsonObject constraints;
    constraints.insert("thd_max_pct", cfg.thd_max_pct);
    constraints.insert("i_ripple_rms_max_a", cfg.i_ripple_rms_max_a);
    constraints.insert("min_pulse_margin_min_s", cfg.min_pulse_margin_min_s);
    constraints.insert("min_pulse_s_effective", inputs.inv_params.min_on_s);
    root.insert("constraints", constraints);

    QJsonObject strategies;
    strategies.insert("baseline", QString::fromUtf8(sim::StrategyLabel(cfg.baseline)));
    QJsonArray candidates;
    for (const auto& id : cfg.candidates)
        candidates.append(QString::fromUtf8(sim::StrategyLabel(id)));
    strategies.insert("candidates", candidates);
    root.insert("strategies", strategies);

    if (!phi_source.isEmpty())
        root.insert("phi_source", phi_source);

    QJsonObject tools;
    tools.insert("simtool_version", "0.1");
    if (!python_version.isEmpty())
        tools.insert("python_version", python_version);
    const QString gitHash = ReadGitHash(QDir::currentPath());
    if (!gitHash.isEmpty())
        tools.insert("git_hash", gitHash);
    root.insert("tool_versions", tools);

    const QString path = QDir(out_dir).filePath("run_manifest.json");
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        if (error)
            *error = QString("Failed to write %1").arg(path);
        return false;
    }
    const QJsonDocument doc(root);
    file.write(doc.toJson(QJsonDocument::Indented));
    return true;
}

static QString PythonVersion(const QString& python_exe)
{
    sim::ProcessOutput output;
    QString error;
    const sim::ProcessStatus status = sim::RunProcess(python_exe, {"--version"}, QString(), &output, &error);
    if (status != sim::ProcessStatus::Ok)
        return QString();
    QString version = output.std_out.trimmed();
    if (version.isEmpty())
        version = output.std_err.trimmed();
    return version;
}

static int PrintError(const QString& message)
{
    QTextStream err(stderr);
    err << message << "\n";
    return 1;
}
} // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);
    QCoreApplication::setApplicationName("simtool");

    QCommandLineParser parser;
    parser.setApplicationDescription("IPMMotorSim CLI tool");
    parser.addHelpOption();

    QCommandLineOption sweepOpt("sweep", "Run sweep", "config.json");
    QCommandLineOption outOpt("out", "Output directory", "dir");
    QCommandLineOption presetOpt("preset", "Powerstage preset name", "name");
    QCommandLineOption makeLutOpt("make-lut", "Build LUT from summary.csv");
    QCommandLineOption inOpt("in", "Input directory", "dir");
    QCommandLineOption phiSourceOpt("phi-source", "Phi source (phi_idiq_deg or phi_pf_deg)", "source");
    QCommandLineOption cycleEvalOpt("cycle-eval", "Run cycle eval");
    QCommandLineOption cycleOpt("cycle", "Cycle CSV", "cycle.csv");
    QCommandLineOption makeReportOpt("make-report", "Make report PDF");
    QCommandLineOption pythonOpt("python", "Python executable", "python");
    QCommandLineOption allOpt("all", "Run full pipeline", "config.json");

    parser.addOption(sweepOpt);
    parser.addOption(outOpt);
    parser.addOption(presetOpt);
    parser.addOption(makeLutOpt);
    parser.addOption(inOpt);
    parser.addOption(phiSourceOpt);
    parser.addOption(cycleEvalOpt);
    parser.addOption(cycleOpt);
    parser.addOption(makeReportOpt);
    parser.addOption(pythonOpt);
    parser.addOption(allOpt);

    parser.process(app);

    const bool doSweep = parser.isSet(sweepOpt);
    const bool doMakeLut = parser.isSet(makeLutOpt);
    const bool doCycleEval = parser.isSet(cycleEvalOpt);
    const bool doMakeReport = parser.isSet(makeReportOpt);
    const bool doAll = parser.isSet(allOpt);

    const int commandCount = (doSweep ? 1 : 0) + (doMakeLut ? 1 : 0) + (doCycleEval ? 1 : 0) + (doMakeReport ? 1 : 0) + (doAll ? 1 : 0);
    if (commandCount != 1)
        return PrintError("Specify exactly one command: --sweep, --make-lut, --cycle-eval, --make-report, or --all");

    const QString pythonExe = parser.value(pythonOpt).isEmpty() ? sim::DefaultPythonExecutable() : parser.value(pythonOpt);
    QJsonDocument sweepDoc;
    bool haveSweepDoc = false;

    if (doSweep || doAll)
    {
        const QString configPath = doSweep ? parser.value(sweepOpt) : parser.value(allOpt);
        const QString outDir = parser.value(outOpt);
        if (configPath.isEmpty() || outDir.isEmpty())
            return PrintError("--sweep/--all requires --out and config path");

        QJsonDocument doc;
        QString error;
        if (!LoadJsonDocument(configPath, &doc, &error))
            return PrintError(error);
        sweepDoc = doc;
        haveSweepDoc = true;

        sim::SweepConfig cfg;
        if (!sim::LoadSweepConfig(configPath, &cfg, &error))
            return PrintError(error);
        cfg.output_dir = outDir;

        const QString presetOverride = parser.value(presetOpt);
        sim::SweepContext ctx;
        sim::PowerModuleParams moduleParams;
        QString presetUsed;
        if (!BuildSweepContext(doc.object(), cfg, presetOverride, &ctx, &moduleParams, &presetUsed, &error))
            return PrintError(error);

        if (ctx.motor)
            ctx.motor->Restart();
        sim::SimInit init{};
        init.motor = ctx.motor;
        ctx.runner->Reset(init);

        sim::SweepResult result;
        if (!sim::RunSweep(cfg, ctx, &result, &error))
            return PrintError(error);

        const QString phiSource = ReadPhiSource(doc.object());
        const QString pyVersion = doAll ? PythonVersion(pythonExe) : QString();
        if (!WriteRunManifest(outDir, configPath, cfg, ctx.base_inputs, moduleParams, presetUsed, phiSource, pyVersion, &error))
            return PrintError(error);

        if (!doAll)
            return 0;
    }

    if (doMakeLut || doAll)
    {
        const QString inDir = doMakeLut ? parser.value(inOpt) : parser.value(outOpt);
        if (inDir.isEmpty())
            return PrintError("--make-lut/--all requires --in (or --out for --all)");
        sim::LutConfig config;
        if (haveSweepDoc)
            ApplyLutConfigFromJson(sweepDoc.object(), &config);
        if (parser.isSet(phiSourceOpt))
            config.phi_source = parser.value(phiSourceOpt);
        const QString summaryPath = QDir(inDir).filePath("summary.csv");
        sim::LutResult lut;
        QString error;
        if (!sim::BuildLutFromSummaryCsv(summaryPath, config, &lut, &error))
            return PrintError(error);
        if (!sim::WriteLutJson(QDir(inDir).filePath("lut.json"), lut, &error) ||
            !sim::WriteLutHeader(QDir(inDir).filePath("lut_pwm_strategy.h"), lut, &error) ||
            !sim::WriteLutRuntimeParams(QDir(inDir).filePath("lut_runtime_params.h"), lut, &error))
            return PrintError(error);
        if (!doAll)
            return 0;
    }

    if (doCycleEval || doAll)
    {
        const QString inDir = doCycleEval ? parser.value(inOpt) : parser.value(outOpt);
        const QString cycleCsv = parser.value(cycleOpt);
        if (inDir.isEmpty() || cycleCsv.isEmpty())
            return PrintError("--cycle-eval/--all requires --in/--out and --cycle");
        sim::ProcessOutput output;
        QString error;
        const QString scriptPath = sim::ResolveToolPath("tools/cycle_eval/eval_cycle.py",
                                                        QCoreApplication::applicationDirPath(),
                                                        QDir::currentPath());
        const QString workdir = sim::ResolveToolWorkdir(scriptPath);
        const sim::ProcessStatus status = sim::RunPython(scriptPath,
                                                         {"--cycle", cycleCsv, "--in", inDir, "--out", inDir},
                                                         pythonExe, workdir, &output, &error);
        if (status != sim::ProcessStatus::Ok)
            return PrintError(QString("Cycle eval failed: %1\n%2%3").arg(error, output.std_out, output.std_err));
        if (!doAll)
            return 0;
    }

    if (doMakeReport || doAll)
    {
        const QString inDir = doMakeReport ? parser.value(inOpt) : parser.value(outOpt);
        if (inDir.isEmpty())
            return PrintError("--make-report/--all requires --in/--out");
        QString reportPath = doMakeReport ? parser.value(outOpt) : QDir(inDir).filePath("report.pdf");
        if (reportPath.isEmpty())
            reportPath = QDir(inDir).filePath("report.pdf");
        QStringList args{"--in", inDir, "--out", reportPath};
        sim::ProcessOutput output;
        QString error;
        const QString scriptPath = sim::ResolveToolPath("tools/report/make_report.py",
                                                        QCoreApplication::applicationDirPath(),
                                                        QDir::currentPath());
        const QString workdir = sim::ResolveToolWorkdir(scriptPath);
        const sim::ProcessStatus status = sim::RunPython(scriptPath, args,
                                                         pythonExe, workdir, &output, &error);
        if (status != sim::ProcessStatus::Ok)
            return PrintError(QString("Report failed: %1\n%2%3").arg(error, output.std_out, output.std_err));
        return 0;
    }

    return 0;
}
