#include "run_orchestrator.h"

#include <QCoreApplication>
#include <QDir>
#include <QJsonObject>
#include <QFile>
#include <QFileInfo>
#include <QTextStream>
#include <QtConcurrent/QtConcurrentRun>
#include <algorithm>
#include <limits>

#include "sim/lut/lut_builder.h"
#include "sim/sweep/sweep_config.h"
#include "sim/util/process_utils.h"

// Windows headers can leak min/max macros that break std::min/std::max.
#ifdef max
#undef max
#endif
#ifdef min
#undef min
#endif

namespace
{
static bool IsFixedPointMode(sim::SweepMode mode)
{
    return mode == sim::SweepMode::FixedPointFreqSweep;
}

struct CoverageStats
{
    double cycle_speed_min = std::numeric_limits<double>::quiet_NaN();
    double cycle_speed_max = std::numeric_limits<double>::quiet_NaN();
    double cycle_iq_min = std::numeric_limits<double>::quiet_NaN();
    double cycle_iq_max = std::numeric_limits<double>::quiet_NaN();

    double map_speed_min = std::numeric_limits<double>::quiet_NaN();
    double map_speed_max = std::numeric_limits<double>::quiet_NaN();
    double map_iq_min = std::numeric_limits<double>::quiet_NaN();
    double map_iq_max = std::numeric_limits<double>::quiet_NaN();

    int cycle_samples = 0;
    int in_bounds_samples = 0;

    int map_speed_bins = 0;
    int map_iq_bins = 0;

    double coverage_pct() const
    {
        return cycle_samples > 0 ? (100.0 * static_cast<double>(in_bounds_samples) / static_cast<double>(cycle_samples)) : 0.0;
    }
};

static bool ReadCsvColumnIndices(const QString& headerLine, const QStringList& keys, QHash<QString, int>* out, QString* error)
{
    if (out)
        out->clear();
    const QStringList headers = headerLine.split(',', Qt::KeepEmptyParts);
    for (const QString& key : keys)
    {
        int idx = -1;
        for (int i = 0; i < headers.size(); ++i)
        {
            if (headers[i].trimmed() == key)
            {
                idx = i;
                break;
            }
        }
        if (idx < 0)
        {
            if (error)
                *error = QString("CSV missing required column '%1'").arg(key);
            return false;
        }
        if (out)
            out->insert(key, idx);
    }
    return true;
}

static bool ComputeMapCoverage(const QString& cycle_csv,
                               const QString& summary_csv,
                               CoverageStats* out,
                               QString* error)
{
    if (out)
        *out = CoverageStats{};
    if (error)
        error->clear();

    QFile cycleFile(cycle_csv);
    if (!cycleFile.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        if (error)
            *error = QString("Failed to open cycle CSV: %1").arg(cycle_csv);
        return false;
    }
    QFile summaryFile(summary_csv);
    if (!summaryFile.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        if (error)
            *error = QString("Failed to open summary.csv: %1").arg(summary_csv);
        return false;
    }

    QTextStream cycleTs(&cycleFile);
    QTextStream summaryTs(&summaryFile);

    const QString cycleHeader = cycleTs.readLine();
    QHash<QString, int> cycleIdx;
    QString err;
    if (!ReadCsvColumnIndices(cycleHeader, {"speed_rpm", "iq_A"}, &cycleIdx, &err))
    {
        if (error)
            *error = QString("Cycle CSV parse error: %1").arg(err);
        return false;
    }

    const QString summaryHeader = summaryTs.readLine();
    QHash<QString, int> sumIdx;
    if (!ReadCsvColumnIndices(summaryHeader, {"speed_rpm", "iq_A"}, &sumIdx, &err))
    {
        if (error)
            *error = QString("summary.csv parse error: %1").arg(err);
        return false;
    }

    QSet<double> speeds;
    QSet<double> iqs;
    double mapSMin = std::numeric_limits<double>::infinity();
    double mapSMax = -std::numeric_limits<double>::infinity();
    double mapQMin = std::numeric_limits<double>::infinity();
    double mapQMax = -std::numeric_limits<double>::infinity();
    while (!summaryTs.atEnd())
    {
        const QString line = summaryTs.readLine();
        if (line.trimmed().isEmpty())
            continue;
        const QStringList cols = line.split(',', Qt::KeepEmptyParts);
        if (cols.size() <= std::max(sumIdx["speed_rpm"], sumIdx["iq_A"]))
            continue;
        bool okS = false, okQ = false;
        const double s = cols[sumIdx["speed_rpm"]].trimmed().toDouble(&okS);
        const double q = cols[sumIdx["iq_A"]].trimmed().toDouble(&okQ);
        if (okS)
        {
            speeds.insert(s);
            mapSMin = std::min(mapSMin, s);
            mapSMax = std::max(mapSMax, s);
        }
        if (okQ)
        {
            iqs.insert(q);
            mapQMin = std::min(mapQMin, q);
            mapQMax = std::max(mapQMax, q);
        }
    }

    double cycleSMin = std::numeric_limits<double>::infinity();
    double cycleSMax = -std::numeric_limits<double>::infinity();
    double cycleQMin = std::numeric_limits<double>::infinity();
    double cycleQMax = -std::numeric_limits<double>::infinity();
    int n = 0;
    int inBounds = 0;
    while (!cycleTs.atEnd())
    {
        const QString line = cycleTs.readLine();
        if (line.trimmed().isEmpty())
            continue;
        const QStringList cols = line.split(',', Qt::KeepEmptyParts);
        if (cols.size() <= std::max(cycleIdx["speed_rpm"], cycleIdx["iq_A"]))
            continue;
        bool okS = false, okQ = false;
        const double s = cols[cycleIdx["speed_rpm"]].trimmed().toDouble(&okS);
        const double q = cols[cycleIdx["iq_A"]].trimmed().toDouble(&okQ);
        if (okS)
        {
            cycleSMin = std::min(cycleSMin, s);
            cycleSMax = std::max(cycleSMax, s);
        }
        if (okQ)
        {
            cycleQMin = std::min(cycleQMin, q);
            cycleQMax = std::max(cycleQMax, q);
        }
        if (okS && okQ && std::isfinite(mapSMin) && std::isfinite(mapSMax) && std::isfinite(mapQMin) && std::isfinite(mapQMax))
        {
            if (s >= mapSMin && s <= mapSMax && q >= mapQMin && q <= mapQMax)
                ++inBounds;
        }
        ++n;
    }

    if (out)
    {
        out->cycle_samples = n;
        out->in_bounds_samples = inBounds;
        out->map_speed_bins = speeds.size();
        out->map_iq_bins = iqs.size();
        out->map_speed_min = std::isfinite(mapSMin) ? mapSMin : std::numeric_limits<double>::quiet_NaN();
        out->map_speed_max = std::isfinite(mapSMax) ? mapSMax : std::numeric_limits<double>::quiet_NaN();
        out->map_iq_min = std::isfinite(mapQMin) ? mapQMin : std::numeric_limits<double>::quiet_NaN();
        out->map_iq_max = std::isfinite(mapQMax) ? mapQMax : std::numeric_limits<double>::quiet_NaN();
        out->cycle_speed_min = std::isfinite(cycleSMin) ? cycleSMin : std::numeric_limits<double>::quiet_NaN();
        out->cycle_speed_max = std::isfinite(cycleSMax) ? cycleSMax : std::numeric_limits<double>::quiet_NaN();
        out->cycle_iq_min = std::isfinite(cycleQMin) ? cycleQMin : std::numeric_limits<double>::quiet_NaN();
        out->cycle_iq_max = std::isfinite(cycleQMax) ? cycleQMax : std::numeric_limits<double>::quiet_NaN();
    }

    return true;
}

static void ApplyLutConfigFromDoc(const QJsonDocument& doc, sim::LutConfig* config)
{
    if (!config || doc.isNull() || !doc.isObject())
        return;
    const QJsonObject root = doc.object();
    const QJsonObject lutObj = root.value("lut").toObject();
    const QString phiSource = lutObj.value("phi_source").toString().trimmed();
    if (!phiSource.isEmpty())
        config->phi_source = phiSource;
    else if (root.contains("phi_source"))
        config->phi_source = root.value("phi_source").toString().trimmed();

    if (lutObj.contains("phi_min_deg"))
        config->phi_min_deg = lutObj.value("phi_min_deg").toDouble(config->phi_min_deg);
    if (lutObj.contains("phi_max_deg"))
        config->phi_max_deg = lutObj.value("phi_max_deg").toDouble(config->phi_max_deg);
    if (lutObj.contains("phi_bins"))
        config->phi_bins = std::max(1, lutObj.value("phi_bins").toInt(config->phi_bins));
    if (lutObj.contains("smooth_islands"))
        config->smooth_islands = lutObj.value("smooth_islands").toBool(config->smooth_islands);
    if (lutObj.contains("island_min_neighbors"))
        config->island_min_neighbors = std::max(0, lutObj.value("island_min_neighbors").toInt(config->island_min_neighbors));
}
} // namespace

RunOrchestrator::RunOrchestrator(QObject* parent)
    : QObject(parent)
{
}

bool RunOrchestrator::isRunning() const
{
    return m_running.load();
}

void RunOrchestrator::start(const Plan& plan, const sim::SweepContext& ctx)
{
    if (m_running.load())
        return;

    m_plan = plan;
    m_ctx = ctx;
    m_cancel.store(false);
    m_running.store(true);

    m_future = QtConcurrent::run([this]()
    {
        runInternal();
    });
}

void RunOrchestrator::cancel()
{
    m_cancel.store(true);
}

void RunOrchestrator::runInternal()
{
    auto finish = [&](bool ok, const QString& message)
    {
        emit logMessage(message);
        emit finished(ok, message);
        m_running.store(false);
    };

    if (m_plan.do_sweep)
    {
        emit stageChanged("Sweep");
        QString error;
        sim::SweepConfig cfg;
        if (!sim::LoadSweepConfig(m_plan.sweep_config_path, &cfg, &error))
        {
            finish(false, error);
            return;
        }
        if ((m_plan.do_cycle || m_plan.do_report) && IsFixedPointMode(cfg.mode))
        {
            finish(false, "FixedPointFreqSweep produces a single operating point; cannot run Cycle Eval. Use OperatingMapSweep (map_coarse.json) or choose a non-cycle scenario.");
            return;
        }
        cfg.output_dir = m_plan.out_dir;

        sim::SweepCallbacks callbacks;
        callbacks.should_abort = [this]() { return m_cancel.load(); };
        callbacks.on_progress = [this](const sim::SweepProgress& progress)
        {
            emit progressUpdated(progress);
        };

        sim::SweepResult result;
        if (!sim::RunSweep(cfg, m_ctx, &result, &error, &callbacks))
        {
            if (error.isEmpty())
                error = "Sweep failed";
            finish(false, error);
            return;
        }
        emit artifactsChanged(m_plan.out_dir);
    }

    if (m_cancel.load())
    {
        finish(false, "Cancelled");
        return;
    }

    if (m_plan.do_lut)
    {
        emit stageChanged("LUT");
        sim::LutConfig config;
        ApplyLutConfigFromDoc(m_plan.sweep_doc, &config);
        if (!m_plan.phi_source.isEmpty())
            config.phi_source = m_plan.phi_source;
        sim::LutResult lut;
        QString error;
        const QString summaryPath = QDir(m_plan.out_dir).filePath("summary.csv");
        if (!sim::BuildLutFromSummaryCsv(summaryPath, config, &lut, &error))
        {
            finish(false, error);
            return;
        }
        const QString lutJsonPath = QDir(m_plan.out_dir).filePath("lut.json");
        const QString lutHeaderPath = QDir(m_plan.out_dir).filePath("lut_pwm_strategy.h");
        const QString lutRuntimePath = QDir(m_plan.out_dir).filePath("lut_runtime_params.h");
        if (!sim::WriteLutJson(lutJsonPath, lut, &error) ||
            !sim::WriteLutHeader(lutHeaderPath, lut, &error) ||
            !sim::WriteLutRuntimeParams(lutRuntimePath, lut, &error))
        {
            finish(false, error);
            return;
        }
        if (!QFile::exists(lutJsonPath) || !QFile::exists(lutHeaderPath) || !QFile::exists(lutRuntimePath))
        {
            finish(false, "LUT build succeeded but expected artifacts were not created (lut.json / lut_pwm_strategy.h / lut_runtime_params.h).");
            return;
        }
        emit artifactsChanged(m_plan.out_dir);
    }

    if (m_cancel.load())
    {
        finish(false, "Cancelled");
        return;
    }

    if (m_plan.do_cycle)
    {
        emit stageChanged("Cycle Eval");
        const QString summaryCsv = QDir(m_plan.out_dir).filePath("summary.csv");
        const QString lutJson = QDir(m_plan.out_dir).filePath("lut.json");
        if (!QFile::exists(summaryCsv))
        {
            finish(false, QString("Cycle Eval requires '%1'.").arg(summaryCsv));
            return;
        }
        if (!QFile::exists(lutJson))
        {
            finish(false, QString("Cycle Eval requires '%1' (build a LUT first).").arg(lutJson));
            return;
        }

        constexpr double kMinCoveragePct = 80.0;
        CoverageStats cov{};
        QString covErr;
        if (ComputeMapCoverage(m_plan.cycle_csv, summaryCsv, &cov, &covErr))
        {
            emit logMessage(QString("Map coverage: %1% (in-bounds %2/%3) | map speed %.0f..%.0f rpm | map iq %.0f..%.0f A | cycle speed %.0f..%.0f rpm | cycle iq %.0f..%.0f A")
                                .arg(QString::number(cov.coverage_pct(), 'f', 2))
                                .arg(cov.in_bounds_samples)
                                .arg(cov.cycle_samples)
                                .arg(cov.map_speed_min)
                                .arg(cov.map_speed_max)
                                .arg(cov.map_iq_min)
                                .arg(cov.map_iq_max)
                                .arg(cov.cycle_speed_min)
                                .arg(cov.cycle_speed_max)
                                .arg(cov.cycle_iq_min)
                                .arg(cov.cycle_iq_max));

            if ((cov.map_speed_bins <= 1 || cov.map_iq_bins <= 1) && !m_plan.allow_low_coverage)
            {
                finish(false, "FixedPointFreqSweep produces a single speed/iq bin; cannot run Cycle Eval. Use OperatingMapSweep (map_coarse.json) or choose a non-cycle scenario.");
                return;
            }

            if (cov.coverage_pct() < kMinCoveragePct && !m_plan.allow_low_coverage)
            {
                finish(false, QString("Cycle coverage is only %1% (<%2%). Refusing to run because results will be dominated by clamping. Check 'Run anyway' to proceed, or run a wider OperatingMapSweep.")
                               .arg(QString::number(cov.coverage_pct(), 'f', 2))
                               .arg(QString::number(kMinCoveragePct, 'f', 0)));
                return;
            }
        }
        else
        {
            emit logMessage(QString("Coverage preflight skipped: %1").arg(covErr));
        }

        sim::ProcessOutput output;
        QString error;
        const QString scriptPath = sim::ResolveToolPath("tools/cycle_eval/eval_cycle.py",
                                                        QCoreApplication::applicationDirPath(),
                                                        QDir::currentPath());
        const QString workdir = sim::ResolveToolWorkdir(scriptPath);
        const sim::ProcessStatus status = sim::RunPython(scriptPath,
                                                         {"--cycle", m_plan.cycle_csv, "--in", m_plan.out_dir, "--out", m_plan.out_dir},
                                                         m_plan.python_exe, workdir, &output, &error,
                                                         &m_cancel,
                                                         [this](const QString& text) { emit logMessage(text); },
                                                         [this](const QString& text) { emit logMessage(text); });
        if (status != sim::ProcessStatus::Ok)
        {
            finish(false, QString("Cycle eval failed: %1\n%2%3").arg(error, output.std_out, output.std_err));
            return;
        }
        const QString cycleEvalPath = QDir(m_plan.out_dir).filePath("cycle_eval.json");
        if (!QFile::exists(cycleEvalPath))
        {
            finish(false, QString("Cycle eval completed but '%1' was not created.").arg(cycleEvalPath));
            return;
        }
        emit artifactsChanged(m_plan.out_dir);
    }

    if (m_cancel.load())
    {
        finish(false, "Cancelled");
        return;
    }

    if (m_plan.do_report)
    {
        emit stageChanged("Report");
        sim::ProcessOutput output;
        QString error;
        const QString reportPath = QDir(m_plan.out_dir).filePath("report.pdf");
        const QString scriptPath = sim::ResolveToolPath("tools/report/make_report.py",
                                                        QCoreApplication::applicationDirPath(),
                                                        QDir::currentPath());
        const QString workdir = sim::ResolveToolWorkdir(scriptPath);
        const sim::ProcessStatus status = sim::RunPython(scriptPath,
                                                         {"--in", m_plan.out_dir, "--out", reportPath},
                                                         m_plan.python_exe, workdir, &output, &error,
                                                         &m_cancel,
                                                         [this](const QString& text) { emit logMessage(text); },
                                                         [this](const QString& text) { emit logMessage(text); });
        if (status != sim::ProcessStatus::Ok)
        {
            finish(false, QString("Report failed: %1\n%2%3").arg(error, output.std_out, output.std_err));
            return;
        }
        QFileInfo reportInfo(reportPath);
        if (!reportInfo.exists() || reportInfo.size() <= 0)
        {
            finish(false, QString("Report completed but '%1' was not created.").arg(reportPath));
            return;
        }
        emit artifactsChanged(m_plan.out_dir);
    }

    finish(true, "Run complete");
}
