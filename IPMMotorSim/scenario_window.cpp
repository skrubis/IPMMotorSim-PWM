#include "scenario_window.h"

#include "ui_scenario_window.h"

#include <QDesktopServices>
#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QMenu>
#include <QMessageBox>
#include <QSet>
#include <QTextCursor>
#include <QTextStream>
#include <QUrl>

#include <algorithm>
#include <cmath>
#include <limits>

#include "mainwindow.h"
#include "run_orchestrator.h"

namespace
{
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

static QStringList UnsupportedFswFromDoc(const QJsonDocument& doc)
{
    QStringList invalid;
    if (doc.isNull() || !doc.isObject())
        return invalid;

    auto checkVal = [&](const QJsonValue& v)
    {
        if (v.isDouble())
        {
            const double hz = v.toDouble(0.0);
            if (hz > 0.0 && !IsAllowedFsw(hz))
                invalid << QString::number(hz, 'f', 3);
        }
    };

    auto checkAxis = [&](const QJsonValue& v)
    {
        if (v.isArray())
        {
            const QJsonArray arr = v.toArray();
            for (const QJsonValue& el : arr)
                checkVal(el);
            return;
        }
        if (v.isObject())
        {
            // "start/stop/steps" axes are allowed, but the endpoints must still land on supported Hz values
            // if the user chooses discrete points. We can't know intermediate points here, so only check
            // explicit arrays/doubles.
            return;
        }
        checkVal(v);
    };

    const QJsonObject root = doc.object();
    const QJsonObject axes = root.value("axes").toObject();
    checkAxis(axes.value("f_sw_Hz"));
    checkAxis(root.value("f_sw_Hz"));
    return invalid;
}

static int AxisBinCount(const QJsonValue& v)
{
    if (v.isArray())
        return std::max(0, static_cast<int>(v.toArray().size()));
    if (v.isDouble() || v.isString())
        return 1;
    if (v.isObject())
    {
        const QJsonObject obj = v.toObject();
        const int steps = obj.value("steps").toInt(1);
        return std::max(1, steps);
    }
    return 0;
}

static bool IsFixedPointMode(const QString& mode)
{
    const QString t = mode.trimmed().toLower();
    return (t == "fixedpointfreqsweep" || t == "fixedpoint");
}

static bool ReadCycleMinMax(const QString& cycle_csv,
                            double* speed_min,
                            double* speed_max,
                            double* iq_min,
                            double* iq_max,
                            int* samples,
                            QString* error)
{
    if (speed_min) *speed_min = std::numeric_limits<double>::quiet_NaN();
    if (speed_max) *speed_max = std::numeric_limits<double>::quiet_NaN();
    if (iq_min) *iq_min = std::numeric_limits<double>::quiet_NaN();
    if (iq_max) *iq_max = std::numeric_limits<double>::quiet_NaN();
    if (samples) *samples = 0;
    if (error) error->clear();

    QFile f(cycle_csv);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        if (error)
            *error = QString("failed to open cycle CSV: %1").arg(cycle_csv);
        return false;
    }
    QTextStream ts(&f);
    const QString headerLine = ts.readLine();
    if (headerLine.trimmed().isEmpty())
        return false;
    const QStringList headers = headerLine.split(',', Qt::KeepEmptyParts);
    auto idxOf = [&](const QString& name)
    {
        for (int i = 0; i < headers.size(); ++i)
        {
            if (headers[i].trimmed() == name)
                return i;
        }
        return -1;
    };
    const int iSpeed = idxOf("speed_rpm");
    const int iIq = idxOf("iq_A");
    if (iSpeed < 0 || iIq < 0)
    {
        if (error)
            *error = "cycle CSV must include columns: speed_rpm, iq_A";
        return false;
    }

    double smin = std::numeric_limits<double>::infinity();
    double smax = -std::numeric_limits<double>::infinity();
    double qmin = std::numeric_limits<double>::infinity();
    double qmax = -std::numeric_limits<double>::infinity();
    int n = 0;
    while (!ts.atEnd())
    {
        const QString line = ts.readLine();
        if (line.trimmed().isEmpty())
            continue;
        const QStringList cols = line.split(',', Qt::KeepEmptyParts);
        if (cols.size() <= std::max(iSpeed, iIq))
            continue;
        bool okS = false, okQ = false;
        const double s = cols[iSpeed].trimmed().toDouble(&okS);
        const double q = cols[iIq].trimmed().toDouble(&okQ);
        if (okS)
        {
            smin = std::min(smin, s);
            smax = std::max(smax, s);
        }
        if (okQ)
        {
            qmin = std::min(qmin, q);
            qmax = std::max(qmax, q);
        }
        ++n;
    }
    if (samples)
        *samples = n;
    if (speed_min) *speed_min = std::isfinite(smin) ? smin : std::numeric_limits<double>::quiet_NaN();
    if (speed_max) *speed_max = std::isfinite(smax) ? smax : std::numeric_limits<double>::quiet_NaN();
    if (iq_min) *iq_min = std::isfinite(qmin) ? qmin : std::numeric_limits<double>::quiet_NaN();
    if (iq_max) *iq_max = std::isfinite(qmax) ? qmax : std::numeric_limits<double>::quiet_NaN();
    return true;
}

static bool ComputeCoveragePct(const QString& cycle_csv,
                               double map_speed_min,
                               double map_speed_max,
                               double map_iq_min,
                               double map_iq_max,
                               double* out_pct,
                               int* out_in_bounds,
                               int* out_total,
                               QString* error)
{
    if (out_pct) *out_pct = 0.0;
    if (out_in_bounds) *out_in_bounds = 0;
    if (out_total) *out_total = 0;
    if (error) error->clear();

    QFile f(cycle_csv);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        if (error)
            *error = QString("failed to open cycle CSV: %1").arg(cycle_csv);
        return false;
    }
    QTextStream ts(&f);
    const QString headerLine = ts.readLine();
    const QStringList headers = headerLine.split(',', Qt::KeepEmptyParts);
    auto idxOf = [&](const QString& name)
    {
        for (int i = 0; i < headers.size(); ++i)
        {
            if (headers[i].trimmed() == name)
                return i;
        }
        return -1;
    };
    const int iSpeed = idxOf("speed_rpm");
    const int iIq = idxOf("iq_A");
    if (iSpeed < 0 || iIq < 0)
        return false;

    int total = 0;
    int inBounds = 0;
    while (!ts.atEnd())
    {
        const QString line = ts.readLine();
        if (line.trimmed().isEmpty())
            continue;
        const QStringList cols = line.split(',', Qt::KeepEmptyParts);
        if (cols.size() <= std::max(iSpeed, iIq))
            continue;
        bool okS = false, okQ = false;
        const double s = cols[iSpeed].trimmed().toDouble(&okS);
        const double q = cols[iIq].trimmed().toDouble(&okQ);
        if (okS && okQ)
        {
            if (s >= map_speed_min && s <= map_speed_max && q >= map_iq_min && q <= map_iq_max)
                ++inBounds;
        }
        ++total;
    }
    if (out_in_bounds) *out_in_bounds = inBounds;
    if (out_total) *out_total = total;
    if (out_pct) *out_pct = total > 0 ? (100.0 * inBounds / total) : 0.0;
    return true;
}

static bool ReadSummaryMinMaxAndBins(const QString& summary_csv,
                                     double* speed_min,
                                     double* speed_max,
                                     double* iq_min,
                                     double* iq_max,
                                     int* speed_bins,
                                     int* iq_bins,
                                     QString* error)
{
    if (speed_min) *speed_min = std::numeric_limits<double>::quiet_NaN();
    if (speed_max) *speed_max = std::numeric_limits<double>::quiet_NaN();
    if (iq_min) *iq_min = std::numeric_limits<double>::quiet_NaN();
    if (iq_max) *iq_max = std::numeric_limits<double>::quiet_NaN();
    if (speed_bins) *speed_bins = 0;
    if (iq_bins) *iq_bins = 0;
    if (error) error->clear();

    QFile f(summary_csv);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        if (error)
            *error = QString("failed to open summary.csv: %1").arg(summary_csv);
        return false;
    }
    QTextStream ts(&f);
    const QString headerLine = ts.readLine();
    if (headerLine.trimmed().isEmpty())
        return false;
    const QStringList headers = headerLine.split(',', Qt::KeepEmptyParts);
    auto idxOf = [&](const QString& name)
    {
        for (int i = 0; i < headers.size(); ++i)
        {
            if (headers[i].trimmed() == name)
                return i;
        }
        return -1;
    };
    const int iSpeed = idxOf("speed_rpm");
    const int iIq = idxOf("iq_A");
    if (iSpeed < 0 || iIq < 0)
    {
        if (error)
            *error = "summary.csv missing expected columns (speed_rpm, iq_A)";
        return false;
    }

    QSet<double> speeds;
    QSet<double> iqs;
    double smin = std::numeric_limits<double>::infinity();
    double smax = -std::numeric_limits<double>::infinity();
    double qmin = std::numeric_limits<double>::infinity();
    double qmax = -std::numeric_limits<double>::infinity();

    while (!ts.atEnd())
    {
        const QString line = ts.readLine();
        if (line.trimmed().isEmpty())
            continue;
        const QStringList cols = line.split(',', Qt::KeepEmptyParts);
        if (cols.size() <= std::max(iSpeed, iIq))
            continue;
        bool okS = false, okQ = false;
        const double s = cols[iSpeed].trimmed().toDouble(&okS);
        const double q = cols[iIq].trimmed().toDouble(&okQ);
        if (okS)
        {
            speeds.insert(s);
            smin = std::min(smin, s);
            smax = std::max(smax, s);
        }
        if (okQ)
        {
            iqs.insert(q);
            qmin = std::min(qmin, q);
            qmax = std::max(qmax, q);
        }
    }

    if (speed_bins) *speed_bins = speeds.size();
    if (iq_bins) *iq_bins = iqs.size();
    if (speed_min) *speed_min = std::isfinite(smin) ? smin : std::numeric_limits<double>::quiet_NaN();
    if (speed_max) *speed_max = std::isfinite(smax) ? smax : std::numeric_limits<double>::quiet_NaN();
    if (iq_min) *iq_min = std::isfinite(qmin) ? qmin : std::numeric_limits<double>::quiet_NaN();
    if (iq_max) *iq_max = std::isfinite(qmax) ? qmax : std::numeric_limits<double>::quiet_NaN();
    return true;
}

static QString ModeToString(sim::ModulationMode mode)
{
    switch (mode)
    {
        case sim::ModulationMode::SVPWM: return QString("SVPWM");
        case sim::ModulationMode::DPWMMIN: return QString("DPWMMIN");
        case sim::ModulationMode::DPWMMAX: return QString("DPWMMAX");
        case sim::ModulationMode::DPWM0: return QString("DPWM0");
        case sim::ModulationMode::DPWM1: return QString("DPWM1");
        case sim::ModulationMode::DPWM2: return QString("DPWM2");
        case sim::ModulationMode::DPWM3: return QString("DPWM3");
        case sim::ModulationMode::Firmware: return QString("Firmware");
        default: return QString("Unknown");
    }
}

static QString Fmt(double v, int decimals)
{
    if (!std::isfinite(v))
        return QString("-");
    return QString::number(v, 'f', decimals);
}

static QString ScenarioKeyFromUi(QComboBox* combo)
{
    if (!combo)
        return {};
    const QString key = combo->currentData().toString().trimmed();
    return key.isEmpty() ? combo->currentText().trimmed() : key;
}

static QString ResolveSiblingDir(const QString& name)
{
    const QString appDir = QCoreApplication::applicationDirPath();
    const QString cwd = QDir::currentPath();
    const QStringList candidates{
        QDir(cwd).filePath(name),
        QDir(appDir).filePath(name),
        QDir(appDir).filePath(QString("../%1").arg(name)),
        QDir(appDir).filePath(QString("../../%1").arg(name))
    };
    for (const QString& path : candidates)
    {
        QDir d(path);
        if (d.exists())
            return d.absolutePath();
    }
    return QString();
}
} // namespace

ScenarioWindow::ScenarioWindow(MainWindow* mainWindow, QWidget* parent)
    : QDialog(parent)
    , m_mainWindow(mainWindow)
    , ui(new Ui::ScenarioWindow)
{
    ui->setupUi(this);

    setAttribute(Qt::WA_DeleteOnClose, true);
    setModal(false);

    ui->scenarioPreset->clear();
    ui->scenarioPreset->addItem("Operating Point (single point, sweep only)", "Operating Point");
    ui->scenarioPreset->addItem("Fixed Point f_sw sweep (sweep only)", "FixedPointFreqSweep");
    ui->scenarioPreset->addItem("Operating Map (sweep only; use for LUT inputs)", "Operating Map");
    ui->scenarioPreset->addItem("Driving Cycle (cycle eval + report only)", "Driving Cycle");
    ui->scenarioPreset->addItem("Full Pipeline (sweep -> LUT -> cycle -> report)", "Full Pipeline");

    if (ui->scenarioPreset)
    {
        ui->scenarioPreset->setToolTip(
            "Choose what 'Run' does:\n"
            "- Sweep-only scenarios write summary.csv (+ optional per-point JSON)\n"
            "- Driving Cycle uses an existing out_dir (summary.csv + lut.json)\n"
            "- Full Pipeline generates everything end-to-end");
    }

    if (ui->sweepConfigPath)
        ui->sweepConfigPath->setToolTip("Path to the sweep JSON file on disk. 'Run' saves the JSON editor to this path before executing.");
    if (ui->pbSweepLoad)
        ui->pbSweepLoad->setToolTip("Load an existing sweep config JSON (e.g. configs/freq_sweep_quick.json or configs/map_coarse.json).");
    if (ui->pbSweepSave)
        ui->pbSweepSave->setToolTip("Save the JSON editor to the path above.");
    if (ui->pbSweepFromUi)
        ui->pbSweepFromUi->setToolTip("Generate a sweep JSON from the Main UI's current operating point (speed/iq/id, Vdc, etc.).");

    if (ui->sweepOutDir)
        ui->sweepOutDir->setToolTip("Output directory for artifacts (summary.csv, lut.json, report.pdf, etc.).");
    if (ui->pbOpenOutDir)
        ui->pbOpenOutDir->setToolTip("Open the output directory in Explorer/Finder.");

    if (ui->cycleCsvPath)
        ui->cycleCsvPath->setToolTip("Motor-space cycle CSV (t_s, speed_rpm, iq_A, id_A). Examples are in cycles/.");
    if (ui->pbBrowseCycle)
        ui->pbBrowseCycle->setToolTip("Select a cycle CSV from disk (format: t_s,speed_rpm,iq_A,id_A).");
    if (ui->pbCycleExamples)
        ui->pbCycleExamples->setToolTip("Pick one of the example cycles shipped in the cycles/ folder.");
    if (ui->pythonExePath)
        ui->pythonExePath->setToolTip("Python executable to use for cycle eval/report. Leave blank to use the system default 'python'.");
    if (ui->pbBrowsePython)
        ui->pbBrowsePython->setToolTip("Select a Python executable (optional). Leave blank to use the system default.");

    if (ui->sweepConfigJson)
        ui->sweepConfigJson->setToolTip(
            "This JSON controls the sweep/LUT workflow.\n"
            "Key fields:\n"
            "- mode: OperatingMapSweep or FixedPointFreqSweep\n"
            "- baseline/candidates: which strategies to compare\n"
            "- axes or point: operating points to run\n"
            "- f_sw_Hz: PWM carrier frequency (Allowed: 4400, 8800, 17600)");

    if (ui->cbRunAnywayCoverage)
    {
        ui->cbRunAnywayCoverage->setChecked(false);
        ui->cbRunAnywayCoverage->setToolTip(
            "If enabled, Cycle Eval will run even when the cycle spends significant time\n"
            "outside the sweep/LUT map bounds (which causes clamping and can invalidate ROI results).\n"
            "Recommended: leave unchecked and instead run a map sweep that covers the cycle.");
    }

    if (ui->runLog)
        ui->runLog->setToolTip("Progress and errors from the run.");
    if (ui->artifactList)
        ui->artifactList->setToolTip("Files written into Output Dir. Select one and click 'Open Artifact'.");
    if (ui->pbOpenArtifact)
        ui->pbOpenArtifact->setToolTip("Open the selected artifact using the OS default handler.");

    ui->pbCancelRun->setEnabled(false);
    ui->runProgress->setRange(0, 100);
    ui->runProgress->setValue(0);
    ui->runStageLabel->setText("Idle");

    m_runOrchestrator = new RunOrchestrator(this);
    connect(m_runOrchestrator, &RunOrchestrator::stageChanged, this,
            [this](const QString& stage)
            {
                ui->runStageLabel->setText(stage);
                appendRunLog(QString("Stage: %1").arg(stage));
            });
    connect(m_runOrchestrator, &RunOrchestrator::progressUpdated, this,
            [this](const sim::SweepProgress& progress)
            {
                const int totalPoints = std::max(1, progress.point_count);
                const int totalStrategies = std::max(1, progress.strategy_count);
                const int totalCells = totalPoints * totalStrategies;
                const int baseIndex = (progress.point_index * totalStrategies) + progress.strategy_index;
                double stepFrac = 0.0;
                if (progress.measure_steps > 0)
                    stepFrac = std::min(1.0, static_cast<double>(progress.measure_step_index) /
                                              static_cast<double>(progress.measure_steps));
                const double overall = (static_cast<double>(baseIndex) + stepFrac) /
                                       static_cast<double>(std::max(1, totalCells));
                ui->runProgress->setValue(static_cast<int>(overall * 100.0));

                const QString autoInfo = (progress.auto_switch_count > 0 || progress.auto_primary_frac > 0.0)
                                             ? QString(" | auto=%1 %2% | switches=%3")
                                                   .arg(ModeToString(progress.auto_primary_mode))
                                                   .arg(Fmt(progress.auto_primary_frac * 100.0, 1))
                                                   .arg(progress.auto_switch_count)
                                             : QString();

                const QString detail = QString("Sweep %1/%2 | Strategy %3/%4 | mode=%5 | loss=%6 W | sw=%7 W | thd=%8%% | margin=%9 s%10")
                                           .arg(progress.point_index + 1)
                                           .arg(totalPoints)
                                           .arg(progress.strategy_index + 1)
                                           .arg(totalStrategies)
                                           .arg(QString::fromUtf8(sim::StrategyLabel(progress.strategy)))
                                           .arg(Fmt(progress.total_loss_w, 2))
                                           .arg(Fmt(progress.switching_loss_w, 2))
                                           .arg(Fmt(progress.thd_proxy_pct, 2))
                                           .arg(Fmt(progress.min_pulse_margin_s, 6))
                                           .arg(autoInfo);
                ui->runStageLabel->setText(detail);
            });
    connect(m_runOrchestrator, &RunOrchestrator::logMessage, this,
            [this](const QString& msg) { appendRunLog(msg); });
    connect(m_runOrchestrator, &RunOrchestrator::artifactsChanged, this,
            [this](const QString& outDir) { refreshArtifacts(outDir); });
    connect(m_runOrchestrator, &RunOrchestrator::finished, this,
            [this](bool ok, const QString& message)
            {
                refreshArtifacts(ui->sweepOutDir->text().trimmed());
                setRunUiEnabled(true);
                appendRunLog(ok ? QString("Done: %1").arg(message)
                                : QString("Failed: %1").arg(message));
                if (!ok && message.contains("Unsupported PWM frequency", Qt::CaseInsensitive))
                {
                    appendRunLog("Hint: f_sw_Hz is the PWM carrier frequency; firmware supports only 4400 / 8800 / 17600 Hz.");
                    appendRunLog("Hint: Try 'Load' -> configs/freq_sweep_quick.json (or edit the JSON f_sw_Hz values).");
                }
            });

    applyScenarioPreset(ScenarioKeyFromUi(ui->scenarioPreset));

    if (ui->cycleCsvPath)
        connect(ui->cycleCsvPath, &QLineEdit::textChanged, this, &ScenarioWindow::updatePreflight);
    if (ui->sweepOutDir)
        connect(ui->sweepOutDir, &QLineEdit::textChanged, this, &ScenarioWindow::updatePreflight);
    if (ui->sweepConfigPath)
        connect(ui->sweepConfigPath, &QLineEdit::textChanged, this, &ScenarioWindow::updatePreflight);
    updatePreflight();
}

ScenarioWindow::~ScenarioWindow()
{
    delete ui;
}

void ScenarioWindow::closeEvent(QCloseEvent* event)
{
    if (m_runOrchestrator && m_runOrchestrator->isRunning())
    {
        const auto res = QMessageBox::question(this, "Close",
                                               "A run is still in progress. Cancel and close?");
        if (res != QMessageBox::Yes)
        {
            event->ignore();
            return;
        }
        m_runOrchestrator->cancel();
    }
    QDialog::closeEvent(event);
}

QJsonObject ScenarioWindow::buildScenarioPresetJson(const QString& scenario) const
{
    if (m_mainWindow)
        return m_mainWindow->buildScenarioPresetJson(scenario);

    // Fallback: minimal valid shape.
    QJsonObject root;
    root.insert("mode", "OperatingMapSweep");
    root.insert("axes", QJsonObject{});
    return root;
}

void ScenarioWindow::applyScenarioPreset(const QString& scenario)
{
    if (m_suppressScenarioApply)
        return;
    if (!ui || !ui->sweepConfigJson)
        return;
    const QJsonObject root = buildScenarioPresetJson(scenario);
    ui->sweepConfigJson->setPlainText(QString::fromUtf8(QJsonDocument(root).toJson(QJsonDocument::Indented)));
    updateScenarioHint(scenario);
    updatePreflight();
}

bool ScenarioWindow::parseSweepJsonText(QJsonDocument* doc, QString* error) const
{
    if (!doc)
        return false;
    *doc = {};

    const QString text = ui->sweepConfigJson ? ui->sweepConfigJson->toPlainText() : QString();
    if (text.trimmed().isEmpty())
    {
        if (error)
            *error = "Sweep JSON is empty";
        return false;
    }
    QJsonParseError parseError{};
    const QJsonDocument json = QJsonDocument::fromJson(text.toUtf8(), &parseError);
    if (parseError.error != QJsonParseError::NoError)
    {
        if (error)
            *error = QString("JSON parse error at %1: %2").arg(parseError.offset).arg(parseError.errorString());
        return false;
    }
    if (!json.isObject())
    {
        if (error)
            *error = "Sweep JSON must be an object";
        return false;
    }
    *doc = json;
    return true;
}

bool ScenarioWindow::saveSweepJsonToPath(const QString& path, QString* error) const
{
    QJsonDocument doc;
    if (!parseSweepJsonText(&doc, error))
        return false;

    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        if (error)
            *error = QString("Failed to write %1").arg(path);
        return false;
    }
    file.write(doc.toJson(QJsonDocument::Indented));
    return true;
}

bool ScenarioWindow::loadSweepJsonFromPath(const QString& path, QString* error)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        if (error)
            *error = QString("Failed to open %1").arg(path);
        return false;
    }
    const QByteArray bytes = file.readAll();
    QJsonParseError parseError{};
    const QJsonDocument doc = QJsonDocument::fromJson(bytes, &parseError);
    if (parseError.error != QJsonParseError::NoError || !doc.isObject())
    {
        if (error)
            *error = QString("JSON parse error: %1").arg(parseError.errorString());
        return false;
    }
    ui->sweepConfigJson->setPlainText(QString::fromUtf8(doc.toJson(QJsonDocument::Indented)));
    return true;
}

void ScenarioWindow::appendRunLog(const QString& text)
{
    if (!ui || !ui->runLog)
        return;
    ui->runLog->appendPlainText(text);
    QTextCursor cursor = ui->runLog->textCursor();
    cursor.movePosition(QTextCursor::End);
    ui->runLog->setTextCursor(cursor);
}

void ScenarioWindow::refreshArtifacts(const QString& outDir)
{
    if (!ui || !ui->artifactList)
        return;
    QDir dir(outDir);
    if (!dir.exists())
        return;
    ui->artifactList->clear();
    const QStringList files = dir.entryList(QDir::Files, QDir::Name);
    for (const QString& file : files)
    {
        auto* item = new QListWidgetItem(file, ui->artifactList);
        item->setData(Qt::UserRole, dir.filePath(file));
    }
}

void ScenarioWindow::setRunUiEnabled(bool enabled)
{
    m_runUiEnabled = enabled;
    ui->pbCancelRun->setEnabled(!enabled);
    ui->scenarioPreset->setEnabled(enabled);
    ui->sweepConfigJson->setReadOnly(!enabled);
    updatePreflight();
}

void ScenarioWindow::on_scenarioPreset_currentIndexChanged(int)
{
    if (!ui || !ui->scenarioPreset)
        return;
    applyScenarioPreset(ScenarioKeyFromUi(ui->scenarioPreset));
}

void ScenarioWindow::on_pbSweepLoad_clicked()
{
    const QString configsDir = ResolveSiblingDir("configs");
    const QString startDir = configsDir.isEmpty() ? QDir::currentPath() : configsDir;
    const QString path = QFileDialog::getOpenFileName(this, "Load Sweep Config", startDir, "JSON Files (*.json)");
    if (path.isEmpty())
        return;
    QString error;
    if (!loadSweepJsonFromPath(path, &error))
    {
        QMessageBox::warning(this, "Load Sweep Config", error);
        return;
    }
    ui->sweepConfigPath->setText(path);
}

void ScenarioWindow::on_pbSweepSave_clicked()
{
    QString path = ui->sweepConfigPath ? ui->sweepConfigPath->text().trimmed() : QString();
    if (path.isEmpty())
    {
        const QString configsDir = ResolveSiblingDir("configs");
        const QString startDir = configsDir.isEmpty() ? QDir::currentPath() : configsDir;
        path = QFileDialog::getSaveFileName(this, "Save Sweep Config", startDir, "JSON Files (*.json)");
    }
    if (path.isEmpty())
        return;
    QString error;
    if (!saveSweepJsonToPath(path, &error))
    {
        QMessageBox::warning(this, "Save Sweep Config", error);
        return;
    }
    ui->sweepConfigPath->setText(path);
}

void ScenarioWindow::on_pbSweepFromUi_clicked()
{
    if (!m_mainWindow)
        return;
    const QJsonObject root = m_mainWindow->buildSweepJsonFromUi();
    ui->sweepConfigJson->setPlainText(QString::fromUtf8(QJsonDocument(root).toJson(QJsonDocument::Indented)));
}

void ScenarioWindow::on_pbBrowseCycle_clicked()
{
    const QString cyclesDir = ResolveSiblingDir("cycles");
    const QString startDir = cyclesDir.isEmpty() ? QDir::currentPath() : cyclesDir;
    const QString path = QFileDialog::getOpenFileName(this, "Select Cycle CSV", startDir, "CSV Files (*.csv)");
    if (path.isEmpty())
        return;
    if (ui && ui->cycleCsvPath)
        ui->cycleCsvPath->setText(path);
}

void ScenarioWindow::on_pbCycleExamples_clicked()
{
    if (!ui || !ui->cycleCsvPath || !ui->pbCycleExamples)
        return;

    const QString cyclesDir = ResolveSiblingDir("cycles");
    if (cyclesDir.isEmpty())
    {
        QMessageBox::information(this, "Examples", "No 'cycles' directory found next to the app (or current working directory).");
        return;
    }

    QDir dir(cyclesDir);
    const QStringList files = dir.entryList(QStringList{"*.csv", "*.CSV"}, QDir::Files, QDir::Name);
    if (files.isEmpty())
    {
        QMessageBox::information(this, "Examples", QString("No cycle CSVs found in: %1").arg(cyclesDir));
        return;
    }

    QMenu menu(this);
    for (const QString& file : files)
    {
        QAction* act = menu.addAction(file);
        act->setData(dir.filePath(file));
    }
    QAction* chosen = menu.exec(ui->pbCycleExamples->mapToGlobal(QPoint(0, ui->pbCycleExamples->height())));
    if (!chosen)
        return;
    const QString path = chosen->data().toString();
    if (!path.isEmpty())
        ui->cycleCsvPath->setText(path);
}

void ScenarioWindow::on_pbBrowsePython_clicked()
{
    const QString startDir = QDir::currentPath();
#ifdef Q_OS_WIN
    const QString filter = "Python Executable (python.exe);;All Files (*.*)";
#else
    const QString filter = "All Files (*)";
#endif
    const QString path = QFileDialog::getOpenFileName(this, "Select Python Executable", startDir, filter);
    if (path.isEmpty())
        return;
    if (ui && ui->pythonExePath)
        ui->pythonExePath->setText(path);
}

void ScenarioWindow::on_pbRunScenario_clicked()
{
    if (!m_mainWindow || !m_runOrchestrator || m_runOrchestrator->isRunning())
        return;

    updatePreflight();
    if (!m_preflightOk)
    {
        bool ok = true;
        const QString msg = preflightMessage(&ok);
        QMessageBox::warning(this, "Run", msg.isEmpty() ? "Preflight failed" : msg);
        return;
    }

    m_mainWindow->enforceSimulationSafeParams(nullptr);

    const QString path = ui->sweepConfigPath ? ui->sweepConfigPath->text().trimmed() : QString();
    if (path.isEmpty())
    {
        QMessageBox::warning(this, "Run", "Sweep JSON path is empty");
        return;
    }

    QString error;
    if (!saveSweepJsonToPath(path, &error))
    {
        QMessageBox::warning(this, "Run", error);
        return;
    }

    QJsonDocument doc;
    if (!parseSweepJsonText(&doc, &error))
    {
        QMessageBox::warning(this, "Run", error);
        return;
    }

    const QString outDir = ui->sweepOutDir ? ui->sweepOutDir->text().trimmed() : QString();
    if (outDir.isEmpty())
    {
        QMessageBox::warning(this, "Run", "Output directory is empty");
        return;
    }

    const QString cycleCsv = ui->cycleCsvPath ? ui->cycleCsvPath->text().trimmed() : QString();
    const QString pythonExe = ui->pythonExePath ? ui->pythonExePath->text().trimmed() : QString();

    RunOrchestrator::Plan plan;
    plan.sweep_config_path = path;
    plan.sweep_doc = doc;
    plan.out_dir = outDir;
    plan.cycle_csv = cycleCsv;
    plan.python_exe = pythonExe;
    plan.allow_low_coverage = ui->cbRunAnywayCoverage && ui->cbRunAnywayCoverage->isChecked();
    const QJsonObject rootObj = doc.object();
    const QJsonObject lutObj = rootObj.value("lut").toObject();
    const QString phiSource = lutObj.value("phi_source").toString().trimmed();
    plan.phi_source = phiSource.isEmpty() ? rootObj.value("phi_source").toString().trimmed() : phiSource;

    const QString scenario = ScenarioKeyFromUi(ui->scenarioPreset);
    const QString scenarioLabel = ui->scenarioPreset ? ui->scenarioPreset->currentText().trimmed() : scenario;
    if (scenario == "Operating Point" || scenario == "Operating Map")
    {
        plan.do_sweep = true;
    }
    else if (scenario == "FixedPointFreqSweep")
    {
        plan.do_sweep = true;
        plan.do_report = true; // summary-only report (freq sweep curves)
    }
    else if (scenario == "Driving Cycle")
    {
        plan.do_cycle = true;
        plan.do_report = true;
    }
    else if (scenario == "Full Pipeline")
    {
        plan.do_sweep = true;
        plan.do_lut = true;
        plan.do_cycle = true;
        plan.do_report = true;
    }

    if ((plan.do_cycle || plan.do_report) && cycleCsv.isEmpty())
    {
        QMessageBox::warning(this, "Run", "Cycle CSV is empty");
        return;
    }

    sim::SweepContext ctx;
    sim::PowerModuleParams moduleParams;
    if (plan.do_sweep && !m_mainWindow->buildSweepContext(&ctx, &moduleParams, &error))
    {
        QMessageBox::warning(this, "Run", error);
        return;
    }

    if (plan.do_sweep)
    {
        if (m_mainWindow->motor)
            m_mainWindow->motor->Restart();
        sim::SimInit init{};
        init.motor = m_mainWindow->motor;
        m_mainWindow->m_simRunner.Reset(init);
    }

    ui->runLog->clear();
    ui->runProgress->setValue(0);
    ui->runStageLabel->setText("Starting");
    setRunUiEnabled(false);
    refreshArtifacts(outDir);
    appendRunLog(QString("Starting scenario: %1").arg(scenarioLabel));

    m_runOrchestrator->start(plan, ctx);
}

void ScenarioWindow::on_pbHelp_clicked()
{
    const QString html =
        "<h3>DPWM ROI workflow (what to click)</h3>"
        "<p><b>Recommended:</b> pick <b>Full Pipeline</b>, set <b>Output Dir</b>, choose a <b>Cycle CSV</b> (e.g. "
        "<code>cycles/ece_like_iq.csv</code>), and <b>Load</b> a map config (e.g. <code>configs/map_coarse.json</code>), then press <b>Run</b>.</p>"
        "<p>This produces: <code>summary.csv</code>, <code>lut.json</code>, <code>lut_pwm_strategy.h</code>, <code>lut_runtime_params.h</code>, "
        "<code>cycle_eval.json</code>, and <code>report.pdf</code> in the Output Dir.</p>"
        "<h3>What each field means</h3>"
        "<ul>"
        "<li><b>Scenario</b>: selects which pipeline stages run.</li>"
        "<li><b>Sweep JSON</b>: the config file path. The JSON editor is saved to this path before running.</li>"
        "<li><b>Output Dir</b>: where artifacts are written (created if needed).</li>"
        "<li><b>Cycle CSV</b>: required for Driving Cycle / Full Pipeline. Format: motor-space columns "
        "<code>t_s,speed_rpm,iq_A,id_A</code>.</li>"
        "<li><b>Python</b>: optional override for the Python executable used by cycle eval/report.</li>"
        "</ul>"
        "<h3>Important: PWM carrier frequencies</h3>"
        "<p><code>f_sw_Hz</code> is the PWM carrier frequency, not the control-loop frequency. "
        "Firmware supports only <b>4400 / 8800 / 17600 Hz</b>. Configs containing any other values will error.</p>"
        "<h3>Quick sanity checks</h3>"
        "<ul>"
        "<li>Fixed-point carrier sweep: <code>configs/freq_sweep_quick.json</code></li>"
        "<li>Coarse LUT map: <code>configs/map_coarse.json</code></li>"
        "<li>High-speed refine map (optional): <code>configs/map_highspeed_refine.json</code></li>"
        "</ul>";

    QMessageBox box(this);
    box.setWindowTitle("Sweeps / LUT / Report - Help");
    box.setTextFormat(Qt::RichText);
    box.setText(html);
    box.exec();
}

void ScenarioWindow::on_pbCancelRun_clicked()
{
    if (m_runOrchestrator && m_runOrchestrator->isRunning())
    {
        appendRunLog("Cancel requested");
        m_runOrchestrator->cancel();
    }
}

void ScenarioWindow::on_pbOpenOutDir_clicked()
{
    const QString outDir = ui->sweepOutDir ? ui->sweepOutDir->text().trimmed() : QString();
    if (outDir.isEmpty())
        return;
    QDesktopServices::openUrl(QUrl::fromLocalFile(QDir(outDir).absolutePath()));
}

void ScenarioWindow::on_pbOpenArtifact_clicked()
{
    if (!ui || !ui->artifactList)
        return;
    QListWidgetItem* item = ui->artifactList->currentItem();
    if (!item)
        return;
    const QString path = item->data(Qt::UserRole).toString();
    if (path.isEmpty())
        return;
    QDesktopServices::openUrl(QUrl::fromLocalFile(path));
}

void ScenarioWindow::updateScenarioHint(const QString& scenario)
{
    if (!ui || !ui->scenarioHint)
        return;
    const QString key = scenario.trimmed();
    if (key == "Operating Point")
        ui->scenarioHint->setText("Operating Point: runs a single operating point using the JSON 'point' (or axes with 1 value each). Writes summary.csv.");
    else if (key == "FixedPointFreqSweep")
        ui->scenarioHint->setText("Fixed Point f_sw sweep: runs the same operating point across f_sw_Hz values and generates a frequency-sweep PDF report (no LUT/cycle). Allowed f_sw_Hz: 4400/8800/17600.");
    else if (key == "Operating Map")
        ui->scenarioHint->setText("Operating Map: runs a grid over (speed_rpm, iq_A, id_A, ...) and writes summary.csv for LUT building.");
    else if (key == "Driving Cycle")
        ui->scenarioHint->setText("Driving Cycle: runs Cycle Eval + Report only (requires existing out_dir/summary.csv + out_dir/lut.json + a cycle CSV).");
    else if (key == "Full Pipeline")
        ui->scenarioHint->setText("Full Pipeline: runs Sweep -> LUT -> Cycle Eval -> Report (recommended ROI flow).");
    else
        ui->scenarioHint->setText(QString());
}

bool ScenarioWindow::canCreateOutputDir(const QString& outDir, QString* reason) const
{
    if (outDir.isEmpty())
    {
        if (reason)
            *reason = "output directory is empty";
        return false;
    }
    QFileInfo info(outDir);
    if (info.exists())
        return true;
    QDir parent = info.absoluteDir();
    if (!parent.exists())
    {
        if (reason)
            *reason = "output directory parent does not exist";
        return false;
    }
    if (!QFileInfo(parent.absolutePath()).isWritable())
    {
        if (reason)
            *reason = "output directory parent is not writable";
        return false;
    }
    return true;
}

QString ScenarioWindow::preflightMessage(bool* ok) const
{
    if (ok)
        *ok = true;
    if (!ui || !ui->scenarioPreset)
        return {};

    const QString scenario = ScenarioKeyFromUi(ui->scenarioPreset);
    const QString outDir = ui->sweepOutDir ? ui->sweepOutDir->text().trimmed() : QString();
    const QString cycleCsv = ui->cycleCsvPath ? ui->cycleCsvPath->text().trimmed() : QString();
    const QString sweepPath = ui->sweepConfigPath ? ui->sweepConfigPath->text().trimmed() : QString();

    QStringList missing;

    const bool scenarioRunsSweep = (scenario == "Operating Point" ||
                                    scenario == "FixedPointFreqSweep" ||
                                    scenario == "Operating Map" ||
                                    scenario == "Full Pipeline");
    if (scenarioRunsSweep && sweepPath.isEmpty())
        missing << "Sweep JSON path (empty)";

    if (scenario == "Driving Cycle")
    {
        if (cycleCsv.isEmpty() || !QFile::exists(cycleCsv))
            missing << (cycleCsv.isEmpty() ? "cycle CSV (path empty)" : QString("cycle CSV not found: %1").arg(cycleCsv));

        if (outDir.isEmpty())
        {
            missing << "output directory (empty)";
        }
        else
        {
            const QString summaryPath = QDir(outDir).filePath("summary.csv");
            const QString lutPath = QDir(outDir).filePath("lut.json");
            if (!QFile::exists(summaryPath))
                missing << QString("%1/summary.csv").arg(outDir);
            if (!QFile::exists(lutPath))
                missing << QString("%1/lut.json").arg(outDir);

            // Cycle Eval requires a map (more than one speed/iq bin).
            if (QFile::exists(summaryPath))
            {
                int speedBins = 0, iqBins = 0;
                QString statsErr;
                ReadSummaryMinMaxAndBins(summaryPath, nullptr, nullptr, nullptr, nullptr, &speedBins, &iqBins, &statsErr);
                if (speedBins <= 1 || iqBins <= 1)
                    missing << "summary.csv has only a single speed/iq bin (FixedPointFreqSweep output); cannot run Cycle Eval. Run OperatingMapSweep (configs/map_coarse.json) first.";
            }
        }
    }
    else if (scenario == "Full Pipeline")
    {
        if (cycleCsv.isEmpty() || !QFile::exists(cycleCsv))
            missing << (cycleCsv.isEmpty() ? "cycle CSV (path empty)" : QString("cycle CSV not found: %1").arg(cycleCsv));

        QString reason;
        if (!canCreateOutputDir(outDir, &reason))
            missing << QString("output directory not creatable: %1").arg(reason);
    }

    // If we can parse the JSON editor, validate f_sw_Hz early so the user doesn't have to run to discover it.
    // Only relevant when the scenario actually runs a sweep.
    if (scenarioRunsSweep)
    {
        QJsonDocument doc;
        QString parseError;
        if (parseSweepJsonText(&doc, &parseError))
        {
            const QStringList invalidFsw = UnsupportedFswFromDoc(doc);
            if (!invalidFsw.isEmpty())
                missing << QString("unsupported f_sw_Hz: %1 (allowed: 4400/8800/17600)").arg(invalidFsw.join(", "));

            if (scenario == "Full Pipeline")
            {
                const QJsonObject root = doc.object();
                const QString mode = root.value("mode").toString("OperatingMapSweep");
                if (IsFixedPointMode(mode))
                {
                    missing << "FixedPointFreqSweep produces a single operating point; cannot run Cycle Eval. Use OperatingMapSweep (configs/map_coarse.json) or choose a non-cycle scenario.";
                }
                const QJsonObject axes = root.value("axes").toObject();
                const int speedBins = AxisBinCount(axes.value("speed_rpm"));
                const int iqBins = AxisBinCount(axes.value("iq_A"));
                if (speedBins <= 1 || iqBins <= 1)
                    missing << QString("map axes too small for cycle eval (need >1 speed_rpm and >1 iq_A bin; got speed=%1 iq=%2)").arg(speedBins).arg(iqBins);
            }
        }
    }

    // Coverage preflight (Driving Cycle only): if cycle spends lots of time outside the map,
    // results are dominated by clamping and are not useful for ROI.
    if (scenario == "Driving Cycle" && !outDir.isEmpty() && !cycleCsv.isEmpty() && QFile::exists(cycleCsv))
    {
        const QString summaryPath = QDir(outDir).filePath("summary.csv");
        if (QFile::exists(summaryPath))
        {
            double cycleSMin = 0, cycleSMax = 0, cycleQMin = 0, cycleQMax = 0;
            int cycleN = 0;
            QString cycleErr;
            if (ReadCycleMinMax(cycleCsv, &cycleSMin, &cycleSMax, &cycleQMin, &cycleQMax, &cycleN, &cycleErr))
            {
                double mapSMin = 0, mapSMax = 0, mapQMin = 0, mapQMax = 0;
                int speedBins = 0, iqBins = 0;
                QString mapErr;
                if (ReadSummaryMinMaxAndBins(summaryPath, &mapSMin, &mapSMax, &mapQMin, &mapQMax, &speedBins, &iqBins, &mapErr) &&
                    std::isfinite(mapSMin) && std::isfinite(mapSMax) && std::isfinite(mapQMin) && std::isfinite(mapQMax))
                {
                    double covPct = 0.0;
                    int inBounds = 0;
                    int total = 0;
                    QString covErr;
                    const bool covOk = ComputeCoveragePct(cycleCsv, mapSMin, mapSMax, mapQMin, mapQMax,
                                                          &covPct, &inBounds, &total, &covErr);
                    if (covOk && covPct < 80.0 && !(ui->cbRunAnywayCoverage && ui->cbRunAnywayCoverage->isChecked()))
                    {
                        missing << QString("map coverage is only %1% (in-bounds %2/%3). Check 'Run anyway' to proceed, or run a wider map sweep.")
                                       .arg(QString::number(covPct, 'f', 2))
                                       .arg(inBounds)
                                       .arg(total);
                    }
                    else if (!covOk && !covErr.isEmpty())
                    {
                        missing << QString("coverage preflight failed: %1").arg(covErr);
                    }
                }
            }
        }
    }

    if (!missing.isEmpty())
    {
        if (ok)
            *ok = false;
        return QString("Preflight issues: %1").arg(missing.join(", "));
    }

    return {};
}

void ScenarioWindow::updatePreflight()
{
    bool ok = true;
    const QString message = preflightMessage(&ok);
    m_preflightOk = ok;
    if (ui && ui->preflightStatus)
    {
        ui->preflightStatus->setText(message);
        ui->preflightStatus->setVisible(!message.isEmpty());
    }
    if (ui && ui->pbRunScenario)
        ui->pbRunScenario->setEnabled(m_runUiEnabled && m_preflightOk);
}
