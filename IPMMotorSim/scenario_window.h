#ifndef SCENARIO_WINDOW_H
#define SCENARIO_WINDOW_H

#include <QDialog>
#include <QJsonDocument>
#include <QString>

#include "sim/sweep/sweep_runner.h"

namespace Ui
{
class ScenarioWindow;
}

class MainWindow;
class RunOrchestrator;

class ScenarioWindow : public QDialog
{
    Q_OBJECT

public:
    explicit ScenarioWindow(MainWindow* mainWindow, QWidget* parent = nullptr);
    ~ScenarioWindow() override;

protected:
    void closeEvent(QCloseEvent* event) override;

private slots:
    void on_scenarioPreset_currentIndexChanged(int index);
    void on_pbSweepLoad_clicked();
    void on_pbSweepSave_clicked();
    void on_pbSweepFromUi_clicked();
    void on_pbRunScenario_clicked();
    void on_pbBrowseCycle_clicked();
    void on_pbCycleExamples_clicked();
    void on_pbBrowsePython_clicked();
    void on_pbHelp_clicked();
    void on_pbCancelRun_clicked();
    void on_pbOpenOutDir_clicked();
    void on_pbOpenArtifact_clicked();

private:
    QJsonObject buildScenarioPresetJson(const QString& scenario) const;
    void applyScenarioPreset(const QString& scenario);
    void updatePreflight();
    void updateScenarioHint(const QString& scenario);
    QString preflightMessage(bool* ok) const;
    bool canCreateOutputDir(const QString& outDir, QString* reason) const;

    bool parseSweepJsonText(QJsonDocument* doc, QString* error) const;
    bool saveSweepJsonToPath(const QString& path, QString* error) const;
    bool loadSweepJsonFromPath(const QString& path, QString* error);

    void appendRunLog(const QString& text);
    void refreshArtifacts(const QString& outDir);
    void setRunUiEnabled(bool enabled);

    MainWindow* m_mainWindow = nullptr; // non-owning
    Ui::ScenarioWindow* ui = nullptr;
    RunOrchestrator* m_runOrchestrator = nullptr;
    bool m_suppressScenarioApply = false;
    bool m_runUiEnabled = true;
    bool m_preflightOk = true;
};

#endif // SCENARIO_WINDOW_H
