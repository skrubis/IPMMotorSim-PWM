#ifndef RUN_ORCHESTRATOR_H
#define RUN_ORCHESTRATOR_H

#include <QObject>
#include <QFuture>
#include <QString>
#include <QJsonDocument>
#include <atomic>

#include "sim/sweep/sweep_runner.h"

class RunOrchestrator : public QObject
{
    Q_OBJECT
public:
    struct Plan
    {
        QString sweep_config_path;
        QJsonDocument sweep_doc;
        QString out_dir;
        QString cycle_csv;
        QString python_exe;
        QString phi_source;
        bool allow_low_coverage = false;
        bool do_sweep = false;
        bool do_lut = false;
        bool do_cycle = false;
        bool do_report = false;
    };

    explicit RunOrchestrator(QObject* parent = nullptr);

    bool isRunning() const;
    void start(const Plan& plan, const sim::SweepContext& ctx);
    void cancel();

signals:
    void stageChanged(const QString& stage);
    void progressUpdated(const sim::SweepProgress& progress);
    void logMessage(const QString& message);
    void artifactsChanged(const QString& out_dir);
    void finished(bool ok, const QString& message);

private:
    void runInternal();

    Plan m_plan;
    sim::SweepContext m_ctx{};
    std::atomic<bool> m_cancel{false};
    std::atomic<bool> m_running{false};
    QFuture<void> m_future;
};

#endif // RUN_ORCHESTRATOR_H
