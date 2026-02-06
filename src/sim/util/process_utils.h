#ifndef SIM_UTIL_PROCESS_UTILS_H
#define SIM_UTIL_PROCESS_UTILS_H

#include <QString>
#include <QStringList>
#include <atomic>
#include <functional>

namespace sim
{
struct ProcessOutput
{
    int exit_code = -1;
    QString std_out;
    QString std_err;
};

enum class ProcessStatus
{
    Ok,
    FailedToStart,
    Cancelled,
    Error
};

ProcessStatus RunProcess(const QString& program,
                        const QStringList& args,
                        const QString& workdir,
                        ProcessOutput* output,
                        QString* error,
                        std::atomic<bool>* cancel = nullptr,
                        std::function<void(const QString&)> on_stdout = {},
                        std::function<void(const QString&)> on_stderr = {});

ProcessStatus RunPython(const QString& script_path,
                        const QStringList& args,
                        const QString& python_exe,
                        const QString& workdir,
                        ProcessOutput* output,
                        QString* error,
                        std::atomic<bool>* cancel = nullptr,
                        std::function<void(const QString&)> on_stdout = {},
                        std::function<void(const QString&)> on_stderr = {});

// Resolve a repo-relative tool path (like "tools/report/make_report.py") from either
// the current working directory or the application directory (useful when running
// from a packaged build output).
QString ResolveToolPath(const QString& relative_path,
                        const QString& app_dir,
                        const QString& cwd);

// Choose a working directory for a resolved tool path. Prefer the directory
// that contains "tools/" (repo root / app root), so the tool can reference
// other files via stable relative paths.
QString ResolveToolWorkdir(const QString& tool_path);

QString DefaultPythonExecutable();
} // namespace sim

#endif // SIM_UTIL_PROCESS_UTILS_H
