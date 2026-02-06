#include "sim/util/process_utils.h"

#include <QProcess>
#include <QDir>
#include <QFile>
#include <QFileInfo>

namespace sim
{
static void AppendOutput(QProcess* process,
                         QString* buffer,
                         std::function<void(const QString&)> callback,
                         bool stderr_stream)
{
    if (!process)
        return;
    QByteArray bytes = stderr_stream ? process->readAllStandardError() : process->readAllStandardOutput();
    if (bytes.isEmpty())
        return;
    const QString text = QString::fromUtf8(bytes);
    if (buffer)
        buffer->append(text);
    if (callback)
        callback(text);
}

ProcessStatus RunProcess(const QString& program,
                        const QStringList& args,
                        const QString& workdir,
                        ProcessOutput* output,
                        QString* error,
                        std::atomic<bool>* cancel,
                        std::function<void(const QString&)> on_stdout,
                        std::function<void(const QString&)> on_stderr)
{
    if (error)
        error->clear();
    if (output)
        *output = ProcessOutput{};

    QProcess process;
    if (!workdir.isEmpty())
        process.setWorkingDirectory(workdir);
    process.setProgram(program);
    process.setArguments(args);
    process.setProcessChannelMode(QProcess::SeparateChannels);

    process.start();
    if (!process.waitForStarted())
    {
        if (error)
            *error = QString("Failed to start '%1': %2").arg(program, process.errorString());
        return ProcessStatus::FailedToStart;
    }

    while (true)
    {
        if (cancel && cancel->load())
        {
            process.kill();
            process.waitForFinished(2000);
            if (error)
                *error = "Cancelled";
            return ProcessStatus::Cancelled;
        }

        if (process.waitForFinished(100))
            break;

        AppendOutput(&process, output ? &output->std_out : nullptr, on_stdout, false);
        AppendOutput(&process, output ? &output->std_err : nullptr, on_stderr, true);
    }

    AppendOutput(&process, output ? &output->std_out : nullptr, on_stdout, false);
    AppendOutput(&process, output ? &output->std_err : nullptr, on_stderr, true);

    if (output)
        output->exit_code = process.exitCode();

    if (process.exitStatus() != QProcess::NormalExit)
    {
        if (error)
            *error = QString("Process crashed: %1").arg(program);
        return ProcessStatus::Error;
    }

    if (process.exitCode() != 0)
    {
        if (error)
            *error = QString("Process failed with exit code %1").arg(process.exitCode());
        return ProcessStatus::Error;
    }

    return ProcessStatus::Ok;
}

ProcessStatus RunPython(const QString& script_path,
                        const QStringList& args,
                        const QString& python_exe,
                        const QString& workdir,
                        ProcessOutput* output,
                        QString* error,
                        std::atomic<bool>* cancel,
                        std::function<void(const QString&)> on_stdout,
                        std::function<void(const QString&)> on_stderr)
{
    const QString exe = python_exe.isEmpty() ? DefaultPythonExecutable() : python_exe;
    QStringList allArgs;
    allArgs << script_path;
    allArgs << args;
    return RunProcess(exe, allArgs, workdir, output, error, cancel, on_stdout, on_stderr);
}

QString ResolveToolPath(const QString& relative_path,
                        const QString& app_dir,
                        const QString& cwd)
{
    const QString rel = QDir::cleanPath(relative_path);
    QString relForward = rel;
    relForward.replace('\\', '/');
    const QString appDir = app_dir.isEmpty() ? QDir::currentPath() : app_dir;
    const QStringList candidates{
        QDir(cwd).filePath(rel),
        QDir(appDir).filePath(rel),
        QDir(appDir).filePath(QString("../%1").arg(rel)),
        QDir(appDir).filePath(QString("../../%1").arg(rel)),
        QDir(cwd).filePath(relForward),
        QDir(appDir).filePath(relForward)
    };
    for (const QString& path : candidates)
    {
        if (QFile::exists(path))
            return QDir::cleanPath(path);
    }
    return rel;
}

QString ResolveToolWorkdir(const QString& tool_path)
{
    const QString p = QDir::cleanPath(tool_path);
    int idx = p.indexOf("/tools/");
    if (idx < 0)
        idx = p.indexOf("\\tools\\");
    if (idx >= 0)
    {
        const QString root = p.left(idx);
        if (!root.isEmpty())
            return QDir(root).absolutePath();
    }
    return QFileInfo(p).dir().absolutePath();
}

QString DefaultPythonExecutable()
{
#ifdef Q_OS_WIN
    return "python";
#else
    return "python3";
#endif
}
} // namespace sim
