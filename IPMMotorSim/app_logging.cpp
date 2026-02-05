#include "app_logging.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QMutex>
#include <QTextStream>
#include <QThread>

namespace app
{
namespace
{
QMutex& LogMutex()
{
    static QMutex mutex;
    return mutex;
}

QString TimestampUtc()
{
    return QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);
}

QDir LogDir()
{
    const QString base = QCoreApplication::applicationDirPath().isEmpty()
                             ? QDir::currentPath()
                             : QCoreApplication::applicationDirPath();
    QDir dir(base);
    if (!dir.exists("logs"))
        dir.mkpath("logs");
    dir.cd("logs");
    return dir;
}

QFile& LogFile()
{
    static QFile file;
    return file;
}

QTextStream& LogStream()
{
    static QTextStream stream;
    return stream;
}

QtMessageHandler& PreviousHandler()
{
    static QtMessageHandler prev = nullptr;
    return prev;
}

const char* TypeName(QtMsgType type)
{
    switch (type)
    {
        case QtDebugMsg:
            return "DEBUG";
        case QtInfoMsg:
            return "INFO";
        case QtWarningMsg:
            return "WARN";
        case QtCriticalMsg:
            return "CRIT";
        case QtFatalMsg:
            return "FATAL";
        default:
            return "LOG";
    }
}

void MessageHandler(QtMsgType type, const QMessageLogContext& context, const QString& message)
{
    QMutexLocker locker(&LogMutex());

    if (LogFile().isOpen())
    {
        const quintptr tid = reinterpret_cast<quintptr>(QThread::currentThreadId());
        LogStream() << TimestampUtc() << " [" << TypeName(type) << "]"
                    << " tid=0x" << QString::number(tid, 16);

        if (context.category && *context.category)
            LogStream() << " cat=" << context.category;
        if (context.file && *context.file)
            LogStream() << " file=" << context.file;
        if (context.line > 0)
            LogStream() << ":" << context.line;
        if (context.function && *context.function)
            LogStream() << " fn=" << context.function;

        LogStream() << " msg=" << message << "\n";
        LogStream().flush();
    }

    if (PreviousHandler())
        PreviousHandler()(type, context, message);
}
} // namespace

void InitLogging()
{
    QMutexLocker locker(&LogMutex());
    if (LogFile().isOpen())
        return;

    const QString stamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    const QString path = LogDir().filePath(QString("app_%1.log").arg(stamp));

    LogFile().setFileName(path);
    if (!LogFile().open(QIODevice::WriteOnly | QIODevice::Text))
        return;

    LogStream().setDevice(&LogFile());
    LogStream().setLocale(QLocale::c());

    PreviousHandler() = qInstallMessageHandler(MessageHandler);

    LogStream() << "# ipmmotorsim_app_log_version=1\n";
    LogStream() << "# started_utc=" << TimestampUtc() << "\n";
    LogStream() << "# app_dir=" << QCoreApplication::applicationDirPath() << "\n";
    LogStream() << "# cwd=" << QDir::currentPath() << "\n";
    LogStream().flush();
}

void Breadcrumb(const QString& message)
{
    QMutexLocker locker(&LogMutex());

    QFile f(LogDir().filePath("last_breadcrumb.txt"));
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate))
        return;

    QTextStream ts(&f);
    ts.setLocale(QLocale::c());
    ts << TimestampUtc() << " " << message << "\n";
    ts.flush();
}
} // namespace app

