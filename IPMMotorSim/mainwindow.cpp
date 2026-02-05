/*
 * This file is part of the IPMMotorSim project
 *
 * Copyright (C) 2022 Pete9008 <openinverter.org>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <algorithm>
#include <cmath>
#include <QtMath>
#include <QDateTime>
#include <QCoreApplication>
#include <QDoubleValidator>
#include <QEvent>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QAction>
#include <QHelpEvent>
#include <QIntValidator>
#include <QFormLayout>
#include <QLabel>
#include <QLocale>
#include <QRandomGenerator>
#include <QRegularExpression>
#include <QSignalBlocker>
#include <QSettings>
#include <QTextStream>
#include <QToolTip>
#include <QApplication>
#include <QCursor>
#include <QGuiApplication>
#include <QScreen>
#include <limits>
#include "pwmgeneration.h"
#include "foc.h"
#include "params.h"
#include "app_logging.h"
#include "inc_encoder.h"
#include "teststubs.h"
#include "my_math.h"
#include "sim/controller.h"
#include "sim/inverter_switching_model.h"
#include "sim/modulator.h"

#define GPIOA 0
#define GPIOB 1
#define GPIOC 2
#include "anain.h"

//Current graph
#define IA 1
#define IB 2
#define IC 3
#define IQ 4
#define ID 5

//Simulation graph
#define M_RPM 1
#define M_MOTOR_POS 2
#define M_CONT_POS 3

//Controller graph
#define VA 1
#define VB 2
#define VC 3
#define VQ 4
#define VD 5
#define C_IQ 6
#define C_ID 7
#define C_IFW 8
#define C_IVLIM 9

//Voltage graph
#define VVD 1
#define VVQ 2
#define VVQ_BEMF 3
#define VVQ_DT_ID 4
#define VVD_DT_IQ 5
#define VVQ_DT_RQ 6
#define VVD_DT_RD 7
#define VVLD 8
#define VVLQ 9

//PWM graph
#define PWM_A 1
#define PWM_B 2
#define PWM_C 3
#define PWM_MIN 4
#define PWM_MAX 5
#define PWM_ZEROSEQ 6
#define PWM_CLAMP_A 7
#define PWM_CLAMP_B 8
#define PWM_CLAMP_C 9
#define PWM_T1 10
#define PWM_T2 11
#define PWM_T0 12
#define PWM_SECTOR 13

//Power/Torque graph
#define POWER 6
#define TORQUE 7
#define ELEC_POWER 8
#define EFFICIENCY 9

//Loss graph
#define LOSS_IGBT_COND 10
#define LOSS_DIODE_COND 11
#define LOSS_IGBT_SW 12
#define LOSS_DIODE_RR 13
#define LOSS_TOTAL 14

//Op point graph
#define IDIQAMPS 2

#define TWO_PI_CONT 65536

//c++ test stubs globals
extern volatile uint16_t g_input_angle;
extern volatile double g_il1_input;
extern volatile double g_il2_input;

// C test stubs globals
extern "C" volatile bool disablePWM;

static double PwmFrequencyHzFromParam(int pwmfrq)
{
    switch(pwmfrq)
    {
        case 0: return 17600.0;
        case 1: return 8800.0;
        case 2: return 4400.0;
        default: return 8800.0;
    }
}

template <size_t N>
static std::array<sim::CurvePoint, N> ParseCurvePoints(const QString& text,
                                                       const std::array<sim::CurvePoint, N>& defaults)
{
    std::array<sim::CurvePoint, N> out = defaults;
    int idx = 0;

    const auto lines = text.split(QRegularExpression("[\\r\\n]+"), Qt::SkipEmptyParts);
    for(const QString& rawLine : lines)
    {
        QString line = rawLine.trimmed();
        if(line.isEmpty() || line.startsWith('#'))
            continue;
        line.replace(',', ' ');
        const auto parts = line.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
        if(parts.size() < 3)
            continue;
        bool okI = false, okA = false, okB = false;
        const double current = parts[0].toDouble(&okI);
        const double v25 = parts[1].toDouble(&okA);
        const double v125 = parts[2].toDouble(&okB);
        if(!okI || !okA || !okB)
            continue;
        if(idx >= static_cast<int>(N))
            break;
        out[static_cast<size_t>(idx)] = {current, v25, v125};
        ++idx;
    }

    return out;
}

namespace
{
struct YamlFrame
{
    int indent = 0;
    QString key;
};

struct TempPoint
{
    double v25 = std::numeric_limits<double>::quiet_NaN();
    double v125 = std::numeric_limits<double>::quiet_NaN();
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

static QMap<QString, double> ParseInlineMap(QString text)
{
    text = text.trimmed();
    if (text.startsWith('{'))
        text = text.mid(1);
    if (text.endsWith('}'))
        text.chop(1);
    QMap<QString, double> values;
    const auto parts = text.split(',', Qt::SkipEmptyParts);
    for (const QString& part : parts)
    {
        const int colon = part.indexOf(':');
        if (colon <= 0)
            continue;
        const QString key = part.left(colon).trimmed();
        const QString valueText = part.mid(colon + 1).trimmed();
        bool ok = false;
        const double val = valueText.toDouble(&ok);
        if (ok)
            values.insert(key, val);
    }
    return values;
}

static QString HumanizeKey(const QString& key)
{
    QString out = key;
    out.replace('_', ' ');
    if (!out.isEmpty())
        out[0] = out[0].toUpper();
    return out;
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

static QVector<sim::CurvePoint> ResampleCurvePoints(const QVector<sim::CurvePoint>& raw, int count)
{
    QVector<sim::CurvePoint> points = NormalizeCurvePoints(raw);
    if (points.isEmpty() || count <= 0)
        return {};
    if (points.size() == 1 || count == 1)
        return {points.front()};

    const double start = points.front().current_A;
    double end = points.back().current_A;
    if (end <= start)
        end = start + 1.0;

    auto interp = [&](double x, bool use125)
    {
        if (x <= points.front().current_A)
            return use125 ? points.front().val_125C : points.front().val_25C;
        if (x >= points.back().current_A)
            return use125 ? points.back().val_125C : points.back().val_25C;
        for (int idx = 1; idx < points.size(); ++idx)
        {
            const auto& p0 = points[idx - 1];
            const auto& p1 = points[idx];
            if (x <= p1.current_A)
            {
                const double t = (x - p0.current_A) / (p1.current_A - p0.current_A);
                const double v0 = use125 ? p0.val_125C : p0.val_25C;
                const double v1 = use125 ? p1.val_125C : p1.val_25C;
                return v0 + t * (v1 - v0);
            }
        }
        return use125 ? points.back().val_125C : points.back().val_25C;
    };

    QVector<sim::CurvePoint> out;
    out.reserve(count);
    for (int idx = 0; idx < count; ++idx)
    {
        const double t = count > 1 ? (static_cast<double>(idx) / (count - 1)) : 0.0;
        const double current = start + (end - start) * t;
        const double v25 = interp(current, false);
        const double v125 = interp(current, true);
        out.append({current, v25, v125});
    }
    return out;
}

static QString FormatNumber(double value, int decimals)
{
    const double rounded = std::round(value);
    if (std::abs(value - rounded) < 1e-6)
        return QString::number(rounded, 'f', 0);
    return QString::number(value, 'f', decimals);
}

static QString FormatCurvePointsText(const QVector<sim::CurvePoint>& raw, int count)
{
    const QVector<sim::CurvePoint> points = ResampleCurvePoints(raw, count);
    QStringList lines;
    lines.reserve(points.size());
    for (const auto& point : points)
    {
        const QString current = FormatNumber(point.current_A, 3);
        const QString v25 = QString::number(point.val_25C, 'f', 3);
        const QString v125 = QString::number(point.val_125C, 'f', 3);
        lines.append(QString("%1, %2, %3").arg(current, v25, v125));
    }
    return lines.join('\n');
}

class ScopedSignalBlockerAll
{
public:
    explicit ScopedSignalBlockerAll(QObject* root)
    {
        add(root);
        const auto children = root ? root->findChildren<QObject*>(QString(), Qt::FindChildrenRecursively) : QList<QObject*>{};
        for (QObject* child : children)
            add(child);
    }

    ~ScopedSignalBlockerAll()
    {
        for (const auto& entry : m_prevState)
        {
            if (entry.object)
                entry.object->blockSignals(entry.prevBlocked);
        }
    }

    ScopedSignalBlockerAll(const ScopedSignalBlockerAll&) = delete;
    ScopedSignalBlockerAll& operator=(const ScopedSignalBlockerAll&) = delete;

private:
    struct Entry
    {
        QObject* object{};
        bool prevBlocked{};
    };

    void add(QObject* object)
    {
        if (!object)
            return;
        m_prevState.push_back({object, object->signalsBlocked()});
        object->blockSignals(true);
    }

    QVector<Entry> m_prevState;
};
} // namespace


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setAttribute(Qt::WA_AlwaysShowToolTips, true);

    QSettings settings("OpenInverter", "IPMMotorSim");
    qInfo().noquote() << QString("QSettings: cb_LogCsv contains=%1 value='%2' throttleCurrent contains=%3 value='%4'")
                             .arg(settings.contains("cb_LogCsv") ? "true" : "false")
                             .arg(settings.value("cb_LogCsv").toString())
                             .arg(settings.contains("throttleCurrent") ? "true" : "false")
                             .arg(settings.value("throttleCurrent").toString());
    const bool okGeo = restoreGeometry(settings.value("mainwin/geometry").toByteArray());
    const bool okState = restoreState(settings.value("mainwin/windowState").toByteArray());
    if(!okGeo || !okState)
    {
        settings.remove("mainwin/geometry");
        settings.remove("mainwin/windowState");
    }
    {
        const QRect available = QGuiApplication::primaryScreen()
                                    ? QGuiApplication::primaryScreen()->availableGeometry()
                                    : QRect(0, 0, 1280, 720);
        const QRect current = frameGeometry();
        if(!available.intersects(current) || width() <= 0 || height() <= 0)
        {
            resize(660, 980);
            move(available.center() - rect().center());
        }
    }

    {
        ScopedSignalBlockerAll blockSignals(this);

        if(settings.contains(ui->vehicleWeight->objectName())) ui->vehicleWeight->setText(settings.value(ui->vehicleWeight->objectName(),QString()).toString());
        if(settings.contains(ui->wheelSize->objectName())) ui->wheelSize->setText(settings.value(ui->wheelSize->objectName(),QString()).toString());
        if(settings.contains(ui->gearRatio->objectName())) ui->gearRatio->setText(settings.value(ui->gearRatio->objectName(),QString()).toString());
        if(settings.contains(ui->Vdc->objectName())) ui->Vdc->setText(settings.value(ui->Vdc->objectName(),QString()).toString());
        if(settings.contains(ui->Lq->objectName())) ui->Lq->setText(settings.value(ui->Lq->objectName(),QString()).toString());
        if(settings.contains(ui->Ld->objectName())) ui->Ld->setText(settings.value(ui->Ld->objectName(),QString()).toString());
        if(settings.contains(ui->Rs->objectName())) ui->Rs->setText(settings.value(ui->Rs->objectName(),QString()).toString());
        if(settings.contains(ui->SyncDelay->objectName())) ui->SyncDelay->setText(settings.value(ui->SyncDelay->objectName(),QString()).toString());
        if(settings.contains(ui->LoopFreq->objectName())) ui->LoopFreq->setText(settings.value(ui->LoopFreq->objectName(),QString()).toString());
        if(settings.contains(ui->SamplingPoint->objectName())) ui->SamplingPoint->setText(settings.value(ui->SamplingPoint->objectName(),QString()).toString());
        if(settings.contains(ui->ExtraCycleDelay->objectName())) ui->ExtraCycleDelay->setChecked(settings.value(ui->ExtraCycleDelay->objectName()).toBool());
        if(settings.contains(ui->AddNoise->objectName())) ui->AddNoise->setChecked(settings.value(ui->AddNoise->objectName()).toBool());
        if(settings.contains(ui->NoiseAmp->objectName())) ui->NoiseAmp->setText(settings.value(ui->NoiseAmp->objectName(),QString()).toString());
        if(settings.contains(ui->runTime->objectName())) ui->runTime->setText(settings.value(ui->runTime->objectName(),QString()).toString());
        if(settings.contains(ui->startRpm->objectName())) ui->startRpm->setText(settings.value(ui->startRpm->objectName(),QString()).toString());
        if(settings.contains(ui->modBlend->objectName())) ui->modBlend->setText(settings.value(ui->modBlend->objectName(),QString()).toString());
        if(settings.contains(ui->deadtimeUs->objectName())) ui->deadtimeUs->setText(settings.value(ui->deadtimeUs->objectName(),QString()).toString());
        if(settings.contains(ui->sinkTemp->objectName())) ui->sinkTemp->setText(settings.value(ui->sinkTemp->objectName(),QString()).toString());
        if(settings.contains(ui->thermalTau->objectName())) ui->thermalTau->setText(settings.value(ui->thermalTau->objectName(),QString()).toString());
        if(settings.contains(ui->vrefV->objectName())) ui->vrefV->setText(settings.value(ui->vrefV->objectName(),QString()).toString());
        if(settings.contains(ui->kvExp->objectName())) ui->kvExp->setText(settings.value(ui->kvExp->objectName(),QString()).toString());
        if(settings.contains(ui->diodeVf25->objectName())) ui->diodeVf25->setText(settings.value(ui->diodeVf25->objectName(),QString()).toString());
        if(settings.contains(ui->diodeVf125->objectName())) ui->diodeVf125->setText(settings.value(ui->diodeVf125->objectName(),QString()).toString());
        if(settings.contains(ui->rthJcIgbt->objectName())) ui->rthJcIgbt->setText(settings.value(ui->rthJcIgbt->objectName(),QString()).toString());
        if(settings.contains(ui->rthJcDiode->objectName())) ui->rthJcDiode->setText(settings.value(ui->rthJcDiode->objectName(),QString()).toString());
        if(settings.contains(ui->rthCs->objectName())) ui->rthCs->setText(settings.value(ui->rthCs->objectName(),QString()).toString());
        if(settings.contains(ui->vcePoints->objectName())) ui->vcePoints->setPlainText(settings.value(ui->vcePoints->objectName(),QString()).toString());
        if(settings.contains(ui->eonPoints->objectName())) ui->eonPoints->setPlainText(settings.value(ui->eonPoints->objectName(),QString()).toString());
        if(settings.contains(ui->eoffPoints->objectName())) ui->eoffPoints->setPlainText(settings.value(ui->eoffPoints->objectName(),QString()).toString());
        if(settings.contains(ui->irrPoints->objectName())) ui->irrPoints->setPlainText(settings.value(ui->irrPoints->objectName(),QString()).toString());
        if(settings.contains(ui->trrPoints->objectName())) ui->trrPoints->setPlainText(settings.value(ui->trrPoints->objectName(),QString()).toString());
        if(settings.contains(ui->RoadGradient->objectName())) ui->RoadGradient->setText(settings.value(ui->RoadGradient->objectName(),QString()).toString());
        if(settings.contains(ui->ThrotRamps->objectName())) ui->ThrotRamps->setChecked(settings.value(ui->ThrotRamps->objectName()).toBool());
        if(settings.contains(ui->cb_Efficiency->objectName())) ui->cb_Efficiency->setChecked(settings.value(ui->cb_Efficiency->objectName()).toBool());
        if(settings.contains(ui->cb_LogCsv->objectName())) ui->cb_LogCsv->setChecked(settings.value(ui->cb_LogCsv->objectName()).toBool());
        if(settings.contains(ui->cb_PwmZeroSeq->objectName())) ui->cb_PwmZeroSeq->setChecked(settings.value(ui->cb_PwmZeroSeq->objectName()).toBool());
        if(settings.contains(ui->cb_PwmClamp->objectName())) ui->cb_PwmClamp->setChecked(settings.value(ui->cb_PwmClamp->objectName()).toBool());
        if(settings.contains(ui->cb_PwmTiming->objectName())) ui->cb_PwmTiming->setChecked(settings.value(ui->cb_PwmTiming->objectName()).toBool());
        if(settings.contains(ui->cb_PwmSector->objectName())) ui->cb_PwmSector->setChecked(settings.value(ui->cb_PwmSector->objectName()).toBool());
        if(settings.contains(ui->cb_ShowLegends->objectName())) ui->cb_ShowLegends->setChecked(settings.value(ui->cb_ShowLegends->objectName()).toBool());
        if(settings.contains(ui->cb_Losses->objectName())) ui->cb_Losses->setChecked(settings.value(ui->cb_Losses->objectName()).toBool());
        if(settings.contains(ui->modulationMode->objectName()))
            ui->modulationMode->setCurrentIndex(settings.value(ui->modulationMode->objectName()).toInt());
    }

    ui->startRpm->setValidator(new QIntValidator(-20000, 20000, ui->startRpm));
    ui->deadtimeUs->setValidator(new QDoubleValidator(0.0, 50.0, 3, ui->deadtimeUs));
    ui->sinkTemp->setValidator(new QDoubleValidator(-40.0, 200.0, 2, ui->sinkTemp));
    ui->thermalTau->setValidator(new QDoubleValidator(0.01, 100.0, 3, ui->thermalTau));
    ui->vrefV->setValidator(new QDoubleValidator(1.0, 1200.0, 1, ui->vrefV));
    ui->kvExp->setValidator(new QDoubleValidator(0.0, 3.0, 2, ui->kvExp));
    ui->diodeVf25->setValidator(new QDoubleValidator(0.0, 10.0, 3, ui->diodeVf25));
    ui->diodeVf125->setValidator(new QDoubleValidator(0.0, 10.0, 3, ui->diodeVf125));
    ui->rthJcIgbt->setValidator(new QDoubleValidator(0.0, 1.0, 3, ui->rthJcIgbt));
    ui->rthJcDiode->setValidator(new QDoubleValidator(0.0, 1.0, 3, ui->rthJcDiode));
    ui->rthCs->setValidator(new QDoubleValidator(0.0, 1.0, 3, ui->rthCs));
    QDoubleValidator *blendValidator = new QDoubleValidator(0.0, 1.0, 3, ui->modBlend);
    blendValidator->setNotation(QDoubleValidator::StandardNotation);
    ui->modBlend->setValidator(blendValidator);

    ui->modBlendSlider->setRange(0, 100);
    ui->modBlendSlider->setValue(static_cast<int>(ui->modBlend->text().toDouble() * 100.0));
    ui->modBlendValue->setText(QString::number(ui->modBlend->text().toDouble(), 'f', 3));

    qApp->installEventFilter(this);

    auto tip = [](QWidget* w, const QString& text)
    {
        if(!w) return;
        w->setToolTip(text);
        w->setWhatsThis(text);
        w->setStatusTip(text);
        w->setToolTipDuration(10000);
    };

    tip(ui->vehicleWeight, "Vehicle mass in kg used in the simple load model.");
    tip(ui->wheelSize, "Wheel radius in meters. Used to convert torque to force and speed.");
    tip(ui->gearRatio, "Overall gear ratio from motor to wheel.");
    tip(ui->Vdc, "DC bus voltage in volts.");
    tip(ui->Lq, "Quadrature-axis inductance in mH.");
    tip(ui->Ld, "Direct-axis inductance in mH.");
    tip(ui->Rs, "Stator phase resistance in ohms.");
    tip(ui->FluxLinkage, "Flux linkage in mWeber (psi).");
    tip(ui->SyncDelay, "Electrical sync delay in microseconds (sampling/angle delay).");
    tip(ui->LoopFreq, "Control loop frequency in Hz.");
    tip(ui->SamplingPoint, "Current sampling point within the PWM period (percent).");
    tip(ui->NoiseAmp, "Injected current measurement noise amplitude (A).");
    tip(ui->RoadGradient, "Road gradient percent. Positive is uphill.");
    tip(ui->ThrotRamps, "Enable throttle ramping to simulate rate limits.");
    tip(ui->ExtraCycleDelay, "Adds one PWM cycle delay to voltages.");
    tip(ui->AddNoise, "Enable noise on current feedback inputs.");
    tip(ui->runTime, "Duration for Run For (s).");
    tip(ui->startRpm, "Initial mechanical speed (RPM), applied on Restart.");
    tip(ui->deadtimeUs, "PWM deadtime in microseconds applied in the inverter model.");
    tip(ui->sinkTemp, "Heatsink/case reference temperature (C) for thermal model.");
    tip(ui->thermalTau, "Thermal time constant (s) for first-order junction tracking.");
    tip(ui->vrefV, "Reference Vdc used for switching-energy curves.");
    tip(ui->kvExp, "Voltage scaling exponent: E = Eref * (Vdc/Vref)^kV.");
    tip(ui->diodeVf25, "Diode forward drop at 25C (V).");
    tip(ui->diodeVf125, "Diode forward drop at 125C (V).");
    tip(ui->rthJcIgbt, "IGBT junction-to-case thermal resistance per switch (C/W).");
    tip(ui->rthJcDiode, "Diode junction-to-case thermal resistance per switch (C/W).");
    tip(ui->rthCs, "Case-to-sink thermal resistance for the module (C/W).");
    tip(ui->vcePoints, "IGBT Vce(sat) curve points: I, V25C, V125C (one per line).");
    tip(ui->eonPoints, "IGBT Eon curve points: I, 25C, 125C (mJ) per line.");
    tip(ui->eoffPoints, "IGBT Eoff curve points: I, 25C, 125C (mJ) per line.");
    tip(ui->irrPoints, "Diode reverse recovery current points: I, 25C, 125C (A) per line.");
    tip(ui->trrPoints, "Diode reverse recovery time points: I, 25C, 125C (us) per line.");
    tip(ui->powerStagePreset, "Select a preset from powerstages.yaml to load power-stage parameters.");
    tip(ui->torqueDemand, "Torque demand in percent.");
    tip(ui->throttleCurrent, "Current per percent throttle (A/%).");
    tip(ui->opMode, "Controller mode: 1=Run, 2=Manual.");
    tip(ui->direction, "Direction: -1 reverse, 0 neutral, 1 forward.");
    tip(ui->IqManual, "Manual q-axis current command (A).");
    tip(ui->IdManual, "Manual d-axis current command (A).");
    tip(ui->Poles, "Motor pole pairs.");
    tip(ui->CurrentKp, "Current controller proportional gain.");
    tip(ui->CurrentKi, "Current controller integral gain.");
    tip(ui->VLimMargin, "Voltage limit margin (firmware units).");
    tip(ui->VLimFlt, "Voltage limit filter (firmware units).");
    tip(ui->LqMinusLd, "Lq - Ld in mH (for MTPA).");
    tip(ui->SyncAdv, "Electrical angle advance (dig/Hz).");
    tip(ui->SyncOfs, "Electrical angle offset (dig).");
    tip(ui->FWCurrMax, "Field weakening current limit (A, typically negative).");
    tip(ui->FreqMax, "Maximum electrical frequency (Hz).");

    tip(ui->modulationMode, "Select modulation. Firmware uses stm32-sine PWM. Others use simulator modulator.");
    tip(ui->modBlend, "Blend between SVPWM (0) and clamped DPWM (1).");
    tip(ui->modBlendSlider, "Blend between SVPWM (0) and clamped DPWM (1).");
    tip(ui->modBlendValue, "Current blend value (read-only).");

    tip(ui->cb_OpPoint, "Show operating point (Id/Iq) window.");
    tip(ui->cb_Simulation, "Show simulation window.");
    tip(ui->cb_ContVolt, "Show controller voltage window.");
    tip(ui->cb_ContCurr, "Show controller current window.");
    tip(ui->cb_MotVolt, "Show motor voltage window.");
    tip(ui->cb_MotCurr, "Show motor current window.");
    tip(ui->cb_PowTorqTime, "Show power/torque window.");
    tip(ui->cb_Pwm, "Show PWM modulation window.");
    tip(ui->cb_PwmZeroSeq, "Show zero-sequence component of modulation.");
    tip(ui->cb_PwmClamp, "Show clamped leg indicator (-1 low, +1 high).");
    tip(ui->cb_PwmTiming, "Show T1/T2/T0 timing components.");
    tip(ui->cb_PwmSector, "Show SVPWM sector (1-6).");
    tip(ui->cb_ShowLegends, "Toggle graph legends on/off.");
    tip(ui->cb_Losses, "Show inverter loss breakdown window.");
    tip(ui->cb_MotorPos, "Include motor position in simulation graph.");
    tip(ui->cb_PhaseVolts, "Include phase voltages in controller volt graph.");
    tip(ui->cb_PhaseCurrs, "Include phase currents in motor current graph.");
    tip(ui->cb_Efficiency, "Include efficiency in power graph.");
    tip(ui->cb_LogCsv, "Write CSV log for each run.");

    const auto formLayouts = findChildren<QFormLayout*>();
    for(QFormLayout* form : formLayouts)
    {
        if(!form) continue;
        for(int row = 0; row < form->rowCount(); ++row)
        {
            QLayoutItem* labelItem = form->itemAt(row, QFormLayout::LabelRole);
            QLayoutItem* fieldItem = form->itemAt(row, QFormLayout::FieldRole);
            QWidget* labelWidget = labelItem ? labelItem->widget() : nullptr;
            QWidget* fieldWidget = fieldItem ? fieldItem->widget() : nullptr;
            if(!labelWidget || !fieldWidget) continue;
            if(!labelWidget->toolTip().isEmpty()) continue;
            if(fieldWidget->toolTip().isEmpty()) continue;
            labelWidget->setToolTip(fieldWidget->toolTip());
            labelWidget->setWhatsThis(fieldWidget->toolTip());
            labelWidget->setStatusTip(fieldWidget->toolTip());
            labelWidget->setToolTipDuration(10000);
        }
    }

    const auto labels = findChildren<QLabel*>();
    for(QLabel* label : labels)
    {
        if(!label || !label->toolTip().isEmpty()) continue;
        if(QWidget* buddy = label->buddy())
        {
            if(!buddy->toolTip().isEmpty())
            {
                label->setToolTip(buddy->toolTip());
                label->setWhatsThis(buddy->toolTip());
                label->setStatusTip(buddy->toolTip());
                label->setToolTipDuration(10000);
            }
        }
    }

    loadPowerStagePresets();

    motorGraph = new DataGraph("motor", this);
    simulationGraph = new DataGraph("sim", this);
    controllerGraph = new DataGraph("cont", this);
    debugGraph = new DataGraph("debug", this);
    voltageGraph = new DataGraph("voltage", this);
    pwmGraph = new DataGraph("pwm", this);
    idigGraph = new IdIqGraph("idig", this);
    powerGraph = new DataGraph("power", this);
    lossGraph = new DataGraph("loss", this);

    motorGraph->hide();//not sure why needed but otherwise always up?

    ANA_IN_CONFIGURE(ANA_IN_LIST);

    //set any parameters that can upset simulation to safe values
    Param::SetInt(Param::syncofs,0); //simulator assumes perfect alignment
    Param::SetInt(Param::pinswap,0); //shouldn't be a problem but may be in the future
    Param::SetInt(Param::respolepairs,Param::GetInt(Param::polepairs)); //force resolver pole pairs to match motor

    // Only populate "openinverter params" UI fields from Param defaults when there's no saved UI setting.
    // Otherwise, we'd overwrite persisted settings and run the sim with unexpected defaults.
    if(!settings.contains(ui->LqMinusLd->objectName())) ui->LqMinusLd->setText(QString::number(Param::GetFloat(Param::lqminusld), 'f', 1));
    if(!settings.contains(ui->FluxLinkage->objectName())) ui->FluxLinkage->setText(QString::number(Param::GetInt(Param::fluxlinkage)));
    if(!settings.contains(ui->SyncAdv->objectName())) ui->SyncAdv->setText(QString::number(Param::GetInt(Param::syncadv)));
    if(!settings.contains(ui->FreqMax->objectName())) ui->FreqMax->setText(QString::number(Param::GetFloat(Param::fmax), 'f', 1));
    if(!settings.contains(ui->Poles->objectName())) ui->Poles->setText(QString::number(Param::GetFloat(Param::polepairs), 'f', 1));
    if(!settings.contains(ui->CurrentKp->objectName())) ui->CurrentKp->setText(QString::number(Param::GetInt(Param::iqkp)));
    if(!settings.contains(ui->CurrentKi->objectName())) ui->CurrentKi->setText(QString::number(Param::GetInt(Param::curki)));
    if(!settings.contains(ui->VLimMargin->objectName())) ui->VLimMargin->setText(QString::number(Param::GetInt(Param::vlimmargin)));
    if(!settings.contains(ui->VLimFlt->objectName())) ui->VLimFlt->setText(QString::number(Param::GetInt(Param::vlimflt)));
    if(!settings.contains(ui->FWCurrMax->objectName())) ui->FWCurrMax->setText(QString::number(Param::GetInt(Param::fwcurmax)));
    if(!settings.contains(ui->IdManual->objectName())) ui->IdManual->setText(QString::number(Param::GetFloat(Param::manualid), 'f', 1));
    if(!settings.contains(ui->IqManual->objectName())) ui->IqManual->setText(QString::number(Param::GetFloat(Param::manualiq), 'f', 1));
    if(!settings.contains(ui->throttleCurrent->objectName())) ui->throttleCurrent->setText(QString::number(Param::GetFloat(Param::throtcur), 'f', 1));

    m_wheelSize = ui->wheelSize->text().toDouble();
    m_vehicleWeight = ui->vehicleWeight->text().toDouble();
    m_gearRatio = ui->gearRatio->text().toDouble();
    m_Lq = ui->Lq->text().toDouble()/1000; //entered in mH
    m_Ld = ui->Ld->text().toDouble()/1000; //entered in mH
    m_Rs = ui->Rs->text().toDouble();
    m_Poles = ui->Poles->text().toDouble();
    m_fluxLinkage = ui->FluxLinkage->text().toDouble()/1000; //entered in mWeber
    m_syncdelay = ui->SyncDelay->text().toDouble()/1000000; //entered in uS
    m_samplingPoint = ui->SamplingPoint->text().toDouble()/100.0; //entered in %
    m_roadGradient = ui->RoadGradient->text().toDouble()/100.0; //entered in %
    m_runTime = ui->runTime->text().toDouble();

    m_timestep = 1.0 / ui->LoopFreq->text().toDouble();
    m_Vdc = ui->Vdc->text().toDouble();
    Param::SetFloat(Param::udc, m_Vdc);

    motor = new sim::MotorPlant(m_wheelSize,m_gearRatio,m_roadGradient,m_vehicleWeight,m_Lq,m_Ld,m_Rs,m_Poles,m_fluxLinkage,m_timestep,m_syncdelay,m_samplingPoint);

    // Ensure Param state reflects the UI (including restored settings) even if the user hasn't focused/edited fields.
    // Without this, persisted UI values like throttle current can be displayed but not actually applied to the controller.
    on_Poles_editingFinished();
    on_FluxLinkage_editingFinished();
    on_LqMinusLd_editingFinished();
    on_SyncAdv_editingFinished();
    on_FreqMax_editingFinished();
    on_CurrentKp_editingFinished();
    on_CurrentKi_editingFinished();
    on_VLimMargin_editingFinished();
    on_VLimFlt_editingFinished();
    on_FWCurrMax_editingFinished();
    on_IdManual_editingFinished();
    on_IqManual_editingFinished();
    on_direction_editingFinished();
    on_opMode_editingFinished();
    on_throttleCurrent_editingFinished();
    on_torqueDemand_editingFinished();

    m_time = 0;
    m_old_time = 0;
    m_old_ms_time = 0;

    m_oldVa = 0;
    m_oldVb = 0;
    m_oldVc = 0;

    m_lastTorqueDemand = 0;

    motorGraph->setWindowTitle("Motor Currents");
    motorGraph->setAxisText("", "Amps (A)", "");
    motorGraph->addSeries("Ia (A)", left, IA);
    motorGraph->addSeries("Ib (A)", left, IB);
    motorGraph->addSeries("Ic (A)", left, IC);
    motorGraph->addSeries("Iq (A)", left, IQ);
    motorGraph->setColour(Qt::blue, IQ);
    motorGraph->addSeries("Id (A)", left, ID);
    motorGraph->setColour(Qt::red, ID);
    if(settings.contains(ui->cb_MotCurr->objectName())) ui->cb_MotCurr->setChecked(settings.value(ui->cb_MotCurr->objectName()).toBool());
    if(settings.contains(ui->cb_PhaseCurrs->objectName())) ui->cb_PhaseCurrs->setChecked(settings.value(ui->cb_PhaseCurrs->objectName()).toBool());

    simulationGraph->setWindowTitle("Simulation Data");
    simulationGraph->setAxisText("", "Angle (Degrees)", "Speed (Hz)");
    simulationGraph->addSeries("Motor Position (degrees)", left, M_MOTOR_POS);
    simulationGraph->setOpacity(0.25, M_MOTOR_POS);
    simulationGraph->addSeries("Controller Position (degrees)", left, M_CONT_POS);
    simulationGraph->setOpacity(0.25, M_CONT_POS);
    simulationGraph->addSeries("Motor Elec Speed (Hz)", right, M_RPM);
    simulationGraph->setColour(Qt::blue, M_RPM);
    if(settings.contains(ui->cb_Simulation->objectName())) ui->cb_Simulation->setChecked(settings.value(ui->cb_Simulation->objectName()).toBool());
    if(settings.contains(ui->cb_MotorPos->objectName())) ui->cb_MotorPos->setChecked(settings.value(ui->cb_MotorPos->objectName()).toBool());

    controllerGraph->setWindowTitle("Controller Voltages");
    controllerGraph->setAxisText("", "Volts (V)", "");
    controllerGraph->addSeries("Va (V)", left, VA);
    controllerGraph->addSeries("Vb (V)", left, VB);
    controllerGraph->addSeries("Vc (V)", left, VC);
    controllerGraph->addSeries("Vq (V)", left, VQ);
    controllerGraph->setColour(Qt::blue, VQ);
    controllerGraph->addSeries("Vd (V)", left, VD);
    controllerGraph->setColour(Qt::red, VD);
    if(settings.contains(ui->cb_ContVolt->objectName())) ui->cb_ContVolt->setChecked(settings.value(ui->cb_ContVolt->objectName()).toBool());
    if(settings.contains(ui->cb_PhaseVolts->objectName())) ui->cb_PhaseVolts->setChecked(settings.value(ui->cb_PhaseVolts->objectName()).toBool());

    debugGraph->setWindowTitle("Controller Currents");
    debugGraph->setAxisText("", "Amps (A)", "");
    debugGraph->addSeries("Iq (A)", left, C_IQ);
    debugGraph->setColour(Qt::blue, C_IQ);
    debugGraph->addSeries("Id (A)", left, C_ID);
    debugGraph->setColour(Qt::red, C_ID);
    debugGraph->addSeries("Ifw (A)", left, C_IFW);
    debugGraph->addSeries("Throttle Reduction (%)", right, C_IVLIM);
    if(settings.contains(ui->cb_ContCurr->objectName())) ui->cb_ContCurr->setChecked(settings.value(ui->cb_ContCurr->objectName()).toBool());

    voltageGraph->setWindowTitle("Motor Voltages");
    voltageGraph->setAxisText("", "Volts (V)", "");
    voltageGraph->addSeries("Vd (V)", left, VVD);
    voltageGraph->setColour(Qt::red, VVD);
    voltageGraph->addSeries("Vq (V)", left, VVQ);
    voltageGraph->setColour(Qt::blue, VVQ);
    voltageGraph->addSeries("Vq_BEMF (V)", left, VVQ_BEMF);
    voltageGraph->addSeries("Vq_LdId (V)", left, VVQ_DT_ID);
    voltageGraph->addSeries("Vd_LqIq (V)", left, VVD_DT_IQ);
    voltageGraph->addSeries("Vq_RqIq (V)", left, VVQ_DT_RQ);
    voltageGraph->addSeries("Vd_RdIq (V)", left, VVD_DT_RD);
    voltageGraph->addSeries("VLd (V)", left, VVLD);
    voltageGraph->addSeries("VLq (V)", left, VVLQ);
    if(settings.contains(ui->cb_MotVolt->objectName())) ui->cb_MotVolt->setChecked(settings.value(ui->cb_MotVolt->objectName()).toBool());

    pwmGraph->setWindowTitle("PWM Modulation");
    pwmGraph->setAxisText("Time (s)", "Duty", "Diag");
    pwmGraph->addSeries("Duty A", left, PWM_A);
    pwmGraph->setColour(QColor(0xE6, 0x9F, 0x00), PWM_A);
    pwmGraph->addSeries("Duty B", left, PWM_B);
    pwmGraph->setColour(QColor(0x56, 0xB4, 0xE9), PWM_B);
    pwmGraph->addSeries("Duty C", left, PWM_C);
    pwmGraph->setColour(QColor(0x00, 0x9E, 0x73), PWM_C);
    pwmGraph->addSeries("Duty Min", left, PWM_MIN);
    pwmGraph->setColour(QColor(0x99, 0x99, 0x99), PWM_MIN);
    pwmGraph->setOpacity(0.6, PWM_MIN);
    pwmGraph->addSeries("Duty Max", left, PWM_MAX);
    pwmGraph->setColour(Qt::black, PWM_MAX);
    pwmGraph->setOpacity(0.6, PWM_MAX);
    pwmGraph->addSeries("Zero Seq", right, PWM_ZEROSEQ);
    pwmGraph->setColour(QColor(0xCC, 0x79, 0xA7), PWM_ZEROSEQ);
    pwmGraph->setOpacity(0.6, PWM_ZEROSEQ);
    pwmGraph->addSeries("Clamp A", right, PWM_CLAMP_A);
    pwmGraph->setColour(QColor(0xD5, 0x5E, 0x00), PWM_CLAMP_A);
    pwmGraph->setOpacity(0.6, PWM_CLAMP_A);
    pwmGraph->addSeries("Clamp B", right, PWM_CLAMP_B);
    pwmGraph->setColour(QColor(0x00, 0x72, 0xB2), PWM_CLAMP_B);
    pwmGraph->setOpacity(0.6, PWM_CLAMP_B);
    pwmGraph->addSeries("Clamp C", right, PWM_CLAMP_C);
    pwmGraph->setColour(QColor(0xF0, 0xE4, 0x42), PWM_CLAMP_C);
    pwmGraph->setOpacity(0.6, PWM_CLAMP_C);
    pwmGraph->addSeries("T1", left, PWM_T1);
    pwmGraph->setColour(QColor(0x8E, 0x44, 0xAD), PWM_T1);
    pwmGraph->setOpacity(0.6, PWM_T1);
    pwmGraph->addSeries("T2", left, PWM_T2);
    pwmGraph->setColour(QColor(0x16, 0xA0, 0x85), PWM_T2);
    pwmGraph->setOpacity(0.6, PWM_T2);
    pwmGraph->addSeries("T0", left, PWM_T0);
    pwmGraph->setColour(QColor(0xC0, 0x39, 0x2B), PWM_T0);
    pwmGraph->setOpacity(0.6, PWM_T0);
    pwmGraph->addSeries("Sector", right, PWM_SECTOR);
    pwmGraph->setColour(QColor(0x7F, 0x7F, 0x7F), PWM_SECTOR);
    pwmGraph->setOpacity(0.6, PWM_SECTOR);
    if(settings.contains(ui->cb_Pwm->objectName())) ui->cb_Pwm->setChecked(settings.value(ui->cb_Pwm->objectName()).toBool());
    on_cb_PwmZeroSeq_toggled(ui->cb_PwmZeroSeq->isChecked());
    on_cb_PwmClamp_toggled(ui->cb_PwmClamp->isChecked());
    on_cb_PwmTiming_toggled(ui->cb_PwmTiming->isChecked());
    on_cb_PwmSector_toggled(ui->cb_PwmSector->isChecked());
    on_cb_ShowLegends_toggled(ui->cb_ShowLegends->isChecked());

    idigGraph->setWindowTitle("Operating Point");
    idigGraph->setAxisText("Id (A)", "Iq (A)", "");
    idigGraph->addSeries("I (A)", left, IDIQAMPS);
    if(settings.contains(ui->cb_OpPoint->objectName())) ui->cb_OpPoint->setChecked(settings.value(ui->cb_OpPoint->objectName()).toBool());    
    if(settings.contains(ui->rb_OP_Amps->objectName()))
    {
        if(settings.value(ui->rb_OP_Amps->objectName()).toBool())
        {
            ui->rb_OP_Amps->setChecked(true);
            idigGraph->setAxisText("Id (A)", "Iq (A)", "");
        }
        else
        {
            ui->rb_OP_Volts->setChecked(true);
            idigGraph->setAxisText("Vd (V)", "Vq (V)", "");
            idigGraph->updateSeries("V (V)", left, IDIQAMPS);
        }
    }

    powerGraph->setWindowTitle("Power/Torque");
    powerGraph->setAxisText("Time (s)", "Power (kW)", "Torque (Nm)");
    powerGraph->addSeries("Power (kW)", left, POWER);
    powerGraph->addSeries("Torque (Nm)", right, TORQUE);
    powerGraph->addSeries("Elec Power (kW)", left, ELEC_POWER);
    powerGraph->addSeries("Efficiency (%)", left, EFFICIENCY);

    lossGraph->setWindowTitle("Inverter Losses");
    lossGraph->setAxisText("Time (s)", "Loss (kW)", "");
    lossGraph->addSeries("IGBT Cond (kW)", left, LOSS_IGBT_COND);
    lossGraph->setColour(QColor(0x56, 0xB4, 0xE9), LOSS_IGBT_COND);
    lossGraph->addSeries("Diode Cond (kW)", left, LOSS_DIODE_COND);
    lossGraph->setColour(QColor(0xE6, 0x9F, 0x00), LOSS_DIODE_COND);
    lossGraph->addSeries("IGBT Sw (kW)", left, LOSS_IGBT_SW);
    lossGraph->setColour(QColor(0xD5, 0x5E, 0x00), LOSS_IGBT_SW);
    lossGraph->addSeries("Diode RR (kW)", left, LOSS_DIODE_RR);
    lossGraph->setColour(QColor(0xCC, 0x79, 0xA7), LOSS_DIODE_RR);
    lossGraph->addSeries("Total Loss (kW)", left, LOSS_TOTAL);
    lossGraph->setColour(QColor(0x00, 0x00, 0x00), LOSS_TOTAL);
    if(settings.contains(ui->cb_PowTorqTime->objectName())) ui->cb_PowTorqTime->setChecked(settings.value(ui->cb_PowTorqTime->objectName()).toBool());
    if(settings.contains(ui->cb_Losses->objectName())) ui->cb_Losses->setChecked(settings.value(ui->cb_Losses->objectName()).toBool());
    on_cb_Losses_toggled(ui->cb_Losses->isChecked());
    if(settings.contains(ui->rb_Speed->objectName()))
    {
        if(settings.value(ui->rb_Speed->objectName()).toBool())
        {
            ui->rb_Speed->setChecked(true);
            powerGraph->setAxisText("Shaft Speed (rpm)", "Power (kW)", "Torque (Nm)");
        }
        else
        {
            ui->rb_Time->setChecked(true);
            powerGraph->setAxisText("Time (s)", "Power (kW)", "Torque (Nm)");
        }
    }

    //following block copied from OpenInverter - probably not needed
    Param::SetInt(Param::version, 4); //backward compatibility

    if (Param::GetInt(Param::snsm) < 12)
        Param::SetInt(Param::snsm, Param::GetInt(Param::snsm) + 10); //upgrade parameter
    if (Param::Get(Param::offthrotregen) > 0)
        Param::Set(Param::offthrotregen, -Param::Get(Param::offthrotregen));


    Param::Change(Param::PARAM_LAST);
    Param::Change(Param::nodeid);

    PwmGeneration::SetOpmode(0);
    PwmGeneration::SetOpmode(ui->opMode->text().toInt());
    Param::SetInt(Param::seldir, ui->direction->text().toInt());

    ui->Poles->setText(QString::number(Param::GetInt(Param::polepairs)));
    ui->throttleCurrent->setText(QString::number(Param::GetFloat(Param::throtcur), 'f', 1));

    FOC::SetMotorParameters(Param::GetFloat(Param::lqminusld)/1000, Param::GetFloat(Param::fluxlinkage)/1000);

    PwmGeneration::SetTorquePercent(ui->torqueDemand->text().toFloat());

    //run for 1sec to complete motor init
    runFor(8789);
    on_pbRestart_clicked();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    QSettings settings("OpenInverter", "IPMMotorSim");
    settings.setValue("mainwin/geometry", saveGeometry());
    settings.setValue("mainwin/windowState", saveState());

    settings.setValue(ui->vehicleWeight->objectName(), ui->vehicleWeight->text());
    settings.setValue(ui->wheelSize->objectName(), ui->wheelSize->text());
    settings.setValue(ui->gearRatio->objectName(), ui->gearRatio->text());
    settings.setValue(ui->Vdc->objectName(), ui->Vdc->text());
    settings.setValue(ui->Lq->objectName(), ui->Lq->text());
    settings.setValue(ui->Ld->objectName(), ui->Ld->text());
    settings.setValue(ui->Rs->objectName(), ui->Rs->text());
    settings.setValue(ui->SyncDelay->objectName(), ui->SyncDelay->text());
    settings.setValue(ui->LoopFreq->objectName(), ui->LoopFreq->text());
    settings.setValue(ui->SamplingPoint->objectName(), ui->SamplingPoint->text());
    settings.setValue(ui->ExtraCycleDelay->objectName(), ui->ExtraCycleDelay->isChecked());
    settings.setValue(ui->AddNoise->objectName(), ui->AddNoise->isChecked());
    settings.setValue(ui->NoiseAmp->objectName(), ui->NoiseAmp->text());
    settings.setValue(ui->runTime->objectName(), ui->runTime->text());
    settings.setValue(ui->startRpm->objectName(), ui->startRpm->text());
    settings.setValue(ui->modBlend->objectName(), ui->modBlend->text());
    settings.setValue(ui->deadtimeUs->objectName(), ui->deadtimeUs->text());
    settings.setValue(ui->sinkTemp->objectName(), ui->sinkTemp->text());
    settings.setValue(ui->thermalTau->objectName(), ui->thermalTau->text());
    settings.setValue(ui->vrefV->objectName(), ui->vrefV->text());
    settings.setValue(ui->kvExp->objectName(), ui->kvExp->text());
    settings.setValue(ui->diodeVf25->objectName(), ui->diodeVf25->text());
    settings.setValue(ui->diodeVf125->objectName(), ui->diodeVf125->text());
    settings.setValue(ui->rthJcIgbt->objectName(), ui->rthJcIgbt->text());
    settings.setValue(ui->rthJcDiode->objectName(), ui->rthJcDiode->text());
    settings.setValue(ui->rthCs->objectName(), ui->rthCs->text());
    settings.setValue(ui->vcePoints->objectName(), ui->vcePoints->toPlainText());
    settings.setValue(ui->eonPoints->objectName(), ui->eonPoints->toPlainText());
    settings.setValue(ui->eoffPoints->objectName(), ui->eoffPoints->toPlainText());
    settings.setValue(ui->irrPoints->objectName(), ui->irrPoints->toPlainText());
    settings.setValue(ui->trrPoints->objectName(), ui->trrPoints->toPlainText());
    settings.setValue(ui->ThrotRamps->objectName(), ui->ThrotRamps->isChecked());
    settings.setValue(ui->RoadGradient->objectName(), ui->RoadGradient->text());
    settings.setValue(ui->modulationMode->objectName(), ui->modulationMode->currentIndex());
    settings.setValue("powerStagePreset", currentPowerStagePresetKey());

    settings.setValue(ui->cb_ContCurr->objectName(), ui->cb_ContCurr->isChecked());
    settings.setValue(ui->cb_ContVolt->objectName(), ui->cb_ContVolt->isChecked());
    settings.setValue(ui->cb_MotCurr->objectName(), ui->cb_MotCurr->isChecked());
    settings.setValue(ui->cb_MotVolt->objectName(), ui->cb_MotVolt->isChecked());
    settings.setValue(ui->cb_Pwm->objectName(), ui->cb_Pwm->isChecked());
    settings.setValue(ui->cb_PwmZeroSeq->objectName(), ui->cb_PwmZeroSeq->isChecked());
    settings.setValue(ui->cb_PwmClamp->objectName(), ui->cb_PwmClamp->isChecked());
    settings.setValue(ui->cb_PwmTiming->objectName(), ui->cb_PwmTiming->isChecked());
    settings.setValue(ui->cb_PwmSector->objectName(), ui->cb_PwmSector->isChecked());
    settings.setValue(ui->cb_ShowLegends->objectName(), ui->cb_ShowLegends->isChecked());
    settings.setValue(ui->cb_Losses->objectName(), ui->cb_Losses->isChecked());
    settings.setValue(ui->cb_OpPoint->objectName(), ui->cb_OpPoint->isChecked());
    settings.setValue(ui->cb_PowTorqTime->objectName(), ui->cb_PowTorqTime->isChecked());
    settings.setValue(ui->cb_Simulation->objectName(), ui->cb_Simulation->isChecked());
    settings.setValue(ui->rb_Speed->objectName(), ui->rb_Speed->isChecked());
    settings.setValue(ui->cb_Efficiency->objectName(), ui->cb_Efficiency->isChecked());

    settings.setValue(ui->cb_PhaseCurrs->objectName(), ui->cb_PhaseCurrs->isChecked());
    settings.setValue(ui->cb_MotorPos->objectName(), ui->cb_MotorPos->isChecked());
    settings.setValue(ui->cb_PhaseVolts->objectName(), ui->cb_PhaseVolts->isChecked());
    settings.setValue(ui->rb_OP_Amps->objectName(), ui->rb_OP_Amps->isChecked());
    settings.setValue(ui->cb_LogCsv->objectName(), ui->cb_LogCsv->isChecked());

    motorGraph->saveWinState();
    simulationGraph->saveWinState();
    controllerGraph->saveWinState();
    debugGraph->saveWinState();
    voltageGraph->saveWinState();
    idigGraph->saveWinState();
    powerGraph->saveWinState();
    lossGraph->saveWinState();
    QWidget::closeEvent(event);
}

static QString helpTextFor(QObject* obj)
{
    if(auto action = qobject_cast<QAction*>(obj))
    {
        if(!action->toolTip().isEmpty()) return action->toolTip();
        if(!action->statusTip().isEmpty()) return action->statusTip();
    }

    if(auto widget = qobject_cast<QWidget*>(obj))
    {
        if(!widget->toolTip().isEmpty()) return widget->toolTip();
        if(!widget->statusTip().isEmpty()) return widget->statusTip();
    }

    if(QWidget* widget = QApplication::widgetAt(QCursor::pos()))
    {
        if(!widget->toolTip().isEmpty()) return widget->toolTip();
        if(!widget->statusTip().isEmpty()) return widget->statusTip();
    }

    return QString();
}

bool MainWindow::eventFilter(QObject* obj, QEvent* event)
{
    const QString helpText = helpTextFor(obj);
    if(helpText.isEmpty())
        return QMainWindow::eventFilter(obj, event);

    switch(event->type())
    {
        case QEvent::ToolTip:
        {
            const QHelpEvent* helpEvent = static_cast<QHelpEvent*>(event);
            const QPoint globalPos = helpEvent ? helpEvent->globalPos() : QCursor::pos();
            QToolTip::showText(globalPos, helpText, nullptr);
            statusBar()->showMessage(helpText);
            break;
        }
        case QEvent::Enter:
        case QEvent::HoverEnter:
        case QEvent::HoverMove:
        case QEvent::FocusIn:
            QToolTip::showText(QCursor::pos(), helpText, nullptr);
            statusBar()->showMessage(helpText);
            break;
        case QEvent::Leave:
        case QEvent::HoverLeave:
        case QEvent::FocusOut:
            QToolTip::hideText();
            statusBar()->clearMessage();
            break;
        case QEvent::StatusTip:
            statusBar()->showMessage(helpText);
            break;
        default:
            break;
    }

    return QMainWindow::eventFilter(obj, event);
}

QString MainWindow::resolvePowerStageYamlPath() const
{
    const QString appDir = QCoreApplication::applicationDirPath();
    const QStringList candidates = {
        QDir(QDir::currentPath()).filePath("powerstages.yaml"),
        QDir(appDir).filePath("powerstages.yaml"),
        QDir(appDir).filePath("../powerstages.yaml"),
        QDir(appDir).filePath("../../powerstages.yaml")
    };
    for (const QString& path : candidates)
    {
        if (QFileInfo::exists(path))
            return QDir::cleanPath(path);
    }
    return QString();
}

const MainWindow::PowerStagePreset* MainWindow::findPowerStagePreset(const QString& key) const
{
    const auto it = m_powerStagePresetByKey.find(key);
    if (it == m_powerStagePresetByKey.end())
        return nullptr;
    const int idx = it.value();
    if (idx < 0 || idx >= m_powerStagePresets.size())
        return nullptr;
    return &m_powerStagePresets[idx];
}

QString MainWindow::currentPowerStagePresetKey() const
{
    if (!ui || !ui->powerStagePreset)
        return QString();
    return ui->powerStagePreset->currentData().toString();
}

void MainWindow::applyPowerStagePreset(const PowerStagePreset& preset)
{
    if (!ui)
        return;

    if (preset.has_deadtime_us)
    {
        ui->deadtimeUs->setText(QString::number(preset.deadtime_us, 'f', 3));
        on_deadtimeUs_editingFinished();
    }
    if (preset.has_vref_v)
        ui->vrefV->setText(QString::number(preset.vref_v, 'f', 1));
    if (preset.has_kv)
        ui->kvExp->setText(QString::number(preset.kv, 'f', 2));
    if (preset.has_diode_vf_25)
        ui->diodeVf25->setText(QString::number(preset.diode_vf_25, 'f', 3));
    if (preset.has_diode_vf_125)
        ui->diodeVf125->setText(QString::number(preset.diode_vf_125, 'f', 3));
    if (preset.has_rth_jc_igbt)
        ui->rthJcIgbt->setText(QString::number(preset.rth_jc_igbt, 'f', 4));
    if (preset.has_rth_jc_diode)
        ui->rthJcDiode->setText(QString::number(preset.rth_jc_diode, 'f', 4));
    if (preset.has_rth_cs)
        ui->rthCs->setText(QString::number(preset.rth_cs, 'f', 4));

    if (!preset.vce_points.isEmpty())
        ui->vcePoints->setPlainText(FormatCurvePointsText(preset.vce_points, 4));
    if (!preset.eon_points.isEmpty())
        ui->eonPoints->setPlainText(FormatCurvePointsText(preset.eon_points, 3));
    if (!preset.eoff_points.isEmpty())
        ui->eoffPoints->setPlainText(FormatCurvePointsText(preset.eoff_points, 3));
    if (!preset.irr_points.isEmpty())
        ui->irrPoints->setPlainText(FormatCurvePointsText(preset.irr_points, 3));
    if (!preset.trr_points.isEmpty())
        ui->trrPoints->setPlainText(FormatCurvePointsText(preset.trr_points, 3));
}

void MainWindow::loadPowerStagePresets()
{
    if (!ui || !ui->powerStagePreset)
        return;

    m_powerStagePresets.clear();
    m_powerStagePresetByKey.clear();

    const QString yamlPath = resolvePowerStageYamlPath();
    if (yamlPath.isEmpty())
    {
        QSignalBlocker blocker(ui->powerStagePreset);
        ui->powerStagePreset->clear();
        ui->powerStagePreset->addItem("Missing powerstages.yaml", QString());
        ui->powerStagePreset->setEnabled(false);
        qWarning().noquote() << "powerstages.yaml not found (preset dropdown disabled)";
        return;
    }

    QFile file(yamlPath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        QSignalBlocker blocker(ui->powerStagePreset);
        ui->powerStagePreset->clear();
        ui->powerStagePreset->addItem("Unable to read powerstages.yaml", QString());
        ui->powerStagePreset->setEnabled(false);
        qWarning().noquote() << QString("Unable to read powerstages.yaml at '%1'").arg(yamlPath);
        return;
    }

    const QString yamlContent = QString::fromUtf8(file.readAll());

    // New schema (schema_version: 1, power_stages: - id / display_name ...)
    if (yamlContent.contains("power_stages:"))
    {
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

            // Apply scalar fields
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

            m_powerStagePresetByKey.insert(current.preset.key, m_powerStagePresets.size());
            m_powerStagePresets.append(current.preset);
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

            // List items (including stage headers and point arrays)
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
                        // Curves without temperature: assume high-temp bucket
                        addPoint3(current.curves.eon_by_current, values[0], 125.0, values[1]);
                    }
                    else if (path.endsWith("device.switching.igbt_eoff_curve_mJ_vs_Ic_A") && values.size() >= 2)
                    {
                        addPoint3(current.curves.eoff_by_current, values[0], 125.0, values[1]);
                    }
                    else if (path.endsWith("device.switching.diode_erec_points_mJ") && values.size() >= 3)
                    {
                        // Convert Erec (mJ) to an equivalent (Irr, trr) pair using Qrr ~= E/V and Qrr ~= 0.5*Irr*trr.
                        // This is an approximation, but it lets the existing Err model consume diode recovery energy.
                        const double current_A = values[0];
                        const double tj = values[1];
                        const double erec_mJ = values[2];
                        if (erec_mJ > 0.0 && current.vref_V > 0.0)
                        {
                            const double qrr_C = (erec_mJ * 1e-3) / current.vref_V;
                            const double trr_s = 0.2e-6; // default if only Erec is provided
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

            // Global defaults
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

            // Stage metadata
            if (path.endsWith("display_name") && !value.isEmpty())
            {
                current.preset.label = unquote(value);
                continue;
            }

            // Gate drive
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

            // Switching reference voltage
            if (path.endsWith("device.switching.reference.vdc_V") || path.endsWith("device.switching.reference.vce_V"))
            {
                bool ok = false;
                const double vref = value.toDouble(&ok);
                if (ok && vref > 0.0)
                    current.vref_V = vref;
                continue;
            }

            // Thermal resistances
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

        QSignalBlocker blocker(ui->powerStagePreset);
        ui->powerStagePreset->setEnabled(true);
        ui->powerStagePreset->clear();
        ui->powerStagePreset->addItem("Custom", QString());
        for (const auto& preset : m_powerStagePresets)
            ui->powerStagePreset->addItem(preset.label, preset.key);

        qInfo().noquote() << QString("Loaded %1 power stage presets from %2").arg(m_powerStagePresets.size()).arg(yamlPath);
        statusBar()->showMessage(QString("Loaded %1 power stages").arg(m_powerStagePresets.size()), 4000);

        const QSettings settings("OpenInverter", "IPMMotorSim");
        const QString savedKey = settings.value("powerStagePreset").toString();
        int targetIndex = 0;
        if (!savedKey.isEmpty())
        {
            const auto it = m_powerStagePresetByKey.find(savedKey);
            if (it != m_powerStagePresetByKey.end())
                targetIndex = it.value() + 1;
        }
        ui->powerStagePreset->setCurrentIndex(targetIndex);

        if (targetIndex > 0)
        {
            const PowerStagePreset* preset = findPowerStagePreset(savedKey);
            if (preset)
                applyPowerStagePreset(*preset);
        }
        return;
    }

    struct PresetBuilder
    {
        PowerStagePreset preset;
        double max_current_A = 0.0;
        double reference_current_A = 0.0;
        QMap<double, TempPoint> vce_by_current;
        QVector<sim::CurvePoint> vce_points_raw;
        QMap<double, double> diode_vf_by_temp;
        QVector<sim::CurvePoint> eon_points_raw;
        QVector<sim::CurvePoint> eoff_points_raw;
        QVector<sim::CurvePoint> irr_points_raw;
        QVector<sim::CurvePoint> trr_points_raw;
        QMap<double, double> eon_by_temp;
        QMap<double, double> eoff_by_temp;
        QMap<double, double> irr_by_temp;
        QMap<double, double> trr_by_temp;
        QMap<double, double> qr_by_temp;
        QMap<double, double> irm_by_temp;

        void noteCurrent(double current)
        {
            if (current > max_current_A)
                max_current_A = current;
            if (reference_current_A <= 0.0)
                reference_current_A = current;
        }
    };

    QVector<PresetBuilder> builders;
    QHash<QString, int> builderIndex;

    auto builderForStack = [&](const QVector<YamlFrame>& stack) -> PresetBuilder*
    {
        if (stack.size() < 2 || stack[0].key != "presets")
            return nullptr;
        const QString presetKey = stack[1].key;
        if (!builderIndex.contains(presetKey))
        {
            PresetBuilder builder;
            builder.preset.key = presetKey;
            builder.preset.label = HumanizeKey(presetKey);
            builderIndex.insert(presetKey, builders.size());
            builders.append(builder);
        }
        return &builders[builderIndex.value(presetKey)];
    };

    auto pathFromStack = [](const QVector<YamlFrame>& stack)
    {
        QStringList parts;
        parts.reserve(stack.size());
        for (const auto& frame : stack)
            parts.append(frame.key);
        return parts.join('.');
    };

    auto tempBucket = [](double tj_C)
    {
        return (tj_C >= 100.0) ? 125.0 : 25.0;
    };

    const QStringList lines = yamlContent.split('\n');
    QVector<YamlFrame> stack;
    for (const QString& rawLine : lines)
    {
        const QString stripped = StripComments(rawLine);
        if (stripped.trimmed().isEmpty())
            continue;
        const int indent = LeadingSpaces(stripped);
        const QString trimmed = stripped.trimmed();

        if (trimmed.startsWith('-'))
        {
            PresetBuilder* builder = builderForStack(stack);
            if (!builder)
                continue;
            const QString path = pathFromStack(stack);
            const QString item = trimmed.mid(1).trimmed();
            if (item.startsWith('{'))
            {
                const auto map = ParseInlineMap(item);
                if (path.endsWith("conduction.igbt_vce_sat"))
                {
                    const double current = map.value("ic_A", 0.0);
                    const double tj = map.value("tj_C", 25.0);
                    const double vce = map.value("v_typ_V", map.value("v_max_V", 0.0));
                    if (current > 0.0 && vce > 0.0)
                    {
                        builder->noteCurrent(current);
                        TempPoint& point = builder->vce_by_current[current];
                        if (tempBucket(tj) < 50.0)
                            point.v25 = vce;
                        else
                            point.v125 = vce;
                    }
                }
                else if (path.endsWith("conduction.igbt_vce_sat_at_400A"))
                {
                    const double tj = map.value("tj_C", 25.0);
                    const double vce = map.value("v_typ_V", map.value("v_max_V", 0.0));
                    if (vce > 0.0)
                    {
                        const double current = 400.0;
                        builder->noteCurrent(current);
                        TempPoint& point = builder->vce_by_current[current];
                        if (tempBucket(tj) < 50.0)
                            point.v25 = vce;
                        else
                            point.v125 = vce;
                    }
                }
                else if (path.endsWith("conduction.diode_vf"))
                {
                    const double tj = map.value("tj_C", 25.0);
                    const double vf = map.value("v_typ_V", map.value("v_max_V", 0.0));
                    if (vf > 0.0)
                        builder->diode_vf_by_temp[tempBucket(tj)] = vf;
                }
                else if (path.endsWith("conduction.diode_vf_at_400A"))
                {
                    const double tj = map.value("tj_C", 25.0);
                    const double vf = map.value("vf_typ_V", map.value("vf_max_V", 0.0));
                    if (vf > 0.0)
                        builder->diode_vf_by_temp[tempBucket(tj)] = vf;
                }
                else if (path.endsWith("switching.igbt_eon_mJ"))
                {
                    const double tj = map.value("tj_C", 25.0);
                    const double e = map.value("e_mJ", 0.0);
                    if (e > 0.0)
                        builder->eon_by_temp[tempBucket(tj)] = e;
                }
                else if (path.endsWith("switching.igbt_eoff_mJ"))
                {
                    const double tj = map.value("tj_C", 25.0);
                    const double e = map.value("e_mJ", 0.0);
                    if (e > 0.0)
                        builder->eoff_by_temp[tempBucket(tj)] = e;
                }
                else if (path.endsWith("switching.diode_recovery.qr_uC"))
                {
                    const double tj = map.value("tj_C", 25.0);
                    const double qr = map.value("q_uC", 0.0);
                    if (qr > 0.0)
                        builder->qr_by_temp[tempBucket(tj)] = qr;
                }
                else if (path.endsWith("switching.diode_recovery.irm_A"))
                {
                    const double tj = map.value("tj_C", 25.0);
                    const double irm = map.value("irm_A", 0.0);
                    if (irm > 0.0)
                        builder->irm_by_temp[tempBucket(tj)] = irm;
                }
            }
            else if (item.startsWith('['))
            {
                const auto values = ParseInlineList(item);
                if (values.size() >= 2)
                {
                    const double current = values[0];
                    const double val = values[1];
                    if (path.endsWith("conduction.igbt_vce_on.piecewise_linear_V_at_A"))
                    {
                        if (current >= 0.0)
                        {
                            builder->noteCurrent(current);
                            builder->vce_points_raw.append({current, val, val});
                        }
                    }
                    else if (path.endsWith("switching_energy_approx_from_plots.esw_on_mJ_vs_ic_A"))
                    {
                        if (current >= 0.0)
                        {
                            builder->noteCurrent(current);
                            builder->eon_points_raw.append({current, val, val});
                        }
                    }
                    else if (path.endsWith("switching_energy_approx_from_plots.esw_off_mJ_vs_ic_A"))
                    {
                        if (current >= 0.0)
                        {
                            builder->noteCurrent(current);
                            builder->eoff_points_raw.append({current, val, val});
                        }
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

        PresetBuilder* builder = builderForStack(stack);
        if (!builder)
            continue;

        const QString path = pathFromStack(stack);
        if (value.isEmpty())
            continue;

        bool ok = false;
        const double num = value.toDouble(&ok);
        if (!ok)
            continue;

        if (path.endsWith("limits.min_deadtime_us"))
        {
            builder->preset.has_deadtime_us = true;
            builder->preset.deadtime_us = num;
        }
        else if (path.endsWith("switching.reference.vdc_V") ||
                 path.endsWith("switching_energy_approx_from_plots.reference.vdc_V"))
        {
            builder->preset.has_vref_v = true;
            builder->preset.vref_v = num;
        }
        else if (path.endsWith("switching.reference.current_A"))
        {
            builder->reference_current_A = num;
            builder->noteCurrent(num);
        }
        else if (path.endsWith("conduction.diode_vf.v_typ_V"))
        {
            builder->diode_vf_by_temp[25.0] = num;
        }
        else if (path.endsWith("switching.igbt_eon_mJ.typ"))
        {
            builder->eon_by_temp[25.0] = num;
            if (!builder->eon_by_temp.contains(125.0))
                builder->eon_by_temp[125.0] = num;
        }
        else if (path.endsWith("switching.igbt_eoff_mJ.typ"))
        {
            builder->eoff_by_temp[25.0] = num;
            if (!builder->eoff_by_temp.contains(125.0))
                builder->eoff_by_temp[125.0] = num;
        }
        else if (path.endsWith("thermal.rth_jc_K_per_W.igbt_max") ||
                 path.endsWith("thermal.rth_jc_K_per_W_per_die.igbt") ||
                 path.endsWith("thermal.rth_jf_K_per_W.igbt"))
        {
            builder->preset.has_rth_jc_igbt = true;
            builder->preset.rth_jc_igbt = num;
        }
        else if (path.endsWith("thermal.rth_jc_K_per_W.diode_max") ||
                 path.endsWith("thermal.rth_jc_K_per_W_per_die.diode") ||
                 path.endsWith("thermal.rth_jf_K_per_W.diode"))
        {
            builder->preset.has_rth_jc_diode = true;
            builder->preset.rth_jc_diode = num;
        }
        else if (path.endsWith("thermal.rth_cs_K_per_W.typ") ||
                 path.endsWith("thermal.rth_case_to_cooler_K_per_W_module.rth_c_f"))
        {
            builder->preset.has_rth_cs = true;
            builder->preset.rth_cs = num;
        }
    }

    for (PresetBuilder& builder : builders)
    {
        PowerStagePreset preset = builder.preset;

        if (!preset.has_diode_vf_25 || !preset.has_diode_vf_125)
        {
            const double v25 = PickTempValue(builder.diode_vf_by_temp, 25.0, preset.diode_vf_25);
            const double v125 = PickTempValue(builder.diode_vf_by_temp, 125.0, v25);
            if (!std::isnan(v25))
            {
                preset.has_diode_vf_25 = true;
                preset.diode_vf_25 = v25;
            }
            if (!std::isnan(v125))
            {
                preset.has_diode_vf_125 = true;
                preset.diode_vf_125 = v125;
            }
        }

        QVector<sim::CurvePoint> vce_points;
        if (!builder.vce_by_current.isEmpty())
        {
            for (auto it = builder.vce_by_current.begin(); it != builder.vce_by_current.end(); ++it)
            {
                double v25 = it.value().v25;
                double v125 = it.value().v125;
                if (std::isnan(v25) && !std::isnan(v125))
                    v25 = v125;
                if (std::isnan(v125) && !std::isnan(v25))
                    v125 = v25;
                if (std::isnan(v25) && std::isnan(v125))
                    continue;
                vce_points.append({it.key(), std::isnan(v25) ? 0.0 : v25, std::isnan(v125) ? v25 : v125});
            }
        }
        if (vce_points.isEmpty())
            vce_points = builder.vce_points_raw;
        preset.vce_points = vce_points;

        QVector<sim::CurvePoint> eon_points = builder.eon_points_raw;
        QVector<sim::CurvePoint> eoff_points = builder.eoff_points_raw;

        const double refCurrent = (builder.reference_current_A > 0.0)
                                      ? builder.reference_current_A
                                      : (builder.max_current_A > 0.0 ? builder.max_current_A : 100.0);

        if (eon_points.isEmpty() && !builder.eon_by_temp.isEmpty())
        {
            const double e25 = PickTempValue(builder.eon_by_temp, 25.0, 0.0);
            const double e125 = PickTempValue(builder.eon_by_temp, 125.0, e25);
            eon_points.append({refCurrent, e25, e125});
        }
        if (eoff_points.isEmpty() && !builder.eoff_by_temp.isEmpty())
        {
            const double e25 = PickTempValue(builder.eoff_by_temp, 25.0, 0.0);
            const double e125 = PickTempValue(builder.eoff_by_temp, 125.0, e25);
            eoff_points.append({refCurrent, e25, e125});
        }

        preset.eon_points = eon_points;
        preset.eoff_points = eoff_points;

        QVector<sim::CurvePoint> irr_points = builder.irr_points_raw;
        QVector<sim::CurvePoint> trr_points = builder.trr_points_raw;

        if (irr_points.isEmpty() && !builder.irm_by_temp.isEmpty())
        {
            const double irr25 = PickTempValue(builder.irm_by_temp, 25.0, 0.0);
            const double irr125 = PickTempValue(builder.irm_by_temp, 125.0, irr25);
            irr_points.append({refCurrent, irr25, irr125});
        }

        if (trr_points.isEmpty())
        {
            if (!builder.trr_by_temp.isEmpty())
            {
                const double trr25 = PickTempValue(builder.trr_by_temp, 25.0, 0.0);
                const double trr125 = PickTempValue(builder.trr_by_temp, 125.0, trr25);
                trr_points.append({refCurrent, trr25, trr125});
            }
            else if (!builder.qr_by_temp.isEmpty() && !builder.irm_by_temp.isEmpty())
            {
                const double qr25 = PickTempValue(builder.qr_by_temp, 25.0, 0.0);
                const double qr125 = PickTempValue(builder.qr_by_temp, 125.0, qr25);
                const double irm25 = PickTempValue(builder.irm_by_temp, 25.0, 0.0);
                const double irm125 = PickTempValue(builder.irm_by_temp, 125.0, irm25);
                const double trr25 = irm25 > 0.0 ? (2.0 * qr25 / irm25) : 0.0;
                const double trr125 = irm125 > 0.0 ? (2.0 * qr125 / irm125) : trr25;
                trr_points.append({refCurrent, trr25, trr125});
            }
        }

        preset.irr_points = irr_points;
        preset.trr_points = trr_points;

        const sim::PowerModuleParams defaults = sim::PM300CLA060();
        if (!preset.has_vref_v)
        {
            preset.has_vref_v = true;
            preset.vref_v = defaults.vref_V;
        }
        if (!preset.has_kv)
        {
            preset.has_kv = true;
            preset.kv = defaults.kv;
        }
        if (!preset.has_diode_vf_25)
        {
            preset.has_diode_vf_25 = true;
            preset.diode_vf_25 = defaults.diode_vf_25C_V;
        }
        if (!preset.has_diode_vf_125)
        {
            preset.has_diode_vf_125 = true;
            preset.diode_vf_125 = defaults.diode_vf_125C_V;
        }
        if (!preset.has_rth_jc_igbt)
        {
            preset.has_rth_jc_igbt = true;
            preset.rth_jc_igbt = defaults.rth_jc_igbt_C_per_W;
        }
        if (!preset.has_rth_jc_diode)
        {
            preset.has_rth_jc_diode = true;
            preset.rth_jc_diode = defaults.rth_jc_diode_C_per_W;
        }
        if (!preset.has_rth_cs)
        {
            preset.has_rth_cs = true;
            preset.rth_cs = defaults.rth_cs_C_per_W;
        }
        if (preset.vce_points.isEmpty())
        {
            preset.vce_points.reserve(static_cast<int>(defaults.igbt_vce_sat.size()));
            for (const auto& point : defaults.igbt_vce_sat)
                preset.vce_points.append(point);
        }
        if (preset.eon_points.isEmpty())
        {
            preset.eon_points.reserve(static_cast<int>(defaults.eon_mJ.size()));
            for (const auto& point : defaults.eon_mJ)
                preset.eon_points.append(point);
        }
        if (preset.eoff_points.isEmpty())
        {
            preset.eoff_points.reserve(static_cast<int>(defaults.eoff_mJ.size()));
            for (const auto& point : defaults.eoff_mJ)
                preset.eoff_points.append(point);
        }
        if (preset.irr_points.isEmpty())
        {
            preset.irr_points.reserve(static_cast<int>(defaults.irr_A.size()));
            for (const auto& point : defaults.irr_A)
                preset.irr_points.append(point);
        }
        if (preset.trr_points.isEmpty())
        {
            preset.trr_points.reserve(static_cast<int>(defaults.trr_us.size()));
            for (const auto& point : defaults.trr_us)
                preset.trr_points.append(point);
        }

        m_powerStagePresetByKey.insert(preset.key, m_powerStagePresets.size());
        m_powerStagePresets.append(preset);
    }

    QSignalBlocker blocker(ui->powerStagePreset);
    ui->powerStagePreset->setEnabled(true);
    ui->powerStagePreset->clear();
    ui->powerStagePreset->addItem("Custom", QString());
    for (const auto& preset : m_powerStagePresets)
        ui->powerStagePreset->addItem(preset.label, preset.key);

    qInfo().noquote() << QString("Loaded %1 power stage presets from %2").arg(m_powerStagePresets.size()).arg(yamlPath);
    statusBar()->showMessage(QString("Loaded %1 power stages").arg(m_powerStagePresets.size()), 4000);

    const QSettings settings("OpenInverter", "IPMMotorSim");
    const QString savedKey = settings.value("powerStagePreset").toString();
    int targetIndex = 0;
    if (!savedKey.isEmpty())
    {
        const auto it = m_powerStagePresetByKey.find(savedKey);
        if (it != m_powerStagePresetByKey.end())
            targetIndex = it.value() + 1;
    }
    ui->powerStagePreset->setCurrentIndex(targetIndex);

    if (targetIndex > 0)
    {
        const PowerStagePreset* preset = findPowerStagePreset(savedKey);
        if (preset)
            applyPowerStagePreset(*preset);
    }
}

void MainWindow::on_powerStagePreset_currentIndexChanged(int index)
{
    Q_UNUSED(index);
    const QString key = currentPowerStagePresetKey();
    if (key.isEmpty())
        return;
    const PowerStagePreset* preset = findPowerStagePreset(key);
    if (preset)
        applyPowerStagePreset(*preset);
}

void MainWindow::runFor(int num_steps)
{
    qInfo().noquote() << QString("runFor: begin steps=%1 dt=%2 time=%3 vdc=%4 pwmfrq_param=%5")
                             .arg(num_steps)
                             .arg(m_timestep, 0, 'g', 9)
                             .arg(m_time, 0, 'g', 9)
                             .arg(m_Vdc, 0, 'f', 3)
                             .arg(Param::GetInt(Param::pwmfrq));
    qInfo().noquote() << QString("runFor: ui_throtcur=%1 Param::throtcur=%2 cb_LogCsv=%3")
                             .arg(ui->throttleCurrent ? ui->throttleCurrent->text() : QString("<null>"))
                             .arg(Param::GetFloat(Param::throtcur), 0, 'g', 9)
                             .arg(ui->cb_LogCsv && ui->cb_LogCsv->isChecked() ? "true" : "false");
    app::Breadcrumb(QString("runFor: begin steps=%1").arg(num_steps));

    double Va = 0;
    double Vb = 0;
    double Vc = 0;
    double Va_cmd = 0;
    double Vb_cmd = 0;
    double Vc_cmd = 0;

    sim::Controller controller;
    sim::Modulator modulator;
    sim::InverterSwitchingModel inverter;
    sim::ModulationMode modMode = sim::ModulationMode::Firmware;
    const int modIndex = ui->modulationMode->currentIndex();
    switch(modIndex)
    {
        case 1: modMode = sim::ModulationMode::SVPWM; break;
        case 2: modMode = sim::ModulationMode::DPWMMIN; break;
        case 3: modMode = sim::ModulationMode::DPWMMAX; break;
        case 4: modMode = sim::ModulationMode::DPWM0; break;
        case 5: modMode = sim::ModulationMode::DPWM1; break;
        default: modMode = sim::ModulationMode::Firmware; break;
    }
    const QString modModeStr = ui->modulationMode->currentText();
    const QString presetKey = currentPowerStagePresetKey();
    double modBlend = ui->modBlend->text().toDouble();
    modBlend = std::clamp(modBlend, 0.0, 1.0);

    sim::InverterParams invParams;
    invParams.pwm_frequency_hz = PwmFrequencyHzFromParam(Param::GetInt(Param::pwmfrq));
    auto readDouble = [](QLineEdit* field, double fallback)
    {
        bool ok = false;
        const double val = field ? field->text().toDouble(&ok) : fallback;
        return ok ? val : fallback;
    };
    invParams.deadtime_s = std::max(0.0, readDouble(ui->deadtimeUs, 2.0)) * 1e-6;
    invParams.sink_temp_C = readDouble(ui->sinkTemp, 25.0);
    invParams.thermal_tau_s = std::max(0.01, readDouble(ui->thermalTau, 1.0));
    sim::PowerModuleParams moduleParams = sim::PM300CLA060();
    moduleParams.vref_V = std::max(1.0, readDouble(ui->vrefV, moduleParams.vref_V));
    moduleParams.kv = std::max(0.0, readDouble(ui->kvExp, moduleParams.kv));
    moduleParams.diode_vf_25C_V = std::max(0.0, readDouble(ui->diodeVf25, moduleParams.diode_vf_25C_V));
    moduleParams.diode_vf_125C_V = std::max(0.0, readDouble(ui->diodeVf125, moduleParams.diode_vf_125C_V));
    moduleParams.rth_jc_igbt_C_per_W = std::max(0.0, readDouble(ui->rthJcIgbt, moduleParams.rth_jc_igbt_C_per_W));
    moduleParams.rth_jc_diode_C_per_W = std::max(0.0, readDouble(ui->rthJcDiode, moduleParams.rth_jc_diode_C_per_W));
    moduleParams.rth_cs_C_per_W = std::max(0.0, readDouble(ui->rthCs, moduleParams.rth_cs_C_per_W));
    moduleParams.igbt_vce_sat = ParseCurvePoints<4>(ui->vcePoints->toPlainText(), moduleParams.igbt_vce_sat);
    moduleParams.eon_mJ = ParseCurvePoints<3>(ui->eonPoints->toPlainText(), moduleParams.eon_mJ);
    moduleParams.eoff_mJ = ParseCurvePoints<3>(ui->eoffPoints->toPlainText(), moduleParams.eoff_mJ);
    moduleParams.irr_A = ParseCurvePoints<3>(ui->irrPoints->toPlainText(), moduleParams.irr_A);
    moduleParams.trr_us = ParseCurvePoints<3>(ui->trrPoints->toPlainText(), moduleParams.trr_us);

    inverter.SetModuleParams(moduleParams);
    inverter.ResetThermals(invParams.sink_temp_C);

    if(num_steps<0)
        return;

    if (!motor)
    {
        qCritical().noquote() << "runFor: motor is null";
        app::Breadcrumb("runFor: motor is null");
        return;
    }

    QList<QPointF> listIa, listIb, listIc, listIq, listId;
    QList<QPointF> listMFreq, listMPos, listContMPos;
    QList<QPointF> listCVa, listCVb, listCVc, listCVq, listCVd, listCIq, listCId, listCifw;//, listCivlim;
    QList<QPointF> listVVd, listVVq, listVVq_bemf, listVVq_dueto_id, listVVd_dueto_iq, listVVq_dueto_Rq, listVVd_dueto_Rd, listVVLd, listVVLq;
    QList<QPointF> listPwmA, listPwmB, listPwmC, listPwmMin, listPwmMax, listPwmZero, listClampA, listClampB, listClampC;
    QList<QPointF> listPwmT1, listPwmT2, listPwmT0, listPwmSector;
    QList<QPointF> listIdIq;
    QList<QPointF> listPower, listTorque, listElecPower, listEfficiency;
    QList<QPointF> listLossIgbtCond, listLossDiodeCond, listLossIgbtSw, listLossDiodeRr, listLossTotal;
    double sumLossIgbtCond = 0.0;
    double sumLossDiodeCond = 0.0;
    double sumLossIgbtSw = 0.0;
    double sumLossDiodeRr = 0.0;
    double sumLossTotal = 0.0;
    double sumInvEff = 0.0;
    double sumTjRiseIgbt = 0.0;
    double sumTjRiseDiode = 0.0;
    int lossSamples = 0;
    int effSamples = 0;

    QFile logFile;
    QTextStream logStream;
    bool logEnabled = ui->cb_LogCsv->isChecked();
    if(logEnabled)
    {
        QDir logDir(QDir::currentPath());
        if(!logDir.exists("logs") && !logDir.mkpath("logs"))
        {
            logEnabled = false;
        }
        else
        {
            QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
            QString logPath = logDir.filePath(QString("logs/run_%1.csv").arg(timestamp));
            logFile.setFileName(logPath);
            if(logFile.open(QIODevice::WriteOnly | QIODevice::Text))
            {
                logStream.setDevice(&logFile);
                logStream.setLocale(QLocale::c());
                logStream.setRealNumberNotation(QTextStream::FixedNotation);
                logStream.setRealNumberPrecision(6);
                logStream << "# ipmmotorsim_log_version=1\n";
                logStream << "# timestamp=" << QDateTime::currentDateTimeUtc().toString(Qt::ISODate) << "\n";
                logStream << "# timestep_s=" << m_timestep << "\n";
                logStream << "# loop_freq_hz=" << (m_timestep > 0 ? (1.0 / m_timestep) : 0.0) << "\n";
                logStream << "# pwmfrq_param=" << Param::GetInt(Param::pwmfrq) << " (" << PWMFRQS << ")\n";
                logStream << "# pwm_freq_hz=" << invParams.pwm_frequency_hz << "\n";
                logStream << "# deadtime_s=" << invParams.deadtime_s << "\n";
                logStream << "# sink_temp_c=" << invParams.sink_temp_C << "\n";
                logStream << "# thermal_tau_s=" << invParams.thermal_tau_s << "\n";
                logStream << "# inverter_module=" << (presetKey.isEmpty() ? "custom" : presetKey) << "\n";
                logStream << "# module_vref_v=" << moduleParams.vref_V << ", kv=" << moduleParams.kv << "\n";
                logStream << "# module_diode_vf_25c=" << moduleParams.diode_vf_25C_V
                          << ", diode_vf_125c=" << moduleParams.diode_vf_125C_V << "\n";
                logStream << "# module_rth_jc_igbt=" << moduleParams.rth_jc_igbt_C_per_W
                          << ", rth_jc_diode=" << moduleParams.rth_jc_diode_C_per_W
                          << ", rth_cs=" << moduleParams.rth_cs_C_per_W << "\n";
                logStream << "# throtcur_A_per_pct=" << Param::GetFloat(Param::throtcur) << "\n";
                logStream << "# fwcurmax_A=" << Param::GetInt(Param::fwcurmax) << "\n";
                logStream << "# vlimmargin=" << Param::GetInt(Param::vlimmargin) << ", vlimflt=" << Param::GetInt(Param::vlimflt) << "\n";
                logStream << "# vdc=" << m_Vdc << "\n";
                logStream << "# modulation_mode=" << modModeStr << "\n";
                logStream << "# modulation_blend=" << modBlend << "\n";
                logStream << "# motor_ld=" << m_Ld << ", motor_lq=" << m_Lq << ", rs=" << m_Rs
                          << ", poles=" << m_Poles << ", fluxlinkage=" << m_fluxLinkage << "\n";
                logStream << "# sampling_point=" << m_samplingPoint << ", sync_delay_s=" << m_syncdelay
                          << ", road_gradient=" << m_roadGradient << "\n";
                logStream << "time_s,step,pwm_enabled,vdc,"
                          << "duty_a,duty_b,duty_c,"
                          << "va_cmd,vb_cmd,vc_cmd,va,vb,vc,"
                          << "ia,ib,ic,id,iq,"
                          << "id_ctrl,iq_ctrl,ifw,vd_ctrl,vq_ctrl,"
                          << "theta_e_deg,rpm,torque_nm,power_w,"
                          << "mod_mode,mod_blend,sector,t1,t2,t0,zero_seq,clamp_leg,clamp_pol,"
                          << "inv_igbt_cond_w,inv_diode_cond_w,inv_igbt_sw_w,inv_diode_rr_w,inv_total_w,inv_eff_pct,"
                          << "tcase_c,tj_igbt_c,tj_diode_c\n";
                statusBar()->showMessage(QString("Logging to %1").arg(logPath), 5000);
            }
            else
            {
                logEnabled = false;
            }
        }
    }

    //PwmGeneration::SetTorquePercent(ui->torqueDemand->text().toFloat());
    for(int i = 0;i<num_steps; i++)
    {
        if ((i & 1023) == 0)
            app::Breadcrumb(QString("runFor: step=%1 time=%2").arg(i).arg(m_time, 0, 'g', 9));

        //routines that need calling every 10ms
        if((uint32_t)(m_time*100) != m_old_time)
        {
            m_old_time = (uint32_t)(m_time*100);
            Encoder::UpdateRotorFrequency(100);

            int requestedTorque = ui->torqueDemand->text().toInt() * 100;
            if(ui->ThrotRamps->isChecked())
            {
                //ramps set at 5% above 0 and 0.5% below
                if(m_lastTorqueDemand != requestedTorque)
                {
                    if(requestedTorque > m_lastTorqueDemand)
                        requestedTorque = RAMPUP(m_lastTorqueDemand, requestedTorque, ((m_lastTorqueDemand>=0)?500:50));
                    else
                        requestedTorque = RAMPDOWN(m_lastTorqueDemand, requestedTorque, ((m_lastTorqueDemand>=0)?500:50));
                    m_lastTorqueDemand = requestedTorque;
                }
                controller.SetTorquePercent(((float)(requestedTorque+50))/100);
            }
            else
                controller.SetTorquePercent(ui->torqueDemand->text().toFloat());
        }

        //routines that need calling every ms
        if((uint32_t)(m_time*1000) != m_old_ms_time)
        {
            m_old_ms_time = (uint32_t)(m_time*100);
            //not used at the moment but left in for future use
        }        

        controller.SetRotorAngle((uint16_t)((motor->getElecPosition()*TWO_PI_CONT)/360.0));
        bool pwmEnabled = controller.PwmEnabled();
        double il1_input = 0;
        double il2_input = 0;
        if(pwmEnabled)
        {
            il1_input = (Param::GetFloat(Param::il1gain)*motor->getIaSamp());
            il2_input = (Param::GetFloat(Param::il2gain)*motor->getIbSamp());
        }

        if(ui->AddNoise->isChecked())
        {
            double noise = ui->NoiseAmp->text().toDouble();
            il1_input += QRandomGenerator::global()->bounded(noise) - (noise/2);
            il2_input += QRandomGenerator::global()->bounded(noise) - (noise/2);
        }

        controller.SetCurrentInputs(il1_input, il2_input);
        controller.Run();

        pwmEnabled = controller.PwmEnabled();
        sim::DutyCycles duty;
        sim::ModulatorDiag modDiag{};
        sim::PhaseVoltages voltages;
        sim::LossBreakdown invLoss{};
        sim::ThermalState invThermal{};
        const sim::PhaseCurrents phaseCurrents{
            motor->getIaSamp(),
            motor->getIbSamp(),
            motor->getIcSamp()
        };
        if(!pwmEnabled) //needed to allow OpenInverter initialisation to complete
        {
            voltages = {};
        }
        else
        {
            const double theta = qDegreesToRadians(motor->getElecPosition());
            const double vd_ctrl = controller.UdVolts(m_Vdc);
            const double vq_ctrl = controller.UqVolts(m_Vdc);
            if (!std::isfinite(theta) || !std::isfinite(vd_ctrl) || !std::isfinite(vq_ctrl))
            {
                qWarning().noquote() << QString("runFor: non-finite ctrl values at step=%1 time=%2 theta=%3 vd=%4 vq=%5")
                                            .arg(i)
                                            .arg(m_time, 0, 'g', 9)
                                            .arg(theta, 0, 'g', 9)
                                            .arg(vd_ctrl, 0, 'g', 9)
                                            .arg(vq_ctrl, 0, 'g', 9);
            }
            const double v_alpha = (vd_ctrl * qCos(theta)) - (vq_ctrl * qSin(theta));
            const double v_beta = (vd_ctrl * qSin(theta)) + (vq_ctrl * qCos(theta));

            if(modMode == sim::ModulationMode::Firmware)
            {
                duty = modulator.GetDutyCycles();
                voltages = inverter.FromDuty(m_Vdc, duty, phaseCurrents, m_timestep, invParams, &invLoss, &invThermal);
                modulator.ComputeFromAlphaBeta(v_alpha, v_beta, m_Vdc, sim::ModulationMode::SVPWM, modBlend, &modDiag);
            }
            else
            {
                duty = modulator.ComputeFromAlphaBeta(v_alpha, v_beta, m_Vdc, modMode, modBlend, &modDiag);
                voltages = inverter.FromDuty(m_Vdc, duty, phaseCurrents, m_timestep, invParams, &invLoss, &invThermal);
            }
        }

        Va = voltages.a;
        Vb = voltages.b;
        Vc = voltages.c;

        Va_cmd = Va;
        Vb_cmd = Vb;
        Vc_cmd = Vc;

        if(ui->cb_Pwm->isChecked())
        {
            listPwmA.append(QPointF(m_time, duty.a_norm));
            listPwmB.append(QPointF(m_time, duty.b_norm));
            listPwmC.append(QPointF(m_time, duty.c_norm));
            const double dutyMin = std::min(duty.a_norm, std::min(duty.b_norm, duty.c_norm));
            const double dutyMax = std::max(duty.a_norm, std::max(duty.b_norm, duty.c_norm));
            listPwmMin.append(QPointF(m_time, dutyMin));
            listPwmMax.append(QPointF(m_time, dutyMax));
            double zeroSeq = modDiag.zero_seq;
            listPwmZero.append(QPointF(m_time, zeroSeq));

            double clampA = 0.0, clampB = 0.0, clampC = 0.0;
            if(modDiag.clamp_leg >= 0)
            {
                if(modDiag.clamp_leg == 0) clampA = modDiag.clamp_polarity;
                if(modDiag.clamp_leg == 1) clampB = modDiag.clamp_polarity;
                if(modDiag.clamp_leg == 2) clampC = modDiag.clamp_polarity;
            }
            else
            {
                const double eps = 1e-4;
                auto clampVal = [eps](double d)
                {
                    if(d <= eps)
                        return -1.0;
                    if(d >= (1.0 - eps))
                        return 1.0;
                    return 0.0;
                };
                clampA = clampVal(duty.a_norm);
                clampB = clampVal(duty.b_norm);
                clampC = clampVal(duty.c_norm);
            }
            listClampA.append(QPointF(m_time, clampA));
            listClampB.append(QPointF(m_time, clampB));
            listClampC.append(QPointF(m_time, clampC));

            listPwmT1.append(QPointF(m_time, modDiag.t1));
            listPwmT2.append(QPointF(m_time, modDiag.t2));
            listPwmT0.append(QPointF(m_time, modDiag.t0));
            listPwmSector.append(QPointF(m_time, modDiag.sector));
        }

        //add voltages to plot here so that we see the SVM waveforms
        if(ui->cb_PhaseVolts->isChecked())
        {
            listCVa.append(QPointF(m_time, Va));
            listCVb.append(QPointF(m_time, Vb));
            listCVc.append(QPointF(m_time, Vc));
        }

        //remove space vector modulation
        inverter.RemoveCommonMode(voltages);
        Va = voltages.a;
        Vb = voltages.b;
        Vc = voltages.c;

        //one period delay to simulate slow timer reload in target hardware
        if(ui->ExtraCycleDelay->isChecked())
            motor->Step(m_oldVa,m_oldVb,m_oldVc);
        else
            motor->Step(Va,Vb,Vc);
        if (!std::isfinite(motor->getIaSamp()) || !std::isfinite(motor->getIbSamp()) || !std::isfinite(motor->getIcSamp()))
        {
            qWarning().noquote() << QString("runFor: non-finite phase currents at step=%1 time=%2 ia=%3 ib=%4 ic=%5")
                                        .arg(i)
                                        .arg(m_time, 0, 'g', 9)
                                        .arg(motor->getIaSamp(), 0, 'g', 9)
                                        .arg(motor->getIbSamp(), 0, 'g', 9)
                                        .arg(motor->getIcSamp(), 0, 'g', 9);
        }
        m_oldVa = Va;
        m_oldVb = Vb;
        m_oldVb = Vb;

        //motor->Step(0,0,0);
        if(ui->cb_PhaseCurrs->isChecked())
        {
            listIa.append(QPointF(m_time, motor->getIaSamp()));
            listIb.append(QPointF(m_time, motor->getIbSamp()));
            listIc.append(QPointF(m_time, motor->getIcSamp()));
        }
        listIq.append(QPointF(m_time, motor->getIq()));
        listId.append(QPointF(m_time, motor->getId()));

        listMFreq.append(QPointF(m_time, (motor->getMotorFreq()*m_Poles)));
        if(ui->cb_MotorPos->isChecked())
        {
            listMPos.append(QPointF(m_time, motor->getMotorPosition()));
            listContMPos.append(QPointF(m_time, (360.0 * PwmGeneration::GetAngle())/TWO_PI_CONT));
        }

          //inlcude here to see sinusoidal waveforms that motor sees
//        if(ui->cb_PhaseVolts->isChecked())
//        {
//            listCVa.append(QPointF(m_time, Va));
//            listCVb.append(QPointF(m_time, Vb));
//            listCVc.append(QPointF(m_time, Vc));
//        }
        listCVq.append(QPointF(m_time, (m_Vdc/65536) * Param::GetFloat(Param::uq)));
        listCVd.append(QPointF(m_time, (m_Vdc/65536) * Param::GetFloat(Param::ud)));

        listCIq.append(QPointF(m_time, Param::GetFloat(Param::iq)));
        listCId.append(QPointF(m_time, Param::GetFloat(Param::id)));

        listCifw.append(QPointF(m_time, Param::GetFloat(Param::ifw)));
        //listCivlim.append(QPointF(m_time, Param::GetFloat(Param::vlim)));

        listVVd.append(QPointF(m_time, motor->getVd()));
        listVVq.append(QPointF(m_time, motor->getVq()));
        listVVq_bemf.append(QPointF(m_time, motor->getVq_bemf()));
        listVVq_dueto_id.append(QPointF(m_time, motor->getVq_dueto_id()));
        listVVd_dueto_iq.append(QPointF(m_time, motor->getVd_dueto_iq()));
        listVVq_dueto_Rq.append(QPointF(m_time, motor->getVq_dueto_Rq()));
        listVVd_dueto_Rd.append(QPointF(m_time, motor->getVd_dueto_Rd()));
        listVVLd.append(QPointF(m_time, motor->getVLd()));
        listVVLq.append(QPointF(m_time, motor->getVLq()));

        if(ui->rb_OP_Amps->isChecked())
            listIdIq.append(QPointF(motor->getId(), motor->getIq()));
        else
            listIdIq.append(QPointF(motor->getVd(), motor->getVq()));

        const double elec_power = (Va * motor->getIaSamp()) + (Vb * motor->getIbSamp()) + (Vc * motor->getIcSamp());
        double efficiency = 0;
        if(ui->cb_Efficiency->isChecked() && std::abs(elec_power) > 1e-9)
            efficiency = 100.0 * (motor->getPower() / elec_power);

        const double inv_igbt_cond_W = invLoss.phase[0].igbt_cond_W + invLoss.phase[1].igbt_cond_W + invLoss.phase[2].igbt_cond_W;
        const double inv_diode_cond_W = invLoss.phase[0].diode_cond_W + invLoss.phase[1].diode_cond_W + invLoss.phase[2].diode_cond_W;
        const double inv_igbt_sw_W = invLoss.phase[0].igbt_sw_W + invLoss.phase[1].igbt_sw_W + invLoss.phase[2].igbt_sw_W;
        const double inv_diode_rr_W = invLoss.phase[0].diode_rr_W + invLoss.phase[1].diode_rr_W + invLoss.phase[2].diode_rr_W;
        const double inv_total_W = inv_igbt_cond_W + inv_diode_cond_W + inv_igbt_sw_W + inv_diode_rr_W;
        double inv_eff = 0.0;
        if(elec_power > 1e-6)
            inv_eff = 100.0 * (elec_power / (elec_power + inv_total_W));
        const double tj_igbt_avg = (invThermal.igbt_C[0] + invThermal.igbt_C[1] + invThermal.igbt_C[2]) / 3.0;
        const double tj_diode_avg = (invThermal.diode_C[0] + invThermal.diode_C[1] + invThermal.diode_C[2]) / 3.0;
        if(pwmEnabled)
        {
            sumLossIgbtCond += inv_igbt_cond_W;
            sumLossDiodeCond += inv_diode_cond_W;
            sumLossIgbtSw += inv_igbt_sw_W;
            sumLossDiodeRr += inv_diode_rr_W;
            sumLossTotal += inv_total_W;
            sumTjRiseIgbt += (tj_igbt_avg - invParams.sink_temp_C);
            sumTjRiseDiode += (tj_diode_avg - invParams.sink_temp_C);
            ++lossSamples;
            if(inv_eff > 0.0)
            {
                sumInvEff += inv_eff;
                ++effSamples;
            }
        }

        if(logEnabled)
        {
            const double vd_ctrl = controller.UdVolts(m_Vdc);
            const double vq_ctrl = controller.UqVolts(m_Vdc);
            const double rpm = motor->getMotorFreq() * 60.0;
            const double zero_seq_log = modDiag.zero_seq;
            logStream << m_time << "," << i << "," << (pwmEnabled ? 1 : 0) << "," << m_Vdc << ","
                      << duty.a_norm << "," << duty.b_norm << "," << duty.c_norm << ","
                      << Va_cmd << "," << Vb_cmd << "," << Vc_cmd << ","
                      << Va << "," << Vb << "," << Vc << ","
                      << motor->getIaSamp() << "," << motor->getIbSamp() << "," << motor->getIcSamp() << ","
                      << motor->getId() << "," << motor->getIq() << ","
                      << controller.Id() << "," << controller.Iq() << "," << controller.Ifw() << ","
                      << vd_ctrl << "," << vq_ctrl << ","
                      << motor->getElecPosition() << "," << rpm << "," << motor->getTorque() << "," << motor->getPower() << ","
                      << modModeStr << "," << modBlend << "," << modDiag.sector << "," << modDiag.t1 << "," << modDiag.t2 << "," << modDiag.t0 << ","
                      << zero_seq_log << "," << modDiag.clamp_leg << "," << modDiag.clamp_polarity << ","
                      << inv_igbt_cond_W << "," << inv_diode_cond_W << "," << inv_igbt_sw_W << "," << inv_diode_rr_W << ","
                      << inv_total_W << "," << inv_eff << ","
                      << invThermal.case_C << "," << tj_igbt_avg << "," << tj_diode_avg
                      << "\n";
        }

        if(ui->rb_Speed->isChecked())
        {
            listPower.append(QPointF(motor->getMotorFreq()*60, motor->getPower()/1000));
            listTorque.append(QPointF(motor->getMotorFreq()*60, motor->getTorque()));
            listLossIgbtCond.append(QPointF(motor->getMotorFreq()*60, inv_igbt_cond_W/1000));
            listLossDiodeCond.append(QPointF(motor->getMotorFreq()*60, inv_diode_cond_W/1000));
            listLossIgbtSw.append(QPointF(motor->getMotorFreq()*60, inv_igbt_sw_W/1000));
            listLossDiodeRr.append(QPointF(motor->getMotorFreq()*60, inv_diode_rr_W/1000));
            listLossTotal.append(QPointF(motor->getMotorFreq()*60, inv_total_W/1000));
            if(ui->cb_Efficiency->isChecked())
            {
                listElecPower.append(QPointF(motor->getMotorFreq()*60, elec_power/1000));
                listEfficiency.append(QPointF(motor->getMotorFreq()*60, efficiency));
            }
        }
        else
        {
            listPower.append(QPointF(m_time, motor->getPower()/1000));
            listTorque.append(QPointF(m_time, motor->getTorque()));
            listLossIgbtCond.append(QPointF(m_time, inv_igbt_cond_W/1000));
            listLossDiodeCond.append(QPointF(m_time, inv_diode_cond_W/1000));
            listLossIgbtSw.append(QPointF(m_time, inv_igbt_sw_W/1000));
            listLossDiodeRr.append(QPointF(m_time, inv_diode_rr_W/1000));
            listLossTotal.append(QPointF(m_time, inv_total_W/1000));
            if(ui->cb_Efficiency->isChecked())
            {
                listElecPower.append(QPointF(m_time, elec_power/1000));
                listEfficiency.append(QPointF(m_time, efficiency));
            }
        }

        m_time += m_timestep;
    }

    qInfo().noquote() << "runFor: end";
    app::Breadcrumb("runFor: end");

    QString lossInfo;
    if(lossSamples > 0)
    {
        const double avgIgbtCond = sumLossIgbtCond / lossSamples;
        const double avgDiodeCond = sumLossDiodeCond / lossSamples;
        const double avgIgbtSw = sumLossIgbtSw / lossSamples;
        const double avgDiodeRr = sumLossDiodeRr / lossSamples;
        const double avgTotal = sumLossTotal / lossSamples;
        const double avgEff = effSamples > 0 ? (sumInvEff / effSamples) : 0.0;
        const double avgTjRiseIgbt = sumTjRiseIgbt / lossSamples;
        const double avgTjRiseDiode = sumTjRiseDiode / lossSamples;

        lossInfo = QString("Avg loss: %1 kW | IGBT cond %2 kW | Diode cond %3 kW | IGBT sw %4 kW | "
                           "Diode RR %5 kW | Inv eff %6% | ΔTj IGBT %7 C | ΔTj Diode %8 C")
                       .arg(avgTotal / 1000.0, 0, 'f', 3)
                       .arg(avgIgbtCond / 1000.0, 0, 'f', 3)
                       .arg(avgDiodeCond / 1000.0, 0, 'f', 3)
                       .arg(avgIgbtSw / 1000.0, 0, 'f', 3)
                       .arg(avgDiodeRr / 1000.0, 0, 'f', 3)
                       .arg(avgEff, 0, 'f', 1)
                       .arg(avgTjRiseIgbt, 0, 'f', 1)
                       .arg(avgTjRiseDiode, 0, 'f', 1);
    }
    lossGraph->setInfoText(lossInfo);

    motorGraph->addDataPoints(listIa, IA);
    motorGraph->addDataPoints(listIb, IB);
    motorGraph->addDataPoints(listIc, IC);
    motorGraph->addDataPoints(listIq, IQ);
    motorGraph->addDataPoints(listId, ID);

    simulationGraph->addDataPoints(listMFreq, M_RPM);
    simulationGraph->addDataPoints(listMPos, M_MOTOR_POS);
    simulationGraph->addDataPoints(listContMPos, M_CONT_POS);

    controllerGraph->addDataPoints(listCVa, VA);
    controllerGraph->addDataPoints(listCVb, VB);
    controllerGraph->addDataPoints(listCVc, VC);
    controllerGraph->addDataPoints(listCVq, VQ);
    controllerGraph->addDataPoints(listCVd, VD);

    debugGraph->addDataPoints(listCIq, C_IQ);
    debugGraph->addDataPoints(listCId, C_ID);
    debugGraph->addDataPoints(listCifw, C_IFW);
    //debugGraph->addDataPoints(listCivlim, C_IVLIM);

    voltageGraph->addDataPoints(listVVd, VVD);
    voltageGraph->addDataPoints(listVVq, VVQ);
    voltageGraph->addDataPoints(listVVq_bemf, VVQ_BEMF);
    voltageGraph->addDataPoints(listVVq_dueto_id, VVQ_DT_ID);
    voltageGraph->addDataPoints(listVVd_dueto_iq, VVD_DT_IQ);
    voltageGraph->addDataPoints(listVVq_dueto_Rq, VVQ_DT_RQ);
    voltageGraph->addDataPoints(listVVd_dueto_Rd, VVD_DT_RD);
    voltageGraph->addDataPoints(listVVLd, VVLD);
    voltageGraph->addDataPoints(listVVLq, VVLQ);

    pwmGraph->addDataPoints(listPwmA, PWM_A);
    pwmGraph->addDataPoints(listPwmB, PWM_B);
    pwmGraph->addDataPoints(listPwmC, PWM_C);
    pwmGraph->addDataPoints(listPwmMin, PWM_MIN);
    pwmGraph->addDataPoints(listPwmMax, PWM_MAX);
    pwmGraph->addDataPoints(listPwmZero, PWM_ZEROSEQ);
    pwmGraph->addDataPoints(listClampA, PWM_CLAMP_A);
    pwmGraph->addDataPoints(listClampB, PWM_CLAMP_B);
    pwmGraph->addDataPoints(listClampC, PWM_CLAMP_C);
    pwmGraph->addDataPoints(listPwmT1, PWM_T1);
    pwmGraph->addDataPoints(listPwmT2, PWM_T2);
    pwmGraph->addDataPoints(listPwmT0, PWM_T0);
    pwmGraph->addDataPoints(listPwmSector, PWM_SECTOR);

    idigGraph->addDataPoints(listIdIq, IDIQAMPS);

    powerGraph->addDataPoints(listPower, POWER);
    powerGraph->addDataPoints(listTorque, TORQUE);
    powerGraph->addDataPoints(listElecPower, ELEC_POWER);
    powerGraph->addDataPoints(listEfficiency, EFFICIENCY);
    lossGraph->addDataPoints(listLossIgbtCond, LOSS_IGBT_COND);
    lossGraph->addDataPoints(listLossDiodeCond, LOSS_DIODE_COND);
    lossGraph->addDataPoints(listLossIgbtSw, LOSS_IGBT_SW);
    lossGraph->addDataPoints(listLossDiodeRr, LOSS_DIODE_RR);
    lossGraph->addDataPoints(listLossTotal, LOSS_TOTAL);

    if(ui->cb_MotCurr->isChecked()) motorGraph->updateGraph();
    if(ui->cb_Simulation->isChecked()) simulationGraph->updateGraph();
    if(ui->cb_ContVolt->isChecked()) controllerGraph->updateGraph();
    if(ui->cb_ContCurr->isChecked()) debugGraph->updateGraph();
    if(ui->cb_MotVolt->isChecked()) voltageGraph->updateGraph();
    if(ui->cb_Pwm->isChecked()) pwmGraph->updateGraph();
    if(ui->cb_OpPoint->isChecked()) idigGraph->updateGraph(ui->rb_OP_Amps->isChecked());
    if(ui->cb_PowTorqTime->isChecked()) powerGraph->updateGraph();
    if(ui->cb_Losses->isChecked()) lossGraph->updateGraph();
}

void MainWindow::on_vehicleWeight_editingFinished()
{
    m_vehicleWeight = ui->vehicleWeight->text().toDouble();
    motor->setVehicleMass(m_vehicleWeight);
}

void MainWindow::on_wheelSize_editingFinished()
{
    m_wheelSize = ui->wheelSize->text().toDouble();
    motor->setWheelSize(m_wheelSize);
}

void MainWindow::on_gearRatio_editingFinished()
{
    m_gearRatio = ui->gearRatio->text().toDouble();
    motor->setGboxRatio(m_gearRatio);
}

void MainWindow::on_Vdc_editingFinished()
{
    m_Vdc = ui->Vdc->text().toDouble();
    Param::SetFloat(Param::udc, m_Vdc);
}

void MainWindow::on_Lq_editingFinished()
{
    m_Lq = ui->Lq->text().toDouble()/1000;
    motor->setLq(m_Lq);
}

void MainWindow::on_Ld_editingFinished()
{
    m_Ld = ui->Ld->text().toDouble()/1000;
    motor->setLd(m_Ld);
}

void MainWindow::on_Rs_editingFinished()
{
    m_Rs = ui->Rs->text().toDouble();
    motor->setRs(m_Rs);
}

void MainWindow::on_Poles_editingFinished()
{
    m_Poles = ui->Poles->text().toDouble();
    Param::Set(Param::polepairs, FP_FROMINT(ui->Poles->text().toInt()));
    Param::Set(Param::respolepairs,FP_FROMINT(ui->Poles->text().toInt())); //force resolver pole pairs to match motor
    motor->setPoles(m_Poles);
}

void MainWindow::on_FluxLinkage_editingFinished()
{
    m_fluxLinkage = ui->FluxLinkage->text().toDouble()/1000;
    Param::Set(Param::fluxlinkage, FP_FROMFLT(ui->FluxLinkage->text().toFloat()));
    motor->setFluxLinkage(m_fluxLinkage);
    PwmGeneration::SetTorquePercent(ui->torqueDemand->text().toFloat()); //make sure is recalculated
}

void MainWindow::on_LoopFreq_editingFinished()
{
    m_timestep = 1.0 / ui->LoopFreq->text().toDouble();
}

void MainWindow::on_pbRunFor_clicked()
{
    qInfo().noquote() << "UI: RunFor clicked";
    app::Breadcrumb("UI: RunFor clicked");
    runFor(int(m_runTime/m_timestep));
}

void MainWindow::on_pbRunFor10s_clicked()
{
    qInfo().noquote() << "UI: RunFor10s clicked";
    app::Breadcrumb("UI: RunFor10s clicked");
    runFor(int(10.0/m_timestep));
}

void MainWindow::on_pbRunFor1s_clicked()
{
    qInfo().noquote() << "UI: RunFor1s clicked";
    app::Breadcrumb("UI: RunFor1s clicked");
    runFor(int(1.0/m_timestep));
}

void MainWindow::on_pbRunFor100ms_clicked()
{
    qInfo().noquote() << "UI: RunFor100ms clicked";
    app::Breadcrumb("UI: RunFor100ms clicked");
    runFor(int(0.1/m_timestep));
}

void MainWindow::on_pbRunFor10ms_clicked()
{
    qInfo().noquote() << "UI: RunFor10ms clicked";
    app::Breadcrumb("UI: RunFor10ms clicked");
    runFor(int(0.01/m_timestep));
}

void MainWindow::on_pbStep_clicked()
{
    runFor(1);
}

void MainWindow::on_pbRestart_clicked()
{
    motor->Restart();
    int throt = ui->torqueDemand->text().toInt();
    ui->torqueDemand->setText(QString::number(0));
    PwmGeneration::SetOpmode(0);
    PwmGeneration::SetOpmode(ui->opMode->text().toInt()); //reset controller integrators
    PwmGeneration::SetTorquePercent(0);
    runFor(6000); //allow controller to complete initialisation
    ui->torqueDemand->setText(QString::number(throt));
    PwmGeneration::SetTorquePercent(ui->torqueDemand->text().toFloat());
    testStubsClearEncoder();
    m_time = 0;
    motor->Restart();
    on_startRpm_editingFinished();
    on_modBlend_editingFinished();
    motor->setMotorRpm(ui->startRpm->text().toDouble());
    motorGraph->clearData();
    simulationGraph->clearData();
    controllerGraph->clearData();
    debugGraph->clearData();
    voltageGraph->clearData();
    pwmGraph->clearData();
    idigGraph->clearData();
    powerGraph->clearData();
    lossGraph->clearData();
}


void MainWindow::on_torqueDemand_editingFinished()
{
    QSettings settings("OpenInverter", "IPMMotorSim");
    settings.setValue(ui->torqueDemand->objectName(), ui->torqueDemand->text());
    PwmGeneration::SetTorquePercent(ui->torqueDemand->text().toFloat());
}

void MainWindow::on_throttleCurrent_editingFinished()
{
    QSettings settings("OpenInverter", "IPMMotorSim");
    settings.setValue(ui->throttleCurrent->objectName(), ui->throttleCurrent->text());
    Param::Set(Param::throtcur, FP_FROMFLT(ui->throttleCurrent->text().toFloat()));
    PwmGeneration::SetTorquePercent(ui->torqueDemand->text().toFloat()); //make sure is recalculated
}

void MainWindow::on_opMode_editingFinished()
{
    PwmGeneration::SetOpmode(ui->opMode->text().toInt());
}

void MainWindow::on_direction_editingFinished()
{
    Param::Set(Param::seldir, FP_FROMINT(ui->direction->text().toInt()));
}

void MainWindow::on_IqManual_editingFinished()
{
    Param::Set(Param::manualiq, FP_FROMFLT(ui->IqManual->text().toFloat()));
}

void MainWindow::on_IdManual_editingFinished()
{
    Param::Set(Param::manualid, FP_FROMFLT(ui->IdManual->text().toFloat()));
}

void MainWindow::on_CurrentKp_editingFinished()
{
    Param::Set(Param::iqkp, FP_FROMINT(ui->CurrentKp->text().toInt()));
    Param::Set(Param::idkp, FP_FROMINT(ui->CurrentKp->text().toInt()));
}

void MainWindow::on_CurrentKi_editingFinished()
{
    Param::Set(Param::curki, FP_FROMINT(ui->CurrentKi->text().toInt()));
}

void MainWindow::on_SyncAdv_editingFinished()
{
    Param::Set(Param::syncadv, FP_FROMINT(ui->SyncAdv->text().toInt()));
}

void MainWindow::on_LqMinusLd_editingFinished()
{
    Param::Set(Param::lqminusld, FP_FROMFLT(ui->LqMinusLd->text().toFloat()));
    PwmGeneration::SetTorquePercent(ui->torqueDemand->text().toFloat()); //make sure MTPA is recalculated
}

void MainWindow::on_SyncDelay_editingFinished()
{
    m_syncdelay = ui->SyncDelay->text().toDouble()/1000000; //entered in uS
    motor->setSyncDelay(m_syncdelay);
}

void MainWindow::on_FreqMax_editingFinished()
{
    Param::Set(Param::fmax, FP_FROMFLT(ui->FreqMax->text().toFloat()));
}

void MainWindow::on_SamplingPoint_editingFinished()
{
    m_samplingPoint = ui->SamplingPoint->text().toDouble()/100.0; //entered in %
    motor->setSamplingPoint(m_samplingPoint);
}

void MainWindow::on_pbTransient_clicked()
{
    QString torque = ui->torqueDemand->text();
    for(int i=0;i<2;i++)
    {
        ui->torqueDemand->setText("0");
        runFor(int(m_runTime/m_timestep));
        ui->torqueDemand->setText(torque);
        runFor(int(m_runTime/m_timestep));
    }
}

void MainWindow::on_SyncOfs_editingFinished()
{
    Param::Set(Param::syncofs, FP_FROMINT(ui->SyncOfs->text().toInt()));
}

void MainWindow::on_pbAccelCoast_clicked()
{   
    QString torque = ui->torqueDemand->text();
    runFor(int(m_runTime/m_timestep));
    ui->torqueDemand->setText("0");
    runFor(int(m_runTime/m_timestep));
    ui->torqueDemand->setText(torque);
}

void MainWindow::on_cb_OpPoint_toggled(bool checked)
{
    if(checked)
    {
        idigGraph->updateGraph(ui->rb_OP_Amps->isChecked());
        idigGraph->show();
    }
    else
        idigGraph->hide();
}

void MainWindow::on_cb_Simulation_toggled(bool checked)
{
    if(checked)
    {
        simulationGraph->updateGraph();
        simulationGraph->show();
    }
    else
        simulationGraph->hide();
}

void MainWindow::on_cb_ContVolt_toggled(bool checked)
{
    if(checked)
    {
        controllerGraph->updateGraph();
        controllerGraph->show();
    }
    else
        controllerGraph->hide();
}

void MainWindow::on_cb_ContCurr_toggled(bool checked)
{
    if(checked)
    {
        debugGraph->updateGraph();
        debugGraph->show();
    }
    else
        debugGraph->hide();
}

void MainWindow::on_cb_MotVolt_toggled(bool checked)
{
    if(checked)
    {
        voltageGraph->updateGraph();
        voltageGraph->show();
    }
    else
        voltageGraph->hide();
}

void MainWindow::on_cb_MotCurr_toggled(bool checked)
{
    if(checked)
    {
        motorGraph->updateGraph();
        motorGraph->show();
    }
    else
        motorGraph->hide();
}

void MainWindow::on_cb_Pwm_toggled(bool checked)
{
    if(checked)
    {
        pwmGraph->updateGraph();
        pwmGraph->show();
    }
    else
        pwmGraph->hide();
}

void MainWindow::on_cb_PwmZeroSeq_toggled(bool checked)
{
    pwmGraph->setOpacity(checked ? 0.6 : 0.0, PWM_ZEROSEQ);
    if(ui->cb_Pwm->isChecked())
        pwmGraph->updateGraph();
}

void MainWindow::on_cb_PwmClamp_toggled(bool checked)
{
    const qreal opacity = checked ? 0.6 : 0.0;
    pwmGraph->setOpacity(opacity, PWM_CLAMP_A);
    pwmGraph->setOpacity(opacity, PWM_CLAMP_B);
    pwmGraph->setOpacity(opacity, PWM_CLAMP_C);
    if(ui->cb_Pwm->isChecked())
        pwmGraph->updateGraph();
}

void MainWindow::on_cb_PwmTiming_toggled(bool checked)
{
    const qreal opacity = checked ? 0.6 : 0.0;
    pwmGraph->setOpacity(opacity, PWM_T1);
    pwmGraph->setOpacity(opacity, PWM_T2);
    pwmGraph->setOpacity(opacity, PWM_T0);
    if(ui->cb_Pwm->isChecked())
        pwmGraph->updateGraph();
}

void MainWindow::on_cb_PwmSector_toggled(bool checked)
{
    pwmGraph->setOpacity(checked ? 0.6 : 0.0, PWM_SECTOR);
    if(ui->cb_Pwm->isChecked())
        pwmGraph->updateGraph();
}

void MainWindow::on_cb_ShowLegends_toggled(bool checked)
{
    motorGraph->setLegendVisible(checked);
    simulationGraph->setLegendVisible(checked);
    controllerGraph->setLegendVisible(checked);
    debugGraph->setLegendVisible(checked);
    voltageGraph->setLegendVisible(checked);
    pwmGraph->setLegendVisible(checked);
    idigGraph->setLegendVisible(checked);
    powerGraph->setLegendVisible(checked);
    lossGraph->setLegendVisible(checked);
    if(ui->cb_MotCurr->isChecked()) motorGraph->updateGraph();
    if(ui->cb_Simulation->isChecked()) simulationGraph->updateGraph();
    if(ui->cb_ContVolt->isChecked()) controllerGraph->updateGraph();
    if(ui->cb_ContCurr->isChecked()) debugGraph->updateGraph();
    if(ui->cb_MotVolt->isChecked()) voltageGraph->updateGraph();
    if(ui->cb_Pwm->isChecked()) pwmGraph->updateGraph();
    if(ui->cb_OpPoint->isChecked()) idigGraph->updateGraph(ui->rb_OP_Amps->isChecked());
    if(ui->cb_PowTorqTime->isChecked()) powerGraph->updateGraph();
    if(ui->cb_Losses->isChecked()) lossGraph->updateGraph();
}

void MainWindow::on_cb_PowTorqTime_toggled(bool checked)
{
    if(checked)
    {
        powerGraph->updateGraph();
        powerGraph->show();
    }
    else
        powerGraph->hide();
}

void MainWindow::on_cb_Losses_toggled(bool checked)
{
    if(checked)
    {
        lossGraph->updateGraph();
        lossGraph->show();
    }
    else
        lossGraph->hide();
}

void MainWindow::on_rb_Speed_toggled(bool checked)
{
    powerGraph->clearData(); //need to restart as data arrays not right for new mode
    lossGraph->clearData();
    if(checked)
    {
        powerGraph->setAxisText("Shaft Speed (rpm)", "Power (kW)", "Torque (Nm)");
        lossGraph->setAxisText("Shaft Speed (rpm)", "Loss (kW)", "");
    }
    else
    {
        powerGraph->setAxisText("Time (s)", "Power (kW)", "Torque (Nm)");
        lossGraph->setAxisText("Time (s)", "Loss (kW)", "");
    }
}

void MainWindow::on_RoadGradient_editingFinished()
{
    m_roadGradient = ui->RoadGradient->text().toDouble()/100.0; //entered in %
    motor->setRoadGradient(m_roadGradient);
}

void MainWindow::on_runTime_editingFinished()
{
    m_runTime = ui->runTime->text().toDouble();

    if(m_runTime<m_timestep)
        m_runTime = 1;
    if(m_runTime>60)
        m_runTime = 60;
}

void MainWindow::on_startRpm_editingFinished()
{
    bool ok = false;
    double rpm = ui->startRpm->text().toDouble(&ok);
    if(!ok)
        rpm = 0.0;
    rpm = std::clamp(rpm, -20000.0, 20000.0);
    ui->startRpm->setText(QString::number(rpm, 'f', 0));
}

void MainWindow::on_deadtimeUs_editingFinished()
{
    bool ok = false;
    double val = ui->deadtimeUs->text().toDouble(&ok);
    if(!ok)
        val = 2.0;
    if(val < 0.0)
        val = 0.0;
    ui->deadtimeUs->setText(QString::number(val, 'f', 3));
}

void MainWindow::on_sinkTemp_editingFinished()
{
    bool ok = false;
    double val = ui->sinkTemp->text().toDouble(&ok);
    if(!ok)
        val = 25.0;
    ui->sinkTemp->setText(QString::number(val, 'f', 1));
}

void MainWindow::on_thermalTau_editingFinished()
{
    bool ok = false;
    double val = ui->thermalTau->text().toDouble(&ok);
    if(!ok || val <= 0.0)
        val = 1.0;
    ui->thermalTau->setText(QString::number(val, 'f', 3));
}

void MainWindow::on_modBlend_editingFinished()
{
    bool ok = false;
    double blend = ui->modBlend->text().toDouble(&ok);
    if(!ok)
        blend = 1.0;
    blend = std::clamp(blend, 0.0, 1.0);
    ui->modBlend->setText(QString::number(blend, 'f', 3));
    ui->modBlendSlider->setValue(static_cast<int>(blend * 100.0));
    ui->modBlendValue->setText(QString::number(blend, 'f', 3));
}

void MainWindow::on_modBlendSlider_valueChanged(int value)
{
    double blend = std::clamp(value / 100.0, 0.0, 1.0);
    ui->modBlend->setText(QString::number(blend, 'f', 3));
    ui->modBlendValue->setText(QString::number(blend, 'f', 3));
}

void MainWindow::on_VLimMargin_editingFinished()
{
    Param::Set(Param::vlimmargin, FP_FROMINT(ui->VLimMargin->text().toInt()));
}

void MainWindow::on_VLimFlt_editingFinished()
{
    Param::Set(Param::vlimflt, FP_FROMINT(ui->VLimFlt->text().toInt()));
}

void MainWindow::on_FWCurrMax_editingFinished()
{
    Param::Set(Param::fwcurmax, FP_FROMINT(ui->FWCurrMax->text().toInt()));
}

void MainWindow::on_rb_OP_Amps_toggled(bool checked)
{
    idigGraph->clearData(); //need to restart as data arrays not right for new mode
    if(checked)
    {
        idigGraph->setAxisText("Id (A)", "Iq (A)", "");
        idigGraph->updateSeries("I (A)", left, IDIQAMPS);
    }
    else
    {
        idigGraph->setAxisText("Vd (V)", "Vq (V)", "");
        idigGraph->updateSeries("V (V)", left, IDIQAMPS);
    }
}

