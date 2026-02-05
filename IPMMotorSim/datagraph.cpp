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

#include "datagraph.h"
#include <QtCore/QRandomGenerator>
#include <QtCore/QtMath>
#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QVBoxLayout>
#include <QSettings>
#include <cmath>
#include <limits>

DataGraph::DataGraph(QString name, QWidget *parent) : QMainWindow(parent)
{
    mName = name;
    QSettings settings("OpenInverter", "IPMMotorSim");

    minY_L =  std::numeric_limits<double>::max();
    maxY_L =  std::numeric_limits<double>::lowest();
    minY_R =  std::numeric_limits<double>::max();
    maxY_R =  std::numeric_limits<double>::lowest();
    minX =  std::numeric_limits<double>::max();
    maxX =  std::numeric_limits<double>::lowest();

    m_chart = new Chart();
    m_chart->legend()->show();

    m_chartView = new ChartView(m_chart);
    m_chartView->setRenderHint(QPainter::Antialiasing);

    QWidget *container = new QWidget(this);
    QVBoxLayout *layout = new QVBoxLayout(container);
    layout->setContentsMargins(6, 4, 6, 6);
    layout->setSpacing(2);
    m_infoLabel = new QLabel(container);
    QFont infoFont = m_infoLabel->font();
    infoFont.setPointSizeF(infoFont.pointSizeF() - 1.0);
    m_infoLabel->setFont(infoFont);
    m_infoLabel->setWordWrap(true);
    m_infoLabel->setVisible(false);
    layout->addWidget(m_infoLabel);
    layout->addWidget(m_chartView, 1);

    m_axisL = new QValueAxis;
    m_axisR = new QValueAxis;
    m_axisX = new QValueAxis;
    m_chart->addAxis(m_axisX, Qt::AlignBottom);
    m_chart->addAxis(m_axisL, Qt::AlignLeft);
    m_chart->addAxis(m_axisR, Qt::AlignRight);

    setCentralWidget(container);
    if(!restoreGeometry(settings.value(mName + "/geometry").toByteArray()) || !restoreState(settings.value(mName + "/windowState").toByteArray()))
    {
        resize(1600, 300);
    }
    grabGesture(Qt::PanGesture);
    grabGesture(Qt::PinchGesture);
    show();
}

void DataGraph::setAxisText(QString x, QString left, QString right)
{
    m_axisX->setTitleText(x);
    m_axisL->setTitleText(left);
    m_axisR->setTitleText(right);
}

void DataGraph::setLegendVisible(bool visible)
{
    m_chart->legend()->setVisible(visible);
}

void DataGraph::setInfoText(const QString& text)
{
    if(!m_infoLabel)
        return;
    if(text.trimmed().isEmpty())
    {
        m_infoLabel->clear();
        m_infoLabel->setVisible(false);
        return;
    }
    m_infoLabel->setText(text);
    m_infoLabel->setVisible(true);
}

void DataGraph::saveWinState()
{
    QSettings settings("OpenInverter", "IPMMotorSim");
    QByteArray geo = saveGeometry();
    QByteArray ste = saveState();
    QString gname = mName + "/geometry";
    QString sname = mName + "/windowState";
    settings.setValue(gname, geo);
    settings.setValue(sname, ste);
    hide();
}

DataGraph::~DataGraph()
{
    clearData();
}

void DataGraph::addSeries(QString legend, axisSel axis, int key)
{
    if(m_series.contains(key))
        return;

    QList<QPointF> *series = new QList<QPointF>;
    m_series[key] = series;
    m_legends[key] = legend;
    m_axis[key] = axis;
}

void DataGraph::updateSeries(QString legend, axisSel axis, int key)
{
    if(!m_series.contains(key))
        return;

    m_legends[key] = legend;
    m_axis[key] = axis;
}



void DataGraph::addDataPoint(double x, double y, int key)
{
    if(m_series.contains(key))
    {
        if( m_axis[key] == left)
        {
            if(y<minY_L) minY_L = y;
            if(y>maxY_L) maxY_L = y;
        }
        else
        {
            if(y<minY_R) minY_R = y;
            if(y>maxY_R) maxY_R = y;
        }
        if(x<minX) minX = x;
        if(x>maxX) maxX = x;

        m_series[key]->append(QPointF(x, y));
    }
}

void DataGraph::addDataPoints(QList<QPointF> pointList, int key)
{
    if(m_series.contains(key))
    {
        QListIterator<QPointF> i(pointList);
        while (i.hasNext())
        {
            QPointF p = i.next();

            if( m_axis[key] == left)
            {
                if(p.y()<minY_L) minY_L = p.y();
                if(p.y()>maxY_L) maxY_L = p.y();
            }
            else
            {
                if(p.y()<minY_R) minY_R = p.y();
                if(p.y()>maxY_R) maxY_R = p.y();
            }
            if(p.x()<minX) minX = p.x();
            if(p.x()>maxX) maxX = p.x();
        }

        m_series[key]->append(pointList);
    }
}

void DataGraph::updateGraph(void)
{
    m_chart->removeAllSeries();

    QMap<int, QList<QPointF> *>::iterator i;
    for (i = m_series.begin(); i != m_series.end(); ++i)
    {
        QLineSeries *series = new QLineSeries(); //chart will take ownership of this and delete when done
        series->append(*i.value());
        m_chart->addSeries(series);
        series->setName(m_legends[i.key()]);
        if(m_colours.contains(i.key())) //if we have a colour then override standard one
            series->setColor(m_colours[i.key()]);
        if(m_opacity.contains(i.key())) //if we have an opacity then override standard one
            series->setOpacity(m_opacity[i.key()]);
        series->attachAxis(m_axisX);
        if(m_axis[i.key()] == left)
            series->attachAxis(m_axisL);
        else
            series->attachAxis(m_axisR);
    }

    auto normalizeRange = [](double& min, double& max, double fallbackMin, double fallbackMax)
    {
        if(!std::isfinite(min) || !std::isfinite(max) || min > max)
        {
            min = fallbackMin;
            max = fallbackMax;
            return;
        }
        if(min == max)
        {
            double delta = (min == 0.0) ? 1.0 : std::abs(min) * 0.1;
            if(delta == 0.0 || !std::isfinite(delta))
                delta = 1.0;
            min -= delta;
            max += delta;
        }
    };

    double xMin = minX, xMax = maxX;
    double yLMin = minY_L, yLMax = maxY_L;
    double yRMin = minY_R, yRMax = maxY_R;

    normalizeRange(xMin, xMax, 0.0, 1.0);
    normalizeRange(yLMin, yLMax, 0.0, 1.0);
    normalizeRange(yRMin, yRMax, 0.0, 1.0);

    m_axisX->setRange(xMin, xMax);
    m_axisL->setRange(yLMin, yLMax);
    m_axisR->setRange(yRMin, yRMax);
}

void DataGraph::updateXaxis(double min, double max)
{
    m_axisX->setRange(min, max);
}

void DataGraph::updateLeftYaxis(double min, double max)
{
    m_axisL->setRange(min, max);
}

void DataGraph::clearData(void)
{
    minY_L =  std::numeric_limits<double>::max();
    maxY_L =  std::numeric_limits<double>::lowest();
    minY_R =  std::numeric_limits<double>::max();
    maxY_R =  std::numeric_limits<double>::lowest();
    minX =  std::numeric_limits<double>::max();
    maxX =  std::numeric_limits<double>::lowest();
    m_chart->removeAllSeries();
    setInfoText(QString());
    QMap<int, QList<QPointF> *>::iterator i;
    for (i = m_series.begin(); i != m_series.end(); ++i)
    {
        i.value()->clear();
    }
}

void DataGraph::setColour(QColor colour, int key)
{
    m_colours[key] = colour;
}

void DataGraph::setOpacity(qreal opacity, int key)
{
    m_opacity[key] = opacity;
}


