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

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QHash>
#include <QVector>
#include <QStringList>
#include "datagraph.h"
#include "idiqgraph.h"
#include "sim/motor_plant.h"
#include "sim/power_module.h"



namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

private:
    struct PowerStagePreset
    {
        QString key;
        QString label;
        bool has_parallel_devices_per_switch = false;
        int parallel_devices_per_switch = 1;
        bool has_deadtime_us = false;
        double deadtime_us = 0.0;
        bool has_vge_on_v = false;
        double vge_on_v = 0.0;
        bool has_vge_off_v = false;
        double vge_off_v = 0.0;
        bool has_rg_on_ohm = false;
        double rg_on_ohm = 0.0;
        bool has_rg_off_ohm = false;
        double rg_off_ohm = 0.0;
        bool has_vref_v = false;
        double vref_v = 0.0;
        bool has_kv = false;
        double kv = 0.0;
        bool has_diode_vf_25 = false;
        double diode_vf_25 = 0.0;
        bool has_diode_vf_125 = false;
        double diode_vf_125 = 0.0;
        bool has_rth_jc_igbt = false;
        double rth_jc_igbt = 0.0;
        bool has_rth_jc_diode = false;
        double rth_jc_diode = 0.0;
        bool has_rth_cs = false;
        double rth_cs = 0.0;
        QVector<sim::CurvePoint> vce_points;
        QVector<sim::CurvePoint> eon_points;
        QVector<sim::CurvePoint> eoff_points;
        QVector<sim::CurvePoint> irr_points;
        QVector<sim::CurvePoint> trr_points;
    };

    void runFor(int num_steps);
    void calcFluxLinkage(void);
    void loadPowerStagePresets();
    QString resolvePowerStageYamlPath() const;
    const PowerStagePreset* findPowerStagePreset(const QString& key) const;
    void applyPowerStagePreset(const PowerStagePreset& preset);
    QString currentPowerStagePresetKey() const;

    void loadOpenInverterParamPresets();
    QStringList discoverOpenInverterParamPresetFiles() const;
    void applyOpenInverterParamPresetPath(const QString& jsonPath);
    void applyOpenInverterParamDefaults();
    void syncOpenInverterUiFromParams();
    void enforceSimulationSafeParams(QStringList* warnings);

    DataGraph *motorGraph;
    DataGraph *simulationGraph;
    DataGraph *controllerGraph;
    DataGraph *debugGraph;
    DataGraph *voltageGraph;
    DataGraph *pwmGraph;
    DataGraph *lossGraph;
    IdIqGraph *idigGraph;
    DataGraph *powerGraph;
    sim::MotorPlant *motor;
    double m_time;
    uint32_t m_old_time;
    uint32_t m_old_ms_time;
    double m_oldVa;
    double m_oldVb;
    double m_oldVc;

    double m_wheelSize;
    double m_vehicleWeight;
    double m_gearRatio;
    double m_Lq;
    double m_Ld;
    double m_Rs;
    double m_Poles;
    double m_fluxLinkage;
    double m_syncdelay;
    double m_samplingPoint;
    double m_roadGradient;

    double m_timestep;
    double m_Vdc;

    double m_runTime;
    int m_lastTorqueDemand;
    QVector<PowerStagePreset> m_powerStagePresets;
    QHash<QString, int> m_powerStagePresetByKey;

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_vehicleWeight_editingFinished();

    void on_pbRunFor_clicked();

    void on_pbStep_clicked();

    void on_wheelSize_editingFinished();

    void on_gearRatio_editingFinished();

    void on_Vdc_editingFinished();

    void on_Lq_editingFinished();

    void on_Ld_editingFinished();

    void on_FluxLinkage_editingFinished();

    void on_LoopFreq_editingFinished();

    void on_pbRunFor10s_clicked();

    void on_pbRunFor1s_clicked();

    void on_pbRunFor100ms_clicked();

    void on_pbRunFor10ms_clicked();

    void on_pbRestart_clicked();

    void on_Rs_editingFinished();

    void on_Poles_editingFinished();

    void on_torqueDemand_editingFinished();

    void on_throttleCurrent_editingFinished();

    void on_opMode_editingFinished();

    void on_direction_editingFinished();

    void on_IqManual_editingFinished();

    void on_IdManual_editingFinished();

    void on_CurrentKp_editingFinished();

    void on_CurrentKi_editingFinished();

    void on_SyncAdv_editingFinished();

    void on_LqMinusLd_editingFinished();

    void on_SyncDelay_editingFinished();

    void on_FreqMax_editingFinished();

    void on_SamplingPoint_editingFinished();

    void on_pbTransient_clicked();

    void on_SyncOfs_editingFinished();

    void on_pbAccelCoast_clicked();

    void on_cb_OpPoint_toggled(bool checked);

    void on_cb_Simulation_toggled(bool checked);

    void on_cb_ContVolt_toggled(bool checked);

    void on_cb_ContCurr_toggled(bool checked);

    void on_cb_MotVolt_toggled(bool checked);

    void on_cb_MotCurr_toggled(bool checked);

    void on_cb_PowTorqTime_toggled(bool checked);

    void on_cb_Pwm_toggled(bool checked);
    void on_cb_PwmZeroSeq_toggled(bool checked);
    void on_cb_PwmClamp_toggled(bool checked);
    void on_cb_PwmTiming_toggled(bool checked);
    void on_cb_PwmSector_toggled(bool checked);
    void on_cb_Losses_toggled(bool checked);
    void on_startRpm_editingFinished();
    void on_modBlend_editingFinished();
    void on_modBlendSlider_valueChanged(int value);
    void on_cb_ShowLegends_toggled(bool checked);
    void on_deadtimeUs_editingFinished();
    void on_minOnUs_editingFinished();
    void on_minOffUs_editingFinished();
    void on_sinkTemp_editingFinished();
    void on_thermalTau_editingFinished();

    void on_rb_Speed_toggled(bool checked);

    void on_RoadGradient_editingFinished();

    void on_runTime_editingFinished();

    void on_VLimMargin_editingFinished();

    void on_VLimFlt_editingFinished();

    void on_FWCurrMax_editingFinished();

    void on_rb_OP_Amps_toggled(bool checked);
    void on_powerStagePreset_currentIndexChanged(int index);

    void on_openInverterPreset_currentIndexChanged(int index);
    void on_browseOpenInverterPreset_clicked();

private:
    Ui::MainWindow *ui;
    void closeEvent(QCloseEvent *bar);

protected:
    bool eventFilter(QObject* obj, QEvent* event) override;
};

#endif // MAINWINDOW_H
