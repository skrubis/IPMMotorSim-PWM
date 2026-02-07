#ifndef SIM_RUNNER_H
#define SIM_RUNNER_H

#include <cstdint>
#include <functional>
#include <string>

#include "controller.h"
#include "inverter_switching_model.h"
#include "modulator.h"
#include "motor_plant.h"

namespace sim
{
struct SimInit
{
    MotorPlant* motor = nullptr;
    double time_s = 0.0;
    double old_va = 0.0;
    double old_vb = 0.0;
    double old_vc = 0.0;
    uint32_t old_time_10ms = 0;
    uint32_t old_time_ms = 0;
    int last_torque_demand = 0;
};

struct SimState
{
    MotorPlant* motor = nullptr;
    double time_s = 0.0;
    double old_va = 0.0;
    double old_vb = 0.0;
    double old_vc = 0.0;
    uint32_t old_time_10ms = 0;
    uint32_t old_time_ms = 0;
    int last_torque_demand = 0;
};

struct SimInputs
{
    double timestep_s = 0.0;
    double vdc_V = 0.0;
    int torque_demand_pct = 0;
    bool throttle_ramps = false;
    bool add_noise = false;
    double noise_amp = 0.0;
    std::function<double(double)> noise_fn;
    bool extra_cycle_delay = false;
    ModulationMode mod_mode = ModulationMode::Firmware;
    std::function<ModulationMode()> mod_mode_fn;
    double mod_blend = 1.0;
    InverterParams inv_params;
    PowerModuleParams module_params;
    MotorModel::OperatingMode operating_mode = MotorModel::OperatingMode::Dynamic;
    double clamped_speed_rpm = 0.0;

    struct ValidityLimits
    {
        double i_hard_max_A = 2000.0;
        double p_hard_max_W = 2.0e6;
        double v_sat_frac_limit = 0.95;
        double v_sat_frac_pct = 50.0;
    } validity_limits;
    bool check_validity = true;
};

struct RunSpec
{
    int steps = 0;
};

struct MotorSnapshot
{
    double ia_samp = 0.0;
    double ib_samp = 0.0;
    double ic_samp = 0.0;
    double id = 0.0;
    double iq = 0.0;
    double motor_freq_hz = 0.0;
    double motor_pos_deg = 0.0;
    double elec_pos_deg = 0.0;
    double torque_nm = 0.0;
    double power_w = 0.0;
    double vd = 0.0;
    double vq = 0.0;
    double vq_bemf = 0.0;
    double vq_dueto_id = 0.0;
    double vd_dueto_iq = 0.0;
    double vq_dueto_rq = 0.0;
    double vd_dueto_rd = 0.0;
    double vld = 0.0;
    double vlq = 0.0;
};

struct ControllerSnapshot
{
    double id = 0.0;
    double iq = 0.0;
    double ifw = 0.0;
    double vd_ctrl = 0.0;
    double vq_ctrl = 0.0;
};

struct StepSnapshot
{
    int step_index = 0;
    double time_s = 0.0;
    bool pwm_enabled = false;
    double vdc_V = 0.0;
    DutyCycles duty;
    ModulatorDiag mod_diag;
    PhaseVoltages voltages_cmd;
    PhaseVoltages voltages_ln;
    PhaseCurrents phase_currents;
    MotorSnapshot motor;
    ControllerSnapshot controller;
    LossBreakdown inv_loss;
    ThermalState inv_thermal;
    PwmRippleDiag inv_ripple;
    double elec_power_w = 0.0;
};

struct RunHooks
{
    std::function<void(const StepSnapshot&)> on_step;
    std::function<bool()> should_abort;
};

struct RunResult
{
    int steps_total = 0;
    bool ok = true;
    std::string error;

    struct PointValidity
    {
        bool valid = true;
        std::string reason;
        double max_abs_i_abc = 0.0;
        double max_abs_idq = 0.0;
        double max_abs_power_w = 0.0;
        double v_sat_frac_max = 0.0;
        double v_sat_frac_pct = 0.0;
    } validity;
};

class SimRunner
{
public:
    void Reset(const SimInit& init);
    RunResult Run(const SimInputs& inputs, const RunSpec& settle, const RunSpec& measure, const RunHooks& hooks);
    const SimState& state() const;

private:
    StepSnapshot StepOnce(int step_index,
                          const SimInputs& inputs,
                          Controller& controller,
                          Modulator& modulator,
                          InverterSwitchingModel& inverter,
                          InverterParams& inv_params);

    SimState m_state{};
};
} // namespace sim

#endif // SIM_RUNNER_H
