#include "sweep/sweep_runner.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTextStream>

#include "my_math.h"
#include "params.h"
#include "pwmgeneration.h"
#include "strategy/strategy_supervisor.h"

namespace sim
{
namespace
{
static std::vector<double> AxisOrDefault(const SweepAxis& axis, double fallback)
{
    if (!axis.values.empty())
        return axis.values;
    return {fallback};
}

static QString StrategyToString(StrategyId id)
{
    return QString::fromUtf8(StrategyLabel(id));
}

static QString ModeToString(ModulationMode mode)
{
    switch (mode)
    {
        case ModulationMode::SVPWM: return "SVPWM";
        case ModulationMode::DPWMMIN: return "DPWMMIN";
        case ModulationMode::DPWMMAX: return "DPWMMAX";
        case ModulationMode::DPWM0: return "DPWM0";
        case ModulationMode::DPWM1: return "DPWM1";
        case ModulationMode::DPWM2: return "DPWM2";
        case ModulationMode::DPWM3: return "DPWM3";
        case ModulationMode::Firmware: return "Firmware";
        default: return "Unknown";
    }
}

static ModulationMode StrategyToMode(StrategyId id)
{
    switch (id)
    {
        case StrategyId::SVPWM: return ModulationMode::SVPWM;
        case StrategyId::DPWMMIN: return ModulationMode::DPWMMIN;
        case StrategyId::DPWMMAX: return ModulationMode::DPWMMAX;
        case StrategyId::DPWM0: return ModulationMode::DPWM0;
        case StrategyId::DPWM1: return ModulationMode::DPWM1;
        case StrategyId::DPWM2: return ModulationMode::DPWM2;
        case StrategyId::DPWM3: return ModulationMode::DPWM3;
        case StrategyId::Firmware: return ModulationMode::Firmware;
        default: return ModulationMode::SVPWM;
    }
}

static const std::vector<ModulationMode>& AutoModes()
{
    static const std::vector<ModulationMode> modes{
        ModulationMode::SVPWM,
        ModulationMode::DPWMMIN,
        ModulationMode::DPWMMAX,
        ModulationMode::DPWM0,
        ModulationMode::DPWM1,
        ModulationMode::DPWM2,
        ModulationMode::DPWM3,
        ModulationMode::Firmware
    };
    return modes;
}

static int ModeIndex(ModulationMode mode)
{
    const auto& modes = AutoModes();
    for (size_t i = 0; i < modes.size(); ++i)
    {
        if (modes[i] == mode)
            return static_cast<int>(i);
    }
    return -1;
}

static bool EnsureDir(const QString& path)
{
    if (path.isEmpty())
        return false;
    QDir dir(path);
    if (dir.exists())
        return true;
    return dir.mkpath(".");
}

static QJsonObject MetricPackToJson(const MetricPack& m)
{
    QJsonObject obj;
    obj.insert("phi_pf_deg", m.phi_pf_deg);
    obj.insert("phi_idiq_deg", m.phi_idiq_deg);
    obj.insert("thd_phase_a_pct", m.thd_phase_a_pct);
    obj.insert("i_ripple_rms", m.i_ripple_rms);
    obj.insert("min_pulse_margin_s", m.min_pulse_margin_s);
    obj.insert("clamp_fraction_a", m.clamp_fraction[0]);
    obj.insert("clamp_fraction_b", m.clamp_fraction[1]);
    obj.insert("clamp_fraction_c", m.clamp_fraction[2]);
    obj.insert("clamp_fraction_high_a", m.clamp_fraction_high[0]);
    obj.insert("clamp_fraction_high_b", m.clamp_fraction_high[1]);
    obj.insert("clamp_fraction_high_c", m.clamp_fraction_high[2]);
    obj.insert("clamp_fraction_low_a", m.clamp_fraction_low[0]);
    obj.insert("clamp_fraction_low_b", m.clamp_fraction_low[1]);
    obj.insert("clamp_fraction_low_c", m.clamp_fraction_low[2]);
    return obj;
}

static double WrapDeg(double deg)
{
    while (deg > 180.0)
        deg -= 360.0;
    while (deg < -180.0)
        deg += 360.0;
    return deg;
}

static void AlphaBetaFromPhase(double a, double b, double& alpha, double& beta)
{
    constexpr double kSqrt3 = 1.7320508075688772;
    alpha = a;
    beta = (a + 2.0 * b) / kSqrt3;
}

static double PhiPfDeg(const PhaseVoltages& v, const PhaseCurrents& i)
{
    constexpr double kPi = 3.14159265358979323846;
    double v_alpha = 0.0;
    double v_beta = 0.0;
    double i_alpha = 0.0;
    double i_beta = 0.0;
    AlphaBetaFromPhase(v.a, v.b, v_alpha, v_beta);
    AlphaBetaFromPhase(i.a, i.b, i_alpha, i_beta);
    const double p_inst = 1.5 * ((v_alpha * i_alpha) + (v_beta * i_beta));
    const double q_inst = 1.5 * ((v_beta * i_alpha) - (v_alpha * i_beta));
    return WrapDeg(std::atan2(q_inst, p_inst) * (180.0 / kPi));
}

static bool CheckMaxThreshold(double value, double max_value)
{
    if (max_value < 0.0)
        return true;
    return std::isfinite(value) && value <= max_value;
}

static bool CheckMinThreshold(double value, double min_value)
{
    return std::isfinite(value) && value >= min_value;
}

struct AutoSummary
{
    std::vector<int> mode_steps;
    std::vector<double> mode_frac;
    int switch_count = 0;
    ModulationMode primary_mode = ModulationMode::SVPWM;
    double primary_frac = 0.0;
    double mean_predicted_saving_w = std::numeric_limits<double>::quiet_NaN();
    double min_predicted_saving_w = std::numeric_limits<double>::quiet_NaN();
    int predicted_samples = 0;
};

static AutoSummary BuildAutoSummary(const std::vector<int>& mode_steps,
                                    int switch_count,
                                    double pred_sum_w,
                                    double pred_min_w,
                                    int pred_samples)
{
    AutoSummary out{};
    out.mode_steps = mode_steps;
    out.mode_frac.resize(mode_steps.size(), 0.0);
    out.switch_count = switch_count;

    int total = 0;
    for (int count : mode_steps)
        total += count;

    if (total > 0)
    {
        const double inv = 1.0 / static_cast<double>(total);
        int best_idx = 0;
        double best_frac = -1.0;
        for (size_t i = 0; i < mode_steps.size(); ++i)
        {
            const double frac = mode_steps[i] * inv;
            out.mode_frac[i] = frac;
            if (frac > best_frac)
            {
                best_frac = frac;
                best_idx = static_cast<int>(i);
            }
        }
        out.primary_mode = AutoModes()[static_cast<size_t>(best_idx)];
        out.primary_frac = best_frac;
    }

    out.predicted_samples = pred_samples;
    if (pred_samples > 0)
    {
        out.mean_predicted_saving_w = pred_sum_w / static_cast<double>(pred_samples);
        out.min_predicted_saving_w = pred_min_w;
    }
    return out;
}

static double AutoFrac(const AutoSummary& summary, ModulationMode mode)
{
    if (summary.mode_frac.empty())
        return 0.0;
    const int idx = ModeIndex(mode);
    if (idx < 0 || static_cast<size_t>(idx) >= summary.mode_frac.size())
        return 0.0;
    return summary.mode_frac[static_cast<size_t>(idx)];
}
} // namespace

std::vector<SweepPoint> BuildOperatingMapPoints(const SweepConfig& cfg)
{
    std::vector<SweepPoint> points;
    const auto speeds = AxisOrDefault(cfg.speed_rpm, 0.0);
    const auto iqs = AxisOrDefault(cfg.iq_A, 0.0);
    const auto ids = AxisOrDefault(cfg.id_A, 0.0);
    const auto vdcs = AxisOrDefault(cfg.vdc_V, 0.0);
    const auto temps = AxisOrDefault(cfg.temp_C, 25.0);
    const auto fsws = AxisOrDefault(cfg.f_sw_Hz, 0.0);

    for (double speed : speeds)
    {
        for (double iq : iqs)
        {
            for (double id : ids)
            {
                for (double vdc : vdcs)
                {
                    for (double temp : temps)
                    {
                        for (double fsw : fsws)
                        {
                            SweepPoint point{};
                            point.speed_rpm = speed;
                            point.iq_A = iq;
                            point.id_A = id;
                            point.vdc_V = vdc;
                            point.temp_C = temp;
                            point.f_sw_Hz = fsw;
                            points.push_back(point);
                        }
                    }
                }
            }
        }
    }
    return points;
}

std::vector<SweepPoint> BuildFixedPointFreqPoints(const SweepConfig& cfg)
{
    std::vector<SweepPoint> points;
    const double speed = cfg.speed_rpm.values.empty() ? 0.0 : cfg.speed_rpm.values.front();
    const double iq = cfg.iq_A.values.empty() ? 0.0 : cfg.iq_A.values.front();
    const double id = cfg.id_A.values.empty() ? 0.0 : cfg.id_A.values.front();
    const double vdc = cfg.vdc_V.values.empty() ? 0.0 : cfg.vdc_V.values.front();
    const double temp = cfg.temp_C.values.empty() ? 25.0 : cfg.temp_C.values.front();
    const auto fsws = AxisOrDefault(cfg.f_sw_Hz, 0.0);

    for (double fsw : fsws)
    {
        SweepPoint point{};
        point.speed_rpm = speed;
        point.iq_A = iq;
        point.id_A = id;
        point.vdc_V = vdc;
        point.temp_C = temp;
        point.f_sw_Hz = fsw;
        points.push_back(point);
    }
    return points;
}

bool ResolveRunSteps(const SweepConfig& cfg, double timestep_s, double elec_freq_hz, int* settle_steps, int* measure_steps)
{
    if (timestep_s <= 0.0)
        return false;

    const double settle_time_s = (cfg.settle_ms > 0.0) ? (cfg.settle_ms / 1000.0)
                                                       : ((cfg.settle_cycles > 0.0 && elec_freq_hz > 0.0)
                                                              ? (cfg.settle_cycles / elec_freq_hz)
                                                              : 0.0);
    const double measure_time_s = (cfg.measure_ms > 0.0) ? (cfg.measure_ms / 1000.0)
                                                         : ((cfg.measure_cycles > 0.0 && elec_freq_hz > 0.0)
                                                                ? (cfg.measure_cycles / elec_freq_hz)
                                                                : 0.0);

    if (settle_steps)
        *settle_steps = static_cast<int>(std::max(0.0, std::round(settle_time_s / timestep_s)));
    if (measure_steps)
        *measure_steps = static_cast<int>(std::max(0.0, std::round(measure_time_s / timestep_s)));
    return true;
}

bool RunSweep(const SweepConfig& cfg, const SweepContext& ctx, SweepResult* out, QString* error,
              const SweepCallbacks* callbacks)
{
    if (!ctx.runner || !ctx.motor)
    {
        if (error)
            *error = "SweepContext missing runner or motor";
        return false;
    }

    if (out)
        out->points.clear();

    const std::vector<SweepPoint> points = (cfg.mode == SweepMode::FixedPointFreqSweep)
                                               ? BuildFixedPointFreqPoints(cfg)
                                               : BuildOperatingMapPoints(cfg);

    std::vector<StrategyId> strategies = cfg.candidates;
    if (strategies.empty())
        strategies.push_back(cfg.baseline);
    else if (std::find(strategies.begin(), strategies.end(), cfg.baseline) == strategies.end())
        strategies.push_back(cfg.baseline);

    std::vector<StrategyId> auto_pred_candidates;
    auto_pred_candidates.reserve(strategies.size());
    for (StrategyId id : strategies)
    {
        if (!StrategyIsAuto(id))
            auto_pred_candidates.push_back(id);
    }
    if (auto_pred_candidates.empty())
    {
        if (!StrategyIsAuto(cfg.baseline))
            auto_pred_candidates.push_back(cfg.baseline);
        else
            auto_pred_candidates.push_back(StrategyId::SVPWM);
    }

    const int poles = cfg.pole_pairs > 0 ? cfg.pole_pairs : ctx.base_inputs.inv_params.pole_pairs;
    QString outputDir = cfg.output_dir;
    if (outputDir.isEmpty())
        outputDir = QDir::currentPath() + "/sweep_out";
    if ((cfg.write_point_json || cfg.write_summary_csv) && !EnsureDir(outputDir))
    {
        if (error)
            *error = "Failed to create output directory";
        return false;
    }

    QDir outDir(outputDir);
    QFile csvFile;
    QTextStream csvStream;
    if (cfg.write_summary_csv)
    {
        csvFile.setFileName(outDir.filePath("summary.csv"));
        if (!csvFile.open(QIODevice::WriteOnly | QIODevice::Text))
        {
            if (error)
                *error = "Failed to open summary.csv for write";
            return false;
        }
        csvStream.setDevice(&csvFile);
        csvStream << "speed_rpm,iq_A,id_A,vdc_V,temp_C,f_sw_Hz,mode,"
                     "phi_pf_deg,phi_idiq_deg,thd_a_pct,i_ripple_rms,min_pulse_margin_s,"
                     "min_pulse_s_effective,"
                     "clamp_frac_a,clamp_frac_b,clamp_frac_c,"
                     "constraint_ok,constraint_thd_ok,constraint_ripple_ok,constraint_pulse_ok,"
                     "thd_max_pct,i_ripple_rms_max_a,min_pulse_margin_min_s,"
                     "auto_switch_count,auto_primary_mode,auto_primary_frac,auto_svpwm_frac,auto_dpwm1_frac,auto_dpwmmax_frac,"
                     "avg_igbt_cond_w,avg_diode_cond_w,avg_igbt_sw_w,avg_diode_rr_w,avg_total_w,avg_inv_eff_pct,"
                     "samples\n";
    }

    const int pointCount = static_cast<int>(points.size());
    const int strategyCount = static_cast<int>(strategies.size());
    int pointIndex = 0;
    for (const SweepPoint& point : points)
    {
        int strategyIndex = 0;
        for (StrategyId strategy : strategies)
        {
            if (callbacks && callbacks->should_abort && callbacks->should_abort())
            {
                if (error)
                    *error = "Cancelled";
                return false;
            }

            SimInputs inputs = ctx.base_inputs;
            inputs.vdc_V = (point.vdc_V > 0.0) ? point.vdc_V : inputs.vdc_V;
            inputs.mod_mode = StrategyToMode(strategy);
            inputs.operating_mode = MotorModel::OperatingMode::ClampedSpeed;
            inputs.clamped_speed_rpm = point.speed_rpm;
            inputs.inv_params.sink_temp_C = (point.temp_C > 0.0) ? point.temp_C : inputs.inv_params.sink_temp_C;
            if (point.f_sw_Hz > 0.0)
                inputs.inv_params.pwm_frequency_hz = point.f_sw_Hz;
            if (cfg.min_pulse_s >= 0.0)
            {
                inputs.inv_params.min_on_s = cfg.min_pulse_s;
                inputs.inv_params.min_off_s = cfg.min_pulse_s;
            }
            const double min_pulse_s_effective = inputs.inv_params.min_on_s;

            const double elec_freq_hz = (poles > 0) ? (std::abs(point.speed_rpm) / 60.0) * poles : 0.0;
            int settle_steps = 0;
            int measure_steps = 0;
            if (!ResolveRunSteps(cfg, inputs.timestep_s, elec_freq_hz, &settle_steps, &measure_steps))
            {
                if (error)
                    *error = "Invalid timestep or run spec";
                return false;
            }
            if (measure_steps <= 0)
                measure_steps = 1;

            Param::Set(Param::manualiq, FP_FROMFLT(static_cast<float>(point.iq_A)));
            Param::Set(Param::manualid, FP_FROMFLT(static_cast<float>(point.id_A)));
            PwmGeneration::SetOpmode(0);
            PwmGeneration::SetOpmode(2);

            ctx.motor->Restart();
            SimInit init{};
            init.motor = ctx.motor;
            ctx.runner->Reset(init);

            MetricConfig metric_config{};
            metric_config.thd.mode = cfg.thd_mode;
            metric_config.thd.samples = std::max(8, std::min(cfg.thd_samples, measure_steps));
            metric_config.thd.fundamental_hz = elec_freq_hz;
            MetricAccumulator accumulator(metric_config, inputs.inv_params, inputs.timestep_s);
            double sum_igbt_cond = 0.0;
            double sum_diode_cond = 0.0;
            double sum_igbt_sw = 0.0;
            double sum_diode_rr = 0.0;
            double sum_total = 0.0;
            double sum_inv_eff = 0.0;
            int loss_samples = 0;

            // Minimal debug instrumentation for sanity checks (measure window only).
            struct DebugAcc
            {
                double sum_abs_i_a = 0.0;
                double sum_abs_i_b = 0.0;
                double sum_abs_i_c = 0.0;
                double sum_id = 0.0;
                double sum_iq = 0.0;
                double sum_elec_power_w = 0.0;
                int samples = 0;
            } dbg;
            bool dbg_bad_loss = false;
            int dbg_bad_step = -1;

            StrategySupervisor supervisor;
            ModulationMode current_mode = inputs.mod_mode;
            if (StrategyIsAuto(strategy))
            {
                StrategyId initial = cfg.baseline;
                if (StrategyIsAuto(initial))
                    initial = StrategyId::SVPWM;
                supervisor.Reset(initial);
                current_mode = StrategyToMode(supervisor.current());
                inputs.mod_mode_fn = [&current_mode]() { return current_mode; };
                inputs.mod_mode = current_mode;
            }

            std::vector<int> auto_mode_steps(AutoModes().size(), 0);
            int auto_switch_count = 0;
            int auto_steps = 0;
            bool auto_last_mode_valid = false;
            ModulationMode auto_last_mode = current_mode;
            double pred_sum_w = 0.0;
            double pred_min_w = std::numeric_limits<double>::infinity();
            int pred_samples = 0;
            int progress_stride = std::max(1, measure_steps / 20);
            int measure_step_index = 0;

            RunHooks hooks;
            hooks.on_step = [&](const StepSnapshot& snap)
            {
                const ModulationMode mode_used = current_mode;
                if (StrategyIsAuto(strategy))
                {
                    const int idx = ModeIndex(mode_used);
                    if (idx >= 0)
                        ++auto_mode_steps[static_cast<size_t>(idx)];
                    if (auto_last_mode_valid && mode_used != auto_last_mode)
                        ++auto_switch_count;
                    auto_last_mode = mode_used;
                    auto_last_mode_valid = true;
                    ++auto_steps;
                }

                if (StrategyIsAuto(strategy))
                {
                    SupervisorInputs supIn{};
                    supIn.time_s = snap.time_s;
                    supIn.phi_pf_deg = PhiPfDeg(snap.voltages_ln, snap.phase_currents);
                    supIn.phi_idiq_deg = WrapDeg(std::atan2(snap.motor.id, snap.motor.iq) * (180.0 / 3.14159265358979323846));
                    supIn.has_phi_pf = std::isfinite(supIn.phi_pf_deg);
                    const double theta_rad = snap.motor.elec_pos_deg * (3.14159265358979323846 / 180.0);
                    supIn.v_alpha = (snap.controller.vd_ctrl * std::cos(theta_rad)) -
                                    (snap.controller.vq_ctrl * std::sin(theta_rad));
                    supIn.v_beta = (snap.controller.vd_ctrl * std::sin(theta_rad)) +
                                   (snap.controller.vq_ctrl * std::cos(theta_rad));
                    supIn.vdc_V = snap.vdc_V;
                    supIn.currents = snap.phase_currents;
                    supIn.inv_params = inputs.inv_params;
                    supIn.module_params = inputs.module_params;
                    supIn.mod_blend = inputs.mod_blend;
                    supIn.sector = snap.mod_diag.sector;

                    StrategyDecision decision{};
                    if (strategy == StrategyId::AUTO_RULE)
                        decision = supervisor.SelectAutoRule(supIn);
                    else
                        decision = supervisor.SelectAutoPred(supIn, auto_pred_candidates);

                    current_mode = StrategyToMode(decision.selected);
                    if (strategy == StrategyId::AUTO_PRED && std::isfinite(decision.benefit_w))
                    {
                        pred_sum_w += decision.benefit_w;
                        pred_min_w = std::min(pred_min_w, decision.benefit_w);
                        ++pred_samples;
                    }
                }

                accumulator.OnStep(snap);
                if (snap.pwm_enabled)
                {
                    const double igbt_cond = snap.inv_loss.phase[0].igbt_cond_W +
                                             snap.inv_loss.phase[1].igbt_cond_W +
                                             snap.inv_loss.phase[2].igbt_cond_W;
                    const double diode_cond = snap.inv_loss.phase[0].diode_cond_W +
                                              snap.inv_loss.phase[1].diode_cond_W +
                                              snap.inv_loss.phase[2].diode_cond_W;
                    const double igbt_sw = snap.inv_loss.phase[0].igbt_sw_W +
                                           snap.inv_loss.phase[1].igbt_sw_W +
                                           snap.inv_loss.phase[2].igbt_sw_W;
                    const double diode_rr = snap.inv_loss.phase[0].diode_rr_W +
                                            snap.inv_loss.phase[1].diode_rr_W +
                                            snap.inv_loss.phase[2].diode_rr_W;
                    const double total = igbt_cond + diode_cond + igbt_sw + diode_rr;
                    if (!std::isfinite(total) || std::abs(total) > 1e12)
                    {
                        dbg_bad_loss = true;
                        dbg_bad_step = snap.step_index;
                    }
                    sum_igbt_cond += igbt_cond;
                    sum_diode_cond += diode_cond;
                    sum_igbt_sw += igbt_sw;
                    sum_diode_rr += diode_rr;
                    sum_total += total;
                    dbg.sum_abs_i_a += std::abs(snap.phase_currents.a);
                    dbg.sum_abs_i_b += std::abs(snap.phase_currents.b);
                    dbg.sum_abs_i_c += std::abs(snap.phase_currents.c);
                    dbg.sum_id += snap.motor.id;
                    dbg.sum_iq += snap.motor.iq;
                    dbg.sum_elec_power_w += snap.elec_power_w;
                    ++dbg.samples;
                    if (snap.elec_power_w > 1e-6)
                    {
                        const double inv_eff = 100.0 * (snap.elec_power_w / (snap.elec_power_w + total));
                        sum_inv_eff += inv_eff;
                    }
                    ++loss_samples;
                }

                if (callbacks && callbacks->on_progress)
                {
                    ++measure_step_index;
                    if ((measure_step_index % progress_stride) == 0 || measure_step_index == measure_steps)
                    {
                        SweepProgress progress{};
                        progress.point_index = pointIndex;
                        progress.point_count = pointCount;
                        progress.strategy_index = strategyIndex;
                        progress.strategy_count = strategyCount;
                        progress.point = point;
                        progress.strategy = strategy;
                        progress.step_index = snap.step_index;
                        progress.measure_step_index = measure_step_index;
                        progress.settle_steps = settle_steps;
                        progress.measure_steps = measure_steps;
                        progress.in_measure = true;
                        progress.current_mode = current_mode;
                        progress.total_loss_w = snap.inv_loss.total_W;
                        progress.switching_loss_w = snap.inv_loss.total_sw_W + snap.inv_loss.total_rr_W;
                        progress.auto_switch_count = auto_switch_count;
                        callbacks->on_progress(progress);
                    }
                }
            };
            if (callbacks && callbacks->should_abort)
                hooks.should_abort = callbacks->should_abort;

            RunSpec settle{};
            settle.steps = settle_steps;
            RunSpec measure{};
            measure.steps = measure_steps;
            const RunResult result = ctx.runner->Run(inputs, settle, measure, hooks);
            if (!result.ok)
            {
                if (error)
                    *error = QString::fromStdString(result.error);
                return false;
            }

            SweepPointResult pointResult{};
            pointResult.point = point;
            pointResult.strategy = strategy;
            pointResult.metrics = accumulator.Finalize();
            pointResult.constraint_thd_ok = CheckMaxThreshold(pointResult.metrics.thd_phase_a_pct, cfg.thd_max_pct);
            pointResult.constraint_ripple_ok = CheckMaxThreshold(pointResult.metrics.i_ripple_rms, cfg.i_ripple_rms_max_a);
            pointResult.constraint_pulse_ok = CheckMinThreshold(pointResult.metrics.min_pulse_margin_s,
                                                               cfg.min_pulse_margin_min_s);
            pointResult.constraint_ok = pointResult.constraint_thd_ok &&
                                        pointResult.constraint_ripple_ok &&
                                        pointResult.constraint_pulse_ok;

            AutoSummary auto_summary{};
            if (StrategyIsAuto(strategy) && auto_steps > 0)
            {
                auto_summary = BuildAutoSummary(auto_mode_steps, auto_switch_count,
                                                pred_sum_w, pred_min_w, pred_samples);
            }
            if (loss_samples > 0)
            {
                const double inv = 1.0 / static_cast<double>(loss_samples);
                pointResult.avg_igbt_cond_w = sum_igbt_cond * inv;
                pointResult.avg_diode_cond_w = sum_diode_cond * inv;
                pointResult.avg_igbt_sw_w = sum_igbt_sw * inv;
                pointResult.avg_diode_rr_w = sum_diode_rr * inv;
                pointResult.avg_total_w = sum_total * inv;
                pointResult.avg_inv_eff_pct = (sum_inv_eff > 0.0) ? (sum_inv_eff * inv) : 0.0;
                pointResult.samples = loss_samples;
            }

            // Physics sanity guard: huge/invalid loss values usually indicate a unit/step bug or instability.
            // Fail early with debug context instead of generating nonsense ROI numbers.
            if (dbg_bad_loss || !std::isfinite(pointResult.avg_total_w) || pointResult.avg_total_w > 1e6)
            {
                const double dt_s = inputs.timestep_s;
                const double loop_freq_hz = dt_s > 0.0 ? (1.0 / dt_s) : 0.0;
                const double pwm_freq_hz = inputs.inv_params.pwm_frequency_hz;
                const double meas_time_s = dt_s * static_cast<double>(measure_steps);
                const double inv_dbg = (dbg.samples > 0) ? (1.0 / static_cast<double>(dbg.samples)) : 0.0;
                const double mean_abs_ia = dbg.sum_abs_i_a * inv_dbg;
                const double mean_abs_ib = dbg.sum_abs_i_b * inv_dbg;
                const double mean_abs_ic = dbg.sum_abs_i_c * inv_dbg;
                const double mean_id = dbg.sum_id * inv_dbg;
                const double mean_iq = dbg.sum_iq * inv_dbg;
                const double mean_elec_p = dbg.sum_elec_power_w * inv_dbg;

                if (error)
                {
                    *error = QString("Sanity guard: avg_total_w=%1 W (samples=%2) exceeded limit or invalid.\n"
                                     "Debug (measure window): loop_freq_hz=%3, pwm_freq_hz=%4, dt_s=%5, settle_steps=%6, measure_steps=%7, meas_time_s=%8.\n"
                                     "Currents: mean(|ia|,|ib|,|ic|)=(%9,%10,%11) A; mean(id,iq)=(%12,%13) A.\n"
                                     "Loss components: avg_igbt_cond=%14 W, avg_diode_cond=%15 W, avg_igbt_sw=%16 W, avg_diode_rr=%17 W.\n"
                                     "Mean electrical power estimate=%18 W.\n"
                                     "First bad step=%19")
                                 .arg(pointResult.avg_total_w, 0, 'g', 6)
                                 .arg(pointResult.samples)
                                 .arg(loop_freq_hz, 0, 'g', 8)
                                 .arg(pwm_freq_hz, 0, 'g', 8)
                                 .arg(dt_s, 0, 'g', 8)
                                 .arg(settle_steps)
                                 .arg(measure_steps)
                                 .arg(meas_time_s, 0, 'g', 8)
                                 .arg(mean_abs_ia, 0, 'g', 6)
                                 .arg(mean_abs_ib, 0, 'g', 6)
                                 .arg(mean_abs_ic, 0, 'g', 6)
                                 .arg(mean_id, 0, 'g', 6)
                                 .arg(mean_iq, 0, 'g', 6)
                                 .arg(pointResult.avg_igbt_cond_w, 0, 'g', 6)
                                 .arg(pointResult.avg_diode_cond_w, 0, 'g', 6)
                                 .arg(pointResult.avg_igbt_sw_w, 0, 'g', 6)
                                 .arg(pointResult.avg_diode_rr_w, 0, 'g', 6)
                                 .arg(mean_elec_p, 0, 'g', 6)
                                 .arg(dbg_bad_step);
                }
                return false;
            }
            if (out)
                out->points.push_back(pointResult);

            if (callbacks && callbacks->on_progress)
            {
                SweepProgress progress{};
                progress.point_index = pointIndex;
                progress.point_count = pointCount;
                progress.strategy_index = strategyIndex;
                progress.strategy_count = strategyCount;
                progress.point = point;
                progress.strategy = strategy;
                progress.settle_steps = settle_steps;
                progress.measure_steps = measure_steps;
                progress.measure_step_index = measure_steps;
                progress.in_measure = false;
                progress.current_mode = current_mode;
                progress.total_loss_w = pointResult.avg_total_w;
                progress.switching_loss_w = pointResult.avg_igbt_sw_w + pointResult.avg_diode_rr_w;
                progress.thd_proxy_pct = pointResult.metrics.thd_phase_a_pct;
                progress.min_pulse_margin_s = pointResult.metrics.min_pulse_margin_s;
                progress.auto_switch_count = auto_summary.switch_count;
                progress.auto_primary_mode = auto_summary.primary_mode;
                progress.auto_primary_frac = auto_summary.primary_frac;
                callbacks->on_progress(progress);
            }

            if (cfg.write_summary_csv)
            {
                const double auto_svpwm_frac = (StrategyIsAuto(strategy) && auto_steps > 0)
                                                   ? AutoFrac(auto_summary, ModulationMode::SVPWM)
                                                   : 0.0;
                const double auto_dpwm1_frac = (StrategyIsAuto(strategy) && auto_steps > 0)
                                                   ? AutoFrac(auto_summary, ModulationMode::DPWM1)
                                                   : 0.0;
                const double auto_dpwmmax_frac = (StrategyIsAuto(strategy) && auto_steps > 0)
                                                     ? AutoFrac(auto_summary, ModulationMode::DPWMMAX)
                                                     : 0.0;

                csvStream << point.speed_rpm << "," << point.iq_A << "," << point.id_A << ","
                          << point.vdc_V << "," << point.temp_C << "," << point.f_sw_Hz << ","
                          << StrategyToString(strategy) << ","
                          << pointResult.metrics.phi_pf_deg << ","
                          << pointResult.metrics.phi_idiq_deg << ","
                          << pointResult.metrics.thd_phase_a_pct << ","
                          << pointResult.metrics.i_ripple_rms << ","
                          << pointResult.metrics.min_pulse_margin_s << ","
                          << min_pulse_s_effective << ","
                          << pointResult.metrics.clamp_fraction[0] << ","
                          << pointResult.metrics.clamp_fraction[1] << ","
                          << pointResult.metrics.clamp_fraction[2] << ","
                          << (pointResult.constraint_ok ? 1 : 0) << ","
                          << (pointResult.constraint_thd_ok ? 1 : 0) << ","
                          << (pointResult.constraint_ripple_ok ? 1 : 0) << ","
                          << (pointResult.constraint_pulse_ok ? 1 : 0) << ","
                          << cfg.thd_max_pct << ","
                          << cfg.i_ripple_rms_max_a << ","
                          << cfg.min_pulse_margin_min_s << ","
                          << (StrategyIsAuto(strategy) ? auto_summary.switch_count : 0) << ","
                          << (StrategyIsAuto(strategy) ? ModeToString(auto_summary.primary_mode) : QString()) << ","
                          << (StrategyIsAuto(strategy) ? auto_summary.primary_frac : 0.0) << ","
                          << auto_svpwm_frac << ","
                          << auto_dpwm1_frac << ","
                          << auto_dpwmmax_frac << ","
                          << pointResult.avg_igbt_cond_w << ","
                          << pointResult.avg_diode_cond_w << ","
                          << pointResult.avg_igbt_sw_w << ","
                          << pointResult.avg_diode_rr_w << ","
                          << pointResult.avg_total_w << ","
                          << pointResult.avg_inv_eff_pct << ","
                          << pointResult.samples << "\n";
            }

            if (cfg.write_point_json)
            {
                QJsonObject root;
                QJsonObject pointObj;
                pointObj.insert("speed_rpm", point.speed_rpm);
                pointObj.insert("iq_A", point.iq_A);
                pointObj.insert("id_A", point.id_A);
                pointObj.insert("vdc_V", point.vdc_V);
                pointObj.insert("temp_C", point.temp_C);
                pointObj.insert("f_sw_Hz", point.f_sw_Hz);
                root.insert("point", pointObj);
                root.insert("mode", StrategyToString(strategy));
                root.insert("metrics", MetricPackToJson(pointResult.metrics));
                root.insert("constraint_ok", pointResult.constraint_ok);
                root.insert("constraint_thd_ok", pointResult.constraint_thd_ok);
                root.insert("constraint_ripple_ok", pointResult.constraint_ripple_ok);
                root.insert("constraint_pulse_ok", pointResult.constraint_pulse_ok);
                QJsonObject constraintsObj;
                constraintsObj.insert("thd_max_pct", cfg.thd_max_pct);
                constraintsObj.insert("i_ripple_rms_max_a", cfg.i_ripple_rms_max_a);
                constraintsObj.insert("min_pulse_margin_min_s", cfg.min_pulse_margin_min_s);
                root.insert("constraints", constraintsObj);
                root.insert("min_pulse_s_effective", min_pulse_s_effective);
                if (StrategyIsAuto(strategy))
                {
                    QJsonObject autoObj;
                    autoObj.insert("switch_count", auto_summary.switch_count);
                    autoObj.insert("primary_mode", ModeToString(auto_summary.primary_mode));
                    autoObj.insert("primary_frac", auto_summary.primary_frac);
                    QJsonObject stepsObj;
                    QJsonObject fracObj;
                    const auto& modes = AutoModes();
                    for (size_t i = 0; i < modes.size(); ++i)
                    {
                        if (auto_summary.mode_steps[i] <= 0)
                            continue;
                        const QString key = ModeToString(modes[i]);
                        stepsObj.insert(key, auto_summary.mode_steps[i]);
                        fracObj.insert(key, auto_summary.mode_frac[i]);
                    }
                    autoObj.insert("mode_steps", stepsObj);
                    autoObj.insert("mode_frac", fracObj);
                    if (strategy == StrategyId::AUTO_PRED && auto_summary.predicted_samples > 0)
                    {
                        autoObj.insert("mean_predicted_saving_w", auto_summary.mean_predicted_saving_w);
                        autoObj.insert("min_predicted_saving_w", auto_summary.min_predicted_saving_w);
                    }
                    root.insert("auto", autoObj);
                }
                root.insert("avg_igbt_cond_w", pointResult.avg_igbt_cond_w);
                root.insert("avg_diode_cond_w", pointResult.avg_diode_cond_w);
                root.insert("avg_igbt_sw_w", pointResult.avg_igbt_sw_w);
                root.insert("avg_diode_rr_w", pointResult.avg_diode_rr_w);
                root.insert("avg_total_w", pointResult.avg_total_w);
                root.insert("avg_inv_eff_pct", pointResult.avg_inv_eff_pct);
                root.insert("samples", pointResult.samples);

                if (pointIndex == 0 && strategyIndex == 0)
                {
                    const double dt_s = inputs.timestep_s;
                    const double loop_freq_hz = dt_s > 0.0 ? (1.0 / dt_s) : 0.0;
                    const double inv_dbg = (dbg.samples > 0) ? (1.0 / static_cast<double>(dbg.samples)) : 0.0;
                    QJsonObject dbgObj;
                    dbgObj.insert("loop_freq_hz", loop_freq_hz);
                    dbgObj.insert("pwm_freq_hz", inputs.inv_params.pwm_frequency_hz);
                    dbgObj.insert("dt_s", dt_s);
                    dbgObj.insert("settle_steps", settle_steps);
                    dbgObj.insert("measure_steps", measure_steps);
                    dbgObj.insert("total_meas_time_s", dt_s * static_cast<double>(measure_steps));
                    dbgObj.insert("mean_abs_ia_A", dbg.sum_abs_i_a * inv_dbg);
                    dbgObj.insert("mean_abs_ib_A", dbg.sum_abs_i_b * inv_dbg);
                    dbgObj.insert("mean_abs_ic_A", dbg.sum_abs_i_c * inv_dbg);
                    dbgObj.insert("mean_id_A", dbg.sum_id * inv_dbg);
                    dbgObj.insert("mean_iq_A", dbg.sum_iq * inv_dbg);
                    dbgObj.insert("mean_elec_power_w", dbg.sum_elec_power_w * inv_dbg);
                    dbgObj.insert("avg_igbt_cond_w", pointResult.avg_igbt_cond_w);
                    dbgObj.insert("avg_diode_cond_w", pointResult.avg_diode_cond_w);
                    dbgObj.insert("avg_igbt_sw_w", pointResult.avg_igbt_sw_w);
                    dbgObj.insert("avg_diode_rr_w", pointResult.avg_diode_rr_w);
                    dbgObj.insert("avg_total_w", pointResult.avg_total_w);
                    root.insert("debug", dbgObj);
                }

                const QString filename = QString("point_%1_%2.json")
                                             .arg(pointIndex, 6, 10, QChar('0'))
                                             .arg(StrategyToString(strategy));
                QFile file(outDir.filePath(filename));
                if (file.open(QIODevice::WriteOnly | QIODevice::Text))
                {
                    const QJsonDocument doc(root);
                    file.write(doc.toJson(QJsonDocument::Indented));
                }
            }
            ++strategyIndex;
        }
        ++pointIndex;
    }

    return true;
}
} // namespace sim
