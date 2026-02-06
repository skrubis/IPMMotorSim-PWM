#include <algorithm>
#include <cmath>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>

#include "sim/modulator_diagnostics.h"
#include "sim/pwm_timeline.h"
#include "sim/metrics/metric_pack.h"
#include "sim/motor_plant.h"
#include "sim/power_module.h"
#include "sim/strategy/strategy_supervisor.h"
#include "sim/sweep/sweep_config.h"
#include "sim/sweep/sweep_runner.h"
#include "sim/sim_runner.h"

namespace
{
struct TestFailure : public std::runtime_error
{
    using std::runtime_error::runtime_error;
};

struct TestCase
{
    const char* name;
    void (*fn)();
};

std::vector<TestCase>& Registry()
{
    static std::vector<TestCase> tests;
    return tests;
}

struct TestReg
{
    TestReg(const char* name, void (*fn)())
    {
        Registry().push_back({name, fn});
    }
};

std::string ToString(double value)
{
    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss.precision(6);
    oss << value;
    return oss.str();
}

void Fail(const char* expr, const char* file, int line, const std::string& msg)
{
    std::ostringstream oss;
    oss << file << ":" << line << " ASSERT(" << expr << ") failed";
    if (!msg.empty())
        oss << ": " << msg;
    throw TestFailure(oss.str());
}

#define ASSERT_TRUE(expr) do { if (!(expr)) Fail(#expr, __FILE__, __LINE__, ""); } while (0)
#define ASSERT_NEAR(val, exp, tol) do { \
    const double v = (val); const double e = (exp); const double t = (tol); \
    if (std::abs(v - e) > t) Fail(#val " ~= " #exp, __FILE__, __LINE__, "got=" + ToString(v) + " exp=" + ToString(e)); \
} while (0)
#define ASSERT_EQ_INT(val, exp) do { \
    const auto v = (val); const auto e = (exp); \
    if (v != e) Fail(#val " == " #exp, __FILE__, __LINE__, "got=" + std::to_string(v) + " exp=" + std::to_string(e)); \
} while (0)

#define TEST(name) \
    static void name(); \
    static TestReg reg_##name(#name, &name); \
    static void name()

void AssertSegmentsNear(sim::ModulationMode mode, double expected_deg, double tol_deg)
{
    const auto samples = sim::SampleClampPattern(mode, 0.8, 400.0, 3600);
    const auto stats = sim::ComputeClampStats(samples);
    ASSERT_TRUE(!stats.segment_lengths_deg.empty());

    double sum = 0.0;
    double max_seg = 0.0;
    int good = 0;
    int partial = 0;

    for (double seg : stats.segment_lengths_deg)
    {
        sum += seg;
        max_seg = std::max(max_seg, seg);
        if (std::abs(seg - expected_deg) <= tol_deg)
            ++good;
        else if (seg < expected_deg - tol_deg)
            ++partial; // allow 2 boundary-split segments
        else
            Fail("seg <= expected", __FILE__, __LINE__, "found segment larger than expected");
    }

    ASSERT_NEAR(sum, 360.0, 1.0);
    ASSERT_NEAR(max_seg, expected_deg, tol_deg);
    ASSERT_TRUE(partial <= 2);
    ASSERT_TRUE(good >= 1);
}

int CountSwitchTransitions(const std::vector<sim::TimelineSegment>& timeline)
{
    if (timeline.empty())
        return 0;
    int transitions = 0;
    auto prev = timeline.front().leg;
    for (size_t i = 1; i < timeline.size(); ++i)
    {
        const auto& cur = timeline[i].leg;
        for (size_t leg = 0; leg < 3; ++leg)
        {
            if (cur[leg] != prev[leg])
                ++transitions;
        }
        prev = cur;
    }
    return transitions;
}
} // namespace

TEST(ClampWindowSegments)
{
    AssertSegmentsNear(sim::ModulationMode::DPWM0, 60.0, 1.0);
    AssertSegmentsNear(sim::ModulationMode::DPWM1, 60.0, 1.0);
    AssertSegmentsNear(sim::ModulationMode::DPWM2, 60.0, 1.0);
    AssertSegmentsNear(sim::ModulationMode::DPWM3, 30.0, 1.0);
    // DPWMMIN/MAX clamp polarity is constant.
    AssertSegmentsNear(sim::ModulationMode::DPWMMIN, 360.0, 1.0);
    AssertSegmentsNear(sim::ModulationMode::DPWMMAX, 360.0, 1.0);
}

TEST(SupervisorDwellAndHysteresis)
{
    sim::SupervisorConfig cfg;
    cfg.min_dwell_s = 0.1;
    cfg.hysteresis_deg = 5.0;
    cfg.sector_lock = false;

    sim::StrategySupervisor sup(cfg);
    sup.Reset(sim::StrategyId::DPWM1);

    sim::SupervisorInputs in{};
    in.has_phi_pf = true;
    in.sector = 1;

    in.time_s = 0.0;
    in.phi_pf_deg = 30.0;
    auto d = sup.SelectAutoRule(in);
    ASSERT_EQ_INT(static_cast<int>(d.selected), static_cast<int>(sim::StrategyId::DPWM2));

    in.time_s = 0.05;
    in.phi_pf_deg = -30.0;
    d = sup.SelectAutoRule(in);
    ASSERT_EQ_INT(static_cast<int>(d.selected), static_cast<int>(sim::StrategyId::DPWM2));

    in.time_s = 0.2;
    in.phi_pf_deg = -30.0;
    d = sup.SelectAutoRule(in);
    ASSERT_EQ_INT(static_cast<int>(d.selected), static_cast<int>(sim::StrategyId::DPWM0));

    in.time_s = 0.3;
    in.phi_pf_deg = -10.0;
    d = sup.SelectAutoRule(in);
    ASSERT_EQ_INT(static_cast<int>(d.selected), static_cast<int>(sim::StrategyId::DPWM0));
}

TEST(ThdProxySineWithHarmonic)
{
    sim::MetricConfig cfg;
    cfg.thd.mode = sim::ThdMode::ControlStepProxy;
    cfg.thd.samples = 1000;
    cfg.thd.fundamental_hz = 50.0;
    sim::InverterParams inv{};
    sim::MetricAccumulator acc(cfg, inv, 0.001);

    constexpr double kPi = 3.14159265358979323846;
    for (int i = 0; i < 1000; ++i)
    {
        const double t = i * 0.001;
        const double s1 = std::sin(2.0 * kPi * 50.0 * t);
        const double s5 = 0.2 * std::sin(2.0 * kPi * 250.0 * t);
        sim::StepSnapshot snap{};
        snap.phase_currents.a = s1 + s5;
        acc.OnStep(snap);
    }

    const sim::MetricPack out = acc.Finalize();
    ASSERT_NEAR(out.thd_phase_a_pct, 20.0, 1.0);
}

TEST(ConstraintSchemaParsing)
{
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());

    auto writeJson = [&](const QString& path, const QJsonObject& root)
    {
        QFile file(path);
        ASSERT_TRUE(file.open(QIODevice::WriteOnly | QIODevice::Text));
        file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    };

    const QString canonicalPath = dir.filePath("canonical.json");
    QJsonObject root1;
    root1.insert("schema_version", 1);
    QJsonObject constraints1;
    constraints1.insert("thd_max_pct", 3.0);
    constraints1.insert("i_ripple_rms_max_a", 2.0);
    constraints1.insert("min_pulse_margin_min_s", 1e-6);
    root1.insert("constraints", constraints1);
    writeJson(canonicalPath, root1);

    const QString legacyPath = dir.filePath("legacy.json");
    QJsonObject root2;
    root2.insert("schema_version", 1);
    QJsonObject constraints2;
    constraints2.insert("THD_max", 3.0);
    constraints2.insert("I_ripple_rms_max", 2.0);
    constraints2.insert("min_pulse_margin_s", 1e-6);
    root2.insert("constraints", constraints2);
    writeJson(legacyPath, root2);

    sim::SweepConfig c1, c2;
    QString err;
    ASSERT_TRUE(sim::LoadSweepConfig(canonicalPath, &c1, &err));
    ASSERT_TRUE(sim::LoadSweepConfig(legacyPath, &c2, &err));

    ASSERT_NEAR(c1.thd_max_pct, c2.thd_max_pct, 1e-9);
    ASSERT_NEAR(c1.i_ripple_rms_max_a, c2.i_ripple_rms_max_a, 1e-9);
    ASSERT_NEAR(c1.min_pulse_margin_min_s, c2.min_pulse_margin_min_s, 1e-12);
}

TEST(ConstraintExportCanonicalKeys)
{
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());

    sim::SweepConfig cfg;
    cfg.mode = sim::SweepMode::OperatingMapSweep;
    cfg.speed_rpm.values = {1000.0};
    cfg.iq_A.values = {10.0};
    cfg.id_A.values = {0.0};
    cfg.vdc_V.values = {400.0};
    cfg.temp_C.values = {25.0};
    cfg.f_sw_Hz.values = {8000.0};
    cfg.settle_ms = 0.0;
    cfg.measure_ms = 1.0;
    cfg.pole_pairs = 4;
    cfg.baseline = sim::StrategyId::SVPWM;
    cfg.thd_max_pct = 5.0;
    cfg.i_ripple_rms_max_a = 2.5;
    cfg.min_pulse_margin_min_s = 2e-6;
    cfg.write_point_json = true;
    cfg.write_summary_csv = false;
    cfg.output_dir = dir.path();

    sim::SimRunner runner;
    sim::MotorPlant motor(0.3, 1.0, 0.0, 1000.0,
                          0.001, 0.001, 0.01, 4.0, 0.05,
                          1e-4, 0.0, 0.5);

    sim::SweepContext ctx;
    ctx.runner = &runner;
    ctx.motor = &motor;
    ctx.base_inputs.timestep_s = 1e-4;
    ctx.base_inputs.vdc_V = 400.0;
    ctx.base_inputs.inv_params.pwm_frequency_hz = 8000.0;
    ctx.base_inputs.inv_params.deadtime_s = 2e-6;
    ctx.base_inputs.inv_params.min_on_s = 0.0;
    ctx.base_inputs.inv_params.min_off_s = 0.0;
    ctx.base_inputs.inv_params.pole_pairs = 4.0;
    ctx.base_inputs.inv_params.flux_Wb = 0.05;
    ctx.base_inputs.inv_params.ld_H = 0.001;
    ctx.base_inputs.inv_params.lq_H = 0.001;
    ctx.base_inputs.module_params = sim::PM300CLA060();

    sim::SweepResult out;
    QString err;
    ASSERT_TRUE(sim::RunSweep(cfg, ctx, &out, &err));

    const QString pointPath = QDir(cfg.output_dir).filePath("point_000000_SVPWM.json");
    QFile file(pointPath);
    ASSERT_TRUE(file.open(QIODevice::ReadOnly | QIODevice::Text));
    const QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
    ASSERT_TRUE(doc.isObject());
    const QJsonObject root = doc.object();
    ASSERT_TRUE(root.contains("constraints"));
    const QJsonObject constraints = root.value("constraints").toObject();
    ASSERT_TRUE(constraints.contains("thd_max_pct"));
    ASSERT_TRUE(constraints.contains("i_ripple_rms_max_a"));
    ASSERT_TRUE(constraints.contains("min_pulse_margin_min_s"));
}

TEST(AutoPredUsesModuleParams)
{
    sim::SupervisorConfig cfg;
    sim::StrategySupervisor sup(cfg);
    sup.Reset(sim::StrategyId::SVPWM);

    sim::SupervisorInputs in{};
    in.time_s = 0.0;
    in.sector = 1;
    in.vdc_V = 400.0;
    in.v_alpha = 100.0;
    in.v_beta = 50.0;
    in.currents = {100.0, -50.0, -50.0};
    in.inv_params.pwm_frequency_hz = 10000.0;
    in.inv_params.deadtime_s = 2e-6;
    in.inv_params.enable_losses = true;

    sim::PowerModuleParams pm1 = sim::PM300CLA060();
    sim::PowerModuleParams pm2 = pm1;
    for (auto& p : pm2.igbt_vce_sat)
    {
        p.val_25C *= 1.5;
        p.val_125C *= 1.5;
    }
    for (auto& p : pm2.eon_mJ)
    {
        p.val_25C *= 2.0;
        p.val_125C *= 2.0;
    }
    for (auto& p : pm2.eoff_mJ)
    {
        p.val_25C *= 2.0;
        p.val_125C *= 2.0;
    }

    in.module_params = pm1;
    auto d1 = sup.SelectAutoPred(in, {sim::StrategyId::SVPWM});

    in.module_params = pm2;
    auto d2 = sup.SelectAutoPred(in, {sim::StrategyId::SVPWM});

    ASSERT_TRUE(std::isfinite(d1.predicted_total_w));
    ASSERT_TRUE(std::isfinite(d2.predicted_total_w));
    ASSERT_TRUE(std::abs(d1.predicted_total_w - d2.predicted_total_w) > 1e-3);
}

TEST(DpwmReducesSwitchingLoss)
{
    const double vdc = 350.0;
    const double v_alpha = 120.0;
    const double v_beta = 60.0;
    const double pwm_freq = 8800.0;
    const double deadtime = 2e-6;

    sim::Modulator mod;
    sim::DutyCycles sv = mod.ComputeFromAlphaBeta(v_alpha, v_beta, vdc, sim::ModulationMode::SVPWM, 1.0, nullptr);
    sim::DutyCycles dp = mod.ComputeFromAlphaBeta(v_alpha, v_beta, vdc, sim::ModulationMode::DPWM1, 1.0, nullptr);

    const double period = 1.0 / pwm_freq;
    const auto timeline_sv = sim::BuildCenterAlignedTimeline(sv, period, deadtime, 0.0, 0.0);
    const auto timeline_dp = sim::BuildCenterAlignedTimeline(dp, period, deadtime, 0.0, 0.0);

    const int transitions_sv = CountSwitchTransitions(timeline_sv);
    const int transitions_dp = CountSwitchTransitions(timeline_dp);
    ASSERT_TRUE(transitions_dp < transitions_sv);

    sim::PhaseCurrents currents{};
    currents.a = 80.0;
    currents.b = -40.0;
    currents.c = -40.0;

    sim::InverterParams params{};
    params.pwm_frequency_hz = pwm_freq;
    params.deadtime_s = deadtime;
    params.enable_losses = true;

    sim::InverterSwitchingModel inv_sv;
    sim::InverterSwitchingModel inv_dp;
    inv_sv.SetModuleParams(sim::PM300CLA060());
    inv_dp.SetModuleParams(sim::PM300CLA060());

    sim::LossBreakdown loss_sv{};
    sim::LossBreakdown loss_dp{};
    inv_sv.FromDuty(vdc, sv, currents, period, params, &loss_sv, nullptr, nullptr);
    inv_dp.FromDuty(vdc, dp, currents, period, params, &loss_dp, nullptr, nullptr);

    ASSERT_TRUE(loss_dp.total_sw_W < loss_sv.total_sw_W);
}

int main()
{
    int failed = 0;
    for (const auto& test : Registry())
    {
        try
        {
            test.fn();
            std::cout << "[PASS] " << test.name << "\n";
        }
        catch (const std::exception& ex)
        {
            ++failed;
            std::cout << "[FAIL] " << test.name << ": " << ex.what() << "\n";
        }
        catch (...)
        {
            ++failed;
            std::cout << "[FAIL] " << test.name << ": unknown error\n";
        }
    }

    if (failed != 0)
        std::cout << failed << " test(s) failed.\n";
    return failed == 0 ? 0 : 1;
}
