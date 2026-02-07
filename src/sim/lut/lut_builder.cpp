#include "lut/lut_builder.h"

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLocale>
#include <QTextStream>

#include <algorithm>
#include <cmath>
#include <limits>
#include <QHash>

#include "sim/modulator.h"
#include "sim/strategy/strategy_supervisor.h"

namespace sim
{
namespace
{
struct SampleRow
{
    double speed_rpm = 0.0;
    double iq_A = 0.0;
    double phi_deg = std::numeric_limits<double>::quiet_NaN();
    StrategyId strategy = StrategyId::SVPWM;
    double loss_w = std::numeric_limits<double>::quiet_NaN();
    bool constraint_ok = false;
};

static std::vector<QString> SplitCsv(const QString& line)
{
    std::vector<QString> fields;
    QString current;
    for (int i = 0; i < line.size(); ++i)
    {
        const QChar ch = line.at(i);
        if (ch == ',')
        {
            fields.push_back(current.trimmed());
            current.clear();
        }
        else
        {
            current.append(ch);
        }
    }
    fields.push_back(current.trimmed());
    return fields;
}

static QString Lower(const QString& text)
{
    return text.trimmed().toLower();
}

static StrategyId ParseStrategy(const QString& text)
{
    const QString t = Lower(text);
    if (t == "sv" || t == "svpwm") return StrategyId::SVPWM;
    if (t == "dpwmmin") return StrategyId::DPWMMIN;
    if (t == "dpwmmax") return StrategyId::DPWMMAX;
    if (t == "dpwm0") return StrategyId::DPWM0;
    if (t == "dpwm1") return StrategyId::DPWM1;
    if (t == "dpwm2") return StrategyId::DPWM2;
    if (t == "dpwm3") return StrategyId::DPWM3;
    if (t == "firmware") return StrategyId::Firmware;
    if (t == "auto_rule" || t == "autorule") return StrategyId::AUTO_RULE;
    if (t == "auto_pred" || t == "autopred") return StrategyId::AUTO_PRED;
    return StrategyId::SVPWM;
}

struct ModeMapEntry
{
    const char* name;
    ModulationMode mode;
    StrategyId strategy;
    int id;
};

static const std::vector<ModeMapEntry>& ModeMap()
{
    static const std::vector<ModeMapEntry> map{
        {"SVPWM", ModulationMode::SVPWM, StrategyId::SVPWM, 0},
        {"DPWM0", ModulationMode::DPWM0, StrategyId::DPWM0, 1},
        {"DPWM1", ModulationMode::DPWM1, StrategyId::DPWM1, 2},
        {"DPWM2", ModulationMode::DPWM2, StrategyId::DPWM2, 3},
        {"DPWM3", ModulationMode::DPWM3, StrategyId::DPWM3, 4},
        {"DPWMMIN", ModulationMode::DPWMMIN, StrategyId::DPWMMIN, 5},
        {"DPWMMAX", ModulationMode::DPWMMAX, StrategyId::DPWMMAX, 6},
        {"Firmware", ModulationMode::Firmware, StrategyId::Firmware, 7}
    };
    return map;
}

static ModulationMode StrategyToMode(StrategyId id)
{
    for (const auto& entry : ModeMap())
    {
        if (entry.strategy == id)
            return entry.mode;
    }
    return ModulationMode::SVPWM;
}

static QString ModeToString(ModulationMode mode)
{
    for (const auto& entry : ModeMap())
    {
        if (entry.mode == mode)
            return entry.name;
    }
    return "Unknown";
}

static int ModeId(ModulationMode mode)
{
    for (const auto& entry : ModeMap())
    {
        if (entry.mode == mode)
            return entry.id;
    }
    return -1;
}

static double WrapDeg(double deg)
{
    while (deg > 180.0)
        deg -= 360.0;
    while (deg < -180.0)
        deg += 360.0;
    return deg;
}

static int PhiBin(double phi_deg, int bins, double min_deg, double max_deg)
{
    if (bins <= 0)
        return -1;
    const double span = max_deg - min_deg;
    if (span <= 0.0)
        return -1;
    const double clamped = std::min(max_deg, std::max(min_deg, phi_deg));
    const double norm = (clamped - min_deg) / span;
    int idx = static_cast<int>(std::floor(norm * bins));
    if (idx < 0)
        idx = 0;
    if (idx >= bins)
        idx = bins - 1;
    return idx;
}

static int FindIndex(const std::vector<double>& values, double target)
{
    for (size_t i = 0; i < values.size(); ++i)
    {
        if (std::abs(values[i] - target) <= 1e-9)
            return static_cast<int>(i);
    }
    return -1;
}

static bool UpdateScalar(double& target, double value, double tol, std::vector<QString>* warnings, const QString& label)
{
    if (!std::isfinite(value))
        return false;
    if (!std::isfinite(target))
    {
        target = value;
        return true;
    }
    if (std::abs(target - value) > tol)
    {
        if (warnings)
            warnings->push_back(QString("%1 varies across rows (keeping NaN)").arg(label));
        target = std::numeric_limits<double>::quiet_NaN();
        return false;
    }
    return true;
}

static QString FindGitHash(const QString& start_path)
{
    QDir dir(QFileInfo(start_path).absoluteDir());
    for (int i = 0; i < 8; ++i)
    {
        if (dir.exists(".git"))
        {
            const QString gitPath = dir.filePath(".git");
            QFile headFile(QDir(gitPath).filePath("HEAD"));
            if (!headFile.open(QIODevice::ReadOnly | QIODevice::Text))
                return {};
            const QString head = QString::fromUtf8(headFile.readAll()).trimmed();
            if (head.startsWith("ref:"))
            {
                const QString ref = head.mid(4).trimmed();
                QFile refFile(QDir(gitPath).filePath(ref));
                if (refFile.open(QIODevice::ReadOnly | QIODevice::Text))
                    return QString::fromUtf8(refFile.readAll()).trimmed();
                QFile packed(QDir(gitPath).filePath("packed-refs"));
                if (packed.open(QIODevice::ReadOnly | QIODevice::Text))
                {
                    QTextStream stream(&packed);
                    while (!stream.atEnd())
                    {
                        const QString line = stream.readLine().trimmed();
                        if (line.startsWith("#") || line.isEmpty())
                            continue;
                        const QStringList parts = line.split(' ');
                        if (parts.size() == 2 && parts[1] == ref)
                            return parts[0];
                    }
                }
                return {};
            }
            return head;
        }
        if (!dir.cdUp())
            break;
    }
    return {};
}

static std::vector<StrategyId> DefaultLutStrategies()
{
    return {
        StrategyId::SVPWM,
        StrategyId::DPWMMIN,
        StrategyId::DPWMMAX,
        StrategyId::DPWM0,
        StrategyId::DPWM1,
        StrategyId::DPWM2,
        StrategyId::DPWM3,
        StrategyId::Firmware
    };
}

struct CellData
{
    std::vector<double> loss;
    std::vector<bool> valid;
    double best_loss = std::numeric_limits<double>::infinity();
    int best_idx = -1;
};

static void UpdateBest(CellData& cell)
{
    cell.best_loss = std::numeric_limits<double>::infinity();
    cell.best_idx = -1;
    for (size_t i = 0; i < cell.loss.size(); ++i)
    {
        if (!cell.valid[i])
            continue;
        if (cell.loss[i] < cell.best_loss)
        {
            cell.best_loss = cell.loss[i];
            cell.best_idx = static_cast<int>(i);
        }
    }
}
} // namespace

bool BuildLutFromSummaryCsv(const QString& summary_csv_path,
                            const LutConfig& config,
                            LutResult* out,
                            QString* error)
{
    if (out)
        *out = LutResult{};

    QFile file(summary_csv_path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        if (error)
            *error = QString("Failed to open %1").arg(summary_csv_path);
        return false;
    }

    QTextStream stream(&file);
    const QString headerLine = stream.readLine();
    if (headerLine.isEmpty())
    {
        if (error)
            *error = "Empty summary.csv";
        return false;
    }

    const auto headers = SplitCsv(headerLine);
    QHash<QString, int> col;
    for (int i = 0; i < static_cast<int>(headers.size()); ++i)
        col[Lower(headers[i])] = i;

    auto colIndex = [&](const QString& name) -> int
    {
        auto it = col.find(Lower(name));
        if (it == col.end())
            return -1;
        return it.value();
    };

    const int c_speed = colIndex("speed_rpm");
    const int c_iq = colIndex("iq_A");
    const int c_phi_pf = colIndex("phi_pf_deg");
    const int c_phi_idiq = colIndex("phi_idiq_deg");
    const int c_mode = colIndex("mode");
    const int c_loss = colIndex("avg_total_w");
    const int c_ok = colIndex("constraint_ok");
    const int c_invalid = colIndex("invalid");
    const int c_thd_max = colIndex("thd_max_pct");
    const int c_ripple_max = colIndex("i_ripple_rms_max_a");
    const int c_pulse_min = colIndex("min_pulse_margin_min_s");
    const int c_min_pulse_effective = colIndex("min_pulse_s_effective");

    if (c_speed < 0 || c_iq < 0 || c_mode < 0 || c_loss < 0 || c_ok < 0)
    {
        if (error)
            *error = "summary.csv missing required columns";
        return false;
    }

    std::vector<SampleRow> rows;
    std::vector<double> speeds;
    std::vector<double> iqs;
    LutConstraints constraints;
    std::vector<QString> warnings;

    while (!stream.atEnd())
    {
        const QString line = stream.readLine();
        if (line.trimmed().isEmpty())
            continue;
        const auto fields = SplitCsv(line);
        if (fields.size() <= static_cast<size_t>(std::max({c_speed, c_iq, c_mode, c_loss, c_ok})))
            continue;

        SampleRow row;
        row.speed_rpm = fields[c_speed].toDouble();
        row.iq_A = fields[c_iq].toDouble();
        row.strategy = ParseStrategy(fields[c_mode]);
        row.loss_w = fields[c_loss].toDouble();
        row.constraint_ok = fields[c_ok].toInt() != 0;

        if (c_invalid >= 0 && c_invalid < static_cast<int>(fields.size()))
        {
            if (fields[c_invalid].toInt() != 0)
                continue;
        }
        if (!std::isfinite(row.loss_w))
            continue;

        if (c_thd_max >= 0 && c_thd_max < static_cast<int>(fields.size()))
            UpdateScalar(constraints.thd_max_pct, fields[c_thd_max].toDouble(), 1e-9, &warnings, "thd_max_pct");
        if (c_ripple_max >= 0 && c_ripple_max < static_cast<int>(fields.size()))
            UpdateScalar(constraints.i_ripple_rms_max_a, fields[c_ripple_max].toDouble(), 1e-9, &warnings, "i_ripple_rms_max_a");
        if (c_pulse_min >= 0 && c_pulse_min < static_cast<int>(fields.size()))
            UpdateScalar(constraints.min_pulse_margin_min_s, fields[c_pulse_min].toDouble(), 1e-12, &warnings, "min_pulse_margin_min_s");
        if (c_min_pulse_effective >= 0 && c_min_pulse_effective < static_cast<int>(fields.size()))
            UpdateScalar(constraints.min_pulse_s_effective, fields[c_min_pulse_effective].toDouble(), 1e-12, &warnings, "min_pulse_s_effective");

        double phi_pf = std::numeric_limits<double>::quiet_NaN();
        double phi_id = std::numeric_limits<double>::quiet_NaN();
        if (c_phi_pf >= 0 && c_phi_pf < static_cast<int>(fields.size()))
            phi_pf = fields[c_phi_pf].toDouble();
        if (c_phi_idiq >= 0 && c_phi_idiq < static_cast<int>(fields.size()))
            phi_id = fields[c_phi_idiq].toDouble();

        if (config.phi_source == "phi_pf_deg")
        {
            if (std::isfinite(phi_pf))
                row.phi_deg = WrapDeg(phi_pf);
        }
        else
        {
            if (std::isfinite(phi_id))
                row.phi_deg = WrapDeg(phi_id);
        }

        if (!std::isfinite(row.phi_deg))
            continue;
        if (StrategyIsAuto(row.strategy))
            continue;

        rows.push_back(row);
        speeds.push_back(row.speed_rpm);
        iqs.push_back(row.iq_A);
    }

    if (rows.empty())
    {
        if (error)
            *error = "No usable rows in summary.csv";
        return false;
    }

    std::sort(speeds.begin(), speeds.end());
    speeds.erase(std::unique(speeds.begin(), speeds.end(), [](double a, double b)
                             { return std::abs(a - b) <= 1e-9; }),
                 speeds.end());

    std::sort(iqs.begin(), iqs.end());
    iqs.erase(std::unique(iqs.begin(), iqs.end(), [](double a, double b)
                          { return std::abs(a - b) <= 1e-9; }),
              iqs.end());

    const int phi_bins = std::max(1, config.phi_bins);
    const int speed_count = static_cast<int>(speeds.size());
    const int iq_count = static_cast<int>(iqs.size());

    const std::vector<StrategyId> strategies = DefaultLutStrategies();
    const int strat_count = static_cast<int>(strategies.size());

    std::vector<CellData> cells(static_cast<size_t>(phi_bins * speed_count * iq_count));
    for (auto& cell : cells)
    {
        cell.loss.assign(strat_count, std::numeric_limits<double>::infinity());
        cell.valid.assign(strat_count, false);
    }

    auto cellIndex = [&](int phi_idx, int speed_idx, int iq_idx)
    {
        return (phi_idx * speed_count + speed_idx) * iq_count + iq_idx;
    };

    auto stratIndex = [&](StrategyId id) -> int
    {
        for (int i = 0; i < strat_count; ++i)
        {
            if (strategies[i] == id)
                return i;
        }
        return -1;
    };

    for (const auto& row : rows)
    {
        const int phi_idx = PhiBin(row.phi_deg, phi_bins, config.phi_min_deg, config.phi_max_deg);
        const int speed_idx = FindIndex(speeds, row.speed_rpm);
        const int iq_idx = FindIndex(iqs, row.iq_A);
        const int sidx = stratIndex(row.strategy);
        if (phi_idx < 0 || speed_idx < 0 || iq_idx < 0 || sidx < 0)
            continue;

        CellData& cell = cells[static_cast<size_t>(cellIndex(phi_idx, speed_idx, iq_idx))];
        if (row.loss_w < cell.loss[static_cast<size_t>(sidx)])
        {
            cell.loss[static_cast<size_t>(sidx)] = row.loss_w;
            cell.valid[static_cast<size_t>(sidx)] = row.constraint_ok;
        }
    }

    for (auto& cell : cells)
        UpdateBest(cell);

    std::vector<int> selected_idx(cells.size(), -1);
    for (size_t i = 0; i < cells.size(); ++i)
        selected_idx[i] = cells[i].best_idx;

    if (config.smooth_islands)
    {
        std::vector<int> smoothed = selected_idx;
        for (int p = 0; p < phi_bins; ++p)
        {
            const int p_prev = (p - 1 + phi_bins) % phi_bins;
            const int p_next = (p + 1) % phi_bins;
            for (int s = 0; s < speed_count; ++s)
            {
                for (int q = 0; q < iq_count; ++q)
                {
                    const int idx = cellIndex(p, s, q);
                    const int cur = selected_idx[static_cast<size_t>(idx)];
                    if (cur < 0)
                        continue;

                    std::vector<int> neighbors;
                    neighbors.reserve(6);

                    const int s_prev = s - 1;
                    const int s_next = s + 1;
                    const int q_prev = q - 1;
                    const int q_next = q + 1;

                    auto pushNeighbor = [&](int pi, int si, int qi)
                    {
                        if (si < 0 || si >= speed_count || qi < 0 || qi >= iq_count)
                            return;
                        const int nidx = cellIndex(pi, si, qi);
                        const int val = selected_idx[static_cast<size_t>(nidx)];
                        if (val >= 0)
                            neighbors.push_back(val);
                    };

                    pushNeighbor(p_prev, s, q);
                    pushNeighbor(p_next, s, q);
                    pushNeighbor(p, s_prev, q);
                    pushNeighbor(p, s_next, q);
                    pushNeighbor(p, s, q_prev);
                    pushNeighbor(p, s, q_next);

                    if (neighbors.size() < 3)
                        continue;

                    std::vector<int> counts(strat_count, 0);
                    for (int v : neighbors)
                        ++counts[static_cast<size_t>(v)];

                    int best_idx = cur;
                    int best_count = counts[static_cast<size_t>(cur)];
                    for (int i = 0; i < strat_count; ++i)
                    {
                        if (counts[static_cast<size_t>(i)] > best_count)
                        {
                            best_idx = i;
                            best_count = counts[static_cast<size_t>(i)];
                        }
                    }

                    if (best_idx != cur && best_count >= config.island_min_neighbors)
                    {
                        const CellData& cell = cells[static_cast<size_t>(idx)];
                        if (best_idx >= 0 && best_idx < strat_count && cell.valid[static_cast<size_t>(best_idx)])
                            smoothed[static_cast<size_t>(idx)] = best_idx;
                    }
                }
            }
        }
        selected_idx.swap(smoothed);
    }

    LutResult result;
    result.speed_rpm = speeds;
    result.iq_A = iqs;
    result.phi_bins = phi_bins;
    result.phi_min_deg = config.phi_min_deg;
    result.phi_max_deg = config.phi_max_deg;
    result.phi_source = config.phi_source;
    result.constraints = constraints;
    result.build_info.timestamp_utc = QDateTime::currentDateTimeUtc().toString(Qt::ISODate);
    result.build_info.git_hash = FindGitHash(summary_csv_path);
    result.build_info.warnings = warnings;
    {
        const QFileInfo summaryInfo(summary_csv_path);
        const QString runManifest = QDir(summaryInfo.absolutePath()).filePath("run_manifest.json");
        if (QFile::exists(runManifest))
            result.run_manifest_path = QFileInfo(runManifest).fileName();
    }

    result.phi_bins_deg.resize(static_cast<size_t>(phi_bins), 0.0);
    if (phi_bins > 0)
    {
        const double span = config.phi_max_deg - config.phi_min_deg;
        const double step = span / static_cast<double>(phi_bins);
        for (int i = 0; i < phi_bins; ++i)
            result.phi_bins_deg[static_cast<size_t>(i)] = config.phi_min_deg + (i + 0.5) * step;
    }
    result.cells.resize(cells.size());
    result.valid_mask.resize(cells.size(), 0);

    double regret_sum = 0.0;
    int regret_samples = 0;
    result.regret_max_w = 0.0;
    result.invalid_cells = 0;
    result.constraint_violations = 0;
    int valid_cells = 0;

    for (size_t i = 0; i < cells.size(); ++i)
    {
        const CellData& cell = cells[i];
        const int sel = selected_idx[i];
        LutCell outCell;
        outCell.valid = (cell.best_idx >= 0);
        outCell.best_loss_w = cell.best_loss;

        if (!outCell.valid)
        {
            result.invalid_cells++;
            outCell.strategy = StrategyId::SVPWM;
            outCell.loss_w = std::numeric_limits<double>::quiet_NaN();
        }
        else
        {
            if (sel >= 0 && sel < strat_count && cell.valid[static_cast<size_t>(sel)])
            {
                outCell.strategy = strategies[static_cast<size_t>(sel)];
                outCell.loss_w = cell.loss[static_cast<size_t>(sel)];
                const double regret = outCell.loss_w - outCell.best_loss_w;
                regret_sum += regret;
                regret_samples++;
                result.regret_max_w = std::max(result.regret_max_w, regret);
            }
            else
            {
                result.constraint_violations++;
                outCell.strategy = strategies[static_cast<size_t>(cell.best_idx)];
                outCell.loss_w = cell.best_loss;
            }
        }

        result.cells[i] = outCell;
        result.valid_mask[i] = outCell.valid ? 1 : 0;
        if (outCell.valid)
            ++valid_cells;
    }

    const int total_cells = static_cast<int>(cells.size());
    if (total_cells > 0)
        result.phi_bin_fill_ratio = static_cast<double>(valid_cells) / static_cast<double>(total_cells);

    if (phi_bins > 0 && speed_count > 0 && iq_count > 0)
    {
        double sum_filled_phi = 0.0;
        for (int s = 0; s < speed_count; ++s)
        {
            for (int q = 0; q < iq_count; ++q)
            {
                int filled = 0;
                for (int p = 0; p < phi_bins; ++p)
                {
                    const int idx = cellIndex(p, s, q);
                    if (idx >= 0 && idx < total_cells && result.valid_mask[static_cast<size_t>(idx)])
                        ++filled;
                }
                sum_filled_phi += static_cast<double>(filled);
            }
        }
        result.avg_filled_phi_bins_per_speediq =
            sum_filled_phi / static_cast<double>(speed_count * iq_count);
    }

    if (regret_samples > 0)
        result.regret_mean_w = regret_sum / static_cast<double>(regret_samples);

    if (phi_bins > 1 && result.avg_filled_phi_bins_per_speediq <= 1.1)
    {
        warnings.push_back("phi LUT appears mostly empty: avg filled phi bins per (speed,iq) is <= 1. Add id_A sweep values or set phi_bins=1.");
    }
    if (phi_bins > 1 && result.phi_bin_fill_ratio < 0.2)
    {
        warnings.push_back("phi LUT fill ratio is low (<20%). DPWM ROI may be unreliable until the sweep covers more phi.");
    }

    result.build_info.warnings = warnings;

    if (out)
        *out = result;

    return true;
}

bool WriteLutJson(const QString& path, const LutResult& lut, QString* error)
{
    QJsonObject root;
    root.insert("phi_bins", lut.phi_bins);

    QJsonArray speedArr;
    for (double v : lut.speed_rpm)
        speedArr.append(v);
    root.insert("speed_rpm", speedArr);
    root.insert("speed_bins_rpm", speedArr);

    QJsonArray iqArr;
    for (double v : lut.iq_A)
        iqArr.append(v);
    root.insert("iq_A", iqArr);
    root.insert("iq_bins_a", iqArr);

    QJsonArray phiBins;
    for (double v : lut.phi_bins_deg)
        phiBins.append(v);
    root.insert("phi_bins_deg", phiBins);
    root.insert("phi_min_deg", lut.phi_min_deg);
    root.insert("phi_max_deg", lut.phi_max_deg);
    root.insert("phi_source", lut.phi_source);

    root.insert("regret_mean_w", lut.regret_mean_w);
    root.insert("regret_max_w", lut.regret_max_w);
    root.insert("invalid_cells", lut.invalid_cells);
    root.insert("constraint_violations", lut.constraint_violations);
    root.insert("phi_bin_fill_ratio", lut.phi_bin_fill_ratio);
    root.insert("avg_filled_phi_bins_per_speediq", lut.avg_filled_phi_bins_per_speediq);
    if (!lut.run_manifest_path.isEmpty())
        root.insert("run_manifest", lut.run_manifest_path);

    QJsonObject constraintsObj;
    constraintsObj.insert("thd_max_pct", lut.constraints.thd_max_pct);
    constraintsObj.insert("i_ripple_rms_max_a", lut.constraints.i_ripple_rms_max_a);
    constraintsObj.insert("min_pulse_margin_min_s", lut.constraints.min_pulse_margin_min_s);
    constraintsObj.insert("min_pulse_s_effective", lut.constraints.min_pulse_s_effective);
    root.insert("constraints", constraintsObj);

    QJsonObject buildObj;
    buildObj.insert("timestamp_utc", lut.build_info.timestamp_utc);
    if (!lut.build_info.git_hash.isEmpty())
        buildObj.insert("git_hash", lut.build_info.git_hash);
    if (!lut.build_info.warnings.empty())
    {
        QJsonArray warnArr;
        for (const auto& w : lut.build_info.warnings)
            warnArr.append(w);
        buildObj.insert("warnings", warnArr);
    }
    root.insert("build_info", buildObj);

    QJsonObject idMap;
    for (const auto& entry : ModeMap())
        idMap.insert(entry.name, entry.id);
    root.insert("mode_id_map", idMap);

    QJsonArray phiArr;
    const int speed_count = static_cast<int>(lut.speed_rpm.size());
    const int iq_count = static_cast<int>(lut.iq_A.size());

    auto cellIndex = [&](int phi_idx, int speed_idx, int iq_idx)
    {
        return (phi_idx * speed_count + speed_idx) * iq_count + iq_idx;
    };

    for (int p = 0; p < lut.phi_bins; ++p)
    {
        QJsonArray speedLayer;
        for (int s = 0; s < speed_count; ++s)
        {
            QJsonArray iqLayer;
            for (int q = 0; q < iq_count; ++q)
            {
                const LutCell& cell = lut.cells[static_cast<size_t>(cellIndex(p, s, q))];
                const ModulationMode mode = StrategyToMode(cell.strategy);
                iqLayer.append(ModeToString(mode));
            }
            speedLayer.append(iqLayer);
        }
        phiArr.append(speedLayer);
    }
    root.insert("lut_mode", phiArr);

    QJsonArray validArr;
    for (int p = 0; p < lut.phi_bins; ++p)
    {
        QJsonArray speedLayer;
        for (int s = 0; s < speed_count; ++s)
        {
            QJsonArray iqLayer;
            for (int q = 0; q < iq_count; ++q)
            {
                const LutCell& cell = lut.cells[static_cast<size_t>(cellIndex(p, s, q))];
                iqLayer.append(cell.valid);
            }
            speedLayer.append(iqLayer);
        }
        validArr.append(speedLayer);
    }
    root.insert("lut_valid", validArr);

    QJsonArray idArr;
    for (int p = 0; p < lut.phi_bins; ++p)
    {
        QJsonArray speedLayer;
        for (int s = 0; s < speed_count; ++s)
        {
            QJsonArray iqLayer;
            for (int q = 0; q < iq_count; ++q)
            {
                const LutCell& cell = lut.cells[static_cast<size_t>(cellIndex(p, s, q))];
                const ModulationMode mode = StrategyToMode(cell.strategy);
                iqLayer.append(ModeId(mode));
            }
            speedLayer.append(iqLayer);
        }
        idArr.append(speedLayer);
    }
    root.insert("lut_mode_id", idArr);

    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        if (error)
            *error = QString("Failed to write %1").arg(path);
        return false;
    }
    const QJsonDocument doc(root);
    file.write(doc.toJson(QJsonDocument::Indented));
    return true;
}

bool WriteLutHeader(const QString& path, const LutResult& lut, QString* error)
{
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        if (error)
            *error = QString("Failed to write %1").arg(path);
        return false;
    }

    QTextStream out(&file);
    out.setLocale(QLocale::c());
    out.setRealNumberNotation(QTextStream::FixedNotation);
    out.setRealNumberPrecision(6);

    const int speed_count = static_cast<int>(lut.speed_rpm.size());
    const int iq_count = static_cast<int>(lut.iq_A.size());

    out << "#ifndef LUT_PWM_STRATEGY_H\n";
    out << "#define LUT_PWM_STRATEGY_H\n\n";
    out << "#include <stdint.h>\n\n";
    out << "#define LUT_PHI_BINS " << lut.phi_bins << "\n";
    out << "#define LUT_SPEED_COUNT " << speed_count << "\n";
    out << "#define LUT_IQ_COUNT " << iq_count << "\n\n";

    out << "// Mode IDs are explicit and do not rely on enum order.\n";
    for (const auto& entry : ModeMap())
        out << "#define LUT_MODE_" << entry.name << " " << entry.id << "\n";
    out << "\n";

    out << "static const float lut_speed_rpm[LUT_SPEED_COUNT] = {";
    for (int i = 0; i < speed_count; ++i)
    {
        if (i)
            out << ", ";
        out << lut.speed_rpm[static_cast<size_t>(i)];
    }
    out << "};\n";

    out << "static const float lut_iq_a[LUT_IQ_COUNT] = {";
    for (int i = 0; i < iq_count; ++i)
    {
        if (i)
            out << ", ";
        out << lut.iq_A[static_cast<size_t>(i)];
    }
    out << "};\n\n";

    out << "#define LUT_INDEX(phi, speed, iq) (((phi) * LUT_SPEED_COUNT + (speed)) * LUT_IQ_COUNT + (iq))\n\n";

    const int total = lut.phi_bins * speed_count * iq_count;
    out << "static const uint8_t lut_mode_id[" << total << "] = {\n";
    for (int idx = 0; idx < total; ++idx)
    {
        const LutCell& cell = lut.cells[static_cast<size_t>(idx)];
        const int mode_id = ModeId(StrategyToMode(cell.strategy));
        out << "  " << mode_id;
        if (idx + 1 < total)
            out << ",";
        if ((idx % 12) == 11)
            out << "\n";
    }
    out << "\n};\n\n";

    out << "static const int16_t lut_alpha_q15[" << total << "] = {0};\n\n";
    out << "#endif // LUT_PWM_STRATEGY_H\n";
    return true;
}

bool WriteLutRuntimeParams(const QString& path, const LutResult& lut, QString* error)
{
    Q_UNUSED(lut);

    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        if (error)
            *error = QString("Failed to write %1").arg(path);
        return false;
    }

    const sim::SupervisorConfig cfg;

    QTextStream out(&file);
    out.setLocale(QLocale::c());
    out.setRealNumberNotation(QTextStream::FixedNotation);
    out.setRealNumberPrecision(6);

    out << "#ifndef LUT_RUNTIME_PARAMS_H\n";
    out << "#define LUT_RUNTIME_PARAMS_H\n\n";
    out << "#include <stdint.h>\n\n";
    out << "// Recommended runtime wrapper defaults (from StrategySupervisor)\n";
    out << "#define LUT_PHI_HYST_DEG " << cfg.hysteresis_deg << "f\n";
    out << "#define LUT_MIN_BENEFIT_W " << cfg.min_benefit_w << "f\n";
    out << "#define LUT_SECTOR_LOCK " << (cfg.sector_lock ? 1 : 0) << "\n";
    out << "#define LUT_MIN_DWELL_S " << cfg.min_dwell_s << "f\n";
    out << "#ifndef LUT_CONTROL_DT_S\n";
    out << "#define LUT_CONTROL_DT_S (1.0f/10000.0f)\n";
    out << "#endif\n";
    out << "#define LUT_DWELL_STEPS ((int)((LUT_MIN_DWELL_S / LUT_CONTROL_DT_S) + 0.5f))\n\n";
    out << "#endif // LUT_RUNTIME_PARAMS_H\n";
    return true;
}
} // namespace sim
