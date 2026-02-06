# Audit (2026-02-05)

This file captures ground-truth status against `tasks2.md` requirements. It is intentionally terse and points to concrete files/symbols.

## 1) DC-link model status
- **No DC-link ripple or capacitor metrics are computed today.**
- No `Vdc_ripple_pp`, `Icap_rms`, or ESR loss exists.
- Files checked: `src/sim/inverter_switching_model.*`, `src/sim/pwm_timeline.*`, `IPMMotorSim/mainwindow.cpp`.

## 2) Bus current availability
- **No `Ibus(t)` is computed today.**
- `InverterSwitchingModel::FromDuty(...)` builds a PWM timeline of per-leg gate states and integrates **phase voltages** only.
- Timeline segments are available via `BuildCenterAlignedTimeline(...)` in `src/sim/pwm_timeline.*`; they do **not** currently produce bus current.

## 3) Ripple metrics
- **Min/max only** are computed when intra-PWM integration is enabled.
- `PwmRippleDiag` in `src/sim/inverter_switching_model.h` stores `i_start`, `i_end`, `i_min`, `i_max` per phase, plus optional torque min/max.
- No RMS ripple metric exists.

## 4) THD feasibility
- **No waveform buffers** or FFT/THD implementation exist today.
- The only intra-PWM integration is for loss inputs and min/max ripple diagnostics (see `InverterSwitchingModel::FromDuty(...)` in `src/sim/inverter_switching_model.cpp`).
- Therefore THD must be a **control-step proxy** until a sampling buffer is added.

## 5) Modulation modes present (and where)
- Enum: `sim::ModulationMode` in `src/sim/modulator.h`
  - `Firmware`
  - `SVPWM`
  - `DPWMMIN`
  - `DPWMMAX`
  - `DPWM0`
  - `DPWM1`
- Implementation: `sim::Modulator::ComputeFromAlphaBeta(...)` in `src/sim/modulator.cpp`
- **DPWM2 and DPWM3 are present** (added in this iteration).
- **DPWM clamp-window phase is defined by the voltage-vector angle** (`atan2(v_beta, v_alpha)` inside `Modulator::ComputeFromAlphaBeta(...)`), which is intentional and aligns with modulation definitions.

### Mode mapping (current behavior)
| Mode | Implementation detail | Notes |
|---|---|---|
| SVPWM | `v_offset = -0.5*(v_max+v_min)` | Classic SVPWM zero-sequence injection |
| DPWMMIN | clamp minimum phase to low rail | Uses `clampMin = true` |
| DPWMMAX | clamp maximum phase to high rail | Uses `clampMax = true` |
| DPWM0 | alternates clampMin/clampMax by sector parity | `clampMax = (sector % 2) == 1` |
| DPWM1 | alternates clampMin/clampMax by sector parity | `clampMin = (sector % 2) == 1` |
| DPWM2 | 60 deg clamp via sin(3*(theta - shift)) | shift = -pi/6, clampMax when sin >= 0 |
| DPWM3 | 30 deg clamp via sin(6*(theta - shift)) | shift = 0, clampMax when sin >= 0 |
| Firmware | uses SVPWM offset fallback in sim | Actual PWM from firmware path |

## 6) PWM frequency control
- Discrete mapping: `PwmFrequencyHzFromParam(int pwmfrq)` in `IPMMotorSim/mainwindow.cpp`.
- `InverterParams.pwm_frequency_hz` set from `Param::GetInt(Param::pwmfrq)` in `MainWindow::runFor()` (same file).
- No numeric `f_sw` override exists today.

## 7) Applied average voltages source
- `InverterSwitchingModel::FromDuty(...)` returns `PhaseVoltages {a,b,c}` computed from event-level timeline integration (per-segment `v_raw` integrated over PWM period).
- In `MainWindow::runFor()`, those voltages are passed through `inverter.RemoveCommonMode(voltages)` before driving `MotorModel::Step(...)`.
- **No direct `v_alpha/v_beta` output exists**, but alpha/beta can be computed from the returned phase voltages after common-mode removal.

## 8) Electrical angle source (theta_e)
- **Motor model electrical position** is the primary angle source.
- `MotorModel::getElecPosition()` (from `IPMMotorSim/motormodel.cpp`) returns electrical position in degrees based on `m_Position` and `m_syncdelay`.
- `SimRunner::StepOnce(...)` uses `motor->getElecPosition()` to:
  - set the controller rotor angle (`Controller::SetRotorAngle(...)`)
  - compute the `theta_rad` used for `vd/vq`→`v_alpha/v_beta` conversion
  - (if enabled) set `InverterParams.elec_angle_rad` for intra-PWM ripple/torque integration.
- **Modulator DPWM2/3 selector currently uses the voltage-vector angle** (`atan2(v_beta, v_alpha)` inside `Modulator::ComputeFromAlphaBeta(...)`), not the motor electrical position.
- **Sweep THD fundamental** uses `elec_freq_hz = speed_rpm/60 * pole_pairs` (in `src/sim/sweep/sweep_runner.cpp`).

## 9) DPWM0/1 observed behavior (pre-change preservation)
- DPWM0/DPWM1 clamp selection is based on **SVPWM sector parity** in `Modulator::ComputeFromAlphaBeta(...)`:
  - `DPWM0`: `clampMax = (sector % 2) == 1`, `clampMin = !clampMax`
  - `DPWM1`: `clampMin = (sector % 2) == 1`, `clampMax = !clampMin`
- This preserves the **existing clamp sequence** (odd/even sector alternation) and serves as the reference for any future shift-based reparameterization.
