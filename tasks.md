# task.md — PWM-edge simulator + DPWM/SVPWM sweep framework (IPMMotorSim fork)

Goal: Fork IPMMotorSim and extend it from "average-ish inverter" into a PWM-edge, deadtime-aware inverter + motor simulation that can:
- generate *actual switching waveforms per PWM cycle*,
- simulate phase current ripple and current paths (MOSFET/IGBT + diode conduction),
- estimate inverter efficiency (conduction + switching loss),
- sweep modulation strategies (SVPWM, DPWM variants, hybrid/blended),
- export results for building a real-time mode scheduler (when to switch which PWM mode).

Non-goals (for v1):
- full EMI modeling, parasitic inductances, snubbers, detailed gate-drive transient waveforms
- thermal coupling beyond simple temperature parameterization

---

## 0) Repo setup + baseline confidence

### Tasks
- [ ] Fork `Pete9008/IPMMotorSim`
- [ ] Add `stm32-sine` as a pinned submodule (or vendor drop) at a known-good commit/tag
- [ ] Create `docs/DEV_NOTES.md` documenting build steps, versions, and any patches
- [ ] Add CI (GitHub Actions) for Linux build + unit tests

### Deliverables
- Builds and runs original sim unchanged (`master` parity)
- CI green

### Acceptance
- Can run a baseline scenario and get the same outputs (within tolerance) as upstream

---

## 1) Measurement + logging scaffolding (before changing physics)

### Tasks
- [ ] Add structured logging hooks to capture per-control-tick and per-PWM-period signals:
  - control tick: `id_ref/iq_ref`, `id/iq`, `vd/vq` or `v_alpha/beta`, `theta_e`, `rpm`, `Vdc`
  - modulator outputs: `dutyA/B/C`, sector, `T1/T2/T0`, clamp phase (if any), blend factor
- [ ] Add a binary-friendly output format (CSV first, then optional Parquet via Python)
- [ ] Add run metadata header: simulation dt, pwm freq, controller freq, motor params

### Deliverables
- `logs/run_<timestamp>.csv` (or similar) with consistent columns
- `tools/plot_quicklook.py` to plot:
  - duties vs time
  - phase currents vs time
  - dq currents vs time

### Acceptance
- A single run produces a usable plot without manual parsing

---

## 2) Clean separation: Controller vs Modulator vs Plant

### Tasks
- [ ] Refactor into explicit modules/interfaces:
  - `Controller` (OpenInverter/stm32-sine FOC code)
  - `Modulator` (SVPWM/DPWM/hybrid; outputs switching schedule)
  - `InverterSwitchingModel` (turns schedule + currents into phase voltages and loss)
  - `MotorPlant` (PMSM/IPM model)
  - `MechanicalLoad` (inertia + load torque)
- [ ] Define a stable interface:
  - Controller step @ `f_ctrl` outputs voltage request or duties
  - Modulator takes voltage request + Vdc and returns either:
    - (A) duties + timing info, or
    - (B) explicit switching events inside the PWM period (recommended)
- [ ] Keep a "compatibility path" that reproduces old average model behavior

### Deliverables
- `src/sim/` directory with clear module boundaries
- Unit tests for interface sanity

### Acceptance
- Baseline results remain consistent when running in “average mode”

---

## 3) Implement modulator family: SVPWM + DPWM variants + blending

### Tasks
- [ ] Implement SVPWM dwell computation (sector, T1/T2/T0) as canonical reference
- [ ] Implement DPWM by **redistributing T0** (without changing requested fundamental vector):
  - DPWM0 / DPWM1 (zero time on one side, sector-dependent)
  - DPWMMIN / DPWMMAX (clamp max/min duty leg)
  - Optional: "current-sign clamping" heuristic (later)
- [ ] Implement continuous blend `λ ∈ [0..1]` between symmetric and clamped zero distribution
- [ ] Add selection:
  - fixed mode (svpwm / dpwm0 / dpwm1 / dpwmmin / dpwmmax / blend)
  - future: auto mode (scheduler)

### Deliverables
- `src/modulation/modulator_*.*`
- `tests/test_modulator_vectors.cpp`:
  - random vectors in linear range: SVPWM vs DPWM must preserve average `v_alpha/beta` within tolerance
  - duty bounds + monotonic behavior

### Acceptance
- For identical voltage command, all modes produce same average `v_alpha/beta` (numerically close), but different clamp behavior

---

## 4) PWM-edge switching engine (event-based, not tiny fixed dt)

### Concept
Simulate within each PWM period using events:
- edges from the switching schedule
- deadtime boundaries
Between events, inverter output is constant → integrate motor ODE analytically or with stable small-step integrator.

### Tasks
- [ ] Choose PWM scheme (center-aligned recommended; match target firmware behavior)
- [ ] Convert modulator output into an ordered list of events:
  - time `t_event`
  - per-leg gate command state {HS on, LS on, both off}
- [ ] Support minimum pulse width clamp to avoid pathological micro-pulses
- [ ] Add optional random jitter/off-time (later) behind a flag

### Deliverables
- `src/inverter/switching_schedule.*`
- `tests/test_schedule_properties.cpp`:
  - monotonic event time ordering
  - correct symmetry (if center-aligned)
  - zero-vector placement matches selected DPWM variant

### Acceptance
- Can output a timeline of switching states for a single PWM period for debugging

---

## 5) Deadtime + diode conduction + current path resolution

### Tasks
- [ ] Implement deadtime: when transitioning HS↔LS, enforce both off for `t_dead`
- [ ] Resolve phase node voltage during deadtime based on phase current sign:
  - current > 0 ⇒ freewheel through lower diode (or upper diode depending on topology definition)
  - current < 0 ⇒ opposite diode
- [ ] Device drop models (start simple; parameterized):
  - MOSFET: `V = I*Rds_on` + diode `Vf`
  - IGBT: `V = Vce_sat(I)` + diode `Vf`
- [ ] Implement line-to-neutral phase voltage output `Va/Vb/Vc` from switching states + drops

### Deliverables
- `src/inverter/leg_model.*`
- `tests/test_deadtime_commutation.cpp`:
  - verify no shoot-through state
  - verify voltage clamps correctly for positive/negative current during deadtime

### Acceptance
- With an RL load, current commutation direction produces expected diode conduction and voltage levels

---

## 6) Motor plant upgrade for PWM ripple fidelity

### Tasks
- [ ] Ensure plant can respond at PWM timescale:
  - PMSM/IPM dq model (Ld/Lq/psi/Rs) with back-EMF
  - Use Clarke/Park transforms at each integration step/event
- [ ] Mechanical model:
  - inertia J, friction, load torque law (constant, quadratic, or user-defined)
- [ ] Validate with known cases:
  - no-load: current small, back-EMF matches rpm
  - locked rotor: current ramps limited by R/L

### Deliverables
- `src/plant/pmsm_dq.*`
- `tests/test_motor_sanity.cpp`

### Acceptance
- PWM ripple appears in phase current at expected frequency; average dq currents align with controller target

---

## 7) Loss + efficiency model (fast sweep-friendly)

### Conduction loss (time-domain)
Integrate per device over each event interval:
- `P_cond = ∫ i_device(t) * v_drop(i_device) dt`

### Switching loss (event-based)
At each turn-on/off event:
- `E_sw = Eon(I,Vdc) + Eoff(I,Vdc)` (simple param model)
- `P_sw = sum(E_sw) / sim_time`

### Tasks
- [ ] Implement conduction loss accounting (HS/LS devices + diodes)
- [ ] Implement switching loss model:
  - start: constant `Eon/Eoff` scaled by |I| and Vdc
  - later: piecewise fit tables (user supplies)
- [ ] Report:
  - per-leg/device loss breakdown
  - inverter total loss
  - electrical in/out and inverter efficiency

### Deliverables
- `src/loss/loss_accounting.*`
- `logs/` includes loss columns
- `tools/plot_losses.py` (loss vs rpm/torque, loss breakdown)

### Acceptance
- DPWM shows fewer switching events and reduced `P_sw` at high modulation (expected trend)

---

## 8) Sweep harness + result extraction

### Tasks
- [ ] Add CLI sweep runner:
  - define grid over rpm, iq (torque), Vdc, f_pwm, mode
  - run each point to steady-state (warm-up time) then record metrics over a window
- [ ] Metrics per operating point:
  - I_rms, I_thd (optional FFT)
  - torque ripple proxy (e.g., stddev of torque estimate)
  - Vcm metrics (optional)
  - P_cond, P_sw, P_total_loss, η_inverter
- [ ] Export summary table `results.csv` + optional per-point waveforms on demand

### Deliverables
- `tools/sweep.py` or `sim --sweep config.yaml`
- `sweeps/<name>/results.csv`
- `sweeps/<name>/config.yaml`

### Acceptance
- Can run a sweep overnight and get a clean results table ready for plotting

---

## 9) Build the “mode scheduler” from sweep results

### Tasks
- [ ] Define objective:
  - minimize loss subject to ripple constraints (or weighted cost)
- [ ] Generate best-mode map over operating space (e.g., |I| vs ω_e vs m)
- [ ] Add hysteresis bands for mode switching
- [ ] Implement `AUTO` modulation mode:
  - reads current operating point
  - selects mode (and blend factor λ)
- [ ] Verify no chatter and smooth transitions (dq tracking stable)

### Deliverables
- `tools/derive_scheduler.py` outputs:
  - LUT or decision rules
  - plots of mode regions
- `src/modulation/mode_scheduler.*` implementing AUTO

### Acceptance
- AUTO mode matches sweep-derived “best mode” in most of the map; transitions are stable

---

## 10) Validation scenarios (must-have)

### Core scenarios
- [ ] Low speed / high torque (near zero speed, high iq)
- [ ] Mid speed / mid torque
- [ ] High speed / field weakening off (initially)
- [ ] High modulation near linear limit
- [ ] Light load (where DPWM can increase distortion)

### Checks
- [ ] dq current tracking quality: overshoot/settling vs mode
- [ ] audible-noise proxy: current ripple magnitude
- [ ] loss trends: DPWM reduces switching loss when expected
- [ ] regression tests to prevent breaking earlier behavior

---

## 11) Documentation + usability polish

### Tasks
- [ ] `README.md` update: new modes, how to run a sweep, how to plot
- [ ] `docs/MODEL_LIMITATIONS.md`: what is and isn’t modeled
- [ ] `docs/MODULATION_NOTES.md`: DPWM variants implemented and equations

### Deliverables
- Reproducible examples:
  - `examples/single_point.yaml`
  - `examples/sweep_basic.yaml`

### Acceptance
- A new user can run: one point → plot waveforms → run sweep → plot efficiency map

---

## Performance targets (keep it practical)

- Single operating point (with PWM-edge sim) should run at least:
  - ~10x faster than real-time (stretch goal)
  - or at minimum, fast enough for sweep grids (hundreds of points) in hours, not days
- Use event-based integration; avoid tiny fixed dt unless necessary

---

## Risks / known gotchas

- Deadtime + diode conduction sign conventions are easy to get wrong → invest in RL-load tests.
- Some “loss improvements” may be artifacts of too-simple Eon/Eoff model → keep it parameterized.
- If the motor model is too ideal, torque ripple metrics can be misleading → validate against bench data later.

---

## Definition of Done (v1)

- DPWM vs SVPWM can be compared on:
  - phase current ripple metrics
  - switching event counts
  - estimated P_sw + P_cond + η_inverter
- Sweep runner outputs a results table
- AUTO scheduler can select modes with hysteresis and runs stably
- Documentation explains how to reproduce the comparisons
