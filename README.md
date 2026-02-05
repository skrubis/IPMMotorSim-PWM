# IPMMotorSim
This project is an IPM Motor simulator intended to be uses with the Huebner inverter project, in particular the stm32-sine FOC firmware build.

# Aims
The main aim of this project is to allow testing and development of the motor control sections of the inverter firmware in a safe reproducible environment.

The model is not intended to be perfect and is in the very early days of its development.  Hopefully it's good enough to be useful now and will develop into something more sophisticated over time.

# Compiling
This repo supports both CMake (preferred) and the legacy qmake project.

First, make sure the stm32-sine submodule is present:

```
git submodule update --init --recursive
```

## CMake (preferred)
Qt has moved to CMake-based builds, so this is the recommended path.

```
cmake -S . -B build
cmake --build build --config Release
```

If CMake cannot find Qt, set `CMAKE_PREFIX_PATH` to your Qt install, for example:

```
cmake -S . -B build -DCMAKE_PREFIX_PATH="C:/Qt/6.6.1/msvc2019_64"
```

## qmake (legacy)
Qt Creator can still open `IPMMotorSim/IPMMotorSim.pro` directly.

A version of the source code for the stm32-sine firmware must be placed in the stm32-sine subdirectory of this project.  This will then be the used by the simulator to control the motor.  The OpenInverter code is in a fairly constant state of flux so it is pot luck whether a particular build will work.  Rel5.24 with the following mod at line 93 of pwmgeneration-foc.cpp is currently the best bet.

      /*if (frqFiltered < FP_FROMINT(30))
         qController.SetMinMaxY(dir <= 0 ? -qlimit : 0, dir > 0 ? qlimit : 0);
      else*/
         qController.SetMinMaxY(-qlimit, qlimit);

# Logging
The UI includes a **Log CSV** checkbox (Window Visibility group). When enabled, each run writes a log to `logs/run_<timestamp>.csv` under the working directory.

Quick plot helper:

```
python tools/plot_quicklook.py <path-to-run.csv>
```

If **PWM ripple (loss)** is enabled (Power Stage -> Basics), the CSV includes additional diagnostics:
- `inv_ia_pp, inv_ib_pp, inv_ic_pp`: peak-to-peak intra-PWM phase current (A) from the ripple approximation.
- `inv_ia_end, inv_ib_end, inv_ic_end`: end-of-period phase current estimate (A) from the ripple approximation.
- `inv_torque_pp`: peak-to-peak torque estimate (Nm) from the ripple approximation.

# Implemented Functionality (this fork)
- SVPWM + DPWM variants (DPWM0/1, DPWMMIN/MAX) with blend control.
- PWM modulation graph with zero-sequence/clamp/timing/sector visibility toggles.
- Event-level PWM timeline for inverter loss estimation (per-edge switching/deadtime/commutation accounting).
- Power stage loss estimation (conduction + switching + diode recovery) with PM300CLA060 placeholder curves.
- Power stage parameter editor (deadtime, min on/off, thermal, Vce/Eon/Eoff/Irr/trr curves, etc.).
- Inverter losses window with per-device loss breakdown and averaged summary strip.
- Optional intra-PWM current ripple approximation for loss inputs (simple RL + BEMF), with ripple/torque-ripple diagnostics.
- CSV logging includes modulation diagnostics, losses, ripple metrics, and basic thermal estimates.

# PWM/Loss Model Shortfalls (current limitations)
- Losses now use an event-level PWM timeline (per edge / deadtime window) but phase currents are still held constant over each PWM period.
- Minimum pulse / timer clamp behavior is still not firmware-accurate (only a basic deadtime-cancels-pulse clamp is applied).
- Switching loss uses curve interpolation without gate-drive dynamics, Miller effects, or parasitics.
- Diode recovery is approximated from Irr/trr; no detailed charge waveform or reverse recovery shape.
- Thermal model is first-order with fixed Rth values; no coupling or Cth from datasheet.
- Motor plant remains the legacy average model, so phase current ripple is not yet resolved (optional intra-PWM RL integration is for loss inputs only).

# Current Limitations
The simulator uses a number of new parameters not yet found in most builds of stm32-sine.  There is a replacement param_prj.h file in the project directory that will be used in place of the one in the subdirectory.  It is up to the user to ensure that the parameters contained in this replacement file are appropriate for whichever versions of the stn32-sine software is being used.

There is NO field validation on the GUI control fields, if you enter invalid values you will either get invalid simulation results or a program crash.

Not all GUI fields are supported by all versions of stm32-sine.

The simulator only exercises the main motor control sections of the stm32-sine software.  It does not fully simulate the lower level hardware peripherals of the processor (although it aims to accurately reflect their behaviour).  Nor does it exercise the higher level vehicle/charger control functions.

The model does not simulate drivetrain shunt (although it could be added relatively easily, see comments on source code).


Please note, as is says in the licence - This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE


For more information on this project please refer to https://openinverter.org/forum/viewtopic.php?t=2611
And here for DPWM efforts:
https://openinverter.org/forum/viewtopic.php?t=6928
