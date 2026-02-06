# CLI + GUI Runs

## Build
```bash
cmake -S . -B build
cmake --build build -j
ctest --test-dir build -R unit_tests --output-on-failure
```

## CLI Examples
Sweep only:
```bash
./build/simtool --sweep configs/map_coarse.json --out out/run1
```

Make LUT:
```bash
./build/simtool --make-lut --in out/run1
```

Cycle eval:
```bash
./build/simtool --cycle-eval --in out/run1 --cycle cycles/ece_like_iq.csv
```

Report:
```bash
./build/simtool --make-report --in out/run1 --out out/run1/report.pdf
```

All-in-one:
```bash
./build/simtool --all configs/map_coarse.json --out out/run1 --cycle cycles/ece_like_iq.csv
```

## GUI Workflow
1. In the main window, open `Tools -> Sweeps / LUT / Report...`.
2. Pick a scenario in the Scenario dropdown.
3. Use `From Main UI` to generate a baseline JSON from the current sim/motor/inverter settings, then edit if needed.
4. Set `Sweep JSON`, `Output Dir`, `Cycle CSV` (if needed), and optional `Python` path.
5. Click `Run`.
6. Watch Stage/Progress/Log and the Artifacts list update as outputs appear.

## Python Dependencies
The cycle and report tools require:
- Python 3
- matplotlib
- pandas
- numpy

Install (example):
```bash
python -m pip install matplotlib pandas numpy
```
