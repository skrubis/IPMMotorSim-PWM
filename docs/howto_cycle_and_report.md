# Cycle Eval + Report How-To

## Configure/build
```
cmake -S . -B build
cmake --build build -j
ctest --test-dir build -R unit_tests --output-on-failure
```

## Run sweep
```
./build/simtool --sweep configs/map_coarse.json --out out/run1
```

## Build LUT
```
./build/simtool --make-lut out/run1
```

## Cycle eval
```
python tools/cycle_eval/eval_cycle.py --cycle cycles/ece_like_iq.csv --in out/run1 --out out/run1
```

## Report
```
python tools/report/make_report.py --in out/run1 --out out/run1/report.pdf
```
