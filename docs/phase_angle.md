# Phase Angle Definitions (PF vs Id/Iq)

This simulator logs two phase-angle metrics used for DPWM strategy selection.

## 1) phi_pf_deg (power-factor / displacement angle)
Definition:
- Compute instantaneous P and Q from v_alpha/v_beta and i_alpha/i_beta in the **same frame**:
  - P_inst = 1.5 * (v_alpha * i_alpha + v_beta * i_beta)
  - Q_inst = 1.5 * (v_beta  * i_alpha - v_alpha * i_beta)
- Average P_inst and Q_inst over the measurement window to get P and Q.
- phi_pf_deg = wrap_deg(atan2(Q, P) * 180/pi)

Sign convention:
- P > 0: motoring (electrical power into motor)
- P < 0: regen
- Q > 0: inductive (current lags voltage)
- Q < 0: capacitive (current leads voltage)

Voltage source policy (Phase 1 lock):
- Prefer **applied average voltages** from the inverter model.
- In this codebase, `InverterSwitchingModel::FromDuty(...)` returns phase voltages {a,b,c}.
  After `RemoveCommonMode(...)`, you can derive v_alpha/v_beta for P/Q.
- If only controller voltage references are available, log `phi_pf_deg_approx` instead.

## 2) phi_idiq_deg (FOC current-vector proxy)
Definition:
- phi_idiq_deg = wrap_deg(atan2(Id, Iq) * 180/pi)

Notes:
- This is a proxy and can diverge from phi_pf in field-weakening or voltage-limited regions.
- We log it anyway for correlation checks and fallbacks.

## 3) Angle wrapping
Use a stable wrap to [-180, +180] deg for logs and binning:

```
float wrap_deg(float a) {
    while (a > 180.f) a -= 360.f;
    while (a < -180.f) a += 360.f;
    return a;
}
```

