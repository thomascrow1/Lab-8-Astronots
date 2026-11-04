# Requirement results

Pointing = solar array angle from the Sun (gyro heading − scan sun heading). ±10° band = R1.

## Graph 1 · R1 + R2

| Source | Run | Start angle | R2 reach (limit 10 s) | R2 | R1 hold | in ±10° | in ±5° | max | RMS | R1 |
|---|---|---|---|---|---|---|---|---|---|---|
| hardware | R1R2_data001 | -178.4° | 3.2 s | PASS | 30.0 s | 100.0% | 87.7% | 9.6° | 3.2° | PASS |
| model | Model: R1/R2 | 179.2° | 3.1 s | PASS | 30.0 s | 100.0% | 97.2% | 10.0° | 2.1° | PASS |

## Graph 2 · R3 (fans)

| Source | Run | Fans | Pointing held | Wheel first at limit | Wheel saturated (≥2 s) | Max error while held | R3 (30 s) |
|---|---|---|---|---|---|---|---|
| hardware | R3_data001 (CCW 100%) | CCW 100% | 68.7 s | 66.6 s | 66.6 s | 9.4° | PASS |
| hardware | R3_data002 (CCW 50%) | CCW 50% | never lost (1040.6 s run) | 906.3 s | 1008.3 s | 7.5° | PASS |
| model | Model: CCW 50% | CCW 50% | 432.7 s | 426.8 s | 429.8 s | 9.9° | PASS |
| model | Model: CCW 100% | CCW 100% | 62.3 s | 61.3 s | 61.3 s | 9.7° | PASS |

## Graph 3 · S2 (momentum dump)

| Source | Run | Disturbance | Dumps finished | Pointing held | Max error while held | S2 |
|---|---|---|---|---|---|---|
| hardware | S2_data001 (CCW 70%) | CCW 70% | 8 | never lost (941.7 s run) | 7.6° | PASS |
| model | Model: CCW 70% + thrusters | CCW 70% | 3 | never lost (567.7 s run) | 4.3° | PASS |

Times are from the requirement's start: the slew back (graph 1) or fans on (graphs 2, 3).
Model fan torque: 2.41 mN·m per pair at 100% (2 Nov hold tests), ~speed² below that.
