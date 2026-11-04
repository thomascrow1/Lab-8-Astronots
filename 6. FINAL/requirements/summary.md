# Requirement results

Pointing = solar array angle from the Sun (gyro heading − scan sun heading). ±10° band = R1.

## Graph 1 · R1 + R2

| Source | Run | Start angle | R2 reach (limit 10 s) | R2 | R1 hold | in ±10° | in ±5° | max | RMS | R1 |
|---|---|---|---|---|---|---|---|---|---|---|
| model | Model: R1/R2 | 179.2° | 3.1 s | PASS | 30.0 s | 100.0% | 97.2% | 10.0° | 2.1° | PASS |

## Graph 2 · R3 (fans)

| Source | Run | Fans | Pointing held | Wheel saturated | Max error while held | R3 (30 s) |
|---|---|---|---|---|---|---|
| hardware | SunTrack_data019 (CW 100%) | CW 100% | 55.1 s | 51.5 s | 7.7° | PASS |
| hardware | SunTrack_data020 (CCW 100%) | CCW 100% | 52.7 s | 49.5 s | 10.0° | PASS |
| model | Model: CCW 50% | CCW 50% | 432.7 s | 426.8 s | 9.9° | PASS |
| model | Model: CCW 100% | CCW 100% | 62.3 s | 61.3 s | 9.7° | PASS |

## Graph 3 · S2 (momentum dump)

| Source | Run | Disturbance | Dumps finished | Pointing held | Max error while held | S2 |
|---|---|---|---|---|---|---|
| model | Model: CCW 70% + thrusters | CCW 70% | 3 | never lost (567.7 s run) | 4.3° | PASS |

Times are from the requirement's start: the slew back (graph 1) or fans on (graphs 2, 3).
Model fan torque: 2.41 mN·m per pair at 100% (2 Nov hold tests), ~speed² below that.
