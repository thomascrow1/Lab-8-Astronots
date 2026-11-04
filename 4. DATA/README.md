# KestrelSAT data

CSV files straight off the SD card, plus the graphs made from them. Paths below are inside `4. DATA/`.

**Run numbers restart.** The SD card numbering started over after the PDR, so there are two different
`SunTrack_data001`–`015`. Always say which folder a run is from (e.g. "30 Oct run 008").

Command numbers in the notes are the ones the firmware had **on that day**. They were renumbered on
2 Nov 2026 (see the project `README.md`): old 13 = sun tracker (now 4), old 15 = AD check (now 6),
old 16 = resistor sweep (now 7), old 18 = fan sweep (removed), old 19 = fan hold test (now 8).

| Folder | What's in it |
|---|---|
| `0. Labs 6-7/Lab 6/` | Lab 6 attitude determination data (`lab_data001–021`, `lab3_data001`, `task6/7`), plus `Lab6_test_data001–002`, recorded after the PDR with the old Lab 6 command. |
| `0. Labs 6-7/Lab 7/` | Lab 7 reaction wheel tests A and B. `Lab7_testA_data001 (after PDR).csv` was recorded after the PDR with the old Lab 7 command (renamed: it clashed with the Lab 7 file). |
| `1. Before PDR (Oct)/csv/` | Sun tracker runs `SunTrack_data001–052` (6 Oct to the PDR). See `AI_LOG.md` 6–13 Oct entries. 051 = 10 kΩ sensor resistors, 052 = 20 kΩ (both in the PDR slides). |
| `1. Before PDR (Oct)/plots/` | 12 PDF plots per run from `analyze_suntrack.m`. 041 and 043 have none. `052/conditions.txt` has that run's setup. |
| `2. After PDR/2026-10-30/csv/` | After rewiring and the 3.3 kΩ resistors: `SunTrack_data001–015` (scan-and-hold, lobing, ±5° work), `ADCheck_data001–002` (old command 15), `PTSweep_data001` (old command 16). |
| `2. After PDR/2026-10-30/graphs/` | `SunTrack_data008/009.png`, resistor sweep plots. |
| `2. After PDR/2026-11-02/csv/` | Fan disturbance day. `SunTrack_data016–018`: tracker, fans off (baselines). `019`: fan hold test CW 100%. `020`: fan hold test CCW 100%. `FanSweep_data001`: fan torque sweep (unusable, rail sag). Fans on 5 V. |
| `2. After PDR/2026-11-02/graphs/` | `fan_hold_report.py` output: `01`–`05` graphs, `summary.md`/`summary.csv`, `runs/` per-run overviews. |
| `2. After PDR/2026-11-03/` | Command 10 requirement runs (CSVs straight in the folder). `R1R2_data001`: R2 + R1 (180° slew back in 3.2 s, 30 s hold, 100% inside ±10°). `R3_data001`: CCW fans 100% (wheel saturated 66.6 s after fans on). `R3_data002`: CCW fans 50% (first at the wheel limit 906 s, saturated 1008 s; pointing never left ±10°; ended by the 10 s saturation stop). Graphs: `6. FINAL/requirements/` (`requirement_graphs.py`). |
| `2. After PDR/analyze_suntrack.m` | MATLAB plots for every `SunTrack_data*.csv` under `2. After PDR/` (writes to `2. After PDR/plots/`). |

## Adding a new test day

1. Make `2. After PDR/YYYY-MM-DD/csv/` and copy that day's CSVs into it.
2. Put graphs in `2. After PDR/YYYY-MM-DD/graphs/`.
3. Add a row above saying what each run was.
