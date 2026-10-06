# GenAI Use Log (Lab 8, GenAI Level 4)

One entry per agent session. Record what the agent made, what we changed, and how we verified it.

## 2026-10-06 · Sun tracker plan + initial implementation
**Tool:** Claude Code (desktop app), model Claude Opus 5.5
**Driver:** _[name]_  **Reviewers:** _[names]_

**Prompts:** Asked for a plan to build a sun tracker using the Lab 6 phototransistors and the Lab 7 reaction wheel. The agent read the Lab 6/7/8 slides and code, proposed a plan, and asked four design questions. Our answers:
- Solar array faces **−X (180°)**
- **PD controller with wheel bias**
- Deliverables: firmware, Python sim, plotting script, docs
- If the sun is lost: **stop the wheel and report**

**Agent made:**
- `KestrelSAT_SunTracker/`: copy of Lab 7 code plus commands 10–14 (calibrate, stream, sign check, track, parameters), `sun_tracker.h/.cpp`, and settings in `definitions.h`
- `sim/sun_tracker_sim.py`: 1-axis simulation using the same controller
- `analysis/plot_suntrack.py`: plots SD logs and scores them against R1/R2
- `README.md`, `CLAUDE.md`, this log

**Agent verified (no hardware):** compiled `sun_tracker.cpp` natively with no warnings, and checked that its outputs match the Python controller exactly. Ran all six sim scenarios. Ran the plot script on a simulated log.
**Not verified:** a full PlatformIO build of `main.cpp` (espressif32 platform not installed on this machine), and anything on KestrelSAT.

**We must verify on hardware:**
- [ ] PlatformIO build + upload succeeds
- [ ] Command 11 shows +X ≈ 0°, +Y ≈ 90°, −X ≈ 180°, −Y ≈ 270° after calibration (10)
- [ ] Command 12 reports consistent gyro and wheel signs. Copy them into definitions.h.
- [ ] Command 13 turns the −X face toward the light, not away from it
- [ ] Measure the string twist period and motor time constant, and rerun the sim with them

**We changed:** _[fill in]_
**Issues found:** _[fill in]_
