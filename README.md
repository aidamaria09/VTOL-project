# VTOL Tri-Engine Mode Management System
 
ArduPilot Lua script + parameter set — maps 2 switches + 1 slider to 3 flight modes on a tilt-tri VTOL.
 
## Hardware / Firmware
- FC: Holybro / N-Factory F405-WING
- Firmware: ArduPlane 4.5+ (QuadPlane build)
- Sim: ArduPilot SITL, frame `quadplane-tilttri`
- Airframe: tilt-tri VTOL, 0.8 m wingspan, 2.7 kg
  - 2 tilting front motors (lift + forward thrust)
  - 1 fixed rear motor (VTOL lift only)
  - 2 ailerons, 2 elevators, 1 rudder
## Files
- `vtol_modes.lua` — mode-switching logic (runtime, on FC's `scripts/` folder)
- Parameter file (any name, e.g. `vtol_quadplane.param`) — static QuadPlane/failsafe config
## Flight Modes
 
| VTOL sw | SAFETY sw | Mode | ArduPilot mode | Behavior |
|---|---|---|---|---|
| OFF | OFF | 1 — MANUAL | MANUAL (0) | Raw stick passthrough, no stabilization |
| ON | OFF | 2 — VTOL ASSIST | QHOVER (18) | Hover, computer-leveled, stick = climb/drift |
| OFF | ON | 3 — STABILIZED | FBWA (5) | Fixed-wing, bank/pitch capped by slider (5°–20°) |
| ON | ON | undefined → defaults to Mode 2 | QHOVER (18) | Fail-safe: hover is recoverable, forward flight isn't |
 
## RC Channel Mapping
| Ch | Function |
|---|---|
| 6 | Safety-throttle slider (continuous, sets bank/pitch limit in Mode 3) |
| 7 | VTOL switch |
| 8 | Safety switch |
 
- `RC6/7/8_OPTION = 0` — no built-in function, pure passthrough
- `FLTMODE_CH = 0` — disables ArduPilot's native mode-select channel (required; otherwise it overrides the script)
## Safety Behavior
- AHRS/gyro failure (after a prior healthy reading) → force MANUAL, one-time GCS warning
- First ~10 s post-boot: "unhealthy" AHRS ignored (EKF init grace period, not a fault)
- Attitude limit floor: 5° (ArduPilot pre-arm check rejects lower)
- RC/GCS loss: short (<1.5 s) → hold mode; long (>5 s) → RTL, QRTL in VTOL (slow vertical descent)
## Installation
1. Flash ArduPlane 4.5+ (QuadPlane build)
2. Mission Planner → Config → Full Parameter List → load parameter file
3. Reboot (`SCR_ENABLE` requires it)
4. Copy `vtol_modes.lua` to FC's SD card `scripts/` folder
5. Reboot again
6. Confirm in Mission Planner Messages tab: `VTOL Tri-Engine mode script started`
## Design Notes / Limitations
- Undefined switch combo (both ON) silently defaults to hover — not configurable
- `param:set_and_save` used (flash write per change); full slider sweep ≈ 20 writes
- No logging beyond GCS text messages
- Attitude limits reset to default (20°) only on transition to MANUAL, not on boot
## Key Parameters
| Param | Value | Purpose |
|---|---|---|
| `Q_FRAME_CLASS` | 7 (Tri) | 3-motor lift mixer |
| `Q_TILT_MASK` | 3 | Motors 1+2 tilt, motor 3 fixed |
| `Q_TILT_TYPE` | 2 (Vectored Yaw) | VTOL yaw via differential tilt |
| `Q_ASSIST_SPEED` | 8 m/s | VTOL motor auto-assist threshold |
| `Q_PILOT_SPD_DN` | 150 cm/s | Max pilot-commanded descent rate |
| `SCR_ENABLE` | 1 | Enables Lua scripting |
 
