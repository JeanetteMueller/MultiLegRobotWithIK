# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A header-only C++ library for the **inverse kinematics and gait control of a walking robot with a configurable number of 3-DOF legs** (4–8 legs; default build is a 5-leg pentapod). Each leg is an independent `RobotLeg` object with its own geometry, mounting angle, and height offset, so asymmetric layouts (non-uniform spacing, mixed segment lengths, clustered legs) are a first-class feature handled by the core math without special-case code — not just symmetric N-pods.

The library targets **ESP32 (Arduino framework)** and drives serial-bus servos (SCServo / ST3020). Joint angles are computed from a desired foot position in 3D space using the law of cosines.

> The repo is mid-migration from a full PlatformIO firmware sketch into a reusable Arduino library (`library.json` is new; `platformio.ini`, the old `*.cpp` sketch, and hardware glue like `servoFunctions.h`/`input.h`/`footSensors.h` were removed from the working tree but still exist in git history at `HEAD`). The library proper is everything under `src/`. There are currently **no `examples/`** and no build entrypoint checked in — building requires a consumer sketch that supplies `setup()`/`loop()` and the hardware drivers.

## Critical contract: `NUMBER_OF_LEGS`

The consumer **must `#define NUMBER_OF_LEGS` before including the library.** It is not a constructor argument shadow — the macro is used at compile time:

- `RobotWithKinematics.h` has `static_assert(NUMBER_OF_LEGS >= 4, ...)`
- `calculateAllLegAngles()` returns `std::array<LegAngles, NUMBER_OF_LEGS>`

The runtime leg count is also passed to the `RobotWithKinematics` constructor; keep the macro and the constructor argument in sync. Forgetting the `#define` is the most likely first-time build failure.

## Build / run

There is no in-repo build target. To compile, create a PlatformIO project that consumes this as a library. Reference config (recovered from the pre-migration `platformio.ini`):

```ini
[env:default]
platform = https://github.com/pioarduino/platform-espressif32/releases/download/stable/platform-espressif32.zip
board = esp32dev
framework = arduino
monitor_speed = 115200
lib_deps =
    workloads/SCServo@^1.0.1
    derdoktor667/FlyskyIBUS@^0.8.1   ; only if using the FlySky IBUS remote input
build_flags = -Wno-packed-bitfield-compat -Wno-unused-function
```

`library.json` declares `frameworks: arduino`, `platforms: espressif32`. SCServo is a hard dependency of the servo path; FlyskyIBUS is only needed if you wire up the RC input. There are **no unit tests** — verification is done on hardware over the serial monitor (115200 baud).

## Per-loop call order

`RobotWithKinematics` is a state machine, not a one-shot solver. The consumer's `loop()` must call its methods in this order each tick:

1. `applyControls(...)` — feed stick/pot inputs (walk X/Y, yaw rotation, foot extend, body height, tilt, yaw). Values are only latched at a clean cycle boundary (`currentPhase == 0 && walkingStep == 0`) so targets never change mid-step.
2. `mainLoop()` — advances the gait clock (`Walk_Idle → Walk_Active → Walk_Stopping`), increments `walkingStep`/`currentPhase`, and rotates which legs are lifted (`currentMovingLegs`).
3. `prepareTargetPositions()` — writes each leg's `targetPosition` (interpolated foot position) for this step; skips legs currently owned by a special pose.
4. `specialPoseLoop()` — ticks the independent special-pose state machine and overwrites `targetPosition` for its two legs.
5. `calculateAllLegAngles()` — runs IK on every leg against the current `BodyPose`, returns the array of `LegAngles`. The consumer converts these to servo commands.

`doSpecialPose(pose)` must be called *every* loop while a gesture should stay active; when calls stop, the legs animate back to their home position automatically.

## Architecture / data flow

Include chain: `MultiLegRobotWithIK.h` → `classes/RobotWithKinematics.h` (which pulls in `Vector3`, `LegAngles`, `BodyPose`, `RobotLeg`) + `basicFunctions.h`.

- **`RobotWithKinematics`** — owns the leg array (`RobotLeg *legs`, not owned/allocated by it) and all motion state: the walk state machine, the gait phase counter, the set of currently-lifted legs, the body pose, and the special-pose sub-state-machine. Decides *which* legs move and *where their feet should go*.
- **`RobotLeg`** — one leg's geometry (`BODY_RADIUS`, `COXA/FEMUR/TIBIA_LENGTH`, `HEIGHT_OFFSET`, `baseAngle`, `baseFootExtend`) plus the actual **3-DOF inverse kinematics** (`solveIK3DOF`) and the per-step foot interpolation (`interpolateSin`, `getStepTargetPosition`, `newTargetPosition`). Owns `baseFootPosition` / `lastTargetPosition` / `targetPosition`.
- **`BodyPose`** — body-level setpoint: `height`, `tiltX` (pitch), `tiltZ` (roll), `rotY` (yaw), and `bodyShiftX/Z` (horizontal translation of the hip plane). Angles stored in **radians**; construct from degrees via `BodyPose::fromDegrees(...)`.
- **`LegAngles`** — IK output: `coxa`/`femur`/`tibia` in **radians** (+ `*Deg()` accessors), a `valid` flag, and `allAnglesInLimit()` which enforces hard joint limits (swing ±65°, lift ±130°, knee ±140°). An unreachable or out-of-limit target yields `valid = false`.
- **`Vector3`** — POD math helper (operators, Y-axis `rotate`, axis rotations).
- **`InputValues`** — plain container for FlySky RC channels (sticks, pots, switches, failsafe); not wired into the kinematics, used by the (removed) input glue.

### Coordinate / kinematics conventions

- World axes: **X/Z is the ground plane, Y is up.** Foot positions live on the ground; the body floats at `height + HEIGHT_OFFSET` above each leg's hip.
- A leg's `baseAngle` (mounting position around the body, radians) is the single source of truth for orientation. `rotateCoordinates` is **derived** as `-2 * baseAngleDeg` — do not reintroduce it as an independent parameter; the comment in `RobotLeg` explains they must never diverge.
- `solveIK3DOF` projects the foot vector into the leg's radial/tangential frame: coxa swings only to correct tangential offset, then femur/tibia are solved as a 2D arm via the law of cosines (knee bends outward, negative tibia angle).
- **Gait balance invariant:** in `newTargetPosition`, the per-leg push factors for moving vs. stationary legs are chosen so the sum of foot translations over all legs is zero (`movingFactor = 1/(M+2)`, `stationaryFactor = M/((M+2)*(N-M))`). Preserve this when changing the gait, or the body will drift.
- `NUM_OF_MOVABLE_LEGS` = `1` for <5 legs (single-leg gait, avoids tipping), else `N/2`. For 4-leg robots, `setBodyShiftFactor` / `updateBodyShift` slide the center of mass toward the leg opposite the lifted one so it stays inside the support triangle.

## Conventions

- **Code comments, doc-comments, and many log strings are in German.** Match this when editing existing files. Identifiers are mostly English.
- Headers use include guards and are written to be `#include`d directly (header-only); there are no `.cpp` translation units in the library.
- `Serial.print` is used both for debug and (in `LegAngles::allAnglesInLimit`) for limit warnings — there is no logging abstraction.
