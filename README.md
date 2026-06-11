# MultiLegRobotWithIK

An Arduino/ESP32 library for the **inverse kinematics and gait control of a walking robot with a configurable number of 3‑DOF legs** — four legs or more.

Each leg is an independent object with its own geometry, mounting position and height offset, so you are **not limited to symmetric layouts**. Build a four‑legged scout, a five‑legged pentapod, a classic hexapod, a symmetric octopod, or something deliberately asymmetric with longer front legs and unusual spacing. The kinematics, gait planning and pose system all read each leg's parameters directly, so asymmetric builds work without any special‑case code.

The library only **computes joint angles**. Driving servos, reading a remote control and handling sensors is left to your sketch, which keeps the library small and hardware‑independent.

> Companion to the [original 5‑legged build](https://github.com/JeanetteMueller/Rocky_MK1). See the Instagram reel linked from the repository for it in motion.

## Features

- **Any leg count ≥ 4** and any angular layout — symmetric or asymmetric.
- **Per‑leg geometry:** body radius, coxa/femur/tibia lengths, mounting angle, height offset and foot extension are set individually for every leg.
- **Sideways knee‑axis offset** per leg: model femur/tibia joints that are mounted offset along their rotation axis instead of in a flat plane (optional, defaults to 0).
- **3‑DOF inverse kinematics** per leg (coxa hip‑swing, femur hip‑lift, tibia knee) using the law of cosines, with automatic reachability and joint‑limit checks.
- **Cyclic gait generator** that adapts to the leg count: one leg lifts at a time for ≤ 4 legs (more stable), half the legs for ≥ 5.
- **Body pose control:** height, pitch, roll and yaw — feet stay planted while the body moves.
- **Body‑shift balancing** for four‑legged robots: the centre of mass slides over the support triangle of the standing legs.
- **Scripted "special poses"** — a pair of legs leaves the gait cycle, moves to a custom position, holds, and returns (waving, greeting, gestures).
- **No external dependencies.** Header‑only, just the Arduino core and the STL.

## Installation

### Arduino IDE

1. Download this repository as a ZIP.
2. *Sketch → Include Library → Add .ZIP Library…* and select it.
3. Open *File → Examples → MultiLegRobotWithIK → BasicWalk*.

### PlatformIO

Add the library to your `platformio.ini`:

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
lib_deps =
    https://github.com/JeanetteMueller/MultiLegRobotWithIK.git
```

## ⚠️ Required: define `NUMBER_OF_LEGS`

The number of legs is a **compile‑time constant** (it sizes the angle array and a static assertion). You **must** define it *before* including the library, and it must match the count you pass to the constructor:

```cpp
#define NUMBER_OF_LEGS 5
#include <MultiLegRobotWithIK.h>
```

If you forget, the library stops with a clear `#error` instead of a cryptic failure.

## Quick start

```cpp
#define NUMBER_OF_LEGS 5
#include <MultiLegRobotWithIK.h>

// One RobotLeg per leg. Here: 5 legs evenly spaced at 72° intervals.
//          RobotLeg(bodyRadius, coxa, femur, tibia, heightOffset, footExtend, baseAngleDeg)
RobotLeg legs[NUMBER_OF_LEGS] = {
    RobotLeg(104.175, 54, 120, 222.5, 0, 170,   0),
    RobotLeg(104.175, 54, 120, 222.5, 0, 170,  72),
    RobotLeg(104.175, 54, 120, 222.5, 0, 170, 144),
    RobotLeg(104.175, 54, 120, 222.5, 0, 170, 216),
    RobotLeg(104.175, 54, 120, 222.5, 0, 170, 288),
};

RobotWithKinematics *robot;

void setup() {
    Serial.begin(115200);
    robot = new RobotWithKinematics(
        NUMBER_OF_LEGS, // number of legs (must match the macro)
        230.0,          // max step width (mm)
        70,             // interpolation steps per gait phase
        5,              // ms between gait ticks
        legs);          // the pre-configured legs
    robot->setPose(200.0, 0, 0, 0); // height, tiltX, tiltZ, yaw (deg)
}

void loop() {
    // Feed control inputs (normally from a joystick / RC receiver).
    robot->applyControls(
        0.0,   115.0, 0.0, // walk X (strafe), walk Y (forward), rotate (deg)
        170.0,             // foot extension (mm)
        200.0,             // body height (mm)
        0.0, 0.0, 0.0);    // body tilt X, tilt Z, yaw (deg)

    // Advance the state machines — always in this order, once per loop.
    robot->mainLoop();
    robot->prepareTargetPositions();
    robot->specialPoseLoop();

    // Solve IK for every leg.
    auto angles = robot->calculateAllLegAngles();

    for (uint8_t i = 0; i < NUMBER_OF_LEGS; i++) {
        if (angles[i].valid) {
            float coxa  = angles[i].coxaDeg();   // hip swing  (degrees)
            float femur = angles[i].femurDeg();  // hip lift    (degrees)
            float tibia = angles[i].tibiaDeg();  // knee        (degrees)
            // → map these to your servos and write them
        }
    }
    delay(5);
}
```

See **`examples/BasicWalk`** for a complete, runnable version.

## The per‑loop pipeline

`RobotWithKinematics` is a state machine, not a one‑shot solver. Call its methods **in this order, once per loop**:

| Step | Call | What it does |
|------|------|--------------|
| 1 | `applyControls(...)` | Latch walk direction, foot extension and body pose. Values are only taken at a clean cycle boundary, so targets never change mid‑step. |
| 2 | `mainLoop()` | Advance the gait clock and rotate which legs are currently lifted. |
| 3 | `prepareTargetPositions()` | Compute each leg's interpolated foot position for this step. |
| 4 | `specialPoseLoop()` | Tick the special‑pose state machine (overrides its two legs). |
| 5 | `calculateAllLegAngles()` | Run inverse kinematics on every leg → `std::array<LegAngles, NUMBER_OF_LEGS>`. |

## API overview

### `RobotLeg`

One leg's geometry and per‑leg math. Construct one per leg:

```cpp
RobotLeg(float bodyRadius,     // centre of body → this leg's first servo (mm)
         float coxaLength,     // hip segment length (mm)
         float thighLength,    // femur length (mm)
         float shinLength,     // tibia length, incl. foot pad (mm)
         float heightOffset,   // vertical offset of this leg's hip (mm)
         float baseFootExtend, // how far the foot rests out from the body (mm)
         double baseAngleDeg,  // where the leg sits around the body (degrees)
         LegLimits legLimits = LegLimits(),     // optional per-joint angle limits
         AxisOffset axisOffset = AxisOffset()); // optional sideways knee-axis offset (mm)
```

`baseAngleDeg` is the single source of truth for a leg's orientation; the command‑vector rotation per leg is derived from it automatically. `getMaxReach()` / `getMinReach()` return the leg's reach limits.

The last two parameters are optional. `legLimits` constrains each joint's angle (default ±180°). `axisOffset` shifts the knee joints sideways along their rotation axis — see below.

#### Sideways knee-axis offset (`AxisOffset`)

Normally the coxa, femur and tibia servos line up when viewed from above, so each leg is a flat plane. If your mechanics mount the femur and/or tibia joint with a sideways offset along their (horizontal) rotation axis, pass an `AxisOffset` to account for it in the IK:

```cpp
struct AxisOffset {
    float femur = 0.0f; // offset of the femur joint (at the coxa end), mm
    float tibia = 0.0f; // offset of the tibia joint (at the femur end), mm
};

// e.g. femur joint shifted +12 mm, tibia joint shifted -8 mm:
RobotLeg(104.175, 54, 120, 222.5, 0, 170, 0, LegLimits(), AxisOffset{12.0f, -8.0f});
```

Both default to `0` (the classic straight‑leg layout, mathematically unchanged). The offset is tangential to the leg's radial direction; the IK rotates the coxa so the foot still lands on its commanded target, while the femur/tibia 2D solve stays in the (now laterally shifted) leg plane.

### `RobotWithKinematics`

Owns the gait and pose state. Key methods:

- `applyControls(walkX, walkY, rotateDeg, footExtend, height, tiltXDeg, tiltZDeg, yawDeg)` — set everything at once (latched on a cycle boundary).
- `setWalkDirection(x, y, r)`, `setBaseFootExtend(mm)`, `setPose(height, tiltXDeg, tiltZDeg, yawDeg)` — individual setters.
- `setBodyShiftFactor(factor)` — fraction of `BODY_RADIUS` the centre of mass slides toward the opposite leg while walking (four‑legged robots only; `0` = off).
- `mainLoop()`, `prepareTargetPositions()`, `specialPoseLoop()` — the per‑loop pipeline.
- `calculateAllLegAngles()` → `std::array<LegAngles, NUMBER_OF_LEGS>`.
- `isValidPose()` — `true` if every leg can reach its current target.
- `doSpecialPose(pose)` — see below.

### `LegAngles`

Result of the IK for one leg:

- `coxa`, `femur`, `tibia` — angles in **radians**; `coxaDeg()`, `femurDeg()`, `tibiaDeg()` for degrees.
- `valid` — `false` if the target is unreachable or a joint limit is exceeded (swing ±65°, lift ±130°, knee ±140°).

### `BodyPose`

The body setpoint (height in mm, tilt/yaw in radians; build it from degrees with `BodyPose::fromDegrees(...)`). You normally set it through `setPose(...)`.

### `Vector3`

A small 3D vector helper (`x` right, `y` up, `z` forward/back) used for positions and rotations.

## Coordinate system & conventions

- **X/Z is the ground plane, Y is up.** Foot targets live on the ground; the body floats `height + heightOffset` above each hip.
- A leg's `baseAngleDeg` defines where it is mounted around the body — use any angles you like, not just `360° / N`.
- The IK projects the foot into each leg's radial/tangential frame: the coxa swings only to correct sideways offset, then femur and tibia are solved as a 2D arm (knee bends outward).
- A leg's `axisOffset` shifts the femur/tibia joints sideways (tangentially) along their rotation axis. Since both axes are parallel, only the sum affects the foot: the coxa angle absorbs the lateral shift, and the 2D femur/tibia solve runs in the shifted plane.

## Gait

When walking, some legs lift and swing forward while the rest stay planted and push the body. The number of simultaneously lifted legs adapts to the configuration:

- **≤ 4 legs:** one leg at a time (avoids tipping). For four legs, enable `setBodyShiftFactor(...)` so the body leans onto the support triangle.
- **≥ 5 legs:** half the legs (`N / 2`) lift together for a faster gait.

The foot push factors are balanced so the net translation over all legs is zero — the body does not drift sideways.

## Special poses

A "special pose" lets a pair of legs leave the gait cycle, move into a custom position, hold it, then return — useful for waving or greeting. Call `doSpecialPose(pose)` **every loop** while the gesture should stay active; when you stop calling it, the legs animate back to their home position automatically.

## Mapping angles to servos

The library outputs angles in degrees per joint. Your sketch is responsible for:

1. Converting each angle to your servo's units (e.g. serial‑bus position counts or a PWM pulse width).
2. Applying per‑servo calibration / direction / offset.
3. Writing the values to the bus.

The original build drives 3 × *N* serial‑bus servos (ST3020 / SCServo) over a single UART and takes input from a FlySky RC receiver over IBUS, but neither is required by this library.

## Reference build (5‑leg pentapod)

| | |
|---|---|
| Legs | 5 (configurable, ≥ 4) |
| Servos per leg | 3 (coxa, femur, tibia) |
| Microcontroller | ESP32 |
| Coxa length | 54 mm |
| Femur length | 120 mm |
| Tibia length | 222.5 mm (217 mm shin + 5.5 mm foot pad) |
| Body radius | ~104 mm (centre → leg) |
| Body height range | 170–285 mm |
| Max step width | 230 mm |
| Language | C++ (Arduino framework) |

## License

Not yet licensed. Until a license is added, default copyright applies — ask the author before redistributing.

## Author

Jeanette Müller — the inverse‑kinematics math was developed together with Claude.ai.
