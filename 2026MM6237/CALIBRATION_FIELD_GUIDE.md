# 🎯 Trim Calibration Field Guide

> **Team 6237 REBUILT – 2026 Competition**
>
> Calibrate per-tag aiming trims **without moving the robot**.
> Place at a known spot, press a button, read the answer.

---

## Equipment

| Item | Notes |
|------|-------|
| **Pool noodle** (~5 ft / 1.5m) | Main measuring stick |
| **4 ft board or stick** (~1.2m) | For shorter measurements |
| Gaffer tape | Optional — mark spots if reusing |
| Laptop w/ SmartDashboard | Connected to robot |
| This printout | Keep in drive team box |

> **All measurements are from the hub structure** — never from a distant wall.
> Longest measurement: 6 ft 7 in (2.0m). Most are ≤ 5 ft.

---

## Quick Procedure (< 2 min per tag)

1. **Power on robot**, connect SmartDashboard
2. **Place robot** at a position below (measure from hub)
3. Select matching position from **TrimCal/Position** dropdown
4. Set **TrimCal/Calc Trim** to `true`
5. Read **TrimCal/Recommended Trim** and **TrimCal/Status**
6. Press **TrimCal/Apply Trim** to push value live
7. Move to next position, repeat

---

## 🔵 Blue Hub Positions

Blue hub front face is the flat wall facing you when you stand in front of it
on the field center side. Hub center Y = field center line.

```
         Top wall (Y = 8.070)
    ┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄
                               
              ┌────┐ Tag 24 (faces top)
              │    │
    Tag 25 ──►│ HUB│          FIELD CENTER
    Tag 26 ──►│    │◄── front face    ──────►  +X
              │    │
              └────┘ Tag 27 (faces bottom)
    
    Blue                        
    wall ┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄
         Bottom wall (Y = 0)
```

---

### B1: Tag 26 — Front Center ⭐ (DO THIS ONE FIRST)

**What you see:** The center front tag, dead ahead.

| Step | Measurement | How |
|------|-------------|-----|
| 1 | Place front bumper **2.0 m (6 ft 7 in)** from hub front face | Pool noodle (5ft) + 1.5 ft board, or tape measure |
| 2 | Center robot on hub (left/right bumpers equidistant from hub sides) | Eyeball — hub is centered on field center line |
| 3 | Point robot **straight at hub** (bumpers parallel to front face) | 180° heading |

> **Shortcut:** 2.0m ≈ the length of a standard pool noodle (5 ft) plus about 1.5 ft.
> Or: lay out **two yardsticks end-to-end** (6 ft) plus 7 inches.

---

### B2: Tag 25 — Front Off-Center

**What you see:** The off-center front tag (the one closer to the top wall).

| Step | Measurement | How |
|------|-------------|-----|
| 1 | Same distance as B1: front bumper **2.0 m** from hub front face | Same measuring stick |
| 2 | Shift robot **0.7 m (2 ft 4 in)** toward the **bottom wall** from B1's centered position | A forearm-length (~2 ft) offset from center |
| 3 | Angle robot **slightly toward the top wall** (~168° heading) | Turn ~12° from straight-on |

> **Why it works:** By shifting the robot low, tag 25 (which is above center on the hub)
> becomes the closest/largest tag in the Limelight's view.

---

### B3: Tag 27 — Side Face, Bottom

**What you see:** The side tag on the bottom of the hub (faces toward bottom wall).

| Step | Measurement | How |
|------|-------------|-----|
| 1 | Find the **bottom-front corner** of the hub structure (where front face meets bottom side) | |
| 2 | Place robot so the nearest bumper corner is **1.2 m (4 ft)** in front of that corner (toward +X) **AND 1.4 m (4 ft 7 in)** below it (toward −Y / bottom wall) | L-shape: one pool noodle from corner toward field center, one from corner toward bottom wall |
| 3 | Aim robot at hub center (~124° heading) | Point toward the middle of the hub structure |

```
                    ┌────┐
                    │ HUB│
                    │    │
                    └──●─┘ ← bottom-front corner
                       │ 
           1.2m (4ft)  │  1.4m (4ft 7in)
           ───────►    ▼
                    ┌─────┐
                    │ROBOT│ ← aim at hub
                    └─────┘
```

---

### B4: Tag 24 — Side Face, Top

**What you see:** The side tag on the top of the hub (faces toward top wall).

| Step | Measurement | How |
|------|-------------|-----|
| 1 | Find the **top-front corner** of the hub structure | |
| 2 | Place robot **1.2 m (4 ft)** in front + **1.4 m (4 ft 7 in)** above that corner | Same L-shape as B3, but toward the top wall |
| 3 | Aim robot at hub center (~236° heading) | Point toward the middle of the hub structure |

> Mirror of B3 on the other side.

---

## 🔴 Red Hub Positions

Identical layout, just on the other end of the field. Red hub front face faces
toward the Red wall (+X direction).

---

### R1: Tag 10 — Front Center ⭐ (DO THIS ONE FIRST)

**What you see:** The center front tag, dead ahead.

| Step | Measurement | How |
|------|-------------|-----|
| 1 | Place front bumper **2.0 m (6 ft 7 in)** from hub front face | Same measuring stick as B1 |
| 2 | Center robot on hub | Eyeball — centered on field center line |
| 3 | Point robot **straight at hub** (0° heading) | Bumpers parallel to front face |

---

### R2: Tag 9 — Front Off-Center

| Step | Measurement | How |
|------|-------------|-----|
| 1 | Front bumper **2.0 m** from hub front face | Same as R1 |
| 2 | Shift robot **0.7 m (2 ft 4 in)** toward the **top wall** | Forearm-length offset |
| 3 | Angle robot slightly toward bottom wall (~−12° heading) | Turn ~12° from straight-on |

> Tag 9 is below center on the Red hub, so shifting the robot above center
> makes tag 9 the dominant target.

---

### R3: Tag 8 — Side Face, Bottom

| Step | Measurement | How |
|------|-------------|-----|
| 1 | Find the **bottom-front corner** of the Red hub structure | |
| 2 | Robot **1.2 m (4 ft)** in front + **1.4 m (4 ft 7 in)** below the corner | L-shape, same as B3 |
| 3 | Aim robot at hub center (~56° heading) | Point at hub |

---

### R4: Tag 11 — Side Face, Top

| Step | Measurement | How |
|------|-------------|-----|
| 1 | Find the **top-front corner** of the Red hub structure | |
| 2 | Robot **1.2 m (4 ft)** in front + **1.4 m (4 ft 7 in)** above the corner | L-shape, mirror of R3 |
| 3 | Aim robot at hub center (~−56° heading) | Point at hub |

---

## 🏃 Speed Run: Minimum Viable Calibration

If you only have **2 minutes**, do just the front-center tags:

1. **B1** (Tag 26) — one measurement from Blue hub front face
2. **R1** (Tag 10) — one measurement from Red hub front face

These are the tags you see during most shots. One pool noodle + board, two placements, done.

If you have **5 minutes**, add the off-center front tags:

3. **B2** (Tag 25) — same distance, shift sideways
4. **R2** (Tag 9) — same distance, shift sideways

---

## Measurement Cheat Sheet

| Distance | What It Is | How to Measure |
|----------|-----------|----------------|
| **2.0 m (6 ft 7 in)** | Front bumper to hub face (B1/B2/R1/R2) | Pool noodle (5ft) + 1.5 ft board |
| **1.2 m (4 ft)** | Side positions, X offset from corner (B3/B4/R3/R4) | 4 ft board or cut pool noodle |
| **1.4 m (4 ft 7 in)** | Side positions, Y offset from corner (B3/B4/R3/R4) | Pool noodle minus ~5 inches |
| **0.7 m (2 ft 4 in)** | Off-center shift (B2/R2) from centered position | Forearm length |

---

## SmartDashboard Fields

| Field | What It Shows |
|-------|---------------|
| `TrimCal/Position` | Dropdown — select your position |
| `TrimCal/Calc Trim` | Button — press to calculate |
| `TrimCal/Status` | Result: OK / WARNING / ERROR |
| `TrimCal/Result Tag ID` | Which tag was seen |
| `TrimCal/Recommended Trim` | **The answer** — the trim to use |
| `TrimCal/Current Trim` | What's currently active |
| `TrimCal/Apply Trim` | Button — push recommended trim live |
| `TrimCal/Expected TX (deg)` | What geometry says the angle should be |
| `TrimCal/Actual TX (deg)` | What Limelight actually reports |

---

## Heading Quick Reference

| Position | Heading | Plain English |
|----------|--------:|---------------|
| B1 (Tag 26) | 180° | Straight at Blue hub (bumpers parallel to face) |
| B2 (Tag 25) | 168° | Almost straight, angled ~12° toward top wall |
| B3 (Tag 27) | 124° | Diagonal, facing upper-left toward hub |
| B4 (Tag 24) | 236° | Diagonal, facing lower-left toward hub |
| R1 (Tag 10) | 0° | Straight at Red hub (bumpers parallel to face) |
| R2 (Tag 9) | −12° | Almost straight, angled ~12° toward bottom wall |
| R3 (Tag 8) | 56° | Diagonal, facing upper-right toward hub |
| R4 (Tag 11) | −56° | Diagonal, facing lower-right toward hub |

> For straight positions (B1, R1): just make bumpers parallel to the hub face.
> For angled positions: point the robot at the hub structure — ±5° is fine.

---

## After Calibration

Once you've recorded good trim values, update them in `Constants.java` → `HubGeometry`
so they persist across redeploys:

```java
public static final double kTrimTag8  = <value from R3>;
public static final double kTrimTag9  = <value from R2>;
public static final double kTrimTag10 = <value from R1>;
public static final double kTrimTag11 = <value from R4>;
public static final double kTrimTag24 = <value from B4>;
public static final double kTrimTag25 = <value from B2>;
public static final double kTrimTag26 = <value from B1>;
public static final double kTrimTag27 = <value from B3>;
```

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| "No hub tag visible" | Verify Limelight is on — check `http://10.62.37.2:5800` |
| Wrong tag ID shown | Adjust robot angle slightly to favor the expected tag |
| Trim seems huge (>15°) | Double-check position and heading — wrong placement = wrong answer |
| Values differ between attempts | Take 3 readings and average — Limelight has frame-to-frame jitter |
| "WARNING: Expected Tag X but saw Tag Y" | Nudge robot to favor expected tag |

---

*All distances from hub structure. Robot: 0.9m × 0.9m. Pool noodle ≈ 5 ft (1.5m).
Generated from 2026-rebuilt-welded.json field geometry.*
