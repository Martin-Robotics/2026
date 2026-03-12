#!/usr/bin/env python3
"""
PathPlanner Path Scaler for Half-Field Practice (v2)
=====================================================
Scales all PathPlanner .path files so the robot travels shorter distances,
while preserving linked-name consistency across chained paths.

KEY INSIGHT (v2 fix): All paths are scaled relative to the SAME global center.
This guarantees that if Path A ends at "Hub" and Path B starts at "Hub",
both will agree on where "Hub" is after scaling.  Multi-path autos work.

Default center: the field center (8.161, 4.035) for the 2026 Rebuilt field.
You can override with --center-x and --center-y to pick a different anchor.

Usage:
  python scale_paths.py [--scale 0.5] [--in-place]
  python scale_paths.py --scale 0.5 --center-x 3.4 --center-y 4.0
  python scale_paths.py --restore

The originals are backed up to paths_backup/ before any in-place modification.
"""

import json
import sys
import shutil
import argparse
from pathlib import Path

# ── Configuration ──────────────────────────────────────────────────────────
PATHS_DIR  = Path(__file__).resolve().parent.parent / "src" / "main" / "deploy" / "pathplanner" / "paths"
BACKUP_DIR = PATHS_DIR.parent / "paths_backup"
SCALED_DIR = PATHS_DIR.parent / "paths_scaled"

# 2026 Rebuilt field dimensions (meters)
FIELD_LENGTH = 16.322
FIELD_WIDTH  = 8.07
DEFAULT_CENTER_X = FIELD_LENGTH / 2   # 8.161
DEFAULT_CENTER_Y = FIELD_WIDTH  / 2   # 4.035


def scale_point(point, center, factor):
    """Scale a {x, y} point relative to a fixed center by the given factor."""
    if point is None:
        return None
    return {
        "x": center[0] + (point["x"] - center[0]) * factor,
        "y": center[1] + (point["y"] - center[1]) * factor,
    }


def scale_path(path_data, center, factor):
    """Scale all coordinates in a PathPlanner path relative to a global center."""
    for wp in path_data.get("waypoints", []):
        wp["anchor"]      = scale_point(wp["anchor"], center, factor)
        wp["prevControl"]  = scale_point(wp.get("prevControl"), center, factor)
        wp["nextControl"]  = scale_point(wp.get("nextControl"), center, factor)

    for zone in path_data.get("pointTowardsZones", []):
        if "fieldPosition" in zone:
            zone["fieldPosition"] = scale_point(zone["fieldPosition"], center, factor)

    return path_data


def verify_linked_names(path_files, center, factor):
    """Check that linked names will be consistent after scaling."""
    linked = {}          # name -> (rounded_x, rounded_y)
    pre_existing = []    # mismatches that already exist in the originals
    scaling_caused = []  # mismatches introduced by scaling (should be zero)

    # First pass: collect original positions of every linked name
    orig_positions = {}  # name -> list of (file, x, y)
    for pf in path_files:
        with open(pf, "r", encoding="utf-8") as f:
            data = json.load(f)
        for wp in data.get("waypoints", []):
            ln = wp.get("linkedName")
            if ln:
                orig_positions.setdefault(ln, []).append(
                    (pf.name, wp["anchor"]["x"], wp["anchor"]["y"])
                )

    # Second pass: check scaled positions
    for pf in path_files:
        with open(pf, "r", encoding="utf-8") as f:
            data = json.load(f)
        for wp in data.get("waypoints", []):
            ln = wp.get("linkedName")
            if not ln:
                continue
            scaled = scale_point(wp["anchor"], center, factor)
            pos = (round(scaled["x"], 6), round(scaled["y"], 6))

            if ln not in linked:
                linked[ln] = {"pos": pos, "file": pf.name}
            else:
                if linked[ln]["pos"] != pos:
                    # Is this mismatch pre-existing in the originals?
                    orig_entries = orig_positions.get(ln, [])
                    orig_coords = set((round(x, 6), round(y, 6)) for _, x, y in orig_entries)
                    if len(orig_coords) > 1:
                        pre_existing.append(ln)
                    else:
                        scaling_caused.append(ln)

    return linked, list(set(pre_existing)), list(set(scaling_caused))


def restore_backup():
    """Restore paths from backup."""
    if not BACKUP_DIR.exists():
        print(f"ERROR: No backup found at {BACKUP_DIR}")
        sys.exit(1)
    for f in PATHS_DIR.glob("*.path"):
        f.unlink()
    count = 0
    for f in BACKUP_DIR.glob("*.path"):
        shutil.copy2(f, PATHS_DIR / f.name)
        count += 1
    print(f"✅ Restored {count} path files from backup")
    print(f"   Backup retained at {BACKUP_DIR}")


def main():
    parser = argparse.ArgumentParser(description="Scale PathPlanner paths for half-field practice")
    parser.add_argument("--scale",    type=float, default=0.5,  help="Scale factor (default 0.5)")
    parser.add_argument("--center-x", type=float, default=None, help=f"Scale center X (default {DEFAULT_CENTER_X:.3f})")
    parser.add_argument("--center-y", type=float, default=None, help=f"Scale center Y (default {DEFAULT_CENTER_Y:.3f})")
    parser.add_argument("--in-place", action="store_true",       help="Overwrite originals (backup created)")
    parser.add_argument("--restore",  action="store_true",       help="Restore originals from backup")
    args = parser.parse_args()

    if args.restore:
        restore_backup()
        return

    factor   = args.scale
    center_x = args.center_x if args.center_x is not None else DEFAULT_CENTER_X
    center_y = args.center_y if args.center_y is not None else DEFAULT_CENTER_Y
    center   = (center_x, center_y)

    if not PATHS_DIR.exists():
        print(f"ERROR: Paths directory not found: {PATHS_DIR}")
        sys.exit(1)

    path_files = sorted(PATHS_DIR.glob("*.path"))
    if not path_files:
        print(f"No .path files found in {PATHS_DIR}")
        sys.exit(1)

    print(f"PathPlanner Path Scaler v2 — Global-Center Scaling")
    print(f"  Scale factor : {factor}x")
    print(f"  Scale center : ({center_x:.3f}, {center_y:.3f})  {'(field center)' if args.center_x is None else '(custom)'}")
    print(f"  Paths dir    : {PATHS_DIR}")
    print(f"  Files found  : {len(path_files)}")
    print()

    # ── Verify linked names ──
    print("Verifying linked-name consistency...")
    linked, pre_existing, scaling_caused = verify_linked_names(path_files, center, factor)

    if scaling_caused:
        print(f"\n  ❌ SCALING WOULD BREAK {len(scaling_caused)} linked name(s):")
        for name in scaling_caused:
            print(f"     '{name}'")
        print("\n  This should not happen with global-center scaling. Bug?")
    else:
        print(f"  ✅ All {len(linked)} linked names stay consistent after scaling")

    if pre_existing:
        print(f"  ⚠  {len(pre_existing)} linked name(s) already mismatched in originals (not our fault):")
        for name in sorted(pre_existing)[:10]:
            print(f"     '{name}'")
        if len(pre_existing) > 10:
            print(f"     ... and {len(pre_existing) - 10} more")
    print()

    # ── Determine output directory ──
    if args.in_place:
        if BACKUP_DIR.exists():
            print(f"  Removing old backup...")
            shutil.rmtree(BACKUP_DIR)
        print(f"  Backing up originals to: {BACKUP_DIR}")
        shutil.copytree(PATHS_DIR, BACKUP_DIR)
        output_dir = PATHS_DIR
    else:
        if SCALED_DIR.exists():
            shutil.rmtree(SCALED_DIR)
        SCALED_DIR.mkdir(parents=True)
        output_dir = SCALED_DIR

    # ── Scale all paths ──
    scaled_count = 0
    for pf in path_files:
        with open(pf, "r", encoding="utf-8") as f:
            data = json.load(f)

        waypoints = data.get("waypoints", [])
        if not waypoints:
            print(f"  SKIP (no waypoints): {pf.name}")
            continue

        orig_first = waypoints[0]["anchor"].copy()
        orig_last  = waypoints[-1]["anchor"].copy()

        data = scale_path(data, center, factor)

        new_first = waypoints[0]["anchor"]
        new_last  = waypoints[-1]["anchor"]

        out_path = output_dir / pf.name
        with open(out_path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)

        scaled_count += 1
        dx = orig_last["x"] - orig_first["x"]
        dy = orig_last["y"] - orig_first["y"]
        dist_orig = (dx**2 + dy**2) ** 0.5
        dx2 = new_last["x"] - new_first["x"]
        dy2 = new_last["y"] - new_first["y"]
        dist_new  = (dx2**2 + dy2**2) ** 0.5
        print(f"  ✓ {pf.name:50s}  {dist_orig:.2f}m → {dist_new:.2f}m")

    print()
    print(f"Scaled {scaled_count} paths ({factor}x, center=({center_x:.3f}, {center_y:.3f}))")
    if args.in_place:
        print(f"  Originals backed up to : {BACKUP_DIR}")
        print(f"  Scaled files in        : {PATHS_DIR}")
        print(f"  To restore             : python tools/scale_paths.py --restore")
    else:
        print(f"  Scaled files in        : {SCALED_DIR}")
        print(f"  To apply               : python tools/scale_paths.py --scale {factor} --in-place")
        print(f"  To restore after tests : python tools/scale_paths.py --restore")


if __name__ == "__main__":
    main()
