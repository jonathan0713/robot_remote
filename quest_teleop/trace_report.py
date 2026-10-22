#!/usr/bin/env python3
"""Where did the tool not follow the hand? A report over teleop_trace.npz files (quest_openarm_teleop.py --run-dir).

For every engaged tick: the target (what the hand asked for, before any limit) against the pose the measured joints
reach (FK of q_meas, else of q_cmd). Rotation lag is split onto the tool's own axes (x, y, z of the tool frame; for
the OpenArm right hand z is the outward tool axis, so "z" lag = spin not followed, "x" / "y" = tilts not followed).
Joints on a limit (within --limit-frac of their range) are counted over all engaged ticks and over the lagging ones.

  python3 quest_teleop/trace_report.py <round>/attempts/*/teleop_trace.npz [--side right] [--json out.json]
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parent))
from robot_profile import load_profile  # noqa: E402


def lag(rows: np.ndarray, columns: list, chain, limit_frac: float = 0.02, lag_deg: float = 10.0) -> dict | None:
    """Lag statistics of one trace (engaged ticks only), or None without any."""
    col = {c: i for i, c in enumerate(columns)}
    block = lambda name, n: rows[:, col[f"{name}1"]:col[f"{name}{n}"] + 1]
    engaged = rows[:, col["engaged"]] > 0.5
    target_p = rows[:, [col["target_x"], col["target_y"], col["target_z"]]]
    target_q = rows[:, [col[f"target_q{k}"] for k in "xyzw"]]
    q = block("q_meas", 7).copy()
    no_meas = np.isnan(q).any(axis=1)
    q[no_meas] = block("q_cmd", 7)[no_meas]
    use = engaged & ~np.isnan(target_p).any(axis=1) & ~np.isnan(q).any(axis=1)
    if not use.any():
        return None
    ach = [chain.fk(qq) for qq in q[use]]
    p_err = np.linalg.norm(target_p[use] - np.array([t[:3, 3] for t in ach]), axis=1)
    r_ach = Rotation.from_matrix(np.array([t[:3, :3] for t in ach]))
    e_tool = (r_ach.inv() * Rotation.from_quat(target_q[use])).as_rotvec()  # target relative to achieved, tool frame
    angle = np.degrees(np.linalg.norm(e_tool, axis=1))
    margin = np.minimum(q[use] - chain.lower, chain.upper - q[use]) / (chain.upper - chain.lower)
    at_limit = margin < limit_frac
    lagging = angle > lag_deg
    return {"engaged_ticks": int(use.sum()), "position_lag_mm": {"median": float(np.median(p_err) * 1e3),
                                                                 "p90": float(np.percentile(p_err, 90) * 1e3)},
            "rotation_lag_deg": {"median": float(np.median(angle)), "p90": float(np.percentile(angle, 90)),
                                 "fraction_over_" + f"{lag_deg:g}": float(lagging.mean())},
            "rotation_lag_rms_deg_per_tool_axis": dict(zip("xyz", np.degrees(np.sqrt((e_tool ** 2).mean(axis=0)))
                                                           .round(2).tolist())),
            "joint_at_limit_fraction": {f"j{j + 1}": float(at_limit[:, j].mean()) for j in range(q.shape[1])},
            "joint_at_limit_fraction_while_lagging": ({f"j{j + 1}": float(at_limit[lagging, j].mean())
                                                       for j in range(q.shape[1])} if lagging.any() else {})}


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("traces", nargs="+", type=Path)
    p.add_argument("--side", default="right")
    p.add_argument("--profile", default="openarm_dual_gripper")
    p.add_argument("--limit-frac", type=float, default=0.02)
    p.add_argument("--lag-deg", type=float, default=10.0)
    p.add_argument("--json", type=Path)
    args = p.parse_args(argv)
    chain = load_profile(args.profile).sides[args.side].chain
    report = {}
    for path in args.traces:
        z = np.load(path)
        if args.side not in z.files:
            continue
        r = lag(z[args.side], list(z["columns"]), chain, args.limit_frac, args.lag_deg)
        if r is None:
            continue
        report[str(path)] = r
        top = sorted(r["joint_at_limit_fraction_while_lagging"].items(), key=lambda kv: -kv[1])[:2]
        print(f"{path.parent.name}: {r['engaged_ticks']} ticks | position lag median {r['position_lag_mm']['median']:.0f} mm"
              f" | rotation lag median {r['rotation_lag_deg']['median']:.1f} deg, p90 {r['rotation_lag_deg']['p90']:.1f},"
              f" >{args.lag_deg:g} deg {r['rotation_lag_deg']['fraction_over_' + f'{args.lag_deg:g}'] * 100:.0f}%"
              f" | per tool axis RMS {r['rotation_lag_rms_deg_per_tool_axis']}"
              + (f" | lagging with {', '.join(f'{j} at limit {v * 100:.0f}%' for j, v in top)}" if top else ""))
    if args.json:
        args.json.write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
