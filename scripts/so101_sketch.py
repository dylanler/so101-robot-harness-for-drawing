#!/usr/bin/env python3
"""SO-101 pen-plotter command line (replaces the old sketch_museum.py).

Offline (no robot):
  preview  DESIGN            render a design to PNG at the drawing-rectangle aspect
  dry-run  DESIGN            run the full motion plan through IK: reachability + time estimate
  trace    IMAGE --name N    image -> strokes/N.json (+ preview); then draw it with `draw strokes/N.json`
  set-surface --z0-mm Z      store an ink-ladder reading in the setup profile
  snap                       save stills from all cameras (dashboards must be running)

On the robot:
  status                     joints, pen tip, servo lag/load, reachability table (no motion)
  init-setup NAME            new setup profile from the current pose (pen tip resting on the paper)
  ladder                     ink-ladder dashes for reading the paper height from the camera
  probe                      servo-lag contact probing (stiff pen mounts only)
  diag                       log one slow descent at the drawing centre (contact-signal debugging)
  draw     DESIGN            home -> draw -> park -> snapshots
  jog / home / park          small moves

DESIGN = built-in name (petronas, petronas-base, petronas-details, artscience),
         a strokes .json file, or an image path (traced on the fly).
Every command accepts --setup PATH (default calibration/setups/pen90_letter_portrait.json)
and --json (print one machine-readable JSON line at the end, for agents/tool calls).
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from so101_sketch import config  # noqa: E402
from so101_sketch.designs import get_design, render_preview, save_strokes, stroke_stats, trace_image  # noqa: E402
from so101_sketch.session import SketchSession, capture_setup, dry_run, preview, set_surface  # noqa: E402
from so101_sketch.setup_profile import SetupProfile  # noqa: E402


def _floats(s: str) -> tuple[float, ...]:
    return tuple(float(x) for x in s.split(",") if x.strip())


def _apply_overrides(prof: SetupProfile, a) -> SetupProfile:
    for key in ("press_mm", "hover_mm", "speed_mm_s"):
        v = getattr(a, key, None)
        if v is not None:
            setattr(prof, key, v)
    if getattr(a, "area_mm", None):
        prof.area_mm = list(_floats(a.area_mm))
    if getattr(a, "centre_xy", None):
        prof.centre_xy_m = list(_floats(a.centre_xy))
    return prof


def main() -> None:
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--setup", default=str(config.DEFAULT_SETUP), help="setup profile JSON")
    common.add_argument("--json", action="store_true", help="print a JSON result line at the end")
    over = argparse.ArgumentParser(add_help=False)
    over.add_argument("--press-mm", dest="press_mm", type=float, help="override: depth below the surface while drawing")
    over.add_argument("--hover-mm", dest="hover_mm", type=float, help="override: travel height above the surface")
    over.add_argument("--speed", dest="speed_mm_s", type=float, help="override: drawing speed mm/s")
    over.add_argument("--area-mm", dest="area_mm", help="override: drawing rectangle 'W,H' in mm")
    over.add_argument("--centre-xy", dest="centre_xy", help="override: drawing centre 'x,y' in base metres")
    over.add_argument("--save", action="store_true", help="write the overrides back into the setup profile")
    trace = argparse.ArgumentParser(add_help=False)
    trace.add_argument("--mode", choices=["edges", "outline"], default="edges", help="image tracing mode")
    trace.add_argument("--max-strokes", type=int, default=250)
    trace.add_argument("--min-len-px", type=float, default=25.0)
    trace.add_argument("--simplify-px", type=float, default=1.5)

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("preview", parents=[common, over, trace], help="render a design (offline)")
    p.add_argument("design")
    p.add_argument("--out", type=Path)
    p = sub.add_parser("dry-run", parents=[common, over, trace], help="IK-plan a design (offline)")
    p.add_argument("design")
    p.add_argument("--step-mm", type=float, default=2.0, help="resampling step for the plan (profile uses 1)")
    p = sub.add_parser("trace", parents=[common, trace], help="image -> strokes JSON (offline)")
    p.add_argument("image")
    p.add_argument("--name", required=True, help="output name: strokes/<name>.json")
    p = sub.add_parser("set-surface", parents=[common], help="store an ink-ladder reading (offline)")
    p.add_argument("--z0-mm", type=float, required=True, help="commanded z (mm) where the pen just inks at the centre")
    p.add_argument("--slope-x", type=float, default=0.0, help="mm of z per metre of x (reach)")
    p.add_argument("--slope-y", type=float, default=0.0, help="mm of z per metre of y")
    p.add_argument("--note", default="")
    p = sub.add_parser("snap", parents=[common], help="camera stills")
    p.add_argument("--tag", default="now")
    p.add_argument("--no-depth", action="store_true")

    sub.add_parser("status", parents=[common], help="read the robot (no motion)")
    p = sub.add_parser("init-setup", parents=[common], help="new setup profile from the current pose")
    p.add_argument("name")
    p.add_argument("--pen-len-mm", type=float, default=0.0, help="0 = estimate (tip resting on the table plane)")
    p.add_argument("--area-mm", default="130,170")
    p.add_argument("--toward-robot-mm", type=float, default=0.0, help="shift the drawing centre from the pen position")
    p.add_argument("--right-mm", type=float, default=0.0)
    p.add_argument("--description", default="")
    p = sub.add_parser("ladder", parents=[common], help="ink ladders for paper-height calibration")
    p.add_argument("--u", type=float, default=1.0, help="ladder u (1.0 = right edge of the drawing area)")
    p.add_argument("--vs", default="0.70,0.40,0.05", help="ladder v positions (near/mid/far reach)")
    p.add_argument("--levels-mm", default="12,10.5,9,7.5,6,4.5", help="commanded heights, high to low")
    p = sub.add_parser("probe", parents=[common], help="servo-lag paper probing (stiff mounts)")
    p.add_argument("--grid", type=int, default=3)
    p = sub.add_parser("diag", parents=[common], help="log one descent at the centre")
    p.add_argument("--depth-mm", type=float, default=25.0)
    p.add_argument("--no-detect", action="store_true")
    p = sub.add_parser("draw", parents=[common, over, trace], help="draw a design")
    p.add_argument("design")
    p.add_argument("--only", default="", help="comma list of stroke indices (touch-ups)")
    p.add_argument("--tag", default=None)
    p = sub.add_parser("jog", parents=[common], help="move the pen tip by dx,dy,dz mm")
    p.add_argument("--dx", type=float, default=0.0)
    p.add_argument("--dy", type=float, default=0.0)
    p.add_argument("--dz", type=float, default=0.0)
    sub.add_parser("home", parents=[common], help="move above the drawing centre")
    sub.add_parser("park", parents=[common], help="lift and pull back so the camera sees the drawing")

    a = ap.parse_args()
    trace_kw = {}
    if hasattr(a, "mode"):
        trace_kw = {"mode": a.mode, "max_strokes": a.max_strokes, "min_len_px": a.min_len_px, "simplify_px": a.simplify_px}
    result: dict = {"cmd": a.cmd}

    # ---- offline ---------------------------------------------------------------------------
    if a.cmd in ("preview", "dry-run"):
        prof = _apply_overrides(SetupProfile.load(a.setup), a)
        if a.cmd == "preview":
            result.update(preview(a.design, prof, a.out, **trace_kw))
            print(f"preview -> {result['preview']}  ({result['strokes']} strokes, {result['ink_mm']:.0f} mm of ink)")
        else:
            result.update(dry_run(a.design, prof, a.step_mm, **trace_kw))
            print(json.dumps(result, indent=1))
    elif a.cmd == "trace":
        prof = SetupProfile.load(a.setup)
        strokes = trace_image(a.image, tuple(prof.area_mm), **trace_kw)
        out = save_strokes(strokes, config.REPO / "strokes" / f"{a.name}.json", {"source": a.image, **trace_kw})
        png = render_preview(strokes, config.LIVE / f"{a.name}-preview.png", tuple(prof.area_mm))
        result.update({"strokes_file": str(out), "preview": str(png), **stroke_stats(strokes, tuple(prof.area_mm))})
        print(f"{len(strokes)} strokes -> {out}\npreview -> {png}")
    elif a.cmd == "set-surface":
        result.update(set_surface(a.setup, a.z0_mm, a.slope_x, a.slope_y, a.note))
        print(f"surface saved in {result['setup']}: coef {result['surface']}")
    elif a.cmd == "snap":
        from so101_sketch.cameras import snapshot_all

        shots = snapshot_all(a.tag, include_depth=not a.no_depth)
        for k, v in shots.items():
            print(f"{k:10s} -> {v}")
        result["snapshots"] = {k: str(v) if v else None for k, v in shots.items()}

    # ---- robot -----------------------------------------------------------------------------
    elif a.cmd == "init-setup":
        from so101_sketch.hardware import connect

        robot = connect()
        try:
            prof = capture_setup(a.name, robot, a.pen_len_mm, (a.toward_robot_mm, a.right_mm), _floats(a.area_mm), a.description)
        finally:
            robot.disconnect()
        out = prof.save(config.SETUPS / f"{a.name}.json")
        result.update({"setup": str(out), "pen_length_mm": prof.pen_length_mm, "pen_pitch_deg": prof.pen_pitch_deg,
                       "centre_xy_m": prof.centre_xy_m})
        print(f"new setup -> {out}\n  pen {prof.pen_length_mm} mm, pitch {prof.pen_pitch_deg} deg, centre {prof.centre_xy_m}")
        print("next: `ladder` (flexible pen) or `probe` (stiff pen) to measure the paper height")
    else:
        prof = _apply_overrides(SetupProfile.load(a.setup), a) if hasattr(a, "press_mm") else SetupProfile.load(a.setup)
        with SketchSession(prof) as s:
            s.setup_path = Path(a.setup)
            if a.cmd == "status":
                result.update(s.status())
                print(json.dumps({k: v for k, v in result.items() if k != "cmd"}, indent=1))
            elif a.cmd == "home":
                s.home()
            elif a.cmd == "park":
                s.park()
            elif a.cmd == "jog":
                result["tip_mm"] = s.jog(a.dx, a.dy, a.dz)
                print("tip now (mm)", result["tip_mm"])
            elif a.cmd == "ladder":
                result.update(s.ladder(a.u, _floats(a.vs), _floats(a.levels_mm)))
                print("read the dashes in", result["snapshots"].get("c922"),
                      "then: set-surface --z0-mm <lowest inking level at the centre> --slope-x <mm per m>")
            elif a.cmd == "probe":
                result.update(s.probe(a.grid))
                print("surface", result)
            elif a.cmd == "diag":
                s.home(height_mm=20)
                xy = s.prof.centre
                z0 = s.sk.tip()[2] - 0.012
                log: list[dict] = []
                try:
                    zc = s.sk.probe_point(xy, z0, log=log, max_depth=a.depth_mm / 1000.0, detect=not a.no_detect)
                finally:
                    for row in log:
                        print(row)
                    s.sk.goto_xyz(xy, z0 + s.prof.hover_mm / 1000.0, speed=0.03)
                result["contact_z_mm"] = round(zc * 1000, 2)
                print(f"contact z = {zc * 1000:.2f} mm (guess {z0 * 1000:.2f})")
            elif a.cmd == "draw":
                only = [int(i) for i in a.only.split(",")] if a.only else None
                result.update(s.draw(a.design, only, a.tag, **trace_kw))
                print(json.dumps({k: v for k, v in result.items() if k != "cmd"}, indent=1))
                if a.save:
                    s.prof.save(a.setup)
    if a.json:
        print("RESULT " + json.dumps(result, default=str))


if __name__ == "__main__":
    main()
