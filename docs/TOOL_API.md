# Tool / agent API

Everything an agent needs to drive the pen plotter, as either a **CLI** (one command = one tool
call, `--json` prints a final `RESULT {...}` line) or a **Python API**. The Cursor skills in
`.cursor/skills/` wrap these.

## CLI tools

| Tool | Command | Robot? | Returns (`--json`) |
| --- | --- | --- | --- |
| Preview a design | `preview DESIGN` | no | `preview` PNG path, `strokes`, `ink_mm`, `travel_mm` |
| Dry-run a design | `dry-run DESIGN [--step-mm 2]` | no | `ok`, `n_failures`, `failures`, `est_minutes`, `pen_pitch_range_deg`, `max_ik_err_mm` |
| Trace an image | `trace IMAGE --name N [--mode edges\|outline]` | no | `strokes_file`, `preview`, stroke stats |
| Store a ladder reading | `set-surface --z0-mm Z [--slope-x S]` | no | `setup`, `surface` |
| Camera stills | `snap [--tag T] [--no-depth]` | no | `snapshots` {camera: path} |
| Robot status | `status` | reads | joints, pen tip, pen pitch, servo lag/load, `unreachable_cells` |
| New setup profile | `init-setup NAME [--pen-len-mm 0] [--area-mm W,H]` | reads | `setup`, pen length/pitch, centre |
| Ink ladder | `ladder [--levels-mm …] [--vs …] [--u 1.0]` | moves | `marks`, `snapshots` |
| Servo-lag probe | `probe [--grid 3]` | moves | `coef`, `points`, `resid_mm` |
| Draw | `draw DESIGN [--only i,j] [--press-mm] [--speed] [--save]` | moves | stats + `snapshots` (before/after) |
| Small moves | `home`, `park`, `jog --dx --dy --dz` | moves | `tip_mm` for jog |

`DESIGN` = `petronas`, `petronas-base`, `petronas-details`, `artscience`, a strokes `.json`, or an
image path. Common flags: `--setup PATH`, `--json`.

Example tool call and result:

```bash
$ python scripts/so101_sketch.py dry-run petronas --json | tail -1
RESULT {"cmd": "dry-run", "strokes": 88, "ink_mm": 3479.3, "travel_mm": 3899.5, "servo_commands": 7099,
        "est_minutes": 4.74, "max_ik_err_mm": 0.03, "pen_pitch_range_deg": [-15.2, -10.4],
        "failures": [], "n_failures": 0, "plan_seconds": 12.1, "ok": true}
```

## Python API

```python
from so101_sketch import SketchSession, preview, dry_run, trace_image, set_surface, list_setups

preview("petronas")                                   # offline
report = dry_run("strokes/traced-petronas-twin-tower.json")
assert report["ok"]

with SketchSession("calibration/setups/pen90_letter_portrait.json") as s:
    print(s.status(scan=False))
    result = s.draw("petronas", tag="final")          # home -> draw -> park -> snapshots
    print(result["snapshots"]["c922"])
```

| Object | Purpose |
| --- | --- |
| `SetupProfile` | Load/save a setup JSON; `paper_xy(u, v)`, `surface_z(xy)`, frame vectors |
| `PaperSurface` | Surface polynomial; `fit(points)` and `from_ladder(z0, slopes)` |
| `Sketcher` | Low-level controller (`home`, `goto_xyz`, `draw`, `ladder`, `probe_grid`, `scan`). `Sketcher(None, profile)` is a dry run |
| `SketchSession` | One robot connection + profile; `status/home/park/jog/snapshot/draw/ladder/set_surface/probe` |
| `get_design(spec)` | Built-in name, strokes JSON, or image → list of `(N, 2)` arrays in `[0, 1]²` |
| `trace_image(path, area_mm, mode=…)` | Image → ordered strokes |

## Stroke file format

```json
{"format": "so101-strokes/v1",
 "meta": {"source": "references/petronas-twin-tower.jpeg", "mode": "edges"},
 "strokes": [[[0.41, 0.12], [0.41, 0.55], [0.43, 0.83]], [[0.20, 0.05], [0.80, 0.05]]]}
```

Points are normalised `(u, v)`: `u` 0 → 1 left to right, `v` 0 → 1 bottom to top, as the picture
looks to the viewer. The profile maps them onto the drawing rectangle. A bare list of strokes is
also accepted.

## Safety contract for agents

* Always `dry-run` before `draw`; do not draw with `n_failures > 0`.
* Do not edit `pen_*`, `home_seed` or `surface` by hand unless re-calibrating; they are measured.
* `draw` is not interruptible mid-stroke from the CLI except by Ctrl-C; the arm keeps torque and
  holds its pose after the process exits. `park` afterwards.
* Keep one robot job at a time (the serial port is exclusive) and do not reconnect in a loop
  (every connect sags the arm slightly).
