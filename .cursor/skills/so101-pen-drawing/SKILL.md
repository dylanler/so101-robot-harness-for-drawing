---
name: so101-pen-drawing
description: Draws pictures with the SO-101 robot arm pen plotter in this repo - preview and dry-run designs, trace images into strokes, draw on paper, verify with the C922 camera, and re-calibrate the pen/paper (ink ladder, servo-lag probe). Use when the user asks the robot to sketch, draw, or plot something, to repeat a drawing, to calibrate the pen or paper height, or mentions so101_sketch, setup profiles, or stroke files.
---

# SO-101 pen drawing

Run everything from the repo root with the lerobot environment active
(`conda activate python3`; the CLI is `python scripts/so101_sketch.py`). Add `--json` to get a
final `RESULT {...}` line. Default profile: `calibration/setups/pen90_letter_portrait.json`.

## Draw a picture (setup unchanged)

```
Progress:
- [ ] 1. Cameras up: curl -s http://127.0.0.1:8090/api/streams (see so101-camera-dashboard skill)
- [ ] 2. Design: built-in name, strokes/*.json, or `trace IMAGE --name N`
- [ ] 3. preview DESIGN  -> open the PNG, check it reads well
- [ ] 4. dry-run DESIGN  -> must be "n_failures": 0
- [ ] 5. User confirms a sheet is in place (same spot as before)
- [ ] 6. draw DESIGN --tag NAME
- [ ] 7. Verify: read work/live/NAME-after-c922.jpg, crop/upscale the drawing area
```

* Built-ins: `petronas` (final, 88 strokes), `petronas-base`, `petronas-details`, `artscience`.
* Tracing: `--mode edges` for photos, `--mode outline` for logos/silhouettes; fewer strokes
  (`--max-strokes 120`) draw cleaner. Hand-authored designs in `so101_sketch/designs/` look best:
  normalised (u right, v up), long strokes, silhouette layer then detail layer, `bold()` outlines.
* Lighter than wanted → `--press-mm 4`; wobbly corners → `--speed 20`. `--save` stores overrides.
* Touch-ups: `draw DESIGN --only 3,7` (stroke indices).
* Practice on a used sheet first when the design is new; offset with `--centre-xy x,y` if needed.

## Verify

The arm parks toward the robot after `draw`. Read the C922 still (1280x720) and crop the sheet:
the drawing occupies roughly the middle third of the frame. Compare against the preview PNG;
report what is visible, what is faint or missing. Wrist cam (90° mount) only sees under the jaws.

## Calibrate (anything physical changed)

1. User poses the pen tip on the new sheet centre → `init-setup NAME --area-mm 130,170 --toward-robot-mm 15`
2. `status --setup calibration/setups/NAME.json` → no `XX` cells, pitch change < ~10°
3. Paper height:
   * flexible refill → `ladder --setup … --levels-mm 20,17,14,11,8,5`, read the C922 crop of the
     right margin (highest inked dash per reach), refine with `--levels-mm 12,10.5,9,7.5,6,4.5`,
     then `set-surface --setup … --z0-mm Z --slope-x S` (S = mm of z per metre of reach)
   * stiff mount → `diag --setup … --no-detect` (lag must show a clear knee) then `probe`
4. `dry-run` → practice → final.

## Rules

* Never `draw` without a passing `dry-run`.
* One robot command at a time; do not loop `status` (every connect sags the arm a little).
* Never re-derive pen pitch or drawing centre from a parked pose; they live in the profile.
* Expected surface for the current profile: commanded z ≈ +8.5 mm (sag), not 0.
* If IK fails mid-run: `park`, then shrink `area_mm` or move `centre_xy_m` toward the robot.

## More

* Concepts and numbers: [docs/CALIBRATION.md](../../../docs/CALIBRATION.md)
* Full tool list and Python API: [docs/TOOL_API.md](../../../docs/TOOL_API.md)
* Past failures to avoid: [docs/LESSONS_LEARNED.md](../../../docs/LESSONS_LEARNED.md)
