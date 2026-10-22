# Legacy code, frozen

This directory holds the original hackathon implementation, exactly as it was.
**Nothing in here is imported by the current system, and nothing in here should
be "fixed".** It is kept as a record of what was tried, and because the defects
it documents are the reason this project exists.

Do not copy code out of here. Do not make it pass tests. If something here
looks like it solves your problem, read [the plan](../PLAN.md) section D first
and check whether the thing you want was already rebuilt for a reason.

## What was here

| Path | What it was | What is wrong with it |
|---|---|---|
| `orchestrator.py` | The rich TUI and the control loop | Imported `get_image` from the repo root, where the file did not exist: `ModuleNotFoundError` on every start. The turn cap of 10 silently truncated successful long episodes. |
| `agent_harness.py` | The LLM tool-calling loop | `in_reasoning` was assigned only inside the reasoning branch and read outside it, so any model that emits content without reasoning tokens raised `UnboundLocalError`. The tool-call accumulator was keyed by index but indexed with `[0]`. `validate_and_execute_tool` checked bounds and then returned `"SUCCESS: Arm moved..."` **without sending anything** - the "success" was fabricated, and the model was told the arm had moved. |
| `sam2_processor.py` | Segmentation and the pixel-to-cm transform | SAM returns masks with **no class label**, so "the red block" could not be bound to an object at all. The transform was `cm = px / 30.0` with the origin at the image corner: the wrong frame entirely, with a constant (`PIXELS_PER_CM = 30.0`) that has no source and is inconsistent with the camera's own resolution. |
| `config.py` | All configuration | Duplicated verbatim at `src/vision-AI/config/config.py`, so there were two sources of truth that could drift. Hardcoded Tailscale IPs and a single model id. |
| `V1_arm_controller_full_status.ino/` | The arm firmware | A blocking `delay()` sweep of hardcoded angles. No network, no IK, no per-joint limits, no watchdog. The e-stop was polled only between `delay(1000)` blocks, so it could take a second to act, and it dropped PWM to zero, which makes the arm go limp instead of holding position. |
| `robot_arm_esp32.ino` | The ESP32 port of the same | A pin remap of the same hardcoded sweep. |

## The three defects this project exists to fix

1. **No identity.** The model could not select an object, because perception
   never told it what any object *was*.
2. **The wrong coordinate frame.** Positions were reported in image pixels
   scaled by a magic number, not in centimetres from the arm base.
3. **Invented Z.** Object height was guessed from the object's class, which
   is the single largest cause of grasp failure.

The current system addresses all three in
[`PLAN.md`](../PLAN.md) sections D.2, D.3 and D.4, and it measures whether the
fix worked in [`docs/RESULTS.md`](../docs/RESULTS.md).
