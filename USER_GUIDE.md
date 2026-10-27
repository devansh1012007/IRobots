# IRobots v2 User Guide

How to run the operator console, give the arm a task, and read what it tells you.

This guide covers day-to-day operation. For setup and project background see the [README](../README.md). For design detail see [ARCHITECTURE.md](ARCHITECTURE.md).

---

## Contents

1. [Before you start](#before-you-start)
2. [Start the console](#start-the-console)
3. [A tour of the screen](#a-tour-of-the-screen)
4. [Run your first task](#run-your-first-task)
5. [Stopping the arm](#stopping-the-arm)
6. [Using the ESP32-CAM](#using-the-esp32-cam)
7. [Reading the displays](#reading-the-displays)
8. [Keyboard shortcuts](#keyboard-shortcuts)
9. [Safety behaviour](#safety-behaviour)
10. [Troubleshooting](#troubleshooting)
11. [Glossary](#glossary)

---

## Before you start

You need:

- The repository cloned and dependencies installed with `tools/install_deps.sh`.
- Any modern browser. The console needs no internet connection.

You do not need a robot arm, a camera or a GPU. By default everything runs in simulation, and the console says **Simulation** in the header.

## Start the console

From the repository root:

```bash
export PYTHONPATH=$PWD/ros2_ws/src/irobots_core:$PWD/ros2_ws/src/irobots_nodes:$PWD/ros2_ws/src/irobots_msgs:$PWD/ros2_ws/src/irobots_eval:$PWD/ros2_ws/src/irobots_bringup
python -m irobots_nodes.web_node
```

Open <http://127.0.0.1:8080>. To use another port, add `--port 9000` or set `IROBOTS_WEB_PORT`.

The console only accepts connections from your own computer unless `allow_remote_bind` is set to `true` in `config/perception.yaml`. Leave it off unless you understand who can reach the machine, because the page can move the arm.

The page follows your system's light or dark setting. It uses black, gray and white only.

## A tour of the screen

**Header.** From left to right:

- **Connection.** A filled dot and "Connected" means the page is talking to the server. A hollow dot and "Reconnecting" means it is retrying every 1.5 seconds.
- **Mode.** "Simulation", "Hybrid: real camera, simulated arm" (double border), or "Real camera" (solid).
- **Safety supervisor.** Clear, rejected, or e-stop latched. Rejected and latched states are shown as a solid block so they cannot be missed.
- **Camera source.** Switch between the simulated camera and the ESP32-CAM.
- **Clock.** Simulation time and step count.
- **E-stop.** Always available.

**Left rail.** Where you work, top to bottom: scene, instruction, plan.

**Stage.** The arm drawing, the camera frame, the tracked-object table, and three logs behind tabs.

**Right rail.** Joint positions, safety, and outcomes.

## Run your first task

1. **Choose a scene.** Under *Scene*, pick a task and a seed, then select **Reset scene**.
   - L1: lift the red block.
   - L2: sort a block into the bin.
   - Seeds 0 to 29 give repeatable layouts. The same seed always produces the same scene.
2. **Give an instruction.** Under *Instruction*, type what the arm should do, or keep the default, then select **Make a plan**.
3. **Read the plan.** The plan appears under *Plan* as a list of skills with their targets, for example `move_above red_block_1`. The status reads "Awaiting approval". Nothing moves yet.
   - If the object you named is not in the scene, the plan is rejected and a message appears under the buttons. Fix the instruction or reset the scene.
4. **Approve.** Select **Approve**. The status changes to "Approved". **Step** and **Run** become available.
5. **Execute.**
   - **Step** runs exactly one skill, then waits. Use it the first few times so you can watch each move.
   - **Run** runs the whole plan. Every skill and the whole episode have time limits, so a run always ends.
6. **Check the result.** Look at *Outcome* on the right rail and the *Action log* tab. The replay hash identifies the exact sequence of actions.

Plan markers use shape rather than colour: a hollow circle is waiting, a thick ring is running, a filled circle is done, and a filled square is failed. The status word is always shown beside it.

## Stopping the arm

Select **E-stop** at any time, or press **E**. It works during a run.

- The arm halts and holds its pose. The safety badge reads "E-stop latched: holding pose", and the e-stop button changes to **Clear e-stop** with a double border.
- **Step** and **Run** are disabled until you clear it.
- To resume, select **Clear e-stop**, then continue.

E-stop is a simulation and software control. It has not been tested against real hardware.

## Using the ESP32-CAM

The default is the simulated camera. To use a physical ESP32-CAM:

1. Flash and power the camera board. See [hardware/CAMERA_CAPTURE.md](hardware/CAMERA_CAPTURE.md).
2. Set the board address before starting the server:
   ```bash
   export BOARD_IP=192.168.1.50
   ```
3. Calibrate the camera with markers and save the correspondences to a file.
4. In the console, enter the file path under *Marker calibration file*, then select **ESP32-CAM** in the header.

What to expect:

- Without a calibration file the switch is refused and the reason appears under the plan buttons. The console stays on the simulated camera.
- Selecting **ESP32-CAM** from the console also allows a real camera with the simulated arm. The mode badge then reads **Hybrid: real camera, simulated arm** and stays that way for the episode.
- In hybrid mode, *Privileged* success reads "n/a (real camera)". The simulator's ground truth is never used to grade real frames.
- If the camera stops responding, the episode ends with a `camera_unavailable` outcome rather than hanging.

Select **Simulated camera** to switch back.

## Reading the displays

### Arm drawing

A line drawing of the arm in its workspace. The dashed ring is the safe workspace. Squares are objects, with the bin drawn as a double square. The cross marks the tool tip, labelled with its height above the table. While e-stop is latched, the label reads "holding" and the arm is drawn heavier.

Use **Iso**, **Side** and **Top** to change the view. The caption gives the tool tip position in metres, the jaw angle, and the jaw opening in millimetres.

### Camera frame

The latest frame from the active source. The caption shows the source (`sim` or `esp32_cam`) and the arm mode. **Refresh** reloads the frame. This is the only place colour appears, because it is the camera image itself.

### Tracked objects

| Column | Meaning |
|---|---|
| Id | Persistent track identifier and label |
| Status | Tracker state for that object |
| Pixel | Bottom of the detection box, in image pixels |
| Raw xy | Table position straight from the camera geometry, in metres |
| Corrected xy | Position after the object-depth correction, in metres |
| Height | Object height and where it came from |

The arm uses the corrected position.

### Joints

One row per joint with its angle in degrees. The line underneath shows where the joint sits between its lower and upper limit. A mark near either end means the joint is close to a limit.

### Safety and outcome

- **Last verdict** is the supervisor's decision on the most recent pose. A rejection names the reason.
- **Failure mode** is `none` unless the episode failed.
- **Privileged** is judged from simulator state. **Perceived** is judged from what the camera sees. They are kept separate on purpose and should not be compared as one number.
- **Predicates** are the individual conditions that make up the task, each shown as true or false.

### Logs

- **Conversation.** What you asked and what the planner and worker replied.
- **Action log.** One record per turn. The replay hash at the bottom is the SHA-256 of the action sequence and matches what `replay.py` computes for the same episode.
- **Arm protocol.** Commands sent to the arm firmware or emulator and the acknowledgements returned.

## Keyboard shortcuts

Shortcuts are ignored while you are typing in a field.

| Key | Action |
|---|---|
| `P` | Approve plan |
| `S` | Step one skill (when enabled) |
| `E` | Trigger or clear e-stop |

## Safety behaviour

- Every pose goes through the safety supervisor before it is executed. A rejected pose is shown as a result, not hidden.
- The model never supplies coordinates. It names tracked objects, and the host works out where they are.
- Exactly one skill runs per observation.
- Runs cannot continue forever. Skills, episodes and trials all have time limits.
- Nothing moves until you approve the plan.

## Troubleshooting

| Symptom | Likely cause | What to do |
|---|---|---|
| Header says "Reconnecting" | The server stopped or the port is wrong | Check the terminal running `web_node`, restart it, and confirm the port |
| Page does not open | Server bound to another port | Run with `--port 8080` or check `IROBOTS_WEB_PORT` |
| **Step** and **Run** are disabled | Plan not approved, or e-stop latched | Select **Approve**, or clear e-stop. Hover the button for the reason |
| Message under the plan buttons after **Make a plan** | The instruction names something not in the scene | Reset the scene or reword the instruction |
| ESP32-CAM switch refused | No marker calibration file | Enter a valid path under *Marker calibration file* |
| Camera frame stays empty | Camera unreachable or no frame yet | Check `BOARD_IP`, power and Wi-Fi, then select **Refresh** |
| Episode ends with `camera_unavailable` | The camera stopped responding | Check the board, then reset the scene |
| Request rejected as too large | Instruction over the 64 KB body limit | Shorten the instruction |
| Results differ between runs | Different seed, or hybrid mode | Use the same seed and the simulated camera |

If something fails that is not listed here, copy the message from under the plan buttons and the last lines from the server terminal when reporting it.

## Glossary

| Term | Meaning |
|---|---|
| Skill | One atomic action from the fixed library: `move_above`, `grasp`, `lift`, `release`, `retreat`, `home` |
| Track | A persistent identity for an object across frames |
| TCP | Tool centre point, the tip of the gripper |
| Supervisor | The safety check every pose must pass |
| Privileged | Measured from simulator ground truth |
| Perceived | Measured from camera frames |
| Hybrid | A real camera driving a simulated arm |
| Replay hash | SHA-256 of the episode's action log |
| Seed | A number that fixes the scene layout so a run can be repeated |
