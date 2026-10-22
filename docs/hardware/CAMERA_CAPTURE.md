# Capturing real frames with the ESP32-CAM

The one piece of physical hardware that needs no assembly. It is the only way
to check the perception half of this artifact against something that is not a
render, and it is the highest-value use of the remaining physical access.

**Why this matters.** Every perception number measured on a MuJoCo render is
optimistic. Renders have no exposure drift, no JPEG artifacts, no motion blur,
no sensor noise, no barrel distortion and no auto-exposure shift. A detector
that reads 34x47 pixels on a clean render can read nothing at all on a real
frame. The real-image set is the check on that, and it is the difference
between "perception works in simulation" and "perception works".

**What it is not.** It does not need the arm. The camera and a printed
calibration board are enough.

---

## 1. Flash the camera

The firmware is at `firmware/esp32_cam/`. It is the Espressif CameraWebServer
example, unmodified except for the credentials.

1. Install the Arduino IDE and the `esp32` board package from Espressif.
2. Copy `camera_index.h` from the Espressif CameraWebServer example into
   `firmware/esp32_cam/`. It is deliberately not in version control because it
   is a large generated blob, and the sketch will not compile without it. A
   copy is committed under `robotic arm schematics/CameraWebServer/`.
3. Put your network name and password at the top of `CameraWebServer.ino`.
4. Board **AI Thinker ESP32 CAM**, PSRAM enabled, partition scheme **Custom**
   (so the sketch's `partitions.csv` is used).
5. Tie GPIO0 to ground, press reset, upload. Remove the jumper, reset again.
6. Open the serial monitor at 115200 baud and note the printed address.

The camera serves a single JPEG at `GET /capture` and an MJPEG stream at
`GET /stream` on port 81. **The capture endpoint is the one to use**: a still
frame is exactly what the perception pipeline consumes, and it avoids the
stream's timing entirely.

```bash
export BOARD_IP=192.168.1.50
curl http://$BOARD_IP/capture --output frame.jpg
```

## 2. Build the mount

No mount exists yet, so this is the one to build. The geometry is **specified**
in `config/scene.yaml`, and the perception code does not care whether the
camera is in simulation or on a bench, so the mount must match the spec.

| Property | Value | Where it comes from |
|---|---|---|
| Height above the table | 0.55 m | `config/scene.yaml` `scene.camera.position` |
| Tilt from vertical | 35 degrees | derived from the position and the target |
| Horizontal field of view | 60 degrees | `scene.camera.hfov_deg` - **calibrate this, do not assume it** |
| Resolution | 640x480 preferred | the ESP32-CAM's native mode is 320x240, which is too small: a 4 cm cube is only 17x24 px there, which is below the size at which open-vocabulary detectors retain AP |

The 35 degree tilt is not a style choice. A **strictly overhead camera cannot
observe object height at all** - a vertical object's height projects to almost
no pixels - which is exactly why the legacy design had to guess Z.

The camera must not move between calibration and capture. A millimetre of
movement invalidates the calibration, and it fails *silently*: positions stay
plausible while being consistently wrong. Mount it rigidly, on something
nothing else touches.

## 3. Print the calibration board

A ChArUco board, 6x5 squares of 30 mm, printed on matte paper at actual size.
Print it, **measure a square with a ruler**, and record the measured size: a
printer that scales by 2% puts a 6 mm error into every calibration.

Four of the board's corners must be at **known positions in the arm base
frame**. Measure them with a tape measure from the arm's base axis, and
record the values in the capture manifest. The arm does not have to exist for
this: the base axis is a marked point on the table.

## 4. Capture

```bash
python tools/capture_real_frames.py --board board.yaml --out data/real/ --count 300
```

The script, in order:

1. Reads the board's corner positions in the arm base frame.
2. Waits for the camera to answer `/capture`, with retries.
3. For each frame: fetch the JPEG, detect the ChArUco board, compute the
   board-to-base transform, write the frame plus its ground-truth corner
   positions to a manifest.
4. Writes `manifest.jsonl`: one line per frame with the file, the timestamp,
   the detected corners, the reprojection RMS, and the board pose.

A frame whose reprojection RMS exceeds 5 mm is **rejected, not kept**: a
calibration that is wrong in a way we cannot see is worse than no frame.

Vary the capture, or the set is useless:

* move the board and the objects to at least 20 different places in the
  workspace;
* change the lighting - dim, bright, and one deliberately uneven;
* include the 4 cm cube at a range of distances and angles;
* include at least one frame with **nothing** in the workspace, so the false
  positive rate is measurable.

## 5. Measure

```bash
python tools/capture_real_frames.py --eval data/real/ --report runs/real/eval.json
```

The report has, for the real set and for the rendered set side by side:

* detection rate (and the false positive rate on the empty frames);
* XY localisation error against the board's known corners, in millimetres;
* the height estimate error against the measured object heights;
* the difference between the two sets - the **sim-to-real gap**, which is the
  number that matters.

The gap is expected to be large. The purpose of measuring it is to know how
large, not to pretend it is small.

---

## What this does and does not establish

**It does establish** that the calibration code, the detector adapter and the
height estimator work on real images, and it quantifies the sim-to-real gap.

**It does not establish** anything about the arm. The arm is not assembled, and
no claim in this repository depends on it.
