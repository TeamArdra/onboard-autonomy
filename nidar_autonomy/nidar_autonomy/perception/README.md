# `nidar_autonomy/perception/` — development perception pipeline

Status: **development infrastructure, added 2026-09-10.** A read-only
ROS 2 observer subsystem — camera capture → pluggable person-detector →
normalized detection contract → ROS → (consumed by `custom-gcs`). Uses a
**pretrained, non-NIDAR** model as a development stand-in so the rest of
the pipeline (camera abstraction, transport, GCS UI) can be built and
tested now, before a real NIDAR-trained detector exists. See
`../../CLAUDE.md` ("Perception (development)") for how this fits into
the repo as a whole, and `custom-gcs/docs/DATA_MODELS.md` §5A / §5 for
why this is explicitly **not** the same thing as `/vision/survivors`.

## 1. Architecture

```
CAMERA (V4L2 / ROS Image topic / synthetic test source)
   |
   v
CameraSource.read() -> Frame  (camera_source.py)
   |
   +---------------------------+
   |                           |
   v                           v
MJPEGStreamServer          PersonDetector.detect(frame) -> list[Detection]
(video_stream.py,          (detector.py — the swap point)
 stdlib http.server,            |
 :8090/stream.mjpg,             v
 independent of ROS)      HuggingFacePersonDetector   (models/huggingface_person_detector.py)
   |                       — or, later —
   |                       NIDARPersonDetector          (models/nidar_detector.py, not yet written)
   |                           |
   |                           v
   |                      Detection (detection_types.py) — stable, normalized
   |                           |
   v                           v
Browser <img> directly    /perception/detections, /perception/status
(bypasses ROS/backend,    (std_msgs/String + JSON, published by perception_node.py)
 per D-6)                      |
                                v
                      custom-gcs FastAPI backend (ros_client.py caches latest)
                                |
                                v
                      GET /api/perception/detections, /status, /api/camera/status
                                |
                                v
                      React GCS: CameraPanel.tsx (video + bbox overlay),
                                 PerceptionPanel.tsx (person list/counts)
```

`perception_node.py` is the only ROS-facing piece; everything else
(`camera_source.py`, `detector.py`, `detector_config.py`,
`detection_types.py`, `video_stream.py`, `models/`) is plain,
ROS-free Python, independently unit-tested (`nidar_autonomy/test/
test_camera_source.py`, `test_*_detector.py`, `test_detection_types.py`,
`test_video_stream.py`, `test_detector_config.py`) — only
`test_perception_node.py` needs `rclpy` (it skips cleanly if ROS isn't
sourced).

**Never touches mavros, `mission_state_node.py`, `flight_command.py`,
`arming_guard.py`, or `arm_trigger.py`.** Like the other observer nodes
(`telemetry_bridge_node.py`, `coverage_tracker_node.py`,
`frontier_explorer_node.py`, `geofence_monitor_node.py`), this is
read-only: it makes no flight decisions and issues no commands. Safe to
run at any time, armed or disarmed, real vehicle or none at all.

## 2. Model — what, why, how to replace it

| | |
|---|---|
| **Name** | YOLO11n (Ultralytics YOLO11, "nano" size) |
| **Source** | Hugging Face Hub, repo `Ultralytics/YOLO11`, file `yolo11n.pt` — fetched via `huggingface_hub.hf_hub_download()`, run through the already-installed `ultralytics` package |
| **Why this one** | COCO-pretrained CNN (~2.6M params) — realistic for Jetson inference, unlike a ViT/transformer detector (e.g. `hustvl/yolos-tiny` via `transformers`), which would add a whole new, heavier, un-installed dependency stack with a poor TensorRT export story. This machine already has `torch 2.8.0`, `torchvision 0.23.0`, `ultralytics 8.4.15`, `opencv-python 4.11.0` installed and working (JetPack R36, CUDA 12.6) — the lowest-risk realistic choice, not a benchmark-driven pick. |
| **Expected input** | BGR `numpy.ndarray` frame (`Frame.image`), any resolution — resized internally to `max_input_size` (default 640px long side) by `ultralytics` |
| **Output format** | Filtered to COCO class id 0 (`"person"`) only, converted to this package's own `Detection` objects — see §3 |
| **Confidence threshold** | `PerceptionConfig.confidence_threshold`, default `0.5`, passed straight through to `ultralytics`' `conf=` inference argument |
| **Approx. compute** | A few million parameters, single-frame CNN inference — orders of magnitude cheaper than a ViT detector at similar accuracy; exact Jetson fps was not benchmarked this session (no sustained real-camera run performed — see "Not hardware-tested" in the top-level report) |
| **Offline runtime behavior** | `PerceptionConfig.allow_model_download` defaults to `False` — `HuggingFacePersonDetector` then calls `hf_hub_download(..., local_files_only=True)`, meaning **normal node startup never touches the network**; it only succeeds if the weights are already cached (or `model_local_path` points at a local file directly, which needs *no* Hugging Face Hub interaction at all, ever). A one-time download (`allow_model_download=True`, or a separate manual `hf_hub_download`/`huggingface-cli download` call) is a deliberate, explicit dev-machine setup step, not something that happens implicitly at runtime. |
| **How to configure** | ROS params under the `perception.*` namespace (see `detector_config.py`'s `PerceptionConfig` — every field is a ROS param of the same name), e.g. `ros2 run nidar_autonomy perception_node --ros-args -p perception.detector_backend:=huggingface -p perception.confidence_threshold:=0.6 -p perception.model_local_path:=/home/ardra/models/yolo11n.pt` |

### Replacing it with the real NIDAR-trained model, later

1. Implement `models/nidar_detector.py`: a `NIDARPersonDetector(PersonDetector)` with the same `detect(frame) -> list[Detection]` signature, `ready`, `name`.
2. Add a `"nidar"` branch to whatever constructs the detector in `perception_node.py` (currently `"mock"` → `MockPersonDetector`, `"huggingface"` → `HuggingFacePersonDetector`).
3. Set `perception.detector_backend:=nidar` (config change only).

**Nothing else changes** — not `detection_types.py`'s `Detection` contract, not `topics.py`'s topic names, not the JSON wire format, not `custom-gcs`'s backend or frontend. That's the entire point of the `PersonDetector` abstraction in `detector.py`.

### Not used, but noted for the record

`~/hawki_yolo11n.pt` and `~/cognizance2026/` (a separate, unrelated git repo on this machine) contain YOLO weights/scripts from a prior project. Not used here — provenance, exact training data/class set, and license weren't verified in-session, and this task specifically asked for a documented Hugging-Face-sourced model. Worth evaluating properly before ever treating them as a fit.

## 3. Detection contract (`detection_types.py`)

`Detection.to_dict()` — the JSON shape published on `/perception/detections` (nested inside a `detections: [...]` array) and consumed verbatim by `custom-gcs/gcs/backend/app/schemas.py`'s `DetectionResponse`:

```json
{
  "detection_id": "a1b2c3d4...",
  "class_name": "person",
  "confidence": 0.94,
  "bbox": {"x_min": 120, "y_min": 40, "x_max": 260, "y_max": 400},
  "frame_width": 640,
  "frame_height": 480,
  "timestamp": 1234567890.123,
  "track_id": null,
  "center_x": 190,
  "center_y": 220,
  "source": "huggingface",
  "model_name": "Ultralytics/YOLO11/yolo11n.pt"
}
```

`bbox` is **pixel coordinates in image space** (`0..frame_width`,
`0..frame_height`) — never meters, never map-frame. This package does
not and must not invent a world/map coordinate from a bounding box; that
would require a real localization fusion step (SLAM pose + detection)
that doesn't exist yet, and is exactly what `/vision/survivors`
(`custom-gcs/docs/DATA_MODELS.md` §5) is reserved for once it does.
`class_name="person"` means *a pretrained model thinks this is a
person*, not *a confirmed survivor* — the GCS UI is required to keep
those two claims visually distinct (see `PerceptionPanel.tsx` vs.
`SurvivorsPanel.tsx`).

## 4. Camera configuration

`PerceptionConfig.camera_backend` selects the `CameraSource`
implementation (`camera_source.py`):

- `"synthetic"` (default) — `SyntheticCameraSource`, deterministic
  generated frames, no hardware, no OpenCV camera call. Used by default
  in tests and for pipeline development without a camera attached.
- `"v4l2"` — `V4L2CameraSource(camera_device)`, wraps
  `cv2.VideoCapture(device)`. The real path for a USB webcam or a
  V4L2-exposed CSI camera on the Jetson. Never raises on a missing
  device — `is_connected` just reports `False` and `/perception/status`
  reflects it honestly.
- `"ros_image"` — `ROSImageCameraSource(node, camera_ros_topic)`,
  subscribes an existing `sensor_msgs/Image` topic (manual `bgr8`/
  `rgb8`/`mono8` decode, no `cv_bridge` dependency) — for a future setup
  where something else already publishes frames to ROS.

Swapping camera hardware (USB → CSI → a different ROS image source)
means picking a different `camera_backend` / writing one more
`CameraSource` implementation — the detector and everything downstream
is unaffected, by the same abstraction principle as the model swap
above.

## 5. Runtime dependencies

- Already installed on this machine (do **not** `pip install` these —
  they're Jetson-matched CUDA builds; a generic PyPI reinstall can
  silently break CUDA support): `torch`, `torchvision`, `ultralytics`,
  `opencv-python` (`cv2`), `numpy`.
- New, added this session: `huggingface_hub` (`pip install --user
  huggingface_hub`) — small, pure-Python, no special aarch64 build
  needed. See `requirements.txt` in this directory.
- `MJPEGStreamServer` (`video_stream.py`) uses only the Python standard
  library (`http.server`) — no new dependency for video streaming.

## 6. Real vs. development, explicitly

| | This package, today | Future |
|---|---|---|
| Model | Pretrained YOLO11n (COCO, generic) | NIDAR-trained (`models/nidar_detector.py`, not written yet) |
| Meaning of a detection | "a pretrained model thinks this looks like a person" — development integration signal | Same contract, but trained on the actual competition domain |
| `/vision/survivors` | Not published by this package (and never will be) | A separate fusion step (detection + SLAM pose) — still not started anywhere in this repo |
| GCS label | "PERSON DETECTED" (`PerceptionPanel.tsx`) | "SURVIVOR CONFIRMED" (`SurvivorsPanel.tsx`, once real) |

Do not read "the perception pipeline exists" as "survivor
detection/localization (Checkpoint 9) is done" — it is real, necessary
groundwork for that checkpoint, not the checkpoint itself.
