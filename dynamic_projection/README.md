# Dynamic Projection Calibration

## Overview

This project calibrates a **camera + projector** pair so projected content can follow a planar calibration board in real time.

The implemented workflow is:

1. **Camera calibration**
2. **Static projector calibration**
3. **Dynamic projector calibration**
4. **Tracking / aligned projection**

The system uses a printed chessboard as the reference plane and an asymmetric circle grid as the projected pattern.

## Build and Run (Windows)

Prerequisites:

- CMake 3.26 or newer and a C++17-capable Visual Studio installation
- OpenCV development files
- gflags development files
- A webcam and a second display connected to the projector

Configure the project from the repository root. CMake discovers gflags from its installed package prefix. If gflags is not installed in a standard search location, add its install prefix to `CMAKE_PREFIX_PATH` (as shown below):

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 `
  -DOpenCV_DIR="C:/path/to/OpenCVConfig.cmake/directory" `
  -DCMAKE_PREFIX_PATH="C:/path/to/gflags-install"
cmake --build build --config Release
```

Run the Release executable. By default, calibration files are written beside the executable:

```powershell
.\build\Release\dynamic_projection.exe
```

The application opens `ProjectionWindow` fullscreen on a display positioned to the right of the primary 1920-pixel-wide display. Press `Esc` to exit.

Both calibration files include their solved intrinsic parameters, per-view reprojection errors, per-view extrinsics, and detected image points. After dynamic calibration succeeds, `projector_params.xml` also includes the final camera-to-projector rotation vector, rotation matrix, translation vector, and stereo RMS error. Set `--calibration_output_dir` to write the files elsewhere.

## Calibration Process

### Sample Capture Gate

For camera, static-projector, and dynamic-projector calibration, the app accepts a sample only after it detects the chessboard. The first valid sample in each phase is accepted immediately; it does **not** wait for `--delay_between_frames` or require board motion. Each later sample requires both:

- the configured minimum interval since the previous accepted sample
- chessboard-corner RMS movement at or above `--minimum_board_motion_px`

Camera and static-projector calibration collect `--calibration_candidate_margin` extra accepted views before selecting their final retained set. Candidate views first pass a robust per-view RMS gate based on the median and median absolute deviation, capped by the configured per-view RMS limit. The retained views are then chosen for diversity across detected-pattern position, board depth, tilt, and roll. Calibration continues collecting views until selection-time coverage checks pass and the final recalibration meets the aggregate-RMS limit.

Position coverage uses the chessboard-corner centroid for camera calibration, but the detected projected-circle centroid for static-projector calibration. It measures the largest pairwise image-space centroid separation after normalizing X by image width and Y by image height; it does not require coverage of particular image regions. Moving only the printed board within an unchanged plane may therefore provide little static-projector position variation.

Depth coverage uses the far-to-near ratio of the absolute board-origin Z translation, and tilt coverage uses the largest pairwise board-normal angle. Camera depth and tilt checks use preliminary calibration poses and are not repeated after the final camera refit. Static-projector checks use the camera-estimated board poses. These are selection-time coverage tests, not a guarantee about poses recomputed with the final camera intrinsics.

### 1. Camera Calibration

The camera is calibrated first from multiple views of a printed chessboard.

For each accepted frame:

- chessboard corners are detected in the camera image
- known 3D board points are paired with 2D image points
- OpenCV `calibrateCamera()` estimates the camera intrinsics

The standard camera model is:

\[
s
\begin{bmatrix}
u \\
v \\
1
\end{bmatrix}
=
K
\begin{bmatrix}
R & t
\end{bmatrix}
\begin{bmatrix}
X \\
Y \\
Z \\
1
\end{bmatrix}
\]

Where:

- \((X,Y,Z)\) is a 3D point in board/world coordinates
- \((u,v)\) is the image point
- \(K\) is the camera intrinsic matrix
- \(R,t\) are the board pose relative to the camera

The code rejects camera RMS outliers, selects `--minimum_frames` diverse views from the remaining candidate pool, and recalibrates before saving the final calibration.

### 2. Static Projector Calibration

The projector displays a fixed asymmetric circle grid on the same physical plane as the printed chessboard.

For each accepted frame:

- the printed chessboard is detected
- the projected circle grid is detected
- the board pose is estimated from the camera calibration
- projected circle image points seen by the camera are back-projected onto the board plane

That gives 3D points on the board plane and their corresponding 2D projector image coordinates. These are used to calibrate the projector intrinsics with `calibrateCamera()`.

The projector is treated like an inverse camera:

\[
s
\begin{bmatrix}
u_p \\
v_p \\
1
\end{bmatrix}
=
K_p
\begin{bmatrix}
R_p & t_p
\end{bmatrix}
\begin{bmatrix}
X \\
Y \\
Z \\
1
\end{bmatrix}
\]

Where:

- \((u_p,v_p)\) is a projector pixel
- \(K_p\) is the projector intrinsic matrix

This phase also rejects projector RMS outliers, selects a diverse retained set, and enforces an aggregate projector RMS limit before accepting the static calibration.

### 3. Stereo Camera-Projector Calibration

Once camera intrinsics and projector intrinsics are available, the system estimates the relative transform between camera and projector with `stereoCalibrate(..., CALIB_FIX_INTRINSIC)`.

The relative transform is:

\[
X_p = R_{cp} X_c + t_{cp}
\]

Where:

- \(X_c\) is a point in camera coordinates
- \(X_p\) is the same point in projector coordinates
- \(R_{cp}, t_{cp}\) are the camera-to-projector extrinsics

In this project:

- projector intrinsics are solved in the static phase
- projector intrinsics are then frozen
- dynamic refinement uses stereo calibration quality as the main completion metric

### 4. Dynamic Projector Calibration

During dynamic calibration the projected circle pattern follows the board.

The code:

- detects the chessboard in the current camera frame
- estimates the current board pose
- smooths that pose
- projects the target circle points into projector image coordinates using the current board pose and the camera-to-projector transform

Projection is done by composing board-to-camera and camera-to-projector transforms:

\[
R_{bp} = R_{cp} R_{bc}, \qquad
t_{bp} = R_{cp} t_{bc} + t_{cp}
\]

Then:

\[
s
\begin{bmatrix}
u_p \\
v_p \\
1
\end{bmatrix}
=
K_p
\begin{bmatrix}
R_{bp} & t_{bp}
\end{bmatrix}
\begin{bmatrix}
X \\
Y \\
Z \\
1
\end{bmatrix}
\]

The dynamic phase ends only when:

- enough dynamic samples have been collected
- at least one dynamic solution has succeeded
- the stereo RMS error is below the configured threshold

### 5. Tracking

In tracking mode the projected circles are placed directly on the chessboard instead of being offset away from it.

## Important Physical Assumption

During calibration, the printed chessboard and the projected circles must lie on the **same rigid plane**.

If the circles are projected onto a different plane than the marker, the 3D reconstruction used by projector calibration becomes invalid.

## Supported Parameters

The application currently supports these command-line parameters from [`include/flags.hpp`](include/flags.hpp):

| Parameter | Default | Meaning |
|---|---:|---|
| `--pattern_width` | `9` | Number of inner corners across the calibration board width |
| `--pattern_height` | `6` | Number of inner corners across the calibration board height |
| `--num_boards_before_dynamic_projector_calib` | `8` | Number of retained static-projector views required before dynamic calibration |
| `--num_boards_final_projector_calib` | `5` | Minimum dynamic accepted samples target used for completion |
| `--minimum_frames` | `8` | Number of retained camera calibration views |
| `--delay_between_frames` | `1000` | Minimum interval in milliseconds after the first accepted sample in a phase |
| `--minimum_board_motion_px` | `15.0` | Minimum chessboard-corner RMS movement in pixels after the first accepted sample in a phase |
| `--calibration_output_dir` | Executable directory | Directory for `camera_params.xml` and `projector_params.xml` |
| `--projector_offset_y_scale` | `0.9` | Offset scale used during dynamic calibration to place the projected circle grid away from the marker |
| `--projected_circle_radius` | `30` | Radius of each projected circle in projector pixels |
| `--projector_smoothing_rate` | `0.4` | Smoothing factor for dynamic pose and projector-point updates |
| `--max_dynamic_stereo_rms` | `3.0` | Maximum stereo RMS error allowed before entering tracking mode |
| `--calibration_candidate_margin` | `4` | Extra camera and static-projector candidates collected before view selection |
| `--max_camera_per_view_rms` | `2.0` | Absolute per-view camera RMS eligibility limit |
| `--max_camera_rms` | `1.5` | Maximum final aggregate camera RMS |
| `--max_projector_per_view_rms` | `3.0` | Absolute per-view projector RMS eligibility limit |
| `--max_projector_rms` | `2.5` | Maximum final aggregate projector RMS |
| `--minimum_calibration_position_span` | `0.25` | Minimum normalized detected-pattern centroid span at view selection |
| `--minimum_calibration_distance_ratio` | `1.1` | Minimum far-to-near board-origin depth ratio at view selection |
| `--minimum_calibration_orientation_span_deg` | `10.0` | Minimum board-normal angular span at view selection |
| `--h` | `false` | Print usage |

## Default Launch Command

Example launch command using the current defaults:

```powershell
.\build\Release\dynamic_projection.exe `
  --pattern_width=9 `
  --pattern_height=6 `
  --num_boards_before_dynamic_projector_calib=8 `
  --num_boards_final_projector_calib=5 `
  --minimum_frames=8 `
  --delay_between_frames=1000 `
  --minimum_board_motion_px=15.0 `
  --projector_offset_y_scale=0.9 `
  --projected_circle_radius=30 `
  --projector_smoothing_rate=0.4 `
  --max_dynamic_stereo_rms=3.0 `
  --calibration_candidate_margin=4 `
  --max_camera_per_view_rms=2.0 `
  --max_camera_rms=1.5 `
  --max_projector_per_view_rms=3.0 `
  --max_projector_rms=2.5 `
  --minimum_calibration_position_span=0.25 `
  --minimum_calibration_distance_ratio=1.1 `
  --minimum_calibration_orientation_span_deg=10.0
```

## Important Notes About Defaults

Some important setup values are still hardcoded in `main.cpp` rather than exposed as flags:

- the printed board square size currently selects either A3 or A4 by editing source
- the projector output resolution is currently set to `1920x1080`
- the projected asymmetric circle pattern size is currently set to `4x5`

At the moment the active board-square size in `main.cpp` determines whether you are calibrating for an A3 or A4 print.

## Accuracy Tips By Phase

### Camera Calibration

- Use a large printed board if possible. A larger board gives more stable corner localization.
- Capture views with varied tilts, distances, and image positions.
- Do not use only floor-parallel views.
- Ensure the entire chessboard is visible.
- Keep the board still when a frame is accepted.

### Static Projector Calibration

- The full chessboard and the full projected circle grid should both be visible.
- Keep both on the same planar surface.
- Avoid clipping the projected circles at the board edge or image edge.
- If possible, keep projected light from washing out the chessboard corners.
- Use several tilted views, not only fronto-parallel views.

### Dynamic Projector Calibration

- Use the offset circle pattern to avoid disturbing chessboard detection too much.
- Hold the board still long enough for reliable detection, then move it between accepted samples. After the first sample, the capture gate requires the configured minimum corner movement (`--minimum_board_motion_px`) as well as the configured delay.
- Include meaningful tilt, not just translation on the floor plane.
- Cover a range of board positions in the camera and projector field of view.
- If tracking looks good only when the board is flat, you likely need more tilted samples.

### Tracking / Final Validation

- A good result should align not only on the floor plane but also when the board tilts.
- If alignment is good only on one plane, the remaining error is usually in the 3D calibration, not the 2D display logic.
- A stable but consistently wrong result usually suggests calibration bias.
- A jittery result usually suggests pose-estimation noise or unstable detections.

## Debug Windows

The app uses:

- `ProjectionWindow`: projector output only
- `CameraDebug`: raw webcam image with detected chessboard and board axes overlay
- `ImageThresholded`: thresholded projector-pattern detection view

The debug windows are sized to `640x360`.

## Current Practical Recommendation

For best results:

- use the larger board if detection is more stable
- collect many tilted views
- keep the board and projected pattern on the same rigid plane
- avoid projector bloom on the marker
- verify the final alignment while tilting the board, not only while sliding it on the floor
