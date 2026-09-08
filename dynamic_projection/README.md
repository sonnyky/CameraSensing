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

Configure the project from the repository root. Replace the two dependency paths with the locations on your machine:

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 `
  -DOpenCV_DIR="C:/path/to/OpenCVConfig.cmake/directory" `
  -Dgflags_DIR="C:/path/to/gflags-install/lib/cmake/gflags"
cmake --build build --config Release
```

Run the Release executable from the repository root so the default calibration files are written there:

```powershell
.\build\Release\dynamic_projection.exe
```

The application opens `ProjectionWindow` fullscreen on a display positioned to the right of the primary 1920-pixel-wide display. Press `Esc` to exit.

## Calibration Process

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

The code keeps the best camera views by per-view reprojection RMS before saving the final calibration.

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

This phase also rejects poor projector views by reprojection error before accepting the static calibration.

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
\begin{bmatrix}
R_{bp} & t_{bp}
\end{bmatrix}
=
\begin{bmatrix}
R_{cp} & t_{cp}
\end{bmatrix}
\begin{bmatrix}
R_{bc} & t_{bc}
\end{bmatrix}
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

The application currently supports these command-line parameters from [`include/flags.hpp`](C:/Users/Sonny/Desktop/Workspace/CameraSensing/dynamic_projection/include/flags.hpp):

| Parameter | Default | Meaning |
|---|---:|---|
| `--pattern_width` | `9` | Number of inner corners across the calibration board width |
| `--pattern_height` | `6` | Number of inner corners across the calibration board height |
| `--pattern_type` | `chessboard` | Calibration board type used for camera calibration |
| `--num_boards_before_dynamic_projector_calib` | `8` | Minimum accepted projector samples before entering dynamic calibration |
| `--num_boards_final_projector_calib` | `5` | Minimum dynamic accepted samples target used for completion |
| `--minimum_frames` | `8` | Minimum accepted camera calibration images |
| `--delay_between_frames` | `1000` | Delay in milliseconds between accepted calibration frames |
| `--camera_filename` | `camera_params.xml` | Output file for camera intrinsics |
| `--projector_filename` | `projector_params.xml` | Output file for projector intrinsics |
| `--projector_offset_y_scale` | `0.9` | Offset scale used during dynamic calibration to place the projected circle grid away from the marker |
| `--projected_circle_radius` | `30` | Radius of each projected circle in projector pixels |
| `--projector_smoothing_rate` | `0.4` | Smoothing factor for dynamic pose and projector-point updates |
| `--write_points` | `false` | Save detected image points to output calibration files |
| `--write_extrinsics` | `true` | Save extrinsic data to output calibration files |
| `--h` | `false` | Print usage |

## Default Launch Command

Example launch command using the current defaults:

```powershell
.\build\Release\dynamic_projection.exe `
  --pattern_width=9 `
  --pattern_height=6 `
  --pattern_type=chessboard `
  --num_boards_before_dynamic_projector_calib=8 `
  --num_boards_final_projector_calib=5 `
  --minimum_frames=8 `
  --delay_between_frames=1000 `
  --camera_filename=camera_params.xml `
  --projector_filename=projector_params.xml `
  --projector_offset_y_scale=0.9 `
  --projected_circle_radius=30 `
  --projector_smoothing_rate=0.4 `
  --write_points=false `
  --write_extrinsics=true
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
- Move the board slowly and let accepted frames happen only when the board is stable.
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
