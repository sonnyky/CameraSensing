# Dynamic Projection Calibration

## Overview

This project calibrates a **camera + projector** pair so projected content can follow a planar calibration board in real time.

The implemented workflow is:

1. **Camera calibration**
2. **Static projector calibration**
3. **Dynamic projector calibration**
4. **Tracking / aligned projection**

The system uses a printed chessboard as the reference plane and an asymmetric circle grid as the projected pattern.

The default printed marker has **9 × 6 inner corners** and **3.6 cm (36 mm) square side length**. This corresponds to 10 × 7 squares, not 9 × 6 squares. Measure the actual printed square size; paper format alone does not determine calibration scale. Board coordinates and saved translations use millimeters.

## Build and Run (Windows)

Prerequisites:

- CMake 3.26 or newer and a C++17-capable Visual Studio installation
- OpenCV 4.7 or newer development files, including the `objdetect` module for ArUco detection
- gflags development files
- A webcam and a second display connected to the projector

Configure the project from the repository root. CMake discovers gflags from its installed package prefix. If gflags is not installed in a standard search location, add its install prefix to `CMAKE_PREFIX_PATH` (as shown below):

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 `
  -DOpenCV_DIR="C:/path/to/OpenCVConfig.cmake/directory" `
  -DCMAKE_PREFIX_PATH="C:/path/to/gflags-install"
cmake --build build --config Release
```

Hardware-independent regression tests are built by default. Run them with:

```powershell
.\build\Release\projector_calibration_tests.exe
# Alternatively, use the subdirectory where the test is registered:
ctest --test-dir build/dynamic_projection -C Release --output-on-failure
```

They cover default/compact static layouts, clipping rejection, actual synthetic circle detection (fixed threshold, Otsu, blob-area filtering and untouched inputs), clean-frame isolation, projection bounds, displayed-pattern snapshots, stereo-fit validation, distortion-corrected board-plane reconstruction at multiple depths, invalid-ray rejection, board-centroid selection, per-view RMS gating, reset behavior, and synthetic projector calibration/refitting with known intrinsics. They do not replace real-board detection and alignment validation. Set `-DDYNAMIC_PROJECTION_BUILD_TESTS=OFF` to omit the test target.

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

Board-position coverage uses the chessboard-corner centroid in both phases, measuring the largest pairwise separation after normalizing X by camera image width and Y by height. Camera selection requires `--minimum_calibration_position_span` (0.25); static-projector selection uses its own `--minimum_projector_board_position_span` (0.10). Static-projector selection additionally checks the normalized diagonal of the bounding box of selected circle centers in **projector pixels**, using `--minimum_projector_pattern_span` (0.20). This is pattern footprint, not circle-centroid movement in the camera. A fixed grid does not need to move across the camera image to pass board-pose diversity. Footprint alone does not guarantee full projector field-of-view coverage or accurate distortion outside that footprint.

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

#### Fitting the static grid to a limited-size board

Keep the camera and projector fixed and move only the board. The fixed 4x5 asymmetric grid can be resized with `--static_projector_spacing_px` and repositioned using `--static_projector_center_x` / `--static_projector_center_y` (fractions of projector image width/height). Defaults preserve the original centered grid: spacing 120 pixels and radius 30 pixels, approximately 900x540 pixels including circle edges.

For a compact starting point on your panel, run from the repository root:

```powershell
.\build\Release\dynamic_projection.exe `
  --static_projector_spacing_px=60 `
  --projected_circle_radius=15 `
  --static_projector_center_x=0.5 `
  --static_projector_center_y=0.5
```

This gives approximately 450x270 projector pixels including circle edges. Adjust the center so the circles land on the white area beside the marker. Pixel footprint is not a physical board measurement: check the actual projection. These flags change static-grid coordinates consistently in rendering and calibration; they do not change the dynamic board-coordinate grid spacing or its offset. The radius flag applies in every phase. Restart calibration after changing flags; do not move/resize the displayed grid during a run.

Use the largest grid that leaves room for different tilts and distances. Very small circles are harder to detect, and a small projector footprint constrains less of its optics. Circles that touch, a static grid clipped by the projector image, or a grid below the minimum projector footprint are rejected before frame collection. The RMS limits, depth ratio and tilt requirements are unchanged; the separate projector board-position default is deliberately less restrictive than the camera default.

#### Understanding capture and selection messages

`current projector candidate count: 3/12` means only three samples have been captured; diversity and RMS selection have not run yet. Capture-rejection messages (throttled to at most one every two seconds between accepted samples) distinguish missing chessboard, missing complete circle grid, waiting for the sample interval, and insufficient corner movement. Check `ImageThresholded` when the circle grid is missing. The capture gate checks movement before attempting circle detection, so a waiting/movement message does not confirm that circles were detected.

After the candidate target is reached, separate messages report insufficient RMS-eligible views, insufficient board translation, depth or tilt, and insufficient projector-image footprint. A footprint failure cannot be fixed by collecting more views of the same fixed grid: enlarge the spacing and restart. A final aggregate RMS failure remains a separate rejection. Preliminary fit RMS can exceed the final completion limit because selection/refitting has not yet completed.

Each fit logs the preliminary per-view RMS and eligibility/selection status for every candidate, using 1-based capture numbers. The robust eligibility threshold can be stricter than the absolute per-view limit. With too few eligible views, coverage is explicitly **not evaluated** rather than printing default zero spans. The final refit logs each retained candidate's RMS with its original capture number. Preliminary values are recomputed as new samples arrive; they are residuals of the joint model, not independent measurements of frame quality. Persistent errors across most views can indicate camera-calibration error, board non-flatness or detection bias as well as poor images. The eight-view target and final RMS limits are not relaxed.

#### Circle-detection diagnostics and tuning

Circle detection operates on an unannotated, inverse-thresholded camera image. Chessboard/debug graphics are drawn only **after** circle detection. `ImageThresholded` then overlays red blob candidates and their count; these include clutter and do not imply a valid grid. Detection requires the correct ordered 4x5 grid, not simply 20 arbitrary blobs. On frames rejected by the chessboard/time/movement gate, circle detection is not attempted and no blob overlay is shown.

The default brightness threshold remains 210/255. Bright projected circles should appear as isolated black disks on a white background in the thresholded image. If they vanish, lower `--projector_detection_threshold`; if they merge into bright surrounding board regions, adjust exposure/lighting or the threshold. `--projector_detection_threshold=-1` enables global Otsu thresholding, but the printed chessboard and scene background can dominate its histogram, so it is not guaranteed to work better.

Blob-area limits are in **camera-image pixels squared**, not projector pixels or board units. Defaults match the previous detector: min area 25, max area 5000, inertia ratio 0.1, convexity 0.95, circularity filtering disabled. Smaller/farther circles may need a lower minimum area; larger/nearer circles may need a higher maximum. Tilt creates elliptical blobs: overly strict shape filters can reject valid circles. Zero disables a shape filter, but relaxing filters also admits clutter. Change settings between runs, not during calibration.

Example starting point for a compact pattern whose captured circles are below brightness 210 (150 is illustrative; choose it from the actual thresholded view):

```powershell
.\build\Release\dynamic_projection.exe `
  --static_projector_spacing_px=60 `
  --projected_circle_radius=15 `
  --projector_detection_threshold=150
```

These detection changes do not alter the camera/projector calibration equations or RMS limits, but different measured centers can change the numerical calibration result. Verify final alignment using real-board poses after tuning.

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
- uses that pose immediately, without smoothing during calibration
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

Tracking keeps the projected circles in the same white-panel region used during dynamic calibration. Entering tracking changes smoothing, not the intended physical placement.

During both dynamic calibration and tracking, the grid is centered inside the configured physical projection region below the chessboard. The default region is approximately 37 cm wide by 29 cm high, with its top edge 25.5 cm below the intended top-left inner corner and its horizontal center at board X=14.4 cm. Board X increases rightward and Y downward on the upright print. The region bounds are X=[-41,329] mm and Y=[255,545] mm. With the unchanged 36 mm asymmetric-grid spacing, circle centers span X=[18,270] mm and Y=[328,472] mm. Allow additional margin for circle radii. These hand-measured dimensions control placement, not calibration scale; square size still defines scale. Static-grid placement, physical scale and RMS limits remain unchanged; distortion-fitting changes are described below. The legacy offset scale applies to both phases only with `--use_projection_region=false`.

### Board origin and projection-region setup

The default orientation anchors use `DICT_6X6_50` (dictionary number 8): ID **0** in the white square above/right of the intended top-left inner corner, and ID **1** in the white square below/left. Their expected centers are respectively (0.5,-0.5) and (-0.5,0.5) square lengths relative to that corner. This matches the glued marker layout decoded from the September 17, 12:46 photo. Larger nested 6x6 dictionaries share these codes, so the original dictionary size cannot be inferred from a photo alone.

At startup, show the entire chessboard and at least one readable anchor marker. Move closer if necessary, without cropping any inner corners. Check the yellow `(0,0)` origin in `CameraDebug` against the intended physical corner. After camera intrinsics are available, a cyan outline shows the configured projection region; verify it matches the white panel. The grid is centered in that region, not at arbitrary image coordinates.

The origin is the marked physical corner, not the image's top-left corner. The board may be rotated at initialization or re-acquisition, including 90, 180 and 270 degrees. All four grid-axis directions are evaluated against the marker locations. Once anchored, continuous full-chessboard detections maintain corner ordering even when the small markers are no longer readable. Rotate gradually while markers are unreadable; excessive motion, ambiguous ordering, conflicting markers, or loss of the complete chessboard clears the anchor. Show a readable marker together with the entire chessboard again to resume, in any orientation. This is conservative frame-to-frame ordering continuity, not tracking an invisible or partially occluded board. Small-marker detection retries at double image scale; blur or insufficient source pixels still requires bringing the marker closer. Failure diagnostics distinguish readable anchor count from missing complete chessboard detection. Detected anchor codes are masked white in a private detection image because they are pasted on white squares, reducing interference with chessboard detection and corner refinement. Grid ordering supports axis reversals, but optically mirrored ArUco codes are not decoded by the standard detector; use an unmirrored camera feed.

Region placement does not automatically detect panel edges, resize the grid, mask background blobs, or guarantee that a particular board pose fits inside the projector image. Keep the entire grid on the same rigid plane and within the projector's field of view. `--require_board_orientation=false` permits legacy unanchored chessboard detection, but cannot guarantee the intended physical origin.

On Windows, builds using shared OpenCV libraries copy the required OpenCV runtime DLLs beside the executable, including the new object-detection dependency.

The dynamic projection update uses a clean copy of the captured frame, isolated from calibration/debug overlays, and does not annotate that copy. The capture loop first displays the queued projection, processes display events and waits `--projector_settle_ms` (100 ms), records an independent snapshot of its projector coordinates, discards `--projector_discard_frames` (2) buffered camera frames, then acquires the measurement frame. Blank displays have no recorded correspondences and cannot contribute calibration samples. New queued coordinates do not overwrite the recorded displayed coordinates. Display/camera latency varies by device; increase settling/discard settings if needed. This is software sequencing and buffering mitigation, not hardware-triggered synchronization.

Back-projection now removes camera lens distortion with `undistortPoints` before intersecting normalized rays with the board plane, using double precision for the intersection. Invalid/grazing rays reject the sample. Stereo fitting uses temporary rotation/translation outputs: only finite fits at or below `--max_dynamic_stereo_rms` (3 pixels) replace the working transform. The initial stereo fit must pass before entering dynamic calibration. A rejected dynamic fit discards its staged sample from the parallel camera/projector arrays; it does not increment the accepted-sample count or update the capture gate. Valid fits can have different RMS as different views are added; the algorithm does not compare RMS from different datasets as though they were identical tests. Stereo refinement fixes the accepted camera/projector intrinsics; the minimum dynamic sample count is unchanged. Start a fresh calibration after rebuilding to use the updated distortion fit and validation.

Dynamic calibration uses the current measured board pose directly, with no pose averaging, pixel-position blending or movement step limit. Every successful update queues the newly computed coordinates directly. Tracking alone smooths one shared board pose using `--projector_smoothing_rate`: translation is blended and rotation follows the shortest relative rotation on SO(3), avoiding rotation-vector wrap-around. All 20 circle centers are projected from that same pose; independent circle blending and 45-pixel movement caps have been removed. Loss of the board clears the tracking smoothing state. Display settling, buffered-frame discard and capture gates remain active; unsmoothed updates do not eliminate acquisition/display latency. Hold the board still during sample capture.

`Projector projection blanked:` identifies missing chessboard detection, missing/invalid calibration parameters, invalid depth, or projection beyond the validated distortion domain. Dynamic calibration additionally blanks a grid whose complete circles do not fit within the projector image, because samples require all 20 points. Tracking allows partial visibility: visible circles are rendered, circles crossing an image edge are clipped by rendering, and wholly off-image circles are skipped. `Projector projection bounds warning:` reports full-circle counts, coordinate ranges and intersecting circle bounds without blanking an otherwise valid tracking projection. Coordinates are not clamped or moved to the boundary, and tracking accumulates no calibration samples. If all circles are off-image, no circles can be displayed. Moving farther can still reach the distortion-domain or depth guards and blank tracking for safety. A grid inside the projector image can still miss the physical board: these diagnostics do not measure panel edges. Messages are throttled separately from sample-capture warnings.

### Projection rejection diagnostics

Projection failures now report an exact reason instead of combining depth, distortion-domain and non-finite failures. The first offending circle is identified using 1-based circle number, row and column in the 4x5 asymmetric grid. The message includes its projector-space XYZ and depth in millimeters, normalized radius `hypot(X/Z,Y/Z)`, and allowed distortion-domain radius. Radius is `N/A` when it cannot safely be evaluated, such as non-positive depth. Depths at or below 0.000001 mm remain rejected; this threshold is unchanged.

Reasons are `non_finite_projector_coordinates`, `non_positive_depth`, `depth_below_minimum`, `outside_distortion_domain`, `non_finite_pixel_projection`, or `empty_grid`. Ordinary clipping uses `screen_clipping`, adding the pixel center, circle radius and crossed screen edges. Only the first offending circle is reported; it need not be the circle with the largest error. Tracking clipping remains a warning, while dynamic-calibration clipping blanks the grid. Repeated messages are throttled to approximately one every two seconds; a changed failure category or blanking/warning transition is reported immediately even within that interval.

Illustrative output (not measurements from your device):

```text
Projector projection blanked: reason=outside_distortion_domain; circle=17/20; row=5; column=1; projector_xyz_mm=(-180.2000,245.7000,420.0000); depth_mm=420.0000; normalized_radius=0.7255; allowed_radius=0.6100
```

These diagnostics do not alter calibration fitting, circle placement, acceptance thresholds or visibility policy.

### Projector distortion validity

Projector intrinsics now fit `k1`, `k2` and tangential distortion while fixing `k3` to zero (`CALIB_FIX_K3`). This reduces poorly constrained sixth-order distortion from the fixed static-grid footprint; it is not a guarantee of accuracy. Camera fitting is unchanged. Existing RMS and viewpoint-coverage limits still apply. Calibration results can change, and a fresh calibration is required after rebuilding.

Before accepting or loading a projector fit, the code checks finite intrinsics, positive focal lengths, a positive radial scale and a strictly increasing distorted radius. Radial polynomial extrema are evaluated over a disk enclosing the image's pinhole-normalized corners with 25% radial margin. The radial/tangential Jacobian is additionally checked at sampled radii and angles. Fits that fold or reverse this mapping are rejected even when their reprojection RMS passes. The sampled tangential check is a practical guard, not a proof of global invertibility or validation against physical ground truth. Failed fits retain candidate views and do not replace the working intrinsic model; rejected files do not replace a valid loaded model.

Projection beyond this validation disk or behind the projector is rejected instead of extrapolating the distortion polynomial unchecked. This deliberately favors a blank output over unstable or scrambled circles. The physical projection region and marker defaults are unchanged. Do not edit an old XML file merely to set `k3=0`: the remaining parameters were jointly fitted with it and must be recalibrated. OpenCV explains why non-monotonic distortion should be treated as calibration failure in its [calibration documentation](https://docs.opencv.org/4.12.0/d9/d0c/group__calib3d.html).

## Important Physical Assumption

During calibration, the printed chessboard and the projected circles must lie on the **same rigid plane**.

If the circles are projected onto a different plane than the marker, the 3D reconstruction used by projector calibration becomes invalid.

## Supported Parameters

The application currently supports these command-line parameters from [`include/flags.hpp`](include/flags.hpp):

| Parameter | Default | Meaning |
|---|---:|---|
| `--pattern_width` | `9` | Number of inner corners across the calibration board width |
| `--pattern_height` | `6` | Number of inner corners across the calibration board height |
| `--board_square_size_cm` | `3.6` | Printed square side length in centimeters; converted internally to millimeters |
| `--num_boards_before_dynamic_projector_calib` | `8` | Number of retained static-projector views required before dynamic calibration |
| `--num_boards_final_projector_calib` | `5` | Minimum dynamic accepted samples target used for completion |
| `--minimum_frames` | `8` | Number of retained camera calibration views |
| `--delay_between_frames` | `1000` | Minimum interval in milliseconds after the first accepted sample in a phase |
| `--minimum_board_motion_px` | `15.0` | Minimum chessboard-corner RMS movement in pixels after the first accepted sample in a phase |
| `--calibration_output_dir` | Executable directory | Directory for `camera_params.xml` and `projector_params.xml` |
| `--require_board_orientation` | `true` | Require a marker anchor initially and after tracking loss |
| `--board_aruco_dictionary` | `8` | OpenCV predefined dictionary number; 8 is DICT_6X6_50 |
| `--board_origin_aruco_id` | `0` | Primary origin marker ID, above/right |
| `--board_origin_secondary_aruco_id` | `1` | Secondary origin marker ID, below/left |
| `--board_origin_marker_x_squares` | `0.5` | Primary marker center X in square lengths |
| `--board_origin_marker_y_squares` | `-0.5` | Primary marker center Y in square lengths |
| `--board_secondary_marker_x_squares` | `-0.5` | Secondary marker center X in square lengths |
| `--board_secondary_marker_y_squares` | `0.5` | Secondary marker center Y in square lengths |
| `--board_origin_marker_tolerance_squares` | `0.6` | Maximum anchor-center matching error in square lengths |
| `--board_tracking_max_motion_squares` | `1.5` | Maximum continuity RMS displacement relative to observed horizontal corner spacing |
| `--use_projection_region` | `true` | Center the dynamic-calibration and tracking grid inside the physical region |
| `--projection_region_width_cm` | `37.0` | Projection-region width |
| `--projection_region_height_cm` | `29.0` | Projection-region height |
| `--projection_region_top_cm` | `25.5` | Region top edge below the anchored origin |
| `--projection_region_center_x_cm` | `14.4` | Region horizontal center relative to the anchored origin |
| `--projector_offset_y_scale` | `0.9` | Legacy dynamic-calibration/tracking board-Y offset; used only when use_projection_region is false |
| `--projected_circle_radius` | `30` | Radius of each projected circle in projector pixels |
| `--static_projector_spacing_px` | `120.0` | Static asymmetric-grid spacing in projector pixels |
| `--static_projector_center_x` | `0.5` | Static grid center as a fraction of projector width |
| `--static_projector_center_y` | `0.5` | Static grid center as a fraction of projector height |
| `--minimum_projector_board_position_span` | `0.10` | Minimum normalized chessboard-centroid span for static-projector selection |
| `--minimum_projector_pattern_span` | `0.20` | Minimum normalized diagonal circle-center footprint in projector pixels |
| `--projector_detection_threshold` | `210.0` | Camera brightness threshold, 0..255; -1 enables global Otsu |
| `--projector_blob_min_area` | `25.0` | Minimum blob area in camera pixels squared |
| `--projector_blob_max_area` | `5000.0` | Maximum blob area in camera pixels squared |
| `--projector_blob_min_circularity` | `0.0` | Minimum blob circularity, 0..1; zero disables |
| `--projector_blob_min_inertia` | `0.1` | Minimum blob inertia ratio, 0..1; zero disables |
| `--projector_blob_min_convexity` | `0.95` | Minimum blob convexity, 0..1; zero disables |
| `--projector_settle_ms` | `100` | Display settling time before camera acquisition, milliseconds (1..5000) |
| `--projector_discard_frames` | `2` | Buffered camera frames discarded after presentation (0..30) |
| `--projector_smoothing_rate` | `0.4` | Shared-pose smoothing factor for tracking only; dynamic calibration updates directly |
| `--max_dynamic_stereo_rms` | `3.0` | Maximum stereo RMS error allowed before entering tracking mode |
| `--calibration_candidate_margin` | `4` | Extra camera and static-projector candidates collected before view selection |
| `--max_camera_per_view_rms` | `2.0` | Absolute per-view camera RMS eligibility limit |
| `--max_camera_rms` | `1.5` | Maximum final aggregate camera RMS |
| `--max_projector_per_view_rms` | `3.0` | Absolute per-view projector RMS eligibility limit |
| `--max_projector_rms` | `2.5` | Maximum final aggregate projector RMS |
| `--minimum_calibration_position_span` | `0.25` | Minimum normalized chessboard-centroid span for camera selection |
| `--minimum_calibration_distance_ratio` | `1.1` | Minimum far-to-near board-origin depth ratio at view selection |
| `--minimum_calibration_orientation_span_deg` | `10.0` | Minimum board-normal angular span at view selection |
| `--h` | `false` | Print usage |

## Default Launch Command

From PowerShell, launch calibration with your marker dimensions:

```powershell
Set-Location "C:\Users\Sonny\Desktop\Workspace\CameraSensing"
.\build\Release\dynamic_projection.exe --pattern_width=9 --pattern_height=6 --board_square_size_cm=3.6
```

These are the defaults, so launching the executable without flags uses the same marker, ArUco IDs, and projection region. Start with a readable ID 1 or 0 and all inner corners visible, then verify the origin overlay before collecting views. The application starts camera calibration, then automatically proceeds through static-projector calibration, dynamic-projector calibration, and tracking. Move the board between accepted samples and vary position, depth, and tilt. With default settings, camera and static-projector selection starts after 12 accepted candidate views each, retaining 8; more candidates may be needed to satisfy quality and coverage checks. Dynamic calibration requires at least 5 accepted samples and the stereo RMS limit. Press `Esc` to exit.

Results are written to `build\Release\camera_params.xml` and `build\Release\projector_params.xml`. Final stereo extrinsics are added only after dynamic calibration completes. Use `--calibration_output_dir="C:/path/to/results"` to select another output directory.

Example launch command using the current defaults:

```powershell
.\build\Release\dynamic_projection.exe `
  --pattern_width=9 `
  --pattern_height=6 `
  --board_square_size_cm=3.6 `
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

The marker defaults are 9 × 6 inner corners and 3.6 cm squares; use the marker flags to match a different physical print. The following setup values are still hardcoded in `main.cpp`:

- the projector output resolution is currently set to `1920x1080`
- the projected asymmetric circle pattern size is currently set to `4x5`

Do not resize the printed marker without updating `--board_square_size_cm` to its measured square side length.

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
- `ImageThresholded`: thresholded projector-pattern view with red blob candidates/count after a circle-detection attempt; successful grid and chessboard annotations are added afterward

The debug windows are sized to `640x360`.

## Current Practical Recommendation

For best results:

- use the larger board if detection is more stable
- collect many tilted views
- keep the board and projected pattern on the same rigid plane
- avoid projector bloom on the marker
- verify the final alignment while tilting the board, not only while sliding it on the floor
