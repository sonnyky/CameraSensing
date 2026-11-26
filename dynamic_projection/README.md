# Dynamic Projection Calibration

## Overview

This project calibrates a **camera + projector** pair so projected content can follow a planar calibration board in real time.

The implemented workflow is:

1. **Camera calibration**
2. **Static projector calibration**
3. **Dynamic projector calibration**
4. **Tracking / aligned projection**

The system uses a printed chessboard as the reference plane and an asymmetric circle grid as the projected pattern.

The default printed marker has **9 × 6 inner corners** and **2.0 cm (20 mm) square side length**. This corresponds to 10 × 7 squares, not 9 × 6 squares. Measure the actual printed square size after printing; paper format alone does not determine calibration scale. Board coordinates and saved translations use millimeters.

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

They cover Gray-code generation, decoding and contrast rejection, actual synthetic circle detection for the dynamic phase, clean-frame isolation, projection bounds, displayed-pattern snapshots, stereo-fit validation, distortion-corrected board-plane reconstruction at multiple depths, invalid-ray rejection, board-centroid selection, per-view RMS gating, reset behavior, and synthetic projector calibration/refitting with known intrinsics. They do not replace real-board detection and alignment validation. Set `-DDYNAMIC_PROJECTION_BUILD_TESTS=OFF` to omit the test target.

Run the Release executable. By default, calibration files are written beside the executable:

```powershell
.\build\Release\dynamic_projection.exe
```

At startup the application enumerates Windows displays, prints each index, desktop coordinate, resolution and primary status, then opens `ProjectionWindow` fullscreen on the first non-primary display. The projector may be positioned left, right, above or below the primary display and may use a different resolution. Windows must be configured for **Extend these displays**, not Duplicate. Use `--projector_display_index=N` to choose a specific printed index; `-1` retains automatic selection. Press `Esc` to exit.

The camera capture request defaults to **1280×720**. Startup prints both the requested DirectShow mode and the size of the first frame actually received. If the latter is not `1280x720`, close other programs using the camera and override `--camera_width`/`--camera_height` only if the camera requires another supported mode. Chessboard detection runs on the full captured frame.

An existing `camera_params.xml` is loaded only when its saved image size exactly matches the live capture size. An incompatible file is ignored and the program requests a fresh camera calibration. This prevents correct detected corners from being combined with stale intrinsics, which would misplace the reprojected cyan checkerboard boundary and later projector geometry. Changing only the printed square size does not invalidate camera intrinsics, but changing capture resolution, camera mode, focus, zoom or camera hardware requires recalibration.

Both calibration files include their solved intrinsic parameters, per-view reprojection errors, per-view extrinsics, and detected image points. As soon as static projector calibration succeeds, `projector_params.xml` also receives a `static_` camera-to-projector transform so an interrupted run can resume at dynamic calibration. Dynamic completion appends the final unprefixed transform used by tracking. Set `--calibration_output_dir` to write the files elsewhere.

Startup defaults to `--calibration_file_mode=redo`, which performs every phase and replaces results only after the corresponding phase succeeds. Use `--calibration_file_mode=reuse` to load validated files: a complete file enters tracking, a static-only file resumes dynamic calibration, a camera-only result starts static projector calibration, and missing/incompatible results fall back to the earliest required phase.

## Calibration Process

### Sample Capture Gate

For camera, static-projector, and dynamic-projector calibration, the app accepts a sample only after it detects the chessboard. The first valid sample in each phase is accepted immediately; it does **not** wait for `--delay_between_frames` or require board motion. Each later sample requires both:

- the configured minimum interval since the previous accepted sample
- chessboard-corner RMS movement at or above `--minimum_board_motion_px`

Camera and static-projector calibration collect `--calibration_candidate_margin` extra accepted views before selecting their final retained set. Candidate views first pass a robust per-view RMS gate based on the median and median absolute deviation, capped by the configured per-view RMS limit. The retained views are then chosen for diversity across detected-pattern position, board depth, tilt, and roll. Calibration continues collecting views until selection-time coverage checks pass and the final recalibration meets the aggregate-RMS limit.

Board-position coverage uses the chessboard-corner centroid in both phases, measuring the largest pairwise separation after normalizing X by camera image width and Y by height. Camera selection requires `--minimum_calibration_position_span` (0.25); static-projector selection uses its own `--minimum_projector_board_position_span` (0.10). Static-projector selection additionally checks the normalized diagonal of decoded points in **projector pixels**, using `--minimum_projector_pattern_span` (0.20). Footprint alone does not guarantee full projector field-of-view coverage or accurate distortion outside that footprint.

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

Static projector calibration uses temporally encoded Gray code instead of a simultaneous 4×5 circle grid. This allows the A4 checkerboard itself to be the projection surface even when projected light obscures the printed pattern.

For each board position, the program automatically captures:

1. a black reference frame, where it detects the complete checkerboard and ID 30 and stores the clean board pose;
2. a white reference frame;
3. normal/inverse Gray-code bit pairs for coarse projector X and Y cells;
4. four sinusoidal phase-shift frames for each axis, which refine the coordinate inside each cell;
5. a final black frame used to verify that the board did not move.

With the default 8×8-projector-pixel Gray cells, 1920×1080 requires 8 X bits and 8 Y bits. Black/white references, normal/inverse Gray pairs, and eight phase frames therefore use 42 reference/code frames plus one final motion-check frame: **43 captured frames per board position**. The Gray cells remain robust through optical blur. Phase patterns use a separate 32-pixel spatial period so their modulation remains visible, while the decoded Gray cell resolves the repeated phase cycle and a consistency check rejects disagreements. The count is derived automatically from projector resolution and does not change with `--static_gray_phase_period_px`. The default static calibration retains 8 diverse board positions and collects 12 candidates (8 plus the default 4-view quality/diversity margin).

The decoder samples known checkerboard-square centers, pairs their board coordinates with decoded projector pixels, and uses those correspondences to calibrate projector intrinsics with `calibrateCamera()`. Printed black/white texture is rejected by normal/inverse comparison rather than being treated as a circle grid.

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

#### Capturing Gray code on the A4 board

Keep the camera and projector fixed and move only the board between complete Gray/phase sequences. The projector remains black while you prop the board; when the complete checkerboard and ID 30 are stable, focus an OpenCV window and press **Space or Enter** to start exactly one 43-frame sequence. After that sequence is accepted or rejected, the application returns to the manual waiting state. Exposure, focus and white balance should remain fixed.

The console reports the required action explicitly:

- `HOLD THE CHECKERBOARD STILL` starts a sequence.
- `READY ... press SPACE or ENTER` means capture is waiting while you finish positioning and propping the board.
- `BOARD NOT STATIC ENOUGH` discards the entire sequence and reports measured RMS motion.
- `READY TO MOVE THE CHECKERBOARD TO A DIFFERENT POSITION` means one view was accepted; vary image position, distance and tilt before holding still again.
- `FINISHED WITH STATIC PROJECTOR CALIBRATION` means projector intrinsics and the initial stereo transform passed their quality checks.

`ImageThresholded` displays the white-reference camera frame with accepted Gray-code sample locations in green and rejected locations in red. A view requires at least 12 decoded points, although the 9×6 board supplies up to 70 square-center candidates.

Rejection diagnostics report `coarse_gray_valid`, `phase_valid`, and `gray_phase_consistent` separately. After two consecutive sequences with zero phase-valid points, automatic capture pauses instead of repeating indefinitely; move the board by the configured sample-motion distance to retry.

#### Understanding capture and selection messages

`current projector candidate count: 3/12` means three complete Gray-code board positions have been accepted; diversity and RMS selection have not run yet. The default retains eight positions after collecting twelve candidates. Waiting messages distinguish a missing complete board/ID 30, insufficient movement since the previous accepted position, motion during a sequence, and insufficient Gray-code contrast.

After the candidate target is reached, separate messages report insufficient RMS-eligible views, insufficient board translation, depth or tilt, and insufficient decoded projector-image footprint. Improve coverage by moving the board through more of the projector field while retaining useful tilt and depth variation. A final aggregate RMS failure remains a separate rejection. Preliminary fit RMS can exceed the final completion limit because selection/refitting has not yet completed.

Each fit logs the preliminary per-view RMS and eligibility/selection status for every candidate, using 1-based capture numbers. The robust eligibility threshold can be stricter than the absolute per-view limit. With too few eligible views, coverage is explicitly **not evaluated** rather than printing default zero spans. The final refit logs each retained candidate's RMS with its original capture number. Preliminary values are recomputed as new samples arrive; they are residuals of the joint model, not independent measurements of frame quality. Persistent errors across most views can indicate camera-calibration error, board non-flatness or detection bias as well as poor images. The eight-view target and final RMS limits are not relaxed.

#### Dynamic circle-detection diagnostics and tuning

The following circle settings apply to dynamic calibration, not the new static Gray-code phase. Circle detection operates on an unannotated, inverse-thresholded camera image. Chessboard/debug graphics are drawn only **after** circle detection. `ImageThresholded` then overlays red blob candidates and their count; these include clutter and do not imply a valid grid. Detection requires the correct ordered 4x5 grid, not simply 20 arbitrary blobs.

The default brightness threshold remains 210/255. Bright projected circles should appear as isolated black disks on a white background in the thresholded image. If they vanish, lower `--projector_detection_threshold`; if they merge into bright surrounding board regions, adjust exposure/lighting or the threshold. `--projector_detection_threshold=-1` enables global Otsu thresholding, but the printed chessboard and scene background can dominate its histogram, so it is not guaranteed to work better.

Blob-area limits are in **camera-image pixels squared**, not projector pixels or board units. Defaults match the previous detector: min area 25, max area 5000, inertia ratio 0.1, convexity 0.95, circularity filtering disabled. Smaller/farther circles may need a lower minimum area; larger/nearer circles may need a higher maximum. Tilt creates elliptical blobs: overly strict shape filters can reject valid circles. Zero disables a shape filter, but relaxing filters also admits clutter. Change settings between runs, not during calibration.

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

Tracking keeps the projected circles at the same checkerboard-square locations used during dynamic calibration. Entering tracking changes smoothing, not the intended physical placement.

By default, both dynamic calibration and tracking place the complete 4×5 asymmetric grid inside the printed checkerboard boundary. For the 9×6-inner-corner, 20 mm board, circle centers span X=[10,150] mm and Y=[10,90] mm relative to the intended first inner corner. Every center is halfway between adjacent inner-corner coordinates in both axes, so it lands at a square center rather than covering an inner corner. The asymmetric row stagger keeps all 20 centers on the same checker color. Circle radius is still specified in projector pixels, so visually confirm that circles do not grow across inner corners at close range.

Set `--project_grid_inside_checkerboard=false` to restore an external placement. In that mode, `--use_projection_region=true` uses the retained 37×29 cm measured region below the checkerboard, while setting both flags false uses the legacy Y-offset formula. Those external defaults are preserved only as fallbacks and do not fit on an A4-only board.

### Board origin and projection placement

The default orientation anchor is the single large `DICT_6X6_50` (dictionary number 8) marker with ID **30**, matching the November 24 photo. No marker-to-board measurement or marker print-size setting is required. When the complete chessboard and ID 30 are visible together, the program finds the outer inner checkerboard corner nearest the marker center and assigns that physical corner as `(0,0)`.

At startup, show the entire chessboard and readable ID 30 together. Move closer if necessary, without cropping any inner corners. Keep the marker clearly closer to the intended origin corner than to the other three outer inner corners. Check the yellow `(0,0)` origin in `CameraDebug` against that corner. After camera intrinsics are available, the cyan outline shows the printed checkerboard's outer boundary when inside-checkerboard placement is enabled. With the external fallback enabled, it instead shows the configured physical projection region.

The origin is the marker-nearest physical corner, not the image's top-left corner. The board may be rotated at initialization or re-acquisition, including 90, 180 and 270 degrees. All four grid-axis directions are evaluated and the candidate whose first corner is nearest the marker is selected. An ambiguity guard rejects a marker that is similarly close to multiple board corners. Once anchored, continuous full-chessboard detections maintain corner ordering even when the marker is no longer readable. Rotate gradually while the marker is unreadable; excessive motion, ambiguous ordering, or loss of the complete chessboard clears the anchor. Show readable ID 30 together with the entire chessboard again to resume, in any orientation. This is conservative frame-to-frame ordering continuity, not tracking an invisible or partially occluded board. Marker detection retries at double image scale; blur or insufficient source pixels still requires bringing the marker closer. Failure diagnostics distinguish readable anchor count from missing complete chessboard detection. A detected anchor is masked white in a private detection image if it overlaps the board, reducing interference with corner refinement. Optically mirrored ArUco codes are not decoded by the standard detector; use an unmirrored camera feed.

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

Projection beyond this validation disk or behind the projector is rejected instead of extrapolating the distortion polynomial unchecked. This deliberately favors a blank output over unstable or scrambled circles. Marker defaults are unchanged. Do not edit an old XML file merely to set `k3=0`: the remaining parameters were jointly fitted with it and must be recalibrated. OpenCV explains why non-monotonic distortion should be treated as calibration failure in its [calibration documentation](https://docs.opencv.org/4.12.0/d9/d0c/group__calib3d.html).

## Important Physical Assumption

During calibration, the printed chessboard and the projected circles must lie on the **same rigid plane**.

If the circles are projected onto a different plane than the marker, the 3D reconstruction used by projector calibration becomes invalid.

## Supported Parameters

The application currently supports these command-line parameters from [`include/flags.hpp`](include/flags.hpp):

| Parameter | Default | Meaning |
|---|---:|---|
| `--pattern_width` | `9` | Number of inner corners across the calibration board width |
| `--pattern_height` | `6` | Number of inner corners across the calibration board height |
| `--board_square_size_cm` | `2.0` | Printed square side length in centimeters; converted internally to millimeters |
| `--num_boards_before_dynamic_projector_calib` | `8` | Number of retained static-projector views required before dynamic calibration |
| `--num_boards_final_projector_calib` | `5` | Minimum dynamic accepted samples target used for completion |
| `--minimum_frames` | `8` | Number of retained camera calibration views |
| `--delay_between_frames` | `1000` | Minimum interval in milliseconds after the first accepted sample in a phase |
| `--minimum_board_motion_px` | `15.0` | Minimum chessboard-corner RMS movement in pixels after the first accepted sample in a phase |
| `--calibration_output_dir` | Executable directory | Directory for `camera_params.xml` and `projector_params.xml` |
| `--projector_display_index` | `-1` | Automatically select the first non-primary display; set a printed startup index to override |
| `--camera_width` | `1280` | Requested DirectShow camera capture width |
| `--camera_height` | `720` | Requested DirectShow camera capture height |
| `--require_board_orientation` | `true` | Require a marker anchor initially and after tracking loss |
| `--board_aruco_dictionary` | `8` | OpenCV predefined dictionary number; 8 is DICT_6X6_50 |
| `--board_origin_aruco_id` | `30` | Single anchor marker ID; its nearest outer inner corner becomes `(0,0)` |
| `--board_tracking_max_motion_squares` | `1.5` | Maximum continuity RMS displacement relative to observed horizontal corner spacing |
| `--project_grid_inside_checkerboard` | `true` | Put dynamic-calibration and tracking centers on square centers inside the printed checkerboard boundary |
| `--use_projection_region` | `true` | When inside placement is disabled, center the grid inside the external physical region |
| `--projection_region_width_cm` | `37.0` | Projection-region width |
| `--projection_region_height_cm` | `29.0` | Projection-region height |
| `--projection_region_top_cm` | `25.5` | Region top edge below the anchored origin |
| `--projection_region_center_x_cm` | `8.0` | Region horizontal center relative to the anchored origin; centers the default 160 mm inner-corner span |
| `--projector_offset_y_scale` | `0.9` | Legacy board-Y offset; used only when both placement flags are false |
| `--projected_circle_radius` | `30` | Radius of each projected circle in projector pixels |
| `--static_gray_max_board_motion_px` | `3.5` | Maximum corner RMS motion over a complete static Gray/phase sequence |
| `--static_gray_min_direct_contrast` | `5.0` | Minimum absolute white-versus-black camera intensity change for a decoded point |
| `--static_gray_min_bit_contrast` | `8.0` | Minimum normal/inverse Gray contrast and phase-modulation amplitude |
| `--static_gray_sample_radius_px` | `0` | Radius of the camera patch sampled at each board-square center; zero uses one pixel |
| `--static_gray_projector_block_size` | `8` | Coarse Gray-cell size in projector pixels; four-step phase decoding refines the coordinate within each cell |
| `--static_gray_phase_period_px` | `32` | Sinusoidal phase-pattern period; must be a multiple of the Gray-cell size |
| `--static_gray_start_delay_seconds` | `3` | Countdown after camera calibration before static Gray-code capture starts |
| `--calibration_file_mode` | `redo` | `redo` performs all phases; `reuse` validates saved files and resumes at the next incomplete phase |
| `--minimum_projector_board_position_span` | `0.10` | Minimum normalized chessboard-centroid span for static-projector selection |
| `--minimum_projector_pattern_span` | `0.20` | Minimum normalized diagonal decoded-point footprint in projector pixels |
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
.\build\Release\dynamic_projection.exe --pattern_width=9 --pattern_height=6 --board_square_size_cm=2.0
```

These are the defaults, so launching the executable without flags uses the same checkerboard, ArUco ID 30, and inside-checkerboard projection placement. Start with readable ID 30 and all inner corners visible, then verify the origin and cyan outer-boundary overlays before collecting views. The application starts camera calibration, then automatically proceeds through static-projector calibration, dynamic-projector calibration, and tracking. Move the board between accepted samples and vary position, depth, and tilt. With default settings, camera and static-projector selection starts after 12 accepted candidate views each, retaining 8; more candidates may be needed to satisfy quality and coverage checks. Dynamic calibration requires at least 5 accepted samples and the stereo RMS limit. Press `Esc` to exit.

Results are written to `build\Release\camera_params.xml` and `build\Release\projector_params.xml`. Final stereo extrinsics are added only after dynamic calibration completes. Use `--calibration_output_dir="C:/path/to/results"` to select another output directory.

Example launch command using the current defaults:

```powershell
.\build\Release\dynamic_projection.exe `
  --pattern_width=9 `
  --pattern_height=6 `
  --board_square_size_cm=2.0 `
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

The checkerboard defaults are 9 × 6 inner corners and 2.0 cm squares; use the board flags to match a different physical print. The following setup values are still hardcoded in `main.cpp`:

- the projected asymmetric circle pattern size is currently set to `4x5`

The projector resolution is detected from the selected Windows display and used for window creation, calibration bounds and projector intrinsics. Recalibrate if the selected display or its resolution changes.

Do not resize the printed marker without updating `--board_square_size_cm` to its measured square side length.

## Accuracy Tips By Phase

### Camera Calibration

- Use a large printed board if possible. A larger board gives more stable corner localization.
- Capture views with varied tilts, distances, and image positions.
- Do not use only floor-parallel views.
- Ensure the entire chessboard is visible.
- Keep the board still when a frame is accepted.

### Static Projector Calibration

- Support the board so it remains within the configured 3.5-pixel RMS motion limit for all 43 captures.
- Keep the full checkerboard and ID 30 visible in the initial and final black frames.
- Lock exposure, focus and white balance; auto-exposure changes reduce normal/inverse contrast.
- Move only after `READY TO MOVE THE CHECKERBOARD TO A DIFFERENT POSITION` appears.
- Use varied translations, distances and tilts, not only fronto-parallel views.

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

The debug windows are displayed at `640x360`, but this does not downsample the frames used for detection or calibration. Long status messages are wrapped before display so their diagnostic details remain visible.

## Current Practical Recommendation

For best results:

- use the larger board if detection is more stable
- collect many tilted views
- keep the board and projected pattern on the same rigid plane
- avoid projector bloom on the marker
- verify the final alignment while tilting the board, not only while sliding it on the floor
