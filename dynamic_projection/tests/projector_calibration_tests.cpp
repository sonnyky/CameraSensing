#include "calibration.hpp"
#include "calibration_view_selection.hpp"
#include "flags.hpp"
#include "projector_circle_detection.hpp"
#include "projector_projection_bounds.hpp"
#include "projector_pattern_snapshot.hpp"
#include "stereo_fit_validation.hpp"
#include "board_orientation_tracker.hpp"
#include "projection_region_layout.hpp"
#include "projector_distortion_validation.hpp"
#include "shared_pose_smoothing.hpp"
#include <opencv2/objdetect/aruco_dictionary.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <filesystem>
#include <chrono>

void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

int main(int argc, char** argv)
{
    try {
		const cv::Mat modelMatrix = (cv::Mat_<double>(3,3) << 3557.498,0,941.969,0,3533.210,122.675,0,0,1);
		std::string modelReason;
		require(Tinker::valid_projector_distortion(modelMatrix, cv::Mat::zeros(5,1,CV_64F), {1920,1080}, modelReason),
			"Zero-distortion projector model rejected");
		cv::Mat folded = (cv::Mat_<double>(5,1) << -2.045639946,83.604952868,0.055676459,0.016060684,-964.263678837);
		require(!Tinker::valid_projector_distortion(modelMatrix, folded, {1920,1080}, modelReason),
			"Actual folded projector fit accepted");
		folded = (cv::Mat_<double>(5,1) << -4,0,0,0,0);
		require(!Tinker::valid_projector_distortion(modelMatrix, folded, {1920,1080}, modelReason),
			"Folded k1-only model accepted despite zero k3");
		folded = (cv::Mat_<double>(5,1) << 0,0,5,0,0);
		require(!Tinker::valid_projector_distortion(modelMatrix, folded, {1920,1080}, modelReason),
			"Tangential fold accepted");
		cv::Mat poseRot = (cv::Mat_<double>(3,1) << 0,0,CV_PI-0.05);
		cv::Mat nextRot = (cv::Mat_<double>(3,1) << 0,0,-CV_PI+0.05);
		cv::Mat poseTrans = (cv::Mat_<double>(3,1) << 0,0,700);
		cv::Mat nextTrans = (cv::Mat_<double>(3,1) << 100,20,900);
		Tinker::smooth_shared_board_pose(nextRot,nextTrans,0.5,poseRot,poseTrans);
		cv::Mat poseMatrix;
		cv::Rodrigues(poseRot,poseMatrix);
		const cv::Mat expectedTranslation=(cv::Mat_<double>(3,1) << 50,10,800);
		require(poseMatrix.at<double>(0,0) < -0.999 && cv::norm(poseTrans - expectedTranslation) < 1e-6,
			"Shared pose smoothing took the long rotation path or wrong translation");
		std::vector<cv::Point3f> rigidGrid = {{0,0,0},{36,0,0},{0,36,0},{36,36,0}};
		for (const auto& point : rigidGrid) {
			cv::Mat transformed = poseMatrix*(cv::Mat_<double>(3,1) << point.x,point.y,point.z);
			require(std::abs(cv::norm(transformed) - cv::norm(point)) < 1e-6, "Shared pose deformed grid geometry");
		}
		Tinker::smooth_shared_board_pose(nextRot,nextTrans,1,poseRot,poseTrans);
		require(cv::norm(poseRot-nextRot)==0 && cv::norm(poseTrans-nextTrans)==0, "Immediate pose update retained smoothing");
		require(FLAGS_board_origin_aruco_id == 0 && FLAGS_board_origin_secondary_aruco_id == 1,
			"Default IDs do not match the glued marker layout");
		const auto regionOrigin = Tinker::centered_projection_grid_origin(cv::Point3f(0, 0, 0), cv::Point3f(36, 0, 0),
			cv::Point3f(0, 36, 0), cv::Size(4, 5), 144, 255, 370, 290);
		require(cv::norm(regionOrigin - cv::Point3f(18, 328, 0)) < 0.01, "Configured white-region grid placement incorrect");
		bool regionRejected = false;
		try { Tinker::centered_projection_grid_origin(cv::Point3f(0, 0, 0), cv::Point3f(36, 0, 0),
			cv::Point3f(0, 36, 0), cv::Size(4, 5), 144, 255, 200, 290); }
		catch (const std::invalid_argument&) { regionRejected = true; }
		require(regionRejected, "Oversized grid accepted for small projection region");
		FLAGS_require_board_orientation = false; // Unmarked legacy image fixtures below.
		const auto bounds = Tinker::projector_projection_bounds({cv::Point2f(50, 50), cv::Point2f(5, 50),
			cv::Point2f(-50, 50), cv::Point2f(std::numeric_limits<float>::quiet_NaN(), 50)}, cv::Size(100, 100), 10);
		require(bounds.finiteCenters == 3 && bounds.fullyVisibleCircles == 1 && bounds.intersectingCircles == 2,
			"Projection bounds failed to distinguish visible, clipped, off-image and non-finite centers");
		require(bounds.minX == -50 && bounds.maxX == 50, "Projection coordinate range incorrect");

		cv::Mat debugFrame(540, 960, CV_8UC1, cv::Scalar(255));
		for (int row = 0; row < 7; ++row) {
			for (int col = 0; col < 10; ++col) {
				if ((row + col) % 2 == 0) cv::rectangle(debugFrame, cv::Rect(100 + col * 40, 100 + row * 40, 40, 40), cv::Scalar(0), -1);
			}
		}
		const auto projectionFrame = debugFrame.clone();
		Tinker::camera_calibration boardDetector;
		boardDetector.setup_parameters(cv::Size(9, 6), debugFrame.size(), 36, 1, 8, Tinker::DETECTION, 0, "unused.xml");
		require(boardDetector.find_board(projectionFrame), "Clean synthetic chessboard was not detected");
		const auto originalProjectionFrame = projectionFrame.clone();
		cv::drawChessboardCorners(debugFrame, cv::Size(9, 6), boardDetector.get_detected_board_points(), true);
		require(cv::norm(debugFrame, projectionFrame, cv::NORM_INF) > 0, "Debug annotation fixture did not change pixels");
		require(cv::norm(projectionFrame, originalProjectionFrame, cv::NORM_INF) == 0 && boardDetector.find_board(projectionFrame),
			"Debug overlays contaminated the isolated projection-update frame");

		Tinker::BoardOrientationTracker tracker;
		std::vector<cv::Point2f> canonical;
		for (int y = 0; y < 6; ++y) for (int x = 0; x < 9; ++x) canonical.emplace_back(140.0f + x * 40, 140.0f + y * 40);
		std::string orientationStatus;
		auto detected = canonical;
		require(!tracker.orient(detected, cv::Size(9, 6), {}, orientationStatus), "Unanchored board silently accepted");
		detected = canonical; std::reverse(detected.begin(), detected.end());
		require(tracker.orient(detected, cv::Size(9, 6), {{0, cv::Point2f(160, 120)}, {1, cv::Point2f(120, 160)}}, orientationStatus) &&
			detected == canonical, "Marker anchor did not recover intended top-left origin");
		for (auto& point : detected) point += cv::Point2f(5, 10);
		std::reverse(detected.begin(), detected.end());
		require(tracker.orient(detected, cv::Size(9, 6), {}, orientationStatus) && detected.front() == cv::Point2f(145, 150),
			"Marker-free tracking flipped chessboard origin");
		for (auto& point : detected) point += cv::Point2f(500, 0);
		require(!tracker.orient(detected, cv::Size(9, 6), {}, orientationStatus), "Large unmarked jump was guessed");
		detected = canonical;
		require(!tracker.orient(detected, cv::Size(9, 6), {}, orientationStatus), "Ambiguity did not require reinitialization");
		require(tracker.orient(detected, cv::Size(9, 6), {{1, cv::Point2f(120, 160)}}, orientationStatus), "Single secondary marker could not reanchor");
		tracker.reset();
		require(!tracker.orient(detected, cv::Size(9, 6), {}, orientationStatus), "Tracking reset retained stale origin");
		require(!tracker.orient(detected, cv::Size(9, 6), {{0, cv::Point2f(160, 120)}, {1, cv::Point2f(500, 300)}}, orientationStatus),
			"Conflicting marker layout was accepted");

		cv::Mat markedBoard = projectionFrame.clone();
		const auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
		cv::Mat marker;
		dictionary.generateImageMarker(0, 16, marker);
		marker.copyTo(markedBoard(cv::Rect(152, 112, 16, 16)));
		dictionary.generateImageMarker(1, 16, marker);
		marker.copyTo(markedBoard(cv::Rect(112, 152, 16, 16)));
		FLAGS_require_board_orientation = true;
		boardDetector.setup_parameters(cv::Size(9, 6), markedBoard.size(), 36, 1, 8, Tinker::DETECTION, 0, "unused.xml");
		require(boardDetector.find_board(markedBoard), "Actual ArUco/chessboard anchoring failed");
		require(cv::norm(boardDetector.get_detected_board_points().front() - cv::Point2f(139.5f, 139.5f)) < 1,
			"Marked board selected wrong physical origin");
		require(boardDetector.find_board(projectionFrame), "Actual unmarked continuation failed");
		require(!boardDetector.find_board(cv::Mat::zeros(markedBoard.size(), CV_8UC1)), "Blank frame was not treated as tracking loss");
		require(!boardDetector.find_board(projectionFrame), "Tracking loss did not require an ArUco marker");
		require(boardDetector.find_board(markedBoard), "Marker reacquisition failed");
		for (int rotation : {cv::ROTATE_90_CLOCKWISE, cv::ROTATE_180, cv::ROTATE_90_COUNTERCLOCKWISE}) {
			cv::Mat rotated, rotatedUnmarked;
			cv::rotate(markedBoard, rotated, rotation);
			cv::rotate(projectionFrame, rotatedUnmarked, rotation);
			require(!boardDetector.find_board(cv::Mat::zeros(rotated.size(), CV_8UC1)), "Loss fixture accepted");
			require(boardDetector.find_board(rotated), "Rotated marker board did not reacquire after loss");
			cv::Point2f expected(139.5f, 139.5f);
			if (rotation == cv::ROTATE_90_CLOCKWISE) expected = {markedBoard.rows - 1 - expected.y, expected.x};
			else if (rotation == cv::ROTATE_180) expected = {markedBoard.cols - 1 - expected.x, markedBoard.rows - 1 - expected.y};
			else expected = {expected.y, markedBoard.cols - 1 - expected.x};
			require(cv::norm(boardDetector.get_detected_board_points().front() - expected) < 1,
				"Rotation changed the physical origin");
			require(boardDetector.find_board(rotatedUnmarked), "Rotated marker-free continuation failed");
		}
		cv::Mat mirrored;
		cv::flip(markedBoard, mirrored, 1);
		require(!boardDetector.find_board(cv::Mat::zeros(mirrored.size(), CV_8UC1)), "Mirror loss fixture accepted");
		// ArUco decoding does not decode mirrored codes. Test mirrored grid
		// ordering directly with known marker centers instead.
		for (int flip = 0; flip < 4; ++flip) {
			detected = canonical;
			for (int y = 0; y < 6; ++y) for (int x = 0; x < 9; ++x)
				detected[y * 9 + x] = canonical[((flip & 2) ? 5-y : y) * 9 + ((flip & 1) ? 8-x : x)];
			tracker.reset();
			require(tracker.orient(detected, {9, 6}, {{0, {160, 120}}, {1, {120, 160}}}, orientationStatus) && detected == canonical,
				"Grid-axis reversal changed the physical origin");
		}
		if (argc == 2) {
			const auto photo = cv::imread(argv[1]);
			boardDetector.setup_parameters(cv::Size(9, 6), photo.size(), 36, 1, 8, Tinker::DETECTION, 0, "unused.xml");
			const bool foundPhoto = boardDetector.find_board(photo);
			std::cout << "Photo origin detection=" << foundPhoto << "; " << boardDetector.get_orientation_status() << '\n';
			if (foundPhoto) std::cout << "Photo canonical origin=" << boardDetector.get_detected_board_points().front() << '\n';
		}
        Tinker::projector_calibration projector;
        auto setup = [&projector](float spacing) {
            projector.setup_projector_parameters(cv::Size(1920, 1080), "unused.xml",
                cv::Size(4, 5), spacing, 8, Tinker::ASYMMETRIC_CIRCLES_GRID, 0, 0);
            projector.set_static_candidate_image_points();
        };
        setup(120);
        const auto original = projector.get_candidate_image_points();
        require(original.size() == 20, "Original grid point count changed");
        require(cv::norm(original.front() - cv::Point2f(540, 300)) < 0.001, "Original default layout changed");
        FLAGS_projected_circle_radius = 15;
        FLAGS_static_projector_center_x = 0.6;
        setup(60);
        const auto compact = projector.get_candidate_image_points();
        require(cv::norm(compact.front() - cv::Point2f(942, 420)) < 0.001, "Compact/offset grid incorrect");
        require(cv::norm(compact[4] - cv::Point2f(1002, 480)) < 0.001, "Asymmetric row offset incorrect");
        FLAGS_static_projector_center_x = 0.0;
        bool clipped = false;
        try { setup(60); } catch (const std::invalid_argument&) { clipped = true; }
        require(clipped, "Clipped grid was not rejected");

        // Actual camera-space detection on an untouched synthetic image.
        cv::Mat captured(540, 960, CV_8UC1, cv::Scalar(60));
        std::vector<cv::Point2f> expected;
        for (int row = 0; row < 5; ++row) {
            for (int col = 0; col < 4; ++col) {
                cv::Point2f center(220.0f + (2 * col + row % 2) * 45.0f, 140.0f + row * 45.0f);
                expected.push_back(center);
                cv::circle(captured, center, 10, cv::Scalar(180), -1);
            }
        }
        const cv::Mat untouched = captured.clone();
        std::vector<cv::Point2f> centers;
        std::vector<cv::KeyPoint> blobs;
        require(!Tinker::detect_projector_circles(Tinker::threshold_projector_circles(captured),
            cv::Size(4, 5), centers, blobs), "Dim circles unexpectedly passed threshold 210");
        FLAGS_projector_detection_threshold = 150;
        const auto thresholded = Tinker::threshold_projector_circles(captured);
        const auto thresholdedCopy = thresholded.clone();
        require(Tinker::detect_projector_circles(thresholded, cv::Size(4, 5), centers, blobs),
            "Configurable threshold failed to recover dim 4x5 grid");
        require(blobs.size() == 20 && centers.size() == 20, "Synthetic blob/grid count incorrect");
        require(cv::norm(captured, untouched, cv::NORM_INF) == 0 &&
            cv::norm(thresholded, thresholdedCopy, cv::NORM_INF) == 0, "Detection mutated its input");
        for (size_t i = 0; i < centers.size(); ++i) {
            require(cv::norm(centers[i] - expected[i]) < 0.1, "Circle center/order changed");
        }
        FLAGS_projector_detection_threshold = -1;
        require(Tinker::detect_projector_circles(Tinker::threshold_projector_circles(captured),
            cv::Size(4, 5), centers, blobs), "Otsu failed on separated intensity fixture");
        FLAGS_projector_blob_min_area = 1000;
        require(!Tinker::detect_projector_circles(Tinker::threshold_projector_circles(captured),
            cv::Size(4, 5), centers, blobs), "Blob area filter was ignored");
        FLAGS_projector_blob_min_area = 25;
        FLAGS_projector_blob_min_inertia = 0;
        FLAGS_projector_blob_min_convexity = 0;
        require(Tinker::detect_projector_circles(Tinker::threshold_projector_circles(captured),
            cv::Size(4, 5), centers, blobs), "Disabling shape filters failed");
        FLAGS_projector_blob_min_inertia = 0.1;
        FLAGS_projector_blob_min_convexity = 0.95;
        FLAGS_projector_detection_threshold = 210;

        std::vector<float> errors(8, 0.5f);
        std::vector<cv::Mat> rotations, translations;
        std::vector<std::vector<cv::Point2f>> boards, stationaryCircles;
        for (int i = 0; i < 8; ++i) {
            rotations.push_back((cv::Mat_<double>(3, 1) << i * 0.04, 0, 0));
            translations.push_back((cv::Mat_<double>(3, 1) << 0, 0, 500 + i * 15));
            boards.push_back({ cv::Point2f(500.0f + i * 35, 400) });
            stationaryCircles.push_back({ cv::Point2f(960, 540) });
        }
        auto select = [&](const std::vector<std::vector<cv::Point2f>>& points) {
            return Tinker::select_calibration_views(errors, rotations, translations, points,
                cv::Size(1920, 1080), 8, 3.0, 0.1, 1.1, 10);
        };
        require(select(boards).hasRequiredCoverage, "Valid board-pose diversity rejected");
        require(!select(stationaryCircles).hasRequiredCoverage, "Stationary-circle regression fixture invalid");
        errors.assign(8, 3.5f);
        require(!select(boards).hasEnoughQualityViews, "Per-view RMS limit bypassed");
        projector.frameMeasuredBoardImagePoints.push_back(boards.front());
        projector.reset_boards();
        require(projector.frameMeasuredBoardImagePoints.empty(), "Reset left stale board measurements");

        // Exercise the actual static calibration/refit and parallel-array selection
        // with known projector rays intersecting differently posed board planes.
        FLAGS_static_projector_center_x = 0.5;
        FLAGS_projected_circle_radius = 30;
        const auto output = std::filesystem::temp_directory_path() /
            ("projector_regression_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".xml");
        struct RemoveOutput {
            std::filesystem::path path;
            ~RemoveOutput() { std::error_code error; std::filesystem::remove(path, error); }
        } cleanup{output};

        Tinker::ProjectorPatternSnapshot snapshot;
        std::vector<cv::Point2f> queued = {cv::Point2f(10, 20)};
        snapshot.record(queued, true);
        queued.front() = cv::Point2f(30, 40);
        require(snapshot.points().front() == cv::Point2f(10, 20), "Queued update changed displayed correspondences");
        snapshot.record(queued, false);
        require(snapshot.points().empty(), "Blank projection retained stale correspondences");

        cv::Mat workingRotation = cv::Mat::zeros(3, 1, CV_64F);
        cv::Mat workingTranslation = (cv::Mat_<double>(3, 1) << 10, 20, 30);
        const auto preservedTranslation = workingTranslation.clone();
        cv::Mat candidateMatrix = cv::Mat::eye(3, 3, CV_64F);
        cv::Mat candidateTranslation = (cv::Mat_<double>(3, 1) << 40, 50, 60);
        double workingRms = 2.39;
        require(!Tinker::commit_valid_stereo_fit(6.5, 3, candidateMatrix, candidateTranslation,
            workingRotation, workingTranslation, workingRms), "High-RMS stereo fit accepted");
        require(cv::norm(workingTranslation, preservedTranslation) == 0 && workingRms == 2.39,
            "Rejected stereo fit changed working parameters");
        require(!Tinker::commit_valid_stereo_fit(std::numeric_limits<double>::quiet_NaN(), 3,
            candidateMatrix, candidateTranslation, workingRotation, workingTranslation, workingRms), "NaN RMS accepted");
        cv::Mat invalidTranslation = candidateTranslation.clone();
        invalidTranslation.at<double>(0) = std::numeric_limits<double>::infinity();
        require(!Tinker::commit_valid_stereo_fit(2, 3, candidateMatrix, invalidTranslation,
            workingRotation, workingTranslation, workingRms), "Non-finite transform accepted");
        require(Tinker::commit_valid_stereo_fit(2.75, 3, candidateMatrix, candidateTranslation,
            workingRotation, workingTranslation, workingRms) && workingRms == 2.75,
            "Valid fit was rejected merely because different-view RMS increased");

        const auto cameraFile = output.string() + ".camera.xml";
        RemoveOutput cleanupCamera{cameraFile};
        const cv::Mat intrinsics = (cv::Mat_<double>(3, 3) << 700, 0, 480, 0, 710, 270, 0, 0, 1);
        const cv::Mat distortion = (cv::Mat_<double>(5, 1) << -0.15, 0.04, 0.001, -0.001, 0);
        {
            cv::FileStorage file(cameraFile, cv::FileStorage::WRITE);
            file << "camera_matrix" << intrinsics << "distortion_coefficients" << distortion;
        }
        Tinker::camera_calibration backProjector;
        backProjector.load_camera_matrix(cameraFile);
        const std::vector<cv::Point3f> knownPlanePoints = {cv::Point3f(0, 0, 0), cv::Point3f(150, 60, 0),
            cv::Point3f(250, 140, 0), cv::Point3f(-150, -80, 0)};
        cv::Mat boardRotation = (cv::Mat_<double>(3, 1) << 0.2, -0.15, 0.05);
        for (double depth : {500.0, 900.0, 1400.0}) {
            cv::Mat boardTranslation = (cv::Mat_<double>(3, 1) << 20, -10, depth);
            std::vector<cv::Point2f> measured;
            cv::projectPoints(knownPlanePoints, boardRotation, boardTranslation, intrinsics, distortion, measured);
            std::vector<cv::Point3f> recovered;
            require(backProjector.back_project(boardRotation, boardTranslation, measured, recovered), "Distorted ray intersection failed");
            for (size_t i = 0; i < recovered.size(); ++i) {
                require(cv::norm(recovered[i] - knownPlanePoints[i]) < 0.05, "Distortion-corrected board points changed with depth");
            }
        }
        std::vector<cv::Point3f> preservedOutput = {cv::Point3f(1, 2, 0)};
        cv::Mat grazingRotation = (cv::Mat_<double>(3, 1) << 0, CV_PI / 2, 0);
        cv::Mat frontTranslation = (cv::Mat_<double>(3, 1) << 0, 0, 500);
        require(!backProjector.back_project(grazingRotation, frontTranslation, {cv::Point2f(480, 270)}, preservedOutput),
            "Grazing ray was not rejected");
        require(preservedOutput.size() == 1 && preservedOutput.front() == cv::Point3f(1, 2, 0),
            "Failed back-projection partially changed output");
        {
            cv::FileStorage file(cameraFile, cv::FileStorage::WRITE);
            file << "camera_matrix" << intrinsics << "distortion_coefficients" << cv::Mat::zeros(5, 1, CV_64F);
        }
        backProjector.load_camera_matrix(cameraFile);
        std::vector<cv::Point2f> undistortedMeasurements;
        cv::projectPoints(knownPlanePoints, boardRotation, frontTranslation, intrinsics, cv::noArray(), undistortedMeasurements);
        std::vector<cv::Point3f> undistortedRecovered;
        require(backProjector.back_project(boardRotation, frontTranslation, undistortedMeasurements, undistortedRecovered),
            "Zero-distortion back-projection failed");
        for (size_t i = 0; i < undistortedRecovered.size(); ++i) {
            require(cv::norm(undistortedRecovered[i] - knownPlanePoints[i]) < 0.01, "Zero-distortion geometry changed");
        }
		const auto projectionModelFile=output.string()+".projection.xml";
		RemoveOutput cleanupProjectionModel{projectionModelFile};
		{
			cv::FileStorage file(projectionModelFile,cv::FileStorage::WRITE);
			file << "camera_matrix" << (cv::Mat_<double>(3,3) << 700,0,960,0,710,540,0,0,1)
				<< "distortion_coefficients" << cv::Mat::zeros(5,1,CV_64F)
				<< "Rotation_Vector" << cv::Mat::zeros(3,1,CV_64F)
				<< "Translation_Vector" << cv::Mat::zeros(3,1,CV_64F);
		}
		auto configureProjection = [&](Tinker::calibration& manager) {
			manager.setup_camera_calibration_parameters({9,6},markedBoard.size(),36,1,8,1000,Tinker::DETECTION,0,cameraFile);
			manager.setup_projector_calibration_parameters({1920,1080},projectionModelFile,{4,5},120,8,5,Tinker::ASYMMETRIC_CIRCLES_GRID,0,0);
			manager.load(cameraFile,projectionModelFile,projectionModelFile);
		};
		Tinker::calibration movingProjection, freshProjection;
		configureProjection(movingProjection); configureProjection(freshProjection);
		cv::Mat shiftedBoard;
		const cv::Mat shift=(cv::Mat_<double>(2,3) << 1,0,100,0,1,0);
		cv::warpAffine(markedBoard,shiftedBoard,shift,markedBoard.size(),cv::INTER_NEAREST,cv::BORDER_CONSTANT,cv::Scalar(255));
		FLAGS_projector_smoothing_rate=0.01;
		require(movingProjection.set_dynamic_projector_image_points(markedBoard,true) &&
			movingProjection.set_dynamic_projector_image_points(shiftedBoard,true) &&
			freshProjection.set_dynamic_projector_image_points(shiftedBoard,true), "Dynamic projection fixture failed");
		cv::Mat movingImage(1080,1920,CV_8UC1), freshImage(1080,1920,CV_8UC1);
		movingProjection.draw_projector_pattern(movingImage); freshProjection.draw_projector_pattern(freshImage);
		require(cv::norm(movingImage,freshImage,cv::NORM_INF)==0, "Dynamic projection retained pose or point smoothing");
		FLAGS_projector_smoothing_rate=1;
		require(movingProjection.set_dynamic_projector_image_points(markedBoard,true) &&
			movingProjection.set_dynamic_projector_image_points(shiftedBoard,false), "Tracking placement fixture failed");
		movingProjection.draw_projector_pattern(movingImage);
		require(cv::norm(movingImage,freshImage,cv::NORM_INF)==0, "Tracking changed region or independently capped circle movement");
		const cv::Mat downwardShift=(cv::Mat_<double>(2,3) << 1,0,0,0,1,150);
		cv::warpAffine(markedBoard,shiftedBoard,downwardShift,markedBoard.size(),cv::INTER_NEAREST,cv::BORDER_CONSTANT,cv::Scalar(255));
		require(!movingProjection.set_dynamic_projector_image_points(shiftedBoard,true), "Clipped dynamic grid was displayed");
		require(movingProjection.set_dynamic_projector_image_points(shiftedBoard,false), "Partial tracking grid was unnecessarily blanked");
		movingProjection.draw_projector_pattern(movingImage);
		require(cv::countNonZero(movingImage)>0, "Partial tracking grid rendered no visible circles");
		require(!movingProjection.set_dynamic_projector_image_points(cv::Mat::zeros(markedBoard.size(),CV_8UC1),false),
			"Tracking displayed a grid after losing the complete chessboard");
		FLAGS_projector_smoothing_rate=0.4;
        projector.setup_projector_parameters(cv::Size(1920, 1080), output.string(),
            cv::Size(4, 5), 120, 8, Tinker::ASYMMETRIC_CIRCLES_GRID, 0, 0);
        projector.set_static_candidate_image_points();
        for (int i = 0; i < 12; ++i) {
            cv::Mat rotation = (cv::Mat_<double>(3, 1) << (i - 5) * 0.04, (i % 3 - 1) * 0.12, i * 0.02);
            cv::Mat translation = (cv::Mat_<double>(3, 1) << (i - 5) * 15, (i % 3 - 1) * 20, 700 + i * 20);
            cv::Mat matrix;
            cv::Rodrigues(rotation, matrix);
            const cv::Mat normal = matrix.col(2);
            std::vector<cv::Point3f> objects;
            for (const auto& pixel : projector.get_candidate_image_points()) {
                cv::Mat ray = (cv::Mat_<double>(3, 1) << (pixel.x - 960) / 1400.0, (pixel.y - 540) / 1400.0, 1);
                cv::Mat point = matrix.t() * (ray * (normal.dot(translation) / normal.dot(ray)) - translation);
                objects.emplace_back(static_cast<float>(point.at<double>(0)), static_cast<float>(point.at<double>(1)), 0.0f);
            }
            projector.objectPoints.push_back(objects);
            projector.imagePoints.push_back(projector.get_candidate_image_points());
            projector.camBoardRotations.push_back(rotation);
            projector.camBoardTranslations.push_back(translation);
            projector.frameMeasuredBoardImagePoints.push_back({cv::Point2f(500.0f + i * 35, 400)});
            projector.frameMeasuredCircleImagePoints.push_back({cv::Point2f(960, 540)});
        }
        FLAGS_minimum_projector_pattern_span = 1.0;
        require(!projector.calibrate(cv::Size(1920, 1080)), "Insufficient projector footprint was accepted");
        require(projector.imagePoints.size() == 12, "Failed coverage discarded candidates");
        FLAGS_minimum_projector_pattern_span = 0.2;
        FLAGS_minimum_projector_board_position_span = 1.0;
        require(!projector.calibrate(cv::Size(1920, 1080)), "Insufficient board position diversity was accepted");
        FLAGS_minimum_projector_board_position_span = 0.1;
        require(projector.calibrate(cv::Size(1920, 1080)), "Synthetic static calibration did not complete");
        require(projector.imagePoints.size() == 8 && projector.objectPoints.size() == 8 &&
            projector.frameMeasuredBoardImagePoints.size() == 8 && projector.frameMeasuredCircleImagePoints.size() == 8 &&
            projector.camBoardRotations.size() == 8 && projector.camBoardTranslations.size() == 8,
            "Selected projector arrays are misaligned");
        require(std::abs(projector.get_camera_matrix().at<double>(0, 0) - 1400) < 1,
            "Known projector focal length was not recovered");
        require(projector.get_last_avg_reprojection_error() < 0.01, "Synthetic projector RMS too high");
		require(projector.get_dist_coeffs().total() == 5 && projector.get_dist_coeffs().at<double>(4) == 0,
			"Projector k3 was not fixed to zero");
		{
			cv::FileStorage file(output.string(), cv::FileStorage::READ);
			int savedFlags=0; file["flags"] >> savedFlags;
			require((savedFlags & cv::CALIB_FIX_K3) != 0, "Saved calibration did not record fixed k3");
		}
		const auto invalidModelFile=output.string()+".invalid.xml";
		RemoveOutput cleanupInvalid{invalidModelFile};
		{
			cv::FileStorage file(invalidModelFile,cv::FileStorage::WRITE);
			file << "camera_matrix" << modelMatrix << "distortion_coefficients" <<
				(cv::Mat_<double>(5,1) << -2.045639946,83.604952868,0.055676459,0.016060684,-964.263678837);
		}
		const auto preservedModel=projector.get_camera_matrix().clone();
		projector.load_calibration_parameters(invalidModelFile);
		require(cv::norm(projector.get_camera_matrix()-preservedModel)==0, "Rejected loaded model replaced working intrinsics");
        std::cout << "Projector calibration regression tests passed\n";
        return 0;
    }
    catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
