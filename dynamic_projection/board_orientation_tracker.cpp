#include "board_orientation_tracker.hpp"
#include "flags.hpp"
#include <algorithm>
#include <cmath>

bool Tinker::BoardOrientationTracker::orient(std::vector<cv::Point2f>& corners, cv::Size boardSize,
    const std::vector<BoardOrientationMarker>& markers, std::string& status)
{
    if (corners.size() != static_cast<size_t>(boardSize.area()) || boardSize.width < 2 || boardSize.height < 2) {
        reset(); status = "complete chessboard missing; show an orientation marker again"; return false;
    }
    // Enumerate grid-axis directions, not image-space "top left". This also
    // handles mirrored camera images while preserving the physical board axes.
    std::vector<std::vector<cv::Point2f>> candidates(4, corners);
    for (int flip = 0; flip < 4; ++flip)
        for (int y = 0; y < boardSize.height; ++y)
            for (int x = 0; x < boardSize.width; ++x)
                candidates[flip][y * boardSize.width + x] = corners[
                    ((flip & 2) ? boardSize.height - 1 - y : y) * boardSize.width +
                    ((flip & 1) ? boardSize.width - 1 - x : x)];
    const auto marker = std::find_if(markers.begin(),markers.end(),[](const BoardOrientationMarker& item) {
        return item.id==FLAGS_board_origin_aruco_id;
    });
    const bool haveMarker=marker!=markers.end();
    auto rank = [&](auto score) {
        std::vector<std::pair<double, size_t>> scores;
        for (size_t i = 0; i < candidates.size(); ++i) scores.emplace_back(score(candidates[i]), i);
        std::sort(scores.begin(), scores.end());
        return scores;
    };
    size_t selected = 0;
    if (haveMarker) {
        const auto scores=rank([&](const std::vector<cv::Point2f>& candidate) {
            return cv::norm(candidate.front()-marker->center);
        });
        double spacing=0;
        size_t count=0;
        for (int y=0;y<boardSize.height;++y) for (int x=1;x<boardSize.width;++x) {
            spacing+=cv::norm(corners[y*boardSize.width+x]-corners[y*boardSize.width+x-1]);
            ++count;
        }
        spacing/=count;
        if (scores[1].first-scores[0].first<spacing*0.5) {
            reset(); status="origin marker is not uniquely nearest to one outer inner corner; move it closer to the intended origin"; return false;
        }
        selected = scores[0].second;
        status = "origin anchored by ArUco ID " + std::to_string(marker->id) + " at nearest inner corner";
    } else {
        if (previous.size() != corners.size()) {
            reset(); status = "origin uninitialized; show ArUco ID " + std::to_string(FLAGS_board_origin_aruco_id) +
                " near the intended origin corner"; return false;
        }
        auto displacement = [&](const std::vector<cv::Point2f>& candidate) {
            double squared = 0;
            for (size_t i = 0; i < candidate.size(); ++i) { const auto delta = candidate[i] - previous[i]; squared += delta.dot(delta); }
            return std::sqrt(squared / candidate.size());
        };
        double spacing = 0;
        size_t count = 0;
        for (int y = 0; y < boardSize.height; ++y) for (int x = 1; x < boardSize.width; ++x) {
            spacing += cv::norm(corners[y * boardSize.width + x] - corners[y * boardSize.width + x - 1]); ++count;
        }
        spacing /= count;
        const auto scores = rank(displacement);
        if (scores[0].first > FLAGS_board_tracking_max_motion_squares * spacing ||
            scores[1].first - scores[0].first < spacing * 0.5) {
            reset(); status = "origin continuity ambiguous or motion too large; show an orientation marker again"; return false;
        }
        selected = scores[0].second;
        status = "origin maintained by chessboard continuity";
    }
    corners = candidates[selected];
    previous = corners;
    return true;
}
