#pragma once
#include <stdexcept>
#include <vector>

namespace Tinker {
struct DisplayGeometry {
    int left=0;
    int top=0;
    int width=0;
    int height=0;
    bool primary=false;
};

inline size_t select_projector_display(const std::vector<DisplayGeometry>& displays, int requestedIndex)
{
    if (displays.empty()) throw std::runtime_error("Windows reported no displays.");
    if (requestedIndex >= 0) {
        if (static_cast<size_t>(requestedIndex) >= displays.size())
            throw std::invalid_argument("--projector_display_index is outside the detected display list.");
        return static_cast<size_t>(requestedIndex);
    }
    if (requestedIndex != -1)
        throw std::invalid_argument("--projector_display_index must be -1 (automatic) or a nonnegative display index.");
    for (size_t index=0;index<displays.size();++index)
        if (!displays[index].primary) return index;
    throw std::runtime_error("No non-primary display detected. In Windows Display settings, connect the projector and select Extend these displays, or explicitly choose a detected index.");
}
}
