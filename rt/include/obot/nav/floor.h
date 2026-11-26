// What a depth camera on the base tells about the floor around it, for driving: where the
// floor has been seen to be free, and how far ahead the nearest thing in the base's way is.
// The C++ counterpart of the same in obot.ui.worldview (see there for the reasoning).
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "obot/nav/voxel_map.h"

namespace obot::nav {

// What stands in the base's way counts between these heights above the floor (what is
// lower the wheels roll over), within this much either side of the middle of the base.
struct Way {
    double half_width = 0.26;
    double z_low = 0.05, z_high = 0.7;
};

// How far ahead of the middle of the wheel axis the nearest thing in the base's way is, m
// (infinity: nothing). `points`: x, y, z in the base frame (x forward, y left, z up from
// the floor). 0 if the middle of the depth picture is mostly without readings: a depth
// camera gives none for what is too close to it.
double look_ahead(const std::vector<float>& base_points, const uint16_t* depth, int width, int height, const Way& way);

// The floor from above as a grid of `cell`-sized squares around the origin of the map,
// reaching `half` cells each way: which of them have been seen to be free.
class FloorSeen {
 public:
    FloorSeen(double cell = 0.06, int half = 250);
    void clear();
    // Where the base stands, `radius` around the middle of its wheel axis is free.
    void stand(double x, double y, double radius);
    // What a picture shows to be free: where it sees the floor, the stretch between two
    // such places one behind the other in one direction with nothing standing in between,
    // and the stretch from the camera to the first of them. `points` in the base frame,
    // (mount_x, mount_y) where the camera is in it, (x, y, yaw) where the base is in the
    // map, `step` the angle between two columns of the picture.
    void see(const std::vector<float>& base_points, double mount_x, double mount_y, double x, double y, double yaw,
             double step, const Way& way);
    // Whether the square [j][i] of a grid whose square [0][0] reaches from (i0 * cell,
    // j0 * cell) has been seen to be free: width * height bytes, row by row.
    std::vector<uint8_t> known(int i0, int j0, int width, int height) const;
    // The map from above for the operator page: the layout of WorldView.grid().
    std::vector<uint8_t> grid(const VoxelMap& map, const Way& way, double keep_clear) const;

    double cell() const { return cell_; }
    int half() const { return half_; }
    std::size_t count() const;
    void dump(std::vector<uint8_t>& out) const;
    bool restore(const uint8_t* data, std::size_t size, std::size_t& used);

 private:
    void mark(double x, double y);
    double cell_;
    int half_, side_;
    std::vector<uint8_t> seen_;  // side_ * side_, row (y) by row
    long stood_i_ = -1, stood_j_ = -1;
};

// The colour (r, g, b) a colour camera sees at each point, the points being in that
// camera's optical frame; 0 0 0 where a point is outside its picture.
std::vector<uint8_t> sample_colors(const std::vector<float>& points, const uint8_t* bgr, const Intrinsics& k);

}  // namespace obot::nav
