// The 3D map of what the depth cameras have seen: one entry per cube of `voxel` metres that
// something was seen in. The C++ counterpart of obot.world.WorldMap in the Python package,
// with the same rules and the same update format for the operator page ("OBW1"), so that
// either can serve a page. See there for the reasoning; in short:
//
//   - a voxel is shown once it has been seen `confirm` times (stray readings never appear),
//   - it is taken out again when a camera has looked through it as often as it had been
//     seen (at most `max_hits` times),
//   - when the map is full, what has not been looked at for longest is forgotten.
//
// Voxels are kept in one dense array, found through a hash table of their indices. Not
// thread safe: whoever owns the map calls it from one thread.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace obot::nav {

// A pinhole camera: pixel = focal length * (x / z, y / z) + centre.
struct Intrinsics {
    int width = 0, height = 0;
    double fx = 0, fy = 0, cx = 0, cy = 0;
};

// A pose as the top three rows of a 4 x 4 matrix, row by row: rotation and translation.
struct Pose {
    std::array<double, 12> m{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};
    void apply(float x, float y, float z, float& ox, float& oy, float& oz) const {
        ox = static_cast<float>(m[0] * x + m[1] * y + m[2] * z + m[3]);
        oy = static_cast<float>(m[4] * x + m[5] * y + m[6] * z + m[7]);
        oz = static_cast<float>(m[8] * x + m[9] * y + m[10] * z + m[11]);
    }
};

// Every stride-th pixel of a depth picture (millimetres, 0 = no reading) as a point in the
// camera's optical frame (x right, y down, z along the view), metres: x, y, z, x, y, z ...
// Readings outside near .. far are left out.
std::vector<float> depth_to_points(const uint16_t* depth, const Intrinsics& k, int stride, double near, double far);

class VoxelMap {
 public:
    explicit VoxelMap(double voxel = 0.03, std::size_t max_voxels = 250000, int confirm = 2, int max_hits = 6);

    // Add points seen now (map frame, metres; n of them) with their colours (r, g, b per
    // point, 0 0 0 = unknown; null: none known).
    void integrate(const float* xyz, const uint8_t* rgb, std::size_t n);
    // Take out what a camera now sees through: voxels in front of what it measures along
    // the same line of sight. `camera`: pose of its optical frame in the map. Returns how
    // many voxels were removed.
    int carve(const Pose& camera, const uint16_t* depth, const Intrinsics& k, double near, double far);
    // Remember where the robot has been (a point for every `step` metres travelled).
    void add_pose(double x, double y, double step = 0.02, std::size_t limit = 20000);
    void clear();

    // What changed after revision `since` of map `epoch`, for a page that has `trail` trail
    // points: the layout of WorldMap.delta().
    std::vector<uint8_t> delta(uint32_t since, uint32_t epoch, uint32_t trail) const;
    // Where the base cannot drive: the (ix, iy) of shown voxels between z_min and z_max
    // above the floor, each place once. The place is (ix * voxel, iy * voxel).
    std::vector<std::pair<int, int>> obstacles(double z_min, double z_max) const;

    // To a stream of bytes and back (the map as it is, for keeping it across restarts).
    void dump(std::vector<uint8_t>& out) const;
    bool restore(const uint8_t* data, std::size_t size, std::size_t& used);

    double voxel() const { return voxel_; }
    std::size_t size() const { return voxels_.size(); }  // with those not yet shown
    std::size_t shown() const { return shown_; }
    uint32_t rev() const { return rev_; }
    uint32_t epoch() const { return epoch_; }
    std::size_t trail_size() const { return trail_.size(); }

 private:
    struct Voxel {
        int16_t ix, iy, iz;
        uint8_t hits;
        uint8_t shown;
        uint8_t r, g, b;
        uint32_t changed;  // revision in which what a page draws of it last changed
        uint32_t seen;     // revision in which it was last observed
    };
    static uint64_t key(int ix, int iy, int iz);
    uint32_t find(uint64_t k) const;  // index into voxels_, or kNone
    void insert(uint64_t k, uint32_t index);
    void rehash(std::size_t slots);
    void drop(const std::vector<uint32_t>& gone);  // indices, ascending
    void new_epoch();

    static constexpr uint32_t kNone = 0xFFFFFFFFu;
    double voxel_;
    std::size_t max_voxels_;
    int confirm_, max_hits_;
    std::vector<Voxel> voxels_;
    std::vector<uint32_t> table_;  // open addressing, linear probing
    std::size_t shown_ = 0;
    uint32_t epoch_ = 1, rev_ = 0, horizon_ = 0;
    struct Removed {
        uint32_t rev;
        std::vector<int16_t> index;  // ix, iy, iz, ...
    };
    std::vector<Removed> removed_;
    std::vector<std::pair<double, double>> trail_;
    std::size_t trail_start_ = 0;
};

}  // namespace obot::nav
