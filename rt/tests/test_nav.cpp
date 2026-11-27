#include "doctest.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <set>
#include <tuple>

#include "obot/nav/floor.h"
#include "obot/nav/voxel_map.h"

using namespace obot::nav;

namespace {

// What a page makes of an update (the inverse of VoxelMap::delta).
struct Update {
    uint32_t epoch, rev;
    bool reset;
    std::set<std::tuple<int, int, int>> changed, removed;
    std::vector<std::pair<float, float>> trail;
    uint32_t trail_start;
};

Update decode(const std::vector<uint8_t>& data) {
    uint32_t head[8];
    std::memcpy(head, data.data(), 32);
    REQUIRE(head[0] == 0x3157424Fu);
    Update u{head[1], head[2], (head[3] & 1) != 0, {}, {}, {}, head[6]};
    std::size_t at = 36;
    for (uint32_t i = 0; i < head[7]; i++, at += 8) {
        float xy[2];
        std::memcpy(xy, data.data() + at, 8);
        u.trail.emplace_back(xy[0], xy[1]);
    }
    for (auto* into : {&u.changed, &u.removed}) {
        const uint32_t n = into == &u.changed ? head[4] : head[5];
        for (uint32_t i = 0; i < n; i++, at += 6) {
            int16_t index[3];
            std::memcpy(index, data.data() + at, 6);
            into->emplace(index[0], index[1], index[2]);
        }
    }
    CHECK(at + head[4] * 3 == data.size());
    return u;
}

}  // namespace

TEST_CASE("a voxel is shown once it has been seen twice, and sent to a page once") {
    VoxelMap map(0.03);
    const float seen[] = {0.30f, 0.0f, 0.0f, 0.31f, 0.0f, 0.0f, 0.60f, 0.0f, 0.0f};  // two points in one voxel
    map.integrate(seen, nullptr, 3);
    CHECK(map.size() == 2);
    CHECK(map.shown() == 0);
    Update first = decode(map.delta(0, 0, 0));
    CHECK(first.reset);
    CHECK(first.changed.empty());
    map.integrate(seen, nullptr, 1);
    CHECK(map.shown() == 1);
    Update second = decode(map.delta(first.rev, first.epoch, 0));
    CHECK_FALSE(second.reset);
    CHECK(second.changed == std::set<std::tuple<int, int, int>>{{10, 0, 0}});
    CHECK(decode(map.delta(second.rev, second.epoch, 0)).changed.empty());
    // a page with another map, or one from the future, is sent everything
    CHECK(decode(map.delta(second.rev, second.epoch + 1, 0)).changed.size() == 1);
    CHECK(decode(map.delta(second.rev + 5, second.epoch, 0)).reset);
}

TEST_CASE("what a camera sees through is taken out, what it sees is kept") {
    VoxelMap map(0.03);
    // a wall of points 1 m in front of a camera at the origin looking along x (optical z = map x)
    std::vector<float> wall;
    for (float y = -0.3f; y <= 0.3f; y += 0.03f) {
        for (float z = -0.3f; z <= 0.3f; z += 0.03f) {
            wall.insert(wall.end(), {1.0f, y, z});
        }
    }
    for (int i = 0; i < 4; i++) {
        map.integrate(wall.data(), nullptr, wall.size() / 3);
    }
    const std::size_t there = map.shown();
    CHECK(there > 300);
    Pose camera;  // optical x = -map y, optical y = -map z, optical z = map x
    camera.m = {0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0};
    const Intrinsics k{160, 120, 100, 100, 79.5, 59.5};
    std::vector<uint16_t> depth(160 * 120, 1000);  // it measures the wall: nothing goes
    CHECK(map.carve(camera, depth.data(), k, 0.2, 3.5) == 0);
    std::fill(depth.begin(), depth.end(), uint16_t{0});  // no reading says nothing
    CHECK(map.carve(camera, depth.data(), k, 0.2, 3.5) == 0);
    std::fill(depth.begin(), depth.end(), uint16_t{2500});  // it sees 2.5 m: the wall has gone
    const Update before = decode(map.delta(0, 0, 0));
    int removed = 0;
    for (int i = 0; i < 4; i++) {
        removed += map.carve(camera, depth.data(), k, 0.2, 3.5);  // (as often as it had been seen)
    }
    CHECK(removed == static_cast<int>(there));
    CHECK(map.shown() == 0);
    const Update after = decode(map.delta(before.rev, before.epoch, 0));
    CHECK(after.removed == before.changed);
    // and what is put in again afterwards is found again (the table has no holes)
    map.integrate(wall.data(), nullptr, wall.size() / 3);
    map.integrate(wall.data(), nullptr, wall.size() / 3);
    CHECK(map.shown() == there);
}

TEST_CASE("a full map forgets what it has not seen for longest") {
    VoxelMap map(0.03, 1000, 1);
    // twelve pictures of a hundred voxels each, one row after the other
    for (int row = 0; row < 12; row++) {
        std::vector<float> points;
        for (int i = 0; i < 100; i++) {
            points.insert(points.end(), {0.03f * static_cast<float>(i), 0.3f * static_cast<float>(row), 0.0f});
        }
        map.integrate(points.data(), nullptr, 100);
        CHECK(map.size() == (row < 10 ? 100u * (row + 1) : 1000u));  // the eleventh makes it too many: the oldest goes
    }
    const auto left = map.obstacles(-1, 1, 1);
    CHECK(left.size() == 1000);
    CHECK(left.front().second == 20);  // the first two rows have gone (rows are 10 voxels apart)
    const float one[] = {0.0f, 3.0f, 0.0f};
    // the colour of a voxel follows what is seen, when it differs visibly
    const uint8_t red[] = {200, 10, 10}, nearly[] = {205, 12, 9};
    Update u = decode(map.delta(0, 0, 0));
    map.integrate(one, red, 1);
    CHECK(decode(map.delta(u.rev, u.epoch, 0)).changed.size() == 1);
    u = decode(map.delta(0, 0, 0));
    map.integrate(one, nearly, 1);
    CHECK(decode(map.delta(u.rev, u.epoch, 0)).changed.empty());
}

TEST_CASE("the trail, clearing, and keeping a map") {
    VoxelMap map(0.03, 250000, 1);
    for (int i = 0; i <= 100; i++) {
        map.add_pose(0.007 * i, 0.0);  // a point when 2 cm have been travelled: every third call
    }
    Update u = decode(map.delta(0, 0, 0));
    CHECK(u.trail.size() == 34);
    CHECK(u.trail.back().first == doctest::Approx(0.693));
    CHECK(decode(map.delta(u.rev, u.epoch, 20)).trail.size() == 14);
    const float p[] = {1.0f, 2.0f, 0.3f};
    map.integrate(p, nullptr, 1);
    std::vector<uint8_t> saved;
    map.dump(saved);
    VoxelMap again(0.03);
    std::size_t used = 0;
    REQUIRE(again.restore(saved.data(), saved.size(), used));
    CHECK(used == saved.size());
    CHECK(again.shown() == 1);
    CHECK(decode(again.delta(0, 0, 0)).trail.size() == 34);
    CHECK(again.obstacles(0.05, 0.7, 1) == std::vector<std::pair<int, int>>{{33, 67}});
    CHECK(again.obstacles(0.05, 0.7).empty());  // seen once: not something to plan a way around yet
    VoxelMap other(0.05);
    used = 0;
    CHECK_FALSE(other.restore(saved.data(), saved.size(), used));  // cells of another size
    map.clear();
    const Update cleared = decode(map.delta(u.rev, u.epoch, 34));
    CHECK(cleared.reset);
    CHECK(cleared.trail.empty());
    CHECK(cleared.trail_start == 34);
}

TEST_CASE("the floor is free where it is seen, and no farther than what stands on it") {
    FloorSeen floor(0.06, 250);
    const Way way;
    // A camera at the middle of the base, 0.15 m up, looking along x: floor readings every
    // 10 cm straight ahead up to 2 m, and at 30 degrees to the left up to 1 m, where
    // something stands; behind it floor again at 1.5 m (seen over it? no: it is tall).
    std::vector<float> points;
    for (int k = 3; k <= 20; k++) {
        points.insert(points.end(), {0.1f * static_cast<float>(k), 0.0f, 0.0f});
    }
    const float c = std::cos(0.5236f), s = std::sin(0.5236f);
    for (int k = 3; k <= 10; k++) {
        points.insert(points.end(), {0.1f * static_cast<float>(k) * c, 0.1f * static_cast<float>(k) * s, 0.0f});
    }
    for (float z = 0.1f; z < 0.6f; z += 0.05f) {
        points.insert(points.end(), {1.1f * c, 1.1f * s, z});
    }
    points.insert(points.end(), {1.6f * c, 1.6f * s, 0.0f});
    floor.see(points, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, way);
    auto free_at = [&](double x, double y) {
        return floor.known(static_cast<int>(std::floor(x / 0.06)), static_cast<int>(std::floor(y / 0.06)), 1, 1)[0] ==
               1;
    };
    CHECK(free_at(0.1, 0.0));   // from the camera to the first floor it sees
    CHECK(free_at(1.05, 0.0));  // between two sightings
    CHECK(free_at(1.95, 0.0));
    CHECK_FALSE(free_at(2.3, 0.0));
    CHECK(free_at(0.9 * c, 0.9 * s));
    CHECK_FALSE(free_at(1.3 * c, 1.3 * s));  // behind the thing: not seen
    CHECK(free_at(1.6 * c, 1.6 * s));        // (the one sighting beyond it is)
    CHECK_FALSE(free_at(0.0, 0.5));
    floor.stand(0.0, 0.5, 0.24);
    CHECK(free_at(0.0, 0.5));
    CHECK(free_at(0.0, 0.7));
    CHECK_FALSE(free_at(0.0, 0.85));

    // the same seen from a base that stands at (1, 2) turned by 90 degrees
    FloorSeen turned(0.06, 250);
    turned.see(points, 0.0, 0.0, 1.0, 2.0, 1.5708, 0.01, way);
    CHECK(turned.known(static_cast<int>(std::floor(1.0 / 0.06)), static_cast<int>(std::floor(3.5 / 0.06)), 1, 1)[0] ==
          1);

    std::vector<uint8_t> saved;
    floor.dump(saved);
    FloorSeen again(0.06, 250);
    std::size_t used = 0;
    REQUIRE(again.restore(saved.data(), saved.size(), used));
    CHECK(again.count() == floor.count());
    CHECK(again.known(-10, -10, 60, 40) == floor.known(-10, -10, 60, 40));
}

TEST_CASE("the floor plan remembers what stood somewhere, and lets go of it slowly") {
    FloorSeen floor(0.06, 250);
    const Way way;
    // A box 1 m ahead, and the floor in front of it, seen from the origin.
    std::vector<float> with_box, without;
    for (int k = 3; k <= 9; k++) {
        with_box.insert(with_box.end(), {0.1f * static_cast<float>(k), 0.0f, 0.0f});
    }
    without = with_box;
    for (int k = 10; k <= 20; k++) {
        without.insert(without.end(), {0.1f * static_cast<float>(k), 0.0f, 0.0f});  // floor where the box was
    }
    for (float z = 0.1f; z < 0.5f; z += 0.05f) {
        with_box.insert(with_box.end(), {1.0f, 0.0f, z});
    }
    auto there = [&] { return floor.things() == std::vector<std::pair<int, int>>{{16, 0}}; };
    // one picture is not enough (a stray reading), two are
    Agreement first = floor.see(with_box, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, way);
    CHECK(first.things == 1);
    CHECK(first.remembered == 0);
    CHECK(floor.things().empty());
    Agreement second = floor.see(with_box, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, way);
    CHECK(second.remembered == 0);  // (it was not remembered yet when this picture came)
    CHECK(there());
    CHECK(floor.see(with_box, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, way).remembered == 1);
    for (int i = 0; i < 30; i++) {
        floor.see(with_box, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, way);
    }
    // The box is taken away. A few pictures of floor in its place do not make the plan
    // forget it; half a minute of them does (at five pictures a second: 33 of them).
    for (int i = 0; i < 10; i++) {
        floor.see(without, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, way);
    }
    CHECK(there());
    for (int i = 0; i < 25; i++) {
        floor.see(without, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, way);
    }
    CHECK(floor.things().empty());
    // Something where the plan has free floor and nothing near: that contradicts it.
    Agreement moved = floor.see(with_box, 0.0, 0.0, 0.0, 0.5, 0.0, 0.01, way);  // (the base half a metre to the side)
    CHECK(moved.things == 1);
    CHECK(moved.contradicted == 0);  // nobody had seen the floor there
    floor.stand(1.0, -0.5, 0.2);
    Agreement elsewhere = floor.see(with_box, 0.0, 0.0, 0.0, -0.5, 0.0, 0.01, way);
    CHECK(elsewhere.contradicted == 1);

    // kept with the rest
    floor.see(with_box, 0.0, 0.0, 0.0, 0.5, 0.0, 0.01, way);
    REQUIRE(floor.things().size() == 1);
    std::vector<uint8_t> saved;
    floor.dump(saved);
    FloorSeen again(0.06, 250);
    std::size_t used = 0;
    REQUIRE(again.restore(saved.data(), saved.size(), used));
    CHECK(used == saved.size());
    CHECK(again.things() == floor.things());
    // and shown on the map from above as remembered (4), the 3D map having nothing there
    const std::vector<uint8_t> grid = again.grid(VoxelMap(0.03), way, 0.38);
    CHECK(std::count(grid.begin() + 24, grid.end(), 4) == 1);
    CHECK(std::count(grid.begin() + 24, grid.end(), 3) > 3);  // with its margin, where floor has been seen
}

TEST_CASE("the map from above: things, free floor, and the margin around things") {
    VoxelMap map(0.03, 250000, 1);
    FloorSeen floor(0.06, 250);
    const Way way;
    std::vector<float> box;
    for (float z = 0.09f; z < 0.4f; z += 0.03f) {
        box.insert(box.end(), {1.0f, 0.0f, z});
    }
    box.insert(box.end(), {0.5f, 0.5f, 0.0f});  // (floor: not a thing)
    for (int seen = 0; seen < 4; seen++) {      // (often enough to be sure of it)
        map.integrate(box.data(), nullptr, box.size() / 3);
    }
    floor.stand(0.0, 0.0, 1.5);
    const std::vector<uint8_t> grid = floor.grid(map, way, 0.38);
    uint32_t magic, width, height;
    int32_t i0, j0;
    float cell;
    std::memcpy(&magic, grid.data(), 4);
    std::memcpy(&i0, grid.data() + 4, 4);
    std::memcpy(&j0, grid.data() + 8, 4);
    std::memcpy(&width, grid.data() + 12, 4);
    std::memcpy(&height, grid.data() + 16, 4);
    std::memcpy(&cell, grid.data() + 20, 4);
    REQUIRE(magic == 0x3147424Fu);
    REQUIRE(grid.size() == 24 + width * height);
    auto at = [&](double x, double y) {
        const long i = static_cast<long>(std::floor(x / cell)) - i0, j = static_cast<long>(std::floor(y / cell)) - j0;
        return static_cast<int>(grid[24 + static_cast<std::size_t>(j) * width + static_cast<std::size_t>(i)]);
    };
    CHECK(at(1.0, 0.0) == 2);
    CHECK(at(0.8, 0.0) == 3);  // free floor, but within 0.38 m of the box
    CHECK(at(0.3, 0.0) == 1);
    CHECK(at(0.5, 0.5) == 1);
    CHECK(at(1.7, 0.0) == 0);  // nobody has seen that far
    CHECK(FloorSeen(0.06, 250).grid(VoxelMap(0.03), way, 0.38).size() == 24);
}

TEST_CASE("how far ahead the nearest thing in the way is") {
    const Way way;
    std::vector<uint16_t> depth(160 * 120, 1500);
    std::vector<float> points;
    for (int i = 0; i < 30; i++) {
        points.insert(points.end(), {1.2f + 0.01f * static_cast<float>(i), 0.1f, 0.3f});  // in the way
        points.insert(points.end(), {0.6f, 0.5f, 0.3f});                                  // beside it
        points.insert(points.end(), {0.5f, 0.0f, 0.01f});                                 // the floor
    }
    CHECK(look_ahead(points, depth.data(), 160, 120, way) == doctest::Approx(1.31));  // the 12th nearest reading
    points.resize(9 * 3);                                                             // too few readings to count
    CHECK(look_ahead(points, depth.data(), 160, 120, way) == std::numeric_limits<double>::infinity());
    std::fill(depth.begin(), depth.end(), uint16_t{0});  // nothing to be seen in the middle: something is too close
    CHECK(look_ahead(points, depth.data(), 160, 120, way) == 0.0);
}

TEST_CASE("depth pictures become points, and points get the colour camera's colours") {
    const Intrinsics k{8, 6, 4, 4, 3.5, 2.5};
    std::vector<uint16_t> depth(8 * 6, 0);
    depth[3 * 8 + 5] = 2000;  // row 3, column 5
    depth[1 * 8 + 1] = 100;   // nearer than `near`
    const std::vector<float> points = depth_to_points(depth.data(), k, 2, 0.2, 3.5);
    REQUIRE(points.size() == 3);
    CHECK(points[0] == doctest::Approx((5 - 3.5) / 4 * 2.0));
    CHECK(points[1] == doctest::Approx((3 - 2.5) / 4 * 2.0));
    CHECK(points[2] == doctest::Approx(2.0));
    std::vector<uint8_t> bgr(8 * 6 * 3, 0);
    bgr[(3 * 8 + 5) * 3 + 2] = 250;  // red there; green and blue 0
    const std::vector<uint8_t> rgb = sample_colors(points, bgr.data(), k);
    CHECK(rgb == std::vector<uint8_t>{250, 1, 1});  // never exactly black: that means "no colour"
    CHECK(sample_colors({9.0f, 0.0f, 1.0f}, bgr.data(), k) == std::vector<uint8_t>{0, 0, 0});  // outside the picture
}
