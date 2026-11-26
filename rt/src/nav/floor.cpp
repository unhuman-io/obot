#include "obot/nav/floor.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <numeric>

namespace obot::nav {

double look_ahead(const std::vector<float>& base_points, const uint16_t* depth, int width, int height, const Way& way) {
    // every fourth pixel of the middle of the picture
    int readings = 0, missing = 0;
    for (int v = height / 4; v < height * 9 / 10; v += 4) {
        for (int u = width * 3 / 10; u < width * 7 / 10; u += 4) {
            readings++;
            missing += depth[static_cast<std::ptrdiff_t>(v) * width + u] == 0;
        }
    }
    if (readings && missing * 2 > readings) {
        return 0.0;
    }
    std::vector<float> ahead;
    for (std::size_t i = 0; i + 2 < base_points.size(); i += 3) {
        const float x = base_points[i], y = base_points[i + 1], z = base_points[i + 2];
        if (z > way.z_low && z < way.z_high && std::fabs(y) < way.half_width && x > 0.0f) {
            ahead.push_back(x);
        }
    }
    if (ahead.size() <= 11) {
        return std::numeric_limits<double>::infinity();
    }
    // a dozen readings, not one: single wrong readings are common
    std::nth_element(ahead.begin(), ahead.begin() + 11, ahead.end());
    return ahead[11];
}

FloorSeen::FloorSeen(double cell, int half)
    : cell_(cell), half_(half), side_(2 * half + 1), seen_(static_cast<std::size_t>(side_) * side_, 0) {}

void FloorSeen::clear() {
    std::fill(seen_.begin(), seen_.end(), 0);
    stood_i_ = stood_j_ = -1;
}

std::size_t FloorSeen::count() const {
    return static_cast<std::size_t>(std::count(seen_.begin(), seen_.end(), 1));
}

void FloorSeen::mark(double x, double y) {
    const long i = static_cast<long>(std::floor(x / cell_)) + half_,
               j = static_cast<long>(std::floor(y / cell_)) + half_;
    if (i >= 0 && i < side_ && j >= 0 && j < side_) {
        seen_[static_cast<std::size_t>(j) * side_ + i] = 1;
    }
}

void FloorSeen::stand(double x, double y, double radius) {
    const long n = static_cast<long>(radius / cell_) + 1;
    const long i = static_cast<long>(std::floor(x / cell_)) + half_,
               j = static_cast<long>(std::floor(y / cell_)) + half_;
    const long limit = 2L * half_ - n;
    if ((i == stood_i_ && j == stood_j_) || !(n <= i && i < limit && n <= j && j < limit)) {
        return;
    }
    stood_i_ = i;
    stood_j_ = j;
    for (long dj = -n; dj <= n; dj++) {
        for (long di = -n; di <= n; di++) {
            if (static_cast<double>(di * di + dj * dj) * cell_ * cell_ <= radius * radius) {
                seen_[static_cast<std::size_t>(j + dj) * side_ + (i + di)] = 1;
            }
        }
    }
}

void FloorSeen::see(const std::vector<float>& base_points, double mount_x, double mount_y, double x, double y,
                    double yaw, double step, const Way& way) {
    struct Reading {
        long direction;
        double distance, dx, dy;
        bool floor;
    };
    std::vector<Reading> readings;
    readings.reserve(base_points.size() / 3);
    bool any_floor = false;
    for (std::size_t i = 0; i + 2 < base_points.size(); i += 3) {
        const double z = base_points[i + 2];
        const bool floor = std::fabs(z) < 0.04;
        const bool thing = z > way.z_low && z < way.z_high;
        if (!floor && !thing) {
            continue;
        }
        const double dx = base_points[i] - mount_x, dy = base_points[i + 1] - mount_y;
        readings.push_back({std::lrint(std::atan2(dy, dx) / step), std::hypot(dx, dy), dx, dy, floor});
        any_floor |= floor;
    }
    if (!any_floor) {
        return;
    }
    std::stable_sort(readings.begin(), readings.end(), [](const Reading& a, const Reading& b) {
        return a.direction != b.direction ? a.direction < b.direction : a.distance < b.distance;
    });
    const double c = std::cos(yaw), s = std::sin(yaw);
    auto free_at = [&](double lx, double ly) {  // a place in the camera's own flat frame
        const double bx = lx + mount_x, by = ly + mount_y;
        mark(x + c * bx - s * by, y + s * bx + c * by);
    };
    const int samples = static_cast<int>(1.0 / (cell_ / 2)) + 2;
    auto stretch = [&](double ax, double ay, double bx, double by) {
        for (int k = 0; k < samples; k++) {
            const double t = static_cast<double>(k) / (samples - 1);
            free_at(ax + (bx - ax) * t, ay + (by - ay) * t);
        }
    };
    for (std::size_t i = 0; i < readings.size(); i++) {
        const Reading& r = readings[i];
        if (!r.floor) {
            continue;
        }
        free_at(r.dx, r.dy);
        const bool first = i == 0 || readings[i - 1].direction != r.direction;
        if (first && r.distance < 1.0) {
            stretch(0.0, 0.0, r.dx, r.dy);
        }
        if (i + 1 < readings.size()) {
            const Reading& next = readings[i + 1];
            const double gap = next.distance - r.distance;
            if (next.direction == r.direction && next.floor && gap < 1.0 && gap > cell_ / 2) {
                stretch(r.dx, r.dy, next.dx, next.dy);
            }
        }
    }
}

std::vector<uint8_t> FloorSeen::known(int i0, int j0, int width, int height) const {
    std::vector<uint8_t> out(static_cast<std::size_t>(std::max(0, width)) * std::max(0, height), 0);
    for (int j = 0; j < height; j++) {
        const long row = static_cast<long>(j0) + j + half_;
        if (row < 0 || row >= side_) {
            continue;
        }
        for (int i = 0; i < width; i++) {
            const long column = static_cast<long>(i0) + i + half_;
            if (column >= 0 && column < side_) {
                out[static_cast<std::size_t>(j) * width + i] = seen_[static_cast<std::size_t>(row) * side_ + column];
            }
        }
    }
    return out;
}

std::vector<uint8_t> FloorSeen::grid(const VoxelMap& map, const Way& way, double keep_clear) const {
    constexpr uint32_t magic = 0x3147424Fu;  // "OBG1"
    std::vector<uint8_t> occupied(seen_.size(), 0);
    for (const auto& [ix, iy] : map.obstacles(way.z_low, way.z_high)) {
        const long i = static_cast<long>(std::floor(ix * map.voxel() / cell_)) + half_;
        const long j = static_cast<long>(std::floor(iy * map.voxel() / cell_)) + half_;
        if (i >= 0 && i < side_ && j >= 0 && j < side_) {
            occupied[static_cast<std::size_t>(j) * side_ + i] = 1;
        }
    }
    long a = side_, b = -1, c = side_, d = -1;  // rows a..b, columns c..d that have anything
    for (long j = 0; j < side_; j++) {
        for (long i = 0; i < side_; i++) {
            const std::size_t at = static_cast<std::size_t>(j) * side_ + i;
            if (seen_[at] || occupied[at]) {
                a = std::min(a, j);
                b = std::max(b, j);
                c = std::min(c, i);
                d = std::max(d, i);
            }
        }
    }
    std::vector<uint8_t> out;
    auto put = [&out](const void* value, std::size_t size) {
        const auto* p = static_cast<const uint8_t*>(value);
        out.insert(out.end(), p, p + size);
    };
    const auto cell = static_cast<float>(cell_);
    if (b < 0) {
        const int32_t zero = 0;
        put(&magic, 4);
        for (int k = 0; k < 4; k++) {
            put(&zero, 4);
        }
        put(&cell, 4);
        return out;
    }
    const long pad = static_cast<long>(0.5 / cell_);
    a = std::max(0L, a - pad);
    b = std::min(static_cast<long>(side_) - 1, b + pad) + 1;
    c = std::max(0L, c - pad);
    d = std::min(static_cast<long>(side_) - 1, d + pad) + 1;
    const long width = d - c, height = b - a;
    std::vector<uint8_t> cells(static_cast<std::size_t>(width * height), 0);
    for (long j = 0; j < height; j++) {
        for (long i = 0; i < width; i++) {
            const std::size_t from = static_cast<std::size_t>(j + a) * side_ + (i + c);
            cells[static_cast<std::size_t>(j * width + i)] = occupied[from] ? 2 : seen_[from];
        }
    }
    // free, but closer to something than keep_clear: the middle of the base cannot be there
    const long reach = static_cast<long>(keep_clear / cell_) + 1;
    for (long j = 0; j < height; j++) {
        for (long i = 0; i < width; i++) {
            if (cells[static_cast<std::size_t>(j * width + i)] != 2) {
                continue;
            }
            for (long dj = std::max(-reach, -j); dj <= reach && j + dj < height; dj++) {
                for (long di = std::max(-reach, -i); di <= reach && i + di < width; di++) {
                    uint8_t& near = cells[static_cast<std::size_t>((j + dj) * width + i + di)];
                    if (near == 1 && std::sqrt(static_cast<double>(di * di + dj * dj)) * cell_ < keep_clear) {
                        near = 3;
                    }
                }
            }
        }
    }
    const auto i0 = static_cast<int32_t>(c - half_), j0 = static_cast<int32_t>(a - half_);
    const auto w = static_cast<uint32_t>(width), h = static_cast<uint32_t>(height);
    put(&magic, 4);
    put(&i0, 4);
    put(&j0, 4);
    put(&w, 4);
    put(&h, 4);
    put(&cell, 4);
    out.insert(out.end(), cells.begin(), cells.end());
    return out;
}

void FloorSeen::dump(std::vector<uint8_t>& out) const {
    // as runs of unseen and seen squares in turn: the grid is mostly one or the other
    const auto* p = reinterpret_cast<const uint8_t*>(&cell_);
    out.insert(out.end(), p, p + sizeof(cell_));
    const auto half = static_cast<int32_t>(half_);
    p = reinterpret_cast<const uint8_t*>(&half);
    out.insert(out.end(), p, p + 4);
    std::vector<uint32_t> runs;
    uint8_t value = 0;
    uint32_t run = 0;
    for (uint8_t cell : seen_) {
        if (cell != value) {
            runs.push_back(run);
            run = 0;
            value = cell;
        }
        run++;
    }
    runs.push_back(run);
    const auto count = static_cast<uint32_t>(runs.size());
    p = reinterpret_cast<const uint8_t*>(&count);
    out.insert(out.end(), p, p + 4);
    p = reinterpret_cast<const uint8_t*>(runs.data());
    out.insert(out.end(), p, p + runs.size() * 4);
}

bool FloorSeen::restore(const uint8_t* data, std::size_t size, std::size_t& used) {
    std::size_t at = used;
    double cell = 0;
    int32_t half = 0;
    uint32_t count = 0;
    if (at + 16 > size) {
        return false;
    }
    std::memcpy(&cell, data + at, 8);
    std::memcpy(&half, data + at + 8, 4);
    std::memcpy(&count, data + at + 12, 4);
    at += 16;
    if (std::abs(cell - cell_) > 1e-9 || half != half_ || at + static_cast<std::size_t>(count) * 4 > size) {
        return false;
    }
    std::vector<uint8_t> seen;
    seen.reserve(seen_.size());
    uint8_t value = 0;
    for (uint32_t k = 0; k < count; k++) {
        uint32_t run = 0;
        std::memcpy(&run, data + at, 4);
        at += 4;
        if (seen.size() + run > seen_.size()) {
            return false;
        }
        seen.insert(seen.end(), run, value);
        value = !value;
    }
    if (seen.size() != seen_.size()) {
        return false;
    }
    seen_ = std::move(seen);
    stood_i_ = stood_j_ = -1;
    used = at;
    return true;
}

std::vector<uint8_t> sample_colors(const std::vector<float>& points, const uint8_t* bgr, const Intrinsics& k) {
    std::vector<uint8_t> rgb(points.size(), 0);
    for (std::size_t i = 0; i + 2 < points.size(); i += 3) {
        const float z = points[i + 2];
        if (!(z > 0.05f)) {
            continue;
        }
        const long u = std::lrint(points[i] / z * k.fx + k.cx), v = std::lrint(points[i + 1] / z * k.fy + k.cy);
        if (u < 0 || u >= k.width || v < 0 || v >= k.height) {
            continue;
        }
        const uint8_t* pixel = bgr + (v * k.width + u) * 3;
        // never exactly black: that means "no colour"
        rgb[i] = std::max<uint8_t>(pixel[2], 1);
        rgb[i + 1] = std::max<uint8_t>(pixel[1], 1);
        rgb[i + 2] = std::max<uint8_t>(pixel[0], 1);
    }
    return rgb;
}

}  // namespace obot::nav
