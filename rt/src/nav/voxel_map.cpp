#include "obot/nav/voxel_map.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <random>

namespace obot::nav {

namespace {

constexpr int kOffset = 32768;
constexpr int kLimit = 32760;             // voxel indices travel as 16 bit numbers
constexpr uint32_t kMagic = 0x3157424Fu;  // "OBW1"

uint64_t mix(uint64_t x) {
    x ^= x >> 33;
    x *= 0xff51afd7ed558ccdULL;
    x ^= x >> 33;
    x *= 0xc4ceb9fe1a85ec53ULL;
    x ^= x >> 33;
    return x;
}

template <class T>
void put(std::vector<uint8_t>& out, const T& value) {
    const auto* p = reinterpret_cast<const uint8_t*>(&value);
    out.insert(out.end(), p, p + sizeof(T));
}

template <class T>
bool get(const uint8_t* data, std::size_t size, std::size_t& at, T& value) {
    if (at + sizeof(T) > size) {
        return false;
    }
    std::memcpy(&value, data + at, sizeof(T));
    at += sizeof(T);
    return true;
}

}  // namespace

std::vector<float> depth_to_points(const uint16_t* depth, const Intrinsics& k, int stride, double near, double far) {
    std::vector<float> points;
    const int half = stride / 2;
    const auto low = static_cast<int>(near * 1000), high = static_cast<int>(far * 1000);
    points.reserve(static_cast<std::size_t>((k.height / stride + 1) * (k.width / stride + 1)) * 3);
    for (int v = half; v < k.height; v += stride) {
        const uint16_t* row = depth + static_cast<std::ptrdiff_t>(v) * k.width;
        for (int u = half; u < k.width; u += stride) {
            const int mm = row[u];
            if (mm < low || mm > high) {
                continue;
            }
            const float z = static_cast<float>(mm) * 0.001f;
            points.push_back(static_cast<float>((u - k.cx) / k.fx * z));
            points.push_back(static_cast<float>((v - k.cy) / k.fy * z));
            points.push_back(z);
        }
    }
    return points;
}

VoxelMap::VoxelMap(double voxel, std::size_t max_voxels, int confirm, int max_hits)
    : voxel_(voxel), max_voxels_(max_voxels), confirm_(confirm), max_hits_(max_hits) {
    rehash(1024);
    new_epoch();
}

void VoxelMap::new_epoch() {
    std::random_device device;
    epoch_ = device();
    if (epoch_ == 0) {
        epoch_ = 1;  // tells a page that this is a different map; 0 means "none yet"
    }
}

uint64_t VoxelMap::key(int ix, int iy, int iz) {
    return (static_cast<uint64_t>(ix + kOffset) << 32) | (static_cast<uint64_t>(iy + kOffset) << 16) |
           static_cast<uint64_t>(iz + kOffset);
}

uint32_t VoxelMap::find(uint64_t k) const {
    const std::size_t mask = table_.size() - 1;
    for (std::size_t slot = mix(k) & mask;; slot = (slot + 1) & mask) {
        const uint32_t at = table_[slot];
        if (at == kNone) {
            return kNone;
        }
        const Voxel& v = voxels_[at];
        if (key(v.ix, v.iy, v.iz) == k) {
            return at;
        }
    }
}

void VoxelMap::insert(uint64_t k, uint32_t index) {
    if ((voxels_.size() + 1) * 2 > table_.size()) {
        rehash(table_.size() * 2);  // (the voxel is in voxels_ already: rehash places it)
        return;
    }
    const std::size_t mask = table_.size() - 1;
    std::size_t slot = mix(k) & mask;
    while (table_[slot] != kNone) {
        slot = (slot + 1) & mask;
    }
    table_[slot] = index;
}

void VoxelMap::rehash(std::size_t slots) {
    table_.assign(slots, kNone);
    const std::size_t mask = slots - 1;
    for (uint32_t i = 0; i < voxels_.size(); i++) {
        const Voxel& v = voxels_[i];
        std::size_t slot = mix(key(v.ix, v.iy, v.iz)) & mask;
        while (table_[slot] != kNone) {
            slot = (slot + 1) & mask;
        }
        table_[slot] = i;
    }
}

void VoxelMap::integrate(const float* xyz, const uint8_t* rgb, std::size_t n) {
    rev_++;
    for (std::size_t p = 0; p < n; p++) {
        // Cells are centred on multiples of the voxel size, so that a floor at z = 0 is in
        // the middle of a layer of cells instead of on the border between two.
        const auto ix = static_cast<long>(std::floor(static_cast<double>(xyz[p * 3]) / voxel_ + 0.5));
        const auto iy = static_cast<long>(std::floor(static_cast<double>(xyz[p * 3 + 1]) / voxel_ + 0.5));
        const auto iz = static_cast<long>(std::floor(static_cast<double>(xyz[p * 3 + 2]) / voxel_ + 0.5));
        if (std::labs(ix) >= kLimit || std::labs(iy) >= kLimit || std::labs(iz) >= kLimit) {
            continue;
        }
        const uint8_t r = rgb ? rgb[p * 3] : 0, g = rgb ? rgb[p * 3 + 1] : 0, b = rgb ? rgb[p * 3 + 2] : 0;
        const uint64_t k = key(static_cast<int>(ix), static_cast<int>(iy), static_cast<int>(iz));
        const uint32_t at = find(k);
        if (at == kNone) {
            Voxel v{};
            v.ix = static_cast<int16_t>(ix);
            v.iy = static_cast<int16_t>(iy);
            v.iz = static_cast<int16_t>(iz);
            v.hits = 1;
            v.shown = confirm_ <= 1;
            v.r = r;
            v.g = g;
            v.b = b;
            v.changed = v.shown ? rev_ : 0;
            v.seen = rev_;
            shown_ += v.shown;
            voxels_.push_back(v);
            insert(k, static_cast<uint32_t>(voxels_.size() - 1));
            continue;
        }
        Voxel& v = voxels_[at];
        if (v.seen == rev_) {
            continue;  // several points of one picture in one voxel count once: the first
        }
        v.seen = rev_;
        v.hits = static_cast<uint8_t>(std::min<int>(v.hits + 1, max_hits_));
        const bool appeared = !v.shown && v.hits >= confirm_;
        // Repainting on every small change of brightness would resend the whole view each
        // time; a visible difference is enough.
        const bool repaint = (r | g | b) != 0 && std::abs(r - v.r) + std::abs(g - v.g) + std::abs(b - v.b) > 40;
        if (repaint) {
            v.r = r;
            v.g = g;
            v.b = b;
        }
        if (appeared) {
            v.shown = 1;
            shown_++;
        }
        if (appeared || (repaint && v.shown)) {
            v.changed = rev_;
        }
    }
    if (voxels_.size() > max_voxels_) {
        // Full: forget what has not been looked at for longest.
        const std::size_t keep = voxels_.size() - max_voxels_ / 10;
        std::vector<uint32_t> seen(voxels_.size());
        for (std::size_t i = 0; i < voxels_.size(); i++) {
            seen[i] = voxels_[i].seen;
        }
        const auto nth = seen.begin() + static_cast<std::ptrdiff_t>(voxels_.size() - keep);
        std::nth_element(seen.begin(), nth, seen.end());
        const uint32_t cutoff = *nth;
        std::vector<uint32_t> gone;
        for (uint32_t i = 0; i < voxels_.size(); i++) {
            if (voxels_[i].seen < cutoff) {
                gone.push_back(i);
            }
        }
        drop(gone);
    }
}

int VoxelMap::carve(const Pose& camera, const uint16_t* depth, const Intrinsics& k, double near, double far) {
    if (voxels_.empty()) {
        return 0;
    }
    const auto v = static_cast<float>(voxel_);
    const auto fx = static_cast<float>(k.fx), fy = static_cast<float>(k.fy);
    const auto cx = static_cast<float>(k.cx), cy = static_cast<float>(k.cy);
    const auto tx = static_cast<float>(camera.m[3]), ty = static_cast<float>(camera.m[7]);
    const auto tz = static_cast<float>(camera.m[11]);
    // the camera's axes in the map: the columns of its rotation
    const float r00 = static_cast<float>(camera.m[0]), r10 = static_cast<float>(camera.m[4]);
    const float r20 = static_cast<float>(camera.m[8]), r01 = static_cast<float>(camera.m[1]);
    const float r11 = static_cast<float>(camera.m[5]), r21 = static_cast<float>(camera.m[9]);
    const float r02 = static_cast<float>(camera.m[2]), r12 = static_cast<float>(camera.m[6]);
    const float r22 = static_cast<float>(camera.m[10]);
    const auto lo = static_cast<float>(near), hi = static_cast<float>(far);
    std::vector<uint32_t> gone;
    for (uint32_t i = 0; i < voxels_.size(); i++) {
        Voxel& voxel = voxels_[i];
        const float dx = static_cast<float>(voxel.ix) * v - tx, dy = static_cast<float>(voxel.iy) * v - ty;
        const float dz = static_cast<float>(voxel.iz) * v - tz;
        const float z = dx * r02 + dy * r12 + dz * r22;
        if (!(z > lo && z < hi)) {
            continue;
        }
        const float x = dx * r00 + dy * r10 + dz * r20, y = dx * r01 + dy * r11 + dz * r21;
        const long u = std::lrintf(x / z * fx + cx), w = std::lrintf(y / z * fy + cy);
        if (u < 0 || u >= k.width || w < 0 || w >= k.height) {
            continue;
        }
        const float measured = static_cast<float>(depth[w * k.width + u]) * 0.001f;
        // Only a reading counts (no reading says nothing), and it has to be behind the voxel
        // by more than the camera can be wrong there: its error grows with the square of
        // the distance (about 0.4 cm at 1 m, 3.6 cm at 3 m). Not by much more than that: a
        // voxel a few centimetres in front of a wall is a wrong reading, and the right
        // ones, of the wall behind it, are what takes it out again.
        if (measured > 0 && measured - z > 0.03f + 0.012f * measured * measured) {
            if (--voxel.hits == 0) {
                gone.push_back(i);
            }
        }
    }
    if (!gone.empty()) {
        rev_++;
        drop(gone);
    }
    return static_cast<int>(gone.size());
}

void VoxelMap::drop(const std::vector<uint32_t>& gone) {
    if (gone.empty()) {
        return;
    }
    Removed told{rev_, {}};
    // From the back, so that the voxel moved into a gap is never one that goes too.
    for (auto it = gone.rbegin(); it != gone.rend(); ++it) {
        const Voxel& v = voxels_[*it];
        if (v.shown) {
            told.index.insert(told.index.end(), {v.ix, v.iy, v.iz});
            shown_--;
        }
        voxels_[*it] = voxels_.back();
        voxels_.pop_back();
    }
    rehash(table_.size());
    if (!told.index.empty()) {
        removed_.push_back(std::move(told));
        if (removed_.size() > 400) {
            horizon_ = removed_.front().rev;  // pages that last asked before this are sent everything
            removed_.erase(removed_.begin());
        }
    }
}

void VoxelMap::add_pose(double x, double y, double step, std::size_t limit) {
    if (!trail_.empty() && std::hypot(x - trail_.back().first, y - trail_.back().second) < step) {
        return;
    }
    trail_.emplace_back(x, y);
    if (trail_.size() > limit) {
        trail_.erase(trail_.begin(), trail_.begin() + static_cast<std::ptrdiff_t>(limit / 2));
        trail_start_ += limit / 2;
    }
}

void VoxelMap::clear() {
    voxels_.clear();
    shown_ = 0;
    rehash(1024);
    removed_.clear();
    trail_start_ += trail_.size();
    trail_.clear();
    rev_++;
    horizon_ = rev_;
}

std::vector<uint8_t> VoxelMap::delta(uint32_t since, uint32_t epoch, uint32_t trail) const {
    const bool reset = epoch != epoch_ || since > rev_ || since < horizon_;
    std::vector<int16_t> changed, removed;
    std::vector<uint8_t> rgb;
    for (const Voxel& v : voxels_) {
        if (v.shown && (reset || v.changed > since)) {
            changed.insert(changed.end(), {v.ix, v.iy, v.iz});
            rgb.insert(rgb.end(), {v.r, v.g, v.b});
        }
    }
    if (reset) {
        trail = 0;
    } else {
        for (const Removed& gone : removed_) {
            if (gone.rev > since) {
                removed.insert(removed.end(), gone.index.begin(), gone.index.end());
            }
        }
    }
    const std::size_t first =
        std::min<std::size_t>(std::max<std::size_t>(trail, trail_start_), trail_start_ + trail_.size());
    const std::size_t path = trail_.size() - (first - trail_start_);
    std::vector<uint8_t> out;
    out.reserve(36 + path * 8 + changed.size() * 2 + removed.size() * 2 + rgb.size());
    for (uint32_t word :
         {kMagic, epoch_, rev_, reset ? 1u : 0u, static_cast<uint32_t>(changed.size() / 3),
          static_cast<uint32_t>(removed.size() / 3), static_cast<uint32_t>(first), static_cast<uint32_t>(path)}) {
        put(out, word);
    }
    put(out, static_cast<float>(voxel_));
    for (std::size_t i = first - trail_start_; i < trail_.size(); i++) {
        put(out, static_cast<float>(trail_[i].first));
        put(out, static_cast<float>(trail_[i].second));
    }
    const auto* c = reinterpret_cast<const uint8_t*>(changed.data());
    out.insert(out.end(), c, c + changed.size() * 2);
    const auto* r = reinterpret_cast<const uint8_t*>(removed.data());
    out.insert(out.end(), r, r + removed.size() * 2);
    out.insert(out.end(), rgb.begin(), rgb.end());
    return out;
}

std::vector<std::pair<int, int>> VoxelMap::obstacles(double z_min, double z_max, int least) const {
    std::vector<std::pair<int, int>> places;
    for (const Voxel& v : voxels_) {
        const double z = v.iz * voxel_;
        if (v.shown && v.hits >= least && z > z_min && z < z_max) {
            places.emplace_back(v.ix, v.iy);
        }
    }
    std::sort(places.begin(), places.end());
    places.erase(std::unique(places.begin(), places.end()), places.end());
    return places;
}

void VoxelMap::dump(std::vector<uint8_t>& out) const {
    put(out, voxel_);
    put(out, static_cast<uint64_t>(voxels_.size()));
    for (const Voxel& v : voxels_) {
        put(out, v.ix);
        put(out, v.iy);
        put(out, v.iz);
        put(out, v.hits);
        put(out, v.shown);
        put(out, v.r);
        put(out, v.g);
        put(out, v.b);
    }
    put(out, static_cast<uint64_t>(trail_.size()));
    for (const auto& p : trail_) {
        put(out, p.first);
        put(out, p.second);
    }
}

bool VoxelMap::restore(const uint8_t* data, std::size_t size, std::size_t& used) {
    std::size_t at = used;
    double voxel = 0;
    uint64_t count = 0;
    if (!get(data, size, at, voxel) || std::abs(voxel - voxel_) > 1e-9 || !get(data, size, at, count) ||
        count > 50'000'000 || at + count * 11 > size) {
        return false;  // (also a map with cells of another size)
    }
    std::vector<Voxel> voxels(count);
    std::size_t shown = 0;
    for (Voxel& v : voxels) {
        get(data, size, at, v.ix);
        get(data, size, at, v.iy);
        get(data, size, at, v.iz);
        get(data, size, at, v.hits);
        get(data, size, at, v.shown);
        get(data, size, at, v.r);
        get(data, size, at, v.g);
        get(data, size, at, v.b);
        v.changed = v.seen = 1;
        shown += v.shown != 0;
    }
    uint64_t points = 0;
    if (!get(data, size, at, points) || points > 10'000'000 || at + points * 16 > size) {
        return false;
    }
    std::vector<std::pair<double, double>> trail(points);
    for (auto& p : trail) {
        get(data, size, at, p.first);
        get(data, size, at, p.second);
    }
    voxels_ = std::move(voxels);
    shown_ = shown;
    std::size_t slots = 1024;
    while (slots < voxels_.size() * 2 + 2) {
        slots *= 2;
    }
    rehash(slots);
    new_epoch();  // for the pages it is a new map
    rev_ = 1;
    horizon_ = 0;
    removed_.clear();
    trail_ = std::move(trail);
    trail_start_ = 0;
    used = at;
    return true;
}

}  // namespace obot::nav
