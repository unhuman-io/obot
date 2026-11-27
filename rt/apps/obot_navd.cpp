// obot-navd: the map of the robot's surroundings, kept by a process of its own.
//
// The depth cameras' pictures are turned into a 3D map (obot::nav::VoxelMap), a record of
// where the floor has been seen to be free (FloorSeen) and how far ahead the nearest thing
// in the base's way is. That used to be done inside the operator page's server, in Python,
// and took most of a Jetson Nano's processor; here it takes a few milliseconds a picture.
//
// For now it is a helper of the page server, which starts it, hands it every depth picture
// with the pose it was taken from, and asks it for what the pages and the navigation need.
// It speaks over its standard input and output:
//
//   request:  u32 length of what follows, u32 what, ... (little endian, packed)
//   answer:   u32 length of what follows, u32 0 or an error number, ...
//
// See python/src/obot/ui/native.py for the other end and the layout of each request.
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <unistd.h>
#include <vector>

#include "obot/nav/floor.h"
#include "obot/nav/voxel_map.h"

using namespace obot::nav;

namespace {

enum What : uint32_t {
    kSetup = 1,    // f64 voxel, u32 max voxels, f64 floor cell, i32 floor half, f64 way half width, low, high
    kPicture = 2,  // see Picture below
    kDelta = 3,    // u32 since, epoch, trail -> the page's update
    kPose = 4,     // f64 x, y, stand radius: the base is here
    kClear = 5,
    kGrid = 6,       // f64 keep clear -> the map from above
    kKnown = 7,      // i32 i0, j0, u32 width, height -> a byte per square
    kObstacles = 8,  // f64 z min, z max, i32 least hits -> u32 count, then i32 ix, iy each
    kSave = 9,       // u32 path length, path, u32 frame length, frame
    kLoad = 10,      // the same -> u8 whether it was taken up
    kCount = 11,
    kThings = 12,  // -> u32 count, then i32 i, j each: the squares something is remembered to stand in     // -> u64
                   // shown voxels, all voxels, floor squares seen; u32 epoch, rev
};

struct Reader {
    const uint8_t* data;
    std::size_t size, at = 0;
    bool ok = true;
    template <class T>
    T take() {
        T value{};
        if (at + sizeof(T) > size) {
            ok = false;
            return value;
        }
        std::memcpy(&value, data + at, sizeof(T));
        at += sizeof(T);
        return value;
    }
    const uint8_t* bytes(std::size_t n) {
        if (at + n > size) {
            ok = false;
            return nullptr;
        }
        const uint8_t* p = data + at;
        at += n;
        return p;
    }
    std::string text() {
        const auto n = take<uint32_t>();
        const uint8_t* p = bytes(n);
        return p ? std::string(reinterpret_cast<const char*>(p), n) : std::string();
    }
    Pose pose() {
        Pose p;
        for (double& v : p.m) {
            v = take<double>();
        }
        return p;
    }
    Intrinsics intrinsics() {
        Intrinsics k;
        k.width = take<int32_t>();
        k.height = take<int32_t>();
        k.fx = take<double>();
        k.fy = take<double>();
        k.cx = take<double>();
        k.cy = take<double>();
        return k;
    }
};

template <class T>
void put(std::vector<uint8_t>& out, const T& value) {
    const auto* p = reinterpret_cast<const uint8_t*>(&value);
    out.insert(out.end(), p, p + sizeof(T));
}

bool read_all(int fd, uint8_t* into, std::size_t n) {
    while (n) {
        const ssize_t got = ::read(fd, into, n);
        if (got == 0 || (got < 0 && errno != EINTR)) {
            return false;
        }
        if (got > 0) {
            into += got;
            n -= static_cast<std::size_t>(got);
        }
    }
    return true;
}

bool write_all(int fd, const uint8_t* from, std::size_t n) {
    while (n) {
        const ssize_t put_ = ::write(fd, from, n);
        if (put_ < 0 && errno != EINTR) {
            return false;
        }
        if (put_ > 0) {
            from += put_;
            n -= static_cast<std::size_t>(put_);
        }
    }
    return true;
}

constexpr uint32_t kFileMagic = 0x324D424Fu;  // "OBM2"

struct Maps {
    VoxelMap map;
    FloorSeen floor;
    Way way;
};

// A picture from a depth camera, and all that is done with it.
//   u32 flags: 1 the camera is on the base (look ahead, floor), 2 use it for the map
//              (not while the base turns fast), 4 a colour picture follows
//   intrinsics (i32 width, height, f64 fx, fy, cx, cy), i32 stride, f64 near, far
//   pose base <- optical (12 f64), pose map <- optical (12 f64), f64 x, y, yaw of the base
//   width * height u16 depth, millimetres
//   with a colour picture: its intrinsics, pose colour optical <- depth optical, width * height * 3 u8 (b, g, r)
// -> f64 ahead (metres; -1: not looked), u32 points, u32 voxels removed, u64 voxels shown now,
//    u32 squares with something standing in this picture, of them remembered, of them contradicted
bool picture(Maps& m, Reader& in, std::vector<uint8_t>& out) {
    const auto flags = in.take<uint32_t>();
    const Intrinsics k = in.intrinsics();
    const auto stride = in.take<int32_t>();
    const auto near = in.take<double>(), far = in.take<double>();
    const Pose to_base = in.pose(), to_map = in.pose();
    const auto x = in.take<double>(), y = in.take<double>(), yaw = in.take<double>();
    if (!in.ok || k.width <= 0 || k.height <= 0 || k.width > 4096 || k.height > 4096 || stride <= 0) {
        return false;
    }
    const auto* depth = reinterpret_cast<const uint16_t*>(in.bytes(static_cast<std::size_t>(k.width) * k.height * 2));
    if (!depth) {
        return false;
    }
    // (the request is a byte buffer: the depth may not be aligned for 16 bit reads)
    std::vector<uint16_t> aligned(static_cast<std::size_t>(k.width) * k.height);
    std::memcpy(aligned.data(), depth, aligned.size() * 2);
    const std::vector<float> points = depth_to_points(aligned.data(), k, stride, near, far);
    const std::size_t n = points.size() / 3;
    double ahead = -1.0;
    uint32_t removed = 0;
    Agreement agreement;
    std::vector<float> moved(points.size());
    if (flags & 1u) {
        for (std::size_t i = 0; i < n; i++) {
            to_base.apply(points[i * 3], points[i * 3 + 1], points[i * 3 + 2], moved[i * 3], moved[i * 3 + 1],
                          moved[i * 3 + 2]);
        }
        ahead = look_ahead(moved, aligned.data(), k.width, k.height, m.way);
    }
    if (flags & 2u) {
        if (flags & 1u) {
            agreement = m.floor.see(moved, to_base.m[3], to_base.m[7], x, y, yaw, stride / k.fx, m.way);
        }
        removed = static_cast<uint32_t>(m.map.carve(to_map, aligned.data(), k, near, far));
        std::vector<uint8_t> rgb;
        if (flags & 4u) {
            const Intrinsics colour = in.intrinsics();
            const Pose to_colour = in.pose();
            if (!in.ok || colour.width <= 0 || colour.height <= 0 || colour.width > 4096 || colour.height > 4096) {
                return false;
            }
            const uint8_t* bgr = in.bytes(static_cast<std::size_t>(colour.width) * colour.height * 3);
            if (!bgr) {
                return false;
            }
            std::vector<float> in_colour(points.size());
            for (std::size_t i = 0; i < n; i++) {
                to_colour.apply(points[i * 3], points[i * 3 + 1], points[i * 3 + 2], in_colour[i * 3],
                                in_colour[i * 3 + 1], in_colour[i * 3 + 2]);
            }
            rgb = sample_colors(in_colour, bgr, colour);
        }
        for (std::size_t i = 0; i < n; i++) {
            to_map.apply(points[i * 3], points[i * 3 + 1], points[i * 3 + 2], moved[i * 3], moved[i * 3 + 1],
                         moved[i * 3 + 2]);
        }
        m.map.integrate(moved.data(), rgb.empty() ? nullptr : rgb.data(), n);
    }
    put(out, ahead);
    put(out, static_cast<uint32_t>(n));
    put(out, removed);
    put(out, static_cast<uint64_t>(m.map.shown()));
    put(out, agreement.things);
    put(out, agreement.remembered);
    put(out, agreement.contradicted);
    return true;
}

bool save(const Maps& m, const std::string& path, const std::string& frame) {
    std::vector<uint8_t> out;
    put(out, kFileMagic);
    put(out, static_cast<uint32_t>(frame.size()));
    out.insert(out.end(), frame.begin(), frame.end());
    m.map.dump(out);
    m.floor.dump(out);
    const std::string part = path + ".part";
    std::FILE* f = std::fopen(part.c_str(), "wb");
    if (!f) {
        return false;
    }
    bool ok = std::fwrite(out.data(), 1, out.size(), f) == out.size();
    ok = std::fflush(f) == 0 && ok;
    ok = ::fsync(::fileno(f)) == 0 && ok;
    ok = std::fclose(f) == 0 && ok;
    return ok && std::rename(part.c_str(), path.c_str()) == 0;
}

bool load(Maps& m, const std::string& path, const std::string& frame) {
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) {
        return false;
    }
    std::vector<uint8_t> data;
    uint8_t chunk[65536];
    for (std::size_t got; (got = std::fread(chunk, 1, sizeof(chunk), f)) > 0;) {
        data.insert(data.end(), chunk, chunk + got);
    }
    std::fclose(f);
    Reader in{data.data(), data.size()};
    if (in.take<uint32_t>() != kFileMagic) {
        return false;
    }
    const std::string saved_in = in.text();
    if (!in.ok || (frame != "*" && saved_in != frame)) {
        return false;  // a map that is in the odometry of another run of obot-rtd ("*": whichever)
    }
    VoxelMap map(m.map.voxel());
    FloorSeen floor(m.floor.cell(), m.floor.half());
    std::size_t at = in.at;
    if (!map.restore(data.data(), data.size(), at) || !floor.restore(data.data(), data.size(), at)) {
        return false;
    }
    m.map = std::move(map);
    m.floor = std::move(floor);
    return true;
}

}  // namespace

int main(int argc, char**) {
    if (argc > 1) {
        std::fprintf(stderr, "obot-navd: started by the operator page's server (obot ui); it takes no arguments\n");
        return 2;
    }
    // It ends when the page server closes its input, and not before: a service being stopped
    // (and Ctrl-C in a terminal) signals every process of it at once, and the page server
    // still has the map to save, which is here.
    std::signal(SIGTERM, SIG_IGN);
    std::signal(SIGINT, SIG_IGN);
    Maps m;
    std::vector<uint8_t> request, answer;
    for (;;) {
        uint32_t head[2];
        if (!read_all(0, reinterpret_cast<uint8_t*>(head), 8)) {
            return 0;  // the page server has gone
        }
        if (head[0] < 4 || head[0] > (64u << 20)) {
            return 1;
        }
        request.resize(head[0] - 4);
        if (!read_all(0, request.data(), request.size())) {
            return 0;
        }
        Reader in{request.data(), request.size()};
        answer.clear();
        bool ok = true;
        switch (head[1]) {
            case kSetup: {
                const auto voxel = in.take<double>();
                const auto max_voxels = in.take<uint32_t>();
                const auto cell = in.take<double>();
                const auto half = in.take<int32_t>();
                Way way;
                way.half_width = in.take<double>();
                way.z_low = in.take<double>();
                way.z_high = in.take<double>();
                ok = in.ok && voxel > 1e-4 && cell > 1e-4 && half > 0 && half <= 4000;
                if (ok) {
                    m.map = VoxelMap(voxel, max_voxels);
                    m.floor = FloorSeen(cell, half);
                    m.way = way;
                }
                break;
            }
            case kPicture:
                ok = picture(m, in, answer);
                break;
            case kDelta: {
                const auto since = in.take<uint32_t>(), epoch = in.take<uint32_t>(), trail = in.take<uint32_t>();
                answer = m.map.delta(since, epoch, trail);
                break;
            }
            case kPose: {
                const auto x = in.take<double>(), y = in.take<double>(), radius = in.take<double>();
                m.map.add_pose(x, y);
                m.floor.stand(x, y, radius);
                break;
            }
            case kClear:
                m.map.clear();
                m.floor.clear();
                break;
            case kGrid:
                answer = m.floor.grid(m.map, m.way, in.take<double>());
                break;
            case kKnown: {
                const auto i0 = in.take<int32_t>(), j0 = in.take<int32_t>();
                const auto width = in.take<uint32_t>(), height = in.take<uint32_t>();
                ok = in.ok && width <= 8192 && height <= 8192;
                if (ok) {
                    answer = m.floor.known(i0, j0, static_cast<int>(width), static_cast<int>(height));
                }
                break;
            }
            case kObstacles: {
                const auto z_min = in.take<double>(), z_max = in.take<double>();
                const auto least = in.take<int32_t>();
                const auto places = m.map.obstacles(z_min, z_max, least);
                put(answer, static_cast<uint32_t>(places.size()));
                for (const auto& [ix, iy] : places) {
                    put(answer, static_cast<int32_t>(ix));
                    put(answer, static_cast<int32_t>(iy));
                }
                break;
            }
            case kSave: {
                const std::string path = in.text(), frame = in.text();
                ok = in.ok && save(m, path, frame);
                break;
            }
            case kLoad: {
                const std::string path = in.text(), frame = in.text();
                put(answer, static_cast<uint8_t>(in.ok && load(m, path, frame)));
                break;
            }
            case kThings: {
                const auto squares = m.floor.things();
                put(answer, static_cast<uint32_t>(squares.size()));
                for (const auto& [i, j] : squares) {
                    put(answer, static_cast<int32_t>(i));
                    put(answer, static_cast<int32_t>(j));
                }
                break;
            }
            case kCount:
                put(answer, static_cast<uint64_t>(m.map.shown()));
                put(answer, static_cast<uint64_t>(m.map.size()));
                put(answer, static_cast<uint64_t>(m.floor.count()));
                put(answer, m.map.epoch());
                put(answer, m.map.rev());
                break;
            default:
                ok = false;
        }
        ok = ok && in.ok;
        const uint32_t reply[2] = {static_cast<uint32_t>((ok ? answer.size() : 0) + 4), ok ? 0u : 1u};
        if (!write_all(1, reinterpret_cast<const uint8_t*>(reply), 8) ||
            (ok && !write_all(1, answer.data(), answer.size()))) {
            return 0;
        }
    }
}
