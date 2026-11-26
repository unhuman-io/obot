// obot-segd: what is in a picture, said by a network on the GPU.
//
// Started by the operator page's server, like obot-navd, and talked to the same way: a
// request on its input is {u32 size of what follows, u32 kind, ...}, the answer on its
// output {u32 size of what follows, u32 status (0 = done), ...}. It ends when its input
// does.
//
//   setup    text model (.onnx), text engine     ->  i32 width, height (of the pictures it
//            (where the network made ready for       wants), i32 columns, rows (of what it
//            this GPU is kept; made if missing)      says), i32 classes
//   picture  width x height x 3 bytes, BGR       ->  columns x rows bytes: the class of
//                                                    each part of the picture; as many
//                                                    again: how sure, 0..255; f32 ms
//
// A text is {u32 length, bytes}. `obot-segd --build MODEL.onnx [ENGINE]` only makes the
// network ready (minutes on a Jetson Nano) and is what a deployment runs.
//
// This is not built with the rest of rt/: it needs TensorRT and CUDA as NVIDIA installs
// them on a Jetson, which there means the robot's own compiler (C++14). See build.sh.

#include <NvInfer.h>
#include <NvOnnxParser.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <unistd.h>
#include <vector>

// The two calls of the CUDA runtime that are needed, declared here: its own header wants
// further packages of headers that a robot has no other use for.
extern "C" {
int cudaMalloc(void** on_gpu, std::size_t size);
int cudaMemcpy(void* to, const void* from, std::size_t size, int kind);
}

namespace {

const int cudaSuccess = 0, cudaMemcpyHostToDevice = 1, cudaMemcpyDeviceToHost = 2;

enum Kind : uint32_t { kSetup = 1, kPicture = 2 };

class Logger : public nvinfer1::ILogger {
    void log(Severity severity, const char* message) override {
        if (severity <= Severity::kWARNING) {
            std::fprintf(stderr, "obot-segd: %s\n", message);
        }
    }
};
Logger g_logger;

bool read_all(int fd, uint8_t* to, std::size_t n) {
    while (n > 0) {
        const ssize_t got = ::read(fd, to, n);
        if (got == 0 || (got < 0 && errno != EINTR)) {
            return false;
        }
        if (got > 0) {
            to += got;
            n -= static_cast<std::size_t>(got);
        }
    }
    return true;
}

bool write_all(int fd, const uint8_t* from, std::size_t n) {
    while (n > 0) {
        const ssize_t put = ::write(fd, from, n);
        if (put < 0 && errno != EINTR) {
            return false;
        }
        if (put > 0) {
            from += put;
            n -= static_cast<std::size_t>(put);
        }
    }
    return true;
}

// The network made ready for this GPU from its ONNX file, at half precision where the GPU
// is quicker with it. Minutes on a Jetson Nano; that is why the result is kept in a file.
bool build(const std::string& model, const std::string& engine_path) {
    nvinfer1::IBuilder* builder = nvinfer1::createInferBuilder(g_logger);
    if (!builder) {
        return false;
    }
    const auto explicit_batch = 1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    nvinfer1::INetworkDefinition* network = builder->createNetworkV2(explicit_batch);
    nvonnxparser::IParser* parser = nvonnxparser::createParser(*network, g_logger);
    if (!parser->parseFromFile(model.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kWARNING))) {
        std::fprintf(stderr, "obot-segd: cannot read the network in %s\n", model.c_str());
        return false;
    }
    nvinfer1::IBuilderConfig* config = builder->createBuilderConfig();
    config->setMaxWorkspaceSize(512u << 20);
    if (builder->platformHasFastFp16()) {
        config->setFlag(nvinfer1::BuilderFlag::kFP16);
    }
    nvinfer1::ICudaEngine* engine = builder->buildEngineWithConfig(*network, *config);
    if (!engine) {
        return false;
    }
    nvinfer1::IHostMemory* blob = engine->serialize();
    const std::string part = engine_path + ".part";
    std::ofstream out(part, std::ios::binary);
    out.write(static_cast<const char*>(blob->data()), static_cast<std::streamsize>(blob->size()));
    out.close();
    const bool ok = out.good() && std::rename(part.c_str(), engine_path.c_str()) == 0;
    blob->destroy();
    engine->destroy();
    config->destroy();
    parser->destroy();
    network->destroy();
    builder->destroy();
    return ok;
}

struct Network {
    nvinfer1::IRuntime* runtime = nullptr;
    nvinfer1::ICudaEngine* engine = nullptr;
    nvinfer1::IExecutionContext* context = nullptr;
    int width = 0, height = 0;               // of the pictures it wants
    int columns = 0, rows = 0, classes = 0;  // of what it says
    void* on_gpu[2] = {nullptr, nullptr};    // input, output
    int input = 0, output = 1;               // which binding is which
    std::vector<float> in, out;

    bool load(const std::string& engine_path) {
        std::ifstream file(engine_path, std::ios::binary);
        if (!file) {
            return false;
        }
        const std::vector<char> data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        runtime = nvinfer1::createInferRuntime(g_logger);
        engine = runtime ? runtime->deserializeCudaEngine(data.data(), data.size(), nullptr) : nullptr;
        if (!engine || engine->getNbBindings() != 2) {
            return false;
        }
        input = engine->bindingIsInput(0) ? 0 : 1;
        output = 1 - input;
        const nvinfer1::Dims shape_in = engine->getBindingDimensions(input);
        const nvinfer1::Dims shape_out = engine->getBindingDimensions(output);
        if (shape_in.nbDims != 4 || shape_out.nbDims != 4 || shape_in.d[1] != 3) {
            return false;  // (1, 3, height, width) in and (1, classes, rows, columns) out is what this knows
        }
        height = shape_in.d[2];
        width = shape_in.d[3];
        classes = shape_out.d[1];
        rows = shape_out.d[2];
        columns = shape_out.d[3];
        in.resize(static_cast<std::size_t>(3) * height * width);
        out.resize(static_cast<std::size_t>(classes) * rows * columns);
        context = engine->createExecutionContext();
        return context && cudaMalloc(&on_gpu[input], in.size() * sizeof(float)) == cudaSuccess &&
               cudaMalloc(&on_gpu[output], out.size() * sizeof(float)) == cudaSuccess;
    }

    // The picture (BGR bytes, width x height) through the network: per part of it the class
    // with the highest score, and how far ahead of the others it is (its share after
    // softmax, 0..255).
    bool look(const uint8_t* bgr, uint8_t* said, uint8_t* sure) {
        // As the network was trained (torchvision): RGB, 0..1, less the mean, over the deviation.
        static const float mean[3] = {0.485f, 0.456f, 0.406f}, deviation[3] = {0.229f, 0.224f, 0.225f};
        const std::size_t pixels = static_cast<std::size_t>(width) * height;
        for (int c = 0; c < 3; c++) {
            float* plane = in.data() + c * pixels;
            const float scale = 1.0f / (255.0f * deviation[c]), offset = mean[c] / deviation[c];
            for (std::size_t p = 0; p < pixels; p++) {
                plane[p] = bgr[p * 3 + (2 - c)] * scale - offset;
            }
        }
        if (cudaMemcpy(on_gpu[input], in.data(), in.size() * sizeof(float), cudaMemcpyHostToDevice) != cudaSuccess ||
            !context->executeV2(on_gpu) ||
            cudaMemcpy(out.data(), on_gpu[output], out.size() * sizeof(float), cudaMemcpyDeviceToHost) != cudaSuccess) {
            return false;
        }
        const std::size_t parts = static_cast<std::size_t>(rows) * columns;
        for (std::size_t p = 0; p < parts; p++) {
            int best = 0;
            float top = out[p];
            for (int c = 1; c < classes; c++) {
                if (out[c * parts + p] > top) {
                    top = out[c * parts + p];
                    best = c;
                }
            }
            float sum = 0;
            for (int c = 0; c < classes; c++) {
                sum += std::exp(out[c * parts + p] - top);
            }
            said[p] = static_cast<uint8_t>(best);
            sure[p] = static_cast<uint8_t>(std::lround(255.0f / sum));
        }
        return true;
    }
};

struct Reader {
    const uint8_t* data;
    std::size_t size, at = 0;
    bool ok = true;
    std::string text() {
        uint32_t n = 0;
        if (at + 4 > size) {
            ok = false;
            return "";
        }
        std::memcpy(&n, data + at, 4);
        at += 4;
        if (at + n > size) {
            ok = false;
            return "";
        }
        std::string s(reinterpret_cast<const char*>(data + at), n);
        at += n;
        return s;
    }
};

template <typename T>
void put(std::vector<uint8_t>& to, T value) {
    const auto* raw = reinterpret_cast<const uint8_t*>(&value);
    to.insert(to.end(), raw, raw + sizeof(T));
}

bool exists(const std::string& path) {
    return ::access(path.c_str(), R_OK) == 0;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc >= 3 && std::string(argv[1]) == "--build") {
        const std::string model = argv[2], engine = argc > 3 ? argv[3] : model + ".engine";
        std::fprintf(stderr, "obot-segd: making %s ready for this GPU (minutes)\n", model.c_str());
        return build(model, engine) ? 0 : 1;
    }
    if (argc > 1) {
        std::fprintf(stderr,
                     "obot-segd: started by the operator page's server (obot ui); or --build MODEL.onnx [ENGINE]\n");
        return 2;
    }
    Network network;
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
        answer.clear();
        bool ok = true;
        if (head[1] == kSetup) {
            Reader in{request.data(), request.size()};
            const std::string model = in.text();
            std::string engine = in.text();
            if (engine.empty()) {
                engine = model + ".engine";
            }
            ok = in.ok && !network.context && (exists(engine) || build(model, engine)) && network.load(engine);
            for (const int v : {network.width, network.height, network.columns, network.rows, network.classes}) {
                put<int32_t>(answer, v);
            }
        } else if (head[1] == kPicture) {
            const std::size_t parts = static_cast<std::size_t>(network.rows) * network.columns;
            ok = network.context && request.size() == static_cast<std::size_t>(network.width) * network.height * 3;
            if (ok) {
                answer.resize(2 * parts);
                const auto began = std::chrono::steady_clock::now();
                ok = network.look(request.data(), answer.data(), answer.data() + parts);
                put<float>(answer,
                           std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - began).count());
            }
        } else {
            ok = false;
        }
        if (!ok) {
            answer.clear();
        }
        const uint32_t out[2] = {static_cast<uint32_t>(answer.size() + 4), ok ? 0u : 1u};
        if (!write_all(1, reinterpret_cast<const uint8_t*>(out), 8) || !write_all(1, answer.data(), answer.size())) {
            return 0;
        }
    }
}
