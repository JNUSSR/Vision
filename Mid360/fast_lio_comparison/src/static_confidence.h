#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <unordered_set>

class StaticConfidence
{
public:
    struct Key
    {
        int x, y, z;
        bool operator==(const Key &other) const
        {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    struct Hash
    {
        std::size_t operator()(const Key &key) const
        {
            std::size_t h = std::hash<int>{}(key.x);
            h ^= std::hash<int>{}(key.y) + 0x9e3779b9 + (h << 6) + (h >> 2);
            h ^= std::hash<int>{}(key.z) + 0x9e3779b9 + (h << 6) + (h >> 2);
            return h;
        }
    };

    StaticConfidence(double voxel_size, double min_fraction, int min_frames)
        : voxel_size_(voxel_size), min_fraction_(min_fraction), min_frames_(min_frames) {}

    void begin_frame() { frame_.clear(); }
    void observe(double x, double y, double z)
    {
        if (valid(x, y, z)) frame_.insert(key(x, y, z));
    }
    void end_frame()
    {
        ++frames_;
        for (const auto &cell : frame_) ++hits_[cell];
    }
    bool ready() const { return frames_ >= min_frames_; }
    int frames() const { return frames_; }
    std::size_t cells() const { return hits_.size(); }
    std::size_t trusted_cells() const
    {
        return std::count_if(hits_.begin(), hits_.end(), [this](const auto &cell) {
            return static_cast<double>(cell.second) / frames_ >= min_fraction_;
        });
    }
    bool trusted(double x, double y, double z) const
    {
        if (!ready() || !valid(x, y, z)) return false;
        const auto it = hits_.find(key(x, y, z));
        return it != hits_.end() &&
            static_cast<double>(it->second) / frames_ >= min_fraction_;
    }

private:
    bool valid(double x, double y, double z) const
    {
        const double limit = static_cast<double>(std::numeric_limits<int>::max()) - 1.0;
        return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) &&
            std::abs(x / voxel_size_) < limit && std::abs(y / voxel_size_) < limit &&
            std::abs(z / voxel_size_) < limit;
    }
    Key key(double x, double y, double z) const
    {
        return {static_cast<int>(std::floor(x / voxel_size_)),
                static_cast<int>(std::floor(y / voxel_size_)),
                static_cast<int>(std::floor(z / voxel_size_))};
    }
    double voxel_size_, min_fraction_;
    int min_frames_, frames_ = 0;
    std::unordered_map<Key, int, Hash> hits_;
    std::unordered_set<Key, Hash> frame_;
};
