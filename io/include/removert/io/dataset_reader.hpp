// Abstract dataset reader. Implementations live alongside (sqlite, KITTI dir,
// HDF5). One interface, three backends.

#pragma once

#include "removert/types.hpp"

#include <cstddef>
#include <vector>

namespace removert::io {

class IDatasetReader {
public:
    [[nodiscard]] virtual std::size_t  size() const                    = 0;
    [[nodiscard]] virtual SensorConfig sensor() const                  = 0;
    [[nodiscard]] virtual ScanFrame    read(std::size_t idx) const     = 0;

    // Default batch reader — backends can override for chunked I/O.
    [[nodiscard]] virtual std::vector<ScanFrame>
    read_batch(std::size_t begin, std::size_t end) const {
        std::vector<ScanFrame> out;
        out.reserve(end - begin);
        for (std::size_t i = begin; i < end; ++i) out.push_back(read(i));
        return out;
    }

    virtual ~IDatasetReader() = default;
};

}  // namespace removert::io
