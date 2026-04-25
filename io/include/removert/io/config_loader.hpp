// Single point of Config materialization. Reads a JSON file (or string) and
// produces a fully populated PipelineConfig. Missing keys fall back to the
// in-source defaults declared in core/types.hpp.

#pragma once

#include "removert/types.hpp"

#include <filesystem>
#include <string>

namespace removert::io {

[[nodiscard]] PipelineConfig load_config_from_json_file(
    std::filesystem::path const& path);

[[nodiscard]] PipelineConfig load_config_from_json_string(std::string const& s);

[[nodiscard]] std::string    dump_config_to_json(PipelineConfig const& cfg);

}  // namespace removert::io
