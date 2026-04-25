// Anchor TU so MSVC/clang link static libs even if every other TU is purely
// inline. Also gives a single place to assert layout invariants.

#include "removert/types.hpp"

#include <type_traits>

namespace removert {

static_assert(std::is_trivially_copyable_v<Point>,
              "Point must stay trivially copyable for GPU upload");
static_assert(sizeof(Point) == 4 * sizeof(float),
              "Point must pack as 4 floats with no padding");
static_assert(std::is_trivially_copyable_v<SphericalPoint>,
              "SphericalPoint must stay trivially copyable");

}  // namespace removert
