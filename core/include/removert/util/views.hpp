// Tiny zip / enumerate helpers. C++17 has no std::ranges, so we keep the
// surface area minimal and only what the algorithms actually use.

#pragma once

#include <cstddef>
#include <iterator>
#include <tuple>

namespace removert::util {

template <typename Container>
struct EnumerateProxy {
    Container& c;

    struct Iter {
        using It = decltype(std::begin(std::declval<Container&>()));
        std::size_t i;
        It it;
        bool operator!=(Iter const& other) const noexcept { return it != other.it; }
        void operator++() { ++i; ++it; }
        auto operator*() const { return std::tuple<std::size_t, decltype(*it)>{i, *it}; }
    };

    Iter begin() { return Iter{0, std::begin(c)}; }
    Iter end()   { return Iter{0, std::end(c)}; }
};

template <typename Container>
[[nodiscard]] EnumerateProxy<Container> enumerate(Container& c) noexcept { return {c}; }

}  // namespace removert::util
