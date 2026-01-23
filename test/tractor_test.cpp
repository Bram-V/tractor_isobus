#include <doctest/doctest.h>
#include <tractor/tractor.hpp>

TEST_CASE("Tractor version") {
    tractor::Tractor trac;
    CHECK(trac.version() == "0.0.1");
}
