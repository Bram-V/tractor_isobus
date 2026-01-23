#pragma once

#include <string>

namespace tractor {

    class Tractor {
      public:
        Tractor() = default;
        ~Tractor() = default;

        std::string version() const;
    };

} // namespace tractor
