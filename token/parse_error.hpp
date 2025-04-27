#pragma once

#include <stdexcept>

namespace nre {

class ParseError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

}  // namespace nre
