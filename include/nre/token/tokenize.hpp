#pragma once

#include <generator>
#include "token.hpp"

namespace nre {

std::generator<Token> tokenize(std::string_view regex);

}  // namespace nre
