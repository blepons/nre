#pragma once

#include <generator>
#include "token.hpp"

namespace nre::token {

std::generator<Token> tokenize(std::string_view regex);

}  // namespace nre::token
