#pragma once

#include <stdexcept>
#include <vector>
#include "nfa.hpp"
#include "token.hpp"

namespace nre::nfa {

class BuildError : public std::runtime_error {
public:
    using BuildError::runtime_error::runtime_error;
};

NFA from_postfix(const std::vector<token::Token>& tokens);

}  // namespace nre::nfa
