#pragma once

#include <vector>
#include "nfa/nfa.hpp"
#include "token/token.hpp"

namespace nre::nfa {

NFA from_postfix(const std::vector<Token>& tokens);

}
