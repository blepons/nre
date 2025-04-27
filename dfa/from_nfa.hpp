#pragma once

#include "dfa.hpp"
#include "nfa/nfa.hpp"

namespace nre::dfa {

DFA from_nfa(const nfa::NFA& nfa);

}  // namespace nre::dfa
