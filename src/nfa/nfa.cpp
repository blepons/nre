#include "nfa.hpp"

namespace nre::nfa {

StateID NFA::create_state() {
    states.emplace_back();
    return states.size() - 1;
}

void NFA::add_transition(StateID from, TransitionCondition cond, StateID to) {
    states.at(from).emplace_back(Transition{std::move(cond), to});
}

}  // namespace nre::nfa
