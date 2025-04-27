#pragma once

#include <cstddef>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace nre::dfa {

using StateID = std::size_t;

class DFA {
public:
    DFA() = default;

    StateID create_state();
    void add_transition(StateID from, char c, StateID to);
    void set_accepting(StateID state, bool accepting);
    bool is_accepting(StateID state) const;

    std::vector<std::unordered_map<char, StateID>> states;
    std::unordered_set<StateID> accept_states;
    StateID start_state = 0;
};

DFA intersect(const DFA& a, const DFA& b);

// TODO: implement these functions:
//
// DFA minimize(const DFA& automaton);
// DFA invert(const DFA& automaton);
// std::string to_regex(const DFA& automaton);

}  // namespace nre::dfa
