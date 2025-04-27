#include "dfa.hpp"
#include <map>
#include <queue>

namespace nre::dfa {

StateID DFA::create_state() {
    states.emplace_back();
    return states.size() - 1;
}

void DFA::add_transition(StateID from, char c, StateID to) {
    states.at(from)[c] = to;
}

void DFA::set_accepting(StateID state, bool accepting) {
    if (accepting) {
        accept_states.insert(state);
    } else {
        accept_states.erase(state);
    }
}

bool DFA::is_accepting(StateID state) const {
    return accept_states.contains(state);
}

DFA intersect(const DFA& a, const DFA& b) {
    DFA product;

    std::unordered_set<char> combined_alphabet;
    for (const auto& state : a.states) {
        for (const auto& [c, _] : state) {
            combined_alphabet.insert(c);
        }
    }
    for (const auto& state : b.states) {
        for (const auto& [c, _] : state) {
            combined_alphabet.insert(c);
        }
    }

    const auto dead_state = product.create_state();
    for (char c : combined_alphabet) {
        product.add_transition(dead_state, c, dead_state);
    }

    std::map<std::pair<StateID, StateID>, StateID> state_map;
    std::queue<std::pair<StateID, StateID>> process_queue;

    const auto initial_pair = std::make_pair(a.start_state, b.start_state);
    state_map[initial_pair] = product.create_state();
    product.start_state = state_map[initial_pair];
    process_queue.push(initial_pair);

    while (!process_queue.empty()) {
        auto [a_s, b_s] = process_queue.front();
        process_queue.pop();
        const StateID current_id = state_map[{a_s, b_s}];

        product.set_accepting(current_id,
                              a.is_accepting(a_s) && b.is_accepting(b_s));

        for (char c : combined_alphabet) {
            const auto& a_trans = a.states[a_s];
            const auto& b_trans = b.states[b_s];

            StateID next_id;
            if (!a_trans.contains(c) || !b_trans.contains(c)) {
                next_id = dead_state;
            } else {
                const auto next_pair =
                    std::make_pair(a_trans.at(c), b_trans.at(c));
                if (!state_map.count(next_pair)) {
                    state_map[next_pair] = product.create_state();
                    process_queue.push(next_pair);
                }
                next_id = state_map[next_pair];
            }
            product.add_transition(current_id, c, next_id);
        }
    }

    return product;
}

}  // namespace nre::dfa
