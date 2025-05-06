#include "from_nfa.hpp"
#include <map>
#include <queue>
#include <ranges>
#include <set>
#include <unordered_set>

namespace nre::dfa {

namespace {

std::unordered_set<char> collect_alphabet(const nfa::NFA& nfa) {
    std::unordered_set<char> chars;
    for (const auto& state_transitions : nfa.states) {
        for (const auto& trans : state_transitions) {
            if (const auto* c = std::get_if<char>(&trans.condition)) {
                chars.insert(*c);
            }
        }
    }
    return chars;
}

}  // namespace

DFA from_nfa(const nfa::NFA& nfa) {
    DFA dfa;
    std::map<std::set<nfa::StateID>, StateID> state_map;
    std::vector<std::set<nfa::StateID>> state_sets;

    auto initial_states = nfa::epsilon_closure<std::set, true>(
        nfa, std::views::single(nfa.start_state));

    const auto initial_id = dfa.create_state();
    state_map[initial_states] = initial_id;
    state_sets.push_back(initial_states);
    dfa.start_state = initial_id;

    std::queue<StateID> processing;
    processing.push(initial_id);

    const auto alphabet = collect_alphabet(nfa);

    while (!processing.empty()) {
        const auto dfa_state = processing.front();
        processing.pop();

        auto nfa_states = state_sets[dfa_state];

        dfa.set_accepting(dfa_state, nfa_states.contains(nfa.accept_state));

        for (char c : alphabet) {
            std::set<nfa::StateID> move_result;
            for (auto nfa_state : nfa_states) {
                for (const auto& trans : nfa.states[nfa_state]) {
                    if (const auto* ch = std::get_if<char>(&trans.condition)) {
                        if (*ch == c) {
                            move_result.insert(trans.target);
                        }
                    }
                }
            }

            auto closure =
                nfa::epsilon_closure<std::set, true>(nfa, move_result);
            if (closure.empty()) {
                continue;
            }

            if (!state_map.contains(closure)) {
                const auto new_id = dfa.create_state();
                state_map[closure] = new_id;
                state_sets.push_back(closure);
                processing.push(new_id);
            }

            dfa.add_transition(dfa_state, c, state_map[closure]);
        }
    }

    return dfa;
}

}  // namespace nre::dfa
