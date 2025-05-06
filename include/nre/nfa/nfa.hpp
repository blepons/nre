#pragma once

#include <cstddef>
#include <queue>
#include <ranges>
#include <variant>
#include <vector>

namespace nre::nfa {

using StateID = std::size_t;

struct EpsilonTransition {};
struct GroupStart {
    int group_id;
};
struct GroupEnd {
    int group_id;
};
struct Lookahead {
    StateID start;
    StateID end;
};
using TransitionCondition =
    std::variant<EpsilonTransition, char, GroupStart, GroupEnd, Lookahead>;

struct Transition {
    TransitionCondition condition;
    StateID target;
};

class NFA {
public:
    NFA() = default;

    StateID create_state();
    void add_transition(StateID from, TransitionCondition cond, StateID to);

    std::vector<std::vector<Transition>> states;
    StateID start_state = 0;
    StateID accept_state = 0;
};

template <template <class...> class Set,
          bool IgnoreLookaheadAssertion = false,
          std::ranges::input_range R>
Set<StateID> epsilon_closure(const NFA& nfa, R&& states) {
    Set<StateID> closure(std::ranges::begin(states), std::ranges::end(states));
    std::queue<StateID> processing_queue(closure.begin(), closure.end());

    while (!processing_queue.empty()) {
        auto current = processing_queue.front();
        processing_queue.pop();

        for (const auto& trans : nfa.states[current]) {
            if (std::holds_alternative<EpsilonTransition>(trans.condition) ||
                std::holds_alternative<GroupStart>(trans.condition) ||
                std::holds_alternative<GroupEnd>(trans.condition) ||
                (IgnoreLookaheadAssertion &&
                 std::holds_alternative<Lookahead>(trans.condition))) {
                if (!closure.contains(trans.target)) {
                    closure.insert(trans.target);
                    processing_queue.push(trans.target);
                }
            }
        }
    }

    return closure;
}

}  // namespace nre::nfa
