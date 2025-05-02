#include "from_postfix.hpp"
#include <stack>
#include <unordered_map>
#include "nfa.hpp"

namespace nre::nfa {

namespace {

struct Fragment {
    std::size_t start;
    std::size_t end;
};

template <typename T>
constexpr bool always_false = false;

void handle_literal(NFA& nfa,
                    std::stack<Fragment>& frag_stack,
                    const token::Literal& tok) {
    const auto start = nfa.create_state();
    const auto end = nfa.create_state();
    nfa.add_transition(start, tok.value, end);
    frag_stack.push(Fragment{start, end});
}

void handle_empty_string(NFA& nfa, std::stack<Fragment>& frag_stack) {
    const auto start = nfa.create_state();
    const auto end = nfa.create_state();
    nfa.add_transition(start, EpsilonTransition{}, end);
    frag_stack.push(Fragment{start, end});
}

void handle_character_class(NFA& nfa,
                            std::stack<Fragment>& frag_stack,
                            const token::CharacterClass& tok) {
    const auto start = nfa.create_state();
    const auto end = nfa.create_state();
    for (char ch : tok.chars) {
        nfa.add_transition(start, ch, end);
    }
    frag_stack.push(Fragment{start, end});
}

void handle_concatenation(NFA& nfa, std::stack<Fragment>& frag_stack) {
    if (frag_stack.size() < 2) {
        throw BuildError("Insufficient operands for concatenation");
    }
    Fragment rhs = frag_stack.top();
    frag_stack.pop();
    Fragment lhs = frag_stack.top();
    frag_stack.pop();

    nfa.add_transition(lhs.end, EpsilonTransition{}, rhs.start);
    frag_stack.push(Fragment{lhs.start, rhs.end});
}

void handle_alternation(NFA& nfa, std::stack<Fragment>& frag_stack) {
    if (frag_stack.size() < 2) {
        throw BuildError("Insufficient operands for alternation");
    }
    Fragment rhs = frag_stack.top();
    frag_stack.pop();
    Fragment lhs = frag_stack.top();
    frag_stack.pop();

    const auto new_start = nfa.create_state();
    const auto new_end = nfa.create_state();

    nfa.add_transition(new_start, EpsilonTransition{}, lhs.start);
    nfa.add_transition(new_start, EpsilonTransition{}, rhs.start);
    nfa.add_transition(lhs.end, EpsilonTransition{}, new_end);
    nfa.add_transition(rhs.end, EpsilonTransition{}, new_end);

    frag_stack.push(Fragment{new_start, new_end});
}

void handle_kleene_star(NFA& nfa, std::stack<Fragment>& frag_stack) {
    if (frag_stack.empty()) {
        throw BuildError("Missing operand for Kleene star");
    }
    Fragment inner = frag_stack.top();
    frag_stack.pop();

    const auto new_start = nfa.create_state();
    const auto new_end = nfa.create_state();

    nfa.add_transition(new_start, EpsilonTransition{}, inner.start);
    nfa.add_transition(new_start, EpsilonTransition{}, new_end);
    nfa.add_transition(inner.end, EpsilonTransition{}, inner.start);
    nfa.add_transition(inner.end, EpsilonTransition{}, new_end);

    frag_stack.push(Fragment{new_start, new_end});
}

void handle_positive_closure(NFA& nfa, std::stack<Fragment>& frag_stack) {
    if (frag_stack.empty()) {
        throw BuildError("Missing operand for positive closure");
    }
    Fragment inner = frag_stack.top();
    frag_stack.pop();

    const auto new_start = nfa.create_state();
    const auto new_end = nfa.create_state();

    nfa.add_transition(new_start, EpsilonTransition{}, inner.start);
    nfa.add_transition(inner.end, EpsilonTransition{}, inner.start);
    nfa.add_transition(inner.end, EpsilonTransition{}, new_end);

    frag_stack.push(Fragment{new_start, new_end});
}

Fragment clone_fragment(NFA& nfa, const Fragment& original) {
    std::unordered_set<StateID> original_states;
    std::queue<StateID> to_visit;
    to_visit.push(original.start);
    original_states.insert(original.start);

    while (!to_visit.empty()) {
        StateID current = to_visit.front();
        to_visit.pop();

        for (const auto& trans : nfa.states[current]) {
            if (original_states.find(trans.target) == original_states.end()) {
                original_states.insert(trans.target);
                to_visit.push(trans.target);
            }
        }
    }

    std::unordered_map<StateID, StateID> state_map;
    for (StateID s : original_states) {
        state_map[s] = nfa.create_state();
    }

    for (StateID original_s : original_states) {
        StateID new_s = state_map[original_s];
        for (const auto& trans : nfa.states[original_s]) {
            TransitionCondition cond = trans.condition;
            StateID original_target = trans.target;

            StateID new_target = original_target;
            if (original_states.contains(original_target)) {
                new_target = state_map[original_target];
            }

            nfa.add_transition(new_s, cond, new_target);
        }
    }

    return Fragment{state_map[original.start], state_map[original.end]};
}

void handle_repetition_range(NFA& nfa,
                             std::stack<Fragment>& frag_stack,
                             const token::RepetitionRange& tok) {
    if (frag_stack.empty()) {
        throw BuildError("Missing operand for repetition range");
    }
    Fragment inner = frag_stack.top();
    frag_stack.pop();

    Fragment current;
    if (tok.min > 0) {
        current = clone_fragment(nfa, inner);
        for (uint32_t i = 1; i < tok.min; ++i) {
            Fragment copy = clone_fragment(nfa, inner);
            nfa.add_transition(current.end, EpsilonTransition{}, copy.start);
            current.end = copy.end;
        }
    } else {
        current.start = current.end = nfa.create_state();
    }

    if (tok.max.has_value()) {
        const uint32_t remaining = *tok.max - tok.min;
        for (uint32_t i = 0; i < remaining; ++i) {
            Fragment copy = clone_fragment(nfa, inner);
            auto branch_start = nfa.create_state();
            auto branch_end = nfa.create_state();

            nfa.add_transition(current.end, EpsilonTransition{}, branch_start);
            nfa.add_transition(branch_start, EpsilonTransition{}, copy.start);
            nfa.add_transition(copy.end, EpsilonTransition{}, branch_end);
            nfa.add_transition(branch_start, EpsilonTransition{}, branch_end);

            current.end = branch_end;
        }
    } else {
        Fragment star_fragment = clone_fragment(nfa, inner);
        auto star_start = nfa.create_state();
        auto star_end = nfa.create_state();

        nfa.add_transition(star_start, EpsilonTransition{},
                           star_fragment.start);
        nfa.add_transition(star_fragment.end, EpsilonTransition{}, star_start);
        nfa.add_transition(star_start, EpsilonTransition{}, star_end);
        nfa.add_transition(star_fragment.end, EpsilonTransition{}, star_end);

        nfa.add_transition(current.end, EpsilonTransition{}, star_start);
        current.end = star_end;
    }

    frag_stack.push(current);
}

void handle_lookahead(NFA& nfa, std::stack<Fragment>& frag_stack) {
    throw BuildError("Lookahead operator not implemented");
}

void handle_group(NFA& nfa,
                  std::stack<Fragment>& frag_stack,
                  const token::Group& group) {
    if (frag_stack.empty()) {
        throw BuildError("Empty group fragment");
    }

    Fragment inner = frag_stack.top();
    frag_stack.pop();

    const auto group_start = nfa.create_state();
    const auto group_end = nfa.create_state();

    nfa.add_transition(group_start, GroupStart{group.group_id}, inner.start);
    nfa.add_transition(inner.end, GroupEnd{group.group_id}, group_end);

    frag_stack.push(Fragment{group_start, group_end});
}

}  // namespace

NFA from_postfix(const std::vector<token::Token>& postfix) {
    NFA nfa;
    std::stack<Fragment> frag_stack;

    for (const auto& token : postfix) {
        std::visit(
            [&](auto&& tok) {
                using T = std::decay_t<decltype(tok)>;

                if constexpr (std::is_same_v<T, token::Literal>) {
                    handle_literal(nfa, frag_stack, tok);
                } else if constexpr (std::is_same_v<T, token::EmptyString>) {
                    handle_empty_string(nfa, frag_stack);
                } else if constexpr (std::is_same_v<T, token::Concatenation>) {
                    handle_concatenation(nfa, frag_stack);
                } else if constexpr (std::is_same_v<T, token::Alternation>) {
                    handle_alternation(nfa, frag_stack);
                } else if constexpr (std::is_same_v<T, token::KleeneStar>) {
                    handle_kleene_star(nfa, frag_stack);
                } else if constexpr (std::is_same_v<T,
                                                    token::PositiveClosure>) {
                    handle_positive_closure(nfa, frag_stack);
                } else if constexpr (std::is_same_v<T, token::CharacterClass>) {
                    handle_character_class(nfa, frag_stack, tok);
                } else if constexpr (std::is_same_v<T,
                                                    token::RepetitionRange>) {
                    handle_repetition_range(nfa, frag_stack, tok);
                } else if constexpr (std::is_same_v<T, token::Lookahead>) {
                    handle_lookahead(nfa, frag_stack);
                } else if constexpr (std::is_same_v<T, token::Group>) {
                    handle_group(nfa, frag_stack, tok);
                } else if constexpr (std::is_same_v<T, token::GroupOpen> ||
                                     std::is_same_v<T, token::GroupClose>) {
                    throw BuildError(
                        "Unexpected grouping operator in postfix notation");
                } else {
                    static_assert(always_false<T>, "Non-exhaustive visitor");
                }
            },
            token);
    }

    if (frag_stack.size() != 1) {
        throw BuildError(
            "Malformed expression stack: multiple fragments remaining");
    }

    const Fragment final_frag = frag_stack.top();
    nfa.start_state = final_frag.start;
    nfa.accept_state = final_frag.end;

    return nfa;
}

}  // namespace nre::nfa
