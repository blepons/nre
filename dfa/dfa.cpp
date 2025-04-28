#include "dfa.hpp"
#include <algorithm>
#include <map>
#include <queue>
#include <ranges>
#include <utility>

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
                if (!state_map.contains(next_pair)) {
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

namespace {

std::unordered_set<char> collect_alphabet(const DFA& dfa) {
    std::unordered_set<char> chars;
    for (const auto& state_transitions : dfa.states) {
        for (auto [ch, _] : state_transitions) {
            chars.insert(ch);
        }
    }
    return chars;
}

DFA complete(const DFA& dfa, const auto& alphabet) {
    DFA complete = dfa;
    if (std::ranges::any_of(dfa.states, [alphabet_size = alphabet.size()](
                                            const auto& transitions) {
            return transitions.size() != alphabet_size;
        })) {
        StateID dead = complete.create_state();
        for (StateID s = 0; s < complete.states.size(); ++s) {
            auto& transitions = complete.states[s];
            for (char c : alphabet) {
                transitions.try_emplace(c, dead);
            }
        }
    }
    return complete;
}

std::vector<std::unordered_set<StateID>> initial_partition(const DFA& dfa) {
    std::vector<std::unordered_set<StateID>> partition;
    auto non_accepting_rg =
        std::views::iota(StateID{}, StateID{dfa.states.size()}) |
        std::views::filter([&dfa](StateID s) { return !dfa.is_accepting(s); });

    std::unordered_set<StateID> non_accepting(non_accepting_rg.begin(),
                                              non_accepting_rg.end());
    if (!dfa.accept_states.empty()) {
        partition.push_back(dfa.accept_states);
    }
    if (!non_accepting.empty()) {
        partition.push_back(std::move(non_accepting));
    }
    return partition;
}

std::vector<std::size_t> compute_signature(
    const DFA& completed,
    StateID state,
    const std::vector<std::unordered_set<StateID>>& partition,
    const std::unordered_set<char>& alphabet) {
    std::vector<size_t> sig;
    for (char c : alphabet) {
        auto it = std::ranges::find_if(
            partition, [next = completed.states[state].at(c)](
                           const auto& group) { return group.contains(next); });
        sig.push_back(std::ranges::distance(partition.begin(), it));
    }
    return sig;
}

std::vector<std::unordered_set<StateID>> process_partition(
    const std::vector<std::unordered_set<StateID>>& partition,
    const std::unordered_set<char>& alphabet,
    const DFA& dfa) {
    std::vector<std::unordered_set<StateID>> new_partition;

    for (const auto& group : partition) {
        if (group.empty())
            continue;

        std::map<std::vector<std::size_t>, std::unordered_set<StateID>> sig_map;
        for (StateID s : group) {
            sig_map[compute_signature(dfa, s, partition, alphabet)].insert(s);
        }

        for (auto&& [_, subgroup] : sig_map) {
            new_partition.push_back(std::move(subgroup));
        }
    }

    return new_partition;
}

std::vector<std::unordered_set<StateID>> compute_partitions(
    const DFA& completed,
    const std::unordered_set<char>& alphabet) {
    auto partition = initial_partition(completed);

    while (true) {
        auto new_partition = process_partition(partition, alphabet, completed);
        if (partition == new_partition) {
            break;
        }
        partition = std::move(new_partition);
    }
    return partition;
}

DFA build_minimal_dfa(const DFA& completed,
                      const std::vector<std::unordered_set<StateID>>& partition,
                      const std::unordered_set<char>& alphabet) {
    DFA minimal;

    for (std::size_t i = 0; i < partition.size(); ++i) {
        minimal.create_state();
        bool accepting = std::ranges::any_of(
            partition[i],
            [&completed](StateID s) { return completed.is_accepting(s); });
        minimal.set_accepting(i, accepting);
    }

    auto it = std::ranges::find_if(
        partition, [start = completed.start_state](const auto& group) {
            return group.contains(start);
        });
    minimal.start_state = std::ranges::distance(partition.begin(), it);

    std::unordered_map<StateID, size_t> state_to_part;
    for (auto [i, group] : std::views::enumerate(partition)) {
        for (StateID s : group) {
            state_to_part[s] = i;
        }
    }

    for (auto [i, group] : std::views::enumerate(partition)) {
        StateID rep = *group.begin();
        for (char c : alphabet) {
            StateID next = completed.states[rep].at(c);
            minimal.add_transition(i, c, state_to_part[next]);
        }
    }

    return minimal;
}

}  // namespace

DFA minimize(const DFA& automaton) {
    auto alphabet = collect_alphabet(automaton);
    DFA completed = complete(automaton, alphabet);
    auto partition = compute_partitions(completed, alphabet);
    return build_minimal_dfa(completed, partition, alphabet);
}

}  // namespace nre::dfa
