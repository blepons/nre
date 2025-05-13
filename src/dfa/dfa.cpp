#include "dfa.hpp"
#include <algorithm>
#include <format>
#include <map>
#include <optional>
#include <queue>
#include <ranges>
#include <string_view>
#include <unordered_set>
#include <utility>
#include "from_nfa.hpp"
#include "nfa.hpp"

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

namespace {

void collect_alphabet(const DFA& dfa, std::unordered_set<char>& chars) {
    for (const auto& state_transitions : dfa.states) {
        for (auto [ch, _] : state_transitions) {
            chars.insert(ch);
        }
    }
}

std::unordered_set<char> collect_alphabet(const DFA& dfa) {
    std::unordered_set<char> chars;
    collect_alphabet(dfa, chars);
    return chars;
}

}  // namespace

DFA intersect(const DFA& a, const DFA& b) {
    DFA product;

    std::unordered_set<char> combined_alphabet;
    collect_alphabet(a, combined_alphabet);
    collect_alphabet(b, combined_alphabet);

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

DFA reverse(const DFA& automaton) {
    nfa::NFA reversed_nfa;
    const auto new_start = reversed_nfa.create_state();
    for (std::size_t i = 0; i < automaton.states.size(); ++i) {
        reversed_nfa.create_state();
    }

    for (StateID s : automaton.accept_states) {
        reversed_nfa.add_transition(new_start, nfa::EpsilonTransition{}, s + 1);
    }
    for (auto [from, transitions] : std::views::enumerate(automaton.states)) {
        for (const auto& [c, to] : transitions) {
            reversed_nfa.add_transition(to + 1, c, from + 1);
        }
    }

    reversed_nfa.accept_state = automaton.start_state + 1;

    return from_nfa(reversed_nfa);
}

namespace {

bool needs_parentheses(std::string_view r) {
    if (r == "[]" || r == "$") {
        return false;
    }
    if (r.size() == 1) {
        return false;
    }
    return true;
}

std::string parenthesize_regex(std::string_view r) {
    return needs_parentheses(r) ? std::format("({})", r) : std::string{r};
}

std::string concat_regex(std::string_view a, std::string_view b) {
    if (a == "[]" || b == "[]") {
        return "[]";
    }
    if (a == "$") {
        return std::string{b};
    }
    if (b == "$") {
        return std::string{a};
    }
    return parenthesize_regex(a) + parenthesize_regex(b);
}

std::string star_regex(std::string_view a) {
    if (a == "[]") {
        return "$";
    }
    if (a == "$") {
        return "$";
    }
    return std::format("{}*", parenthesize_regex(a));
}

std::string union_regex(std::string_view a, std::string_view b) {
    if (a == "[]") {
        return std::string{b};
    }
    if (b == "[]") {
        return std::string{a};
    }
    if (a == b) {
        return std::string{a};
    }
    return std::format("{}|{}", parenthesize_regex(a), parenthesize_regex(b));
}

template <typename T, typename K>
std::optional<typename std::remove_cvref_t<T>::mapped_type> try_get(T&& map,
                                                                    K&& key) {
    auto it = map.find(std::forward<K>(key));
    if (it == map.end()) {
        return std::nullopt;
    }
    return it->second;
}

using Transitions = std::vector<std::unordered_map<StateID, std::string>>;

Transitions build_base_transitions(const DFA& automaton) {
    Transitions transitions(automaton.states.size() + 2);
    for (auto [from, state_trans] : std::views::enumerate(automaton.states)) {
        for (const auto& [c, to] : state_trans) {
            std::string ch_str(1, c);
            if (transitions[from].contains(to)) {
                transitions[from][to] =
                    union_regex(transitions[from][to], ch_str);
            } else {
                transitions[from][to] = ch_str;
            }
        }
    }
    return transitions;
}

void add_start_and_accept_transitions(Transitions& transitions,
                                      const DFA& automaton,
                                      StateID s_id,
                                      StateID a_id) {
    transitions[s_id][automaton.start_state] = "$";
    for (StateID accept_state : automaton.accept_states) {
        transitions[accept_state][a_id] = "$";
    }
}

std::unordered_map<StateID, std::string> incoming_transitions(
    const Transitions& transitions,
    StateID k) {
    std::unordered_map<StateID, std::string> R_ik;
    for (StateID i = 0; i < transitions.size(); ++i) {
        if (i == k) {
            continue;
        }
        R_ik[i] = try_get(transitions[i], k).value_or("[]");
    }
    return R_ik;
}

std::unordered_map<StateID, std::string> outgoing_ransitions(
    const Transitions& transitions,
    StateID k) {
    std::unordered_map<StateID, std::string> R_kj;
    const auto& k_trans = transitions[k];
    for (StateID j = 0; j < transitions.size(); ++j) {
        if (j == k) {
            continue;
        }
        R_kj[j] = try_get(k_trans, j).value_or("[]");
    }
    return R_kj;
}

void update_transition_pairs(
    Transitions& transitions,
    const std::unordered_map<StateID, std::string>& R_ik,
    const std::unordered_map<StateID, std::string>& R_kj,
    std::string_view R_kk_star) {
    for (const auto& [i, rik] : R_ik) {
        for (const auto& [j, rkj] : R_kj) {
            std::string term = concat_regex(rik, concat_regex(R_kk_star, rkj));
            std::string current = try_get(transitions[i], j).value_or("[]");
            std::string new_regex = union_regex(current, term);

            if (new_regex != "[]") {
                transitions[i][j] = new_regex;
            } else {
                transitions[i].erase(j);
            }
        }
    }
}

void eliminate_state(Transitions& transitions, StateID k) {
    auto R_ik = incoming_transitions(transitions, k);
    auto R_kj = outgoing_ransitions(transitions, k);
    std::string R_kk = try_get(transitions[k], k).value_or("[]");
    std::string R_kk_star = star_regex(R_kk);

    update_transition_pairs(transitions, R_ik, R_kj, R_kk_star);

    for (auto& state_trans : transitions) {
        state_trans.erase(k);
    }
}

}  // namespace

std::string to_regex(const DFA& automaton) {
    if (automaton.states.size() == 0) {
        return "[]";
    }

    const auto s_id = automaton.states.size();
    const auto a_id = s_id + 1;
    auto transitions = build_base_transitions(automaton);
    add_start_and_accept_transitions(transitions, automaton, s_id, a_id);

    for (StateID k = 0; k < automaton.states.size(); ++k) {
        eliminate_state(transitions, k);
    }

    return try_get(transitions[s_id], a_id).value_or("[]");
}

}  // namespace nre::dfa
