#include "nfa_matcher.hpp"
#include <algorithm>
#include <cstddef>
#include <optional>
#include <queue>
#include <ranges>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include "from_postfix.hpp"
#include "nfa.hpp"
#include "shunting_yard.hpp"
#include "tokenize.hpp"

namespace nre::nfa {

Captures::Captures(std::vector<std::string_view>&& captures)
    : captures_(std::move(captures)) {}

std::string_view Captures::get_capture(std::size_t index) const {
    return index < captures_.size() ? captures_[index] : "";
}

Captures::const_iterator Captures::begin() const {
    return captures_.cbegin();
}

Captures::const_iterator Captures::end() const {
    return captures_.cend();
}

NFAMatcher::NFAMatcher(std::string_view regex)
    : nfa_(from_postfix(token::shunting_yard(token::tokenize(regex)))) {}

NFAMatcher::NFAMatcher(NFA&& nfa) : nfa_(std::move(nfa)) {}

namespace {

std::unordered_set<StateID> epsilon_closure_with_lookahead(
    const NFA& nfa,
    std::unordered_set<StateID>&& states,
    std::string_view str);

constexpr std::size_t match_npos = -1;

template <bool LazyMatching = false>
std::size_t match_string(const NFA& nfa,
                         StateID start,
                         StateID end,
                         std::string_view str) {
    auto matched = match_npos;
    auto current = epsilon_closure_with_lookahead(nfa, {start}, str);
    if (current.contains(end)) {
        matched = 0;
        if constexpr (LazyMatching) {
            return matched;
        }
    }

    for (auto [pos, c] : std::views::enumerate(str)) {
        std::unordered_set<StateID> next_states;

        for (auto state : current) {
            for (const auto& trans : nfa.states[state]) {
                if (const auto* ch = std::get_if<char>(&trans.condition)) {
                    if (*ch == c) {
                        next_states.insert(trans.target);
                    }
                }
            }
        }

        if (next_states.empty()) {
            return matched;
        }

        current = epsilon_closure_with_lookahead(nfa, std::move(next_states),
                                                 str.substr(pos + 1));
        if (current.contains(end)) {
            matched = pos + 1;
            if constexpr (LazyMatching) {
                return matched;
            }
        }
    }

    return matched;
}

std::unordered_set<StateID> epsilon_closure_with_lookahead(
    const NFA& nfa,
    std::unordered_set<StateID>&& states,
    std::string_view str) {
    std::unordered_set<StateID> closure(std::move(states));
    std::queue<StateID> processing_queue(closure.begin(), closure.end());

    while (!processing_queue.empty()) {
        StateID current = processing_queue.front();
        processing_queue.pop();

        for (const auto& trans : nfa.states[current]) {
            std::visit(
                [&](auto&& condition) {
                    using T = std::decay_t<decltype(condition)>;
                    if constexpr (std::is_same_v<T, GroupStart> ||
                                  std::is_same_v<T, GroupEnd> ||
                                  std::is_same_v<T, EpsilonTransition>) {
                        if (!closure.contains(trans.target)) {
                            closure.insert(trans.target);
                            processing_queue.push(trans.target);
                        }
                    } else if constexpr (std::is_same_v<T, Lookahead>) {
                        if (match_string<true>(nfa, condition.start,
                                               condition.end,
                                               str) != match_npos) {
                            if (!closure.contains(trans.target)) {
                                closure.insert(trans.target);
                                processing_queue.push(trans.target);
                            }
                        }
                    }
                },
                trans.condition);
        }
    }

    return closure;
}

}  // namespace

bool NFAMatcher::is_match(std::string_view str) const {
    return match_string(nfa_, nfa_.start_state, nfa_.accept_state, str) ==
           str.length();
}

namespace {

struct GroupPositions {
    std::size_t start;
    std::size_t end;

    bool operator==(const GroupPositions&) const = default;
};

struct SimulationState {
    std::size_t id;
    std::unordered_map<int, std::size_t> active_groups;
    std::unordered_map<int, GroupPositions> completed_groups;

    bool operator==(const SimulationState&) const = default;
};

template <class T>
static std::size_t combine_hash(std::size_t seed, const T& v) {
    seed ^= std::hash<T>{}(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
}

struct SimulationStateHash {
    std::size_t operator()(const SimulationState& s) const {
        auto hasher = std::hash<std::size_t>{};
        auto h = hasher(s.id);
        for (const auto& [gid, start_pos] : s.active_groups) {
            h = combine_hash(h, gid);
            h = combine_hash(h, start_pos);
        }
        for (const auto& [gid, pos] : s.completed_groups) {
            h = combine_hash(h, gid);
            h = combine_hash(h, pos.start);
            h = combine_hash(h, pos.end);
        }
        return h;
    }
};

std::unordered_set<SimulationState, SimulationStateHash>
epsilon_closure_with_groups(
    const NFA& nfa,
    std::unordered_set<SimulationState, SimulationStateHash>&& states,
    std::size_t current_pos,
    std::string_view str) {
    std::unordered_set<SimulationState, SimulationStateHash> closure =
        std::move(states);
    std::queue<SimulationState> processing_queue(closure.begin(),
                                                 closure.end());
    while (!processing_queue.empty()) {
        auto current = std::move(processing_queue.front());
        processing_queue.pop();

        for (const auto& trans : nfa.states[current.id]) {
            if (std::holds_alternative<char>(trans.condition)) {
                continue;
            }

            SimulationState new_state = current;
            new_state.id = trans.target;
            bool transition_valid = true;

            std::visit(
                [&](auto&& condition) {
                    using T = std::decay_t<decltype(condition)>;
                    if constexpr (std::is_same_v<T, GroupStart>) {
                        new_state.active_groups[condition.group_id] =
                            current_pos;
                    } else if constexpr (std::is_same_v<T, GroupEnd>) {
                        if (auto it = new_state.active_groups.find(
                                condition.group_id);
                            it != new_state.active_groups.end()) {
                            new_state.completed_groups[condition.group_id] = {
                                it->second, current_pos - 1};
                            new_state.active_groups.erase(it);
                        }
                    } else if constexpr (std::is_same_v<T, Lookahead>) {
                        if (match_string<true>(
                                nfa, condition.start, condition.end,
                                str.substr(current_pos)) == match_npos) {
                            transition_valid = false;
                        }
                    }
                },
                trans.condition);

            if (transition_valid && !closure.contains(new_state)) {
                closure.insert(new_state);
                processing_queue.push(new_state);
            }
        }
    }

    return closure;
}

}  // namespace

// BUG: group indices are not correct if nfa has lookaheads
std::optional<Captures> NFAMatcher::captures(std::string_view str) const {
    std::unordered_set<SimulationState, SimulationStateHash> current_states{
        SimulationState{nfa_.start_state, {}, {}}};
    current_states =
        epsilon_closure_with_groups(nfa_, std::move(current_states), 0, str);

    for (auto [pos, c] : std::views::enumerate(str)) {
        std::unordered_set<SimulationState, SimulationStateHash> next_states;

        for (const auto& state : current_states) {
            for (const auto& trans : nfa_.states[state.id]) {
                if (const auto* ch = std::get_if<char>(&trans.condition)) {
                    if (*ch == c) {
                        SimulationState new_state = state;
                        new_state.id = trans.target;
                        next_states.insert(std::move(new_state));
                    }
                }
            }
        }

        if (next_states.empty()) {
            return std::nullopt;
        }

        current_states = epsilon_closure_with_groups(
            nfa_, std::move(next_states), pos + 1, str);
    }

    const SimulationState* best = nullptr;
    for (const auto& state : current_states) {
        if (state.id == nfa_.accept_state) {
            if (!best ||
                state.completed_groups.size() > best->completed_groups.size()) {
                best = &state;
            }
        }
    }

    if (!best) {
        return std::nullopt;
    }

    auto group_indices = best->completed_groups | std::views::keys;
    auto max = std::ranges::max_element(group_indices);
    int max_group = (max != std::ranges::end(group_indices)) ? *max : 0;

    std::vector<std::string_view> captures;

    captures.resize(max_group + 1);
    captures[0] = str;
    for (const auto& [gid, pos] : best->completed_groups) {
        captures[gid] = str.substr(pos.start, pos.end - pos.start + 1);
    }

    return Captures(std::move(captures));
}

NFA NFAMatcher::extract() && {
    return std::move(nfa_);
}

void NFAMatcher::replace(NFA&& nfa) {
    nfa_ = std::move(nfa);
}

}  // namespace nre::nfa
