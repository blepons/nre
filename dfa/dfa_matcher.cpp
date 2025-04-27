#include "dfa_matcher.hpp"

namespace nre::dfa {

DFAMatcher::DFAMatcher(DFA&& dfa) : dfa_(std::move(dfa)) {}

bool DFAMatcher::is_match(std::string_view input) const {
    auto current = dfa_.start_state;
    for (char c : input) {
        const auto& transitions = dfa_.states[current];
        if (auto it = transitions.find(c); it != transitions.end()) {
            current = it->second;
        } else {
            return false;
        }
    }
    return dfa_.is_accepting(current);
}

}  // namespace nre::dfa
