#include "dfa_matcher.hpp"
#include "from_nfa.hpp"
#include "from_postfix.hpp"
#include "shunting_yard.hpp"
#include "tokenize.hpp"

namespace nre::dfa {

DFAMatcher::DFAMatcher(std::string_view regex)
    : dfa_(minimize(from_nfa(
          nfa::from_postfix(token::shunting_yard(token::tokenize(regex)))))) {}

DFAMatcher::DFAMatcher(DFA&& dfa) : dfa_(std::move(dfa)) {}

bool DFAMatcher::is_match(std::string_view str) const {
    auto current = dfa_.start_state;
    for (char c : str) {
        const auto& transitions = dfa_.states[current];
        if (auto it = transitions.find(c); it != transitions.end()) {
            current = it->second;
        } else {
            return false;
        }
    }
    return dfa_.is_accepting(current);
}

DFA DFAMatcher::extract() && {
    return std::move(dfa_);
}

void DFAMatcher::replace(DFA&& dfa) {
    dfa_ = std::move(dfa);
}

}  // namespace nre::dfa
