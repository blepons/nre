#pragma once

#include <string_view>
#include "dfa.hpp"

namespace nre::dfa {

class DFAMatcher {
public:
    explicit DFAMatcher(std::string_view regex);

    explicit DFAMatcher(DFA&& dfa);

    bool is_match(std::string_view str) const;

    DFA extract() &&;

    void replace(DFA&& dfa);

private:
    DFA dfa_;
};

}  // namespace nre::dfa
