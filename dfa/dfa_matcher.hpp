#pragma once

#include <string_view>
#include "dfa.hpp"

namespace nre::dfa {

class DFAMatcher {
public:
    DFAMatcher(DFA&& dfa);

    bool is_match(std::string_view input) const;

private:
    DFA dfa_;
};

}  // namespace nre::dfa
