#pragma once

#include <optional>
#include <string_view>
#include <vector>
#include "nfa.hpp"

namespace nre::nfa {

class Captures {
public:
    using const_iterator = std::vector<std::string_view>::const_iterator;
    using iterator = const_iterator;

    explicit Captures(std::vector<std::string_view>&& captures);

    const_iterator begin() const;
    const_iterator end() const;

    std::string_view get_capture(std::size_t index) const;

private:
    std::vector<std::string_view> captures_;
};

class NFAMatcher {
public:
    explicit NFAMatcher(std::string_view regex, bool no_groups = false);

    explicit NFAMatcher(NFA&& nfa);

    bool is_match(std::string_view str) const;
    std::optional<Captures> captures(std::string_view str) const;

    NFA extract() &&;

    void replace(NFA&& nfa);

private:
    NFA nfa_;
};

}  // namespace nre::nfa
