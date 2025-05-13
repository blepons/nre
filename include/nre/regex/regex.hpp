#pragma once

#include <optional>
#include <string_view>
#include <variant>
#include "dfa_matcher.hpp"
#include "nfa_matcher.hpp"

namespace nre {

namespace regex_constants {

using OptionType = std::size_t;

inline constexpr OptionType none = 0;
inline constexpr OptionType no_groups = 1;
inline constexpr OptionType optimize = 1 << 1;

}  // namespace regex_constants

class Regex {
public:
    using FlagType = regex_constants::OptionType;
    using Captures = nfa::Captures;

    explicit Regex(std::string_view regex, FlagType f = regex_constants::none);

    bool is_match(std::string_view str) const;
    std::optional<Captures> captures(std::string_view str) const;

    friend Regex intersect(const Regex& a, const Regex& b);
    friend Regex reverse(const Regex& re);
    friend std::string to_str(const Regex& re);

private:
    explicit Regex(std::variant<nfa::NFAMatcher, dfa::DFAMatcher>&& matcher);

    std::variant<nfa::NFAMatcher, dfa::DFAMatcher> matcher_;
};

bool match(std::string_view regex, std::string_view str);

std::optional<Regex::Captures> captures(std::string_view regex,
                                        std::string_view str);

}  // namespace nre
