#include "regex.hpp"
#include <stdexcept>
#include <utility>
#include <variant>

namespace nre {

using matcher_variant = std::variant<nfa::NFAMatcher, dfa::DFAMatcher>;

namespace {

matcher_variant make_matcher(std::string_view regex, Regex::FlagType f) {
    if (f & regex_constants::optimize) {
        return dfa::DFAMatcher(regex);
    } else {
        return nfa::NFAMatcher(regex, f & regex_constants::no_groups);
    }
}

}  // namespace

Regex::Regex(matcher_variant&& matcher) : matcher_(std::move(matcher)) {}

Regex::Regex(std::string_view regex, FlagType f)
    : Regex(make_matcher(regex, f)) {}

bool Regex::is_match(std::string_view str) const {
    return std::visit(
        [str](const auto& matcher) { return matcher.is_match(str); }, matcher_);
}

std::optional<Regex::Captures> Regex::captures(std::string_view str) const {
    return std::visit(
        [str](const auto& matcher) -> std::optional<Captures> {
            using T = std::decay_t<decltype(matcher)>;

            if constexpr (std::is_same_v<T, nfa::NFAMatcher>) {
                return matcher.captures(str);
            } else if constexpr (std::is_same_v<T, dfa::DFAMatcher>) {
                throw std::runtime_error("Captures are not supported in DFA");
            }
        },
        matcher_);
}

bool match(std::string_view regex, std::string_view str) {
    return nfa::NFAMatcher(regex, true).is_match(str);
}

std::optional<Regex::Captures> captures(std::string_view regex,
                                        std::string_view str) {
    return nfa::NFAMatcher(regex, false).captures(str);
}

}  // namespace nre
