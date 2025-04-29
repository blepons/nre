#pragma once

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <unordered_set>
#include <variant>

namespace nre {

struct Literal {
    char value;
};
struct EmptyString {};
struct Concatenation {};
struct Alternation {};
struct KleeneStar {};
struct PositiveClosure {};
struct CharacterClass {
    std::unordered_set<char> chars;
};
struct RepetitionRange {
    uint32_t min;
    std::optional<uint32_t> max;
};
struct GroupOpen {
    int group_id;
};
struct GroupClose {
    int group_id;
};
struct Group {
    int group_id;
};
struct Lookahead {};

using Token = std::variant<Literal,
                           EmptyString,
                           Concatenation,
                           Alternation,
                           KleeneStar,
                           PositiveClosure,
                           CharacterClass,
                           RepetitionRange,
                           GroupOpen,
                           GroupClose,
                           Lookahead,
                           Group>;

class ParseError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

}  // namespace nre
