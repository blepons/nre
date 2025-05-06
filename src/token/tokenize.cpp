#include "tokenize.hpp"
#include <cctype>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stack>
#include <string_view>
#include <system_error>
#include <utility>

namespace nre::token {

namespace {

struct ParseResult {
    std::size_t new_pos;
    Token token;
};

ParseResult parse_character_class(std::string_view regex,
                                  std::size_t start_pos) {
    std::size_t pos = start_pos;
    std::unordered_set<char> chars;

    if (pos >= regex.size()) {
        throw ParseError("Unclosed character class");
    }
    if (regex[pos] == '^') {
        throw ParseError("Negated character classes are not supported");
    }

    auto handle_escape = [&] {
        if (++pos >= regex.size()) {
            throw ParseError("Escape at end of regex");
        }
        chars.insert(regex[pos++]);
    };

    auto get_char = [regex](std::size_t pos) -> std::pair<char, std::size_t> {
        if (pos >= regex.size()) {
            throw ParseError("Unexpected end of regex in range");
        }
        if (regex[pos] == '%') {
            ++pos;
            if (pos >= regex.size()) {
                throw ParseError("Escape at end of regex in range");
            }
        }
        return {regex[pos], pos + 1};
    };

    while (pos < regex.size()) {
        if (regex[pos] == '%') {
            handle_escape();
        } else if (regex[pos] == ']') {
            return {++pos, CharacterClass{std::move(chars)}};
        } else {
            char start = regex[pos];
            ++pos;

            if (pos < regex.size() && regex[pos] == '-') {
                if (pos + 1 >= regex.size()) {
                    chars.insert(start);
                    chars.insert('-');
                    ++pos;
                } else if (regex[pos + 1] == ']') {
                    chars.insert(start);
                    chars.insert('-');
                } else {
                    auto [end, new_pos] = get_char(pos + 1);
                    if (start > end) {
                        throw ParseError("Invalid range: start > end");
                    }
                    for (char c = start; c <= end; ++c) {
                        chars.insert(c);
                    }
                    pos = new_pos;
                }
            } else {
                chars.insert(start);
            }
        }
    }

    throw ParseError("Unclosed character class");
}

ParseResult parse_repetition_range(std::string_view regex,
                                   std::size_t start_pos) {
    auto extract_num = [](std::string_view s,
                          std::size_t& pos) -> std::optional<uint32_t> {
        uint32_t value = 0;
        auto [ptr, ec] =
            std::from_chars(s.data() + pos, s.data() + s.size(), value);
        pos = ptr - s.data();
        return ec == std::errc{} ? std::make_optional(value) : std::nullopt;
    };

    std::size_t pos = start_pos;
    uint32_t min = extract_num(regex, pos).value_or(0);

    if (pos >= regex.size()) {
        throw ParseError("Unexpected end in repetition range");
    }

    char sep = regex[pos];
    ++pos;
    std::optional<uint32_t> max;

    if (sep == '}') {
        max = min;
    } else if (sep == ',') {
        max = extract_num(regex, pos);
        if (pos >= regex.size() || regex[pos] != '}') {
            throw ParseError("Repetition range missing closing '}'");
        }
        ++pos;
    } else {
        throw ParseError("Invalid separator in repetition range");
    }

    return {pos, RepetitionRange{min, max}};
}

}  // namespace

std::generator<Token> tokenize(std::string_view regex) {
    std::size_t pos = 0;
    int group_counter = 0;
    std::stack<int> group_stack;
    if (regex.empty()) {
        co_yield EmptyString{};
        co_return;
    }
    while (pos < regex.size()) {
        const char c = regex[pos];
        switch (c) {
            case '%':
                if (++pos >= regex.size()) {
                    throw ParseError("Incomplete escape sequence");
                }
                co_yield Literal{regex[pos]};
                ++pos;
                break;
            case '$':
                co_yield EmptyString{};
                ++pos;
                break;
            case '|':
                co_yield Alternation{};
                ++pos;
                break;
            case '*':
                co_yield KleeneStar{};
                ++pos;
                break;
            case '+':
                co_yield PositiveClosure{};
                ++pos;
                break;
            case '.':
                co_yield Concatenation{};
                ++pos;
                break;
            case '(': {
                int group_id = ++group_counter;
                group_stack.push(group_id);
                co_yield GroupOpen{group_id};
                ++pos;
                break;
            }
            case ')':
                if (group_stack.empty()) {
                    throw ParseError("Unbalanced parentheses");
                }
                co_yield GroupClose{group_stack.top()};
                group_stack.pop();
                ++pos;
                break;
            case '[': {
                ++pos;
                auto [new_pos, token] = parse_character_class(regex, pos);
                pos = new_pos;
                co_yield std::move(token);
                break;
            }
            case '{': {
                ++pos;
                auto [new_pos, token] = parse_repetition_range(regex, pos);
                pos = new_pos;
                co_yield token;
                break;
            }
            case '/':
                co_yield Lookahead{};
                ++pos;
                break;
            default:
                co_yield Literal{c};
                ++pos;
                break;
        }
    }
}

}  // namespace nre::token
