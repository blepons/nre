#pragma once

#include <stack>
#include <vector>
#include "token/token.hpp"

namespace nre {

namespace detail {

template <typename... Alts, typename... Ts>
constexpr bool holds_any_of(const std::variant<Ts...>& v) noexcept {
    return (std::holds_alternative<Alts>(v) || ...);
}

inline int precedence(const Token& token) {
    return std::visit(
        [](auto&& tok) {
            using T = std::decay_t<decltype(tok)>;
            if constexpr (std::is_same_v<T, KleeneStar> ||
                          std::is_same_v<T, PositiveClosure> ||
                          std::is_same_v<T, RepetitionRange>) {
                return 4;
            } else if constexpr (std::is_same_v<T, Lookahead>) {
                return 3;
            } else if constexpr (std::is_same_v<T, Concatenation>) {
                return 2;
            } else if constexpr (std::is_same_v<T, Alternation>) {
                return 1;
            } else {
                return 0;
            }
        },
        token);
}

inline bool is_operator(const Token& token) {
    return holds_any_of<KleeneStar, PositiveClosure, RepetitionRange, Lookahead,
                        Concatenation, Alternation>(token);
}

inline bool is_left_associative(const Token& token) {
    return holds_any_of<Alternation, Concatenation, Lookahead>(token);
}

inline bool is_unary_operator(const Token& token) {
    return holds_any_of<KleeneStar, PositiveClosure, RepetitionRange>(token);
}

inline bool is_binary_operator(const Token& token) {
    return holds_any_of<Alternation, Concatenation, Lookahead>(token);
}

inline bool can_start_expr(const Token& token) {
    return holds_any_of<Literal, EmptyString, CharacterClass, GroupOpen>(token);
}

}  // namespace detail

template <std::ranges::input_range R>
std::vector<Token> shunting_yard(R&& tokens) {
    using namespace detail;
    std::vector<Token> output;
    std::stack<Token> op_stack;
    bool prev_was_operand = false;
    std::stack<std::pair<int, std::size_t>> group_stack;

    for (auto&& token : tokens) {
        if (prev_was_operand && can_start_expr(token)) {
            Token concat = Concatenation{};
            while (!op_stack.empty() && is_operator(op_stack.top()) &&
                   precedence(op_stack.top()) >= precedence(concat)) {
                output.push_back(std::move(op_stack.top()));
                op_stack.pop();
            }
            op_stack.push(std::move(concat));
        }

        if (std::holds_alternative<GroupOpen>(token)) {
            auto& group_open = std::get<GroupOpen>(token);
            op_stack.push(token);
            group_stack.emplace(group_open.group_id, output.size());
            prev_was_operand = false;
        } else if (std::holds_alternative<GroupClose>(token)) {
            auto& group_close = std::get<GroupClose>(token);

            while (!op_stack.empty() &&
                   !std::holds_alternative<GroupOpen>(op_stack.top())) {
                output.push_back(std::move(op_stack.top()));
                op_stack.pop();
            }

            if (op_stack.empty()) {
                throw ParseError("Unbalanced parentheses");
            }

            op_stack.pop();

            auto [group_id, start_index] = group_stack.top();
            group_stack.pop();

            if (output.size() == start_index) {
                output.insert(output.begin() + start_index, EmptyString{});
            }

            output.emplace_back(Group{group_close.group_id});
            prev_was_operand = true;
        } else if (is_operator(token)) {
            if (is_binary_operator(token) && !prev_was_operand) {
                throw ParseError("Missing left operand for binary operator");
            }
            if (is_unary_operator(token) && !prev_was_operand) {
                throw ParseError("Missing operand for unary operator");
            }

            while (!op_stack.empty() && is_operator(op_stack.top())) {
                auto& top = op_stack.top();
                if (precedence(top) > precedence(token) ||
                    (precedence(top) == precedence(token) &&
                     is_left_associative(token))) {
                    output.push_back(std::move(top));
                    op_stack.pop();
                } else {
                    break;
                }
            }

            op_stack.push(std::move(token));
            prev_was_operand = is_unary_operator(token);
        } else {
            output.push_back(std::move(token));
            prev_was_operand = true;
        }
    }

    while (!op_stack.empty()) {
        if (std::holds_alternative<GroupOpen>(op_stack.top())) {
            throw ParseError("Unbalanced parentheses");
        }
        output.push_back(std::move(op_stack.top()));
        op_stack.pop();
    }

    return output;
}

}  // namespace nre
