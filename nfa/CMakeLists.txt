add_library(nfa
    nfa.hpp
    nfa.cpp
)

target_include_directories(nfa
    PUBLIC ..
)


add_library(nfa_matcher
    nfa_matcher.hpp
    nfa_matcher.cpp
)

target_include_directories(nfa_matcher
    PUBLIC ..
)

target_link_libraries(nfa_matcher
    PUBLIC nfa
)


add_library(from_postfix
    from_postfix.hpp
    from_postfix.cpp
)

target_include_directories(from_postfix
    PUBLIC ..
)

target_link_libraries(from_postfix
    PUBLIC nfa
    PUBLIC token
)
