add_library(dfa
    dfa.hpp
    dfa.cpp
)

target_include_directories(dfa
    PUBLIC ..
)


add_library(dfa_matcher
    dfa_matcher.hpp
    dfa_matcher.cpp
)

target_include_directories(dfa_matcher
    PUBLIC ..
)

target_link_libraries(dfa_matcher
    PUBLIC dfa
)


add_library(from_nfa
    from_nfa.hpp
    from_nfa.cpp
)

target_include_directories(from_nfa
    PUBLIC ..
)

target_link_libraries(from_nfa
    PUBLIC dfa
    PUBLIC nfa
)
