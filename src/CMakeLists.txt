target_sources(nre
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/token/tokenize.cpp
    PUBLIC
        FILE_SET HEADERS
        BASE_DIRS ${CMAKE_CURRENT_LIST_DIR}/../include/nre/token
        FILES
            ${CMAKE_CURRENT_LIST_DIR}/../include/nre/token/token.hpp
            ${CMAKE_CURRENT_LIST_DIR}/../include/nre/token/tokenize.hpp
            ${CMAKE_CURRENT_LIST_DIR}/../include/nre/token/shunting_yard.hpp
)

target_sources(nre
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/nfa/nfa.cpp
        ${CMAKE_CURRENT_LIST_DIR}/nfa/nfa_matcher.cpp
        ${CMAKE_CURRENT_LIST_DIR}/nfa/from_postfix.cpp
    PUBLIC
        FILE_SET HEADERS
        BASE_DIRS ${CMAKE_CURRENT_LIST_DIR}/../include/nre/nfa
        FILES
            ${CMAKE_CURRENT_LIST_DIR}/../include/nre/nfa/nfa.hpp
            ${CMAKE_CURRENT_LIST_DIR}/../include/nre/nfa/nfa_matcher.hpp
            ${CMAKE_CURRENT_LIST_DIR}/../include/nre/nfa/from_postfix.hpp
)

target_sources(nre
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/dfa/dfa.cpp
        ${CMAKE_CURRENT_LIST_DIR}/dfa/dfa_matcher.cpp
        ${CMAKE_CURRENT_LIST_DIR}/dfa/from_nfa.cpp
    PUBLIC
        FILE_SET HEADERS
        BASE_DIRS ${CMAKE_CURRENT_LIST_DIR}/../include/nre/dfa
        FILES
            ${CMAKE_CURRENT_LIST_DIR}/../include/nre/dfa/dfa.hpp
            ${CMAKE_CURRENT_LIST_DIR}/../include/nre/dfa/dfa_matcher.hpp
            ${CMAKE_CURRENT_LIST_DIR}/../include/nre/dfa/from_nfa.hpp
)

target_sources(nre
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/regex/regex.cpp
    PUBLIC
        FILE_SET HEADERS
        BASE_DIRS ${CMAKE_CURRENT_LIST_DIR}/../include/nre/regex/
        FILES
            ${CMAKE_CURRENT_LIST_DIR}/../include/nre/regex/regex.hpp
)
