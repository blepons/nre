cmake_minimum_required(VERSION 3.20)
project(nregex LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

add_compile_options(
    $<$<COMPILE_LANGUAGE:CXX>:-Wall>
    $<$<COMPILE_LANGUAGE:CXX>:-Wextra>
)

add_subdirectory(token)

add_subdirectory(nfa)

add_subdirectory(dfa)
