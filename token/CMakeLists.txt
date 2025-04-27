add_library(token
    INTERFACE
    token.hpp
)

target_include_directories(token
    INTERFACE ..
)


add_library(tokenize
    tokenize.hpp
    tokenize.cpp
)

target_include_directories(tokenize
    PUBLIC ..
)

target_link_libraries(tokenize
    PUBLIC token
)


add_library(shunting_yard
    INTERFACE
    shunting_yard.hpp
)

target_include_directories(shunting_yard
    INTERFACE ..
)

target_link_libraries(shunting_yard
    INTERFACE token
)
