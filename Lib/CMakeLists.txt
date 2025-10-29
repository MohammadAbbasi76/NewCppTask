project(PonoorLib LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# Create library
add_library(${PROJECT_NAME}
    # PiFracMotor.cpp
    # PiFracMotor.h
    # Ponoor_L6470Commands.cpp
    # Ponoor_L6470Config.cpp
    # Ponoor_L6470Constants.h
    # Ponoor_L6470Library.cpp
    # Ponoor_L6470Library.h
    # Ponoor_L6470Support.cpp
)

# Include directories - include both current and project include dirs
target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)


if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${PROJECT_NAME} PRIVATE /W4)
else()
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()

message(STATUS "Ponoor Library")