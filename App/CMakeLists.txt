cmake_minimum_required(VERSION 3.16)

project(AppLayer LANGUAGES CXX)


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(Lib/Ponoor)
add_subdirectory(Abstract)

# Add source files
add_executable(${PROJECT_NAME} 
    main.cpp

)


target_link_libraries(${PROJECT_NAME}
    PUBLIC
    PonoorLib
    AbstractLayer
)

# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)


message(STATUS "App Layer")