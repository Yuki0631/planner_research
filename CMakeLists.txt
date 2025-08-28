cmake_minimum_required(VERSION 3.20)
project(planner_not_fd CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ビルドタイプ既定
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# 警告と最適化（GCC/Clang想定）
add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion -O3 -march=native)
# （デバッグ時）-fsanitize=address,undefined を付ける用のオプションを後で切替

add_executable(planner_not_fd
  src/main.cpp
  src/lexer.cpp src/parser.cpp
  src/grounding.cpp
  src/strips.cpp
  src/heuristic.cpp
)
target_include_directories(planner_not_fd PRIVATE include)

add_executable(lexer_pair_test
    src/lexer.cpp
    tests/lexer_pair_test.cpp
)
target_include_directories(lexer_pair_test PRIVATE ${CMAKE_SOURCE_DIR}/include)
