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

## ライブラリ化
# lexer
add_library(planner_lexer STATIC src/lexer.cpp)
target_include_directories(planner_lexer PUBLIC ${CMAKE_SOURCE_DIR}/include)

# parser
add_library(planner_parser STATIC src/parser.cpp)
target_link_libraries(planner_parser PUBLIC planner_lexer)
target_include_directories(planner_parser PUBLIC ${CMAKE_SOURCE_DIR}/include)

# grounding
add_library(planner_grounding STATIC src/grounding.cpp)
target_link_libraries(planner_grounding PUBLIC planner_parser)
target_include_directories(planner_grounding PUBLIC ${CMAKE_SOURCE_DIR}/include)

## テスト実行ファイル
# lexer
add_executable(lexer_pair_test
    src/lexer.cpp
    tests/lexer_pair_test.cpp
)
target_include_directories(lexer_pair_test PRIVATE ${CMAKE_SOURCE_DIR}/include)

# parser
add_executable(parse_demo tests/parse_demo.cpp)
target_link_libraries(parse_demo PRIVATE planner_parser)
target_include_directories(parse_demo PRIVATE ${CMAKE_SOURCE_DIR}/include)

# grounding
add_executable(grounding_test tests/grounding_test.cpp)
target_link_libraries(grounding_test PRIVATE planner_grounding)
target_include_directories(grounding_test PRIVATE ${CMAKE_SOURCE_DIR}/include)
