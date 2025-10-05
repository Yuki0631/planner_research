cmake_minimum_required(VERSION 3.20)
project(planner CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion -O3 -march=native)

# --- ライブラリ ---
# ヘッダ
include_directories(${CMAKE_SOURCE_DIR}/include)

# 下位層からライブラリを構成する
add_library(planner_lexer STATIC src/lexer.cpp)
target_include_directories(planner_lexer PUBLIC ${CMAKE_SOURCE_DIR}/include)

add_library(planner_parser STATIC src/parser.cpp)
target_link_libraries(planner_parser PUBLIC planner_lexer)
target_include_directories(planner_parser PUBLIC ${CMAKE_SOURCE_DIR}/include)

add_library(planner_grounding STATIC src/grounding.cpp)
target_link_libraries(planner_grounding PUBLIC planner_parser)
target_include_directories(planner_grounding PUBLIC ${CMAKE_SOURCE_DIR}/include)

add_library(planner_strips STATIC src/strips.cpp)
target_link_libraries(planner_strips PUBLIC planner_grounding)
target_include_directories(planner_strips PUBLIC ${CMAKE_SOURCE_DIR}/include)

add_library(planner_search STATIC src/heuristic.cpp src/search.cpp)
target_link_libraries(planner_search PUBLIC planner_strips)
target_include_directories(planner_search PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_compile_definitions(planner_search PUBLIC USE_ROBIN_HOOD)

# SAS 形式のプランナ
add_library(sas_reader STATIC
    src/sas/sas_reader.cpp
)
target_include_directories(sas_reader PUBLIC ${CMAKE_SOURCE_DIR}/include)

add_library(planner_sas_lib STATIC
    src/sas/sas_heuristic.cpp
    src/sas/sas_search.cpp
)
target_link_libraries(planner_sas_lib PUBLIC sas_reader)
target_include_directories(planner_sas_lib PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_compile_definitions(planner_sas_lib PUBLIC USE_ROBIN_HOOD)

# --- 実行ファイル ---
# 自作プランナ
add_executable(planner src/main.cpp)
target_link_libraries(planner PRIVATE planner_search)

if (UNIX)
  target_link_libraries(planner PRIVATE pthread)
endif()

# SAS 流用プランナ
add_executable(planner_sas
    src/sas/main.cpp
)
target_link_libraries(planner_sas PRIVATE planner_sas_lib)

if (UNIX)
  target_link_libraries(planner_sas PRIVATE pthread)
endif()

# --- テストケース ---
add_executable(lexer_pair_test tests/lexer_pair_test.cpp)
target_link_libraries(lexer_pair_test PRIVATE planner_lexer)
target_include_directories(lexer_pair_test PRIVATE ${CMAKE_SOURCE_DIR}/include)

add_executable(parse_demo tests/parse_demo.cpp)
target_link_libraries(parse_demo PRIVATE planner_parser)
target_include_directories(parse_demo PRIVATE ${CMAKE_SOURCE_DIR}/include)

add_executable(grounding_test tests/grounding_test.cpp)
target_link_libraries(grounding_test PRIVATE planner_grounding)
target_include_directories(grounding_test PRIVATE ${CMAKE_SOURCE_DIR}/include)

add_executable(grounding_generic_test tests/grounding_generic_test.cpp)
target_link_libraries(grounding_generic_test PRIVATE planner_grounding)
target_include_directories(grounding_generic_test PRIVATE ${CMAKE_SOURCE_DIR}/include)

add_executable(strips_test tests/strips_test.cpp)
target_link_libraries(strips_test PRIVATE planner_strips)
target_include_directories(strips_test PRIVATE ${CMAKE_SOURCE_DIR}/include)

add_executable(sas_reader_test tests/sas_reader_test.cpp)
target_link_libraries(sas_reader_test PRIVATE sas_reader)
target_include_directories(sas_reader_test PRIVATE ${CMAKE_SOURCE_DIR}/include)