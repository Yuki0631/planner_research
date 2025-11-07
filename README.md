# planner_research
## 1. What is this planner?
The planner implemented by the owner, Yuki Suzuki.  <br>
The planner has  two types. One is of the STRIPS type and the other one is of the FDR type. <br>
The FDR type planner uses fast-downward translator, but other functions are implemented by the owner. <br>
Some search algorithms and heuristic functions will be added later. <br>
It is assumed that this planner is used in Linux environment. <br>
<br>

## 2. Parallel Search Planner (parallel_SOC)
Parallel search system is under construct. <br>
You can check my source code via "include/sas/parallel_SOC" or "src/sas/parallel_SOC".

## 3. Directory Structure
The directory structure is shown in the following tree diagram. 

```{tree}
.
├── CMakeLists.txt
├── LICENSE
├── README.md
├── include
│   ├── bucket_pq.hpp
│   ├── grounding.hpp
│   ├── heuristic.hpp
│   ├── lexer.hpp
│   ├── parser.hpp
│   ├── robin_hood.h
│   ├── sas
│   │   ├── parallel_SOC
│   │   │   ├── closed_table.hpp
│   │   │   ├── concurrency.hpp
│   │   │   ├── expander.hpp
│   │   │   ├── heuristic_adapter.hpp
│   │   │   ├── id_allocator.hpp
│   │   │   ├── node.hpp
│   │   │   ├── parallel_search.hpp
│   │   │   ├── parallel_soc_all.hpp
│   │   │   ├── params.hpp
│   │   │   ├── shared_open_list.hpp
│   │   │   ├── state_hasher.hpp
│   │   │   ├── state_store.hpp
│   │   │   ├── stats.hpp
│   │   │   ├── termination.hpp
│   │   │   └── thread_pool.hpp
│   │   ├── sas_heuristic.hpp
│   │   ├── sas_reader.hpp
│   │   └── sas_search.hpp
│   ├── search.hpp
│   └── strips.hpp
├── sas
│   └── test.sas
├── src
│   ├── grounding.cpp
│   ├── heuristic.cpp
│   ├── lexer.cpp
│   ├── main.cpp
│   ├── parser.cpp
│   ├── sas
│   │   ├── main.cpp
│   │   ├── parallel_SOC
│   │   │   ├── closed_table.cpp
│   │   │   ├── expander.cpp
│   │   │   ├── parallel_search.cpp
│   │   │   ├── shared_open_list.cpp
│   │   │   ├── termination.cpp
│   │   │   └── thread_pool.cpp
│   │   ├── sas_heuristic.cpp
│   │   ├── sas_reader.cpp
│   │   └── sas_search.cpp
│   ├── search.cpp
│   └── strips.cpp
└── tests
    ├── grounding_generic_test.cpp
    ├── grounding_test.cpp
    ├── lexer_pair_test.cpp
    ├── parse_demo.cpp
    ├── sas_reader_test.cpp
    └── strips_test.cpp

9 directories, 53 files
```

This diagram omits pddl folder because it only includes test pddl files. <br>
You can check your own clon directory structure via a bash code like the following one.

```{bash}
tree -L 4 -I build -I pddl
```


## 4. How to use this planner
1. Clone this repository. <br>
2. Create a build directory directly under the planner_research folder. <br>
    
```{bash}
cd planner_research
mkdir build
 ```
<br>
3. Build the project with CMake. <br>

```{bash}
cd build
cmake .. && cmake --build . -j
```
    
<br>
4. Execute the project in your build directory. <br>

```{bash}
./planner <domain.pddl> <problem.pddl> [--algo astar|gbfs] "<< "[--h blind|goalcount|wgoalcount W] [--plan-dir <DIR>]
```

<br>
4.1 If you would like to use FDR type planner, please enter this command. <br>

```{bash}
./planner_sas <domain.pddl> <problem.pddl> [--algo astar|gbfs] [--fd <fast-downward.sif>] [--sas-file <DIR>] [--h goalcount|blind] [--keep-sas] [--plan-out <DIR>] [--check-mutex on|off|auto] [--val <validate>] [--val-args]
```

<br>