# planner_research
## 1. What is this planner?
The planner implemented by the owner, Yuki Suzuki.

The planner has  two types. One is of the STRIPS type and the other one is of the FDR type. 

The FDR type planner uses fast-downward translator, but other functions are implemented by the owner. 

Some search algorithms and heuristic functions will be added later. 

**ff heuristic and landmark heuristic** is introduce. (2025/12).

**canonical bi-directional search algorithm** is introduced. (2025/12/03).

It is assumed that this planner is used in Linux environment. 


## 2. Parallel Search Planner (parallel_SOC)
Parallel search system is under construct. 
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
│   │   ├── bi_search.hpp
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
│   │   ├── bi_search.cpp
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

9 directories, 55 files
```

This diagram omits pddl folder because it only includes test pddl files.

You can check your own clon directory structure via a bash code like the following one.

```{bash}
tree -L 4 -I build -I pddl
```


## 4. How to use this planner
1. Clone this repository. 
2. Create a build directory directly under the planner_research folder.

    
```{bash}
cd planner_research
mkdir build
 ```

3. Build the project with CMake. 

```{bash}
cd build
cmake .. && cmake --build . -j
```
    

4. You can execute this planner after building it. The basic bash commands are described in the following sections.

4.1 If you would like to do planning **from parsing/grounding to search**, please enter this command.  

```{bash}
./planner <domain.pddl> <problem.pddl> [--algo astar|gbfs] [--h blind|goalcount|wgoalcount W] [--plan-out <DIR>] 
```


4.2 If you would like to use a **FDR type** planner, please enter this command. 

```{bash}
./planner_sas <domain.pddl> <problem.pddl> [--algo astar|gbfs|bi_search] [--search-cpu-limit int(second)] [--search-mem-limit-mb int(MB)] [--fd <fast-downward.sif>] [--sas-file <DIR>] [--h goalcount|blind] [--keep-sas] [--plan-out <DIR>] [--check-mutex on|off|auto] [--val <validate>] [--val-args]
```


4.3 If you would like to use **parallel type** planner, please enter this command.

```{bash}
./planner_sas <domain.pddl> <problem.pddl> [--algo soc_astar] [--search-cpu-limit int(second)] [--search-mem-limit-mb int(MB)] [--fd <fast-downward.sif>] [--sas-file <DIR>] [--h goalcount|blind] [--keep-sas] [--plan-out <DIR>] [--check-mutex on|off|auto] [--val <validate>] [--val-args] [--soc-threads N] [--soc-open multi|bucket] [--soc-queues Q] [--soc-k K]
```


