# planner_research
## What is this planner?
The planner implemented by the owner, Yuki Suzuki.  <br>
The planner has  two types. One is of the STRIPS type and the other one is of the FDR type. <br>
The FDR type planner uses fast-downward translator, but other functions are implemented by the owner. <br>
Some search algorithms and heuristic functions will be added later. <br>
<br>

## How to use this planner
1. Clone this repository. <br>
2. Create a build directory directly under the planner_research folder. <br>
    
    ```{bash}
    cd planner_research
    mkdir build
    ```
<br>
3. Build the project with CMake. <br>

    cd build
    cmake .. && cmake --build . -j
    
<br>
4. Execute the project in your build directory. <br>

    ./planner <domain.pddl> <problem.pddl> [--algo astar|gbfs] "<< "[--h blind|goalcount|wgoalcount W] [--plan-dir <DIR>]

<br>
4.1 If you would like to use FDR type planner, please enter this command. <br>

    ./planner_sas <domain.pddl> <problem.pddl> [--algo astar|gbfs] [--fd <fast-downward.sif>] [--sas-file <DIR>] [--h goalcount|blind] [--keep-sas] [--plan-out <DIR>] [--check-mutex on|off|auto] [--val <validate>] [--val-args]

<br>