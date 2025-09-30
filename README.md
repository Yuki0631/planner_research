# planner_research
## What is this planner?
The planner implemented by the owner, Yuki Suzuki.  <br>
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