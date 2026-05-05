# Hospital Domain Programming Project Skeleton — Java

This zip contains a Java skeleton for a project client that is intended to be much more advanced than the warmup assignment.

## What is included

```text
searchclient_java/searchclient/
  Action.java
  Color.java
  Frontier.java
  GraphSearch.java
  Heuristic.java
  Memory.java
  NotImplementedException.java
  SearchClient.java
  State.java

  ProjectSolver.java
  LevelAnalyzer.java
  Position.java
  Task.java
  TaskManager.java
  SingleAgentPlanner.java
  LowLevelState.java
  LowLevelNode.java
  ReservationTable.java
  Edge.java
  AgentPlan.java
  MultiAgentCoordinator.java
  DeadlockDetector.java
```

## Main idea

The warmup client searches directly in the full joint-state space. This skeleton adds a hierarchical project architecture:

1. `LevelAnalyzer` preprocesses walls, distances, and simple dead squares.
2. `TaskManager` turns the level into box-goal-agent tasks.
3. `SingleAgentPlanner` is where you implement reduced weighted A* for one agent and one box.
4. `ReservationTable` is where you reserve cells/edges over time for multi-agent coordination.
5. `MultiAgentCoordinator` converts individual plans into MAvis joint actions.
6. `ProjectSolver` wires everything together and currently falls back to global Weighted A* until the low-level planner is implemented.

## Compile

From the `searchclient_java` folder:

```bash
javac searchclient/*.java
```

## Run with the server

Use the project mode:

```bash
java -jar server.jar -c "java -Xmx8g searchclient.SearchClient -project" -l path/to/level.lvl -t 180 -g
```

You can still run the warmup strategies:

```bash
java -jar server.jar -c "java -Xmx8g searchclient.SearchClient -wastar 5" -l path/to/level.lvl -t 180 -g
java -jar server.jar -c "java -Xmx8g searchclient.SearchClient -greedy" -l path/to/level.lvl -t 180 -g
```

## Important TODOs

Implement these in this order:

1. `SingleAgentPlanner.expand(...)`
   - Generate valid Move/Push/Pull transitions for the assigned agent and assigned box.
   - Treat walls, other boxes, and reserved cells as obstacles.

2. `SingleAgentPlanner.planBoxToGoal(...)`
   - Call the included `weightedAStar(...)` method instead of returning `null`.

3. `ReservationTable.reserve(AgentPlan plan)`
   - Simulate positions over time and reserve cells/edges.

4. `MultiAgentCoordinator.coordinate(...)`
   - Simulate joint actions and insert NoOps or trigger replanning when conflicts occur.

5. `TaskManager.orderTasks(...)`
   - Improve task ordering using dependencies, corridors, solved boxes, and blocking boxes.

6. `DeadlockDetector`
   - Add 2x2 deadlocks, wall deadlocks, and tunnel deadlocks.

## Notes

`SearchClient.java` has been patched with a `-project` mode and a corrected joint-action print method using `|` between agents.
