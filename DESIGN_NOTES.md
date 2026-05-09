# Hybrid Multi-Agent Hospital Solver

## Overview

This solver is a deliberately hybrid planner for the hospital domain. The domain combines
multi-agent path finding with Sokoban-style box manipulation, so one pure method is not
practical under the three-minute competition limit. The implementation combines:

- topology analysis of the level,
- box-goal task decomposition,
- heuristic task ordering,
- weighted A* / best-first low-level planning,
- bounded CBS-style coordination,
- general repair and fallback searches,
- strict final simulation and verification before returning a plan.

The submitted version avoids benchmark-name macros and exact coordinate scripts. Repairs are
kept only when they are expressed as general structural mechanisms such as compact goal rooms,
boundary pockets, wrong-goal blocker clearing, small endgames, or final move-only agent cleanup.

## Solver Pipeline

`ProjectSolver.solve` runs a verified portfolio. Each strategy builds a candidate plan, simulates
it from the original initial state, and only complete valid plans may be returned.

The main strategies are:

1. CBS-first hybrid: attempt a bounded Box-CBS prefix, then use the heuristic fallback.
2. Heuristic fallback from the initial state.
3. Conservative CBS-assisted fallback: only try small, local, low-conflict box tasks first.
4. Small CBS-prefix hybrid: use a shorter CBS prefix before fallback.
5. Fallback-first final-agent CBS: the fallback attempts CBS for the final agent-goal suffix when all box goals are solved.

The final returned solution is the shortest verified complete candidate. If no candidate verifies,
the solver returns an empty plan rather than a partial or invalid plan.

## CBS Components

`CBSPlanner` implements standard move-only MAPF CBS for final agent-goal cleanup:

- high-level nodes store one path per agent and a set of constraints,
- low-level A* plans one agent at a time over row, column, and time,
- constraints forbid vertex and edge conflicts,
- conflicts are detected between agents,
- children are created by adding one constraint and replanning the affected agent.

Boxes are treated as static obstacles in this phase. This CBS is only used when all box goals are
already solved, because it is a move-only planner.

`BoxCBSPlanner` is a bounded CBS-style coordinator for selected box tasks. It is not a full
optimal CBS solver over the complete multi-agent Sokoban state space. Instead, it coordinates
one selected box task at a time:

- one color-compatible agent may move the target box with Move, Push, Pull, and NoOp,
- other agents may Move or NoOp to avoid conflicts,
- non-target boxes are static obstacles,
- constraints can restrict agent positions, agent edges, box positions, and box movement,
- detected conflicts cause high-level branching and replanning of one constrained agent.

This makes CBS a real coordination mechanism while keeping the low-level search small enough to
be useful within the time limit.

## Search And Heuristics

The box-moving part is mainly handled by task decomposition plus low-level search:

- `TaskManager` creates box-goal tasks from the current state.
- Tasks are ordered by distance, compatible agents, boundary/dead-end pressure, cluster pressure,
  duplicate-letter assignment quality, and solved-goal protection.
- `SingleAgentPlanner` uses best-first / weighted-A*-style planning for moving one selected box
  to one selected goal.
- Repair planners use bounded state-space search for compact rooms, boundary pockets, wrong-goal
  blockers, and small endgames.

The solver protects correctness by simulating committed actions and by rejecting candidate
suffixes that do not improve solved goals or that permanently regress solved box goals.

## Why Not Pure Full CBS?

Full optimal CBS for the complete hospital problem would need to reason over all agents, all
movable boxes, all push/pull interactions, and time at once. Each box movement changes the
world for every other agent, which makes the low-level search much closer to multi-agent Sokoban
than ordinary MAPF. That is too expensive for the competition time limit on many levels.

The hybrid approach is more practical: CBS handles selected coordination problems, while
heuristic task planning and bounded repairs handle the larger box-moving structure.

## Verification

Before `ProjectSolver.solve` returns any non-empty plan, the complete joint plan is simulated
from the original initial state. Verification checks:

- the joint action has one action per agent,
- no action is null,
- every joint action is applicable,
- no joint action is conflicting,
- the final state satisfies all goals.

Partial plans are rejected, even if they solve many goals.

## Limitations

- `BoxCBSPlanner` is bounded and task-level, not globally optimal.
- The heuristic fallback often solves more levels than the CBS prefix.
- Some general repair methods are still engineering heuristics rather than complete planners.
- Very large box clusters, narrow dependency chains, and difficult duplicate-letter rooms can
  still fail within the time limit.
