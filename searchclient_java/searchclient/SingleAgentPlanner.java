package searchclient;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;

public final class SingleAgentPlanner {
    private final LevelAnalyzer analyzer;
    private static final int DEFAULT_WEIGHT = 5;
    private static final int MAX_EXPANSIONS = 200_000;

    public SingleAgentPlanner(LevelAnalyzer analyzer) {
        this.analyzer = analyzer;
    }

public List<Action> planBoxToGoal(State globalState, Task task, ReservationTable reservations) {
    return weightedAStar(globalState, task, reservations, true);
}

public List<Action> planBoxToGoalRelaxed(State globalState, Task task, ReservationTable reservations) {
    return weightedAStar(globalState, task, reservations, false);
}

    private List<Action> weightedAStar(
        State globalState,
        Task task,
        ReservationTable reservations,
        boolean useDeadSquarePruning
) {
        PriorityQueue<LowLevelNode> open = new PriorityQueue<>();
        HashSet<LowLevelState> closed = new HashSet<>();

        LowLevelState start = LowLevelState.from(globalState, task);
        int startH = heuristic(start, task);

        if (startH >= LevelAnalyzer.INF) {
            return null;
        }

        open.add(new LowLevelNode(start, null, null, 0, DEFAULT_WEIGHT * startH));

        int expansions = 0;

        while (!open.isEmpty()) {
            if (++expansions > MAX_EXPANSIONS) {
                return null;
            }

            LowLevelNode node = open.poll();

            if (node.state.boxPos.equals(task.goal)) {
                return extractPlan(node);
            }

            if (!closed.add(node.state)) {
                continue;
            }

            List<LowLevelTransition> transitions = expand(node.state, globalState, task, reservations);
            transitions.sort(Comparator.comparingInt(t -> heuristic(t.next, task)));

            for (LowLevelTransition transition : transitions) {
                if (closed.contains(transition.next)) {
                    continue;
                }

               if (useDeadSquarePruning && isDeadlockedBox(transition.next.boxPos, globalState, task)) {
    continue;
}

                int h = heuristic(transition.next, task);
                if (h >= LevelAnalyzer.INF) {
                    continue;
                }

                int g = node.g + 1;
                int f = g + DEFAULT_WEIGHT * h;

                open.add(new LowLevelNode(transition.next, node, transition.action, g, f));
            }
        }

        return null;
    }

    private int heuristic(LowLevelState s, Task task) {
        int boxToGoal = analyzer.distance(s.boxPos, task.goal);
        int agentToBox = analyzer.distance(s.agentPos, s.boxPos);

        if (boxToGoal >= LevelAnalyzer.INF || agentToBox >= LevelAnalyzer.INF) {
            return LevelAnalyzer.INF;
        }

        return 5 * boxToGoal + agentToBox;
    }

    private List<LowLevelTransition> expand(
            LowLevelState state,
            State globalState,
            Task task,
            ReservationTable reservations
    ) {
        ArrayList<LowLevelTransition> result = new ArrayList<>();

        for (Action action : Action.values()) {
            // For now, skip NoOp in the low-level planner.
            // Otherwise the planner can waste many states by waiting forever.
            if (action.type == ActionType.NoOp) {
                continue;
            }

            LowLevelState next = applyIfPossible(state, globalState, task, reservations, action);

            if (next != null) {
                result.add(new LowLevelTransition(next, action));
            }
        }

        return result;
    }

    private LowLevelState applyIfPossible(
            LowLevelState state,
            State globalState,
            Task task,
            ReservationTable reservations,
            Action action
    ) {
        Position agent = state.agentPos;
        Position box = state.boxPos;

        switch (action.type) {
            case Move:
                return tryMove(state, globalState, task, reservations, action, agent, box);

            case Push:
                return tryPush(state, globalState, task, reservations, action, agent, box);

            case Pull:
                return tryPull(state, globalState, task, reservations, action, agent, box);

            default:
                return null;
        }
    }

    private LowLevelState tryMove(
            LowLevelState state,
            State globalState,
            Task task,
            ReservationTable reservations,
            Action action,
            Position agent,
            Position box
    ) {
        Position newAgent = agent.translate(action.agentRowDelta, action.agentColDelta);

        if (!cellFreeForAgent(newAgent, box, globalState, task)) {
            return null;
        }

        if (!reservations.isFree(newAgent, state.time + 1)) {
            return null;
        }

        return new LowLevelState(newAgent, box, state.time + 1);
    }

    private LowLevelState tryPush(
            LowLevelState state,
            State globalState,
            Task task,
            ReservationTable reservations,
            Action action,
            Position agent,
            Position box
    ) {
        Position expectedBox = agent.translate(action.agentRowDelta, action.agentColDelta);

        if (!expectedBox.equals(box)) {
            return null;
        }

        // A push cannot move the box back into the agent's old cell.
        // Example: Push(E,W) is invalid.
        if (action.agentRowDelta + action.boxRowDelta == 0 &&
            action.agentColDelta + action.boxColDelta == 0) {
            return null;
        }

        Position newBox = box.translate(action.boxRowDelta, action.boxColDelta);

        if (!cellFreeForBox(newBox, globalState, task)) {
            return null;
        }

        if (!reservations.isFree(newBox, state.time + 1)) {
            return null;
        }

        Position newAgent = box;

        return new LowLevelState(newAgent, newBox, state.time + 1);
    }

    private LowLevelState tryPull(
            LowLevelState state,
            State globalState,
            Task task,
            ReservationTable reservations,
            Action action,
            Position agent,
            Position box
    ) {
        Position newAgent = agent.translate(action.agentRowDelta, action.agentColDelta);

        if (!cellFreeForAgent(newAgent, box, globalState, task)) {
            return null;
        }

        Position expectedBox = agent.translate(-action.boxRowDelta, -action.boxColDelta);

        if (!expectedBox.equals(box)) {
            return null;
        }

        if (!reservations.isFree(newAgent, state.time + 1)) {
            return null;
        }

        Position newBox = agent;

        return new LowLevelState(newAgent, newBox, state.time + 1);
    }

    private boolean cellFreeForAgent(Position p, Position activeBox, State globalState, Task task) {
        if (!analyzer.inBounds(p)) {
            return false;
        }

        if (State.walls[p.row][p.col]) {
            return false;
        }

        if (p.equals(activeBox)) {
            return false;
        }

        if (occupiedByOtherAgent(p, globalState, task.assignedAgent)) {
            return false;
        }

        char staticBox = globalState.boxes[p.row][p.col];

        // The active box started at task.boxStart, but in the low-level state
        // it is represented by activeBox. Therefore task.boxStart must be treated
        // as free after the active box has moved away.
        return staticBox == 0 || p.equals(task.boxStart);
    }

    private boolean cellFreeForBox(Position p, State globalState, Task task) {
        if (!analyzer.inBounds(p)) {
            return false;
        }

        if (State.walls[p.row][p.col]) {
            return false;
        }

        if (occupiedByOtherAgent(p, globalState, task.assignedAgent)) {
            return false;
        }

        char staticBox = globalState.boxes[p.row][p.col];

        // Same logic: the active box's original cell is not a static obstacle.
        return staticBox == 0 || p.equals(task.boxStart);
    }

    private boolean isDeadlockedBox(Position box, State globalState, Task task) {
        if (box.equals(task.goal)) {
            return false;
        }

        if (analyzer.isDeadSquare(box, task.boxLetter)) {
            return true;
        }

        return false;
    }

    private boolean occupiedByOtherAgent(Position p, State state, int activeAgent) {
        for (int agent = 0; agent < state.agentRows.length; agent++) {
            if (agent == activeAgent) {
                continue;
            }

            if (state.agentRows[agent] == p.row && state.agentCols[agent] == p.col) {
                return true;
            }
        }

        return false;
    }

    private List<Action> extractPlan(LowLevelNode node) {
        LinkedList<Action> result = new LinkedList<>();

        while (node != null && node.action != null) {
            result.addFirst(node.action);
            node = node.parent;
        }

        return result;
    }

    private static final class LowLevelTransition {
        final LowLevelState next;
        final Action action;

        LowLevelTransition(LowLevelState next, Action action) {
            this.next = next;
            this.action = action;
        }
    }
}
