package searchclient;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;

public final class LocalGroupPlanner {
    private static final int WEIGHT = 7;
    private static final int MAX_EXPANSIONS = 500_000;

    private final LevelAnalyzer analyzer;

    public LocalGroupPlanner(LevelAnalyzer analyzer) {
        this.analyzer = analyzer;
    }

    public List<Action> planGroup(State globalState, GoalGroup group) {
        LocalGroupState start = LocalGroupState.from(globalState, group);
        int maxExpansions = maxExpansionsFor(globalState, group);

        if (start.agentPos == null) {
            return null;
        }

        for (Position boxPos : start.boxPositions) {
            if (boxPos == null) {
                return null;
            }
        }

        PriorityQueue<LocalGroupNode> open = new PriorityQueue<>();
        HashSet<LocalGroupState> closed = new HashSet<>();

        int h = heuristic(start, group);
        if (h >= LevelAnalyzer.INF) {
            return null;
        }

        open.add(new LocalGroupNode(start, null, null, 0, WEIGHT * h));

        int expansions = 0;

        while (!open.isEmpty()) {
            if (++expansions > maxExpansions) {
                System.err.format(
                        "LocalGroupPlanner hit expansion limit for %s. Open size: %,d.%n",
                        group,
                        open.size()
                );
                return null;
            }

            LocalGroupNode node = open.poll();

            if (isGoal(node.state, group)) {
                System.err.format(
                        "LocalGroupPlanner solved %s with %,d expansions and %,d actions.%n",
                        group,
                        expansions,
                        node.g
                );
                return extractPlan(node);
            }

            if (!closed.add(node.state)) {
                continue;
            }

            List<Transition> transitions = expand(node.state, globalState, group);
            transitions.sort(Comparator.comparingInt(t -> heuristic(t.next, group)));

            for (Transition transition : transitions) {
                if (closed.contains(transition.next)) {
                    continue;
                }

                int nextH = heuristic(transition.next, group);
                if (nextH >= LevelAnalyzer.INF) {
                    continue;
                }

                int g = node.g + 1;
                int f = g + WEIGHT * nextH;

                open.add(new LocalGroupNode(transition.next, node, transition.action, g, f));
            }
        }

        System.err.format(
                "LocalGroupPlanner failed for %s after %,d expansions. Open size: %,d.%n",
                group,
                expansions,
                open.size()
        );
        return null;
    }

    private int maxExpansionsFor(State globalState, GoalGroup group) {
        int boardArea = State.walls.length * State.walls[0].length;

        if (boardArea > 900 && group.size() <= 3) {
            return 60_000;
        }

        if (boardArea > 1_800 && group.size() >= 8) {
            return 120_000;
        }

        return MAX_EXPANSIONS;
    }

    private boolean isGoal(LocalGroupState state, GoalGroup group) {
        boolean[] matchedGoals = new boolean[group.goals.size()];

        for (int boxIndex = 0; boxIndex < state.boxPositions.length; boxIndex++) {
            int goalIndex = matchingGoalIndex(state.boxPositions[boxIndex], group.letters.get(boxIndex), group, matchedGoals);

            if (goalIndex == -1) {
                return false;
            }

            matchedGoals[goalIndex] = true;
        }

        return true;
    }

    private int heuristic(LocalGroupState state, GoalGroup group) {
        boolean[] usedGoals = new boolean[group.goals.size()];
        int boxCost = assignBoxGoalCost(state, group, usedGoals, 0);

        if (boxCost >= LevelAnalyzer.INF) {
            return LevelAnalyzer.INF;
        }

        int unsolved = 0;
        int bestAgentToUsefulBox = LevelAnalyzer.INF;

        for (int i = 0; i < state.boxPositions.length; i++) {
            Position box = state.boxPositions[i];

            if (matchingGoalIndex(box, group.letters.get(i), group, new boolean[group.goals.size()]) == -1) {
                unsolved++;
            }

            int agentD = analyzer.distance(state.agentPos, box);
            bestAgentToUsefulBox = Math.min(bestAgentToUsefulBox, agentD);
        }

        if (bestAgentToUsefulBox >= LevelAnalyzer.INF) {
            return LevelAnalyzer.INF;
        }

        int total = 20 * boxCost + bestAgentToUsefulBox + 100 * unsolved;

        return total;
    }

    private int assignBoxGoalCost(LocalGroupState state, GoalGroup group, boolean[] usedGoals, int boxIndex) {
        if (boxIndex == state.boxPositions.length) {
            return 0;
        }

        int best = LevelAnalyzer.INF;
        char letter = group.letters.get(boxIndex);
        Position box = state.boxPositions[boxIndex];

        for (int goalIndex = 0; goalIndex < group.goals.size(); goalIndex++) {
            if (usedGoals[goalIndex] || group.letters.get(goalIndex) != letter) {
                continue;
            }

            int distance = analyzer.distance(box, group.goals.get(goalIndex));

            if (distance >= LevelAnalyzer.INF) {
                continue;
            }

            usedGoals[goalIndex] = true;
            int rest = assignBoxGoalCost(state, group, usedGoals, boxIndex + 1);
            usedGoals[goalIndex] = false;

            if (rest < LevelAnalyzer.INF) {
                best = Math.min(best, distance + rest);
            }
        }

        return best;
    }

    private int matchingGoalIndex(Position box, char letter, GoalGroup group, boolean[] usedGoals) {
        for (int goalIndex = 0; goalIndex < group.goals.size(); goalIndex++) {
            if (!usedGoals[goalIndex]
                    && group.letters.get(goalIndex) == letter
                    && group.goals.get(goalIndex).equals(box)) {
                return goalIndex;
            }
        }

        return -1;
    }

    private boolean badBoxMove(Position oldBox, Position newBox, Position goal) {
        int oldDistance = analyzer.distance(oldBox, goal);
        int newDistance = analyzer.distance(newBox, goal);

        if (oldDistance >= LevelAnalyzer.INF || newDistance >= LevelAnalyzer.INF) {
            return true;
        }

        if (newDistance <= oldDistance) {
            return false;
        }

        return newDistance > oldDistance + 1;
    }

    private List<Transition> expand(LocalGroupState state, State globalState, GoalGroup group) {
        ArrayList<Transition> result = new ArrayList<>();

        for (Action action : Action.values()) {
            if (action.type == ActionType.NoOp) {
                continue;
            }

            LocalGroupState next = applyIfPossible(state, globalState, group, action);

            if (next != null) {
                result.add(new Transition(next, action));
            }
        }

        return result;
    }

    private LocalGroupState applyIfPossible(
            LocalGroupState state,
            State globalState,
            GoalGroup group,
            Action action
    ) {
        switch (action.type) {
            case Move:
                return tryMove(state, globalState, group, action);

            case Push:
                return tryPush(state, globalState, group, action);

            case Pull:
                return tryPull(state, globalState, group, action);

            default:
                return null;
        }
    }

    private LocalGroupState tryMove(
            LocalGroupState state,
            State globalState,
            GoalGroup group,
            Action action
    ) {
        Position newAgent = state.agentPos.translate(action.agentRowDelta, action.agentColDelta);

        if (!cellFreeForAgent(newAgent, state, globalState, group)) {
            return null;
        }

        return state.with(newAgent, state.boxPositions);
    }

    private LocalGroupState tryPush(
            LocalGroupState state,
            State globalState,
            GoalGroup group,
            Action action
    ) {
        Position expectedBox = state.agentPos.translate(action.agentRowDelta, action.agentColDelta);
        int boxIndex = state.boxIndexAt(expectedBox);

        if (boxIndex == -1) {
            return null;
        }

        if (action.agentRowDelta + action.boxRowDelta == 0 &&
            action.agentColDelta + action.boxColDelta == 0) {
            return null;
        }

        Position newBox = expectedBox.translate(action.boxRowDelta, action.boxColDelta);

        if (!insideGroupArea(newBox, group)) {
            return null;
        }

        if (!cellFreeForBox(newBox, state, globalState, group, boxIndex)) {
            return null;
        }

        if (badGroupBoxMove(expectedBox, newBox, group.letters.get(boxIndex), group)) {
            return null;
        }

        Position[] newBoxes = state.boxPositions.clone();
        newBoxes[boxIndex] = newBox;

        Position newAgent = expectedBox;

        return state.with(newAgent, newBoxes);
    }

    private LocalGroupState tryPull(
            LocalGroupState state,
            State globalState,
            GoalGroup group,
            Action action
    ) {
        Position newAgent = state.agentPos.translate(action.agentRowDelta, action.agentColDelta);

        if (!cellFreeForAgent(newAgent, state, globalState, group)) {
            return null;
        }

        Position expectedBox = state.agentPos.translate(-action.boxRowDelta, -action.boxColDelta);
        int boxIndex = state.boxIndexAt(expectedBox);

        if (boxIndex == -1) {
            return null;
        }

        Position newBox = state.agentPos;

        if (!insideGroupArea(newBox, group)) {
            return null;
        }

        if (badGroupBoxMove(expectedBox, newBox, group.letters.get(boxIndex), group)) {
            return null;
        }

        Position[] newBoxes = state.boxPositions.clone();
        newBoxes[boxIndex] = newBox;

        return state.with(newAgent, newBoxes);
    }

private boolean insideGroupArea(Position p, GoalGroup group) {
    int minRow = Integer.MAX_VALUE;
    int maxRow = Integer.MIN_VALUE;
    int minCol = Integer.MAX_VALUE;
    int maxCol = Integer.MIN_VALUE;

    for (Position goal : group.goals) {
        minRow = Math.min(minRow, goal.row);
        maxRow = Math.max(maxRow, goal.row);
        minCol = Math.min(minCol, goal.col);
        maxCol = Math.max(maxCol, goal.col);
    }

    /*
     * The first local-area version was too restrictive:
     * if the assigned agent starts outside the goal corridor area, no first move is possible.
     *
     * This version keeps the search focused around the goal group but gives the agent enough
     * room to approach the boxes from below and from the sides.
     */
    int rowMarginUp = 1;
    int rowMarginDown = 12;
    int colMargin = 8;

    int allowedMinRow = Math.max(0, minRow - rowMarginUp);
    int allowedMaxRow = Math.min(analyzer.rows - 1, maxRow + rowMarginDown);
    int allowedMinCol = Math.max(0, minCol - colMargin);
    int allowedMaxCol = Math.min(analyzer.cols - 1, maxCol + colMargin);

    return p.row >= allowedMinRow
            && p.row <= allowedMaxRow
            && p.col >= allowedMinCol
            && p.col <= allowedMaxCol;
}

    private boolean badGroupBoxMove(Position oldBox, Position newBox, char letter, GoalGroup group) {
        if (matchingGoalIndex(newBox, letter, group, new boolean[group.goals.size()]) != -1) {
            return false;
        }

        if (analyzer.isDeadSquare(newBox, letter)) {
            return true;
        }

        int oldDistance = nearestMatchingGoalDistance(oldBox, letter, group);
        int newDistance = nearestMatchingGoalDistance(newBox, letter, group);

        if (oldDistance >= LevelAnalyzer.INF || newDistance >= LevelAnalyzer.INF) {
            return true;
        }

        return newDistance > oldDistance + 1;
    }

    private int nearestMatchingGoalDistance(Position box, char letter, GoalGroup group) {
        int best = LevelAnalyzer.INF;

        for (int i = 0; i < group.goals.size(); i++) {
            if (group.letters.get(i) != letter) {
                continue;
            }

            best = Math.min(best, analyzer.distance(box, group.goals.get(i)));
        }

        return best;
    }

    private boolean cellFreeForAgent(
            Position p,
            LocalGroupState localState,
            State globalState,
            GoalGroup group
    ) {
        if (!analyzer.inBounds(p)) {
            return false;
        }

        if (State.walls[p.row][p.col]) {
            return false;
        }

        if (localState.boxIndexAt(p) != -1) {
            return false;
        }

        if (staticBoxObstacle(p, globalState, group)) {
            return false;
        }

        if (occupiedByOtherAgent(p, globalState, group.assignedAgent)) {
            return false;
        }

        return true;
    }

    private boolean cellFreeForBox(
            Position p,
            LocalGroupState localState,
            State globalState,
            GoalGroup group,
            int movingBoxIndex
    ) {
        if (!analyzer.inBounds(p)) {
            return false;
        }

        if (State.walls[p.row][p.col]) {
            return false;
        }

        for (int i = 0; i < localState.boxPositions.length; i++) {
            if (i == movingBoxIndex) {
                continue;
            }

            if (localState.boxPositions[i].equals(p)) {
                return false;
            }
        }

        if (staticBoxObstacle(p, globalState, group)) {
            return false;
        }

        if (occupiedByOtherAgent(p, globalState, group.assignedAgent)) {
            return false;
        }

        return true;
    }

    private boolean staticBoxObstacle(Position p, State globalState, GoalGroup group) {
        char box = globalState.boxes[p.row][p.col];

        if (box == 0) {
            return false;
        }

        for (char letter : group.letters) {
            if (box == letter) {
                return false;
            }
        }

        return true;
    }

    private boolean occupiedByOtherAgent(Position p, State globalState, int activeAgent) {
        for (int agent = 0; agent < globalState.agentRows.length; agent++) {
            if (agent == activeAgent) {
                continue;
            }

            if (globalState.agentRows[agent] == p.row && globalState.agentCols[agent] == p.col) {
                return true;
            }
        }

        return false;
    }

    private List<Action> extractPlan(LocalGroupNode node) {
        LinkedList<Action> result = new LinkedList<>();

        while (node != null && node.action != null) {
            result.addFirst(node.action);
            node = node.parent;
        }

        return result;
    }

    private static final class Transition {
        final LocalGroupState next;
        final Action action;

        Transition(LocalGroupState next, Action action) {
            this.next = next;
            this.action = action;
        }
    }
}
