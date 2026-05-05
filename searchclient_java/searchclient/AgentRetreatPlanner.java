package searchclient;

import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;

/**
 * Moves an agent away after it has completed a box-goal task.
 *
 * Why this matters:
 * After pushing/pulling a box onto a goal, the active agent is often left in a doorway,
 * corridor, or push-position cell. If the agent stays there, later tasks may become
 * impossible because other agents/boxes treat it as an obstacle.
 *
 * This retreat planner performs a bounded BFS over Move actions only. It searches for
 * a safe parking cell that:
 * - is not a wall
 * - is not occupied by a box
 * - is not occupied by another agent
 * - is not a goal cell
 * - is not near unsolved box goals
 * - is not in the top goal corridor area
 * - is preferably far away from important goal/box traffic
 */
public final class AgentRetreatPlanner {
    private final LevelAnalyzer analyzer;

    public AgentRetreatPlanner(LevelAnalyzer analyzer) {
        this.analyzer = analyzer;
    }

    public List<Action> planRetreat(State state, int agent, int maxDepth) {
        Position start = new Position(state.agentRows[agent], state.agentCols[agent]);

        ArrayDeque<Node> queue = new ArrayDeque<>();
        HashSet<Position> visited = new HashSet<>();

        queue.add(new Node(start, null, null, 0));
        visited.add(start);

        Node bestFallback = null;
        int bestFallbackScore = Integer.MIN_VALUE;

        while (!queue.isEmpty()) {
            Node node = queue.poll();

            if (node.depth > 0) {
                int score = parkingScore(state, node.position, agent);

                if (score > bestFallbackScore) {
                    bestFallbackScore = score;
                    bestFallback = node;
                }

                if (isGoodParkingCell(state, node.position, agent)) {
                    return extractPlan(node);
                }
            }

            if (node.depth >= maxDepth) {
                continue;
            }

            for (Action action : moveActions()) {
                Position next = node.position.translate(action.agentRowDelta, action.agentColDelta);

                if (!cellFreeForAgent(state, next, agent)) {
                    continue;
                }

                if (!visited.add(next)) {
                    continue;
                }

                queue.add(new Node(next, node, action, node.depth + 1));
            }
        }

        // If no ideal parking cell exists, still move to the best reachable fallback.
        // This is usually better than staying directly next to a solved box/goal.
        if (bestFallback != null && bestFallbackScore > Integer.MIN_VALUE / 2) {
            return extractPlan(bestFallback);
        }

        return Collections.emptyList();
    }

    private boolean isGoodParkingCell(State state, Position p, int agent) {
        if (!cellFreeForAgent(state, p, agent)) {
            return false;
        }

        // Important for help.lvl and similar goal-row levels:
        // Do not park inside the top goal corridor or directly below it.
        if (p.row <= 2) {
            return false;
        }

        // Do not park directly on any goal cell.
        if (State.goals[p.row][p.col] != 0) {
            return false;
        }

        // Avoid parking adjacent to unsolved box goals.
        if (adjacentToUnsolvedGoal(state, p)) {
            return false;
        }

        // Avoid parking adjacent to boxes, because these cells are often needed
        // as push/pull positions for future tasks.
        if (adjacentToBox(state, p)) {
            return false;
        }

        // Avoid simple chokepoint-like cells when possible.
        // If a cell has only two free neighbors and they are opposite, it is a corridor.
        // Parking there often blocks traffic.
        if (isNarrowCorridorCell(state, p)) {
            return false;
        }

        return true;
    }

    /**
     * Higher score means better parking cell.
     * This score is used only as fallback if no perfect parking cell is found.
     */
    private int parkingScore(State state, Position p, int agent) {
        if (!cellFreeForAgent(state, p, agent)) {
            return Integer.MIN_VALUE / 2;
        }

        int score = 0;

        // Prefer not being in top corridor area.
        if (p.row <= 2) {
            score -= 10_000;
        }

        // Never prefer goal cells.
        if (State.goals[p.row][p.col] != 0) {
            score -= 5_000;
        }

        if (adjacentToUnsolvedGoal(state, p)) {
            score -= 2_000;
        }

        if (adjacentToBox(state, p)) {
            score -= 1_000;
        }

        if (isNarrowCorridorCell(state, p)) {
            score -= 500;
        }

        // Prefer cells farther away from unsolved goals.
        score += distanceToNearestUnsolvedGoal(state, p);

        // Prefer cells farther away from boxes.
        score += distanceToNearestBox(state, p);

        // Prefer lower/open areas slightly, useful for top goal-room structures.
        score += 2 * p.row;

        return score;
    }

    private boolean adjacentToUnsolvedGoal(State state, Position p) {
        for (Action move : moveActions()) {
            Position q = p.translate(move.agentRowDelta, move.agentColDelta);

            if (!analyzer.inBounds(q)) {
                continue;
            }

            char goal = State.goals[q.row][q.col];

            if ('A' <= goal && goal <= 'Z' && state.boxes[q.row][q.col] != goal) {
                return true;
            }
        }

        return false;
    }

    private boolean adjacentToBox(State state, Position p) {
        for (Action move : moveActions()) {
            Position q = p.translate(move.agentRowDelta, move.agentColDelta);

            if (!analyzer.inBounds(q)) {
                continue;
            }

            if (state.boxes[q.row][q.col] != 0) {
                return true;
            }
        }

        return false;
    }

    private boolean isNarrowCorridorCell(State state, Position p) {
        boolean north = isFreeStatic(p.row - 1, p.col);
        boolean south = isFreeStatic(p.row + 1, p.col);
        boolean west = isFreeStatic(p.row, p.col - 1);
        boolean east = isFreeStatic(p.row, p.col + 1);

        int freeCount = 0;
        if (north) freeCount++;
        if (south) freeCount++;
        if (west) freeCount++;
        if (east) freeCount++;

        if (freeCount <= 1) {
            return true;
        }

        // Straight narrow corridor.
        if (freeCount == 2) {
            if (north && south) return true;
            if (west && east) return true;
        }

        return false;
    }

    private boolean isFreeStatic(int row, int col) {
        return analyzer.inBounds(row, col) && !State.walls[row][col];
    }

    private int distanceToNearestUnsolvedGoal(State state, Position p) {
        int best = 100;

        for (int r = 0; r < State.goals.length; r++) {
            for (int c = 0; c < State.goals[r].length; c++) {
                char goal = State.goals[r][c];

                if ('A' <= goal && goal <= 'Z' && state.boxes[r][c] != goal) {
                    best = Math.min(best, p.manhattanDistance(new Position(r, c)));
                }
            }
        }

        return Math.min(best, 100);
    }

    private int distanceToNearestBox(State state, Position p) {
        int best = 100;

        for (int r = 0; r < state.boxes.length; r++) {
            for (int c = 0; c < state.boxes[r].length; c++) {
                if (state.boxes[r][c] != 0) {
                    best = Math.min(best, p.manhattanDistance(new Position(r, c)));
                }
            }
        }

        return Math.min(best, 100);
    }

    private boolean cellFreeForAgent(State state, Position p, int agent) {
        if (!analyzer.inBounds(p)) {
            return false;
        }

        if (State.walls[p.row][p.col]) {
            return false;
        }

        if (state.boxes[p.row][p.col] != 0) {
            return false;
        }

        for (int other = 0; other < state.agentRows.length; other++) {
            if (other == agent) {
                continue;
            }

            if (state.agentRows[other] == p.row && state.agentCols[other] == p.col) {
                return false;
            }
        }

        return true;
    }

    private List<Action> extractPlan(Node node) {
        LinkedList<Action> result = new LinkedList<>();

        while (node != null && node.action != null) {
            result.addFirst(node.action);
            node = node.parent;
        }

        return result;
    }

    private List<Action> moveActions() {
        return Arrays.asList(
                Action.MoveN,
                Action.MoveS,
                Action.MoveE,
                Action.MoveW
        );
    }

    private static final class Node {
        final Position position;
        final Node parent;
        final Action action;
        final int depth;

        Node(Position position, Node parent, Action action, int depth) {
            this.position = position;
            this.parent = parent;
            this.action = action;
            this.depth = depth;
        }
    }
}