package searchclient;

import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;

public final class AgentGoalPlanner {

    public List<Action> planAgentToGoal(State state, int agent) {
        Position goal = findAgentGoal(agent);

        if (goal == null) {
            return null;
        }

        Position start = new Position(state.agentRows[agent], state.agentCols[agent]);

        if (start.equals(goal)) {
            return new LinkedList<>();
        }

        ArrayDeque<Node> queue = new ArrayDeque<>();
        HashSet<Position> visited = new HashSet<>();

        queue.add(new Node(start, null, null));
        visited.add(start);

        while (!queue.isEmpty()) {
            Node node = queue.poll();

            if (node.position.equals(goal)) {
                return extractPlan(node);
            }

            for (Action action : moveActions()) {
                Position next = node.position.translate(action.agentRowDelta, action.agentColDelta);

                if (!cellFreeForAgent(state, next, agent)) {
                    continue;
                }

                if (!visited.add(next)) {
                    continue;
                }

                queue.add(new Node(next, node, action));
            }
        }

        return null;
    }

    private Position findAgentGoal(int agent) {
        char goalChar = (char) ('0' + agent);

        for (int r = 0; r < State.goals.length; r++) {
            for (int c = 0; c < State.goals[r].length; c++) {
                if (State.goals[r][c] == goalChar) {
                    return new Position(r, c);
                }
            }
        }

        return null;
    }

    private boolean cellFreeForAgent(State state, Position p, int activeAgent) {
        if (p.row < 0 || p.row >= State.walls.length ||
            p.col < 0 || p.col >= State.walls[0].length) {
            return false;
        }

        if (State.walls[p.row][p.col]) {
            return false;
        }

        if (state.boxes[p.row][p.col] != 0) {
            return false;
        }

        for (int agent = 0; agent < state.agentRows.length; agent++) {
            if (agent == activeAgent) {
                continue;
            }

            if (state.agentRows[agent] == p.row && state.agentCols[agent] == p.col) {
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

        Node(Position position, Node parent, Action action) {
            this.position = position;
            this.parent = parent;
            this.action = action;
        }
    }
}