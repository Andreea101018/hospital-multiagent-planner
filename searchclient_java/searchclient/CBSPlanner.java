package searchclient;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.PriorityQueue;

public final class CBSPlanner {
    private static final Action[] MOVE_ACTIONS = {
            Action.MoveN,
            Action.MoveS,
            Action.MoveE,
            Action.MoveW,
            Action.NoOp
    };

    private CBSPlanner() {}

    public static Action[][] planAgentGoals(
            State state,
            LevelAnalyzer analyzer,
            long deadline,
            int maxHighLevelNodes
    ) {
        PriorityQueue<CBSNode> open = new PriorityQueue<>(
                Comparator
                        .comparingInt((CBSNode node) -> node.cost)
                        .thenComparingInt(node -> node.constraints.size())
        );

        ArrayList<List<Action>> rootPlans = new ArrayList<>();
        ArrayList<Constraint> rootConstraints = new ArrayList<>();

        for (int agent = 0; agent < state.agentRows.length; agent++) {
            Position goal = findAgentGoal(agent);
            if (goal == null) {
                goal = new Position(state.agentRows[agent], state.agentCols[agent]);
            }

            List<Action> plan = lowLevelPlan(
                    state,
                    agent,
                    goal,
                    rootConstraints,
                    analyzer,
                    deadline,
                    80_000
            );

            if (plan == null) {
                return null;
            }

            rootPlans.add(plan);
        }

        open.add(new CBSNode(rootPlans, rootConstraints));
        int expanded = 0;

        while (!open.isEmpty() && System.nanoTime() < deadline && expanded++ < maxHighLevelNodes) {
            CBSNode node = open.poll();
            Conflict conflict = firstConflict(state, node.individualPlans);

            if (conflict == null) {
                return combinePlans(state.agentRows.length, node.individualPlans);
            }

            addChildForConflict(open, state, analyzer, node, conflict, conflict.agent1, deadline);
            addChildForConflict(open, state, analyzer, node, conflict, conflict.agent2, deadline);
        }

        return null;
    }

    private static void addChildForConflict(
            PriorityQueue<CBSNode> open,
            State state,
            LevelAnalyzer analyzer,
            CBSNode parent,
            Conflict conflict,
            int constrainedAgent,
            long deadline
    ) {
        ArrayList<Constraint> constraints = new ArrayList<>(parent.constraints);
        constraints.add(conflict.toConstraint(constrainedAgent));

        Position goal = findAgentGoal(constrainedAgent);
        if (goal == null) {
            goal = new Position(state.agentRows[constrainedAgent], state.agentCols[constrainedAgent]);
        }

        List<Action> repairedPlan = lowLevelPlan(
                state,
                constrainedAgent,
                goal,
                constraints,
                analyzer,
                deadline,
                80_000
        );

        if (repairedPlan == null) {
            return;
        }

        ArrayList<List<Action>> plans = new ArrayList<>(parent.individualPlans);
        plans.set(constrainedAgent, repairedPlan);
        open.add(new CBSNode(plans, constraints));
    }

    private static List<Action> lowLevelPlan(
            State state,
            int agent,
            Position goal,
            List<Constraint> constraints,
            LevelAnalyzer analyzer,
            long deadline,
            int maxExpansions
    ) {
        int startRow = state.agentRows[agent];
        int startCol = state.agentCols[agent];
        int latestConstraintTime = latestConstraintTime(agent, constraints);
        int timeLimit = Math.max(
                latestConstraintTime + analyzer.rows * analyzer.cols,
                manhattan(startRow, startCol, goal.row, goal.col) + 20
        );

        PriorityQueue<LowLevelNode> open = new PriorityQueue<>(
                Comparator
                        .comparingInt((LowLevelNode node) -> node.f)
                        .thenComparingInt(node -> node.time)
        );
        HashMap<LowLevelKey, Integer> bestG = new HashMap<>();

        if (violatesVertexConstraint(agent, startRow, startCol, 0, constraints)) {
            return null;
        }

        open.add(new LowLevelNode(
                startRow,
                startCol,
                0,
                null,
                null,
                manhattan(startRow, startCol, goal.row, goal.col)
        ));

        int expansions = 0;

        while (!open.isEmpty() && System.nanoTime() < deadline) {
            if (++expansions > maxExpansions) {
                return null;
            }

            LowLevelNode node = open.poll();
            LowLevelKey key = new LowLevelKey(node.row, node.col, node.time);
            Integer best = bestG.get(key);

            if (best != null && best <= node.time) {
                continue;
            }

            bestG.put(key, node.time);

            if (node.row == goal.row && node.col == goal.col && node.time >= latestConstraintTime) {
                return extractPlan(node);
            }

            if (node.time >= timeLimit) {
                continue;
            }

            for (Action action : MOVE_ACTIONS) {
                int nextRow = node.row + action.agentRowDelta;
                int nextCol = node.col + action.agentColDelta;
                int nextTime = node.time + 1;

                if (!cellFreeForMoveOnly(state, nextRow, nextCol)) {
                    continue;
                }

                if (violatesVertexConstraint(agent, nextRow, nextCol, nextTime, constraints)
                        || violatesEdgeConstraint(agent, node.row, node.col, nextRow, nextCol, nextTime, constraints)) {
                    continue;
                }

                int h = manhattan(nextRow, nextCol, goal.row, goal.col);
                open.add(new LowLevelNode(
                        nextRow,
                        nextCol,
                        nextTime,
                        node,
                        action,
                        nextTime + h
                ));
            }
        }

        return null;
    }

    private static Conflict firstConflict(State state, List<List<Action>> plans) {
        int horizon = 0;
        for (List<Action> plan : plans) {
            horizon = Math.max(horizon, plan.size());
        }

        for (int time = 0; time <= horizon; time++) {
            for (int a1 = 0; a1 < plans.size(); a1++) {
                Position p1 = positionAt(state, a1, plans.get(a1), time);

                for (int a2 = a1 + 1; a2 < plans.size(); a2++) {
                    Position p2 = positionAt(state, a2, plans.get(a2), time);

                    if (p1.equals(p2)) {
                        return Conflict.vertex(a1, a2, p1.row, p1.col, time);
                    }

                    if (time == 0) {
                        continue;
                    }

                    Position prev1 = positionAt(state, a1, plans.get(a1), time - 1);
                    Position prev2 = positionAt(state, a2, plans.get(a2), time - 1);

                    if (prev1.equals(p2) && prev2.equals(p1)) {
                        return Conflict.edge(a1, a2, prev1, p1, time);
                    }

                    // Hospital-domain applicability checks the cell before the
                    // joint action is applied, so an agent may not move into a
                    // cell occupied by another agent at the start of the same
                    // timestep, even if that other agent is moving away.
                    if (p1.equals(prev2) && !p1.equals(p2)) {
                        return Conflict.vertex(a1, a2, p1.row, p1.col, time);
                    }

                    if (p2.equals(prev1) && !p2.equals(p1)) {
                        return Conflict.vertex(a2, a1, p2.row, p2.col, time);
                    }
                }
            }
        }

        return null;
    }

    private static Position positionAt(State state, int agent, List<Action> plan, int time) {
        int row = state.agentRows[agent];
        int col = state.agentCols[agent];
        int steps = Math.min(time, plan.size());

        for (int i = 0; i < steps; i++) {
            Action action = plan.get(i);
            row += action.agentRowDelta;
            col += action.agentColDelta;
        }

        return new Position(row, col);
    }

    private static Action[][] combinePlans(int numAgents, List<List<Action>> plans) {
        int horizon = 0;
        for (List<Action> plan : plans) {
            horizon = Math.max(horizon, plan.size());
        }

        Action[][] jointPlan = new Action[horizon][numAgents];

        for (int time = 0; time < horizon; time++) {
            for (int agent = 0; agent < numAgents; agent++) {
                List<Action> plan = plans.get(agent);
                jointPlan[time][agent] = time < plan.size() ? plan.get(time) : Action.NoOp;
            }
        }

        return jointPlan;
    }

    private static boolean violatesVertexConstraint(
            int agent,
            int row,
            int col,
            int time,
            List<Constraint> constraints
    ) {
        for (Constraint constraint : constraints) {
            if (constraint.agent == agent
                    && !constraint.edge
                    && constraint.time == time
                    && constraint.row == row
                    && constraint.col == col) {
                return true;
            }
        }

        return false;
    }

    private static boolean violatesEdgeConstraint(
            int agent,
            int fromRow,
            int fromCol,
            int toRow,
            int toCol,
            int time,
            List<Constraint> constraints
    ) {
        for (Constraint constraint : constraints) {
            if (constraint.agent == agent
                    && constraint.edge
                    && constraint.time == time
                    && constraint.fromRow == fromRow
                    && constraint.fromCol == fromCol
                    && constraint.row == toRow
                    && constraint.col == toCol) {
                return true;
            }
        }

        return false;
    }

    private static int latestConstraintTime(int agent, List<Constraint> constraints) {
        int latest = 0;

        for (Constraint constraint : constraints) {
            if (constraint.agent == agent) {
                latest = Math.max(latest, constraint.time);
            }
        }

        return latest;
    }

    private static boolean cellFreeForMoveOnly(State state, int row, int col) {
        return inBounds(row, col)
                && !State.walls[row][col]
                && state.boxes[row][col] == 0;
    }

    private static boolean inBounds(int row, int col) {
        return 0 <= row
                && row < State.walls.length
                && 0 <= col
                && col < State.walls[row].length;
    }

    private static Position findAgentGoal(int agent) {
        char goalChar = (char) ('0' + agent);

        for (int row = 0; row < State.goals.length; row++) {
            for (int col = 0; col < State.goals[row].length; col++) {
                if (State.goals[row][col] == goalChar) {
                    return new Position(row, col);
                }
            }
        }

        return null;
    }

    private static int manhattan(int row1, int col1, int row2, int col2) {
        return Math.abs(row1 - row2) + Math.abs(col1 - col2);
    }

    private static List<Action> extractPlan(LowLevelNode node) {
        ArrayList<Action> reversed = new ArrayList<>();

        for (LowLevelNode current = node; current.parent != null; current = current.parent) {
            reversed.add(current.action);
        }

        ArrayList<Action> plan = new ArrayList<>(reversed.size());
        for (int i = reversed.size() - 1; i >= 0; i--) {
            plan.add(reversed.get(i));
        }

        return plan;
    }

    static final class Constraint {
        final int agent;
        final int row;
        final int col;
        final int time;
        final boolean edge;
        final int fromRow;
        final int fromCol;

        Constraint(int agent, int row, int col, int time) {
            this.agent = agent;
            this.row = row;
            this.col = col;
            this.time = time;
            this.edge = false;
            this.fromRow = -1;
            this.fromCol = -1;
        }

        Constraint(int agent, int fromRow, int fromCol, int toRow, int toCol, int time) {
            this.agent = agent;
            this.row = toRow;
            this.col = toCol;
            this.time = time;
            this.edge = true;
            this.fromRow = fromRow;
            this.fromCol = fromCol;
        }
    }

    static final class CBSNode {
        final ArrayList<List<Action>> individualPlans;
        final ArrayList<Constraint> constraints;
        final int cost;

        CBSNode(ArrayList<List<Action>> individualPlans, ArrayList<Constraint> constraints) {
            this.individualPlans = individualPlans;
            this.constraints = constraints;
            int total = 0;
            for (List<Action> plan : individualPlans) {
                total += plan.size();
            }
            this.cost = total;
        }
    }

    static final class Conflict {
        final int agent1;
        final int agent2;
        final int row;
        final int col;
        final int time;
        final boolean edge;
        final int fromRow;
        final int fromCol;
        final int toRow;
        final int toCol;

        private Conflict(
                int agent1,
                int agent2,
                int row,
                int col,
                int time,
                boolean edge,
                int fromRow,
                int fromCol,
                int toRow,
                int toCol
        ) {
            this.agent1 = agent1;
            this.agent2 = agent2;
            this.row = row;
            this.col = col;
            this.time = time;
            this.edge = edge;
            this.fromRow = fromRow;
            this.fromCol = fromCol;
            this.toRow = toRow;
            this.toCol = toCol;
        }

        static Conflict vertex(int agent1, int agent2, int row, int col, int time) {
            return new Conflict(agent1, agent2, row, col, time, false, -1, -1, -1, -1);
        }

        static Conflict edge(int agent1, int agent2, Position from, Position to, int time) {
            return new Conflict(agent1, agent2, to.row, to.col, time, true, from.row, from.col, to.row, to.col);
        }

        Constraint toConstraint(int agent) {
            if (!edge) {
                return new Constraint(agent, row, col, time);
            }

            if (agent == agent1) {
                return new Constraint(agent, fromRow, fromCol, toRow, toCol, time);
            }

            return new Constraint(agent, toRow, toCol, fromRow, fromCol, time);
        }
    }

    private static final class LowLevelNode {
        final int row;
        final int col;
        final int time;
        final LowLevelNode parent;
        final Action action;
        final int f;

        LowLevelNode(int row, int col, int time, LowLevelNode parent, Action action, int f) {
            this.row = row;
            this.col = col;
            this.time = time;
            this.parent = parent;
            this.action = action;
            this.f = f;
        }
    }

    private static final class LowLevelKey {
        final int row;
        final int col;
        final int time;

        LowLevelKey(int row, int col, int time) {
            this.row = row;
            this.col = col;
            this.time = time;
        }

        @Override
        public boolean equals(Object obj) {
            if (!(obj instanceof LowLevelKey)) {
                return false;
            }

            LowLevelKey other = (LowLevelKey) obj;
            return row == other.row && col == other.col && time == other.time;
        }

        @Override
        public int hashCode() {
            int result = row;
            result = 31 * result + col;
            result = 31 * result + time;
            return result;
        }
    }
}
