package searchclient;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.PriorityQueue;

/**
 * A practical CBS coordinator for box-moving tasks.
 *
 * High-level CBS branches on conflicts between single-agent plans. The low-level
 * planner replans one constrained agent with weighted A*. For each box task only
 * one selected box is movable; all other boxes are treated as static obstacles.
 *
 * This is intentionally narrower than full MAPF-Sokoban CBS: coordinating moving
 * boxes is harder than move-only CBS because each agent path can also change box
 * positions over time, creating agent-box and box-box conflicts in addition to
 * ordinary agent-agent conflicts.
 */
public final class BoxCBSPlanner {
    private static final int LOW_LEVEL_LIMIT = 90_000;
    private static final int HIGH_LEVEL_LIMIT = 1_200;
    private static final int MAX_CBS_BOX_TASKS = 8;
    private static final int CONSERVATIVE_MAX_BOX_GOAL_DISTANCE = 10;
    private static final int CONSERVATIVE_MAX_AGENT_ACCESS_DISTANCE = 25;
    private static final int CONSERVATIVE_MAX_SEGMENT_LENGTH = 45;

    private static final Action[] LOW_LEVEL_ACTIONS = Action.values();
    private static Result lastResult = Result.empty();

    private BoxCBSPlanner() {}

    public static Result lastResult() {
        return lastResult;
    }

    public static Result solveBoxTasks(State initialState, LevelAnalyzer analyzer, long deadline) {
        return solveBoxTasks(initialState, analyzer, deadline, MAX_CBS_BOX_TASKS);
    }

    public static Result solveBoxTasks(State initialState, LevelAnalyzer analyzer, long deadline, int maxBoxTasks) {
        return solveBoxTasks(initialState, analyzer, deadline, maxBoxTasks, false);
    }

    public static Result solveConservativeBoxTasks(State initialState, LevelAnalyzer analyzer, long deadline, int maxBoxTasks) {
        return solveBoxTasks(initialState, analyzer, deadline, maxBoxTasks, true);
    }

    private static Result solveBoxTasks(
            State initialState,
            LevelAnalyzer analyzer,
            long deadline,
            int maxBoxTasks,
            boolean conservative
    ) {
        TaskManager taskManager = new TaskManager(analyzer);
        State current = copyState(initialState);
        ArrayList<Action[]> committedPlan = new ArrayList<>();
        int solvedTasks = 0;
        int attemptedTasks = 0;
        boolean attempted = false;

        while (solvedTasks < maxBoxTasks
                && countUnsolvedBoxGoals(current) > 0
                && System.nanoTime() < deadline) {
            List<Task> tasks = taskManager.orderTasks(taskManager.createTasks(current));
            if (tasks.isEmpty()) {
                break;
            }

            if (conservative) {
                tasks = conservativeTasks(current, analyzer, tasks);
                if (tasks.isEmpty()) {
                    break;
                }
            }

            attempted = true;
            Task selected = null;
            Action[][] segment = null;

            int attempts = Math.min(conservative ? 6 : 10, tasks.size());
            for (int i = 0; i < attempts && System.nanoTime() < deadline; i++) {
                Task candidate = tasks.get(i);
                attemptedTasks++;
                Action[][] candidateSegment = planTaskWithCBS(current, analyzer, candidate, deadline);

                if (candidateSegment != null && candidateSegment.length > 0) {
                    if (conservative && candidateSegment.length > CONSERVATIVE_MAX_SEGMENT_LENGTH) {
                        continue;
                    }
                    selected = candidate;
                    segment = candidateSegment;
                    break;
                }
            }

            if (selected == null || segment == null || segment.length == 0) {
                break;
            }

            State next = simulateJointPlan(current, segment);
            if (next.boxes[selected.goal.row][selected.goal.col] != selected.boxLetter) {
                break;
            }

            appendJointPlan(committedPlan, segment);
            current = next;
            solvedTasks++;
            System.err.format("Box-CBS committed task %s using %,d actions.%n", selected, segment.length);
        }

        boolean solvedAllBoxes = countUnsolvedBoxGoals(current) == 0;
        lastResult = new Result(
                committedPlan.toArray(new Action[0][]),
                current,
                solvedTasks,
                attemptedTasks,
                attempted,
                solvedAllBoxes
        );
        return lastResult;
    }

    private static List<Task> conservativeTasks(State state, LevelAnalyzer analyzer, List<Task> tasks) {
        ArrayList<Task> filtered = new ArrayList<>();

        for (Task task : tasks) {
            if (isConservativeTask(state, analyzer, task)) {
                filtered.add(task);
            }
        }

        filtered.sort(
                Comparator
                        .comparingInt((Task task) -> task.boxStart.manhattanDistance(task.goal))
                        .thenComparingInt(task -> nearestAccessDistance(state, task))
                        .thenComparingInt(task -> task.priority)
        );
        return filtered;
    }

    private static boolean isConservativeTask(State state, LevelAnalyzer analyzer, Task task) {
        if (task.assignedAgent < 0 || task.assignedAgent >= state.agentRows.length) {
            return false;
        }

        if (!inBounds(task.boxStart.row, task.boxStart.col)
                || state.boxes[task.boxStart.row][task.boxStart.col] != task.boxLetter) {
            return false;
        }

        if (State.agentColors[task.assignedAgent] != State.boxColors[task.boxLetter - 'A']) {
            return false;
        }

        int boxGoalDistance = task.boxStart.manhattanDistance(task.goal);
        if (boxGoalDistance > CONSERVATIVE_MAX_BOX_GOAL_DISTANCE) {
            return false;
        }

        if (analyzer.isDeadSquare(task.goal, task.boxLetter)) {
            return false;
        }

        if (isTightBoundaryPocket(analyzer, task.goal) && boxGoalDistance > 4) {
            return false;
        }

        return nearestAccessDistance(state, task) <= CONSERVATIVE_MAX_AGENT_ACCESS_DISTANCE;
    }

    private static boolean isTightBoundaryPocket(LevelAnalyzer analyzer, Position goal) {
        int boundaryCount = 0;
        if (goal.row <= 1 || goal.row >= analyzer.rows - 2) {
            boundaryCount++;
        }
        if (goal.col <= 1 || goal.col >= analyzer.cols - 2) {
            boundaryCount++;
        }

        int walls = 0;
        if (analyzer.isWall(goal.row - 1, goal.col)) {
            walls++;
        }
        if (analyzer.isWall(goal.row + 1, goal.col)) {
            walls++;
        }
        if (analyzer.isWall(goal.row, goal.col - 1)) {
            walls++;
        }
        if (analyzer.isWall(goal.row, goal.col + 1)) {
            walls++;
        }

        return boundaryCount > 0 && walls >= 2;
    }

    private static int nearestAccessDistance(State state, Task task) {
        int best = LevelAnalyzer.INF;
        Position agentStart = new Position(state.agentRows[task.assignedAgent], state.agentCols[task.assignedAgent]);

        for (Position access : accessCells(task.boxStart)) {
            if (!cellFreeForAgentRoute(state, task.assignedAgent, access)) {
                continue;
            }

            best = Math.min(best, routeDistance(state, task.assignedAgent, agentStart, access));
        }

        return best;
    }

    private static ArrayList<Position> accessCells(Position box) {
        ArrayList<Position> cells = new ArrayList<>(4);
        cells.add(new Position(box.row - 1, box.col));
        cells.add(new Position(box.row + 1, box.col));
        cells.add(new Position(box.row, box.col - 1));
        cells.add(new Position(box.row, box.col + 1));
        return cells;
    }

    private static int routeDistance(State state, int activeAgent, Position start, Position target) {
        ArrayList<Position> queue = new ArrayList<>();
        HashMap<Position, Integer> distance = new HashMap<>();
        queue.add(start);
        distance.put(start, 0);

        for (int head = 0; head < queue.size(); head++) {
            Position current = queue.get(head);
            int currentDistance = distance.get(current);
            if (current.equals(target)) {
                return currentDistance;
            }

            for (Position next : accessCells(current)) {
                if (distance.containsKey(next) || !cellFreeForAgentRoute(state, activeAgent, next)) {
                    continue;
                }

                distance.put(next, currentDistance + 1);
                queue.add(next);
            }
        }

        return LevelAnalyzer.INF;
    }

    private static boolean cellFreeForAgentRoute(State state, int activeAgent, Position position) {
        if (!inBounds(position.row, position.col)
                || State.walls[position.row][position.col]
                || state.boxes[position.row][position.col] != 0) {
            return false;
        }

        for (int agent = 0; agent < state.agentRows.length; agent++) {
            if (agent != activeAgent
                    && state.agentRows[agent] == position.row
                    && state.agentCols[agent] == position.col) {
                return false;
            }
        }

        return true;
    }

    private static Action[][] planTaskWithCBS(
            State state,
            LevelAnalyzer analyzer,
            Task task,
            long deadline
    ) {
        PriorityQueue<CBSNode> open = new PriorityQueue<>(
                Comparator
                        .comparingInt((CBSNode node) -> node.cost)
                        .thenComparingInt(node -> node.constraints.size())
        );

        ArrayList<Constraint> rootConstraints = new ArrayList<>();
        ArrayList<List<Action>> rootPlans = new ArrayList<>();

        for (int agent = 0; agent < state.agentRows.length; agent++) {
            List<Action> plan = lowLevelPlan(state, analyzer, task, agent, rootConstraints, deadline);
            if (plan == null) {
                return null;
            }
            rootPlans.add(plan);
        }

        open.add(new CBSNode(rootPlans, rootConstraints));
        int expanded = 0;

        while (!open.isEmpty() && System.nanoTime() < deadline && expanded++ < HIGH_LEVEL_LIMIT) {
            CBSNode node = open.poll();
            Conflict conflict = firstConflict(state, task, node.individualPlans);

            if (conflict == null) {
                return combinePlans(state.agentRows.length, node.individualPlans);
            }

            addChild(open, state, analyzer, task, node, conflict, conflict.agent1, deadline);
            addChild(open, state, analyzer, task, node, conflict, conflict.agent2, deadline);
        }

        return null;
    }

    private static void addChild(
            PriorityQueue<CBSNode> open,
            State state,
            LevelAnalyzer analyzer,
            Task task,
            CBSNode parent,
            Conflict conflict,
            int constrainedAgent,
            long deadline
    ) {
        ArrayList<Constraint> constraints = new ArrayList<>(parent.constraints);
        constraints.add(conflict.toConstraint(constrainedAgent));

        List<Action> repairedPlan = lowLevelPlan(state, analyzer, task, constrainedAgent, constraints, deadline);
        if (repairedPlan == null) {
            return;
        }

        ArrayList<List<Action>> plans = new ArrayList<>(parent.individualPlans);
        plans.set(constrainedAgent, repairedPlan);
        open.add(new CBSNode(plans, constraints));
    }

    private static List<Action> lowLevelPlan(
            State state,
            LevelAnalyzer analyzer,
            Task task,
            int agent,
            List<Constraint> constraints,
            long deadline
    ) {
        if (agent != task.assignedAgent) {
            return lowLevelAvoidancePlan(state, agent, constraints, deadline);
        }

        PriorityQueue<LowLevelNode> open = new PriorityQueue<>(
                Comparator
                        .comparingInt((LowLevelNode node) -> node.f)
                        .thenComparingInt(node -> node.time)
        );
        HashMap<LowLevelKey, Integer> bestG = new HashMap<>();
        int latestConstraint = latestConstraintTime(agent, constraints);
        int timeLimit = Math.max(
                latestConstraint + analyzer.rows * analyzer.cols,
                task.boxStart.manhattanDistance(task.goal)
                        + Math.abs(state.agentRows[agent] - task.boxStart.row)
                        + Math.abs(state.agentCols[agent] - task.boxStart.col)
                        + 30
        );

        LowLevelNode root = new LowLevelNode(
                state.agentRows[agent],
                state.agentCols[agent],
                task.boxStart.row,
                task.boxStart.col,
                0,
                null,
                null,
                heuristic(state.agentRows[agent], state.agentCols[agent], task.boxStart.row, task.boxStart.col, task.goal)
        );
        open.add(root);
        int expansions = 0;

        while (!open.isEmpty() && System.nanoTime() < deadline) {
            if (++expansions > LOW_LEVEL_LIMIT) {
                return null;
            }

            LowLevelNode node = open.poll();
            LowLevelKey key = new LowLevelKey(node.agentRow, node.agentCol, node.boxRow, node.boxCol, node.time);
            Integer best = bestG.get(key);
            if (best != null && best <= node.time) {
                continue;
            }
            bestG.put(key, node.time);

            if (node.boxRow == task.goal.row && node.boxCol == task.goal.col && node.time >= latestConstraint) {
                return extractPlan(node);
            }

            if (node.time >= timeLimit) {
                continue;
            }

            for (Action action : LOW_LEVEL_ACTIONS) {
                if (action.type != ActionType.Move && action.type != ActionType.Push
                        && action.type != ActionType.Pull && action != Action.NoOp) {
                    continue;
                }

                LowLevelNode child = applyLowLevelAction(state, task, agent, node, action);
                if (child == null) {
                    continue;
                }

                if (violatesConstraints(agent, node, child, constraints)) {
                    continue;
                }

                int h = heuristic(child.agentRow, child.agentCol, child.boxRow, child.boxCol, task.goal);
                child.f = child.time + 5 * h;
                open.add(child);
            }
        }

        return null;
    }

    private static List<Action> lowLevelAvoidancePlan(
            State state,
            int agent,
            List<Constraint> constraints,
            long deadline
    ) {
        int latest = latestConstraintTime(agent, constraints);
        PriorityQueue<MoveNode> open = new PriorityQueue<>(
                Comparator
                        .comparingInt((MoveNode node) -> node.f)
                        .thenComparingInt(node -> node.time)
        );
        HashMap<MoveKey, Integer> bestG = new HashMap<>();
        open.add(new MoveNode(state.agentRows[agent], state.agentCols[agent], 0, null, null, 0));
        int expansions = 0;

        while (!open.isEmpty() && System.nanoTime() < deadline) {
            if (++expansions > 20_000) {
                return null;
            }

            MoveNode node = open.poll();
            MoveKey key = new MoveKey(node.row, node.col, node.time);
            Integer best = bestG.get(key);
            if (best != null && best <= node.time) {
                continue;
            }
            bestG.put(key, node.time);

            if (node.time >= latest) {
                return extractMovePlan(node);
            }

            for (Action action : new Action[] { Action.NoOp, Action.MoveN, Action.MoveS, Action.MoveE, Action.MoveW }) {
                int nextRow = node.row + action.agentRowDelta;
                int nextCol = node.col + action.agentColDelta;
                int nextTime = node.time + 1;

                if (!cellFreeForMoveOnly(state, nextRow, nextCol)) {
                    continue;
                }

                if (violatesAgentConstraint(agent, node.row, node.col, nextRow, nextCol, nextTime, constraints)) {
                    continue;
                }

                open.add(new MoveNode(nextRow, nextCol, nextTime, node, action, nextTime));
            }
        }

        return null;
    }

    private static LowLevelNode applyLowLevelAction(
            State state,
            Task task,
            int agent,
            LowLevelNode node,
            Action action
    ) {
        int nextAgentRow = node.agentRow + action.agentRowDelta;
        int nextAgentCol = node.agentCol + action.agentColDelta;
        int nextBoxRow = node.boxRow;
        int nextBoxCol = node.boxCol;

        if (action == Action.NoOp) {
            return new LowLevelNode(nextAgentRow, nextAgentCol, nextBoxRow, nextBoxCol, node.time + 1, node, action, 0);
        }

        if (!inBounds(nextAgentRow, nextAgentCol) || State.walls[nextAgentRow][nextAgentCol]) {
            return null;
        }

        switch (action.type) {
            case Move:
                if (nextAgentRow == node.boxRow && nextAgentCol == node.boxCol) {
                    return null;
                }
                if (fixedBoxAt(state, task, nextAgentRow, nextAgentCol)) {
                    return null;
                }
                break;

            case Push:
                if (action.agentRowDelta != action.boxRowDelta || action.agentColDelta != action.boxColDelta) {
                    return null;
                }
                if (nextAgentRow != node.boxRow || nextAgentCol != node.boxCol) {
                    return null;
                }
                nextBoxRow = node.boxRow + action.boxRowDelta;
                nextBoxCol = node.boxCol + action.boxColDelta;
                if (!boxCellFree(state, task, nextBoxRow, nextBoxCol)) {
                    return null;
                }
                break;

            case Pull:
                if (action.agentRowDelta != action.boxRowDelta || action.agentColDelta != action.boxColDelta) {
                    return null;
                }
                int sourceBoxRow = node.agentRow - action.boxRowDelta;
                int sourceBoxCol = node.agentCol - action.boxColDelta;
                if (sourceBoxRow != node.boxRow || sourceBoxCol != node.boxCol) {
                    return null;
                }
                if (!boxCellFree(state, task, nextAgentRow, nextAgentCol)) {
                    return null;
                }
                nextBoxRow = node.agentRow;
                nextBoxCol = node.agentCol;
                break;

            default:
                return null;
        }

        return new LowLevelNode(nextAgentRow, nextAgentCol, nextBoxRow, nextBoxCol, node.time + 1, node, action, 0);
    }

    private static Conflict firstConflict(State state, Task task, List<List<Action>> plans) {
        ArrayList<Trajectory> trajectories = new ArrayList<>();
        int horizon = 0;

        for (int agent = 0; agent < plans.size(); agent++) {
            Trajectory trajectory = buildTrajectory(state, task, agent, plans.get(agent));
            trajectories.add(trajectory);
            horizon = Math.max(horizon, trajectory.agentRows.size() - 1);
        }

        for (int time = 0; time <= horizon; time++) {
            for (int a1 = 0; a1 < trajectories.size(); a1++) {
                Trajectory t1 = trajectories.get(a1);
                int a1Row = t1.agentRowAt(time);
                int a1Col = t1.agentColAt(time);

                for (int a2 = a1 + 1; a2 < trajectories.size(); a2++) {
                    Trajectory t2 = trajectories.get(a2);
                    int a2Row = t2.agentRowAt(time);
                    int a2Col = t2.agentColAt(time);

                    if (a1Row == a2Row && a1Col == a2Col) {
                        return Conflict.agentVertex(a1, a2, a1Row, a1Col, time);
                    }

                    if (time > 0
                            && t1.agentRowAt(time - 1) == a2Row
                            && t1.agentColAt(time - 1) == a2Col
                            && t2.agentRowAt(time - 1) == a1Row
                            && t2.agentColAt(time - 1) == a1Col) {
                        return Conflict.agentEdge(a1, a2, t1.agentRowAt(time - 1), t1.agentColAt(time - 1), a1Row, a1Col, time);
                    }

                    if (t1.boxExistsAt(time) && a2Row == t1.boxRowAt(time) && a2Col == t1.boxColAt(time)) {
                        return Conflict.agentBox(a2, a1, a2Row, a2Col, time);
                    }

                    if (t2.boxExistsAt(time) && a1Row == t2.boxRowAt(time) && a1Col == t2.boxColAt(time)) {
                        return Conflict.agentBox(a1, a2, a1Row, a1Col, time);
                    }

                    if (t1.movedBoxAt(time) && t2.movedBoxAt(time)) {
                        if (t1.boxRowAt(time) == t2.boxRowAt(time) && t1.boxColAt(time) == t2.boxColAt(time)) {
                            return Conflict.boxVertex(a1, a2, t1.boxRowAt(time), t1.boxColAt(time), time);
                        }

                        if (time > 0
                                && t1.boxRowAt(time - 1) == t2.boxRowAt(time)
                                && t1.boxColAt(time - 1) == t2.boxColAt(time)
                                && t2.boxRowAt(time - 1) == t1.boxRowAt(time)
                                && t2.boxColAt(time - 1) == t1.boxColAt(time)) {
                            return Conflict.boxEdge(a1, a2, t1.boxRowAt(time - 1), t1.boxColAt(time - 1), t1.boxRowAt(time), t1.boxColAt(time), time);
                        }
                    }

                    if (t1.movedBoxAt(time) && t2.movedBoxAt(time)) {
                        return Conflict.sameBox(a1, a2, t1.boxRowAt(time), t1.boxColAt(time), time);
                    }
                }
            }
        }

        return null;
    }

    private static Trajectory buildTrajectory(State state, Task task, int agent, List<Action> plan) {
        Trajectory trajectory = new Trajectory();
        int agentRow = state.agentRows[agent];
        int agentCol = state.agentCols[agent];
        int boxRow = task.boxStart.row;
        int boxCol = task.boxStart.col;

        trajectory.add(agentRow, agentCol, boxRow, boxCol, agent == task.assignedAgent);

        for (Action action : plan) {
            switch (action.type) {
                case Move:
                    agentRow += action.agentRowDelta;
                    agentCol += action.agentColDelta;
                    break;
                case Push:
                    agentRow += action.agentRowDelta;
                    agentCol += action.agentColDelta;
                    if (agent == task.assignedAgent) {
                        boxRow += action.boxRowDelta;
                        boxCol += action.boxColDelta;
                    }
                    break;
                case Pull:
                    if (agent == task.assignedAgent) {
                        boxRow = agentRow;
                        boxCol = agentCol;
                    }
                    agentRow += action.agentRowDelta;
                    agentCol += action.agentColDelta;
                    break;
                default:
                    break;
            }
            trajectory.add(agentRow, agentCol, boxRow, boxCol, agent == task.assignedAgent);
        }

        return trajectory;
    }

    private static boolean violatesConstraints(int agent, LowLevelNode from, LowLevelNode to, List<Constraint> constraints) {
        if (violatesAgentConstraint(agent, from.agentRow, from.agentCol, to.agentRow, to.agentCol, to.time, constraints)) {
            return true;
        }

        for (Constraint constraint : constraints) {
            if (constraint.agent != agent || constraint.time != to.time) {
                continue;
            }

            if (constraint.type == ConstraintType.BOX_VERTEX
                    && constraint.row == to.boxRow
                    && constraint.col == to.boxCol) {
                return true;
            }

            if (constraint.type == ConstraintType.BOX_EDGE
                    && constraint.fromRow == from.boxRow
                    && constraint.fromCol == from.boxCol
                    && constraint.row == to.boxRow
                    && constraint.col == to.boxCol) {
                return true;
            }

            if (constraint.type == ConstraintType.BOX_INTERACTION
                    && from.boxRow != to.boxRow
                    && from.boxCol != to.boxCol) {
                return true;
            }
        }

        return false;
    }

    private static boolean violatesAgentConstraint(
            int agent,
            int fromRow,
            int fromCol,
            int toRow,
            int toCol,
            int time,
            List<Constraint> constraints
    ) {
        for (Constraint constraint : constraints) {
            if (constraint.agent != agent || constraint.time != time) {
                continue;
            }

            if (constraint.type == ConstraintType.AGENT_VERTEX
                    && constraint.row == toRow
                    && constraint.col == toCol) {
                return true;
            }

            if (constraint.type == ConstraintType.AGENT_EDGE
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

    private static int heuristic(int agentRow, int agentCol, int boxRow, int boxCol, Position goal) {
        return Math.abs(agentRow - boxRow)
                + Math.abs(agentCol - boxCol)
                + Math.abs(boxRow - goal.row)
                + Math.abs(boxCol - goal.col);
    }

    private static boolean boxCellFree(State state, Task task, int row, int col) {
        return inBounds(row, col)
                && !State.walls[row][col]
                && !fixedBoxAt(state, task, row, col);
    }

    private static boolean fixedBoxAt(State state, Task task, int row, int col) {
        return state.boxes[row][col] != 0
                && !(row == task.boxStart.row && col == task.boxStart.col);
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

    private static List<Action> extractMovePlan(MoveNode node) {
        ArrayList<Action> reversed = new ArrayList<>();
        for (MoveNode current = node; current.parent != null; current = current.parent) {
            reversed.add(current.action);
        }

        ArrayList<Action> plan = new ArrayList<>(reversed.size());
        for (int i = reversed.size() - 1; i >= 0; i--) {
            plan.add(reversed.get(i));
        }
        return plan;
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

    private static State simulateJointPlan(State state, Action[][] plan) {
        State current = copyState(state);
        for (Action[] jointAction : plan) {
            current = applyJointAction(current, jointAction);
        }
        return current;
    }

    private static State applyJointAction(State state, Action[] jointAction) {
        State next = copyState(state);
        for (int agent = 0; agent < jointAction.length; agent++) {
            Action action = jointAction[agent];
            int ar = next.agentRows[agent];
            int ac = next.agentCols[agent];

            switch (action.type) {
                case Move:
                    next.agentRows[agent] = ar + action.agentRowDelta;
                    next.agentCols[agent] = ac + action.agentColDelta;
                    break;
                case Push: {
                    int boxRow = ar + action.agentRowDelta;
                    int boxCol = ac + action.agentColDelta;
                    int newBoxRow = boxRow + action.boxRowDelta;
                    int newBoxCol = boxCol + action.boxColDelta;
                    char box = next.boxes[boxRow][boxCol];
                    next.boxes[boxRow][boxCol] = 0;
                    next.boxes[newBoxRow][newBoxCol] = box;
                    next.agentRows[agent] = ar + action.agentRowDelta;
                    next.agentCols[agent] = ac + action.agentColDelta;
                    break;
                }
                case Pull: {
                    int boxRow = ar - action.boxRowDelta;
                    int boxCol = ac - action.boxColDelta;
                    int newBoxRow = boxRow + action.boxRowDelta;
                    int newBoxCol = boxCol + action.boxColDelta;
                    char box = next.boxes[boxRow][boxCol];
                    next.boxes[boxRow][boxCol] = 0;
                    next.boxes[newBoxRow][newBoxCol] = box;
                    next.agentRows[agent] = ar + action.agentRowDelta;
                    next.agentCols[agent] = ac + action.agentColDelta;
                    break;
                }
                default:
                    break;
            }
        }
        return next;
    }

    private static State copyState(State state) {
        int[] rows = state.agentRows.clone();
        int[] cols = state.agentCols.clone();
        char[][] boxes = new char[state.boxes.length][];
        for (int row = 0; row < state.boxes.length; row++) {
            boxes[row] = state.boxes[row].clone();
        }
        return new State(rows, cols, State.agentColors, State.walls, boxes, State.boxColors, State.goals);
    }

    private static void appendJointPlan(ArrayList<Action[]> plan, Action[][] suffix) {
        for (Action[] jointAction : suffix) {
            plan.add(jointAction.clone());
        }
    }

    private static int countUnsolvedBoxGoals(State state) {
        int count = 0;
        for (int row = 0; row < State.goals.length; row++) {
            for (int col = 0; col < State.goals[row].length; col++) {
                char goal = State.goals[row][col];
                if ('A' <= goal && goal <= 'Z' && state.boxes[row][col] != goal) {
                    count++;
                }
            }
        }
        return count;
    }

    public static final class Result {
        public final Action[][] plan;
        public final State state;
        public final int solvedBoxTasks;
        public final int attemptedBoxTasks;
        public final boolean attempted;
        public final boolean solvedAllBoxes;

        Result(Action[][] plan, State state, int solvedBoxTasks, int attemptedBoxTasks, boolean attempted, boolean solvedAllBoxes) {
            this.plan = plan;
            this.state = state;
            this.solvedBoxTasks = solvedBoxTasks;
            this.attemptedBoxTasks = attemptedBoxTasks;
            this.attempted = attempted;
            this.solvedAllBoxes = solvedAllBoxes;
        }

        static Result empty() {
            return new Result(new Action[0][], null, 0, 0, false, false);
        }
    }

    static final class Constraint {
        final int agent;
        final ConstraintType type;
        final int row;
        final int col;
        final int time;
        final int fromRow;
        final int fromCol;

        Constraint(int agent, ConstraintType type, int row, int col, int time) {
            this(agent, type, row, col, time, -1, -1);
        }

        Constraint(int agent, ConstraintType type, int row, int col, int time, int fromRow, int fromCol) {
            this.agent = agent;
            this.type = type;
            this.row = row;
            this.col = col;
            this.time = time;
            this.fromRow = fromRow;
            this.fromCol = fromCol;
        }
    }

    enum ConstraintType {
        AGENT_VERTEX,
        AGENT_EDGE,
        BOX_VERTEX,
        BOX_EDGE,
        BOX_INTERACTION
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
        final ConstraintType type;
        final int row;
        final int col;
        final int fromRow;
        final int fromCol;
        final int time;

        private Conflict(int agent1, int agent2, ConstraintType type, int row, int col, int fromRow, int fromCol, int time) {
            this.agent1 = agent1;
            this.agent2 = agent2;
            this.type = type;
            this.row = row;
            this.col = col;
            this.fromRow = fromRow;
            this.fromCol = fromCol;
            this.time = time;
        }

        static Conflict agentVertex(int a1, int a2, int row, int col, int time) {
            return new Conflict(a1, a2, ConstraintType.AGENT_VERTEX, row, col, -1, -1, time);
        }

        static Conflict agentEdge(int a1, int a2, int fromRow, int fromCol, int row, int col, int time) {
            return new Conflict(a1, a2, ConstraintType.AGENT_EDGE, row, col, fromRow, fromCol, time);
        }

        static Conflict agentBox(int agent, int boxAgent, int row, int col, int time) {
            return new Conflict(agent, boxAgent, ConstraintType.AGENT_VERTEX, row, col, -1, -1, time);
        }

        static Conflict boxVertex(int a1, int a2, int row, int col, int time) {
            return new Conflict(a1, a2, ConstraintType.BOX_VERTEX, row, col, -1, -1, time);
        }

        static Conflict boxEdge(int a1, int a2, int fromRow, int fromCol, int row, int col, int time) {
            return new Conflict(a1, a2, ConstraintType.BOX_EDGE, row, col, fromRow, fromCol, time);
        }

        static Conflict sameBox(int a1, int a2, int row, int col, int time) {
            return new Conflict(a1, a2, ConstraintType.BOX_INTERACTION, row, col, -1, -1, time);
        }

        Constraint toConstraint(int agent) {
            if (type == ConstraintType.AGENT_EDGE || type == ConstraintType.BOX_EDGE) {
                return new Constraint(agent, type, row, col, time, fromRow, fromCol);
            }
            return new Constraint(agent, type, row, col, time);
        }
    }

    private static final class LowLevelNode {
        final int agentRow;
        final int agentCol;
        final int boxRow;
        final int boxCol;
        final int time;
        final LowLevelNode parent;
        final Action action;
        int f;

        LowLevelNode(int agentRow, int agentCol, int boxRow, int boxCol, int time, LowLevelNode parent, Action action, int f) {
            this.agentRow = agentRow;
            this.agentCol = agentCol;
            this.boxRow = boxRow;
            this.boxCol = boxCol;
            this.time = time;
            this.parent = parent;
            this.action = action;
            this.f = f;
        }
    }

    private static final class LowLevelKey {
        final int agentRow;
        final int agentCol;
        final int boxRow;
        final int boxCol;
        final int time;

        LowLevelKey(int agentRow, int agentCol, int boxRow, int boxCol, int time) {
            this.agentRow = agentRow;
            this.agentCol = agentCol;
            this.boxRow = boxRow;
            this.boxCol = boxCol;
            this.time = time;
        }

        @Override
        public boolean equals(Object obj) {
            if (!(obj instanceof LowLevelKey)) {
                return false;
            }
            LowLevelKey other = (LowLevelKey) obj;
            return agentRow == other.agentRow
                    && agentCol == other.agentCol
                    && boxRow == other.boxRow
                    && boxCol == other.boxCol
                    && time == other.time;
        }

        @Override
        public int hashCode() {
            int result = agentRow;
            result = 31 * result + agentCol;
            result = 31 * result + boxRow;
            result = 31 * result + boxCol;
            result = 31 * result + time;
            return result;
        }
    }

    private static final class MoveNode {
        final int row;
        final int col;
        final int time;
        final MoveNode parent;
        final Action action;
        final int f;

        MoveNode(int row, int col, int time, MoveNode parent, Action action, int f) {
            this.row = row;
            this.col = col;
            this.time = time;
            this.parent = parent;
            this.action = action;
            this.f = f;
        }
    }

    private static final class MoveKey {
        final int row;
        final int col;
        final int time;

        MoveKey(int row, int col, int time) {
            this.row = row;
            this.col = col;
            this.time = time;
        }

        @Override
        public boolean equals(Object obj) {
            if (!(obj instanceof MoveKey)) {
                return false;
            }
            MoveKey other = (MoveKey) obj;
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

    private static final class Trajectory {
        final ArrayList<Integer> agentRows = new ArrayList<>();
        final ArrayList<Integer> agentCols = new ArrayList<>();
        final ArrayList<Integer> boxRows = new ArrayList<>();
        final ArrayList<Integer> boxCols = new ArrayList<>();
        final ArrayList<Boolean> hasBox = new ArrayList<>();

        void add(int agentRow, int agentCol, int boxRow, int boxCol, boolean boxExists) {
            agentRows.add(agentRow);
            agentCols.add(agentCol);
            boxRows.add(boxRow);
            boxCols.add(boxCol);
            hasBox.add(boxExists);
        }

        int agentRowAt(int time) {
            return agentRows.get(Math.min(time, agentRows.size() - 1));
        }

        int agentColAt(int time) {
            return agentCols.get(Math.min(time, agentCols.size() - 1));
        }

        int boxRowAt(int time) {
            return boxRows.get(Math.min(time, boxRows.size() - 1));
        }

        int boxColAt(int time) {
            return boxCols.get(Math.min(time, boxCols.size() - 1));
        }

        boolean boxExistsAt(int time) {
            return hasBox.get(Math.min(time, hasBox.size() - 1));
        }

        boolean movedBoxAt(int time) {
            if (!boxExistsAt(time) || time == 0) {
                return false;
            }
            return boxRowAt(time) != boxRowAt(time - 1) || boxColAt(time) != boxColAt(time - 1);
        }
    }
}
