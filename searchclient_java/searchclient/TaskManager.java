package searchclient;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Creates and orders high-level box-goal-agent tasks.
 *
 * Competition idea:
 * - One task per box goal.
 * - One physical box can only be assigned once.
 * - Agents must be color-compatible with the box.
 * - Prefer dependency-aware ordering over pure nearest-first ordering.
 */
public final class TaskManager {
    private static final int MAX_CANDIDATES_PER_GOAL_SMALL_LEVEL = 2;

    private final LevelAnalyzer analyzer;

    public TaskManager(LevelAnalyzer analyzer) {
        this.analyzer = analyzer;
    }

    public List<Task> createTasks(State state) {
        ArrayList<Task> tasks = new ArrayList<>();
        int unsolvedBoxGoals = countUnsolvedBoxGoals(state);
        int maxCandidatesPerGoal = unsolvedBoxGoals == 1
                ? 24
                : (unsolvedBoxGoals <= 4 ? MAX_CANDIDATES_PER_GOAL_SMALL_LEVEL : 1);

        for (int r = 0; r < State.goals.length; r++) {
            for (int c = 0; c < State.goals[r].length; c++) {
                char goal = State.goals[r][c];

                if (!isBoxGoal(goal)) {
                    continue;
                }

                Position goalPos = new Position(r, c);

                // If this goal is already solved, no task is needed.
                if (state.boxes[r][c] == goal) {
                    continue;
                }

                for (Candidate candidate : findCandidatesForGoal(state, goal, goalPos, maxCandidatesPerGoal)) {
                    tasks.add(new Task(goal, candidate.boxPos, goalPos, candidate.agent, candidate.priority));
                }
            }
        }

        return tasks;
    }

    public List<Task> orderTasks(List<Task> tasks) {
        tasks.sort(
                Comparator
                        .comparingInt(this::orderingScore)
                        .thenComparingInt(t -> t.priority)
                        .thenComparingInt(this::sameRowSplitTieBreaker)
                        .thenComparingInt(t -> t.goal.row)
                        .thenComparingInt(t -> t.goal.col)
        );

        return tasks;
    }

    /**
     * Lower score means earlier task.
     *
     * Important hospital-domain rule:
     * In goal rooms / goal rows, solve deeper goals first.
     * For the top-row structure in help.lvl, this means solving the far-left/far-right goals
     * before the goals closer to the central doorway.
     */
    private int orderingScore(Task task) {
        int centerRow = analyzer.rows / 2;
        int centerCol = analyzer.cols / 2;

        int goalDepth =
                Math.abs(task.goal.row - centerRow)
                        + Math.abs(task.goal.col - centerCol);

        // Strong special case: top goal row.
        // This handles levels where boxes must be moved through a doorway into a top goal corridor.
        if (task.goal.row <= 1) {
            return -100_000 - 100 * Math.abs(task.goal.col - centerCol);
        }

        // Right-wall goal rooms behave like storage pockets: place the deepest
        // lower/right boxes first, before upper/left goals close the doorway.
        if (task.goal.col >= analyzer.cols - 3) {
            return -90_000 - 100 * task.goal.row - task.goal.col;
        }

        // General rule: prefer deeper goals, but still respect normal task cost.
        return task.priority - 10 * goalDepth;
    }

    private int sameRowSplitTieBreaker(Task task) {
        int centerCol = analyzer.cols / 2;

        if (Math.abs(task.goal.col - centerCol) < 4) {
            return 0;
        }

        boolean goalOnRight = task.goal.col > centerCol;
        boolean boxOnRight = task.boxStart.col > centerCol;

        if (goalOnRight == boxOnRight) {
            return -100;
        }

        return 100 - task.goal.col;
    }

    private List<Candidate> findCandidatesForGoal(
            State state,
            char goalLetter,
            Position goalPos,
            int maxCandidates
    ) {
        ArrayList<Candidate> candidates = new ArrayList<>();

        for (int r = 0; r < state.boxes.length; r++) {
            for (int c = 0; c < state.boxes[r].length; c++) {
                if (state.boxes[r][c] != goalLetter) {
                    continue;
                }

                Position boxPos = new Position(r, c);

                if (State.goals[r][c] == goalLetter) {
                    continue;
                }

                for (int agent : compatibleAgents(state, goalLetter)) {
                    int boxToGoal = analyzer.distance(boxPos, goalPos);
                    int agentToBox = analyzer.distance(
                            new Position(state.agentRows[agent], state.agentCols[agent]),
                            boxPos
                    );

                    if (boxToGoal >= LevelAnalyzer.INF || agentToBox >= LevelAnalyzer.INF) {
                        continue;
                    }

                    int priority = computePriority(state, goalLetter, boxPos, goalPos, agent, boxToGoal, agentToBox);

                    candidates.add(new Candidate(boxPos, agent, priority, boxToGoal));
                }
            }
        }

        candidates.sort(
                maxCandidates > MAX_CANDIDATES_PER_GOAL_SMALL_LEVEL
                        ? Comparator
                                .comparingInt((Candidate candidate) -> candidate.boxToGoal)
                                .thenComparingInt(candidate -> candidate.priority)
                        : Comparator.comparingInt(candidate -> candidate.priority)
        );

        if (candidates.size() > maxCandidates) {
            return new ArrayList<>(candidates.subList(0, maxCandidates));
        }

        return candidates;
    }

    private int countUnsolvedBoxGoals(State state) {
        int count = 0;

        for (int r = 0; r < State.goals.length; r++) {
            for (int c = 0; c < State.goals[r].length; c++) {
                char goal = State.goals[r][c];

                if (isBoxGoal(goal) && state.boxes[r][c] != goal) {
                    count++;
                }
            }
        }

        return count;
    }

    private List<Integer> compatibleAgents(State state, char boxLetter) {
        ArrayList<Integer> result = new ArrayList<>();
        Color boxColor = State.boxColors[boxLetter - 'A'];

        for (int agent = 0; agent < state.agentRows.length; agent++) {
            if (State.agentColors[agent] == boxColor) {
                result.add(agent);
            }
        }

        return result;
    }

    private int computePriority(
            State state,
            char boxLetter,
            Position boxPos,
            Position goalPos,
            int agent,
            int boxToGoal,
            int agentToBox
    ) {
        int priority = 0;

        // Moving the box matters more than walking the agent.
        priority += 4 * boxToGoal;
        priority += agentToBox;

        // Prefer boxes already near their goal.
        if (boxToGoal <= 2) {
            priority -= 10;
        }

        // For right-wall pockets, a same-side box is usually the intended box.
        // Penalize distant bank boxes heavily so the planner does not seal the
        // pocket with other colors before the reachable right-side box is used.
        if (goalPos.col >= analyzer.cols - 3) {
            int centerCol = analyzer.cols / 2;

            if (boxPos.col < centerCol) {
                priority += 1000;
            } else {
                priority -= 150;
                priority += 50 * Math.abs(goalPos.col - boxPos.col);
            }
        }

        // Avoid moving a box that is already correctly solved.
        if (State.goals[boxPos.row][boxPos.col] == boxLetter) {
            priority += 1000;
        }

        return priority;
    }

    private boolean isBoxGoal(char c) {
        return 'A' <= c && c <= 'Z';
    }

    private static final class Candidate {
        final Position boxPos;
        final int agent;
        final int priority;
        final int boxToGoal;

        Candidate(Position boxPos, int agent, int priority, int boxToGoal) {
            this.boxPos = boxPos;
            this.agent = agent;
            this.priority = priority;
            this.boxToGoal = boxToGoal;
        }
    }
}
