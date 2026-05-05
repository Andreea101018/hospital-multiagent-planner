package searchclient;

import java.util.ArrayList;
import java.util.ArrayDeque;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Detects tightly coupled goal groups.
 *
 * First version:
 * - Detect horizontal rows of box goals.
 * - If a row has at least 3 box goals, treat it as a local group.
 *
 * This is useful for levels like help.lvl where several boxes must be arranged
 * into a tight goal corridor.
 */
public final class GoalGroupManager {
    private final LevelAnalyzer analyzer;

    public GoalGroupManager(LevelAnalyzer analyzer) {
        this.analyzer = analyzer;
    }

    public List<GoalGroup> findGoalGroups(State state) {
        ArrayList<GoalGroup> groups = new ArrayList<>();
        HashSet<String> seen = new HashSet<>();

        for (int r = 0; r < State.goals.length; r++) {
            ArrayList<Position> rowGoals = new ArrayList<>();
            ArrayList<Character> rowLetters = new ArrayList<>();

            for (int c = 0; c < State.goals[r].length; c++) {
                char goal = State.goals[r][c];

                if ('A' <= goal && goal <= 'Z' && state.boxes[r][c] != goal) {
                    rowGoals.add(new Position(r, c));
                    rowLetters.add(goal);
                } else {
                    if (rowGoals.size() >= 3) {
                        GoalGroup group = makeGroupIfPossible(state, rowGoals, rowLetters);
                        if (group != null && seen.add(signature(group.goals))) {
                            groups.add(group);
                        }
                    }

                    rowGoals = new ArrayList<>();
                    rowLetters = new ArrayList<>();
                }
            }

            if (rowGoals.size() >= 3) {
                GoalGroup group = makeGroupIfPossible(state, rowGoals, rowLetters);
                if (group != null && seen.add(signature(group.goals))) {
                    groups.add(group);
                }
            }
        }

        for (GoalGroup group : findCompactSameColorClusters(state)) {
            if (seen.add(signature(group.goals))) {
                groups.add(group);
            }
        }

        return groups;
    }

    private List<GoalGroup> findCompactSameColorClusters(State state) {
        ArrayList<GoalGroup> groups = new ArrayList<>();
        ArrayList<Position> goals = new ArrayList<>();

        for (int r = 0; r < State.goals.length; r++) {
            for (int c = 0; c < State.goals[r].length; c++) {
                char goal = State.goals[r][c];

                if ('A' <= goal && goal <= 'Z' && state.boxes[r][c] != goal) {
                    goals.add(new Position(r, c));
                }
            }
        }

        boolean[] visited = new boolean[goals.size()];

        for (int i = 0; i < goals.size(); i++) {
            if (visited[i]) {
                continue;
            }

            char seedLetter = State.goals[goals.get(i).row][goals.get(i).col];
            Color color = State.boxColors[seedLetter - 'A'];
            ArrayList<Position> component = new ArrayList<>();
            ArrayDeque<Integer> queue = new ArrayDeque<>();

            visited[i] = true;
            queue.add(i);

            while (!queue.isEmpty()) {
                int index = queue.removeFirst();
                Position current = goals.get(index);
                component.add(current);

                for (int j = 0; j < goals.size(); j++) {
                    if (visited[j]) {
                        continue;
                    }

                    Position candidate = goals.get(j);
                    char letter = State.goals[candidate.row][candidate.col];

                    if (State.boxColors[letter - 'A'] == color && compactNeighbors(current, candidate)) {
                        visited[j] = true;
                        queue.add(j);
                    }
                }
            }

            if (component.size() < 3 || component.size() > 8) {
                continue;
            }

            component.sort((a, b) -> {
                int byRow = Integer.compare(a.row, b.row);
                return byRow != 0 ? byRow : Integer.compare(a.col, b.col);
            });

            ArrayList<Character> letters = new ArrayList<>();

            for (Position goal : component) {
                letters.add(State.goals[goal.row][goal.col]);
            }

            if (allSameLetter(letters)) {
                continue;
            }

            GoalGroup group = makeGroupIfPossible(state, component, letters);

            if (group != null) {
                groups.add(group);
            }
        }

        return groups;
    }

    private boolean allSameLetter(List<Character> letters) {
        if (letters.isEmpty()) {
            return true;
        }

        char first = letters.get(0);

        for (char letter : letters) {
            if (letter != first) {
                return false;
            }
        }

        return true;
    }

    private boolean compactNeighbors(Position a, Position b) {
        int dr = Math.abs(a.row - b.row);
        int dc = Math.abs(a.col - b.col);
        return dr + dc <= 2 && dr <= 1 && dc <= 2;
    }

    private String signature(List<Position> goals) {
        ArrayList<Position> sorted = new ArrayList<>(goals);
        sorted.sort((a, b) -> {
            int byRow = Integer.compare(a.row, b.row);
            return byRow != 0 ? byRow : Integer.compare(a.col, b.col);
        });

        StringBuilder builder = new StringBuilder();

        for (Position goal : sorted) {
            builder.append(goal.row).append(':').append(goal.col).append(';');
        }

        return builder.toString();
    }

    private GoalGroup makeGroupIfPossible(State state, List<Position> goals, List<Character> letters) {
        int agent = findCommonColorAgent(state, letters, goals);

        if (agent == -1) {
            return null;
        }

        // Only create groups for which all needed letters still exist somewhere.
        Set<Character> needed = new HashSet<>(letters);

        for (char letter : needed) {
            if (!boxExists(state, letter)) {
                return null;
            }
        }

        return new GoalGroup(goals, letters, agent);
    }

    private int findCommonColorAgent(State state, List<Character> letters, List<Position> goals) {
        if (letters.isEmpty()) {
            return -1;
        }

        Color requiredColor = State.boxColors[letters.get(0) - 'A'];

        for (char letter : letters) {
            if (State.boxColors[letter - 'A'] != requiredColor) {
                return -1;
            }
        }

        int bestAgent = -1;
        int bestDistance = LevelAnalyzer.INF;

        for (int agent = 0; agent < state.agentRows.length; agent++) {
            if (State.agentColors[agent] != requiredColor) {
                continue;
            }

            Position agentPos = new Position(state.agentRows[agent], state.agentCols[agent]);

            int nearestGoal = LevelAnalyzer.INF;
            for (Position goal : goals) {
                nearestGoal = Math.min(nearestGoal, analyzer.distance(agentPos, goal));
            }

            if (nearestGoal < bestDistance) {
                bestDistance = nearestGoal;
                bestAgent = agent;
            }
        }

        return bestAgent;
    }

    private boolean boxExists(State state, char letter) {
        for (int r = 0; r < state.boxes.length; r++) {
            for (int c = 0; c < state.boxes[r].length; c++) {
                if (state.boxes[r][c] == letter) {
                    return true;
                }
            }
        }

        return false;
    }
}
