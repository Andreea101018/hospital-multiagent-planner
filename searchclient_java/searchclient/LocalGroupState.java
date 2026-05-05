package searchclient;

import java.util.HashSet;
import java.util.Arrays;
import java.util.Objects;
import java.util.Set;

public final class LocalGroupState {
    public final Position agentPos;
    public final Position[] boxPositions;
    public final int time;

    public LocalGroupState(Position agentPos, Position[] boxPositions, int time) {
        this.agentPos = agentPos;
        this.boxPositions = boxPositions.clone();
        this.time = time;
    }

    public static LocalGroupState from(State state, GoalGroup group) {
        Position agent = new Position(
                state.agentRows[group.assignedAgent],
                state.agentCols[group.assignedAgent]
        );

        Position[] boxes = new Position[group.letters.size()];

        Set<Position> usedBoxes = new HashSet<>();

        for (int i = 0; i < group.letters.size(); i++) {
            boxes[i] = findBox(state, group.letters.get(i), group.goals.get(i), usedBoxes);

            if (boxes[i] != null) {
                usedBoxes.add(boxes[i]);
            }
        }

        return new LocalGroupState(agent, boxes, 0);
    }

    private static Position findBox(State state, char letter, Position goal, Set<Position> usedBoxes) {
        Position best = null;
        int bestDistance = Integer.MAX_VALUE;

        for (int r = 0; r < state.boxes.length; r++) {
            for (int c = 0; c < state.boxes[r].length; c++) {
                if (state.boxes[r][c] == letter) {
                    Position candidate = new Position(r, c);

                    if (usedBoxes.contains(candidate)) {
                        continue;
                    }

                    int distance = Math.abs(candidate.row - goal.row) + Math.abs(candidate.col - goal.col);

                    if (distance < bestDistance) {
                        bestDistance = distance;
                        best = candidate;
                    }
                }
            }
        }

        return best;
    }

    public int boxIndexAt(Position p) {
        for (int i = 0; i < boxPositions.length; i++) {
            if (boxPositions[i] != null && boxPositions[i].equals(p)) {
                return i;
            }
        }

        return -1;
    }

    public LocalGroupState with(Position newAgentPos, Position[] newBoxPositions) {
        return new LocalGroupState(newAgentPos, newBoxPositions, this.time + 1);
    }

    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof LocalGroupState)) {
            return false;
        }

        LocalGroupState other = (LocalGroupState) obj;

        return Objects.equals(this.agentPos, other.agentPos)
                && Arrays.equals(this.boxPositions, other.boxPositions);
    }

    @Override
    public int hashCode() {
        return 31 * Objects.hashCode(this.agentPos) + Arrays.hashCode(this.boxPositions);
    }
}
