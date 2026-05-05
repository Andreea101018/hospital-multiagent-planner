package searchclient;

import java.util.Objects;

/**
 * Reduced planning state for one assigned agent moving one assigned box.
 *
 * This is deliberately much smaller than the full State used by GraphSearch.
 */
public final class LowLevelState {
    public final Position agentPos;
    public final Position boxPos;
    public final int time;

    public LowLevelState(Position agentPos, Position boxPos, int time) {
        this.agentPos = agentPos;
        this.boxPos = boxPos;
        this.time = time;
    }

    public static LowLevelState from(State globalState, Task task) {
        return new LowLevelState(
                new Position(globalState.agentRows[task.assignedAgent], globalState.agentCols[task.assignedAgent]),
                task.boxStart,
                0
        );
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof LowLevelState)) return false;
        LowLevelState that = (LowLevelState) o;
        return Objects.equals(agentPos, that.agentPos) && Objects.equals(boxPos, that.boxPos);
    }

    @Override
    public int hashCode() {
        return Objects.hash(agentPos, boxPos);
    }
}
