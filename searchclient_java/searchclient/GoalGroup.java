package searchclient;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public final class GoalGroup {
    public final List<Position> goals;
    public final List<Character> letters;
    public final int assignedAgent;

    public GoalGroup(List<Position> goals, List<Character> letters, int assignedAgent) {
        this.goals = Collections.unmodifiableList(new ArrayList<>(goals));
        this.letters = Collections.unmodifiableList(new ArrayList<>(letters));
        this.assignedAgent = assignedAgent;
    }

    public int size() {
        return goals.size();
    }

    @Override
    public String toString() {
        return "GoalGroup{goals=" + goals + ", letters=" + letters + ", agent=" + assignedAgent + "}";
    }
}