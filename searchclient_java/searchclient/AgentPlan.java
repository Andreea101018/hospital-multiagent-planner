package searchclient;

import java.util.*;

public final class AgentPlan {
    public final int agent;
    private final ArrayList<Action> actions;

    public AgentPlan(int agent, List<Action> actions) {
        this.agent = agent;
        this.actions = new ArrayList<>(actions);
    }

    public int length() {
        return actions.size();
    }

    public Action actionAt(int time) {
        if (time < 0 || time >= actions.size()) return Action.NoOp;
        return actions.get(time);
    }

    public List<Action> actions() {
        return Collections.unmodifiableList(actions);
    }
}
