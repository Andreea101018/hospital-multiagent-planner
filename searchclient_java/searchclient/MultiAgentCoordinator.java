package searchclient;

import java.util.*;

/**
 * Converts per-agent plans into MAvis joint actions.
 *
 * First version: combines actions by time and inserts NoOp for agents without a plan.
 * Improvement: simulate the joint state and detect conflicts before printing each step.
 */
public final class MultiAgentCoordinator {
    public Action[][] coordinate(List<AgentPlan> plans, State initialState) {
        int numAgents = initialState.agentRows.length;
        int horizon = 0;
        for (AgentPlan plan : plans) {
            horizon = Math.max(horizon, plan.length());
        }

        if (horizon == 0) {
            return new Action[0][numAgents];
        }

        Action[][] jointPlan = new Action[horizon][numAgents];
        for (int t = 0; t < horizon; t++) {
            Arrays.fill(jointPlan[t], Action.NoOp);
            for (AgentPlan plan : plans) {
                jointPlan[t][plan.agent] = plan.actionAt(t);
            }
        }
        return jointPlan;
    }
}
