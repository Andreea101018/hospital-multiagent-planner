package searchclient;

final class PlanVerifier {
    private PlanVerifier() {}

    static boolean verifyComplete(State initialState, Action[][] plan) {
        return verifyComplete(initialState, plan, "candidate").passed;
    }

    static PlanVerificationResult verifyComplete(State initialState, Action[][] plan, String label) {
        State current = ProjectSolver.copyState(initialState);

        for (int step = 0; step < plan.length; step++) {
            Action[] jointAction = plan[step];
            if (jointAction == null || jointAction.length != current.agentRows.length) {
                return PlanVerificationResult.rejected(
                        String.format("step %,d: joint action has wrong arity", step)
                );
            }

            for (int agent = 0; agent < jointAction.length; agent++) {
                if (jointAction[agent] == null || !ProjectSolver.isApplicable(current, agent, jointAction[agent])) {
                    int targetRow = current.agentRows[agent] + (jointAction[agent] == null ? 0 : jointAction[agent].agentRowDelta);
                    int targetCol = current.agentCols[agent] + (jointAction[agent] == null ? 0 : jointAction[agent].agentColDelta);
                    char targetBox = inBounds(current, targetRow, targetCol) ? current.boxes[targetRow][targetCol] : 0;
                    boolean targetWall = inBounds(current, targetRow, targetCol) && State.walls[targetRow][targetCol];
                    return PlanVerificationResult.rejected(
                            String.format(
                                    "step %,d: agent %,d action %s is not applicable at (%d,%d), target (%d,%d), wall=%s, box=%s",
                                    step,
                                    agent,
                                    jointAction[agent] == null ? "null" : jointAction[agent].name,
                                    current.agentRows[agent],
                                    current.agentCols[agent],
                                    targetRow,
                                    targetCol,
                                    targetWall ? "yes" : "no",
                                    targetBox == 0 ? "." : Character.toString(targetBox)
                            )
                    );
                }
            }

            if (ProjectSolver.isConflicting(current, jointAction)) {
                return PlanVerificationResult.rejected(
                        String.format("step %,d: joint action is conflicting", step)
                );
            }

            current = ProjectSolver.applyJointAction(current, jointAction);
        }

        if (!current.isGoalState()) {
            return PlanVerificationResult.rejected(
                    String.format("after %,d actions: final state is not a goal state", plan.length)
            );
        }

        return PlanVerificationResult.accepted();
    }

    static boolean verifyPrefix(State initialState, Action[][] plan) {
        State current = ProjectSolver.copyState(initialState);

        for (Action[] jointAction : plan) {
            if (jointAction == null || jointAction.length != current.agentRows.length) {
                return false;
            }

            for (int agent = 0; agent < jointAction.length; agent++) {
                if (jointAction[agent] == null || !ProjectSolver.isApplicable(current, agent, jointAction[agent])) {
                    return false;
                }
            }

            if (ProjectSolver.isConflicting(current, jointAction)) {
                return false;
            }

            current = ProjectSolver.applyJointAction(current, jointAction);
        }

        return true;
    }

    private static boolean inBounds(State state, int row, int col) {
        return row >= 0 && row < state.boxes.length && col >= 0 && col < state.boxes[row].length;
    }
}
