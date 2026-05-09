package searchclient;

import java.util.ArrayList;
import java.util.ArrayDeque;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;

public final class ProjectSolver {
    private static final int MAX_TOTAL_ACTIONS = 19_500;
private static final int MAX_ITERATIONS = 120;
private static final int BEAM_WIDTH = 10;
private static final int TASK_BRANCHING = 8;
private static final Action[] MOVE_ACTIONS = {
        Action.MoveN,
        Action.MoveS,
        Action.MoveE,
        Action.MoveW
};
private static HybridRunStats activeStats = null;

    private ProjectSolver() {}

    public static Action[][] solve(State initialState) {
        long startTime = System.nanoTime();
        System.err.println("Project solver: verified hybrid portfolio planner.");
        long deadline = System.nanoTime() + 170L * 1_000_000_000L;

        LevelAnalyzer analyzer = analyzeLevel(initialState);

        ArrayList<StrategyResult> results = new ArrayList<>();
        results.add(runCbsPrefixStrategy(
                "CBS-first hybrid",
                initialState,
                analyzer,
                deadline,
                8,
                40L
        ));

        StrategyResult fallbackFromInitial = runFallbackStrategy(
                "Heuristic fallback from initial",
                initialState
        );
        results.add(fallbackFromInitial);

        if (System.nanoTime() < deadline) {
            results.add(runConservativeCbsAssistedStrategy(
                    "Conservative CBS-assisted fallback",
                    initialState,
                    analyzer,
                    deadline,
                    3,
                    12L
            ));
        }

        if (System.nanoTime() < deadline) {
            results.add(runCbsPrefixStrategy(
                    "Small CBS-prefix hybrid",
                    initialState,
                    analyzer,
                    deadline,
                    2,
                    15L
            ));
        }

        if (System.nanoTime() < deadline) {
            results.add(fallbackFromInitial.withName("Fallback-first final-agent CBS"));
        }

        StrategyResult best = null;
        boolean cbsContributedToVerifiedCandidate = false;
        for (StrategyResult result : results) {
            printStrategySummary(result);
            if (result.verified && result.cbsContributed()) {
                cbsContributedToVerifiedCandidate = true;
            }
            if (result.verified
                    && (best == null || result.actionCount < best.actionCount)) {
                best = result;
            }
        }

        double totalRuntime = (System.nanoTime() - startTime) / 1_000_000_000.0;
        if (best == null) {
            System.err.format(
                    "Verified portfolio found no complete solution in %.3f seconds; returning empty plan.%n",
                    totalRuntime
            );
            return new Action[0][];
        }

        System.err.format(
                "Verified portfolio winner: %s with %,d action(s) in %.3f total seconds.%n",
                best.name,
                best.actionCount,
                totalRuntime
        );
        System.err.format(
                "CBS contribution: winning=%s, any verified candidate=%s.%n",
                best.cbsContributed() ? "yes" : "no",
                cbsContributedToVerifiedCandidate ? "yes" : "no"
        );
        return best.plan;
    }

    private static StrategyResult runCbsPrefixStrategy(
            String name,
            State initialState,
            LevelAnalyzer analyzer,
            long deadline,
            int maxBoxTasks,
            long cbsSeconds
    ) {
        long startTime = System.nanoTime();
        HybridRunStats stats = new HybridRunStats();
        activeStats = stats;
        long cbsDeadline = Math.min(deadline, System.nanoTime() + cbsSeconds * 1_000_000_000L);
        BoxCBSPlanner.Result cbsResult = BoxCBSPlanner.solveBoxTasks(initialState, analyzer, cbsDeadline, maxBoxTasks);
        stats.cbsBoxAttempted = cbsResult.attemptedBoxTasks;
        stats.cbsBoxSolved = cbsResult.solvedBoxTasks;

        Action[][] candidate = new Action[0][];
        if (cbsResult.solvedBoxTasks > 0 && System.nanoTime() < deadline) {
            stats.fallbackUsed = true;
            Action[][] suffix = heuristicFallbackSolve(cbsResult.state);
            candidate = concatenatePlans(cbsResult.plan, suffix);
        } else if (cbsResult.solvedBoxTasks == 0) {
            stats.rejectionReason = "Box-CBS found no usable prefix";
        } else {
            stats.rejectionReason = "deadline reached before fallback suffix";
        }

        PlanVerificationResult verification = PlanVerifier.verifyComplete(initialState, candidate, name);
        stats.verificationPassed = candidate.length <= MAX_TOTAL_ACTIONS && verification.passed;
        if (!stats.verificationPassed && stats.rejectionReason == null) {
            stats.rejectionReason = candidate.length > MAX_TOTAL_ACTIONS
                    ? "plan exceeds action budget"
                    : verification.reason;
        }
        return StrategyResult.from(name, candidate, stats, startTime);
    }

    private static StrategyResult runConservativeCbsAssistedStrategy(
            String name,
            State initialState,
            LevelAnalyzer analyzer,
            long deadline,
            int maxBoxTasks,
            long cbsSeconds
    ) {
        long startTime = System.nanoTime();
        HybridRunStats stats = new HybridRunStats();
        activeStats = stats;
        long cbsDeadline = Math.min(deadline, System.nanoTime() + cbsSeconds * 1_000_000_000L);
        BoxCBSPlanner.Result cbsResult = BoxCBSPlanner.solveConservativeBoxTasks(
                initialState,
                analyzer,
                cbsDeadline,
                maxBoxTasks
        );
        stats.cbsBoxAttempted = cbsResult.attemptedBoxTasks;
        stats.cbsBoxSolved = cbsResult.solvedBoxTasks;

        Action[][] candidate = new Action[0][];
        if (cbsResult.solvedBoxTasks == 0) {
            stats.rejectionReason = "conservative Box-CBS found no safe prefix";
        } else if (!isPrefixValid(initialState, cbsResult.plan)) {
            stats.rejectionReason = "conservative Box-CBS prefix failed validation";
        } else if (System.nanoTime() < deadline) {
            stats.fallbackUsed = true;
            Action[][] suffix = heuristicFallbackSolve(cbsResult.state);
            candidate = concatenatePlans(cbsResult.plan, suffix);
        } else {
            stats.rejectionReason = "deadline reached before fallback suffix";
        }

        PlanVerificationResult verification = PlanVerifier.verifyComplete(initialState, candidate, name);
        stats.verificationPassed = candidate.length <= MAX_TOTAL_ACTIONS && verification.passed;
        if (!stats.verificationPassed && stats.rejectionReason == null) {
            stats.rejectionReason = candidate.length > MAX_TOTAL_ACTIONS
                    ? "plan exceeds action budget"
                    : verification.reason;
        }
        return StrategyResult.from(name, candidate, stats, startTime);
    }

    private static StrategyResult runFallbackStrategy(String name, State initialState) {
        long startTime = System.nanoTime();
        HybridRunStats stats = new HybridRunStats();
        activeStats = stats;
        stats.fallbackUsed = true;
        Action[][] candidate = heuristicFallbackSolve(initialState);
        PlanVerificationResult verification = PlanVerifier.verifyComplete(initialState, candidate, name);
        stats.verificationPassed = candidate.length <= MAX_TOTAL_ACTIONS && verification.passed;
        if (!stats.verificationPassed) {
            stats.rejectionReason = candidate.length > MAX_TOTAL_ACTIONS
                    ? "plan exceeds action budget"
                    : verification.reason;
        }
        return StrategyResult.from(name, candidate, stats, startTime);
    }

    private static Action[][] heuristicFallbackSolve(State initialState) {
        System.err.println("Heuristic fallback solver: beam-search hierarchical task planner.");
        long deadline = System.nanoTime() + 170L * 1_000_000_000L;

        LevelAnalyzer analyzer = analyzeLevel(initialState);

        Action[][] openingRepair = tryInitialBoundedGlobalRepair(initialState, analyzer, deadline);
        if (openingRepair != null) {
            return openingRepair;
        }

        TaskManager taskManager = new TaskManager(analyzer);
        GoalGroupManager groupManager = new GoalGroupManager(analyzer);
        SingleAgentPlanner planner = new SingleAgentPlanner(analyzer);
        AgentRetreatPlanner retreatPlanner = new AgentRetreatPlanner(analyzer);
        LocalGroupPlanner groupPlanner = new LocalGroupPlanner(analyzer);

        List<GoalGroup> groups = groupManager.findGoalGroups(initialState);
        System.err.format("Detected %,d goal group(s).%n", groups.size());
        for (GoalGroup group : groups) {
            System.err.println("  " + group);
        }

        State groupState = copyState(initialState);
        ArrayList<Action[]> groupPlan = new ArrayList<>();

        for (GoalGroup group : groups) {
            if (System.nanoTime() > deadline) {
                break;
            }

            if (isCompactRoomGroup(group)) {
                PocketResult compactResult = tryCompactGroupSequentialPlanner(
                        groupState,
                        groupPlan,
                        analyzer,
                        planner,
                        retreatPlanner,
                        group,
                        deadline
                );

                if (compactResult != null && countSolvedGoals(compactResult.state) > countSolvedGoals(groupState)) {
                    groupState = compactResult.state;
                    groupPlan = compactResult.plan;
                    System.err.format(
                            "Compact room sequential planner committed progress. Solved goals now %,d.%n",
                            countSolvedGoals(groupState)
                    );

                    if (groupState.isGoalState()) {
                        return groupPlan.toArray(new Action[0][]);
                    }

                    continue;
                }
            }

            System.err.println("Trying local group planner for " + group);

            List<Action> localPlan = groupPlanner.planGroup(groupState, group);

            if (localPlan == null || localPlan.isEmpty()) {
                System.err.println("Local group planner could not solve " + group);
                continue;
            }

            if (groupPlan.size() + localPlan.size() > MAX_TOTAL_ACTIONS) {
                System.err.println("Local group plan skipped because it would exceed the action budget.");
                continue;
            }

            appendAsJointActions(groupPlan, localPlan, group.assignedAgent, groupState.agentRows.length);
            groupState = simulateSingleAgentPlan(groupState, localPlan, group.assignedAgent);

            System.err.format(
                    "Local group planner committed %,d actions. Solved goals now %,d.%n",
                    localPlan.size(),
                    countSolvedGoals(groupState)
            );

            List<Action> retreatPlan = retreatPlanner.planRetreat(groupState, group.assignedAgent, 20);

            if (retreatPlan != null
                    && !retreatPlan.isEmpty()
                    && groupPlan.size() + retreatPlan.size() <= MAX_TOTAL_ACTIONS) {
                appendAsJointActions(groupPlan, retreatPlan, group.assignedAgent, groupState.agentRows.length);
                groupState = simulateSingleAgentPlan(groupState, retreatPlan, group.assignedAgent);
            }
        }

        if (groupState.isGoalState()) {
            System.err.format("Group planner solved level with %,d actions.%n", groupPlan.size());
            return groupPlan.toArray(new Action[0][]);
        }

        boolean compactRoomProgress = true;

        while (compactRoomProgress && !groupState.isGoalState() && System.nanoTime() < deadline) {
            compactRoomProgress = false;

            for (GoalGroup group : groups) {
                if (!isCompactRoomGroup(group) || System.nanoTime() > deadline) {
                    continue;
                }

                PocketResult compactResult = tryCompactGroupSequentialPlanner(
                        groupState,
                        groupPlan,
                        analyzer,
                        planner,
                        retreatPlanner,
                        group,
                        deadline
                );

                if (compactResult != null && countSolvedGoals(compactResult.state) > countSolvedGoals(groupState)) {
                    groupState = compactResult.state;
                    groupPlan = compactResult.plan;
                    compactRoomProgress = true;
                    System.err.format(
                            "Compact room follow-up pass improved solved goals to %,d.%n",
                            countSolvedGoals(groupState)
                    );
                    break;
                }
            }
        }

        boolean compactRepairProgress = true;

        while (compactRepairProgress && !groupState.isGoalState() && System.nanoTime() < deadline) {
            compactRepairProgress = false;

            for (GoalGroup group : groups) {
                if (!isCompactRoomGroup(group) || compactGroupSolved(groupState, group) || System.nanoTime() > deadline) {
                    continue;
                }

                long compactDeadline = Math.min(deadline, System.nanoTime() + 25L * 1_000_000_000L);
                Action[][] compactRepair = compactRoomRepair(
                        groupState,
                        analyzer,
                        group,
                        buildCompactRoomRegion(groupState, group),
                        compactDeadline,
                        3_000_000,
                        180
                );

                if (compactRepair != null
                        && compactRepair.length > 0
                        && groupPlan.size() + compactRepair.length <= MAX_TOTAL_ACTIONS) {
                    State repairedState = simulateJointPlan(groupState, compactRepair);

                    if (countSolvedGoals(repairedState) <= countSolvedGoals(groupState)) {
                        System.err.format(
                                "Compact room repair skipped because it did not improve total solved goals (%,d -> %,d).%n",
                                countSolvedGoals(groupState),
                                countSolvedGoals(repairedState)
                        );
                        continue;
                    }

                    appendJointPlan(groupPlan, compactRepair);
                    groupState = repairedState;
                    compactRepairProgress = true;
                    System.err.format(
                            "Compact room repair finished a pocket using %,d actions. Solved goals now %,d.%n",
                            compactRepair.length,
                            countSolvedGoals(groupState)
                    );
                    break;
                }
            }
        }

        Action[][] preBoundaryEvacuation = preBoundaryLongAgentEvacuation(
                groupState,
                analyzer,
                new AgentGoalPlanner(),
                deadline
        );

        if (preBoundaryEvacuation != null
                && preBoundaryEvacuation.length > 0
                && groupPlan.size() + preBoundaryEvacuation.length <= MAX_TOTAL_ACTIONS) {
            appendJointPlan(groupPlan, preBoundaryEvacuation);
            groupState = simulateJointPlan(groupState, preBoundaryEvacuation);
            System.err.format("Pre-boundary evacuation moved agents using %,d actions.%n", preBoundaryEvacuation.length);
        }

        PocketResult pocketResult = isTightManyColorCorridorCandidate(groupState)
                ? null
                : tryBoundaryPocketPlanner(
                        groupState,
                        groupPlan,
                        analyzer,
                        planner,
                        retreatPlanner,
                        deadline
                );

        if (pocketResult != null && countSolvedGoals(pocketResult.state) > countSolvedGoals(groupState)) {
            groupState = pocketResult.state;
            groupPlan = pocketResult.plan;
            System.err.format("Boundary pocket planner made progress. Solved goals now %,d.%n", countSolvedGoals(groupState));

            if (groupState.isGoalState()) {
                System.err.format("Boundary pocket planner solved level with %,d actions.%n", groupPlan.size());
                return groupPlan.toArray(new Action[0][]);
            }
        }

        PriorityQueue<SearchNode> beam = new PriorityQueue<>(Comparator.comparingInt(SearchNode::score));
        beam.add(new SearchNode(
        copyState(groupState),
        copyPlan(groupPlan),
        countSolvedGoals(groupState),
        estimateRemainingCost(analyzer, groupState)
));

        SearchNode bestNode = beam.peek();

        for (int iteration = 0; iteration < MAX_ITERATIONS; iteration++) {
            if (System.nanoTime() > deadline) {
    System.err.println("Project solver time budget reached.");
    break;
}
            PriorityQueue<SearchNode> nextBeam = new PriorityQueue<>(Comparator.comparingInt(SearchNode::score));

            System.err.format("Beam iteration %,d with %,d candidate state(s).%n", iteration, beam.size());

            while (!beam.isEmpty()) {
                if (System.nanoTime() > deadline) {
    break;
}
                SearchNode node = beam.poll();

                if (node.state.isGoalState()) {
                    System.err.format("Project solver solved level with %,d joint actions.%n", node.plan.size());
                    return node.plan.toArray(new Action[0][]);
                }

                if (isBetterPartial(node, bestNode)) {
                    bestNode = node;
                }

                List<Task> tasks = taskManager.orderTasks(taskManager.createTasks(node.state));

                if (tasks.isEmpty()) {
                    continue;
                }

int created = 0;

for (Task task : selectExpansionTasks(tasks)) {
    if (created >= TASK_BRANCHING + 3) {
        break;
    }

                    //System.err.println("Trying " + task);

                    List<Action> singleAgentPlan = planner.planBoxToGoal(
                            node.state,
                            task,
                            new ReservationTable()
                    );

                    if (singleAgentPlan == null || singleAgentPlan.isEmpty()) {
                        continue;
                    }

                    if (node.plan.size() + singleAgentPlan.size() > MAX_TOTAL_ACTIONS) {
                        continue;
                    }

                    ArrayList<Action[]> newPlan = copyPlan(node.plan);
                    appendAsJointActions(newPlan, singleAgentPlan, task.assignedAgent, node.state.agentRows.length);

                    State newState = simulateSingleAgentPlan(node.state, singleAgentPlan, task.assignedAgent);

                    // Move the active agent away from the solved box/goal area if possible.
                    List<Action> retreatPlan = retreatPlanner.planRetreat(newState, task.assignedAgent, 20);

                    if (!retreatPlan.isEmpty()
                            && newPlan.size() + retreatPlan.size() <= MAX_TOTAL_ACTIONS) {
                        appendAsJointActions(newPlan, retreatPlan, task.assignedAgent, newState.agentRows.length);
                        newState = simulateSingleAgentPlan(newState, retreatPlan, task.assignedAgent);
                    }

                    int solvedGoals = countSolvedGoals(newState);
                    int estimate = estimateRemainingCost(analyzer, newState);

                    SearchNode child = new SearchNode(newState, newPlan, solvedGoals, estimate);
                    nextBeam.add(child);
                    created++;

// System.err.format(
//         "Created candidate: solvedGoals=%,d, planLength=%,d, estimate=%,d.%n",
//         solvedGoals,
//         newPlan.size(),
//         estimate
// );
                }
            }

            if (nextBeam.isEmpty()) {
                System.err.println("Beam search has no more candidates.");
                break;
            }

            beam = trimBeam(nextBeam, BEAM_WIDTH);
        }

        if (bestNode != null && bestNode.state.isGoalState()) {
            return bestNode.plan.toArray(new Action[0][]);
        }

System.err.println("Project beam solver did not finish. Trying final box-goal cleanup first.");

if (bestNode == null) {
    return new Action[0][];
}

ArrayList<Action[]> cleanedPlan = copyPlan(bestNode.plan);
State cleanedState = copyState(bestNode.state);

AgentGoalPlanner agentGoalPlanner = new AgentGoalPlanner();
boolean triedMixedGoalRoomRepair = false;

if (countUnsolvedBoxGoals(cleanedState) == 1
        && isTightCorridorMap(cleanedState)
        && System.nanoTime() < deadline) {
    triedMixedGoalRoomRepair = true;
    System.err.println("Trying mixed-color goal-room repair before parking agents.");

    Action[][] evacuationPlan = preSealAgentEvacuation(cleanedState, analyzer, agentGoalPlanner, deadline);

    if (evacuationPlan != null && evacuationPlan.length > 0
            && cleanedPlan.size() + evacuationPlan.length <= MAX_TOTAL_ACTIONS) {
        appendJointPlan(cleanedPlan, evacuationPlan);
        cleanedState = simulateJointPlan(cleanedState, evacuationPlan);
        System.err.format("Pre-seal agent evacuation moved agents using %,d actions.%n", evacuationPlan.length);
    }

    List<Action> approachPlan = planTargetAgentToNearestBox(cleanedState, analyzer);

    if (approachPlan != null && !approachPlan.isEmpty()) {
        UnsolvedBoxGoal target = findOnlyUnsolvedBoxGoal(cleanedState);
        int targetAgent = firstAgentWithColor(State.boxColors[target.letter - 'A']);

        if (targetAgent != -1 && cleanedPlan.size() + approachPlan.size() <= MAX_TOTAL_ACTIONS) {
            appendAsJointActions(cleanedPlan, approachPlan, targetAgent, cleanedState.agentRows.length);
            cleanedState = simulateSingleAgentPlan(cleanedState, approachPlan, targetAgent);
            System.err.format("Moved target-color agent %,d near final box using %,d actions.%n", targetAgent, approachPlan.size());
        }
    }

    long repairDeadline = Math.min(deadline, System.nanoTime() + 18L * 1_000_000_000L);
    Action[][] repairPlan = mixedGoalRoomRepair(cleanedState, analyzer, repairDeadline, 650_000);

    if (repairPlan != null && cleanedPlan.size() + repairPlan.length <= MAX_TOTAL_ACTIONS) {
        appendJointPlan(cleanedPlan, repairPlan);
        cleanedState = simulateJointPlan(cleanedState, repairPlan);
        System.err.format("Mixed-color goal-room repair solved box room before parking with %,d actions.%n", repairPlan.length);
    }
}

if (cleanedState.agentRows.length > 3
        && countUnsolvedBoxGoals(cleanedState) > 0
        && countUnsolvedGoals(cleanedState) > 8) {
    System.err.println("Moving agents to parking before final box cleanup.");

    for (int agent = 0; agent < cleanedState.agentRows.length; agent++) {
        List<Action> retreatPlan = retreatPlanner.planRetreat(cleanedState, agent, 30);

        if (retreatPlan != null && !retreatPlan.isEmpty()
                && cleanedPlan.size() + retreatPlan.size() <= MAX_TOTAL_ACTIONS) {
            appendAsJointActions(cleanedPlan, retreatPlan, agent, cleanedState.agentRows.length);
            cleanedState = simulateSingleAgentPlan(cleanedState, retreatPlan, agent);

            System.err.format(
                    "Pre-cleanup retreat moved agent %,d using %,d actions.%n",
                    agent,
                    retreatPlan.size()
            );
        }
    }
} else {
    System.err.println("Skipping pre-cleanup parking.");
}
// ------------------------------------------------------------
// 1. Final box cleanup FIRST.
// Important: do this before moving agents to their numbered goals,
// because parked agents can block remaining box paths.
// ------------------------------------------------------------
boolean boxCleanupProgress = true;

while (boxCleanupProgress && !cleanedState.isGoalState()) {
    boxCleanupProgress = false;

    List<Task> remainingTasks = taskManager.orderTasks(taskManager.createTasks(cleanedState));

    System.err.format("Final box cleanup sees %,d remaining task(s).%n", remainingTasks.size());

for (Task originalTask : remainingTasks) {
    System.err.println("Final box cleanup original task: " + originalTask);

    PocketResult blockerClear = clearWrongGoalBlocker(
            cleanedState,
            cleanedPlan,
            originalTask,
            planner,
            deadline
    );

    if (blockerClear != null) {
        cleanedState = blockerClear.state;
        cleanedPlan = blockerClear.plan;
        boxCleanupProgress = true;
        break;
    }

    for (Task task : expandTaskAgents(originalTask, cleanedState)) {
        System.err.println("  Trying final cleanup with agent " + task.assignedAgent);

List<Action> boxPlan = planner.planBoxToGoal(
        cleanedState,
        task,
        new ReservationTable()
);

        if (boxPlan == null || boxPlan.isEmpty()) {
            boxPlan = planner.planBoxToGoalRelaxed(
                    cleanedState,
                    task,
                    new ReservationTable()
            );
        }

        if (boxPlan == null || boxPlan.isEmpty()) {
            continue;
        }

        if (cleanedPlan.size() + boxPlan.size() > MAX_TOTAL_ACTIONS) {
            continue;
        }

        appendAsJointActions(cleanedPlan, boxPlan, task.assignedAgent, cleanedState.agentRows.length);
        cleanedState = simulateSingleAgentPlan(cleanedState, boxPlan, task.assignedAgent);

        List<Action> retreatPlan = retreatPlanner.planRetreat(cleanedState, task.assignedAgent, 20);

        if (retreatPlan != null && !retreatPlan.isEmpty()
                && cleanedPlan.size() + retreatPlan.size() <= MAX_TOTAL_ACTIONS) {
            appendAsJointActions(cleanedPlan, retreatPlan, task.assignedAgent, cleanedState.agentRows.length);
            cleanedState = simulateSingleAgentPlan(cleanedState, retreatPlan, task.assignedAgent);
        }

        System.err.format(
                "Final box cleanup solved task %s using %,d actions. Solved goals now %,d.%n",
                task,
                boxPlan.size(),
                countSolvedGoals(cleanedState)
        );

        boxCleanupProgress = true;
        break;
    }

    if (boxCleanupProgress) {
        break;
    }
}
}

if (countUnsolvedBoxGoals(cleanedState) >= 1
        && System.nanoTime() < deadline) {
    PocketResult verifiedSmallEndgame = tryVerifiedSmallEndgameBoxRepair(
            cleanedState,
            cleanedPlan,
            analyzer,
            planner,
            retreatPlanner,
            deadline
    );

    if (verifiedSmallEndgame != null
            && (countUnsolvedBoxGoals(verifiedSmallEndgame.state) < countUnsolvedBoxGoals(cleanedState)
            || countSolvedGoals(verifiedSmallEndgame.state) > countSolvedGoals(cleanedState))) {
        cleanedState = verifiedSmallEndgame.state;
        cleanedPlan = verifiedSmallEndgame.plan;
        System.err.format(
                "Verified small-endgame repair accepted. Solved goals %,d, unsolved box goals %,d.%n",
                countSolvedGoals(cleanedState),
                countUnsolvedBoxGoals(cleanedState)
        );

        if (cleanedState.isGoalState()) {
            return cleanedPlan.toArray(new Action[0][]);
        }
    }
}

if (!triedMixedGoalRoomRepair && countUnsolvedBoxGoals(cleanedState) == 1 && System.nanoTime() < deadline) {
    System.err.println("Trying mixed-color goal-room repair before agent cleanup.");

    long agentRepairSeconds = countUnsolvedAgentGoals(cleanedState) <= 3 ? 24L : 8L;
    long repairDeadline = Math.min(deadline, System.nanoTime() + agentRepairSeconds * 1_000_000_000L);
    Action[][] repairPlan = mixedGoalRoomRepair(cleanedState, analyzer, repairDeadline, 160_000);

    if (repairPlan != null && cleanedPlan.size() + repairPlan.length <= MAX_TOTAL_ACTIONS) {
        appendJointPlan(cleanedPlan, repairPlan);
        cleanedState = simulateJointPlan(cleanedState, repairPlan);
        System.err.format("Mixed-color goal-room repair solved box room with %,d actions.%n", repairPlan.length);
    }
}

// ------------------------------------------------------------
// 2. Agent-goal cleanup SECOND.
// Once boxes are done or no more box progress is possible,
// move agents to their numbered goals.
// ------------------------------------------------------------
boolean agentCleanupProgress = true;

while (agentCleanupProgress && !cleanedState.isGoalState()) {
    agentCleanupProgress = false;

    if (countUnsolvedBoxGoals(cleanedState) == 0 && System.nanoTime() < deadline) {
        State verifiedPrefixState = simulateValidatedJointPlan(initialState, cleanedPlan);
        if (verifiedPrefixState == null) {
            System.err.println("Skipping final CBS: committed prefix is not valid from the original state.");
        } else {
            cleanedState = verifiedPrefixState;
            Action[][] cbsPlan = solveRemainingAgentsWithCBS(cleanedState, analyzer, deadline);

            if (cbsPlan != null
                    && cleanedPlan.size() + cbsPlan.length <= MAX_TOTAL_ACTIONS) {
                State cbsState = simulateJointPlan(cleanedState, cbsPlan);

                if (cbsState.isGoalState()) {
                    if (activeStats != null) {
                        activeStats.finalAgentCBSUsed = true;
                    }
                    appendJointPlan(cleanedPlan, cbsPlan);
                    System.err.format("CBS solved final agent-goal cleanup with %,d actions.%n", cbsPlan.length);
                    return cleanedPlan.toArray(new Action[0][]);
                }
            }
        }

        long coordinatedDeadline = Math.min(deadline, System.nanoTime() + 4L * 1_000_000_000L);
        Action[][] coordinatedPlan = moveOnlyAgentGoalRepair(cleanedState, analyzer, coordinatedDeadline, 650_000);

        if (coordinatedPlan != null
                && cleanedPlan.size() + coordinatedPlan.length <= MAX_TOTAL_ACTIONS) {
            State coordinatedState = simulateJointPlan(cleanedState, coordinatedPlan);

            if (coordinatedState.isGoalState()) {
                appendJointPlan(cleanedPlan, coordinatedPlan);
                System.err.format("Dependency-aware agent MAPF solved suffix with %,d actions.%n", coordinatedPlan.length);
                return cleanedPlan.toArray(new Action[0][]);
            }
        }

        Action[][] displacementPlan = agentGoalOccupantRepair(cleanedState, analyzer, deadline);

        if (displacementPlan != null
                && displacementPlan.length > 0
                && cleanedPlan.size() + displacementPlan.length <= MAX_TOTAL_ACTIONS) {
            appendJointPlan(cleanedPlan, displacementPlan);
            cleanedState = simulateJointPlan(cleanedState, displacementPlan);
            agentCleanupProgress = true;
            System.err.format(
                    "Agent goal displacement repair used %,d actions. Solved goals now %,d.%n",
                    displacementPlan.length,
                    countSolvedGoals(cleanedState)
            );
            continue;
        }
    }

    ArrayList<Integer> cleanupOrder = countUnsolvedBoxGoals(cleanedState) == 0
            ? dependencyAwareAgentCleanupOrder(cleanedState, analyzer)
            : agentCleanupOrder(cleanedState);

    for (int agent : cleanupOrder) {
        if (countUnsolvedBoxGoals(cleanedState) > 0 && agentOwnsRemainingBoxColor(cleanedState, agent)) {
            continue;
        }

        List<Action> agentPlan = agentGoalPlanner.planAgentToGoal(cleanedState, agent);

        if (agentPlan != null && !agentPlan.isEmpty()
                && cleanedPlan.size() + agentPlan.size() <= MAX_TOTAL_ACTIONS) {
            appendAsJointActions(cleanedPlan, agentPlan, agent, cleanedState.agentRows.length);
            cleanedState = simulateSingleAgentPlan(cleanedState, agentPlan, agent);
            agentCleanupProgress = true;

            System.err.format(
                    "Agent cleanup moved agent %,d using %,d actions. Solved goals now %,d.%n",
                    agent,
                    agentPlan.size(),
                    countSolvedGoals(cleanedState)
            );

            if (cleanedState.isGoalState()) {
                break;
            }
        } else if (countUnsolvedBoxGoals(cleanedState) == 0 && System.nanoTime() < deadline) {
            long pushDeadline = Math.min(deadline, System.nanoTime() + 4L * 1_000_000_000L);
            Action[][] pushPlan = agentGoalPushRepair(cleanedState, agent, pushDeadline, 80_000);

            if (pushPlan != null && pushPlan.length > 0
                    && cleanedPlan.size() + pushPlan.length <= MAX_TOTAL_ACTIONS) {
                appendJointPlan(cleanedPlan, pushPlan);
                cleanedState = simulateJointPlan(cleanedState, pushPlan);
                agentCleanupProgress = true;

                System.err.format(
                        "Agent cleanup with pushing moved agent %,d using %,d actions. Solved goals now %,d.%n",
                        agent,
                        pushPlan.length,
                        countSolvedGoals(cleanedState)
                );

                if (cleanedState.isGoalState()) {
                    break;
                }

                break;
            }
        }
    }
}

if (cleanedState.isGoalState()) {
    System.err.format("Final cleanup solved level. Total length %,d.%n", cleanedPlan.size());
    return cleanedPlan.toArray(new Action[0][]);
}

System.err.println("Cleanup failed.");
printUnsolvedGoals(cleanedState);

if (countUnsolvedBoxGoals(cleanedState) > 0
        && countUnsolvedBoxGoals(cleanedState) <= 3
        && countUnsolvedGoals(cleanedState) <= 8
        && cleanedState.agentRows.length <= 5
        && System.nanoTime() < deadline) {
    System.err.println("Trying bounded mixed endgame repair.");

    long repairDeadline = Math.min(deadline, System.nanoTime() + 30L * 1_000_000_000L);
    Action[][] repairPlan = boundedGlobalRepair(cleanedState, analyzer, repairDeadline, 2_500_000);

    if (repairPlan != null && cleanedPlan.size() + repairPlan.length <= MAX_TOTAL_ACTIONS) {
        appendJointPlan(cleanedPlan, repairPlan);
        System.err.format("Bounded mixed endgame repair solved suffix with %,d actions.%n", repairPlan.length);
        return cleanedPlan.toArray(new Action[0][]);
    }
}

if (countUnsolvedBoxGoals(cleanedState) > 0
        && countUnsolvedBoxGoals(cleanedState) <= 2
        && System.nanoTime() < deadline) {
    System.err.println("Trying bounded small-remaining-box repair.");

    PocketResult smallBoxRepair = trySmallRemainingBoxRepair(
            cleanedState,
            cleanedPlan,
            analyzer,
            deadline
    );

    if (smallBoxRepair != null && countSolvedGoals(smallBoxRepair.state) > countSolvedGoals(cleanedState)) {
        cleanedState = smallBoxRepair.state;
        cleanedPlan = smallBoxRepair.plan;
        System.err.format(
                "Small remaining-box repair improved solved goals to %,d.%n",
                countSolvedGoals(cleanedState)
        );

        if (cleanedState.isGoalState()) {
            return cleanedPlan.toArray(new Action[0][]);
        }
    }
}

if (countUnsolvedBoxGoals(cleanedState) == 1
        && isTightCorridorMap(cleanedState)
        && System.nanoTime() < deadline) {
    System.err.println("Trying nearby solved-box unblock repair.");

    PocketResult unblockRepair = tryNearbySolvedBoxUnblockRepair(
            cleanedState,
            cleanedPlan,
            analyzer,
            planner,
            deadline
    );

    if (unblockRepair != null && countSolvedGoals(unblockRepair.state) > countSolvedGoals(cleanedState)) {
        cleanedState = unblockRepair.state;
        cleanedPlan = unblockRepair.plan;
        System.err.format(
                "Nearby solved-box unblock repair improved solved goals to %,d.%n",
                countSolvedGoals(cleanedState)
        );

        if (cleanedState.isGoalState()) {
            return cleanedPlan.toArray(new Action[0][]);
        }
    }
}

if (countUnsolvedBoxGoals(cleanedState) == 1 && System.nanoTime() < deadline) {
    System.err.println("Trying adjacent goal-pair unblock repair.");

    PocketResult adjacentRepair = tryAdjacentGoalPairUnblockRepair(
            cleanedState,
            cleanedPlan,
            analyzer,
            planner,
            deadline
    );

    if (adjacentRepair != null && countSolvedGoals(adjacentRepair.state) > countSolvedGoals(cleanedState)) {
        cleanedState = adjacentRepair.state;
        cleanedPlan = adjacentRepair.plan;
        System.err.format(
                "Adjacent goal-pair unblock repair improved solved goals to %,d.%n",
                countSolvedGoals(cleanedState)
        );

        if (cleanedState.isGoalState()) {
            return cleanedPlan.toArray(new Action[0][]);
        }
    }
}

if (countUnsolvedBoxGoals(cleanedState) == 0
        && countUnsolvedGoals(cleanedState) <= 8
        && System.nanoTime() < deadline) {
    System.err.println("Trying move-only multi-agent goal repair.");

    long agentRepairSeconds = countUnsolvedAgentGoals(cleanedState) <= 3 ? 24L : 8L;
    long repairDeadline = Math.min(deadline, System.nanoTime() + agentRepairSeconds * 1_000_000_000L);
    Action[][] repairPlan = agentSwapRepair(cleanedState, analyzer, repairDeadline);

    if (repairPlan == null) {
        repairPlan = prioritizedAgentGoalRepair(cleanedState, repairDeadline, 20_000);
    }

    if (repairPlan == null) {
        int moveOnlyCap = countUnsolvedAgentGoals(cleanedState) <= 3 ? 1_800_000 : 300_000;
        repairPlan = moveOnlyAgentGoalRepair(cleanedState, analyzer, repairDeadline, moveOnlyCap);
    }

    if (repairPlan != null) {
        State repairedState = simulateJointPlan(cleanedState, repairPlan);

        if (!repairedState.isGoalState() && System.nanoTime() < repairDeadline) {
            int moveOnlyCap = countUnsolvedAgentGoals(repairedState) <= 3 ? 1_800_000 : 300_000;
            Action[][] tail = moveOnlyAgentGoalRepair(repairedState, analyzer, repairDeadline, moveOnlyCap);

            if (tail != null) {
                ArrayList<Action[]> combinedRepair = new ArrayList<>();
                appendJointPlan(combinedRepair, repairPlan);
                appendJointPlan(combinedRepair, tail);
                repairPlan = combinedRepair.toArray(new Action[0][]);
            }
        }
    }

    if (repairPlan != null && cleanedPlan.size() + repairPlan.length <= MAX_TOTAL_ACTIONS) {
        State repairedState = simulateJointPlan(cleanedState, repairPlan);

        if (repairedState.isGoalState()) {
            appendJointPlan(cleanedPlan, repairPlan);
            System.err.format("Move-only agent repair solved final state with %,d actions.%n", repairPlan.length);
            return cleanedPlan.toArray(new Action[0][]);
        }
    }
}

if (countUnsolvedBoxGoals(cleanedState) == 1
        && System.nanoTime() < deadline) {
    System.err.println("Trying focused one-box multi-agent repair search.");

    long repairDeadline = Math.min(deadline, System.nanoTime() + 10L * 1_000_000_000L);
    Action[][] repairPlan = focusedOneBoxRepair(cleanedState, analyzer, repairDeadline, 120_000);

    if (repairPlan != null && cleanedPlan.size() + repairPlan.length <= MAX_TOTAL_ACTIONS) {
        appendJointPlan(cleanedPlan, repairPlan);
        cleanedState = simulateJointPlan(cleanedState, repairPlan);
        System.err.format("Focused one-box repair placed the final box with %,d actions.%n", repairPlan.length);

        boolean postRepairAgentProgress = true;

        while (postRepairAgentProgress && !cleanedState.isGoalState()) {
            postRepairAgentProgress = false;

            for (int agent = 0; agent < cleanedState.agentRows.length; agent++) {
                List<Action> agentPlan = agentGoalPlanner.planAgentToGoal(cleanedState, agent);

                if (agentPlan != null && !agentPlan.isEmpty()
                        && cleanedPlan.size() + agentPlan.size() <= MAX_TOTAL_ACTIONS) {
                    appendAsJointActions(cleanedPlan, agentPlan, agent, cleanedState.agentRows.length);
                    cleanedState = simulateSingleAgentPlan(cleanedState, agentPlan, agent);
                    postRepairAgentProgress = true;

                    if (cleanedState.isGoalState()) {
                        break;
                    }
                }
            }
        }

        if (cleanedState.isGoalState()) {
            System.err.format("Focused one-box repair solved final state. Total length %,d.%n", cleanedPlan.size());
            return cleanedPlan.toArray(new Action[0][]);
        }
    }

    if (!cleanedState.isGoalState() && cleanedState.agentRows.length <= 3 && System.nanoTime() < deadline) {
        System.err.println("Trying compact one-box global repair.");
        long compactDeadline = Math.min(deadline, System.nanoTime() + 20L * 1_000_000_000L);
        Action[][] compactRepair = boundedGlobalRepair(cleanedState, analyzer, compactDeadline, 2_000_000);

        if (compactRepair != null && cleanedPlan.size() + compactRepair.length <= MAX_TOTAL_ACTIONS) {
            appendJointPlan(cleanedPlan, compactRepair);
            System.err.format("Compact one-box repair solved suffix with %,d actions.%n", compactRepair.length);
            return cleanedPlan.toArray(new Action[0][]);
        }
    }
}

if (countUnsolvedBoxGoals(cleanedState) == 0
        && countUnsolvedGoals(cleanedState) <= 4
        && System.nanoTime() < deadline) {
    System.err.println("Trying bounded global repair search for the remaining goals.");

    boolean compactAgentRepair = cleanedState.agentRows.length <= 3;
    long repairDeadline = Math.min(deadline, System.nanoTime() + (compactAgentRepair ? 15L : 3L) * 1_000_000_000L);
    Action[][] repairPlan = boundedGlobalRepair(cleanedState, analyzer, repairDeadline, compactAgentRepair ? 1_500_000 : 60_000);

    if (repairPlan != null && cleanedPlan.size() + repairPlan.length <= MAX_TOTAL_ACTIONS) {
        appendJointPlan(cleanedPlan, repairPlan);
        System.err.format("Bounded global repair solved final state with %,d actions.%n", repairPlan.length);
        return cleanedPlan.toArray(new Action[0][]);
    }
}

if (countSolvedGoals(cleanedState) > countSolvedGoals(bestNode.state)) {
    System.err.println("Returning cleaned partial plan because it solves more goals than the beam plan.");
    return cleanedPlan.toArray(new Action[0][]);
}

System.err.println("Returning best beam partial plan.");
return bestNode.plan.toArray(new Action[0][]);
    }

    private static boolean isBetterPartial(SearchNode candidate, SearchNode best) {
        if (best == null) {
            return true;
        }

        if (candidate.solvedGoals != best.solvedGoals) {
            return candidate.solvedGoals > best.solvedGoals;
        }

        if (candidate.estimateRemaining != best.estimateRemaining) {
            return candidate.estimateRemaining < best.estimateRemaining;
        }

        return candidate.plan.size() < best.plan.size();
    }

    private static LevelAnalyzer analyzeLevel(State initialState) {
        LevelAnalyzer analyzer = new LevelAnalyzer(initialState);
        analyzer.analyze();
        return analyzer;
    }

    private static Action[][] tryInitialBoundedGlobalRepair(
            State initialState,
            LevelAnalyzer analyzer,
            long deadline
    ) {
        if (countUnsolvedGoals(initialState) > 6
                || initialState.agentRows.length > 3
                || State.walls.length * State.walls[0].length > 700) {
            return null;
        }

        System.err.println("Trying bounded global opening search for small level.");
        Action[][] openingRepair = boundedGlobalRepair(
                initialState,
                analyzer,
                Math.min(deadline, System.nanoTime() + 25L * 1_000_000_000L),
                300_000
        );

        if (openingRepair != null) {
            System.err.format("Opening global search solved level with %,d actions.%n", openingRepair.length);
        }

        return openingRepair;
    }

    private static Action[][] solveRemainingAgentsWithCBS(State state, LevelAnalyzer analyzer, long deadline) {
        if (countUnsolvedBoxGoals(state) != 0 || System.nanoTime() >= deadline) {
            return null;
        }

        long cbsDeadline = Math.min(deadline, System.nanoTime() + 4L * 1_000_000_000L);
        return CBSPlanner.planAgentGoals(state, analyzer, cbsDeadline, 4_000);
    }

    private static Action[][] concatenatePlans(Action[][] prefix, Action[][] suffix) {
        Action[][] combined = new Action[prefix.length + suffix.length][];

        for (int i = 0; i < prefix.length; i++) {
            combined[i] = prefix[i].clone();
        }

        for (int i = 0; i < suffix.length; i++) {
            combined[prefix.length + i] = suffix[i].clone();
        }

        return combined;
    }

    private static boolean verifyJointPlan(State initialState, Action[][] plan) {
        return verifyJointPlan(initialState, plan, "candidate");
    }

    private static boolean verifyJointPlan(State initialState, Action[][] plan, String label) {
        return PlanVerifier.verifyComplete(initialState, plan, label).passed;
    }

    private static boolean isPrefixValid(State initialState, Action[][] plan) {
        return PlanVerifier.verifyPrefix(initialState, plan);
    }

    private static void printHybridSummary(HybridRunStats stats, int actionCount, long startTime) {
        double runtime = (System.nanoTime() - startTime) / 1_000_000_000.0;
        System.err.println("Hybrid solver summary:");
        System.err.format("  CBS box tasks attempted: %,d%n", stats.cbsBoxAttempted);
        System.err.format("  CBS box tasks solved: %,d%n", stats.cbsBoxSolved);
        System.err.format("  Fallback used: %s%n", stats.fallbackUsed ? "yes" : "no");
        System.err.format("  Final agent CBS used: %s%n", stats.finalAgentCBSUsed ? "yes" : "no");
        System.err.format("  Final action count: %,d%n", actionCount);
        System.err.format("  Runtime: %.3f seconds%n", runtime);
        System.err.format("  Verification passed: %s%n", stats.verificationPassed ? "yes" : "no");
    }

    private static void printStrategySummary(StrategyResult result) {
        System.err.format(
                "Strategy %-37s solved=%s verified=%s actions=%,d runtime=%.3fs CBS-box=%,d/%,d final-agent-CBS=%s fallback=%s%s%n",
                result.name + ":",
                result.verified ? "yes" : "no",
                result.verified ? "yes" : "no",
                result.actionCount,
                result.runtimeSeconds,
                result.cbsBoxSolved,
                result.cbsBoxAttempted,
                result.finalAgentCBSUsed ? "yes" : "no",
                result.fallbackUsed ? "yes" : "no",
                result.verified || result.rejectionReason == null ? "" : ", rejection=" + result.rejectionReason
        );
    }

    private static ArrayList<Integer> agentCleanupOrder(State state) {
        ArrayList<Integer> agents = new ArrayList<>();

        for (int agent = 0; agent < state.agentRows.length; agent++) {
            Position goal = findAgentGoal(agent);

            if (goal != null && !(state.agentRows[agent] == goal.row && state.agentCols[agent] == goal.col)) {
                agents.add(agent);
            }
        }

        agents.sort(
                Comparator
                        .comparingInt((Integer agent) -> agentGoalClusterScore(agent) > 0 ? 0 : 1)
                        .thenComparingInt(ProjectSolver::agentGoalRow)
                        .thenComparingInt(ProjectSolver::agentGoalCol)
                        .thenComparingInt((Integer agent) -> -agentGoalClusterScore(agent))
                        .thenComparingInt(agent -> -agentGoalDistance(state, agent))
        );

        return agents;
    }

    private static boolean agentOwnsRemainingBoxColor(State state, int agent) {
        if (agent < 0 || agent >= State.agentColors.length) {
            return false;
        }

        Color agentColor = State.agentColors[agent];

        for (int r = 0; r < State.goals.length; r++) {
            for (int c = 0; c < State.goals[r].length; c++) {
                char goal = State.goals[r][c];

                if ('A' <= goal
                        && goal <= 'Z'
                        && state.boxes[r][c] != goal
                        && State.boxColors[goal - 'A'] == agentColor) {
                    return true;
                }
            }
        }

        return false;
    }

    private static ArrayList<Integer> dependencyAwareAgentCleanupOrder(State state, LevelAnalyzer analyzer) {
        ArrayList<Integer> agents = new ArrayList<>();

        for (int agent = 0; agent < state.agentRows.length; agent++) {
            Position goal = findAgentGoal(agent);

            if (goal != null && !(state.agentRows[agent] == goal.row && state.agentCols[agent] == goal.col)) {
                agents.add(agent);
            }
        }

        agents.sort(
                Comparator
                        .comparingInt((Integer agent) -> -agentGoalOccupancyPressure(state, agent))
                        .thenComparingInt(agent -> agentGoalMouthPenalty(state, analyzer, agent))
                        .thenComparingInt(agent -> -agentGraphGoalDistance(state, analyzer, agent))
                        .thenComparingInt(ProjectSolver::agentGoalRow)
                        .thenComparingInt(ProjectSolver::agentGoalCol)
        );

        return agents;
    }

    private static int agentGoalMouthPenalty(State state, LevelAnalyzer analyzer, int agent) {
        Position goal = findAgentGoal(agent);

        if (goal == null) {
            return 0;
        }

        for (int other = 0; other < state.agentRows.length; other++) {
            if (other == agent) {
                continue;
            }

            Position otherGoal = findAgentGoal(other);

            if (otherGoal == null || (state.agentRows[other] == otherGoal.row && state.agentCols[other] == otherGoal.col)) {
                continue;
            }

            Position otherPos = new Position(state.agentRows[other], state.agentCols[other]);

            if (goal.manhattanDistance(otherPos) <= 2
                    && analyzer.distance(otherPos, otherGoal) > analyzer.distance(goal, otherGoal)) {
                return 1;
            }
        }

        return 0;
    }

    private static int agentGoalOccupancyPressure(State state, int agent) {
        int pressure = 0;

        Position agentPos = new Position(state.agentRows[agent], state.agentCols[agent]);

        for (int other = 0; other < state.agentRows.length; other++) {
            if (other == agent) {
                continue;
            }

            Position otherGoal = findAgentGoal(other);

            if (otherGoal != null && agentPos.equals(otherGoal)) {
                pressure += 10;
            }
        }

        Position ownGoal = findAgentGoal(agent);

        if (ownGoal != null) {
            int occupant = agentAt(state, ownGoal.row, ownGoal.col);

            if (occupant != -1 && occupant != agent) {
                pressure += 20;
            }
        }

        return pressure;
    }

    private static int agentGraphGoalDistance(State state, LevelAnalyzer analyzer, int agent) {
        Position goal = findAgentGoal(agent);

        if (goal == null) {
            return 0;
        }

        int distance = analyzer.distance(new Position(state.agentRows[agent], state.agentCols[agent]), goal);

        if (distance >= LevelAnalyzer.INF) {
            return Math.abs(state.agentRows[agent] - goal.row) + Math.abs(state.agentCols[agent] - goal.col);
        }

        return distance;
    }

    private static boolean isCompactRoomGroup(GoalGroup group) {
        if (group.size() < 3) {
            return false;
        }

        int minRow = Integer.MAX_VALUE;
        int maxRow = Integer.MIN_VALUE;
        int minCol = Integer.MAX_VALUE;
        int maxCol = Integer.MIN_VALUE;

        for (Position goal : group.goals) {
            minRow = Math.min(minRow, goal.row);
            maxRow = Math.max(maxRow, goal.row);
            minCol = Math.min(minCol, goal.col);
            maxCol = Math.max(maxCol, goal.col);
        }

        int rowSpan = maxRow - minRow;
        int colSpan = maxCol - minCol;

        if ((rowSpan == 0 && colSpan <= 6) || (colSpan == 0 && rowSpan <= 6)) {
            return true;
        }

        if (group.size() == 3) {
            return (rowSpan <= 1 && colSpan <= 3) || (rowSpan <= 3 && colSpan <= 1);
        }

        if (rowSpan <= 3 && colSpan <= 3) {
            return true;
        }

        if (group.size() > 8 || rowSpan > 4 || colSpan > 5) {
            return false;
        }

        HashSet<Integer> rows = new HashSet<>();
        HashSet<Integer> cols = new HashSet<>();

        for (Position goal : group.goals) {
            rows.add(goal.row);
            cols.add(goal.col);
        }

        return rows.size() <= 3 && cols.size() <= 4;
    }

    private static PocketResult tryCompactGroupSequentialPlanner(
            State start,
            ArrayList<Action[]> basePlan,
            LevelAnalyzer analyzer,
            SingleAgentPlanner planner,
            AgentRetreatPlanner retreatPlanner,
            GoalGroup group,
            long deadline
    ) {
        PocketResult best = null;

        for (ArrayList<Position> order : compactGroupOrders(group)) {
            if (System.nanoTime() > deadline) {
                break;
            }

            State current = copyState(start);
            ArrayList<Action[]> plan = copyPlan(basePlan);
            boolean progress = false;

            for (Position goal : order) {
                if (System.nanoTime() > deadline || plan.size() >= MAX_TOTAL_ACTIONS) {
                    break;
                }

                char letter = State.goals[goal.row][goal.col];

                if (current.boxes[goal.row][goal.col] == letter) {
                    continue;
                }

                Task task = bestSameSideTaskForGoal(current, analyzer, letter, goal, dominantBoundary(goal));

                if (task == null) {
                    continue;
                }

                List<Action> boxPlan = planner.planBoxToGoal(current, task, new ReservationTable());

                if (boxPlan == null || boxPlan.isEmpty()) {
                    boxPlan = planner.planBoxToGoalRelaxed(current, task, new ReservationTable());
                }

                if (boxPlan == null || boxPlan.isEmpty() || plan.size() + boxPlan.size() > MAX_TOTAL_ACTIONS) {
                    continue;
                }

                appendAsJointActions(plan, boxPlan, task.assignedAgent, current.agentRows.length);
                current = simulateSingleAgentPlan(current, boxPlan, task.assignedAgent);
                progress = true;

                System.err.format(
                        "Compact room placed %c at %s using %,d actions.%n",
                        letter,
                        goal,
                        boxPlan.size()
                );

                List<Action> retreat = retreatPlanner.planRetreat(current, task.assignedAgent, 8);

                if (retreat != null && !retreat.isEmpty() && plan.size() + retreat.size() <= MAX_TOTAL_ACTIONS) {
                    appendAsJointActions(plan, retreat, task.assignedAgent, current.agentRows.length);
                    current = simulateSingleAgentPlan(current, retreat, task.assignedAgent);
                }
            }

            if (!progress) {
                continue;
            }

            PocketResult candidate = new PocketResult(current, plan);

            if (best == null
                    || countSolvedGoals(candidate.state) > countSolvedGoals(best.state)
                    || (countSolvedGoals(candidate.state) == countSolvedGoals(best.state)
                    && candidate.plan.size() < best.plan.size())) {
                best = candidate;
            }
        }

        return best;
    }

    private static ArrayList<ArrayList<Position>> compactGroupOrders(GoalGroup group) {
        ArrayList<ArrayList<Position>> orders = new ArrayList<>();
        ArrayList<Position> base = new ArrayList<>(group.goals);

        ArrayList<Position> topFirst = new ArrayList<>(base);
        topFirst.sort(Comparator.comparingInt((Position p) -> p.row).thenComparingInt(p -> p.col));
        addPositionOrder(orders, topFirst);

        ArrayList<Position> bottomFirst = new ArrayList<>(base);
        bottomFirst.sort(Comparator.comparingInt((Position p) -> -p.row).thenComparingInt(p -> p.col));
        addPositionOrder(orders, bottomFirst);

        ArrayList<Position> leftFirst = new ArrayList<>(base);
        leftFirst.sort(Comparator.comparingInt((Position p) -> p.col).thenComparingInt(p -> p.row));
        addPositionOrder(orders, leftFirst);

        ArrayList<Position> rightFirst = new ArrayList<>(base);
        rightFirst.sort(Comparator.comparingInt((Position p) -> -p.col).thenComparingInt(p -> p.row));
        addPositionOrder(orders, rightFirst);

        ArrayList<Position> boundaryFirst = new ArrayList<>(base);
        boundaryFirst.sort(Comparator.comparingInt(ProjectSolver::boundaryTightness).thenComparingInt(p -> p.row).thenComparingInt(p -> p.col));
        addPositionOrder(orders, boundaryFirst);

        return orders;
    }

    private static void addPositionOrder(ArrayList<ArrayList<Position>> orders, ArrayList<Position> order) {
        for (ArrayList<Position> existing : orders) {
            if (existing.equals(order)) {
                return;
            }
        }

        orders.add(order);
    }

    private static int agentGoalClusterScore(int agent) {
        Position goal = findAgentGoal(agent);

        if (goal == null) {
            return 0;
        }

        int score = 0;

        for (int dr = -1; dr <= 1; dr++) {
            for (int dc = -1; dc <= 1; dc++) {
                if (Math.abs(dr) + Math.abs(dc) != 1) {
                    continue;
                }

                int r = goal.row + dr;
                int c = goal.col + dc;

                if (inBounds(r, c) && '0' <= State.goals[r][c] && State.goals[r][c] <= '9') {
                    score++;
                }
            }
        }

        return score;
    }

    private static int agentGoalDistance(State state, int agent) {
        Position goal = findAgentGoal(agent);

        if (goal == null) {
            return 0;
        }

        return Math.abs(state.agentRows[agent] - goal.row)
                + Math.abs(state.agentCols[agent] - goal.col);
    }

    private static int agentGoalRow(int agent) {
        Position goal = findAgentGoal(agent);
        return goal == null ? Integer.MAX_VALUE : goal.row;
    }

    private static int agentGoalCol(int agent) {
        Position goal = findAgentGoal(agent);
        return goal == null ? Integer.MAX_VALUE : goal.col;
    }

    private static Action[][] compactRoomRepair(
            State start,
            LevelAnalyzer analyzer,
            GoalGroup group,
            long deadline,
            int maxExpansions
    ) {
        return compactRoomRepair(start, analyzer, group, buildCompactRoomRegion(start, group), deadline, maxExpansions);
    }

    private static Action[][] compactRoomRepair(
            State start,
            LevelAnalyzer analyzer,
            GoalGroup group,
            RepairRegion region,
            long deadline,
            int maxExpansions
    ) {
        return compactRoomRepair(start, analyzer, group, region, deadline, maxExpansions, 180);
    }

    private static Action[][] compactRoomRepair(
            State start,
            LevelAnalyzer analyzer,
            GoalGroup group,
            RepairRegion region,
            long deadline,
            int maxExpansions,
            int maxRegionArea
    ) {
        int regionArea = (region.maxRow - region.minRow + 1) * (region.maxCol - region.minCol + 1);

        if (regionArea > maxRegionArea) {
            System.err.format(
                    "Skipping compact room repair for %s because the active region is too large (%,d cells).%n",
                    group,
                    regionArea
            );
            return null;
        }

        System.err.format(
                "Trying compact room repair for %s in rows %,d..%,d cols %,d..%,d.%n",
                group,
                region.minRow,
                region.maxRow,
                region.minCol,
                region.maxCol
        );

        for (int activeAgent : compactRoomRepairAgents(start, group, region)) {
            if (System.nanoTime() >= deadline) {
                break;
            }

            Action[][] candidate = compactRoomRepairWithAgent(
                    start,
                    analyzer,
                    group,
                    region,
                    deadline,
                    maxExpansions,
                    activeAgent
            );

            if (candidate == null) {
                continue;
            }

            return candidate;
        }

        return null;
    }

    private static Action[][] compactRoomRepairWithAgent(
            State start,
            LevelAnalyzer analyzer,
            GoalGroup group,
            RepairRegion region,
            long deadline,
            int maxExpansions,
            int activeAgent
    ) {
        System.err.format("  Trying compact room repair with agent %,d.%n", activeAgent);

        PriorityQueue<FocusedNode> open = new PriorityQueue<>(Comparator.comparingInt(node -> node.f));
        HashMap<State, Integer> bestG = new HashMap<>();

        open.add(new FocusedNode(
                start,
                null,
                null,
                0,
                compactRoomHeuristic(start, analyzer, group, region, activeAgent)
        ));

        int expansions = 0;

        while (!open.isEmpty() && System.nanoTime() < deadline) {
            if (++expansions > maxExpansions) {
                System.err.format("Compact room repair hit expansion limit. Open size: %,d.%n", open.size());
                return null;
            }

            FocusedNode node = open.poll();
            Integer seen = bestG.get(node.state);

            if (seen != null && seen <= node.g) {
                continue;
            }

            bestG.put(node.state, node.g);

            if (compactGroupSolved(node.state, group)) {
                System.err.format("Compact room repair expanded %,d states.%n", expansions);
                return extractFocusedPlan(node, start.agentRows.length);
            }

            for (Action action : compactRoomActions(node.state, activeAgent, group, region)) {
                Action[] jointAction = noopJointAction(node.state.agentRows.length);
                jointAction[activeAgent] = action;

                if (isConflicting(node.state, jointAction)) {
                    continue;
                }

                State next = applyJointAction(node.state, jointAction);
                int nextG = node.g + 1;
                Integer bestKnown = bestG.get(next);

                if (bestKnown != null && bestKnown <= nextG) {
                    continue;
                }

                int h = compactRoomHeuristic(next, analyzer, group, region, activeAgent);
                if (h >= LevelAnalyzer.INF) {
                    continue;
                }

                open.add(new FocusedNode(next, node, jointAction, nextG, nextG + 9 * h));
            }
        }

        System.err.format("Compact room repair exhausted search after %,d expansions.%n", expansions);
        return null;
    }

    private static ArrayList<Integer> compactRoomRepairAgents(State state, GoalGroup group, RepairRegion region) {
        ArrayList<Integer> agents = new ArrayList<>();
        addCompactRoomRepairAgent(agents, group.assignedAgent);

        for (int agent = 0; agent < state.agentRows.length; agent++) {
            if (agent == group.assignedAgent || !agentCanMoveCompactGroupLetter(agent, group)) {
                continue;
            }

            addCompactRoomRepairAgent(agents, agent);
        }

        agents.sort(Comparator
                .comparingInt((Integer agent) -> agent == group.assignedAgent ? 0 : 1)
                .thenComparingInt(agent -> distanceToRegion(state.agentRows[agent], state.agentCols[agent], region)));
        return agents;
    }

    private static void addCompactRoomRepairAgent(ArrayList<Integer> agents, int agent) {
        if (agent < 0 || agents.contains(agent)) {
            return;
        }

        agents.add(agent);
    }

    private static boolean agentCanMoveCompactGroupLetter(int agent, GoalGroup group) {
        for (char letter : group.letters) {
            if (State.agentColors[agent] == State.boxColors[letter - 'A']) {
                return true;
            }
        }

        return false;
    }

    private static RepairRegion buildCompactRoomRegion(State state, GoalGroup group) {
        int minRow = Integer.MAX_VALUE;
        int maxRow = Integer.MIN_VALUE;
        int minCol = Integer.MAX_VALUE;
        int maxCol = Integer.MIN_VALUE;

        for (Position goal : group.goals) {
            minRow = Math.min(minRow, goal.row);
            maxRow = Math.max(maxRow, goal.row);
            minCol = Math.min(minCol, goal.col);
            maxCol = Math.max(maxCol, goal.col);
        }

        int nearbyBoxThreshold = group.size() <= 6 ? 8 : 12;

        for (char letter : group.letters) {
            for (int r = 0; r < state.boxes.length; r++) {
                for (int c = 0; c < state.boxes[r].length; c++) {
                    if (state.boxes[r][c] == letter
                            && nearestGroupGoalManhattan(group, letter, new Position(r, c)) <= nearbyBoxThreshold) {
                        minRow = Math.min(minRow, r);
                        maxRow = Math.max(maxRow, r);
                        minCol = Math.min(minCol, c);
                        maxCol = Math.max(maxCol, c);
                    }
                }
            }
        }

        minRow = Math.max(0, minRow - 2);
        maxRow = Math.min(State.walls.length - 1, maxRow + 3);
        minCol = Math.max(0, minCol - 3);
        maxCol = Math.min(State.walls[0].length - 1, maxCol + 3);

        return new RepairRegion(minRow, maxRow, minCol, maxCol);
    }

    private static int nearestGroupGoalManhattan(GoalGroup group, char letter, Position box) {
        int best = Integer.MAX_VALUE;

        for (int i = 0; i < group.goals.size(); i++) {
            if (group.letters.get(i) == letter) {
                best = Math.min(best, box.manhattanDistance(group.goals.get(i)));
            }
        }

        return best;
    }

    private static boolean isTightManyColorCorridorCandidate(State state) {
        return isTightCorridorMap(state)
                && state.agentRows.length >= 8
                && countUnsolvedBoxGoals(state) >= 8;
    }

    private static boolean isTightCorridorMap(State state) {
        return State.walls.length <= 12
                && State.walls[0].length <= 25;
    }

    private static Position nearestFreeNonGoalCell(State state, Position origin, int radiusLimit) {
        for (int radius = 1; radius <= radiusLimit; radius++) {
            for (int row = origin.row - radius; row <= origin.row + radius; row++) {
                for (int col = origin.col - radius; col <= origin.col + radius; col++) {
                    if (Math.abs(row - origin.row) + Math.abs(col - origin.col) != radius) {
                        continue;
                    }

                    if (inBounds(row, col)
                            && State.goals[row][col] == 0
                            && cellIsFree(state, row, col)) {
                        return new Position(row, col);
                    }
                }
            }
        }

        return null;
    }

    private static int nearestAgentWithColor(State state, Color color, Position target) {
        int bestAgent = -1;
        int bestDistance = Integer.MAX_VALUE;

        for (int agent = 0; agent < state.agentRows.length; agent++) {
            if (State.agentColors[agent] != color) {
                continue;
            }

            int distance = Math.abs(state.agentRows[agent] - target.row) + Math.abs(state.agentCols[agent] - target.col);

            if (distance < bestDistance) {
                bestDistance = distance;
                bestAgent = agent;
            }
        }

        return bestAgent;
    }

    private static boolean containsLetter(char[] letters, char letter) {
        for (char candidate : letters) {
            if (candidate == letter) {
                return true;
            }
        }

        return false;
    }

    private static ArrayList<Integer> agentsForLetterColor(State state, char letter, RepairRegion region) {
        ArrayList<Integer> agents = new ArrayList<>();
        Color color = State.boxColors[letter - 'A'];

        for (int agent = 0; agent < state.agentRows.length; agent++) {
            if (State.agentColors[agent] == color) {
                agents.add(agent);
            }
        }

        agents.sort(Comparator.comparingInt(agent -> distanceToRegion(state.agentRows[agent], state.agentCols[agent], region)));
        return agents;
    }

    private static boolean compactGroupSolved(State state, GoalGroup group) {
        for (int i = 0; i < group.goals.size(); i++) {
            Position goal = group.goals.get(i);

            if (state.boxes[goal.row][goal.col] != group.letters.get(i)) {
                return false;
            }
        }

        return true;
    }

    private static int compactRoomHeuristic(
            State state,
            LevelAnalyzer analyzer,
            GoalGroup group,
            RepairRegion region,
            int activeAgent
    ) {
        int h = 0;
        int unsolved = 0;

        for (int i = 0; i < group.goals.size(); i++) {
            Position goal = group.goals.get(i);
            char letter = group.letters.get(i);

            if (state.boxes[goal.row][goal.col] == letter) {
                continue;
            }

            unsolved++;
            int best = LevelAnalyzer.INF;

            for (int r = region.minRow; r <= region.maxRow; r++) {
                for (int c = region.minCol; c <= region.maxCol; c++) {
                    if (state.boxes[r][c] == letter) {
                        best = Math.min(best, analyzer.distance(new Position(r, c), goal));
                    }
                }
            }

            if (best >= LevelAnalyzer.INF) {
                return LevelAnalyzer.INF;
            }

            h += 30 * best;
        }

        h += 220 * unsolved;
        h += distanceToRegion(state.agentRows[activeAgent], state.agentCols[activeAgent], region);

        return h;
    }

    private static Action[] compactRoomActions(State state, int agent, GoalGroup group, RepairRegion region) {
        ArrayList<Action> actions = new ArrayList<>();

        for (Action action : Action.values()) {
            if (action.type == ActionType.NoOp) {
                continue;
            }

            if (!compactActionStaysRelevant(state, agent, action, group, region)) {
                continue;
            }

            if (compactActionMovesForeignBox(state, agent, action, group)) {
                continue;
            }

            if (isApplicable(state, agent, action)) {
                actions.add(action);
            }
        }

        actions.sort(Comparator.comparingInt(action -> compactActionScore(state, agent, action, group, region)));
        return actions.toArray(new Action[0]);
    }

    private static boolean compactActionStaysRelevant(
            State state,
            int agent,
            Action action,
            GoalGroup group,
            RepairRegion region
    ) {
        int ar = state.agentRows[agent];
        int ac = state.agentCols[agent];
        int agentDestRow = ar;
        int agentDestCol = ac;
        int boxDestRow = -1;
        int boxDestCol = -1;

        switch (action.type) {
            case Move:
                agentDestRow = ar + action.agentRowDelta;
                agentDestCol = ac + action.agentColDelta;
                break;

            case Push: {
                int boxRow = ar + action.agentRowDelta;
                int boxCol = ac + action.agentColDelta;
                agentDestRow = boxRow;
                agentDestCol = boxCol;
                boxDestRow = boxRow + action.boxRowDelta;
                boxDestCol = boxCol + action.boxColDelta;
                break;
            }

            case Pull: {
                agentDestRow = ar + action.agentRowDelta;
                agentDestCol = ac + action.agentColDelta;
                int boxRow = ar - action.boxRowDelta;
                int boxCol = ac - action.boxColDelta;
                boxDestRow = boxRow + action.boxRowDelta;
                boxDestCol = boxCol + action.boxColDelta;
                break;
            }
        }

        if (!region.containsWithMargin(agentDestRow, agentDestCol, 2)) {
            return distanceToRegion(ar, ac, region) > distanceToRegion(agentDestRow, agentDestCol, region);
        }

        return boxDestRow == -1 || region.containsWithMargin(boxDestRow, boxDestCol, 1);
    }

    private static boolean compactActionMovesForeignBox(State state, int agent, Action action, GoalGroup group) {
        int[] source = movedBoxSource(state, agent, action);

        if (source == null || !inBounds(source[0], source[1])) {
            return false;
        }

        char box = state.boxes[source[0]][source[1]];

        if (!('A' <= box && box <= 'Z')) {
            return false;
        }

        return !group.letters.contains(box);
    }

    private static boolean compactActionBreaksSolvedBox(State state, int agent, Action action, GoalGroup group) {
        int[] source = movedBoxSource(state, agent, action);

        if (source == null || !inBounds(source[0], source[1])) {
            return false;
        }

        char box = state.boxes[source[0]][source[1]];

        if (!('A' <= box && box <= 'Z')) {
            return false;
        }

        if (!group.letters.contains(box)) {
            return false;
        }

        Position pos = new Position(source[0], source[1]);

        for (int i = 0; i < group.goals.size(); i++) {
            if (group.goals.get(i).equals(pos) && group.letters.get(i) == box) {
                int[] dest = movedBoxDestination(state, agent, action);
                return dest == null
                        || !inBounds(dest[0], dest[1])
                        || State.goals[dest[0]][dest[1]] != box
                        || !group.goals.contains(new Position(dest[0], dest[1]));
            }
        }

        return false;
    }

    private static int compactActionScore(
            State state,
            int agent,
            Action action,
            GoalGroup group,
            RepairRegion region
    ) {
        int score = action.type == ActionType.Move ? 3 : 0;
        int[] dest = movedBoxDestination(state, agent, action);

        if (dest != null && inBounds(dest[0], dest[1])) {
            char box = movedBoxLetter(state, agent, action);

            if ('A' <= box && box <= 'Z') {
                Position newBox = new Position(dest[0], dest[1]);
                int best = LevelAnalyzer.INF;

                for (int i = 0; i < group.goals.size(); i++) {
                    if (group.letters.get(i) == box) {
                        best = Math.min(best, newBox.manhattanDistance(group.goals.get(i)));
                    }
                }

                score += best;
                if (State.goals[dest[0]][dest[1]] == box) {
                    score -= 20;
                }
            }
        }

        int ar = state.agentRows[agent] + action.agentRowDelta;
        int ac = state.agentCols[agent] + action.agentColDelta;
        score += distanceToRegion(ar, ac, region);

        return score;
    }

    private static char movedBoxLetter(State state, int agent, Action action) {
        int[] source = movedBoxSource(state, agent, action);

        if (source == null || !inBounds(source[0], source[1])) {
            return 0;
        }

        return state.boxes[source[0]][source[1]];
    }

    private static PocketResult tryBoundaryPocketPlanner(
            State start,
            ArrayList<Action[]> basePlan,
            LevelAnalyzer analyzer,
            SingleAgentPlanner planner,
            AgentRetreatPlanner retreatPlanner,
            long deadline
    ) {
        PocketResult best = null;
        PocketResult stagedCorner = tryStagedCornerPocketPlanner(
                start,
                basePlan,
                analyzer,
                planner,
                retreatPlanner,
                deadline
        );

        if (stagedCorner != null) {
            best = stagedCorner;

            if (best.state.isGoalState()) {
                return best;
            }
        }

        for (ArrayList<Position> order : boundaryGoalOrderPortfolio(start)) {
            if (System.nanoTime() > deadline) {
                break;
            }

            PocketResult candidate = tryBoundaryGoalOrder(
                    start,
                    basePlan,
                    analyzer,
                    planner,
                    retreatPlanner,
                    deadline,
                    order
            );

            if (candidate == null) {
                continue;
            }

            if (best == null
                    || countSolvedGoals(candidate.state) > countSolvedGoals(best.state)
                    || (countSolvedGoals(candidate.state) == countSolvedGoals(best.state)
                    && topDoorCorridorScore(candidate.state) > topDoorCorridorScore(best.state))
                    || (countSolvedGoals(candidate.state) == countSolvedGoals(best.state)
                    && topDoorCorridorScore(candidate.state) == topDoorCorridorScore(best.state)
                    && sideColumnSealScore(candidate.state) > sideColumnSealScore(best.state))
                    || (countSolvedGoals(candidate.state) == countSolvedGoals(best.state)
                    && topDoorCorridorScore(candidate.state) == topDoorCorridorScore(best.state)
                    && sideColumnSealScore(candidate.state) == sideColumnSealScore(best.state)
                    && candidate.plan.size() < best.plan.size())) {
                best = candidate;
            }
        }

        return best;
    }

    private static int topDoorCorridorScore(State state) {
        if (State.walls.length <= 2 || State.walls[0].length < 8) {
            return 0;
        }

        int doorCol = topBoundaryDoorCol();
        int score = 0;

        for (int c = 1; c < State.goals[1].length - 1; c++) {
            char goal = State.goals[1][c];

            if ('A' <= goal && goal <= 'Z' && Math.abs(c - doorCol) <= Math.max(4, State.goals[1].length / 3)
                    && state.boxes[1][c] == goal) {
                score++;
            }
        }

        return score;
    }

    private static int sideColumnSealScore(State state) {
        int score = 0;
        int rightOuterCol = State.goals[0].length - 2;

        for (int r = 0; r < State.goals.length; r++) {
            if (State.goals[r].length > 2
                    && 'A' <= State.goals[r][2]
                    && State.goals[r][2] <= 'Z'
                    && state.boxes[r][2] == State.goals[r][2]) {
                score++;
            }

            if (rightOuterCol >= 0
                    && rightOuterCol < State.goals[r].length
                    && 'A' <= State.goals[r][rightOuterCol]
                    && State.goals[r][rightOuterCol] <= 'Z'
                    && state.boxes[r][rightOuterCol] == State.goals[r][rightOuterCol]) {
                score++;
            }
        }

        return score;
    }

    private static PocketResult tryStagedCornerPocketPlanner(
            State start,
            ArrayList<Action[]> basePlan,
            LevelAnalyzer analyzer,
            SingleAgentPlanner planner,
            AgentRetreatPlanner retreatPlanner,
            long deadline
    ) {
        for (Position targetGoal : allBoundaryBoxGoals()) {
            if (System.nanoTime() > deadline || !isCornerPocketGoal(targetGoal)) {
                continue;
            }

            char targetLetter = State.goals[targetGoal.row][targetGoal.col];

            if (start.boxes[targetGoal.row][targetGoal.col] == targetLetter) {
                continue;
            }

            for (Position blockerStart : cornerCorridorBlockers(start, targetGoal)) {
                char blockerLetter = start.boxes[blockerStart.row][blockerStart.col];

                if (!('A' <= blockerLetter && blockerLetter <= 'Z') || blockerLetter == targetLetter) {
                    continue;
                }

                Position blockerGoal = findBoxGoal(blockerLetter);

                if (blockerGoal == null || start.boxes[blockerGoal.row][blockerGoal.col] == blockerLetter) {
                    continue;
                }

                PocketResult staged = tryStagedCornerSequence(
                        start,
                        basePlan,
                        analyzer,
                        planner,
                        retreatPlanner,
                        deadline,
                        targetLetter,
                        targetGoal,
                        blockerLetter,
                        blockerStart,
                        blockerGoal
                );

                if (staged != null) {
                    return staged;
                }
            }
        }

        return null;
    }

    private static ArrayList<Position> cornerCorridorBlockers(State state, Position targetGoal) {
        ArrayList<Position> blockers = new ArrayList<>();
        int[][] directions = {
                {-1, 0},
                {1, 0},
                {0, -1},
                {0, 1}
        };

        for (int[] direction : directions) {
            for (int distance = 1; distance <= 8; distance++) {
                int row = targetGoal.row + direction[0] * distance;
                int col = targetGoal.col + direction[1] * distance;

                if (!inBounds(row, col) || State.walls[row][col]) {
                    break;
                }

                char box = state.boxes[row][col];

                if ('A' <= box && box <= 'Z') {
                    blockers.add(new Position(row, col));
                    break;
                }
            }
        }

        blockers.sort(Comparator.comparingInt(p -> p.manhattanDistance(targetGoal)));
        return blockers;
    }

    private static PocketResult tryStagedCornerSequence(
            State start,
            ArrayList<Action[]> basePlan,
            LevelAnalyzer analyzer,
            SingleAgentPlanner planner,
            AgentRetreatPlanner retreatPlanner,
            long deadline,
            char targetLetter,
            Position targetGoal,
            char blockerLetter,
            Position blockerStart,
            Position blockerGoal
    ) {
        int blockerAgent = firstAgentWithColor(State.boxColors[blockerLetter - 'A']);
        int targetAgent = firstAgentWithColor(State.boxColors[targetLetter - 'A']);

        if (blockerAgent == -1 || targetAgent == -1) {
            return null;
        }

        int rowDirection = Integer.compare(blockerStart.row, targetGoal.row);
        int colDirection = Integer.compare(blockerStart.col, targetGoal.col);

        for (Position stage : cornerStagingCells(start, targetGoal, rowDirection, colDirection)) {
            if (System.nanoTime() > deadline) {
                return null;
            }

            State current = copyState(start);
            ArrayList<Action[]> plan = copyPlan(basePlan);

            Task stageTask = new Task(blockerLetter, blockerStart, stage, blockerAgent, 0);
            List<Action> stagePlan = planner.planBoxToGoalRelaxed(current, stageTask, new ReservationTable());

            if (stagePlan == null || stagePlan.isEmpty() || plan.size() + stagePlan.size() > MAX_TOTAL_ACTIONS) {
                continue;
            }

            appendAsJointActions(plan, stagePlan, blockerAgent, current.agentRows.length);
            current = simulateSingleAgentPlan(current, stagePlan, blockerAgent);

            Task targetTask = bestSameSideTaskForGoal(current, analyzer, targetLetter, targetGoal, dominantBoundary(targetGoal));

            if (targetTask == null) {
                continue;
            }

            List<Action> targetPlan = planner.planBoxToGoal(current, targetTask, new ReservationTable());

            if (targetPlan == null || targetPlan.isEmpty()) {
                targetPlan = planner.planBoxToGoalRelaxed(current, targetTask, new ReservationTable());
            }

            if (targetPlan == null || targetPlan.isEmpty() || plan.size() + targetPlan.size() > MAX_TOTAL_ACTIONS) {
                continue;
            }

            appendAsJointActions(plan, targetPlan, targetTask.assignedAgent, current.agentRows.length);
            current = simulateSingleAgentPlan(current, targetPlan, targetTask.assignedAgent);

            Task restoreTask = new Task(blockerLetter, stage, blockerGoal, blockerAgent, 0);
            List<Action> restorePlan = planner.planBoxToGoal(current, restoreTask, new ReservationTable());

            if (restorePlan == null || restorePlan.isEmpty()) {
                restorePlan = planner.planBoxToGoalRelaxed(current, restoreTask, new ReservationTable());
            }

            if (restorePlan == null || restorePlan.isEmpty() || plan.size() + restorePlan.size() > MAX_TOTAL_ACTIONS) {
                continue;
            }

            appendAsJointActions(plan, restorePlan, blockerAgent, current.agentRows.length);
            current = simulateSingleAgentPlan(current, restorePlan, blockerAgent);

            System.err.format(
                    "Staged corner repair moved %c via %s, placed %c at %s, then restored %c at %s.%n",
                    blockerLetter,
                    stage,
                    targetLetter,
                    targetGoal,
                    blockerLetter,
                    blockerGoal
            );

            List<Action> retreat = retreatPlanner.planRetreat(current, blockerAgent, 10);

            if (retreat != null && !retreat.isEmpty() && plan.size() + retreat.size() <= MAX_TOTAL_ACTIONS) {
                appendAsJointActions(plan, retreat, blockerAgent, current.agentRows.length);
                current = simulateSingleAgentPlan(current, retreat, blockerAgent);
            }

            PocketResult tail = tryBoundaryGoalOrder(
                    current,
                    plan,
                    analyzer,
                    planner,
                    retreatPlanner,
                    deadline,
                    dependencyOrderedBoundaryGoals(current)
            );

            return tail != null ? tail : new PocketResult(current, plan);
        }

        return null;
    }

    private static ArrayList<Position> cornerStagingCells(
            State state,
            Position targetGoal,
            int rowDirection,
            int colDirection
    ) {
        ArrayList<Position> result = new ArrayList<>();

        for (int distance = 2; distance <= 8; distance++) {
            int row = targetGoal.row + rowDirection * distance;
            int col = targetGoal.col + colDirection * distance;

            if (!inBounds(row, col) || State.walls[row][col]) {
                break;
            }

            if (cellIsFree(state, row, col) && State.goals[row][col] == 0) {
                result.add(new Position(row, col));
            }
        }

        if (result.isEmpty()) {
            for (Position access : accessCells(targetGoal)) {
                for (Position side : accessCells(access)) {
                    if (!side.equals(targetGoal)
                            && cellIsFree(state, side.row, side.col)
                            && State.goals[side.row][side.col] == 0) {
                        result.add(side);
                    }
                }
            }
        }

        result.sort(Comparator.comparingInt(p -> -p.manhattanDistance(targetGoal)));
        return result;
    }

    private static boolean isCornerPocketGoal(Position goal) {
        return nearBoundary(goal.row, goal.col, Boundary.LEFT) && nearBoundary(goal.row, goal.col, Boundary.BOTTOM)
                || nearBoundary(goal.row, goal.col, Boundary.LEFT) && nearBoundary(goal.row, goal.col, Boundary.TOP)
                || nearBoundary(goal.row, goal.col, Boundary.RIGHT) && nearBoundary(goal.row, goal.col, Boundary.BOTTOM)
                || nearBoundary(goal.row, goal.col, Boundary.RIGHT) && nearBoundary(goal.row, goal.col, Boundary.TOP);
    }

    private static Position findBoxGoal(char letter) {
        for (int r = 0; r < State.goals.length; r++) {
            for (int c = 0; c < State.goals[r].length; c++) {
                if (State.goals[r][c] == letter) {
                    return new Position(r, c);
                }
            }
        }

        return null;
    }

    private static PocketResult tryBoundaryGoalOrder(
            State start,
            ArrayList<Action[]> basePlan,
            LevelAnalyzer analyzer,
            SingleAgentPlanner planner,
            AgentRetreatPlanner retreatPlanner,
            long deadline,
            ArrayList<Position> order
    ) {
        State current = copyState(start);
        ArrayList<Action[]> plan = copyPlan(basePlan);
        boolean progress = false;

        for (Position goal : order) {
            if (System.nanoTime() > deadline || plan.size() >= MAX_TOTAL_ACTIONS) {
                break;
            }

            char letter = State.goals[goal.row][goal.col];

            if (current.boxes[goal.row][goal.col] == letter) {
                continue;
            }

            Boundary boundary = dominantBoundary(goal);
            Task task = bestSameSideTaskForGoal(current, analyzer, letter, goal, boundary);

            if (task == null) {
                continue;
            }

            List<Action> boxPlan = planner.planBoxToGoal(current, task, new ReservationTable());

            if (boxPlan == null || boxPlan.isEmpty()) {
                boxPlan = planner.planBoxToGoalRelaxed(current, task, new ReservationTable());
            }

            if (boxPlan == null || boxPlan.isEmpty() || plan.size() + boxPlan.size() > MAX_TOTAL_ACTIONS) {
                continue;
            }

            appendAsJointActions(plan, boxPlan, task.assignedAgent, current.agentRows.length);
            current = simulateSingleAgentPlan(current, boxPlan, task.assignedAgent);
            progress = true;

            System.err.format(
                    "%s pocket placed %c at %s using %,d actions.%n",
                    boundary,
                    letter,
                    goal,
                    boxPlan.size()
            );

            List<Action> retreat = retreatPlanner.planRetreat(current, task.assignedAgent, 10);

            if (retreat != null && !retreat.isEmpty() && plan.size() + retreat.size() <= MAX_TOTAL_ACTIONS) {
                appendAsJointActions(plan, retreat, task.assignedAgent, current.agentRows.length);
                current = simulateSingleAgentPlan(current, retreat, task.assignedAgent);
            }
        }

        return progress ? new PocketResult(current, plan) : null;
    }

    private static ArrayList<ArrayList<Position>> boundaryGoalOrderPortfolio(State state) {
        ArrayList<ArrayList<Position>> orders = new ArrayList<>();
        ArrayList<Position> legacyOrder = new ArrayList<>();

        for (Boundary boundary : Boundary.values()) {
            ArrayList<Position> goals = boundaryPocketGoals(boundary);

            if (goals.size() >= 2) {
                legacyOrder.addAll(goals);
            }
        }

        addBoundaryOrder(orders, legacyOrder);

        ArrayList<Position> dependencyOrder = dependencyOrderedBoundaryGoals(state);
        addBoundaryOrder(orders, dependencyOrder);

        ArrayList<Position> bottomFirst = new ArrayList<>(dependencyOrder);
        bottomFirst.sort(Comparator
                .comparingInt((Position p) -> -p.row)
                .thenComparingInt(p -> p.col)
                .thenComparingInt(p -> boundaryTightness(p)));
        addBoundaryOrder(orders, bottomFirst);

        ArrayList<Position> leftFirst = new ArrayList<>(dependencyOrder);
        leftFirst.sort(Comparator
                .comparingInt((Position p) -> p.col)
                .thenComparingInt(p -> -p.row)
                .thenComparingInt(p -> boundaryTightness(p)));
        addBoundaryOrder(orders, leftFirst);

        ArrayList<Position> leftTopDown = new ArrayList<>(dependencyOrder);
        leftTopDown.sort(Comparator
                .comparingInt((Position p) -> p.col)
                .thenComparingInt(p -> p.row)
                .thenComparingInt(p -> boundaryTightness(p)));
        addBoundaryOrder(orders, leftTopDown);

        ArrayList<Position> pairedSideColumnOrder = pairedSideColumnBoundaryOrder(dependencyOrder);
        addBoundaryOrder(orders, pairedSideColumnOrder);

        ArrayList<Position> pairedSideRowsTopDown = pairedSideRowBoundaryOrder(dependencyOrder, true);
        addBoundaryOrder(orders, pairedSideRowsTopDown);

        ArrayList<Position> pairedSideRowsBottomUp = pairedSideRowBoundaryOrder(dependencyOrder, false);
        addBoundaryOrder(orders, pairedSideRowsBottomUp);

        ArrayList<Position> topDoorCorridorOrder = topDoorCorridorBoundaryOrder(dependencyOrder);
        addBoundaryOrder(orders, topDoorCorridorOrder);

        return orders;
    }

    private static ArrayList<Position> topDoorCorridorBoundaryOrder(ArrayList<Position> goals) {
        ArrayList<Position> order = new ArrayList<>(goals);
        int doorCol = topBoundaryDoorCol();

        order.sort(Comparator
                .comparingInt((Position p) -> topDoorCorridorGroup(p, doorCol))
                .thenComparingInt(p -> Math.abs(p.col - doorCol))
                .thenComparingInt(p -> p.col < doorCol ? -p.col : p.col)
                .thenComparingInt(p -> p.row)
                .thenComparingInt(ProjectSolver::boundaryTightness));
        return order;
    }

    private static int topDoorCorridorGroup(Position p, int doorCol) {
        return p.row <= 2 && Math.abs(p.col - doorCol) <= Math.max(4, State.goals[0].length / 4) ? 0 : 1;
    }

    private static int topBoundaryDoorCol() {
        if (State.walls.length <= 2) {
            return State.goals[0].length / 2;
        }

        for (int c = 1; c < State.walls[2].length - 1; c++) {
            if (!State.walls[2][c]) {
                return c;
            }
        }

        return State.goals[0].length / 2;
    }

    private static ArrayList<Position> pairedSideColumnBoundaryOrder(ArrayList<Position> goals) {
        ArrayList<Position> order = new ArrayList<>(goals);
        int rightOuterCol = State.goals[0].length - 2;
        int rightInnerCol = State.goals[0].length - 3;

        order.sort(Comparator
                .comparingInt((Position p) -> pairedSideColumnRank(p, rightInnerCol, rightOuterCol))
                .thenComparingInt(p -> -p.row)
                .thenComparingInt(p -> p.col)
                .thenComparingInt(ProjectSolver::boundaryTightness));
        return order;
    }

    private static ArrayList<Position> pairedSideRowBoundaryOrder(ArrayList<Position> goals, boolean topDown) {
        ArrayList<Position> order = new ArrayList<>(goals);
        int rightOuterCol = State.goals[0].length - 2;
        int rightInnerCol = State.goals[0].length - 3;

        order.sort(Comparator
                .comparingInt((Position p) -> pairedSideRowGroup(p, rightInnerCol, rightOuterCol))
                .thenComparingInt(p -> topDown ? p.row : -p.row)
                .thenComparingInt(p -> pairedSideColumnRank(p, rightInnerCol, rightOuterCol))
                .thenComparingInt(ProjectSolver::boundaryTightness));
        return order;
    }

    private static int pairedSideRowGroup(Position p, int rightInnerCol, int rightOuterCol) {
        if (p.col == 1 || p.col == 2 || p.col == rightInnerCol || p.col == rightOuterCol) {
            return 0;
        }

        return 1;
    }

    private static int pairedSideColumnRank(Position p, int rightInnerCol, int rightOuterCol) {
        if (p.col == 2) {
            return 0;
        }

        if (p.col == 1) {
            return 1;
        }

        if (p.col == rightOuterCol) {
            return 2;
        }

        if (p.col == rightInnerCol) {
            return 3;
        }

        return 4;
    }

    private static void addBoundaryOrder(ArrayList<ArrayList<Position>> orders, ArrayList<Position> order) {
        if (order.size() < 2) {
            return;
        }

        for (ArrayList<Position> existing : orders) {
            if (existing.equals(order)) {
                return;
            }
        }

        orders.add(order);
    }

    private static ArrayList<Position> dependencyOrderedBoundaryGoals(State state) {
        ArrayList<Position> goals = allBoundaryBoxGoals();

        if (goals.size() < 2) {
            return goals;
        }

        HashMap<Position, HashSet<Position>> before = new HashMap<>();
        HashMap<Position, Integer> indegree = new HashMap<>();

        for (Position goal : goals) {
            before.put(goal, new HashSet<>());
            indegree.put(goal, 0);
        }

        for (Position blockedGoal : goals) {
            for (Position access : accessCells(blockedGoal)) {
                char box = state.boxes[access.row][access.col];

                if (!('A' <= box && box <= 'Z') || box == State.goals[blockedGoal.row][blockedGoal.col]) {
                    continue;
                }

                for (Position clearingGoal : goals) {
                    if (State.goals[clearingGoal.row][clearingGoal.col] == box
                            && !clearingGoal.equals(blockedGoal)
                            && before.get(clearingGoal).add(blockedGoal)) {
                        indegree.put(blockedGoal, indegree.get(blockedGoal) + 1);
                    }
                }
            }
        }

        PriorityQueue<Position> ready = new PriorityQueue<>(boundaryDependencyComparator());

        for (Position goal : goals) {
            if (indegree.get(goal) == 0) {
                ready.add(goal);
            }
        }

        ArrayList<Position> ordered = new ArrayList<>();

        while (!ready.isEmpty()) {
            Position goal = ready.poll();
            ordered.add(goal);

            for (Position dependent : before.get(goal)) {
                int remaining = indegree.get(dependent) - 1;
                indegree.put(dependent, remaining);

                if (remaining == 0) {
                    ready.add(dependent);
                }
            }
        }

        if (ordered.size() != goals.size()) {
            goals.sort(boundaryDependencyComparator());
            return goals;
        }

        return ordered;
    }

    private static ArrayList<Position> allBoundaryBoxGoals() {
        ArrayList<Position> goals = new ArrayList<>();

        for (int r = 0; r < State.goals.length; r++) {
            for (int c = 0; c < State.goals[r].length; c++) {
                char goal = State.goals[r][c];
                Position position = new Position(r, c);

                if ('A' <= goal && goal <= 'Z'
                        && (nearBoundary(r, c, Boundary.LEFT)
                        || nearBoundary(r, c, Boundary.RIGHT)
                        || nearBoundary(r, c, Boundary.TOP)
                        || nearBoundary(r, c, Boundary.BOTTOM))) {
                    goals.add(position);
                }
            }
        }

        goals.sort(boundaryDependencyComparator());
        return goals;
    }

    private static ArrayList<Position> accessCells(Position goal) {
        ArrayList<Position> cells = new ArrayList<>();
        int[][] deltas = {
                {-1, 0},
                {1, 0},
                {0, -1},
                {0, 1}
        };

        for (int[] delta : deltas) {
            int row = goal.row + delta[0];
            int col = goal.col + delta[1];

            if (inBounds(row, col) && !State.walls[row][col]) {
                cells.add(new Position(row, col));
            }
        }

        return cells;
    }

    private static Comparator<Position> boundaryDependencyComparator() {
        return Comparator
                .comparingInt((Position p) -> -p.row)
                .thenComparingInt(p -> p.col)
                .thenComparingInt(ProjectSolver::boundaryTightness);
    }

    private static int boundaryTightness(Position p) {
        int top = p.row;
        int bottom = State.goals.length - 1 - p.row;
        int left = p.col;
        int right = State.goals[p.row].length - 1 - p.col;
        return Math.min(Math.min(top, bottom), Math.min(left, right));
    }

    private static Boundary dominantBoundary(Position goal) {
        int top = goal.row;
        int bottom = State.goals.length - 1 - goal.row;
        int left = goal.col;
        int right = State.goals[goal.row].length - 1 - goal.col;
        int best = Math.min(Math.min(top, bottom), Math.min(left, right));

        if (best == bottom) {
            return Boundary.BOTTOM;
        }

        if (best == left) {
            return Boundary.LEFT;
        }

        if (best == right) {
            return Boundary.RIGHT;
        }

        return Boundary.TOP;
    }

    private static ArrayList<Position> boundaryPocketGoals(Boundary boundary) {
        ArrayList<Position> goals = new ArrayList<>();

        for (int r = 0; r < State.goals.length; r++) {
            for (int c = 0; c < State.goals[r].length; c++) {
                char goal = State.goals[r][c];

                if ('A' <= goal && goal <= 'Z' && nearBoundary(r, c, boundary)) {
                    goals.add(new Position(r, c));
                }
            }
        }

        goals.sort(boundaryGoalComparator(boundary));

        return goals;
    }

    private static boolean nearBoundary(int row, int col, Boundary boundary) {
        switch (boundary) {
            case RIGHT:
                return col >= State.goals[row].length - 4;
            case LEFT:
                return col <= 3;
            case TOP:
                return row <= 2;
            case BOTTOM:
                return row >= State.goals.length - 3;
        }

        return false;
    }

    private static Comparator<Position> boundaryGoalComparator(Boundary boundary) {
        switch (boundary) {
            case RIGHT:
                return Comparator.comparingInt((Position p) -> -p.row).thenComparingInt(p -> -p.col);
            case LEFT:
                return Comparator.comparingInt((Position p) -> -p.row).thenComparingInt(p -> p.col);
            case TOP:
                return Comparator.comparingInt((Position p) -> p.col).thenComparingInt(p -> p.row);
            case BOTTOM:
                return Comparator.comparingInt((Position p) -> p.col).thenComparingInt(p -> -p.row);
        }

        return Comparator.comparingInt(p -> p.row);
    }

    private static Task bestSameSideTaskForGoal(
            State state,
            LevelAnalyzer analyzer,
            char letter,
            Position goal,
            Boundary boundary
    ) {
        int centerCol = analyzer.cols / 2;
        int centerRow = analyzer.rows / 2;
        Task bestTask = null;
        int bestScore = Integer.MAX_VALUE;

        for (int r = 0; r < state.boxes.length; r++) {
            for (int c = 0; c < state.boxes[r].length; c++) {
                if (state.boxes[r][c] != letter || State.goals[r][c] == letter) {
                    continue;
                }

                Position box = new Position(r, c);
                int boxDistance = analyzer.distance(box, goal);

                if (boxDistance >= LevelAnalyzer.INF) {
                    continue;
                }

                for (int agent = 0; agent < state.agentRows.length; agent++) {
                    if (State.agentColors[agent] != State.boxColors[letter - 'A']) {
                        continue;
                    }

                    int agentDistance = analyzer.distance(
                            new Position(state.agentRows[agent], state.agentCols[agent]),
                            box
                    );

                    if (agentDistance >= LevelAnalyzer.INF) {
                        continue;
                    }

                    int score = 20 * boxDistance + agentDistance + 30 * Math.abs(goal.col - box.col);

                    if (!sameBoundarySide(box, centerRow, centerCol, boundary)) {
                        score += 10_000;
                    } else {
                        score -= 200;
                    }

                    if (score < bestScore) {
                        bestScore = score;
                        bestTask = new Task(letter, box, goal, agent, score);
                    }
                }
            }
        }

        return bestTask;
    }

    private static boolean sameBoundarySide(Position box, int centerRow, int centerCol, Boundary boundary) {
        switch (boundary) {
            case RIGHT:
                return box.col >= centerCol;
            case LEFT:
                return box.col <= centerCol;
            case TOP:
                return box.row <= centerRow;
            case BOTTOM:
                return box.row >= centerRow;
        }

        return false;
    }

    private static PriorityQueue<SearchNode> trimBeam(PriorityQueue<SearchNode> queue, int width) {
        PriorityQueue<SearchNode> result = new PriorityQueue<>(Comparator.comparingInt(SearchNode::score));

        int count = 0;
        while (!queue.isEmpty() && count < width) {
            result.add(queue.poll());
            count++;
        }

        return result;
    }

    private static int countSolvedGoals(State state) {
        int count = 0;

        for (int r = 0; r < State.goals.length; r++) {
            for (int c = 0; c < State.goals[r].length; c++) {
                char goal = State.goals[r][c];

                if ('A' <= goal && goal <= 'Z') {
                    if (state.boxes[r][c] == goal) {
                        count++;
                    }
                } else if ('0' <= goal && goal <= '9') {
                    int agent = goal - '0';
                    if (agent < state.agentRows.length &&
                        state.agentRows[agent] == r &&
                        state.agentCols[agent] == c) {
                        count++;
                    }
                }
            }
        }

        return count;
    }

    private static int countSolvedBoxGoals(State state) {
        int count = 0;

        for (int r = 0; r < State.goals.length; r++) {
            for (int c = 0; c < State.goals[r].length; c++) {
                char goal = State.goals[r][c];

                if ('A' <= goal && goal <= 'Z' && state.boxes[r][c] == goal) {
                    count++;
                }
            }
        }

        return count;
    }

    private static int estimateRemainingCost(LevelAnalyzer analyzer, State state) {
        int total = 0;

        for (int r = 0; r < State.goals.length; r++) {
            for (int c = 0; c < State.goals[r].length; c++) {
                char goal = State.goals[r][c];

                if ('A' <= goal && goal <= 'Z' && state.boxes[r][c] != goal) {
                    Position goalPos = new Position(r, c);
                    int best = LevelAnalyzer.INF;

                    for (int br = 0; br < state.boxes.length; br++) {
                        for (int bc = 0; bc < state.boxes[br].length; bc++) {
                            if (state.boxes[br][bc] == goal) {
                                int d = analyzer.distance(new Position(br, bc), goalPos);
                                best = Math.min(best, d);
                            }
                        }
                    }

                    if (best < LevelAnalyzer.INF) {
                        total += 10 * best;
                    } else {
                        total += 10_000;
                    }
                } else if ('0' <= goal && goal <= '9') {
                    int agent = goal - '0';
                    if (agent < state.agentRows.length &&
                        !(state.agentRows[agent] == r && state.agentCols[agent] == c)) {
                        int d = analyzer.distance(
                                new Position(state.agentRows[agent], state.agentCols[agent]),
                                new Position(r, c)
                        );
                        total += d < LevelAnalyzer.INF ? 2 * d : 10_000;
                    }
                }
            }
        }

        return total;
    }

    private static int countUnsolvedGoals(State state) {
        int count = 0;

        for (int r = 0; r < State.goals.length; r++) {
            for (int c = 0; c < State.goals[r].length; c++) {
                char goal = State.goals[r][c];

                if ('A' <= goal && goal <= 'Z') {
                    if (state.boxes[r][c] != goal) {
                        count++;
                    }
                } else if ('0' <= goal && goal <= '9') {
                    int agent = goal - '0';
                    if (agent < state.agentRows.length &&
                            !(state.agentRows[agent] == r && state.agentCols[agent] == c)) {
                        count++;
                    }
                }
            }
        }

        return count;
    }

    private static int countUnsolvedBoxGoals(State state) {
        int count = 0;

        for (int r = 0; r < State.goals.length; r++) {
            for (int c = 0; c < State.goals[r].length; c++) {
                char goal = State.goals[r][c];

                if ('A' <= goal && goal <= 'Z' && state.boxes[r][c] != goal) {
                    count++;
                }
            }
        }

        return count;
    }

    private static Action[][] boundedGlobalRepair(
            State start,
            LevelAnalyzer analyzer,
            long deadline,
            int maxExpansions
    ) {
        final int weight = 5;

        PriorityQueue<RepairNode> open = new PriorityQueue<>(Comparator.comparingInt(node -> node.f));
        java.util.HashSet<State> closed = new java.util.HashSet<>();

        open.add(new RepairNode(start, weight * estimateRemainingCost(analyzer, start)));

        int expansions = 0;

        while (!open.isEmpty() && System.nanoTime() < deadline) {
            if (++expansions > maxExpansions) {
                System.err.format("Bounded global repair hit expansion limit. Open size: %,d.%n", open.size());
                return null;
            }

            State state = open.poll().state;

            if (state.isGoalState()) {
                System.err.format("Bounded global repair expanded %,d states.%n", expansions);
                return state.extractPlan();
            }

            if (!closed.add(state)) {
                continue;
            }

            for (State child : state.getExpandedStates()) {
                if (closed.contains(child)) {
                    continue;
                }

                int h = estimateRemainingCost(analyzer, child);
                open.add(new RepairNode(child, child.g() + weight * h));
            }
        }

        return null;
    }

    private static Action[][] focusedOneBoxRepair(
            State start,
            LevelAnalyzer analyzer,
            long deadline,
            int maxExpansions
    ) {
        UnsolvedBoxGoal target = findOnlyUnsolvedBoxGoal(start);

        if (target == null) {
            return null;
        }

        Color boxColor = State.boxColors[target.letter - 'A'];
        int bestExpansionCount = 0;

        for (int activeAgent = 0; activeAgent < start.agentRows.length; activeAgent++) {
            if (State.agentColors[activeAgent] != boxColor || System.nanoTime() >= deadline) {
                continue;
            }

            int[] helpers = selectFocusedHelpers(start, activeAgent, target.goal);
            int activeBoxDistanceLimit = closestBoxDistance(analyzer, start, target) + 4;
            PriorityQueue<FocusedNode> open = new PriorityQueue<>(Comparator.comparingInt(node -> node.f));
            HashMap<State, Integer> bestG = new HashMap<>();

            int h = focusedRepairHeuristic(analyzer, start, target);
            open.add(new FocusedNode(start, null, null, 0, h));

            int expansions = 0;

            while (!open.isEmpty() && System.nanoTime() < deadline) {
                if (++expansions > maxExpansions) {
                    System.err.format(
                            "Focused repair for agent %,d hit expansion limit. Open size: %,d.%n",
                            activeAgent,
                            open.size()
                    );
                    bestExpansionCount += expansions;
                    break;
                }

                FocusedNode node = open.poll();
                Integer seen = bestG.get(node.state);

                if (seen != null && seen <= node.g) {
                    continue;
                }

                bestG.put(node.state, node.g);

                if (allBoxGoalsSolved(node.state)) {
                    System.err.format(
                            "Focused one-box repair solved with agent %,d after %,d expansions.%n",
                            activeAgent,
                            bestExpansionCount + expansions
                    );
                    return extractFocusedPlan(node, start.agentRows.length);
                }

                for (FocusedStep step : expandFocused(
                        node.state,
                        activeAgent,
                        helpers,
                        target,
                        activeBoxDistanceLimit,
                        analyzer
                )) {
                    int nextG = node.g + 1;
                    Integer bestKnown = bestG.get(step.state);

                    if (bestKnown != null && bestKnown <= nextG) {
                        continue;
                    }

                    int nextH = focusedRepairHeuristic(analyzer, step.state, target);
                    open.add(new FocusedNode(
                            step.state,
                            node,
                            step.jointAction,
                            nextG,
                            nextG + 5 * nextH
                    ));
                }
            }

            bestExpansionCount += expansions;
        }

        return null;
    }

    private static Action[][] focusedSingleGoalRepair(
            State start,
            LevelAnalyzer analyzer,
            UnsolvedBoxGoal target,
            long deadline,
            int maxExpansions
    ) {
        Color boxColor = State.boxColors[target.letter - 'A'];
        int bestExpansionCount = 0;

        for (int activeAgent = 0; activeAgent < start.agentRows.length; activeAgent++) {
            if (State.agentColors[activeAgent] != boxColor || System.nanoTime() >= deadline) {
                continue;
            }

            int[] helpers = selectFocusedHelpers(start, activeAgent, target.goal);
            int activeBoxDistanceLimit = closestBoxDistance(analyzer, start, target) + 8;
            PriorityQueue<FocusedNode> open = new PriorityQueue<>(Comparator.comparingInt(node -> node.f));
            HashMap<State, Integer> bestG = new HashMap<>();

            int h = focusedRepairHeuristic(analyzer, start, target);
            open.add(new FocusedNode(start, null, null, 0, h));

            int expansions = 0;

            while (!open.isEmpty() && System.nanoTime() < deadline) {
                if (++expansions > maxExpansions) {
                    System.err.format(
                            "Focused single-goal repair for agent %,d hit expansion limit. Open size: %,d.%n",
                            activeAgent,
                            open.size()
                    );
                    bestExpansionCount += expansions;
                    break;
                }

                FocusedNode node = open.poll();
                Integer seen = bestG.get(node.state);

                if (seen != null && seen <= node.g) {
                    continue;
                }

                bestG.put(node.state, node.g);

                if (node.state.boxes[target.goal.row][target.goal.col] == target.letter) {
                    System.err.format(
                            "Focused single-goal repair solved %c at %s with agent %,d after %,d expansions.%n",
                            target.letter,
                            target.goal,
                            activeAgent,
                            bestExpansionCount + expansions
                    );
                    return extractFocusedPlan(node, start.agentRows.length);
                }

                for (FocusedStep step : expandFocused(
                        node.state,
                        activeAgent,
                        helpers,
                        target,
                        activeBoxDistanceLimit,
                        analyzer
                )) {
                    int nextG = node.g + 1;
                    Integer bestKnown = bestG.get(step.state);

                    if (bestKnown != null && bestKnown <= nextG) {
                        continue;
                    }

                    int nextH = focusedRepairHeuristic(analyzer, step.state, target);
                    open.add(new FocusedNode(
                            step.state,
                            node,
                            step.jointAction,
                            nextG,
                            nextG + 5 * nextH
                    ));
                }
            }

            bestExpansionCount += expansions;
        }

        return null;
    }

    private static PocketResult trySmallRemainingBoxRepair(
            State start,
            ArrayList<Action[]> basePlan,
            LevelAnalyzer analyzer,
            long deadline
    ) {
        if (countUnsolvedBoxGoals(start) > 2) {
            return null;
        }

        State current = copyState(start);
        ArrayList<Action[]> plan = copyPlan(basePlan);
        int previousSolved = countSolvedGoals(current);

        for (int pass = 0; pass < 2 && countUnsolvedBoxGoals(current) > 0; pass++) {
            ArrayList<UnsolvedBoxGoal> targets = remainingBoxGoals(current);
            final State sortingState = current;
            targets.sort(Comparator
                    .comparingInt((UnsolvedBoxGoal target) -> -closestBoxDistance(analyzer, sortingState, target))
                    .thenComparingInt(target -> target.goal.row)
                    .thenComparingInt(target -> target.goal.col));

            boolean progress = false;

            for (UnsolvedBoxGoal target : targets) {
                if (System.nanoTime() >= deadline || current.boxes[target.goal.row][target.goal.col] == target.letter) {
                    continue;
                }

                long seconds = current.agentRows.length <= 4 ? 10L : 6L;
                int expansions = current.agentRows.length <= 4 ? 1_200_000 : 700_000;
                long targetDeadline = Math.min(deadline, System.nanoTime() + seconds * 1_000_000_000L);

                Action[][] repair = focusedSingleGoalRepair(
                        current,
                        analyzer,
                        target,
                        targetDeadline,
                        expansions
                );

                if (repair == null || repair.length == 0 || plan.size() + repair.length > MAX_TOTAL_ACTIONS) {
                    continue;
                }

                appendJointPlan(plan, repair);
                current = simulateJointPlan(current, repair);
                progress = true;

                System.err.format(
                        "Small remaining-box repair placed %c at %s using %,d actions.%n",
                        target.letter,
                        target.goal,
                        repair.length
                );

                if (allBoxGoalsSolved(current)) {
                    return new PocketResult(current, plan);
                }
            }

            if (!progress) {
                break;
            }
        }

        return countSolvedGoals(current) > previousSolved ? new PocketResult(current, plan) : null;
    }

    private static PocketResult tryVerifiedSmallEndgameBoxRepair(
            State start,
            ArrayList<Action[]> basePlan,
            LevelAnalyzer analyzer,
            SingleAgentPlanner planner,
            AgentRetreatPlanner retreatPlanner,
            long deadline
    ) {
        int initialUnsolvedBoxes = countUnsolvedBoxGoals(start);
        if (initialUnsolvedBoxes < 1) {
            return null;
        }

        State current = copyState(start);
        ArrayList<Action[]> plan = copyPlan(basePlan);
        long repairDeadline = Math.min(deadline, System.nanoTime() + 12L * 1_000_000_000L);
        int initialSolvedGoals = countSolvedGoals(current);
        int initialSolvedBoxGoals = countSolvedBoxGoals(current);
        System.err.format(
                "Starting verified small-endgame box repair with %,d unsolved box goal(s).%n",
                initialUnsolvedBoxes
        );

        boolean anyProgress = false;
        for (int pass = 0; pass < 2
                && countUnsolvedBoxGoals(current) > 0
                && System.nanoTime() < repairDeadline; pass++) {
            ArrayList<ArrayList<UnsolvedBoxGoal>> components = orderSmallEndgameComponents(current, analyzer);
            System.err.format(
                    "Small-endgame component pass %,d: %,d remaining box goal(s), %,d component(s).%n",
                    pass + 1,
                    countUnsolvedBoxGoals(current),
                    components.size()
            );
            boolean passProgress = false;

            for (ArrayList<UnsolvedBoxGoal> component : components) {
                if (System.nanoTime() >= repairDeadline) {
                    break;
                }

                if (component.isEmpty() || component.size() > 6) {
                    continue;
                }

                RepairRegion componentRegion = buildSmallEndgameRepairRegion(current, analyzer, component);
                if (componentRegion == null) {
                    System.err.format(
                            "Small-endgame component rejected by region size/access: %s.%n",
                            describeTargets(component)
                    );
                    continue;
                }

                int area = repairRegionArea(componentRegion);
                long componentDeadline = Math.min(repairDeadline, System.nanoTime() + 4L * 1_000_000_000L);
                ArrayList<UnsolvedBoxGoal> targets = orderSmallEndgameTargets(current, analyzer, component);
                System.err.format(
                        "Small-endgame component %s region rows %d..%d cols %d..%d area %,d.%n",
                        describeTargets(targets),
                        componentRegion.minRow,
                        componentRegion.maxRow,
                        componentRegion.minCol,
                        componentRegion.maxCol,
                        area
                );

                for (UnsolvedBoxGoal target : targets) {
                    if (System.nanoTime() >= componentDeadline
                            || current.boxes[target.goal.row][target.goal.col] == target.letter) {
                        continue;
                    }

                    System.err.format("Small-endgame attempting %c at %s.%n", target.letter, target.goal);
                    PocketResult candidate = trySmallEndgameTarget(
                            current,
                            plan,
                            analyzer,
                            planner,
                            retreatPlanner,
                            target,
                            targets,
                            componentDeadline
                    );

                    if (candidate == null) {
                        System.err.format("Small-endgame target %c at %s failed.%n", target.letter, target.goal);
                        continue;
                    }

                    int beforeUnsolvedBoxes = countUnsolvedBoxGoals(current);
                    int beforeSolvedGoals = countSolvedGoals(current);
                    int beforeSolvedBoxGoals = countSolvedBoxGoals(current);
                    int afterUnsolvedBoxes = countUnsolvedBoxGoals(candidate.state);
                    int afterSolvedGoals = countSolvedGoals(candidate.state);
                    int afterSolvedBoxGoals = countSolvedBoxGoals(candidate.state);

                    if (afterSolvedBoxGoals < beforeSolvedBoxGoals) {
                        System.err.println("Small-endgame rejected candidate: solved box goal would regress.");
                        continue;
                    }

                    if (afterUnsolvedBoxes >= beforeUnsolvedBoxes && afterSolvedGoals <= beforeSolvedGoals) {
                        System.err.println("Small-endgame rejected candidate: no verified improvement.");
                        continue;
                    }

                    current = candidate.state;
                    plan = candidate.plan;
                    anyProgress = true;
                    passProgress = true;
                    System.err.format(
                            "Small-endgame accepted %c at %s. Solved goals %,d, unsolved box goals %,d.%n",
                            target.letter,
                            target.goal,
                            afterSolvedGoals,
                            afterUnsolvedBoxes
                    );

                    if (allBoxGoalsSolved(current) || System.nanoTime() >= componentDeadline) {
                        break;
                    }
                }

                if (passProgress || allBoxGoalsSolved(current)) {
                    break;
                }
            }

            if (!passProgress) {
                break;
            }
        }

        System.err.format(
                "Finished verified small-endgame repair. Solved goals %,d -> %,d, unsolved box goals %,d -> %,d.%n",
                initialSolvedGoals,
                countSolvedGoals(current),
                initialUnsolvedBoxes,
                countUnsolvedBoxGoals(current)
        );

        if (!anyProgress || countSolvedBoxGoals(current) < initialSolvedBoxGoals) {
            return null;
        }

        return new PocketResult(current, plan);
    }

    private static PocketResult trySmallEndgameTarget(
            State start,
            ArrayList<Action[]> basePlan,
            LevelAnalyzer analyzer,
            SingleAgentPlanner planner,
            AgentRetreatPlanner retreatPlanner,
            UnsolvedBoxGoal target,
            ArrayList<UnsolvedBoxGoal> componentTargets,
            long deadline
    ) {
        PocketResult direct = trySmallEndgameDirect(
                start,
                basePlan,
                analyzer,
                planner,
                retreatPlanner,
                target,
                deadline,
                false
        );
        if (direct != null) {
            System.err.println("Small-endgame direct planner succeeded.");
            return direct;
        }
        System.err.println("Small-endgame direct planner failed.");

        PocketResult relaxed = trySmallEndgameDirect(
                start,
                basePlan,
                analyzer,
                planner,
                retreatPlanner,
                target,
                deadline,
                true
        );
        if (relaxed != null) {
            System.err.println("Small-endgame relaxed planner succeeded.");
            return relaxed;
        }
        System.err.println("Small-endgame relaxed planner failed.");

        PocketResult local = trySmallEndgameLocalSearch(start, basePlan, analyzer, target, componentTargets, deadline);
        if (local != null) {
            System.err.println("Small-endgame local region search succeeded.");
            return local;
        }
        System.err.println("Small-endgame local region search failed.");

        PocketResult focused = trySmallEndgameFocused(start, basePlan, analyzer, target, deadline);
        if (focused != null) {
            System.err.println("Small-endgame focused repair succeeded.");
            return focused;
        }
        System.err.println("Small-endgame focused repair failed.");

        PocketResult staged = trySmallEndgameWithStaging(
                start,
                basePlan,
                analyzer,
                planner,
                retreatPlanner,
                target,
                deadline
        );
        if (staged != null) {
            System.err.println("Small-endgame blocker staging succeeded.");
            return staged;
        }
        System.err.println("Small-endgame blocker staging failed.");
        return null;
    }

    private static PocketResult trySmallEndgameDirect(
            State start,
            ArrayList<Action[]> basePlan,
            LevelAnalyzer analyzer,
            SingleAgentPlanner planner,
            AgentRetreatPlanner retreatPlanner,
            UnsolvedBoxGoal target,
            long deadline,
            boolean relaxed
    ) {
        for (Task task : smallEndgameTasksForTarget(start, analyzer, target)) {
            if (System.nanoTime() >= deadline) {
                return null;
            }

            List<Action> boxPlan = relaxed
                    ? planner.planBoxToGoalRelaxed(start, task, new ReservationTable())
                    : planner.planBoxToGoal(start, task, new ReservationTable());

            if (boxPlan == null || boxPlan.isEmpty()
                    || basePlan.size() + boxPlan.size() > MAX_TOTAL_ACTIONS) {
                continue;
            }

            PocketResult candidate = appendValidatedSingleAgentPlan(
                    start,
                    basePlan,
                    boxPlan,
                    task.assignedAgent
            );

            if (candidate == null) {
                continue;
            }

            List<Action> retreat = retreatPlanner.planRetreat(candidate.state, task.assignedAgent, 8);
            if (retreat != null && !retreat.isEmpty()
                    && candidate.plan.size() + retreat.size() <= MAX_TOTAL_ACTIONS) {
                PocketResult withRetreat = appendValidatedSingleAgentPlan(
                        candidate.state,
                        candidate.plan,
                        retreat,
                        task.assignedAgent
                );

                if (withRetreat != null) {
                    candidate = withRetreat;
                }
            }

            if (candidate.state.boxes[target.goal.row][target.goal.col] == target.letter) {
                return candidate;
            }
        }

        return null;
    }

    private static PocketResult trySmallEndgameFocused(
            State start,
            ArrayList<Action[]> basePlan,
            LevelAnalyzer analyzer,
            UnsolvedBoxGoal target,
            long deadline
    ) {
        long targetDeadline = Math.min(deadline, System.nanoTime() + 8L * 1_000_000_000L);
        Action[][] repair = focusedSingleGoalRepair(start, analyzer, target, targetDeadline, 900_000);

        if (repair == null || repair.length == 0 || basePlan.size() + repair.length > MAX_TOTAL_ACTIONS) {
            return null;
        }

        State repaired = simulateValidatedJointPlan(start, java.util.Arrays.asList(repair));
        if (repaired == null || repaired.boxes[target.goal.row][target.goal.col] != target.letter) {
            return null;
        }

        ArrayList<Action[]> updatedPlan = copyPlan(basePlan);
        appendJointPlan(updatedPlan, repair);
        return new PocketResult(repaired, updatedPlan);
    }

    private static PocketResult trySmallEndgameLocalSearch(
            State start,
            ArrayList<Action[]> basePlan,
            LevelAnalyzer analyzer,
            UnsolvedBoxGoal target,
            ArrayList<UnsolvedBoxGoal> componentTargets,
            long deadline
    ) {
        ArrayList<UnsolvedBoxGoal> targets = componentTargets == null
                ? remainingBoxGoals(start)
                : new ArrayList<>(componentTargets);
        RepairRegion region = buildSmallEndgameRepairRegion(start, analyzer, targets);

        if (region == null) {
            System.err.println("Small-endgame local region rejected before search.");
            return null;
        }

        long localDeadline = Math.min(deadline, System.nanoTime() + 750L * 1_000_000L);
        int maxExpansions = smallEndgameLocalExpansionLimit(region);

        for (int activeAgent : localRepairAgents(start, region, target)) {
            if (System.nanoTime() >= localDeadline) {
                return null;
            }

            Action[][] repair = boundedSmallEndgameLocalSearch(
                    start,
                    analyzer,
                    targets,
                    target,
                    region,
                    activeAgent,
                    localDeadline,
                    maxExpansions
            );

            if (repair == null || repair.length == 0 || basePlan.size() + repair.length > MAX_TOTAL_ACTIONS) {
                continue;
            }

            State repaired = simulateValidatedJointPlan(start, java.util.Arrays.asList(repair));
            if (repaired == null) {
                continue;
            }

            if (countSolvedBoxGoals(repaired) < countSolvedBoxGoals(start)) {
                continue;
            }

            if (repaired.boxes[target.goal.row][target.goal.col] != target.letter
                    && countUnsolvedBoxGoals(repaired) >= countUnsolvedBoxGoals(start)) {
                continue;
            }

            ArrayList<Action[]> updatedPlan = copyPlan(basePlan);
            appendJointPlan(updatedPlan, repair);
            return new PocketResult(repaired, updatedPlan);
        }

        return null;
    }

    private static Action[][] boundedSmallEndgameLocalSearch(
            State start,
            LevelAnalyzer analyzer,
            ArrayList<UnsolvedBoxGoal> targets,
            UnsolvedBoxGoal primaryTarget,
            RepairRegion region,
            int activeAgent,
            long deadline,
            int maxExpansions
    ) {
        PriorityQueue<FocusedNode> open = new PriorityQueue<>(Comparator.comparingInt(node -> node.f));
        HashMap<State, Integer> bestG = new HashMap<>();
        int initialSolvedBoxGoals = countSolvedBoxGoals(start);
        int initialUnsolvedBoxGoals = countUnsolvedBoxGoals(start);

        open.add(new FocusedNode(
                start,
                null,
                null,
                0,
                smallEndgameLocalHeuristic(start, analyzer, targets, primaryTarget)
        ));

        int expansions = 0;

        while (!open.isEmpty() && System.nanoTime() < deadline) {
            if (++expansions > maxExpansions) {
                System.err.format("Small-endgame local search hit expansion limit. Open size: %,d.%n", open.size());
                return null;
            }

            FocusedNode node = open.poll();
            Integer seen = bestG.get(node.state);

            if (seen != null && seen <= node.g) {
                continue;
            }

            bestG.put(node.state, node.g);

            if (countSolvedBoxGoals(node.state) >= initialSolvedBoxGoals
                    && (node.state.boxes[primaryTarget.goal.row][primaryTarget.goal.col] == primaryTarget.letter
                    || countUnsolvedBoxGoals(node.state) < initialUnsolvedBoxGoals
                    || node.state.isGoalState())) {
                System.err.format("Small-endgame local search expanded %,d states.%n", expansions);
                return extractFocusedPlan(node, start.agentRows.length);
            }

            for (FocusedStep step : expandSmallEndgameLocal(
                    node.state,
                    analyzer,
                    targets,
                    primaryTarget,
                    region,
                    activeAgent
            )) {
                int nextG = node.g + 1;
                Integer bestKnown = bestG.get(step.state);

                if (bestKnown != null && bestKnown <= nextG) {
                    continue;
                }

                if (countSolvedBoxGoals(step.state) < initialSolvedBoxGoals) {
                    continue;
                }

                int h = smallEndgameLocalHeuristic(step.state, analyzer, targets, primaryTarget);
                open.add(new FocusedNode(step.state, node, step.jointAction, nextG, nextG + 6 * h));
            }
        }

        return null;
    }

    private static ArrayList<FocusedStep> expandSmallEndgameLocal(
            State state,
            LevelAnalyzer analyzer,
            ArrayList<UnsolvedBoxGoal> targets,
            UnsolvedBoxGoal primaryTarget,
            RepairRegion region,
            int activeAgent
    ) {
        int numAgents = state.agentRows.length;
        Action[][] actionSets = new Action[numAgents][];

        for (int agent = 0; agent < numAgents; agent++) {
            if (agent == activeAgent) {
                actionSets[agent] = applicableSmallEndgameLocalActions(
                        state,
                        analyzer,
                        targets,
                        primaryTarget,
                        region,
                        agent,
                        true
                );
            } else if (localHelperRelevant(state, region, agent)) {
                actionSets[agent] = applicableSmallEndgameLocalActions(
                        state,
                        analyzer,
                        targets,
                        primaryTarget,
                        region,
                        agent,
                        false
                );
            } else {
                actionSets[agent] = new Action[] { Action.NoOp };
            }
        }

        ArrayList<FocusedStep> result = new ArrayList<>();
        Action[] jointAction = new Action[numAgents];
        enumerateFocusedActions(state, actionSets, jointAction, 0, result);
        result.sort(Comparator
                .comparingInt((FocusedStep step) -> smallEndgameLocalStepScore(step, analyzer, targets, primaryTarget, region))
                .thenComparingInt(step -> movingActionCount(step.jointAction)));
        return result;
    }

    private static Action[] applicableSmallEndgameLocalActions(
            State state,
            LevelAnalyzer analyzer,
            ArrayList<UnsolvedBoxGoal> targets,
            UnsolvedBoxGoal primaryTarget,
            RepairRegion region,
            int agent,
            boolean canMoveBoxes
    ) {
        ArrayList<Action> actions = new ArrayList<>();

        for (Action action : Action.values()) {
            if (action.type == ActionType.NoOp) {
                actions.add(action);
                continue;
            }

            if (!isApplicable(state, agent, action)) {
                continue;
            }

            if (!localActionStaysInRegion(state, agent, action, region)) {
                continue;
            }

            if (!canMoveBoxes && action.type != ActionType.Move) {
                continue;
            }

            if ((action.type == ActionType.Push || action.type == ActionType.Pull)
                    && !smallEndgameBoxMoveAllowed(state, analyzer, targets, primaryTarget, agent, action, region)) {
                continue;
            }

            actions.add(action);
        }

        actions.sort(Comparator.comparingInt(action -> smallEndgameActionPriority(state, agent, action, primaryTarget)));
        if (actions.size() > 5) {
            return actions.subList(0, 5).toArray(new Action[0]);
        }
        return actions.toArray(new Action[0]);
    }

    private static boolean smallEndgameBoxMoveAllowed(
            State state,
            LevelAnalyzer analyzer,
            ArrayList<UnsolvedBoxGoal> targets,
            UnsolvedBoxGoal primaryTarget,
            int agent,
            Action action,
            RepairRegion region
    ) {
        int agentRow = state.agentRows[agent];
        int agentCol = state.agentCols[agent];
        int boxRow = action.type == ActionType.Push
                ? agentRow + action.agentRowDelta
                : agentRow - action.boxRowDelta;
        int boxCol = action.type == ActionType.Push
                ? agentCol + action.agentColDelta
                : agentCol - action.boxColDelta;
        int newBoxRow = boxRow + action.boxRowDelta;
        int newBoxCol = boxCol + action.boxColDelta;

        if (!inBounds(boxRow, boxCol) || !region.containsWithMargin(newBoxRow, newBoxCol, 0)) {
            return false;
        }

        char box = state.boxes[boxRow][boxCol];
        if (!('A' <= box && box <= 'Z')) {
            return false;
        }

        if (State.goals[boxRow][boxCol] == box && !blocksRemainingTarget(state, targets, new Position(boxRow, boxCol))) {
            return false;
        }

        if (!isRemainingTargetLetter(targets, box) && !wrongGoalBlockerForRemainingTarget(state, targets, boxRow, boxCol)) {
            return false;
        }

        Position oldBox = new Position(boxRow, boxCol);
        Position newBox = new Position(newBoxRow, newBoxCol);
        int before = nearestMatchingRemainingGoalDistance(analyzer, targets, box, oldBox);
        int after = nearestMatchingRemainingGoalDistance(analyzer, targets, box, newBox);

        return box == primaryTarget.letter || after <= before + 2 || wrongGoalBlockerForRemainingTarget(state, targets, boxRow, boxCol);
    }

    private static boolean localActionStaysInRegion(State state, int agent, Action action, RepairRegion region) {
        int agentRow = state.agentRows[agent];
        int agentCol = state.agentCols[agent];
        int newAgentRow = agentRow + action.agentRowDelta;
        int newAgentCol = agentCol + action.agentColDelta;

        if (!region.containsWithMargin(newAgentRow, newAgentCol, 0)) {
            return false;
        }

        if (action.type == ActionType.Push || action.type == ActionType.Pull) {
            int boxRow = action.type == ActionType.Push
                    ? agentRow + action.agentRowDelta
                    : agentRow - action.boxRowDelta;
            int boxCol = action.type == ActionType.Push
                    ? agentCol + action.agentColDelta
                    : agentCol - action.boxColDelta;
            int newBoxRow = boxRow + action.boxRowDelta;
            int newBoxCol = boxCol + action.boxColDelta;
            return region.containsWithMargin(boxRow, boxCol, 0)
                    && region.containsWithMargin(newBoxRow, newBoxCol, 0);
        }

        return true;
    }

    private static RepairRegion buildSmallEndgameRepairRegion(
            State state,
            LevelAnalyzer analyzer,
            ArrayList<UnsolvedBoxGoal> targets
    ) {
        if (targets.isEmpty() || targets.size() > 6) {
            return null;
        }

        int minRow = analyzer.rows;
        int maxRow = -1;
        int minCol = analyzer.cols;
        int maxCol = -1;

        for (UnsolvedBoxGoal target : targets) {
            minRow = Math.min(minRow, target.goal.row);
            maxRow = Math.max(maxRow, target.goal.row);
            minCol = Math.min(minCol, target.goal.col);
            maxCol = Math.max(maxCol, target.goal.col);
        }

        HashSet<Character> letters = new HashSet<>();
        for (UnsolvedBoxGoal target : targets) {
            letters.add(target.letter);
        }

        for (int r = 0; r < state.boxes.length; r++) {
            for (int c = 0; c < state.boxes[r].length; c++) {
                char box = state.boxes[r][c];
                Position boxPos = new Position(r, c);
                if ((letters.contains(box) && boxRelevantToComponent(analyzer, targets, box, boxPos))
                        || wrongGoalBlockerForRemainingTarget(state, targets, r, c)) {
                    minRow = Math.min(minRow, r);
                    maxRow = Math.max(maxRow, r);
                    minCol = Math.min(minCol, c);
                    maxCol = Math.max(maxCol, c);
                }
            }
        }

        for (UnsolvedBoxGoal target : targets) {
            for (Position cell : adjacentCells(target.goal)) {
                if (!inBounds(cell.row, cell.col)) {
                    continue;
                }

                char blocker = state.boxes[cell.row][cell.col];
                if ('A' <= blocker && blocker <= 'Z' && State.goals[cell.row][cell.col] != blocker) {
                    minRow = Math.min(minRow, cell.row);
                    maxRow = Math.max(maxRow, cell.row);
                    minCol = Math.min(minCol, cell.col);
                    maxCol = Math.max(maxCol, cell.col);
                }
            }
        }

        if (maxRow < minRow || maxCol < minCol) {
            return null;
        }

        int margin = 4;
        minRow = Math.max(0, minRow - margin);
        maxRow = Math.min(analyzer.rows - 1, maxRow + margin);
        minCol = Math.max(0, minCol - margin);
        maxCol = Math.min(analyzer.cols - 1, maxCol + margin);

        int area = (maxRow - minRow + 1) * (maxCol - minCol + 1);
        if (area > 260) {
            return null;
        }

        return new RepairRegion(minRow, maxRow, minCol, maxCol);
    }

    private static boolean boxRelevantToComponent(
            LevelAnalyzer analyzer,
            ArrayList<UnsolvedBoxGoal> targets,
            char box,
            Position boxPos
    ) {
        if (!('A' <= box && box <= 'Z')) {
            return false;
        }

        int bestDistance = LevelAnalyzer.INF;
        for (UnsolvedBoxGoal target : targets) {
            if (target.letter == box) {
                bestDistance = Math.min(bestDistance, analyzer.distance(boxPos, target.goal));
            }
        }

        return bestDistance <= 22;
    }

    private static int smallEndgameLocalExpansionLimit(RepairRegion region) {
        int area = repairRegionArea(region);
        return Math.max(4_000, Math.min(12_000, 80 * area));
    }

    private static int repairRegionArea(RepairRegion region) {
        return (region.maxRow - region.minRow + 1) * (region.maxCol - region.minCol + 1);
    }

    private static ArrayList<Integer> localRepairAgents(State state, RepairRegion region, UnsolvedBoxGoal target) {
        ArrayList<Integer> agents = new ArrayList<>();
        Color color = State.boxColors[target.letter - 'A'];

        for (int agent = 0; agent < state.agentRows.length; agent++) {
            if (State.agentColors[agent] == color) {
                agents.add(agent);
            }
        }

        agents.sort(Comparator.comparingInt(agent -> distanceToRegion(state.agentRows[agent], state.agentCols[agent], region)));
        if (agents.size() > 3) {
            return new ArrayList<>(agents.subList(0, 3));
        }
        return agents;
    }

    private static boolean localHelperRelevant(State state, RepairRegion region, int agent) {
        if (region.containsWithMargin(state.agentRows[agent], state.agentCols[agent], 2)) {
            return true;
        }
        return distanceToRegion(state.agentRows[agent], state.agentCols[agent], region) <= 4;
    }

    private static int smallEndgameLocalHeuristic(
            State state,
            LevelAnalyzer analyzer,
            ArrayList<UnsolvedBoxGoal> targets,
            UnsolvedBoxGoal primaryTarget
    ) {
        int score = 500 * countUnsolvedBoxGoals(state);

        for (UnsolvedBoxGoal target : targets) {
            if (state.boxes[target.goal.row][target.goal.col] == target.letter) {
                continue;
            }

            score += 20 * closestBoxDistance(analyzer, state, target);
            if (target == primaryTarget) {
                score += 30 * closestBoxDistance(analyzer, state, target);
            }

            if (wrongBoxPressure(state, target)) {
                score += 200;
            }
        }

        return score;
    }

    private static int smallEndgameLocalStepScore(
            FocusedStep step,
            LevelAnalyzer analyzer,
            ArrayList<UnsolvedBoxGoal> targets,
            UnsolvedBoxGoal primaryTarget,
            RepairRegion region
    ) {
        int score = smallEndgameLocalHeuristic(step.state, analyzer, targets, primaryTarget);

        for (int agent = 0; agent < step.state.agentRows.length; agent++) {
            score += distanceToRegion(step.state.agentRows[agent], step.state.agentCols[agent], region);
        }

        return score;
    }

    private static int smallEndgameActionPriority(State state, int agent, Action action, UnsolvedBoxGoal target) {
        if (action.type == ActionType.NoOp) {
            return 50;
        }

        int nextRow = state.agentRows[agent] + action.agentRowDelta;
        int nextCol = state.agentCols[agent] + action.agentColDelta;
        int score = Math.abs(nextRow - target.goal.row) + Math.abs(nextCol - target.goal.col);

        if (action.type == ActionType.Push || action.type == ActionType.Pull) {
            score -= 20;
        }

        return score;
    }

    private static boolean isRemainingTargetLetter(ArrayList<UnsolvedBoxGoal> targets, char letter) {
        for (UnsolvedBoxGoal target : targets) {
            if (target.letter == letter) {
                return true;
            }
        }
        return false;
    }

    private static boolean wrongGoalBlockerForRemainingTarget(
            State state,
            ArrayList<UnsolvedBoxGoal> targets,
            int row,
            int col
    ) {
        char box = state.boxes[row][col];
        if (!('A' <= box && box <= 'Z')) {
            return false;
        }

        for (UnsolvedBoxGoal target : targets) {
            if (target.goal.row == row && target.goal.col == col && box != target.letter) {
                return true;
            }
        }

        return false;
    }

    private static boolean blocksRemainingTarget(
            State state,
            ArrayList<UnsolvedBoxGoal> targets,
            Position boxPosition
    ) {
        for (UnsolvedBoxGoal target : targets) {
            if (boxPosition.manhattanDistance(target.goal) <= 1 && state.boxes[target.goal.row][target.goal.col] != target.letter) {
                return true;
            }
        }
        return false;
    }

    private static int nearestMatchingRemainingGoalDistance(
            LevelAnalyzer analyzer,
            ArrayList<UnsolvedBoxGoal> targets,
            char letter,
            Position box
    ) {
        int best = LevelAnalyzer.INF;
        for (UnsolvedBoxGoal target : targets) {
            if (target.letter == letter) {
                best = Math.min(best, analyzer.distance(box, target.goal));
            }
        }
        return best;
    }

    private static PocketResult trySmallEndgameWithStaging(
            State start,
            ArrayList<Action[]> basePlan,
            LevelAnalyzer analyzer,
            SingleAgentPlanner planner,
            AgentRetreatPlanner retreatPlanner,
            UnsolvedBoxGoal target,
            long deadline
    ) {
        ArrayList<Position> blockers = smallEndgameBlockers(start, target);

        for (Position blockerPos : blockers) {
            if (System.nanoTime() >= deadline) {
                return null;
            }

            char blocker = start.boxes[blockerPos.row][blockerPos.col];
            if (!('A' <= blocker && blocker <= 'Z')
                    || State.goals[blockerPos.row][blockerPos.col] == blocker) {
                continue;
            }

            for (Position destination : blockerClearDestinations(start, blockerPos, blocker)) {
                if (System.nanoTime() >= deadline) {
                    return null;
                }

                PocketResult staged = tryMoveSpecificBoxTo(
                        start,
                        basePlan,
                        planner,
                        blocker,
                        blockerPos,
                        destination,
                        deadline
                );

                if (staged == null || countSolvedBoxGoals(staged.state) < countSolvedBoxGoals(start)) {
                    continue;
                }

                PocketResult placed = trySmallEndgameDirect(
                        staged.state,
                        staged.plan,
                        analyzer,
                        planner,
                        retreatPlanner,
                        target,
                        deadline,
                        false
                );

                if (placed == null) {
                    placed = trySmallEndgameDirect(
                            staged.state,
                            staged.plan,
                            analyzer,
                            planner,
                            retreatPlanner,
                            target,
                            deadline,
                            true
                    );
                }

                if (placed == null) {
                    placed = trySmallEndgameFocused(staged.state, staged.plan, analyzer, target, deadline);
                }

                if (placed != null
                        && placed.state.boxes[target.goal.row][target.goal.col] == target.letter
                        && countSolvedBoxGoals(placed.state) >= countSolvedBoxGoals(start)) {
                    return placed;
                }
            }
        }

        return null;
    }

    private static PocketResult tryMoveSpecificBoxTo(
            State start,
            ArrayList<Action[]> basePlan,
            SingleAgentPlanner planner,
            char box,
            Position source,
            Position destination,
            long deadline
    ) {
        if (start.boxes[source.row][source.col] != box || !cellIsFree(start, destination.row, destination.col)) {
            return null;
        }

        for (int agent : agentsWithBoxColorByManhattan(start, box, source)) {
            if (System.nanoTime() >= deadline) {
                return null;
            }

            Task task = new Task(box, source, destination, agent, 0);
            List<Action> plan = planner.planBoxToGoal(start, task, new ReservationTable());

            if (plan == null || plan.isEmpty()) {
                plan = planner.planBoxToGoalRelaxed(start, task, new ReservationTable());
            }

            if (plan == null || plan.isEmpty() || basePlan.size() + plan.size() > MAX_TOTAL_ACTIONS) {
                continue;
            }

            PocketResult candidate = appendValidatedSingleAgentPlan(start, basePlan, plan, agent);
            if (candidate != null && candidate.state.boxes[destination.row][destination.col] == box) {
                System.err.format(
                        "Small-endgame staged blocker %c from %s to %s using %,d actions.%n",
                        box,
                        source,
                        destination,
                        plan.size()
                );
                return candidate;
            }
        }

        return null;
    }

    private static PocketResult appendValidatedSingleAgentPlan(
            State start,
            ArrayList<Action[]> basePlan,
            List<Action> singleAgentPlan,
            int activeAgent
    ) {
        ArrayList<Action[]> suffix = new ArrayList<>();
        appendAsJointActions(suffix, singleAgentPlan, activeAgent, start.agentRows.length);
        State next = simulateValidatedJointPlan(start, suffix);

        if (next == null) {
            return null;
        }

        ArrayList<Action[]> updatedPlan = copyPlan(basePlan);
        updatedPlan.addAll(suffix);
        return new PocketResult(next, updatedPlan);
    }

    private static ArrayList<Task> smallEndgameTasksForTarget(
            State state,
            LevelAnalyzer analyzer,
            UnsolvedBoxGoal target
    ) {
        ArrayList<Task> tasks = new ArrayList<>();

        for (int r = 0; r < state.boxes.length; r++) {
            for (int c = 0; c < state.boxes[r].length; c++) {
                if (state.boxes[r][c] != target.letter) {
                    continue;
                }

                Position box = new Position(r, c);
                if (State.goals[r][c] == target.letter && !box.equals(target.goal)) {
                    continue;
                }

                for (int agent = 0; agent < state.agentRows.length; agent++) {
                    if (State.agentColors[agent] == State.boxColors[target.letter - 'A']) {
                        int priority = analyzer.distance(box, target.goal)
                                + analyzer.distance(new Position(state.agentRows[agent], state.agentCols[agent]), box);
                        tasks.add(new Task(target.letter, box, target.goal, agent, priority));
                    }
                }
            }
        }

        tasks.sort(Comparator
                .comparingInt((Task task) -> task.priority)
                .thenComparingInt(task -> task.boxStart.manhattanDistance(target.goal))
                .thenComparingInt(task -> task.assignedAgent));
        return tasks;
    }

    private static ArrayList<UnsolvedBoxGoal> orderSmallEndgameTargets(State state, LevelAnalyzer analyzer) {
        return orderSmallEndgameTargets(state, analyzer, remainingBoxGoals(state));
    }

    private static ArrayList<UnsolvedBoxGoal> orderSmallEndgameTargets(
            State state,
            LevelAnalyzer analyzer,
            ArrayList<UnsolvedBoxGoal> targets
    ) {
        targets = new ArrayList<>(targets);

        targets.sort(Comparator
                .comparingInt((UnsolvedBoxGoal target) -> wrongBoxPressure(state, target) ? 0 : 1)
                .thenComparingInt(target -> boundaryOrCornerPressure(analyzer, target.goal) ? 0 : 1)
                .thenComparingInt((UnsolvedBoxGoal target) -> -goalConstraintScore(state, analyzer, target.goal))
                .thenComparingInt(target -> closestBoxDistance(analyzer, state, target))
                .thenComparingInt(target -> target.goal.row)
                .thenComparingInt(target -> target.goal.col));
        return targets;
    }

    private static ArrayList<ArrayList<UnsolvedBoxGoal>> orderSmallEndgameComponents(
            State state,
            LevelAnalyzer analyzer
    ) {
        ArrayList<UnsolvedBoxGoal> remaining = remainingBoxGoals(state);
        ArrayList<ArrayList<UnsolvedBoxGoal>> components = splitSmallEndgameComponents(state, analyzer, remaining);
        components = splitOversizedSmallEndgameComponents(state, analyzer, components);

        components.sort(Comparator
                .comparingInt((ArrayList<UnsolvedBoxGoal> component) -> component.size())
                .thenComparingInt(component -> candidateBoxCountForComponent(state, component))
                .thenComparingInt(component -> boundaryComponent(component, analyzer) ? 0 : 1)
                .thenComparingInt(component -> -wrongGoalBlockerCount(state, component))
                .thenComparingInt(component -> componentBoundingArea(component)));
        return components;
    }

    private static ArrayList<ArrayList<UnsolvedBoxGoal>> splitOversizedSmallEndgameComponents(
            State state,
            LevelAnalyzer analyzer,
            ArrayList<ArrayList<UnsolvedBoxGoal>> components
    ) {
        ArrayList<ArrayList<UnsolvedBoxGoal>> result = new ArrayList<>();

        for (ArrayList<UnsolvedBoxGoal> component : components) {
            if (component.size() <= 6) {
                result.add(component);
                continue;
            }

            ArrayList<UnsolvedBoxGoal> ordered = orderSmallEndgameTargets(state, analyzer, component);
            for (UnsolvedBoxGoal target : ordered) {
                ArrayList<UnsolvedBoxGoal> singleton = new ArrayList<>();
                singleton.add(target);
                result.add(singleton);
            }
        }

        return result;
    }

    private static ArrayList<ArrayList<UnsolvedBoxGoal>> splitSmallEndgameComponents(
            State state,
            LevelAnalyzer analyzer,
            ArrayList<UnsolvedBoxGoal> goals
    ) {
        ArrayList<ArrayList<UnsolvedBoxGoal>> components = new ArrayList<>();
        boolean[] seen = new boolean[goals.size()];

        for (int i = 0; i < goals.size(); i++) {
            if (seen[i]) {
                continue;
            }

            ArrayDeque<Integer> queue = new ArrayDeque<>();
            ArrayList<UnsolvedBoxGoal> component = new ArrayList<>();
            seen[i] = true;
            queue.add(i);

            while (!queue.isEmpty()) {
                int index = queue.removeFirst();
                UnsolvedBoxGoal goal = goals.get(index);
                component.add(goal);

                for (int j = 0; j < goals.size(); j++) {
                    if (!seen[j] && goalsBelongToSameSmallComponent(state, analyzer, goal, goals.get(j))) {
                        seen[j] = true;
                        queue.add(j);
                    }
                }
            }

            components.add(component);
        }

        return components;
    }

    private static boolean goalsBelongToSameSmallComponent(
            State state,
            LevelAnalyzer analyzer,
            UnsolvedBoxGoal a,
            UnsolvedBoxGoal b
    ) {
        int goalDistance = a.goal.manhattanDistance(b.goal);
        if (goalDistance <= 8) {
            return true;
        }

        if ((a.goal.row == b.goal.row || a.goal.col == b.goal.col) && goalDistance <= 18) {
            return clearLineBetweenGoals(a.goal, b.goal);
        }

        if (analyzer.distance(a.goal, b.goal) < LevelAnalyzer.INF && goalDistance <= 14) {
            return true;
        }

        return false;
    }

    private static boolean clearLineBetweenGoals(Position a, Position b) {
        if (a.row == b.row) {
            int minCol = Math.min(a.col, b.col);
            int maxCol = Math.max(a.col, b.col);
            for (int col = minCol; col <= maxCol; col++) {
                if (State.walls[a.row][col]) {
                    return false;
                }
            }
            return true;
        }

        if (a.col == b.col) {
            int minRow = Math.min(a.row, b.row);
            int maxRow = Math.max(a.row, b.row);
            for (int row = minRow; row <= maxRow; row++) {
                if (State.walls[row][a.col]) {
                    return false;
                }
            }
            return true;
        }

        return false;
    }

    private static int candidateBoxCountForComponent(State state, ArrayList<UnsolvedBoxGoal> component) {
        HashSet<Character> letters = new HashSet<>();
        for (UnsolvedBoxGoal target : component) {
            letters.add(target.letter);
        }

        int count = 0;
        for (int r = 0; r < state.boxes.length; r++) {
            for (int c = 0; c < state.boxes[r].length; c++) {
                if (letters.contains(state.boxes[r][c])) {
                    count++;
                }
            }
        }
        return count;
    }

    private static boolean boundaryComponent(ArrayList<UnsolvedBoxGoal> component, LevelAnalyzer analyzer) {
        for (UnsolvedBoxGoal target : component) {
            if (boundaryOrCornerPressure(analyzer, target.goal)) {
                return true;
            }
        }
        return false;
    }

    private static int wrongGoalBlockerCount(State state, ArrayList<UnsolvedBoxGoal> component) {
        int count = 0;
        for (UnsolvedBoxGoal target : component) {
            if (wrongBoxPressure(state, target)) {
                count++;
            }
        }
        return count;
    }

    private static int componentBoundingArea(ArrayList<UnsolvedBoxGoal> component) {
        int minRow = Integer.MAX_VALUE;
        int maxRow = Integer.MIN_VALUE;
        int minCol = Integer.MAX_VALUE;
        int maxCol = Integer.MIN_VALUE;

        for (UnsolvedBoxGoal target : component) {
            minRow = Math.min(minRow, target.goal.row);
            maxRow = Math.max(maxRow, target.goal.row);
            minCol = Math.min(minCol, target.goal.col);
            maxCol = Math.max(maxCol, target.goal.col);
        }

        return (maxRow - minRow + 1) * (maxCol - minCol + 1);
    }

    private static String describeTargets(ArrayList<UnsolvedBoxGoal> targets) {
        ArrayList<String> parts = new ArrayList<>();
        for (UnsolvedBoxGoal target : targets) {
            parts.add(target.letter + "@" + target.goal);
        }
        return parts.toString();
    }

    private static boolean wrongBoxPressure(State state, UnsolvedBoxGoal target) {
        char box = state.boxes[target.goal.row][target.goal.col];
        return 'A' <= box && box <= 'Z' && box != target.letter;
    }

    private static boolean boundaryOrCornerPressure(LevelAnalyzer analyzer, Position goal) {
        if (goal.row <= 1 || goal.row >= analyzer.rows - 2 || goal.col <= 1 || goal.col >= analyzer.cols - 2) {
            return true;
        }
        return goalConstraintScore(null, analyzer, goal) >= 2;
    }

    private static int goalConstraintScore(State state, LevelAnalyzer analyzer, Position goal) {
        int score = 0;
        for (Position cell : adjacentCells(goal)) {
            if (!analyzer.inBounds(cell) || State.walls[cell.row][cell.col]) {
                score += 2;
            } else if (state != null && state.boxes[cell.row][cell.col] != 0) {
                score++;
            } else if (state != null && agentAt(state, cell.row, cell.col) != -1) {
                score++;
            }
        }
        return score;
    }

    private static ArrayList<Position> smallEndgameBlockers(State state, UnsolvedBoxGoal target) {
        ArrayList<Position> blockers = new ArrayList<>();

        if (wrongBoxPressure(state, target)) {
            blockers.add(target.goal);
        }

        for (Position cell : adjacentCells(target.goal)) {
            if (!inBounds(cell.row, cell.col)) {
                continue;
            }

            char box = state.boxes[cell.row][cell.col];
            if ('A' <= box && box <= 'Z'
                    && State.goals[cell.row][cell.col] != box
                    && !blockers.contains(cell)) {
                blockers.add(cell);
            }
        }

        blockers.sort(Comparator.comparingInt(p -> p.manhattanDistance(target.goal)));
        return blockers;
    }

    private static ArrayList<Position> adjacentCells(Position position) {
        ArrayList<Position> cells = new ArrayList<>(4);
        cells.add(new Position(position.row - 1, position.col));
        cells.add(new Position(position.row + 1, position.col));
        cells.add(new Position(position.row, position.col - 1));
        cells.add(new Position(position.row, position.col + 1));
        return cells;
    }

    private static PocketResult tryNearbySolvedBoxUnblockRepair(
            State start,
            ArrayList<Action[]> basePlan,
            LevelAnalyzer analyzer,
            SingleAgentPlanner planner,
            long deadline
    ) {
        UnsolvedBoxGoal target = findOnlyUnsolvedBoxGoal(start);

        if (target == null) {
            return null;
        }

        ArrayList<Position> blockers = nearbySolvedBoxGoals(start, target.goal, 6);

        for (Position blockerGoal : blockers) {
            if (System.nanoTime() >= deadline) {
                break;
            }

            char blocker = start.boxes[blockerGoal.row][blockerGoal.col];

            if (!('A' <= blocker && blocker <= 'Z')) {
                continue;
            }

            ArrayList<Integer> blockerAgents = agentsWithBoxColorByManhattan(start, blocker, blockerGoal);

            for (int blockerAgent : blockerAgents) {
                for (Position stage : blockerClearDestinations(start, blockerGoal, blocker)) {
                    if (System.nanoTime() >= deadline) {
                        return null;
                    }

                    if (stage.equals(blockerGoal) || stage.manhattanDistance(target.goal) <= 1) {
                        continue;
                    }

                    State current = copyState(start);
                    ArrayList<Action[]> plan = copyPlan(basePlan);

                    Task stageTask = new Task(blocker, blockerGoal, stage, blockerAgent, 0);
                    List<Action> stagePlan = planner.planBoxToGoalRelaxed(current, stageTask, new ReservationTable());

                    if (stagePlan == null || stagePlan.isEmpty()
                            || plan.size() + stagePlan.size() > MAX_TOTAL_ACTIONS) {
                        continue;
                    }

                    appendAsJointActions(plan, stagePlan, blockerAgent, current.agentRows.length);
                    current = simulateSingleAgentPlan(current, stagePlan, blockerAgent);

                    long targetDeadline = Math.min(deadline, System.nanoTime() + 8L * 1_000_000_000L);
                    Action[][] targetPlan = focusedSingleGoalRepair(
                            current,
                            analyzer,
                            target,
                            targetDeadline,
                            800_000
                    );

                    if (targetPlan == null || targetPlan.length == 0
                            || plan.size() + targetPlan.length > MAX_TOTAL_ACTIONS) {
                        continue;
                    }

                    appendJointPlan(plan, targetPlan);
                    current = simulateJointPlan(current, targetPlan);

                    Task restoreTask = new Task(blocker, stage, blockerGoal, blockerAgent, 0);
                    List<Action> restorePlan = planner.planBoxToGoalRelaxed(current, restoreTask, new ReservationTable());

                    if (restorePlan == null || restorePlan.isEmpty()
                            || plan.size() + restorePlan.size() > MAX_TOTAL_ACTIONS) {
                        continue;
                    }

                    appendAsJointActions(plan, restorePlan, blockerAgent, current.agentRows.length);
                    current = simulateSingleAgentPlan(current, restorePlan, blockerAgent);

                    System.err.format(
                            "Solved-box unblock moved %c via %s, placed %c at %s, restored %c at %s.%n",
                            blocker,
                            stage,
                            target.letter,
                            target.goal,
                            blocker,
                            blockerGoal
                    );

                    return new PocketResult(current, plan);
                }
            }
        }

        return null;
    }

    private static PocketResult tryAdjacentGoalPairUnblockRepair(
            State start,
            ArrayList<Action[]> basePlan,
            LevelAnalyzer analyzer,
            SingleAgentPlanner planner,
            long deadline
    ) {
        UnsolvedBoxGoal target = findOnlyUnsolvedBoxGoal(start);

        if (target == null) {
            return null;
        }

        for (Position blockerGoal : adjacentSolvedBoxGoals(start, target.goal)) {
            if (System.nanoTime() >= deadline) {
                return null;
            }

            char blocker = start.boxes[blockerGoal.row][blockerGoal.col];

            if (State.boxColors[blocker - 'A'] != State.boxColors[target.letter - 'A']) {
                continue;
            }

            ArrayList<Integer> blockerAgents = agentsWithBoxColorByManhattan(start, blocker, blockerGoal);
            ArrayList<Position> stages = blockerClearDestinations(start, blockerGoal, blocker);
            int triedStages = 0;

            for (int blockerAgent : blockerAgents) {
                for (Position stage : stages) {
                    if (System.nanoTime() >= deadline) {
                        return null;
                    }

                    if (stage.equals(blockerGoal)
                            || stage.equals(target.goal)
                            || stage.manhattanDistance(blockerGoal) > 5
                            || ++triedStages > 16) {
                        continue;
                    }

                    PocketResult result = tryStageTargetRestore(
                            start,
                            basePlan,
                            analyzer,
                            planner,
                            deadline,
                            target,
                            blocker,
                            blockerGoal,
                            blockerAgent,
                            stage,
                            0
                    );

                    if (result != null) {
                        return result;
                    }
                }
            }
        }

        return null;
    }

    private static PocketResult tryStageTargetRestore(
            State start,
            ArrayList<Action[]> basePlan,
            LevelAnalyzer analyzer,
            SingleAgentPlanner planner,
            long deadline,
            UnsolvedBoxGoal target,
            char blocker,
            Position blockerGoal,
            int blockerAgent,
            Position stage,
            int targetExpansionLimit
    ) {
        State current = copyState(start);
        ArrayList<Action[]> plan = copyPlan(basePlan);

        Task stageTask = new Task(blocker, blockerGoal, stage, blockerAgent, 0);
        List<Action> stagePlan = planner.planBoxToGoalRelaxed(current, stageTask, new ReservationTable());

        if (stagePlan == null || stagePlan.isEmpty()
                || plan.size() + stagePlan.size() > MAX_TOTAL_ACTIONS) {
            return null;
        }

        appendAsJointActions(plan, stagePlan, blockerAgent, current.agentRows.length);
        current = simulateSingleAgentPlan(current, stagePlan, blockerAgent);

        Action[][] targetPlan = null;
        Task targetTask = bestSameSideTaskForGoal(
                current,
                analyzer,
                target.letter,
                target.goal,
                dominantBoundary(target.goal)
        );

        if (targetTask != null) {
            List<Action> directTargetPlan = planner.planBoxToGoal(current, targetTask, new ReservationTable());

            if (directTargetPlan == null || directTargetPlan.isEmpty()) {
                directTargetPlan = planner.planBoxToGoalRelaxed(current, targetTask, new ReservationTable());
            }

            if (directTargetPlan != null && !directTargetPlan.isEmpty()) {
                ArrayList<Action[]> directJointPlan = new ArrayList<>();
                appendAsJointActions(directJointPlan, directTargetPlan, targetTask.assignedAgent, current.agentRows.length);
                targetPlan = directJointPlan.toArray(new Action[0][]);
            }
        }

        if ((targetPlan == null || targetPlan.length == 0) && targetExpansionLimit > 0) {
            long targetDeadline = Math.min(deadline, System.nanoTime() + 10L * 1_000_000_000L);
            targetPlan = focusedSingleGoalRepair(
                    current,
                    analyzer,
                    target,
                    targetDeadline,
                    targetExpansionLimit
            );
        }

        if (targetPlan == null || targetPlan.length == 0
                || plan.size() + targetPlan.length > MAX_TOTAL_ACTIONS) {
            return null;
        }

        appendJointPlan(plan, targetPlan);
        current = simulateJointPlan(current, targetPlan);

        Task restoreTask = new Task(blocker, stage, blockerGoal, blockerAgent, 0);
        List<Action> restorePlan = planner.planBoxToGoalRelaxed(current, restoreTask, new ReservationTable());

        if (restorePlan == null || restorePlan.isEmpty()
                || plan.size() + restorePlan.size() > MAX_TOTAL_ACTIONS) {
            return null;
        }

        appendAsJointActions(plan, restorePlan, blockerAgent, current.agentRows.length);
        current = simulateSingleAgentPlan(current, restorePlan, blockerAgent);

        System.err.format(
                "Adjacent pair unblock moved %c via %s, placed %c at %s, restored %c at %s.%n",
                blocker,
                stage,
                target.letter,
                target.goal,
                blocker,
                blockerGoal
        );

        return new PocketResult(current, plan);
    }

    private static boolean appendValidatedSingleAgentStep(
            ArrayList<Action[]> plan,
            State state,
            int agent,
            Action action
    ) {
        if (!isApplicable(state, agent, action) || plan.size() + 1 > MAX_TOTAL_ACTIONS) {
            return false;
        }

        Action[] jointAction = noopJointAction(state.agentRows.length);
        jointAction[agent] = action;

        if (isConflicting(state, jointAction)) {
            return false;
        }

        plan.add(jointAction);
        return true;
    }

    private static ArrayList<Position> adjacentSolvedBoxGoals(State state, Position target) {
        ArrayList<Position> result = new ArrayList<>();

        for (Position cell : accessCells(target)) {
            char box = state.boxes[cell.row][cell.col];

            if ('A' <= box && box <= 'Z' && State.goals[cell.row][cell.col] == box) {
                result.add(cell);
            }
        }

        result.sort(Comparator.comparingInt(p -> p.manhattanDistance(target)));
        return result;
    }

    private static ArrayList<Position> nearbySolvedBoxGoals(State state, Position target, int maxDistance) {
        ArrayList<Position> result = new ArrayList<>();

        for (int r = 0; r < State.goals.length; r++) {
            for (int c = 0; c < State.goals[r].length; c++) {
                char goal = State.goals[r][c];

                if ('A' <= goal
                        && goal <= 'Z'
                        && state.boxes[r][c] == goal
                        && new Position(r, c).manhattanDistance(target) <= maxDistance) {
                    result.add(new Position(r, c));
                }
            }
        }

        result.sort(Comparator.comparingInt(p -> p.manhattanDistance(target)));
        return result;
    }

    private static Action[][] preSealAgentEvacuation(
            State start,
            LevelAnalyzer analyzer,
            AgentGoalPlanner agentGoalPlanner,
            long deadline
    ) {
        UnsolvedBoxGoal finalBox = findOnlyUnsolvedBoxGoal(start);

        if (finalBox == null) {
            return null;
        }

        State current = copyState(start);
        ArrayList<Action[]> plan = new ArrayList<>();
        boolean progress = true;

        while (progress && System.nanoTime() < deadline) {
            progress = false;
            ArrayList<Integer> agents = unsolvedAgentsByGraphDistance(current, analyzer);

            for (int agent : agents) {
                Position goal = findAgentGoal(agent);

                if (goal == null || goal.manhattanDistance(finalBox.goal) <= 2) {
                    continue;
                }

                int distance = agentGraphGoalDistance(current, analyzer, agent);

                if (distance < 12) {
                    continue;
                }

                List<Action> route = agentGoalPlanner.planAgentToGoal(current, agent);

                if (route == null || route.isEmpty() || plan.size() + route.size() > 160) {
                    continue;
                }

                appendAsJointActions(plan, route, agent, current.agentRows.length);
                current = simulateSingleAgentPlan(current, route, agent);
                progress = true;

                System.err.format(
                        "Pre-seal evacuation moved agent %,d using %,d actions.%n",
                        agent,
                        route.size()
                );
                break;
            }
        }

        return plan.isEmpty() ? null : plan.toArray(new Action[0][]);
    }

    private static Action[][] preBoundaryLongAgentEvacuation(
            State start,
            LevelAnalyzer analyzer,
            AgentGoalPlanner agentGoalPlanner,
            long deadline
    ) {
        if (countUnsolvedBoxGoals(start) > 3 || countUnsolvedAgentGoals(start) == 0) {
            return null;
        }

        State current = copyState(start);
        ArrayList<Action[]> plan = new ArrayList<>();
        ArrayList<Integer> agents = unsolvedAgentsByGraphDistance(current, analyzer);

        for (int agent : agents) {
            if (System.nanoTime() >= deadline) {
                break;
            }

            Position goal = findAgentGoal(agent);

            if (goal == null || agentGraphGoalDistance(current, analyzer, agent) < 20) {
                continue;
            }

            List<Action> route = agentGoalPlanner.planAgentToGoal(current, agent);

            if (route == null || route.isEmpty() || plan.size() + route.size() > 80) {
                continue;
            }

            appendAsJointActions(plan, route, agent, current.agentRows.length);
            current = simulateSingleAgentPlan(current, route, agent);

            System.err.format(
                    "Pre-boundary evacuation moved agent %,d using %,d actions.%n",
                    agent,
                    route.size()
            );
            break;
        }

        return plan.isEmpty() ? null : plan.toArray(new Action[0][]);
    }

    private static ArrayList<Integer> unsolvedAgentsByGraphDistance(State state, LevelAnalyzer analyzer) {
        ArrayList<Integer> agents = new ArrayList<>();

        for (int agent = 0; agent < state.agentRows.length; agent++) {
            Position goal = findAgentGoal(agent);

            if (goal != null && !(state.agentRows[agent] == goal.row && state.agentCols[agent] == goal.col)) {
                agents.add(agent);
            }
        }

        agents.sort(Comparator.comparingInt((Integer agent) -> -agentGraphGoalDistance(state, analyzer, agent)));
        return agents;
    }

    private static PocketResult clearWrongGoalBlocker(
            State state,
            ArrayList<Action[]> currentPlan,
            Task blockedTask,
            SingleAgentPlanner planner,
            long deadline
    ) {
        Position goal = blockedTask.goal;
        char blocker = state.boxes[goal.row][goal.col];

        if (!('A' <= blocker && blocker <= 'Z') || blocker == blockedTask.boxLetter) {
            return null;
        }

        ArrayList<Integer> blockerAgents = agentsWithBoxColorByManhattan(state, blocker, goal);

        if (blockerAgents.isEmpty()) {
            return null;
        }

        for (int blockerAgent : blockerAgents) {
            for (Position destination : blockerClearDestinations(state, goal, blocker)) {
                if (System.nanoTime() > deadline) {
                    return null;
                }

                Task clearTask = new Task(blocker, goal, destination, blockerAgent, 0);
                List<Action> clearPlan = planner.planBoxToGoal(state, clearTask, new ReservationTable());

                if (clearPlan == null || clearPlan.isEmpty()) {
                    clearPlan = planner.planBoxToGoalRelaxed(state, clearTask, new ReservationTable());
                }

                if (clearPlan == null || clearPlan.isEmpty()
                        || currentPlan.size() + clearPlan.size() > MAX_TOTAL_ACTIONS) {
                    continue;
                }

                ArrayList<Action[]> updatedPlan = copyPlan(currentPlan);
                appendAsJointActions(updatedPlan, clearPlan, blockerAgent, state.agentRows.length);
                State updatedState = simulateSingleAgentPlan(state, clearPlan, blockerAgent);

                System.err.format(
                        "Cleared wrong-goal blocker %c from %s to %s with agent %,d using %,d actions.%n",
                        blocker,
                        goal,
                        destination,
                        blockerAgent,
                        clearPlan.size()
                );

                return new PocketResult(updatedState, updatedPlan);
            }
        }

        return null;
    }

    private static ArrayList<Integer> agentsWithBoxColorByManhattan(State state, char box, Position target) {
        ArrayList<Integer> agents = new ArrayList<>();
        Color color = State.boxColors[box - 'A'];

        for (int agent = 0; agent < state.agentRows.length; agent++) {
            if (State.agentColors[agent] == color) {
                agents.add(agent);
            }
        }

        agents.sort(Comparator.comparingInt(agent ->
                Math.abs(state.agentRows[agent] - target.row) + Math.abs(state.agentCols[agent] - target.col)));
        return agents;
    }

    private static ArrayList<Position> blockerClearDestinations(State state, Position blockedGoal, char blocker) {
        ArrayList<Position> destinations = new ArrayList<>();

        for (int r = 0; r < State.goals.length; r++) {
            for (int c = 0; c < State.goals[r].length; c++) {
                if (State.goals[r][c] == blocker && state.boxes[r][c] != blocker && cellIsFree(state, r, c)) {
                    destinations.add(new Position(r, c));
                }
            }
        }

        destinations.sort(Comparator.comparingInt(p -> p.manhattanDistance(blockedGoal)));

        int maxRadius = Math.max(State.walls.length, State.walls[0].length);

        for (int radius = 1; radius <= Math.min(10, maxRadius); radius++) {
            for (int row = blockedGoal.row - radius; row <= blockedGoal.row + radius; row++) {
                for (int col = blockedGoal.col - radius; col <= blockedGoal.col + radius; col++) {
                    if (Math.abs(row - blockedGoal.row) + Math.abs(col - blockedGoal.col) != radius) {
                        continue;
                    }

                    if (inBounds(row, col)
                            && State.goals[row][col] == 0
                            && cellIsFree(state, row, col)) {
                        Position stage = new Position(row, col);

                        if (!destinations.contains(stage)) {
                            destinations.add(stage);
                        }
                    }
                }
            }
        }

        return destinations;
    }

    private static Action[][] mixedGoalRoomRepair(
            State start,
            LevelAnalyzer analyzer,
            long deadline,
            int maxExpansions
    ) {
        UnsolvedBoxGoal target = findOnlyUnsolvedBoxGoal(start);

        if (target == null) {
            return null;
        }

        RepairRegion region = buildGoalRoomRegion(target.goal, start);
        int[] activeAgents = selectGoalRoomAgents(start, region, target);

        if (activeAgents.length == 0) {
            return null;
        }

        System.err.format(
                "Goal-room repair region rows %,d..%,d cols %,d..%,d agents %s.%n",
                region.minRow,
                region.maxRow,
                region.minCol,
                region.maxCol,
                java.util.Arrays.toString(activeAgents)
        );

        PriorityQueue<FocusedNode> open = new PriorityQueue<>(Comparator.comparingInt(node -> node.f));
        HashMap<State, Integer> bestG = new HashMap<>();
        int activeBoxDistanceLimit = closestBoxDistance(analyzer, start, target) + 6;

        open.add(new FocusedNode(
                start,
                null,
                null,
                0,
                goalRoomRepairHeuristic(analyzer, start, target, region)
        ));

        int expansions = 0;

        while (!open.isEmpty() && System.nanoTime() < deadline) {
            if (++expansions > maxExpansions) {
                System.err.format("Mixed-color goal-room repair hit expansion limit. Open size: %,d.%n", open.size());
                return null;
            }

            FocusedNode node = open.poll();
            Integer seen = bestG.get(node.state);

            if (seen != null && seen <= node.g) {
                continue;
            }

            bestG.put(node.state, node.g);

            if (allBoxGoalsSolved(node.state)) {
                System.err.format("Mixed-color goal-room repair expanded %,d states.%n", expansions);
                return extractFocusedPlan(node, start.agentRows.length);
            }

            for (FocusedStep step : expandGoalRoomRepair(
                    node.state,
                    activeAgents,
                    region,
                    target,
                    activeBoxDistanceLimit,
                    analyzer
            )) {
                int nextG = node.g + 1;
                Integer bestKnown = bestG.get(step.state);

                if (bestKnown != null && bestKnown <= nextG) {
                    continue;
                }

                int nextH = goalRoomRepairHeuristic(analyzer, step.state, target, region);
                open.add(new FocusedNode(
                        step.state,
                        node,
                        step.jointAction,
                        nextG,
                        nextG + 12 * nextH
                ));
            }
        }

        return null;
    }

    private static Action[][] moveOnlyAgentGoalRepair(
            State start,
            LevelAnalyzer analyzer,
            long deadline,
            int maxExpansions
    ) {
        PriorityQueue<FocusedNode> open = new PriorityQueue<>(Comparator.comparingInt(node -> node.f));
        HashMap<State, Integer> bestG = new HashMap<>();

        open.add(new FocusedNode(
                start,
                null,
                null,
                0,
                moveOnlyAgentGoalHeuristic(start, analyzer)
        ));

        int expansions = 0;

        while (!open.isEmpty() && System.nanoTime() < deadline) {
            if (++expansions > maxExpansions) {
                System.err.format("Move-only agent repair hit expansion limit. Open size: %,d.%n", open.size());
                return null;
            }

            FocusedNode node = open.poll();
            Integer seen = bestG.get(node.state);

            if (seen != null && seen <= node.g) {
                continue;
            }

            bestG.put(node.state, node.g);

            if (node.state.isGoalState()) {
                System.err.format("Move-only agent repair expanded %,d states.%n", expansions);
                return extractFocusedPlan(node, start.agentRows.length);
            }

            for (FocusedStep step : expandMoveOnlyAgentGoals(node.state, analyzer)) {
                int nextG = node.g + 1;
                Integer bestKnown = bestG.get(step.state);

                if (bestKnown != null && bestKnown <= nextG) {
                    continue;
                }

                int nextH = moveOnlyAgentGoalHeuristic(step.state, analyzer);
                open.add(new FocusedNode(
                        step.state,
                        node,
                        step.jointAction,
                        nextG,
                        nextG + 8 * nextH
                ));
            }
        }

        return null;
    }

    private static Action[][] agentGoalPushRepair(
            State start,
            int activeAgent,
            long deadline,
            int maxExpansions
    ) {
        Position goal = findAgentGoal(activeAgent);

        if (goal == null || (start.agentRows[activeAgent] == goal.row && start.agentCols[activeAgent] == goal.col)) {
            return null;
        }

        PriorityQueue<FocusedNode> open = new PriorityQueue<>(Comparator.comparingInt(node -> node.f));
        HashMap<State, Integer> bestG = new HashMap<>();

        open.add(new FocusedNode(
                start,
                null,
                null,
                0,
                agentGoalPushHeuristic(start, activeAgent, goal)
        ));

        int expansions = 0;

        while (!open.isEmpty() && System.nanoTime() < deadline) {
            if (++expansions > maxExpansions) {
                return null;
            }

            FocusedNode node = open.poll();
            Integer seen = bestG.get(node.state);

            if (seen != null && seen <= node.g) {
                continue;
            }

            bestG.put(node.state, node.g);

            if (node.state.agentRows[activeAgent] == goal.row
                    && node.state.agentCols[activeAgent] == goal.col
                    && allBoxGoalsSolved(node.state)) {
                System.err.format(
                        "Agent push repair for agent %,d expanded %,d states.%n",
                        activeAgent,
                        expansions
                );
                return extractFocusedPlan(node, start.agentRows.length);
            }

            for (Action action : Action.values()) {
                if (action.type == ActionType.NoOp || !isApplicable(node.state, activeAgent, action)) {
                    continue;
                }

                if (movesSolvedBoxGoal(node.state, activeAgent, action)) {
                    continue;
                }

                Action[] jointAction = noopJointAction(node.state.agentRows.length);
                jointAction[activeAgent] = action;

                if (isConflicting(node.state, jointAction)) {
                    continue;
                }

                State next = applyJointAction(node.state, jointAction);
                int nextG = node.g + 1;
                Integer bestKnown = bestG.get(next);

                if (bestKnown != null && bestKnown <= nextG) {
                    continue;
                }

                int h = agentGoalPushHeuristic(next, activeAgent, goal);
                open.add(new FocusedNode(next, node, jointAction, nextG, nextG + 6 * h));
            }
        }

        return null;
    }

    private static int agentGoalPushHeuristic(State state, int agent, Position goal) {
        int distance = Math.abs(state.agentRows[agent] - goal.row)
                + Math.abs(state.agentCols[agent] - goal.col);
        return 10 * distance + 1000 * countUnsolvedBoxGoals(state);
    }

    private static boolean movesSolvedBoxGoal(State state, int agent, Action action) {
        int agentRow = state.agentRows[agent];
        int agentCol = state.agentCols[agent];
        int boxRow;
        int boxCol;

        if (action.type == ActionType.Push) {
            boxRow = agentRow + action.agentRowDelta;
            boxCol = agentCol + action.agentColDelta;
        } else if (action.type == ActionType.Pull) {
            boxRow = agentRow - action.boxRowDelta;
            boxCol = agentCol - action.boxColDelta;
        } else {
            return false;
        }

        if (!inBounds(boxRow, boxCol)) {
            return false;
        }

        char box = state.boxes[boxRow][boxCol];
        return 'A' <= box && box <= 'Z' && State.goals[boxRow][boxCol] == box;
    }

    private static Action[][] agentSwapRepair(State start, LevelAnalyzer analyzer, long deadline) {
        for (int a = 0; a < start.agentRows.length; a++) {
            Position goalA = findAgentGoal(a);

            if (goalA == null || (start.agentRows[a] == goalA.row && start.agentCols[a] == goalA.col)) {
                continue;
            }

            for (int b = a + 1; b < start.agentRows.length; b++) {
                Position goalB = findAgentGoal(b);

                if (goalB == null || (start.agentRows[b] == goalB.row && start.agentCols[b] == goalB.col)) {
                    continue;
                }

                if (start.agentRows[a] == goalB.row
                        && start.agentCols[a] == goalB.col
                        && start.agentRows[b] == goalA.row
                        && start.agentCols[b] == goalA.col) {
                    Action[][] repaired = tryAgentSwap(start, analyzer, deadline, a, b, goalA, goalB);

                    if (repaired != null) {
                        return repaired;
                    }
                }
            }
        }

        return null;
    }

    private static Action[][] agentGoalOccupantRepair(State start, LevelAnalyzer analyzer, long deadline) {
        for (int owner = 0; owner < start.agentRows.length; owner++) {
            Position ownerGoal = findAgentGoal(owner);

            if (ownerGoal == null || (start.agentRows[owner] == ownerGoal.row && start.agentCols[owner] == ownerGoal.col)) {
                continue;
            }

            int occupant = agentAt(start, ownerGoal.row, ownerGoal.col);

            if (occupant == -1 || occupant == owner) {
                continue;
            }

            Position occupantGoal = findAgentGoal(occupant);

            if (occupantGoal == null) {
                continue;
            }

            for (Position temp : swapTempCells(start, analyzer, ownerGoal, occupantGoal)) {
                if (System.nanoTime() > deadline) {
                    return null;
                }

                if (temp.equals(ownerGoal) || temp.equals(occupantGoal)) {
                    continue;
                }

                State current = copyState(start);
                ArrayList<Action[]> plan = new ArrayList<>();

                List<Action> moveOccupantOut = planAgentToPosition(current, occupant, temp, 50_000);

                if (moveOccupantOut == null || moveOccupantOut.isEmpty()) {
                    continue;
                }

                appendAsJointActions(plan, moveOccupantOut, occupant, current.agentRows.length);
                current = simulateSingleAgentPlan(current, moveOccupantOut, occupant);

                List<Action> moveOwnerHome = planAgentToPosition(current, owner, ownerGoal, 50_000);

                if (moveOwnerHome == null || moveOwnerHome.isEmpty()) {
                    continue;
                }

                appendAsJointActions(plan, moveOwnerHome, owner, current.agentRows.length);
                current = simulateSingleAgentPlan(current, moveOwnerHome, owner);

                List<Action> moveOccupantHome = planAgentToPosition(current, occupant, occupantGoal, 50_000);

                if (moveOccupantHome != null && !moveOccupantHome.isEmpty()) {
                    appendAsJointActions(plan, moveOccupantHome, occupant, current.agentRows.length);
                    current = simulateSingleAgentPlan(current, moveOccupantHome, occupant);
                }

                if (current.agentRows[owner] == ownerGoal.row
                        && current.agentCols[owner] == ownerGoal.col
                        && !(current.agentRows[occupant] == ownerGoal.row && current.agentCols[occupant] == ownerGoal.col)
                        && countSolvedGoals(current) >= countSolvedGoals(start)) {
                    System.err.format(
                            "Agent displacement moved occupant %,d from agent %,d goal via %s using %,d actions.%n",
                            occupant,
                            owner,
                            temp,
                            plan.size()
                    );
                    return plan.toArray(new Action[0][]);
                }
            }
        }

        return null;
    }

    private static Action[][] tryAgentSwap(
            State start,
            LevelAnalyzer analyzer,
            long deadline,
            int a,
            int b,
            Position goalA,
            Position goalB
    ) {
        for (Position temp : swapTempCells(start, analyzer, goalA, goalB)) {
            if (System.nanoTime() > deadline) {
                return null;
            }

            if ((temp.row == goalA.row && temp.col == goalA.col)
                    || (temp.row == goalB.row && temp.col == goalB.col)) {
                continue;
            }

            ArrayList<Action[]> plan = new ArrayList<>();
            State current = copyState(start);

            List<Action> moveAOut = planAgentToPosition(current, a, temp, 50_000);

            if (moveAOut == null || moveAOut.isEmpty()) {
                continue;
            }

            appendAsJointActions(plan, moveAOut, a, current.agentRows.length);
            current = simulateSingleAgentPlan(current, moveAOut, a);

            List<Action> moveBHome = planAgentToPosition(current, b, goalB, 50_000);

            if (moveBHome == null || moveBHome.isEmpty()) {
                continue;
            }

            appendAsJointActions(plan, moveBHome, b, current.agentRows.length);
            current = simulateSingleAgentPlan(current, moveBHome, b);

            List<Action> moveAHome = planAgentToPosition(current, a, goalA, 50_000);

            if (moveAHome == null || moveAHome.isEmpty()) {
                continue;
            }

            appendAsJointActions(plan, moveAHome, a, current.agentRows.length);

            System.err.format(
                    "Agent swap repair routed agents %,d and %,d via %s using %,d actions.%n",
                    a,
                    b,
                    temp,
                    plan.size()
            );

            return plan.toArray(new Action[0][]);
        }

        return null;
    }

    private static ArrayList<Position> swapTempCells(State state, LevelAnalyzer analyzer, Position goalA, Position goalB) {
        ArrayList<Position> cells = new ArrayList<>();

        for (int r = 0; r < State.walls.length; r++) {
            for (int c = 0; c < State.walls[r].length; c++) {
                if (!cellIsFree(state, r, c) || State.goals[r][c] != 0) {
                    continue;
                }

                Position cell = new Position(r, c);

                if (analyzer.distance(cell, goalA) < LevelAnalyzer.INF
                        && analyzer.distance(cell, goalB) < LevelAnalyzer.INF) {
                    cells.add(cell);
                }
            }
        }

        cells.sort(Comparator
                .comparingInt((Position p) -> Math.min(
                        analyzer.distance(p, goalA),
                        analyzer.distance(p, goalB)
                ))
                .thenComparingInt(p -> p.manhattanDistance(goalA) + p.manhattanDistance(goalB)));
        return cells;
    }

    private static List<Action> planAgentToPosition(State state, int agent, Position target, int maxExpansions) {
        PriorityQueue<AgentMoveNode> open = new PriorityQueue<>(Comparator.comparingInt(node -> node.f));
        HashMap<Position, Integer> bestG = new HashMap<>();
        Position start = new Position(state.agentRows[agent], state.agentCols[agent]);

        open.add(new AgentMoveNode(start, null, null, 0, start.manhattanDistance(target)));

        int expansions = 0;

        while (!open.isEmpty()) {
            if (++expansions > maxExpansions) {
                return null;
            }

            AgentMoveNode node = open.poll();
            Integer seen = bestG.get(node.pos);

            if (seen != null && seen <= node.g) {
                continue;
            }

            bestG.put(node.pos, node.g);

            if (node.pos.equals(target)) {
                return extractAgentMovePlan(node);
            }

            for (Action action : MOVE_ACTIONS) {
                Position next = node.pos.translate(action.agentRowDelta, action.agentColDelta);

                if (!cellFreeForSingleAgentRoute(state, agent, next, target)) {
                    continue;
                }

                int g = node.g + 1;
                Integer best = bestG.get(next);

                if (best != null && best <= g) {
                    continue;
                }

                open.add(new AgentMoveNode(next, node, action, g, g + next.manhattanDistance(target)));
            }
        }

        return null;
    }

    private static boolean cellFreeForSingleAgentRoute(State state, int activeAgent, Position position, Position target) {
        if (!inBounds(position.row, position.col) || State.walls[position.row][position.col]) {
            return false;
        }

        if (state.boxes[position.row][position.col] != 0) {
            return false;
        }

        for (int agent = 0; agent < state.agentRows.length; agent++) {
            if (agent == activeAgent) {
                continue;
            }

            if (state.agentRows[agent] == position.row && state.agentCols[agent] == position.col) {
                return position.equals(target);
            }
        }

        return true;
    }

    private static Action[][] prioritizedAgentGoalRepair(State start, long deadline, int maxExpansionsPerAgent) {
        State current = copyState(start);
        ArrayList<Action[]> plan = new ArrayList<>();
        boolean progress = true;

        while (progress && !current.isGoalState() && System.nanoTime() < deadline) {
            progress = false;
            ArrayList<Integer> agents = unsolvedAgentsByDistance(current);

            for (int agent : agents) {
                if (System.nanoTime() >= deadline) {
                    break;
                }

                List<Action> route = planAgentToGoalIgnoringAgents(current, agent, maxExpansionsPerAgent);

                if (route == null || route.isEmpty()) {
                    continue;
                }

                ArrayList<Action[]> suffix = new ArrayList<>();
                State routedState = executeAgentRouteWithEvacuations(current, agent, route, suffix, deadline);

                if (routedState == null) {
                    continue;
                }

                int before = countSolvedGoals(current);
                int after = countSolvedGoals(routedState);

                if (after >= before && !suffix.isEmpty()) {
                    appendJointPlan(plan, suffix.toArray(new Action[0][]));
                    current = routedState;
                    progress = true;

                    if (current.isGoalState()) {
                        return plan.toArray(new Action[0][]);
                    }
                }
            }
        }

        return current.isGoalState() ? plan.toArray(new Action[0][]) : null;
    }

    private static ArrayList<Integer> unsolvedAgentsByDistance(State state) {
        ArrayList<Integer> agents = new ArrayList<>();

        for (int agent = 0; agent < state.agentRows.length; agent++) {
            Position goal = findAgentGoal(agent);

            if (goal != null && !(state.agentRows[agent] == goal.row && state.agentCols[agent] == goal.col)) {
                agents.add(agent);
            }
        }

        agents.sort(Comparator.comparingInt(agent -> {
            Position goal = findAgentGoal(agent);
            return -Math.abs(state.agentRows[agent] - goal.row) - Math.abs(state.agentCols[agent] - goal.col);
        }));

        return agents;
    }

    private static List<Action> planAgentToGoalIgnoringAgents(State state, int agent, int maxExpansions) {
        Position goal = findAgentGoal(agent);

        if (goal == null) {
            return null;
        }

        PriorityQueue<AgentMoveNode> open = new PriorityQueue<>(Comparator.comparingInt(node -> node.f));
        java.util.HashSet<Position> closed = new java.util.HashSet<>();
        Position start = new Position(state.agentRows[agent], state.agentCols[agent]);

        open.add(new AgentMoveNode(start, null, null, 0, start.manhattanDistance(goal)));

        int expansions = 0;

        while (!open.isEmpty() && expansions++ < maxExpansions) {
            AgentMoveNode node = open.poll();

            if (!closed.add(node.pos)) {
                continue;
            }

            if (node.pos.equals(goal)) {
                return extractAgentMovePlan(node);
            }

            for (Action action : MOVE_ACTIONS) {
                Position next = node.pos.translate(action.agentRowDelta, action.agentColDelta);

                if (closed.contains(next) || !cellFreeForAgentRouteIgnoringAgents(state, next)) {
                    continue;
                }

                int g = node.g + 1;
                int h = next.manhattanDistance(goal);
                open.add(new AgentMoveNode(next, node, action, g, g + 5 * h));
            }
        }

        return null;
    }

    private static boolean cellFreeForAgentRouteIgnoringAgents(State state, Position pos) {
        return inBounds(pos.row, pos.col)
                && !State.walls[pos.row][pos.col]
                && state.boxes[pos.row][pos.col] == 0;
    }

    private static State executeAgentRouteWithEvacuations(
            State start,
            int movingAgent,
            List<Action> route,
            ArrayList<Action[]> suffix,
            long deadline
    ) {
        State current = copyState(start);

        for (Action action : route) {
            if (System.nanoTime() >= deadline) {
                return null;
            }

            Position destination = new Position(
                    current.agentRows[movingAgent] + action.agentRowDelta,
                    current.agentCols[movingAgent] + action.agentColDelta
            );
            int blocker = agentAt(current, destination.row, destination.col);

            if (blocker != -1 && blocker != movingAgent) {
                Action blockerMove = evacuationMove(current, blocker, movingAgent, destination);

                if (blockerMove == null) {
                    return null;
                }

                Action[] blockerJoint = noopJointAction(current.agentRows.length);
                blockerJoint[blocker] = blockerMove;

                if (isConflicting(current, blockerJoint)) {
                    return null;
                }

                current = applyJointAction(current, blockerJoint);
                suffix.add(blockerJoint);
            }

            if (!isApplicable(current, movingAgent, action)) {
                return null;
            }

            Action[] jointAction = noopJointAction(current.agentRows.length);
            jointAction[movingAgent] = action;

            if (isConflicting(current, jointAction)) {
                return null;
            }

            current = applyJointAction(current, jointAction);
            suffix.add(jointAction);
        }

        return current;
    }

    private static Action evacuationMove(State state, int blocker, int movingAgent, Position blockedCell) {
        Position moverGoal = findAgentGoal(movingAgent);
        Action bestAction = null;
        int bestScore = Integer.MAX_VALUE;

        for (Action action : MOVE_ACTIONS) {
            if (!isApplicable(state, blocker, action)) {
                continue;
            }

            Position next = new Position(
                    state.agentRows[blocker] + action.agentRowDelta,
                    state.agentCols[blocker] + action.agentColDelta
            );

            if (next.equals(blockedCell)) {
                continue;
            }

            int score = moverGoal == null ? 0 : next.manhattanDistance(moverGoal);
            Position blockerGoal = findAgentGoal(blocker);

            if (blockerGoal != null) {
                score += next.manhattanDistance(blockerGoal);
            }

            if (score < bestScore) {
                bestScore = score;
                bestAction = action;
            }
        }

        return bestAction;
    }

    private static ArrayList<FocusedStep> expandMoveOnlyAgentGoals(State state, LevelAnalyzer analyzer) {
        ArrayList<FocusedStep> result = new ArrayList<>();
        boolean allowSolvedHelpers = countUnsolvedAgentGoals(state) <= 2;

        for (int agent = 0; agent < state.agentRows.length; agent++) {
            Position goal = findAgentGoal(agent);

            if (goal == null) {
                continue;
            }

            if (!allowSolvedHelpers && state.agentRows[agent] == goal.row && state.agentCols[agent] == goal.col) {
                continue;
            }

            for (Action action : MOVE_ACTIONS) {
                if (!isApplicable(state, agent, action)) {
                    continue;
                }

                Action[] jointAction = noopJointAction(state.agentRows.length);
                jointAction[agent] = action;

                if (!isConflicting(state, jointAction)) {
                    result.add(new FocusedStep(applyJointAction(state, jointAction), jointAction));
                }
            }
        }

        result.sort(Comparator.comparingInt(step -> moveOnlyAgentGoalHeuristic(step.state, analyzer)));
        return result;
    }

    private static int countUnsolvedAgentGoals(State state) {
        int count = 0;

        for (int agent = 0; agent < state.agentRows.length; agent++) {
            Position goal = findAgentGoal(agent);

            if (goal != null && !(state.agentRows[agent] == goal.row && state.agentCols[agent] == goal.col)) {
                count++;
            }
        }

        return count;
    }

    private static int moveOnlyAgentGoalHeuristic(State state, LevelAnalyzer analyzer) {
        int h = 0;

        for (int agent = 0; agent < state.agentRows.length; agent++) {
            Position goal = findAgentGoal(agent);

            if (goal == null) {
                continue;
            }

            Position agentPos = new Position(state.agentRows[agent], state.agentCols[agent]);
            int distance = analyzer.distance(agentPos, goal);

            if (distance >= LevelAnalyzer.INF) {
                distance = Math.abs(state.agentRows[agent] - goal.row)
                        + Math.abs(state.agentCols[agent] - goal.col)
                        + 1000;
            }

            h += 10 * distance;

            char goalCell = State.goals[state.agentRows[agent]][state.agentCols[agent]];
            if ('0' <= goalCell && goalCell <= '9' && goalCell - '0' != agent) {
                h += 50;
            }
        }

        return h;
    }

    private static Position findAgentGoal(int agent) {
        char goal = (char) ('0' + agent);

        for (int r = 0; r < State.goals.length; r++) {
            for (int c = 0; c < State.goals[r].length; c++) {
                if (State.goals[r][c] == goal) {
                    return new Position(r, c);
                }
            }
        }

        return null;
    }

    private static UnsolvedBoxGoal findOnlyUnsolvedBoxGoal(State state) {
        UnsolvedBoxGoal result = null;

        for (int r = 0; r < State.goals.length; r++) {
            for (int c = 0; c < State.goals[r].length; c++) {
                char goal = State.goals[r][c];

                if ('A' <= goal && goal <= 'Z' && state.boxes[r][c] != goal) {
                    if (result != null) {
                        return null;
                    }

                    result = new UnsolvedBoxGoal(goal, new Position(r, c));
                }
            }
        }

        return result;
    }

    private static ArrayList<UnsolvedBoxGoal> remainingBoxGoals(State state) {
        ArrayList<UnsolvedBoxGoal> result = new ArrayList<>();

        for (int r = 0; r < State.goals.length; r++) {
            for (int c = 0; c < State.goals[r].length; c++) {
                char goal = State.goals[r][c];

                if ('A' <= goal && goal <= 'Z' && state.boxes[r][c] != goal) {
                    result.add(new UnsolvedBoxGoal(goal, new Position(r, c)));
                }
            }
        }

        return result;
    }

    private static int[] selectFocusedHelpers(State state, int activeAgent, Position targetGoal) {
        ArrayList<Integer> candidates = new ArrayList<>();

        for (int agent = 0; agent < state.agentRows.length; agent++) {
            if (agent != activeAgent) {
                candidates.add(agent);
            }
        }

        candidates.sort(Comparator.comparingInt(agent ->
                Math.abs(state.agentRows[agent] - targetGoal.row)
                        + Math.abs(state.agentCols[agent] - targetGoal.col)
        ));

        int helperCount = Math.min(2, candidates.size());
        int[] helpers = new int[helperCount];

        for (int i = 0; i < helperCount; i++) {
            helpers[i] = candidates.get(i);
        }

        return helpers;
    }

    private static int focusedRepairHeuristic(LevelAnalyzer analyzer, State state, UnsolvedBoxGoal target) {
        int bestBoxDistance = LevelAnalyzer.INF;
        int bestAgentDistance = LevelAnalyzer.INF;

        for (int r = 0; r < state.boxes.length; r++) {
            for (int c = 0; c < state.boxes[r].length; c++) {
                if (state.boxes[r][c] != target.letter) {
                    continue;
                }

                Position box = new Position(r, c);
                int boxDistance = analyzer.distance(box, target.goal);
                bestBoxDistance = Math.min(bestBoxDistance, boxDistance);

                for (int agent = 0; agent < state.agentRows.length; agent++) {
                    if (State.agentColors[agent] == State.boxColors[target.letter - 'A']) {
                        int agentDistance = analyzer.distance(
                                new Position(state.agentRows[agent], state.agentCols[agent]),
                                box
                        );
                        bestAgentDistance = Math.min(bestAgentDistance, agentDistance);
                    }
                }
            }
        }

        int remaining = estimateRemainingCost(analyzer, state);
        int boxCost = bestBoxDistance < LevelAnalyzer.INF ? 8 * bestBoxDistance : 10_000;
        int agentCost = bestAgentDistance < LevelAnalyzer.INF ? bestAgentDistance : 1_000;

        return remaining + boxCost + agentCost;
    }

    private static int closestBoxDistance(LevelAnalyzer analyzer, State state, UnsolvedBoxGoal target) {
        int best = LevelAnalyzer.INF;

        for (int r = 0; r < state.boxes.length; r++) {
            for (int c = 0; c < state.boxes[r].length; c++) {
                if (state.boxes[r][c] == target.letter) {
                    best = Math.min(best, analyzer.distance(new Position(r, c), target.goal));
                }
            }
        }

        return best;
    }

    private static boolean allBoxGoalsSolved(State state) {
        return countUnsolvedBoxGoals(state) == 0;
    }

    private static List<Action> planTargetAgentToNearestBox(State state, LevelAnalyzer analyzer) {
        UnsolvedBoxGoal target = findOnlyUnsolvedBoxGoal(state);

        if (target == null) {
            return null;
        }

        int agent = firstAgentWithColor(State.boxColors[target.letter - 'A']);

        if (agent == -1) {
            return null;
        }

        BoxApproach approach = bestAccessibleTargetBoxApproach(state, analyzer, agent, target);

        if (approach == null) {
            return null;
        }

        System.err.format(
                "Target-agent approach: agent %,d at (%d,%d), nearest %c box %s, plan=%s.%n",
                agent,
                state.agentRows[agent],
                state.agentCols[agent],
                target.letter,
                approach.box,
                approach.plan == null ? "none" : Integer.toString(approach.plan.size())
        );
        return approach.plan;
    }

    private static int firstAgentWithColor(Color color) {
        for (int agent = 0; agent < State.agentColors.length; agent++) {
            if (State.agentColors[agent] == color) {
                return agent;
            }
        }

        return -1;
    }

    private static Position nearestBoxToGoal(State state, LevelAnalyzer analyzer, UnsolvedBoxGoal target) {
        Position bestBox = null;
        int bestDistance = LevelAnalyzer.INF;

        for (int r = 0; r < state.boxes.length; r++) {
            for (int c = 0; c < state.boxes[r].length; c++) {
                if (state.boxes[r][c] != target.letter) {
                    continue;
                }

                int distance = analyzer.distance(new Position(r, c), target.goal);

                if (distance < bestDistance) {
                    bestDistance = distance;
                    bestBox = new Position(r, c);
                }
            }
        }

        return bestBox;
    }

    private static BoxApproach bestAccessibleTargetBoxApproach(
            State state,
            LevelAnalyzer analyzer,
            int agent,
            UnsolvedBoxGoal target
    ) {
        BoxApproach best = null;
        int bestScore = Integer.MAX_VALUE;

        for (int r = 0; r < state.boxes.length; r++) {
            for (int c = 0; c < state.boxes[r].length; c++) {
                if (state.boxes[r][c] != target.letter) {
                    continue;
                }

                Position box = new Position(r, c);
                int boxDistance = analyzer.distance(box, target.goal);

                if (boxDistance >= LevelAnalyzer.INF) {
                    continue;
                }

                // In late goal-room repair, distant bank boxes are usually decoys.
                if (c < target.goal.col - 8 && boxDistance > 10) {
                    continue;
                }

                List<Action> plan = planAgentToAnyAdjacentCell(state, agent, box, 20_000);

                if (plan == null) {
                    continue;
                }

                int score = 8 * boxDistance + plan.size();

                if (score < bestScore) {
                    bestScore = score;
                    best = new BoxApproach(box, plan);
                }
            }
        }

        return best;
    }

    private static List<Action> planAgentToAnyAdjacentCell(State state, int agent, Position box, int maxExpansions) {
        PriorityQueue<AgentMoveNode> open = new PriorityQueue<>(Comparator.comparingInt(node -> node.f));
        java.util.HashSet<Position> closed = new java.util.HashSet<>();
        Position start = new Position(state.agentRows[agent], state.agentCols[agent]);

        open.add(new AgentMoveNode(start, null, null, 0, start.manhattanDistance(box)));

        int expansions = 0;

        while (!open.isEmpty() && expansions++ < maxExpansions) {
            AgentMoveNode node = open.poll();

            if (!closed.add(node.pos)) {
                continue;
            }

            if (node.pos.manhattanDistance(box) == 1) {
                return extractAgentMovePlan(node);
            }

            for (Action action : MOVE_ACTIONS) {
                Position next = node.pos.translate(action.agentRowDelta, action.agentColDelta);

                if (closed.contains(next) || !cellFreeForAgentMove(state, next, agent)) {
                    continue;
                }

                int g = node.g + 1;
                int h = next.manhattanDistance(box);
                open.add(new AgentMoveNode(next, node, action, g, g + 3 * h));
            }
        }

        return null;
    }

    private static boolean cellFreeForAgentMove(State state, Position pos, int movingAgent) {
        if (!inBounds(pos.row, pos.col) || State.walls[pos.row][pos.col] || state.boxes[pos.row][pos.col] != 0) {
            return false;
        }

        for (int agent = 0; agent < state.agentRows.length; agent++) {
            if (agent == movingAgent) {
                continue;
            }

            if (state.agentRows[agent] == pos.row && state.agentCols[agent] == pos.col) {
                return false;
            }
        }

        return true;
    }

    private static List<Action> extractAgentMovePlan(AgentMoveNode node) {
        java.util.LinkedList<Action> plan = new java.util.LinkedList<>();

        while (node.parent != null) {
            plan.addFirst(node.action);
            node = node.parent;
        }

        return plan;
    }

    private static RepairRegion buildGoalRoomRegion(Position targetGoal, State state) {
        int minRow = Math.max(0, targetGoal.row - 3);
        int maxRow = Math.min(State.walls.length - 1, targetGoal.row + 5);
        int minCol = Math.max(0, targetGoal.col - 6);
        int maxCol = Math.min(State.walls[0].length - 1, targetGoal.col);

        for (int r = 0; r < State.goals.length; r++) {
            for (int c = 0; c < State.goals[r].length; c++) {
                char goal = State.goals[r][c];

                if ('A' <= goal && goal <= 'Z'
                        && Math.abs(r - targetGoal.row) <= 3
                        && Math.abs(c - targetGoal.col) <= 6) {
                    minRow = Math.min(minRow, r - 1);
                    maxRow = Math.max(maxRow, r + 1);
                    minCol = Math.min(minCol, c - 1);
                    maxCol = Math.max(maxCol, c + 1);
                }
            }
        }

        return new RepairRegion(
                Math.max(0, minRow),
                Math.min(State.walls.length - 1, maxRow),
                Math.max(0, minCol),
                Math.min(State.walls[0].length - 1, maxCol)
        );
    }

    private static int[] selectGoalRoomAgents(State state, RepairRegion region, UnsolvedBoxGoal target) {
        ArrayList<Color> colors = new ArrayList<>();
        colors.add(State.boxColors[target.letter - 'A']);

        for (int r = region.minRow; r <= region.maxRow; r++) {
            for (int c = region.minCol; c <= region.maxCol; c++) {
                char box = state.boxes[r][c];

                if ('A' <= box && box <= 'Z') {
                    Color color = State.boxColors[box - 'A'];

                    if (!colors.contains(color)) {
                        colors.add(color);
                    }
                }
            }
        }

        ArrayList<Integer> agents = new ArrayList<>();

        for (Color color : colors) {
            int bestAgent = -1;
            int bestDistance = Integer.MAX_VALUE;

            for (int agent = 0; agent < state.agentRows.length; agent++) {
                if (State.agentColors[agent] != color || agents.contains(agent)) {
                    continue;
                }

                int distance = distanceToRegion(state.agentRows[agent], state.agentCols[agent], region);

                if (distance < bestDistance) {
                    bestDistance = distance;
                    bestAgent = agent;
                }
            }

            if (bestAgent != -1) {
                agents.add(bestAgent);
            }
        }

        int[] result = new int[Math.min(3, agents.size())];

        for (int i = 0; i < result.length; i++) {
            result[i] = agents.get(i);
        }

        return result;
    }

    private static int distanceToRegion(int row, int col, RepairRegion region) {
        int rowDistance = row < region.minRow
                ? region.minRow - row
                : (row > region.maxRow ? row - region.maxRow : 0);
        int colDistance = col < region.minCol
                ? region.minCol - col
                : (col > region.maxCol ? col - region.maxCol : 0);

        return rowDistance + colDistance;
    }

    private static int goalRoomRepairHeuristic(
            LevelAnalyzer analyzer,
            State state,
            UnsolvedBoxGoal target,
            RepairRegion region
    ) {
        int h = 0;
        int bestTargetBoxDistance = LevelAnalyzer.INF;

        for (int r = region.minRow; r <= region.maxRow; r++) {
            for (int c = region.minCol; c <= region.maxCol; c++) {
                if (state.boxes[r][c] == target.letter) {
                    bestTargetBoxDistance = Math.min(
                            bestTargetBoxDistance,
                            analyzer.distance(new Position(r, c), target.goal)
                    );
                }
            }
        }

        h += bestTargetBoxDistance < LevelAnalyzer.INF ? 25 * bestTargetBoxDistance : 20_000;

        for (int r = region.minRow; r <= region.maxRow; r++) {
            for (int c = region.minCol; c <= region.maxCol; c++) {
                char goal = State.goals[r][c];

                if ('A' <= goal && goal <= 'Z' && state.boxes[r][c] != goal) {
                    h += goal == target.letter ? 500 : 70;
                }
            }
        }

        for (int c = Math.max(region.minCol, target.goal.col - 2); c < target.goal.col; c++) {
            char box = state.boxes[target.goal.row][c];

            if ('A' <= box && box <= 'Z' && box != target.letter) {
                h += 250;
            }
        }

        return h;
    }

    private static ArrayList<FocusedStep> expandGoalRoomRepair(
            State state,
            int[] activeAgents,
            RepairRegion region,
            UnsolvedBoxGoal target,
            int activeBoxDistanceLimit,
            LevelAnalyzer analyzer
    ) {
        int numAgents = state.agentRows.length;
        ArrayList<FocusedStep> result = new ArrayList<>();

        for (int agent : activeAgents) {
            boolean targetColor = State.agentColors[agent] == State.boxColors[target.letter - 'A'];
            Action[] actions = applicableGoalRoomActions(
                    state,
                    agent,
                    region,
                    targetColor ? target : null,
                    targetColor ? activeBoxDistanceLimit : LevelAnalyzer.INF,
                    analyzer
            );

            for (Action action : actions) {
                if (action.type == ActionType.NoOp) {
                    continue;
                }

                Action[] jointAction = noopJointAction(numAgents);
                jointAction[agent] = action;

                if (!isConflicting(state, jointAction)) {
                    result.add(new FocusedStep(applyJointAction(state, jointAction), jointAction));
                }
            }
        }

        result.sort(Comparator.comparingInt(step -> goalRoomStepScore(step, region)));
        return result;
    }

    private static Action[] noopJointAction(int numAgents) {
        Action[] jointAction = new Action[numAgents];

        for (int agent = 0; agent < numAgents; agent++) {
            jointAction[agent] = Action.NoOp;
        }

        return jointAction;
    }

    private static Action[] applicableGoalRoomActions(
            State state,
            int agent,
            RepairRegion region,
            UnsolvedBoxGoal activeTarget,
            int activeBoxDistanceLimit,
            LevelAnalyzer analyzer
    ) {
        ArrayList<Action> actions = new ArrayList<>();

        for (Action action : Action.values()) {
            if (activeTarget != null
                    && movesNonFocusedActiveBox(state, agent, action, activeTarget, activeBoxDistanceLimit, analyzer)) {
                continue;
            }

            if (!actionStaysNearGoalRoom(state, agent, action, region)) {
                continue;
            }

            if (movesProtectedRoomBox(state, agent, action, region, activeTarget)) {
                continue;
            }

            if (isApplicable(state, agent, action)) {
                actions.add(action);
            }
        }

        return actions.toArray(new Action[0]);
    }

    private static boolean movesProtectedRoomBox(
            State state,
            int agent,
            Action action,
            RepairRegion region,
            UnsolvedBoxGoal target
    ) {
        if (action.type != ActionType.Push && action.type != ActionType.Pull) {
            return false;
        }

        int[] boxSource = movedBoxSource(state, agent, action);

        if (boxSource == null) {
            return false;
        }

        int boxRow = boxSource[0];
        int boxCol = boxSource[1];

        if (!region.containsWithMargin(boxRow, boxCol, 0)) {
            return false;
        }

        char box = state.boxes[boxRow][boxCol];

        if (!('A' <= box && box <= 'Z')) {
            return false;
        }

        if (target != null && box == target.letter) {
            return false;
        }

        if (State.goals[boxRow][boxCol] != box) {
            return false;
        }

        return !isTargetPocketBlocker(boxRow, boxCol, target);
    }

    private static int[] movedBoxSource(State state, int agent, Action action) {
        if (action.type != ActionType.Push && action.type != ActionType.Pull) {
            return null;
        }

        int agentRow = state.agentRows[agent];
        int agentCol = state.agentCols[agent];

        if (action.type == ActionType.Push) {
            return new int[] {
                    agentRow + action.agentRowDelta,
                    agentCol + action.agentColDelta
            };
        }

        return new int[] {
                agentRow - action.boxRowDelta,
                agentCol - action.boxColDelta
        };
    }

    private static int[] movedBoxDestination(State state, int agent, Action action) {
        int[] source = movedBoxSource(state, agent, action);

        if (source == null) {
            return null;
        }

        return new int[] {
                source[0] + action.boxRowDelta,
                source[1] + action.boxColDelta
        };
    }

    private static boolean isTargetPocketBlocker(int row, int col, UnsolvedBoxGoal target) {
        return target != null
                && row == target.goal.row
                && col >= target.goal.col - 2
                && col < target.goal.col;
    }

    private static boolean actionStaysNearGoalRoom(State state, int agent, Action action, RepairRegion region) {
        if (action.type == ActionType.NoOp) {
            return true;
        }

        int ar = state.agentRows[agent];
        int ac = state.agentCols[agent];
        int agentDestRow = ar;
        int agentDestCol = ac;
        int boxDestRow = -1;
        int boxDestCol = -1;

        switch (action.type) {
            case Move:
                agentDestRow = ar + action.agentRowDelta;
                agentDestCol = ac + action.agentColDelta;
                break;

            case Push: {
                int boxRow = ar + action.agentRowDelta;
                int boxCol = ac + action.agentColDelta;
                agentDestRow = boxRow;
                agentDestCol = boxCol;
                boxDestRow = boxRow + action.boxRowDelta;
                boxDestCol = boxCol + action.boxColDelta;
                break;
            }

            case Pull: {
                agentDestRow = ar + action.agentRowDelta;
                agentDestCol = ac + action.agentColDelta;
                int boxRow = ar - action.boxRowDelta;
                int boxCol = ac - action.boxColDelta;
                boxDestRow = boxRow + action.boxRowDelta;
                boxDestCol = boxCol + action.boxColDelta;
                break;
            }
        }

        if (!region.containsWithMargin(agentDestRow, agentDestCol, 3)) {
            return distanceToRegion(ar, ac, region) > distanceToRegion(agentDestRow, agentDestCol, region);
        }

        return boxDestRow == -1 || region.containsWithMargin(boxDestRow, boxDestCol, 1);
    }

    private static int goalRoomStepScore(FocusedStep step, RepairRegion region) {
        int score = movingActionCount(step.jointAction);

        for (int agent = 0; agent < step.jointAction.length; agent++) {
            Action action = step.jointAction[agent];

            if (action.type == ActionType.Push || action.type == ActionType.Pull) {
                score -= 2;
            }
        }

        for (int agent = 0; agent < step.state.agentRows.length; agent++) {
            score += distanceToRegion(step.state.agentRows[agent], step.state.agentCols[agent], region);
        }

        return score;
    }

    private static ArrayList<FocusedStep> expandFocused(
            State state,
            int activeAgent,
            int[] helpers,
            UnsolvedBoxGoal target,
            int activeBoxDistanceLimit,
            LevelAnalyzer analyzer
    ) {
        int numAgents = state.agentRows.length;
        Action[][] actionSets = new Action[numAgents][];

        for (int agent = 0; agent < numAgents; agent++) {
            if (agent == activeAgent) {
                actionSets[agent] = applicableActions(
                        state,
                        agent,
                        true,
                        target,
                        activeBoxDistanceLimit,
                        analyzer
                );
            } else if (containsAgent(helpers, agent)) {
                actionSets[agent] = applicableActions(
                        state,
                        agent,
                        helperCanMoveBlockingBox(state, agent, target),
                        null,
                        LevelAnalyzer.INF,
                        analyzer
                );
            } else {
                actionSets[agent] = new Action[] { Action.NoOp };
            }
        }

        ArrayList<FocusedStep> result = new ArrayList<>();
        Action[] jointAction = new Action[numAgents];

        enumerateFocusedActions(state, actionSets, jointAction, 0, result);
        result.sort(Comparator.comparingInt(step -> movingActionCount(step.jointAction)));
        return result;
    }

    private static boolean helperCanMoveBlockingBox(State state, int agent, UnsolvedBoxGoal target) {
        Color helperColor = State.agentColors[agent];

        for (int r = Math.max(0, target.goal.row - 2); r <= Math.min(state.boxes.length - 1, target.goal.row + 2); r++) {
            for (int c = Math.max(0, target.goal.col - 2); c <= Math.min(state.boxes[r].length - 1, target.goal.col + 2); c++) {
                char box = state.boxes[r][c];

                if ('A' <= box && box <= 'Z' && State.boxColors[box - 'A'] == helperColor) {
                    return true;
                }
            }
        }

        return false;
    }

    private static void enumerateFocusedActions(
            State state,
            Action[][] actionSets,
            Action[] jointAction,
            int agent,
            ArrayList<FocusedStep> result
    ) {
        if (agent == actionSets.length) {
            if (!isConflicting(state, jointAction)) {
                result.add(new FocusedStep(applyJointAction(state, jointAction), jointAction.clone()));
            }

            return;
        }

        for (Action action : actionSets[agent]) {
            jointAction[agent] = action;
            enumerateFocusedActions(state, actionSets, jointAction, agent + 1, result);
        }
    }

    private static Action[] applicableActions(
            State state,
            int agent,
            boolean canMoveBoxes,
            UnsolvedBoxGoal activeTarget,
            int activeBoxDistanceLimit,
            LevelAnalyzer analyzer
    ) {
        ArrayList<Action> actions = new ArrayList<>();

        for (Action action : Action.values()) {
            if (!canMoveBoxes && action.type != ActionType.NoOp && action.type != ActionType.Move) {
                continue;
            }

            if (activeTarget != null
                    && movesNonFocusedActiveBox(state, agent, action, activeTarget, activeBoxDistanceLimit, analyzer)) {
                continue;
            }

            if (isApplicable(state, agent, action)) {
                actions.add(action);
            }
        }

        return actions.toArray(new Action[0]);
    }

    private static boolean movesNonFocusedActiveBox(
            State state,
            int agent,
            Action action,
            UnsolvedBoxGoal target,
            int activeBoxDistanceLimit,
            LevelAnalyzer analyzer
    ) {
        if (action.type != ActionType.Push && action.type != ActionType.Pull) {
            return false;
        }

        int agentRow = state.agentRows[agent];
        int agentCol = state.agentCols[agent];
        int boxRow = action.type == ActionType.Push
                ? agentRow + action.agentRowDelta
                : agentRow - action.boxRowDelta;
        int boxCol = action.type == ActionType.Push
                ? agentCol + action.agentColDelta
                : agentCol - action.boxColDelta;

        if (!inBounds(boxRow, boxCol)) {
            return false;
        }

        char box = state.boxes[boxRow][boxCol];

        if (box != target.letter) {
            return false;
        }

        int distance = analyzer.distance(new Position(boxRow, boxCol), target.goal);
        return distance > activeBoxDistanceLimit;
    }

    static boolean isApplicable(State state, int agent, Action action) {
        int agentRow = state.agentRows[agent];
        int agentCol = state.agentCols[agent];
        Color agentColor = State.agentColors[agent];

        switch (action.type) {
            case NoOp:
                return true;

            case Move:
                return cellIsFree(state, agentRow + action.agentRowDelta, agentCol + action.agentColDelta);

            case Push: {
                int boxRow = agentRow + action.agentRowDelta;
                int boxCol = agentCol + action.agentColDelta;

                if (!inBounds(boxRow, boxCol)) {
                    return false;
                }

                char box = state.boxes[boxRow][boxCol];

                if (box == 0 || State.boxColors[box - 'A'] != agentColor) {
                    return false;
                }

                return cellIsFree(state, boxRow + action.boxRowDelta, boxCol + action.boxColDelta);
            }

            case Pull: {
                int destRow = agentRow + action.agentRowDelta;
                int destCol = agentCol + action.agentColDelta;

                if (!cellIsFree(state, destRow, destCol)) {
                    return false;
                }

                int boxRow = agentRow - action.boxRowDelta;
                int boxCol = agentCol - action.boxColDelta;

                if (!inBounds(boxRow, boxCol)) {
                    return false;
                }

                char box = state.boxes[boxRow][boxCol];
                return box != 0 && State.boxColors[box - 'A'] == agentColor;
            }
        }

        return false;
    }

    static boolean isConflicting(State state, Action[] jointAction) {
        int n = state.agentRows.length;
        int[] agentDestRow = new int[n];
        int[] agentDestCol = new int[n];
        int[] boxSrcRow = new int[n];
        int[] boxSrcCol = new int[n];
        int[] boxDestRow = new int[n];
        int[] boxDestCol = new int[n];
        boolean[] movesBox = new boolean[n];

        for (int i = 0; i < n; i++) {
            Action action = jointAction[i];
            int ar = state.agentRows[i];
            int ac = state.agentCols[i];

            agentDestRow[i] = ar;
            agentDestCol[i] = ac;

            switch (action.type) {
                case Move:
                    agentDestRow[i] = ar + action.agentRowDelta;
                    agentDestCol[i] = ac + action.agentColDelta;
                    break;

                case Push: {
                    int boxRow = ar + action.agentRowDelta;
                    int boxCol = ac + action.agentColDelta;
                    agentDestRow[i] = boxRow;
                    agentDestCol[i] = boxCol;
                    boxSrcRow[i] = boxRow;
                    boxSrcCol[i] = boxCol;
                    boxDestRow[i] = boxRow + action.boxRowDelta;
                    boxDestCol[i] = boxCol + action.boxColDelta;
                    movesBox[i] = true;
                    break;
                }

                case Pull: {
                    agentDestRow[i] = ar + action.agentRowDelta;
                    agentDestCol[i] = ac + action.agentColDelta;
                    int boxRow = ar - action.boxRowDelta;
                    int boxCol = ac - action.boxColDelta;
                    boxSrcRow[i] = boxRow;
                    boxSrcCol[i] = boxCol;
                    boxDestRow[i] = boxRow + action.boxRowDelta;
                    boxDestCol[i] = boxCol + action.boxColDelta;
                    movesBox[i] = true;
                    break;
                }
            }
        }

        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                if (agentDestRow[i] == agentDestRow[j] && agentDestCol[i] == agentDestCol[j]) {
                    return true;
                }

                if (movesBox[i] && movesBox[j]
                        && boxDestRow[i] == boxDestRow[j]
                        && boxDestCol[i] == boxDestCol[j]) {
                    return true;
                }

                if (movesBox[i] && movesBox[j]
                        && boxSrcRow[i] == boxSrcRow[j]
                        && boxSrcCol[i] == boxSrcCol[j]) {
                    return true;
                }

                if (movesBox[j]
                        && agentDestRow[i] == boxDestRow[j]
                        && agentDestCol[i] == boxDestCol[j]) {
                    return true;
                }

                if (movesBox[i]
                        && agentDestRow[j] == boxDestRow[i]
                        && agentDestCol[j] == boxDestCol[i]) {
                    return true;
                }

                if (movesBox[i]
                        && boxDestRow[i] == state.agentRows[j]
                        && boxDestCol[i] == state.agentCols[j]) {
                    return true;
                }

                if (movesBox[j]
                        && boxDestRow[j] == state.agentRows[i]
                        && boxDestCol[j] == state.agentCols[i]) {
                    return true;
                }
            }
        }

        return false;
    }

    static State applyJointAction(State state, Action[] jointAction) {
        State next = copyState(state);

        for (int agent = 0; agent < jointAction.length; agent++) {
            Action action = jointAction[agent];
            int ar = next.agentRows[agent];
            int ac = next.agentCols[agent];

            switch (action.type) {
                case NoOp:
                    break;

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
                    next.agentRows[agent] = boxRow;
                    next.agentCols[agent] = boxCol;
                    break;
                }

                case Pull: {
                    int boxRow = ar - action.boxRowDelta;
                    int boxCol = ac - action.boxColDelta;
                    int newAgentRow = ar + action.agentRowDelta;
                    int newAgentCol = ac + action.agentColDelta;
                    int newBoxRow = boxRow + action.boxRowDelta;
                    int newBoxCol = boxCol + action.boxColDelta;
                    char box = next.boxes[boxRow][boxCol];
                    next.boxes[boxRow][boxCol] = 0;
                    next.boxes[newBoxRow][newBoxCol] = box;
                    next.agentRows[agent] = newAgentRow;
                    next.agentCols[agent] = newAgentCol;
                    break;
                }
            }
        }

        return next;
    }

    private static Action[][] extractFocusedPlan(FocusedNode goalNode, int numAgents) {
        ArrayList<Action[]> reversed = new ArrayList<>();
        FocusedNode node = goalNode;

        while (node.parent != null) {
            reversed.add(node.jointAction);
            node = node.parent;
        }

        Action[][] plan = new Action[reversed.size()][numAgents];

        for (int i = 0; i < reversed.size(); i++) {
            plan[i] = reversed.get(reversed.size() - 1 - i).clone();
        }

        return plan;
    }

    private static boolean containsAgent(int[] agents, int target) {
        for (int agent : agents) {
            if (agent == target) {
                return true;
            }
        }

        return false;
    }

    private static int movingActionCount(Action[] jointAction) {
        int count = 0;

        for (Action action : jointAction) {
            if (action.type != ActionType.NoOp) {
                count++;
            }
        }

        return count;
    }

    private static boolean cellIsFree(State state, int row, int col) {
        return inBounds(row, col)
                && !State.walls[row][col]
                && state.boxes[row][col] == 0
                && agentAt(state, row, col) == -1;
    }

    private static int agentAt(State state, int row, int col) {
        for (int agent = 0; agent < state.agentRows.length; agent++) {
            if (state.agentRows[agent] == row && state.agentCols[agent] == col) {
                return agent;
            }
        }

        return -1;
    }

    private static boolean inBounds(int row, int col) {
        return row >= 0
                && row < State.walls.length
                && col >= 0
                && col < State.walls[0].length;
    }

    private static void appendAsJointActions(
            ArrayList<Action[]> finalPlan,
            List<Action> singleAgentPlan,
            int activeAgent,
            int numAgents
    ) {
        for (Action action : singleAgentPlan) {
            Action[] jointAction = new Action[numAgents];

            for (int agent = 0; agent < numAgents; agent++) {
                jointAction[agent] = Action.NoOp;
            }

            jointAction[activeAgent] = action;
            finalPlan.add(jointAction);
        }
    }

    private static void appendJointPlan(ArrayList<Action[]> finalPlan, Action[][] suffix) {
        for (Action[] jointAction : suffix) {
            finalPlan.add(jointAction.clone());
        }
    }

    private static State simulateSingleAgentPlan(State state, List<Action> plan, int activeAgent) {
        State current = copyState(state);

        for (Action action : plan) {
            current = applySingleAgentAction(current, action, activeAgent);
        }

        return current;
    }

    private static State simulateJointPlan(State state, Action[][] plan) {
        State current = copyState(state);

        for (Action[] jointAction : plan) {
            current = applyJointAction(current, jointAction);
        }

        return current;
    }

    private static State simulateValidatedJointPlan(State state, List<Action[]> plan) {
        State current = copyState(state);

        for (Action[] jointAction : plan) {
            if (jointAction == null || jointAction.length != current.agentRows.length) {
                return null;
            }

            for (int agent = 0; agent < jointAction.length; agent++) {
                if (jointAction[agent] == null || !isApplicable(current, agent, jointAction[agent])) {
                    return null;
                }
            }

            if (isConflicting(current, jointAction)) {
                return null;
            }

            current = applyJointAction(current, jointAction);
        }

        return current;
    }

    private static State applySingleAgentAction(State state, Action action, int agent) {
        int[] agentRows = state.agentRows.clone();
        int[] agentCols = state.agentCols.clone();
        char[][] boxes = copyBoxes(state.boxes);

        int ar = agentRows[agent];
        int ac = agentCols[agent];

        switch (action.type) {
            case NoOp:
                break;

            case Move:
                agentRows[agent] = ar + action.agentRowDelta;
                agentCols[agent] = ac + action.agentColDelta;
                break;

            case Push: {
                int boxRow = ar + action.agentRowDelta;
                int boxCol = ac + action.agentColDelta;

                int newBoxRow = boxRow + action.boxRowDelta;
                int newBoxCol = boxCol + action.boxColDelta;

                char box = boxes[boxRow][boxCol];
                boxes[boxRow][boxCol] = 0;
                boxes[newBoxRow][newBoxCol] = box;

                agentRows[agent] = boxRow;
                agentCols[agent] = boxCol;
                break;
            }

            case Pull: {
                int boxRow = ar - action.boxRowDelta;
                int boxCol = ac - action.boxColDelta;

                int newAgentRow = ar + action.agentRowDelta;
                int newAgentCol = ac + action.agentColDelta;

                int newBoxRow = boxRow + action.boxRowDelta;
                int newBoxCol = boxCol + action.boxColDelta;

                char box = boxes[boxRow][boxCol];
                boxes[boxRow][boxCol] = 0;
                boxes[newBoxRow][newBoxCol] = box;

                agentRows[agent] = newAgentRow;
                agentCols[agent] = newAgentCol;
                break;
            }
        }

        return new State(
                agentRows,
                agentCols,
                State.agentColors,
                State.walls,
                boxes,
                State.boxColors,
                State.goals
        );
    }

    static State copyState(State state) {
        return new State(
                state.agentRows.clone(),
                state.agentCols.clone(),
                State.agentColors,
                State.walls,
                copyBoxes(state.boxes),
                State.boxColors,
                State.goals
        );
    }

    private static char[][] copyBoxes(char[][] boxes) {
        char[][] copy = new char[boxes.length][];

        for (int r = 0; r < boxes.length; r++) {
            copy[r] = boxes[r].clone();
        }

        return copy;
    }

    private static ArrayList<Action[]> copyPlan(ArrayList<Action[]> plan) {
        ArrayList<Action[]> copy = new ArrayList<>(plan.size());

        for (Action[] jointAction : plan) {
            copy.add(jointAction.clone());
        }

        return copy;
    }
private static void printUnsolvedGoals(State state) {
    System.err.println("Unsolved goals:");

    for (int r = 0; r < State.goals.length; r++) {
        for (int c = 0; c < State.goals[r].length; c++) {
            char goal = State.goals[r][c];

            if ('A' <= goal && goal <= 'Z') {
                if (state.boxes[r][c] != goal) {
                    System.err.format(
                            "  Box goal %c at (%d,%d) has '%c'%n",
                            goal,
                            r,
                            c,
                            state.boxes[r][c] == 0 ? '.' : state.boxes[r][c]
                    );
                }
            } else if ('0' <= goal && goal <= '9') {
                int agent = goal - '0';

                if (agent < state.agentRows.length &&
                        !(state.agentRows[agent] == r && state.agentCols[agent] == c)) {
                    System.err.format(
                            "  Agent goal %c at (%d,%d), agent currently at (%d,%d)%n",
                            goal,
                            r,
                            c,
                            state.agentRows[agent],
                            state.agentCols[agent]
                    );
                }
            }
        }
    }
}

private static List<Task> expandTaskAgents(Task task, State state) {
    ArrayList<Task> result = new ArrayList<>();

    Color boxColor = State.boxColors[task.boxLetter - 'A'];

    for (int agent = 0; agent < state.agentRows.length; agent++) {
        if (State.agentColors[agent] == boxColor) {
            result.add(new Task(
                    task.boxLetter,
                    task.boxStart,
                    task.goal,
                    agent,
                    task.priority
            ));
        }
    }

    return result;
}

private static List<Task> selectExpansionTasks(List<Task> tasks) {
    ArrayList<Task> selected = new ArrayList<>();

    // First: take the normal best tasks.
    for (int i = 0; i < tasks.size() && selected.size() < TASK_BRANCHING; i++) {
        selected.add(tasks.get(i));
    }

    // Second: also include some hard/far tasks from the end of the list.
    // This prevents important long-distance boxes from being postponed forever.
    for (int i = tasks.size() - 1; i >= 0 && selected.size() < TASK_BRANCHING + 3; i--) {
        Task task = tasks.get(i);

        if (!selected.contains(task)) {
            selected.add(task);
        }
    }

    return selected;
}
    private static final class SearchNode {
        final State state;
        final ArrayList<Action[]> plan;
        final int solvedGoals;
        final int estimateRemaining;

        SearchNode(State state, ArrayList<Action[]> plan, int solvedGoals, int estimateRemaining) {
            this.state = state;
            this.plan = plan;
            this.solvedGoals = solvedGoals;
            this.estimateRemaining = estimateRemaining;
        }

        int score() {
            // Lower is better.
            return this.plan.size()
                    + 3 * this.estimateRemaining
                    - 1000 * this.solvedGoals;
        }
    }

    private static final class PocketResult {
        final State state;
        final ArrayList<Action[]> plan;

        PocketResult(State state, ArrayList<Action[]> plan) {
            this.state = state;
            this.plan = plan;
        }
    }

    private enum Boundary {
        RIGHT,
        LEFT,
        TOP,
        BOTTOM
    }

    private static final class RepairNode {
        final State state;
        final int f;

        RepairNode(State state, int f) {
            this.state = state;
            this.f = f;
        }
    }

    private static final class UnsolvedBoxGoal {
        final char letter;
        final Position goal;

        UnsolvedBoxGoal(char letter, Position goal) {
            this.letter = letter;
            this.goal = goal;
        }
    }

    private static final class FocusedStep {
        final State state;
        final Action[] jointAction;

        FocusedStep(State state, Action[] jointAction) {
            this.state = state;
            this.jointAction = jointAction;
        }
    }

    private static final class AgentMoveNode {
        final Position pos;
        final AgentMoveNode parent;
        final Action action;
        final int g;
        final int f;

        AgentMoveNode(Position pos, AgentMoveNode parent, Action action, int g, int f) {
            this.pos = pos;
            this.parent = parent;
            this.action = action;
            this.g = g;
            this.f = f;
        }
    }

    private static final class BoxApproach {
        final Position box;
        final List<Action> plan;

        BoxApproach(Position box, List<Action> plan) {
            this.box = box;
            this.plan = plan;
        }
    }

    private static final class RepairRegion {
        final int minRow;
        final int maxRow;
        final int minCol;
        final int maxCol;

        RepairRegion(int minRow, int maxRow, int minCol, int maxCol) {
            this.minRow = minRow;
            this.maxRow = maxRow;
            this.minCol = minCol;
            this.maxCol = maxCol;
        }

        boolean containsWithMargin(int row, int col, int margin) {
            return row >= minRow - margin
                    && row <= maxRow + margin
                    && col >= minCol - margin
                    && col <= maxCol + margin;
        }
    }

    private static final class FocusedNode {
        final State state;
        final FocusedNode parent;
        final Action[] jointAction;
        final int g;
        final int f;

        FocusedNode(State state, FocusedNode parent, Action[] jointAction, int g, int f) {
            this.state = state;
            this.parent = parent;
            this.jointAction = jointAction;
            this.g = g;
            this.f = f;
        }
    }

}
