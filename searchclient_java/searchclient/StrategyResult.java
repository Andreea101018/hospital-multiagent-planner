package searchclient;

final class StrategyResult {
    final String name;
    final Action[][] plan;
    final boolean verified;
    final int actionCount;
    final double runtimeSeconds;
    final int cbsBoxAttempted;
    final int cbsBoxSolved;
    final boolean fallbackUsed;
    final boolean finalAgentCBSUsed;
    final String rejectionReason;

    private StrategyResult(
            String name,
            Action[][] plan,
            HybridRunStats stats,
            long startTime
    ) {
        this(
                name,
                plan,
                stats.verificationPassed,
                plan.length,
                (System.nanoTime() - startTime) / 1_000_000_000.0,
                stats.cbsBoxAttempted,
                stats.cbsBoxSolved,
                stats.fallbackUsed,
                stats.finalAgentCBSUsed,
                stats.rejectionReason
        );
    }

    private StrategyResult(
            String name,
            Action[][] plan,
            boolean verified,
            int actionCount,
            double runtimeSeconds,
            int cbsBoxAttempted,
            int cbsBoxSolved,
            boolean fallbackUsed,
            boolean finalAgentCBSUsed,
            String rejectionReason
    ) {
        this.name = name;
        this.plan = plan;
        this.verified = verified;
        this.actionCount = actionCount;
        this.runtimeSeconds = runtimeSeconds;
        this.cbsBoxAttempted = cbsBoxAttempted;
        this.cbsBoxSolved = cbsBoxSolved;
        this.fallbackUsed = fallbackUsed;
        this.finalAgentCBSUsed = finalAgentCBSUsed;
        this.rejectionReason = rejectionReason;
    }

    static StrategyResult from(String name, Action[][] plan, HybridRunStats stats, long startTime) {
        return new StrategyResult(name, plan, stats, startTime);
    }

    StrategyResult withName(String newName) {
        return new StrategyResult(
                newName,
                plan,
                verified,
                actionCount,
                runtimeSeconds,
                cbsBoxAttempted,
                cbsBoxSolved,
                fallbackUsed,
                finalAgentCBSUsed,
                rejectionReason
        );
    }

    boolean cbsContributed() {
        return cbsBoxSolved > 0 || finalAgentCBSUsed;
    }
}
