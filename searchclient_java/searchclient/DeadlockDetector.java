package searchclient;

/**
 * Place project-specific deadlock rules here.
 *
 * LevelAnalyzer already marks simple non-goal corners. This class is where you can add:
 * - 2x2 box/wall deadlocks,
 * - frozen boxes along walls without matching goals,
 * - tunnel deadlocks,
 * - solved boxes that should not be moved again.
 */
public final class DeadlockDetector {
    private final LevelAnalyzer analyzer;

    public DeadlockDetector(LevelAnalyzer analyzer) {
        this.analyzer = analyzer;
    }

    public boolean isObviouslyDead(Position boxPosition, char boxLetter) {
        return analyzer.isDeadSquare(boxPosition, boxLetter);
    }
}
