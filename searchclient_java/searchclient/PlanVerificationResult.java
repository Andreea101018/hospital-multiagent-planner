package searchclient;

final class PlanVerificationResult {
    final boolean passed;
    final String reason;

    private PlanVerificationResult(boolean passed, String reason) {
        this.passed = passed;
        this.reason = reason;
    }

    static PlanVerificationResult accepted() {
        return new PlanVerificationResult(true, null);
    }

    static PlanVerificationResult rejected(String reason) {
        return new PlanVerificationResult(false, reason);
    }
}
