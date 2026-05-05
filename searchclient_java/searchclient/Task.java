package searchclient;

public final class Task {
    public final char boxLetter;
    public final Position boxStart;
    public final Position goal;
    public final int assignedAgent;
    public final int priority;

    public Task(char boxLetter, Position boxStart, Position goal, int assignedAgent, int priority) {
        this.boxLetter = boxLetter;
        this.boxStart = boxStart;
        this.goal = goal;
        this.assignedAgent = assignedAgent;
        this.priority = priority;
    }

    @Override
    public String toString() {
        return "Task{" + boxLetter + " " + boxStart + " -> " + goal + ", agent=" + assignedAgent + ", priority=" + priority + "}";
    }
}
