package searchclient;

public final class LowLevelNode implements Comparable<LowLevelNode> {
    public final LowLevelState state;
    public final LowLevelNode parent;
    public final Action action;
    public final int g;
    public final int f;

    public LowLevelNode(LowLevelState state, LowLevelNode parent, Action action, int g, int f) {
        this.state = state;
        this.parent = parent;
        this.action = action;
        this.g = g;
        this.f = f;
    }

    @Override
    public int compareTo(LowLevelNode other) {
        int byF = Integer.compare(this.f, other.f);
        if (byF != 0) return byF;
        return Integer.compare(other.g, this.g); // prefer deeper nodes on ties
    }
}
