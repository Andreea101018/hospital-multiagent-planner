package searchclient;

public final class LocalGroupNode implements Comparable<LocalGroupNode> {
    public final LocalGroupState state;
    public final LocalGroupNode parent;
    public final Action action;
    public final int g;
    public final int f;

    public LocalGroupNode(LocalGroupState state, LocalGroupNode parent, Action action, int g, int f) {
        this.state = state;
        this.parent = parent;
        this.action = action;
        this.g = g;
        this.f = f;
    }

    @Override
    public int compareTo(LocalGroupNode other) {
        return Integer.compare(this.f, other.f);
    }
}