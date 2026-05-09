package searchclient;

import java.util.*;

/**
 * Time-indexed reservations for prioritized multi-agent planning.
 */
public final class ReservationTable {
    private final Map<Integer, Set<Position>> occupiedAtTime = new HashMap<>();
    private final Map<Integer, Set<Edge>> edgesAtTime = new HashMap<>();

    public boolean isFree(Position p, int time) {
        return !occupiedAtTime.getOrDefault(time, Collections.emptySet()).contains(p);
    }

    public boolean isEdgeFree(Position from, Position to, int time) {
        Set<Edge> edges = edgesAtTime.getOrDefault(time, Collections.emptySet());
        return !edges.contains(new Edge(from, to)) && !edges.contains(new Edge(to, from));
    }

    public void reserve(Position p, int time) {
        occupiedAtTime.computeIfAbsent(time, k -> new HashSet<>()).add(p);
    }

    public void reserveEdge(Position from, Position to, int time) {
        edgesAtTime.computeIfAbsent(time, k -> new HashSet<>()).add(new Edge(from, to));
    }
}
