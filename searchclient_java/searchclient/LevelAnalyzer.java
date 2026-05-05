package searchclient;

import java.util.*;

/**
 * Static preprocessing for the hospital level.
 *
 * This class is intentionally separate from State because the project solver should not
 * recompute wall distances, dead-square information, etc. for every generated state.
 */
public final class LevelAnalyzer {
    public static final int INF = 1_000_000_000;

    public final boolean[][] walls;
    public final char[][] goals;
    public final int rows;
    public final int cols;

    private final Map<Position, int[][]> distanceCache = new HashMap<>();
    private boolean[][] deadSquares;

    public LevelAnalyzer(State initialState) {
        this.walls = State.walls;
        this.goals = State.goals;
        this.rows = walls.length;
        this.cols = walls[0].length;
    }

    public void analyze() {
        this.deadSquares = computeDeadSquares();
        // Distances are computed lazily and cached by distance(...).
    }

    public boolean inBounds(int r, int c) {
        return r >= 0 && r < rows && c >= 0 && c < cols;
    }

    public boolean inBounds(Position p) {
    return p != null && inBounds(p.row, p.col);
    }

    public boolean isWall(int r, int c) {
        return !inBounds(r, c) || walls[r][c];
    }

    public boolean isFreeStatic(Position p) {
        return inBounds(p.row, p.col) && !walls[p.row][p.col];
    }

    public int distance(Position from, Position to) {
        if (from == null || to == null) return INF;
        int[][] dist = distanceCache.computeIfAbsent(from, this::bfsFrom);
        if (!inBounds(to.row, to.col)) return INF;
        return dist[to.row][to.col];
    }

    private int[][] bfsFrom(Position source) {
        int[][] dist = new int[rows][cols];
        for (int[] row : dist) Arrays.fill(row, INF);
        if (!isFreeStatic(source)) return dist;

        ArrayDeque<Position> q = new ArrayDeque<>();
        dist[source.row][source.col] = 0;
        q.add(source);

        int[] dr = {-1, 1, 0, 0};
        int[] dc = {0, 0, -1, 1};
        while (!q.isEmpty()) {
            Position p = q.poll();
            for (int i = 0; i < 4; i++) {
                int nr = p.row + dr[i];
                int nc = p.col + dc[i];
                if (inBounds(nr, nc) && !walls[nr][nc] && dist[nr][nc] == INF) {
                    dist[nr][nc] = dist[p.row][p.col] + 1;
                    q.add(new Position(nr, nc));
                }
            }
        }
        return dist;
    }

    private boolean[][] computeDeadSquares() {
        boolean[][] dead = new boolean[rows][cols];
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                if (walls[r][c] || isGoalCell(r, c)) continue;

                boolean north = isWall(r - 1, c);
                boolean south = isWall(r + 1, c);
                boolean west = isWall(r, c - 1);
                boolean east = isWall(r, c + 1);

                // Simple but useful: a box pushed into a non-goal corner is permanently stuck.
                dead[r][c] = (north || south) && (west || east);
            }
        }
        return dead;
    }

    public boolean isDeadSquare(Position p, char boxLetter) {
        if (p == null || !inBounds(p.row, p.col)) return true;
        if (goals[p.row][p.col] == boxLetter) return false;
        return deadSquares != null && deadSquares[p.row][p.col];
    }

    public boolean isGoalCell(int r, int c) {
        return inBounds(r, c) && goals[r][c] != 0;
    }

    public List<Position> goalsForBox(char boxLetter) {
        ArrayList<Position> result = new ArrayList<>();
        for (int r = 0; r < goals.length; r++) {
            for (int c = 0; c < goals[r].length; c++) {
                if (goals[r][c] == boxLetter) result.add(new Position(r, c));
            }
        }
        return result;
    }
}
