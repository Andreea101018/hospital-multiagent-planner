package searchclient;

import java.util.Objects;

public final class Position {
    public final int row;
    public final int col;

    public Position(int row, int col) {
        this.row = row;
        this.col = col;
    }

    public Position plus(int dr, int dc) {
        return new Position(this.row + dr, this.col + dc);
    }

    public int manhattanDistance(Position other) {
        return Math.abs(this.row - other.row) + Math.abs(this.col - other.col);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof Position)) return false;
        Position position = (Position) o;
        return row == position.row && col == position.col;
    }

    @Override
    public int hashCode() {
        return Objects.hash(row, col);
    }

    @Override
    public String toString() {
        return "(" + row + "," + col + ")";
    }

    public Position translate(int rowDelta, int colDelta) {
    return new Position(this.row + rowDelta, this.col + colDelta);
}
}
