package searchclient;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Random;

public class State
{
    private static final Random RNG = new Random(1);

    public int[] agentRows;
    public int[] agentCols;
    public static Color[] agentColors;

    public static boolean[][] walls;
    public char[][] boxes;
    public static char[][] goals;

    public static Color[] boxColors;

    public final State parent;
    public final Action[] jointAction;
    private final int g;

    private int hash = 0;

    public State(int[] agentRows, int[] agentCols, Color[] agentColors, boolean[][] walls,
                 char[][] boxes, Color[] boxColors, char[][] goals)
    {
        this.agentRows = agentRows;
        this.agentCols = agentCols;
        this.agentColors = agentColors;
        this.walls = walls;
        this.boxes = boxes;
        this.boxColors = boxColors;
        this.goals = goals;
        this.parent = null;
        this.jointAction = null;
        this.g = 0;
    }

    private State(State parent, Action[] jointAction)
    {
        this.agentRows = Arrays.copyOf(parent.agentRows, parent.agentRows.length);
        this.agentCols = Arrays.copyOf(parent.agentCols, parent.agentCols.length);
        this.boxes = new char[parent.boxes.length][];
        for (int i = 0; i < parent.boxes.length; i++)
        {
            this.boxes[i] = Arrays.copyOf(parent.boxes[i], parent.boxes[i].length);
        }

        this.parent = parent;
        this.jointAction = Arrays.copyOf(jointAction, jointAction.length);
        this.g = parent.g + 1;

        int numAgents = this.agentRows.length;
        for (int agent = 0; agent < numAgents; ++agent)
        {
            Action action = jointAction[agent];
            char box;

            int ar = this.agentRows[agent];
            int ac = this.agentCols[agent];

            switch (action.type)
            {
                case NoOp:
                    break;

                case Move:
                    this.agentRows[agent] += action.agentRowDelta;
                    this.agentCols[agent] += action.agentColDelta;
                    break;

                case Push:
                {
                    int boxRow = ar + action.agentRowDelta;
                    int boxCol = ac + action.agentColDelta;

                    int newBoxRow = boxRow + action.boxRowDelta;
                    int newBoxCol = boxCol + action.boxColDelta;

                    box = this.boxes[boxRow][boxCol];
                    this.boxes[boxRow][boxCol] = 0;
                    this.boxes[newBoxRow][newBoxCol] = box;

                    this.agentRows[agent] += action.agentRowDelta;
                    this.agentCols[agent] += action.agentColDelta;
                    break;
                }

                case Pull:
                {
                    int boxRow = ar - action.boxRowDelta;
                    int boxCol = ac - action.boxColDelta;

                    int newAgentRow = ar + action.agentRowDelta;
                    int newAgentCol = ac + action.agentColDelta;

                    int newBoxRow = boxRow + action.boxRowDelta;
                    int newBoxCol = boxCol + action.boxColDelta;

                    box = this.boxes[boxRow][boxCol];
                    this.boxes[boxRow][boxCol] = 0;
                    this.boxes[newBoxRow][newBoxCol] = box;

                    this.agentRows[agent] = newAgentRow;
                    this.agentCols[agent] = newAgentCol;
                    break;
                }
            }
        }
    }

    public int g()
    {
        return this.g;
    }

    public boolean isGoalState()
    {
        for (int row = 1; row < this.goals.length - 1; row++)
        {
            for (int col = 1; col < this.goals[row].length - 1; col++)
            {
                char goal = this.goals[row][col];

                if ('A' <= goal && goal <= 'Z' && this.boxes[row][col] != goal)
                    return false;
                else if ('0' <= goal && goal <= '9' &&
                         !(this.agentRows[goal - '0'] == row && this.agentCols[goal - '0'] == col))
                    return false;
            }
        }
        return true;
    }

    public ArrayList<State> getExpandedStates()
    {
        int numAgents = this.agentRows.length;

        Action[][] applicableActions = new Action[numAgents][];
        for (int agent = 0; agent < numAgents; ++agent)
        {
            ArrayList<Action> agentActions = new ArrayList<>(Action.values().length);
            for (Action action : Action.values())
            {
                if (this.isApplicable(agent, action))
                {
                    agentActions.add(action);
                }
            }
            applicableActions[agent] = agentActions.toArray(new Action[0]);
        }

        Action[] jointAction = new Action[numAgents];
        int[] actionsPermutation = new int[numAgents];
        ArrayList<State> expandedStates = new ArrayList<>(16);

        while (true)
        {
            for (int agent = 0; agent < numAgents; ++agent)
            {
                jointAction[agent] = applicableActions[agent][actionsPermutation[agent]];
            }

            if (!this.isConflicting(jointAction))
            {
                expandedStates.add(new State(this, jointAction));
            }

            boolean done = false;
            for (int agent = 0; agent < numAgents; ++agent)
            {
                if (actionsPermutation[agent] < applicableActions[agent].length - 1)
                {
                    ++actionsPermutation[agent];
                    break;
                }
                else
                {
                    actionsPermutation[agent] = 0;
                    if (agent == numAgents - 1)
                        done = true;
                }
            }

            if (done)
                break;
        }

        Collections.shuffle(expandedStates, State.RNG);
        return expandedStates;
    }

    private boolean isApplicable(int agent, Action action)
    {
        int agentRow = this.agentRows[agent];
        int agentCol = this.agentCols[agent];
        Color agentColor = this.agentColors[agent];

        switch (action.type)
        {
            case NoOp:
                return true;

            case Move:
                return this.cellIsFree(agentRow + action.agentRowDelta,
                                       agentCol + action.agentColDelta);

            case Push:
            {
                int boxRow = agentRow + action.agentRowDelta;
                int boxCol = agentCol + action.agentColDelta;

                if (!inBounds(boxRow, boxCol))
                    return false;

                char box = boxes[boxRow][boxCol];
                if (box == 0)
                    return false;

                if (boxColors[box - 'A'] != agentColor)
                    return false;

                return cellIsFree(boxRow + action.boxRowDelta,
                                  boxCol + action.boxColDelta);
            }

            case Pull:
            {
                int destRow = agentRow + action.agentRowDelta;
                int destCol = agentCol + action.agentColDelta;

                if (!cellIsFree(destRow, destCol))
                    return false;

                int boxRow = agentRow - action.boxRowDelta;
                int boxCol = agentCol - action.boxColDelta;

                if (!inBounds(boxRow, boxCol))
                    return false;

                char box = boxes[boxRow][boxCol];
                if (box == 0)
                    return false;

                return boxColors[box - 'A'] == agentColor;
            }
        }

        return false;
    }

    private boolean isConflicting(Action[] jointAction)
    {
        int n = agentRows.length;

        int[] agentDestRow = new int[n];
        int[] agentDestCol = new int[n];
        int[] boxSrcRow = new int[n];
        int[] boxSrcCol = new int[n];
        int[] boxDestRow = new int[n];
        int[] boxDestCol = new int[n];
        boolean[] movesBox = new boolean[n];

        for (int i = 0; i < n; i++)
        {
            Action a = jointAction[i];
            int ar = agentRows[i];
            int ac = agentCols[i];

            agentDestRow[i] = ar;
            agentDestCol[i] = ac;

            switch (a.type)
            {
                case Move:
                    agentDestRow[i] = ar + a.agentRowDelta;
                    agentDestCol[i] = ac + a.agentColDelta;
                    break;

                case Push:
                {
                    int boxRow = ar + a.agentRowDelta;
                    int boxCol = ac + a.agentColDelta;

                    agentDestRow[i] = boxRow;
                    agentDestCol[i] = boxCol;

                    boxSrcRow[i] = boxRow;
                    boxSrcCol[i] = boxCol;

                    boxDestRow[i] = boxRow + a.boxRowDelta;
                    boxDestCol[i] = boxCol + a.boxColDelta;

                    movesBox[i] = true;
                    break;
                }

                case Pull:
                {
                    agentDestRow[i] = ar + a.agentRowDelta;
                    agentDestCol[i] = ac + a.agentColDelta;

                    int boxRow = ar - a.boxRowDelta;
                    int boxCol = ac - a.boxColDelta;

                    boxSrcRow[i] = boxRow;
                    boxSrcCol[i] = boxCol;

                    boxDestRow[i] = boxRow + a.boxRowDelta;
                    boxDestCol[i] = boxCol + a.boxColDelta;

                    movesBox[i] = true;
                    break;
                }
            }
        }

        for (int i = 0; i < n; i++)
        {
            for (int j = i + 1; j < n; j++)
            {
                if (agentDestRow[i] == agentDestRow[j] &&
                    agentDestCol[i] == agentDestCol[j])
                    return true;

                if (movesBox[i] && movesBox[j] &&
                    boxDestRow[i] == boxDestRow[j] &&
                    boxDestCol[i] == boxDestCol[j])
                    return true;

                if (movesBox[i] && movesBox[j] &&
                    boxSrcRow[i] == boxSrcRow[j] &&
                    boxSrcCol[i] == boxSrcCol[j])
                    return true;

                if (movesBox[j] &&
                    agentDestRow[i] == boxDestRow[j] &&
                    agentDestCol[i] == boxDestCol[j])
                    return true;

                if (movesBox[i] &&
                    agentDestRow[j] == boxDestRow[i] &&
                    agentDestCol[j] == boxDestCol[i])
                    return true;

                if (movesBox[i] &&
                    boxDestRow[i] == agentRows[j] &&
                    boxDestCol[i] == agentCols[j])
                    return true;

                if (movesBox[j] &&
                    boxDestRow[j] == agentRows[i] &&
                    boxDestCol[j] == agentCols[i])
                    return true;
            }
        }

        return false;
    }

    private boolean inBounds(int r, int c)
    {
        return r >= 0 && r < walls.length &&
               c >= 0 && c < walls[0].length;
    }

    private boolean cellIsFree(int row, int col)
    {
        return inBounds(row, col) &&
               !this.walls[row][col] &&
               this.boxes[row][col] == 0 &&
               this.agentAt(row, col) == 0;
    }

    private char agentAt(int row, int col)
    {
        for (int i = 0; i < this.agentRows.length; i++)
        {
            if (this.agentRows[i] == row && this.agentCols[i] == col)
            {
                return (char) ('0' + i);
            }
        }
        return 0;
    }

    public Action[][] extractPlan()
    {
        Action[][] plan = new Action[this.g][];
        State state = this;
        while (state.jointAction != null)
        {
            plan[state.g - 1] = state.jointAction;
            state = state.parent;
        }
        return plan;
    }

    @Override
    public int hashCode()
    {
        if (this.hash == 0)
        {
            final int prime = 31;
            int result = 1;
            result = prime * result + Arrays.hashCode(this.agentColors);
            result = prime * result + Arrays.hashCode(this.boxColors);
            result = prime * result + Arrays.deepHashCode(this.walls);
            result = prime * result + Arrays.deepHashCode(this.goals);
            result = prime * result + Arrays.hashCode(this.agentRows);
            result = prime * result + Arrays.hashCode(this.agentCols);
            for (int row = 0; row < this.boxes.length; ++row)
            {
                for (int col = 0; col < this.boxes[row].length; ++col)
                {
                    char c = this.boxes[row][col];
                    if (c != 0)
                        result = prime * result + (row * this.boxes[row].length + col) * c;
                }
            }
            this.hash = result;
        }
        return this.hash;
    }

    @Override
    public boolean equals(Object obj)
    {
        if (this == obj)
            return true;
        if (obj == null)
            return false;
        if (this.getClass() != obj.getClass())
            return false;
        State other = (State) obj;
        return Arrays.equals(this.agentRows, other.agentRows) &&
               Arrays.equals(this.agentCols, other.agentCols) &&
               Arrays.equals(this.agentColors, other.agentColors) &&
               Arrays.deepEquals(this.walls, other.walls) &&
               Arrays.deepEquals(this.boxes, other.boxes) &&
               Arrays.equals(this.boxColors, other.boxColors) &&
               Arrays.deepEquals(this.goals, other.goals);
    }

    @Override
    public String toString()
    {
        StringBuilder s = new StringBuilder();
        for (int row = 0; row < this.walls.length; row++)
        {
            for (int col = 0; col < this.walls[row].length; col++)
            {
                if (this.boxes[row][col] > 0)
                    s.append(this.boxes[row][col]);
                else if (this.walls[row][col])
                    s.append("+");
                else if (this.agentAt(row, col) != 0)
                    s.append(this.agentAt(row, col));
                else
                    s.append(" ");
            }
            s.append("\n");
        }
        return s.toString();
    }
}
