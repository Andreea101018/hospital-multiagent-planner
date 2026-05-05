package searchclient;

import java.util.Comparator;

public abstract class Heuristic
        implements Comparator<State>
{
        protected int[][][] goalDistances;
        protected int numAgents;
        protected int[][][] boxGoalDistances;
        protected boolean[] boxGoalExists;
        protected boolean[] agentGoalExists;


    public Heuristic(State initialState)
    {
        this.numAgents = initialState.agentRows.length;
        this.goalDistances = new int[numAgents][][];
        this.agentGoalExists = new boolean[numAgents];

        for (int i = 0; i < numAgents; i++)
        {
            goalDistances[i] = computeDistancesForAgent(i);
        }

        int rows = State.walls.length;
        int cols = State.walls[0].length;
        this.boxGoalDistances = new int[26][rows][cols];
        this.boxGoalExists = new boolean[26];

        for (int i = 0; i < 26; i++)
        {
            boxGoalDistances[i] = computeDistancesForBoxLetter((char) ('A' + i));
        }
    }

private int[][] computeDistancesForAgent(int agent)
{
    int rows = State.walls.length;
    int cols = State.walls[0].length;

    int[][] dist = new int[rows][cols];

    for (int r = 0; r < rows; r++)
        for (int c = 0; c < cols; c++)
            dist[r][c] = Integer.MAX_VALUE;

    int goalRow = -1;
    int goalCol = -1;

    // Find this agent's goal
    for (int r = 0; r < State.goals.length; r++)
    {
        for (int c = 0; c < State.goals[r].length; c++)
        {
            if (State.goals[r][c] == (char)('0' + agent))
            {
                goalRow = r;
                goalCol = c;
            }
        }
    }

    if (goalRow == -1 || goalCol == -1)
        return dist;

    this.agentGoalExists[agent] = true;

    java.util.ArrayDeque<int[]> queue = new java.util.ArrayDeque<>();
    queue.add(new int[]{goalRow, goalCol});
    dist[goalRow][goalCol] = 0;

    int[] dr = {-1, 1, 0, 0};
    int[] dc = {0, 0, -1, 1};

    while (!queue.isEmpty())
    {
        int[] cell = queue.poll();
        int r = cell[0];
        int c = cell[1];

        for (int d = 0; d < 4; d++)
        {
            int nr = r + dr[d];
            int nc = c + dc[d];

            if (nr >= 0 && nr < rows &&
                nc >= 0 && nc < cols &&
                !State.walls[nr][nc] &&
                dist[nr][nc] == Integer.MAX_VALUE)

            {
                dist[nr][nc] = dist[r][c] + 1;
                queue.add(new int[]{nr, nc});
            }
        }
    }

    return dist;
}

private int[][] computeDistancesForBoxLetter(char goalLetter)
{
    int rows = State.walls.length;
    int cols = State.walls[0].length;

    int[][] dist = new int[rows][cols];
    for (int r = 0; r < rows; r++)
        for (int c = 0; c < cols; c++)
            dist[r][c] = Integer.MAX_VALUE;

    java.util.ArrayDeque<int[]> queue = new java.util.ArrayDeque<>();
    for (int r = 0; r < State.goals.length; r++)
    {
        for (int c = 0; c < State.goals[r].length; c++)
        {
            if (State.goals[r][c] == goalLetter)
            {
                dist[r][c] = 0;
                queue.add(new int[]{r, c});
                boxGoalExists[goalLetter - 'A'] = true;
            }
        }
    }

    if (!boxGoalExists[goalLetter - 'A'])
        return dist;

    int[] dr = {-1, 1, 0, 0};
    int[] dc = {0, 0, -1, 1};

    while (!queue.isEmpty())
    {
        int[] cell = queue.poll();
        int r = cell[0];
        int c = cell[1];

        for (int d = 0; d < 4; d++)
        {
            int nr = r + dr[d];
            int nc = c + dc[d];

            if (nr >= 0 && nr < rows &&
                nc >= 0 && nc < cols &&
                !State.walls[nr][nc] &&
                dist[nr][nc] == Integer.MAX_VALUE)
            {
                dist[nr][nc] = dist[r][c] + 1;
                queue.add(new int[]{nr, nc});
            }
        }
    }

    return dist;
}


public int h(State s)
{
    int total = 0;

    for (int r = 0; r < State.goals.length; r++)
    {
        for (int c = 0; c < State.goals[r].length; c++)
        {
            char goal = State.goals[r][c];

            if ('A' <= goal && goal <= 'Z')
            {
                if (s.boxes[r][c] != goal)
                {
                    total++;
                }
            }
        }
    }

    return total;
}

public int hPrime(State s)
{
    int sumBoxDistances = 0;
    int sumAgentDistances = 0;
    int maxBoxDistance = 0;
    int maxAgentDistance = 0;

    for (int r = 0; r < s.boxes.length; r++)
    {
        for (int c = 0; c < s.boxes[r].length; c++)
        {
            char box = s.boxes[r][c];
            if ('A' <= box && box <= 'Z' && box != State.goals[r][c])
            {
                int idx = box - 'A';
                if (!boxGoalExists[idx])
                    return Integer.MAX_VALUE;

                int d = boxGoalDistances[idx][r][c];
                if (d == Integer.MAX_VALUE)
                    return Integer.MAX_VALUE;

                sumBoxDistances += d;
                if (d > maxBoxDistance)
                    maxBoxDistance = d;
            }
        }
    }

    for (int i = 0; i < numAgents; i++)
    {
        if (!agentGoalExists[i])
            continue;

        int d = goalDistances[i][s.agentRows[i]][s.agentCols[i]];
        if (d == Integer.MAX_VALUE)
            return Integer.MAX_VALUE;

        sumAgentDistances += d;
        if (d > maxAgentDistance)
            maxAgentDistance = d;
    }

    if (numAgents == 1)
        return sumBoxDistances + sumAgentDistances;

    return Math.max(maxBoxDistance, maxAgentDistance);
}

    public abstract int f(State s);

    @Override
    public int compare(State s1, State s2)
    {
        return Integer.compare(this.f(s1), this.f(s2));

    }
}

class HeuristicAStar
        extends Heuristic
{
    public HeuristicAStar(State initialState)
    {
        super(initialState);
    }

    @Override
    public int f(State s)
    {
        return s.g() + this.hPrime(s);
    }

    @Override
    public String toString()
    {
        return "A* evaluation";
    }
}

class HeuristicWeightedAStar
        extends Heuristic
{
    private int w;

    public HeuristicWeightedAStar(State initialState, int w)
    {
        super(initialState);
        this.w = w;
    }

    @Override
    public int f(State s)
    {
        return s.g() + this.w * this.hPrime(s);
    }

    @Override
    public String toString()
    {
        return String.format("WA*(%d) evaluation", this.w);
    }
}

class HeuristicGreedy
        extends Heuristic
{
    public HeuristicGreedy(State initialState)
    {
        super(initialState);
    }

    @Override
    public int f(State s)
    {
        return this.hPrime(s);
    }

    @Override
    public String toString()
    {
        return "greedy evaluation";
    }
}
