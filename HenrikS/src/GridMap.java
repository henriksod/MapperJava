
import java.util.LinkedList;

/**
 * The GridMap contains the map information.
 * @author Henrik Söderlund on 12/23/2017.
 */
public class GridMap {
    // Maximum value for each tile, a tile with this value is considered as an object
    public static final int MAX_VAL=15;

    // The current map grid
    public int[][] grid;
    // The previous map grid
    public int[][] prevGrid;
    // The frontier grid
    public boolean[][] frontierGrid;
    // The probability grid (used for updating)
    public double[][] probabilityGrid;
    // Number of columns and rows that makes up the grids
    public final int numCols;
    public final int numRows;
    // Starting position offsets
    public int xOffset = 0;
    public int yOffset = 0;

    // The visual map object (GUI)
    private final ShowMap showMap;
    // Whether to show the GUI or not
    private boolean showGUI = false;

    /**
     * The contructor of GridMap.
     * @param numCols Number of columns that makes uo the map
     * @param numRows Number of rows that makes up the map
     * @param xOffset Starting position x offset
     * @param yOffset Starting position y offset
     */
    public GridMap(int numCols, int numRows, int xOffset, int yOffset) {
        this.grid = new int[numCols][numRows];
        this.prevGrid = new int[numCols][numRows];
        this.frontierGrid = new boolean[numCols][numRows];
        this.probabilityGrid = new double[numCols][numRows];
        this.numCols = numCols;
        this.numRows = numRows;

        this.xOffset = xOffset;
        this.yOffset = yOffset;

        for (int i = 0; i < grid.length; i++) {
            for (int j = 0; j < grid[i].length; j++) {
                grid[i][j] = -1;
                prevGrid[i][j] = -1;
                probabilityGrid[i][j] = 0;
                frontierGrid[i][j] = false;
            }
        }

        this.showMap = new ShowMap(numCols, numRows, showGUI);
    }

    /**
     * Updates the GUI
     * @param robotRow the row where the robot is
     * @param robotCol the column where the robot is
     */
    public void update(int robotRow, int robotCol, int destRow, int destCol, LinkedList<Astar.Node> path){
        showMap.updateMap(grid,frontierGrid,MAX_VAL,
                robotRow,robotCol,
                destRow,destCol,
                path);
    }

    /**
     * Method for checking if coordinates are outside or inside the map
     * @param xElement x coord
     * @param yElement y coord
     * @return True if out of bounds, else False
     */
    public boolean outOfBounds(int xElement, int yElement) {
        if (xElement < 0 || xElement >= numCols)
            return true;
        if (yElement < 0 || yElement >= numRows)
            return true;
        return false;
    }

    /**
     * Decrements an element in the grid, used for the HIMM implementation to detect open areas
     * @param xElement the x grid coordinate (column)
     * @param yElement the y grid coordinate (row)
     */
    public void open(int xElement, int yElement) {
        if (!outOfBounds(xElement, yElement)) {
            if (grid[xElement][yElement] == -1) {
                grid[xElement][yElement] = 7;
            }
            grid[xElement][yElement]--;
            if (grid[xElement][yElement] < 0) grid[xElement][yElement] = 0;
        }
    }

    /**
     * Increments an element in the grid by 3, used for the HIMM implementation to detect closed areas
     * @param xElement the x grid coordinate (column)
     * @param yElement the y grid coordinate (row)
     */
    public void close(int xElement, int yElement) {
        if (!outOfBounds(xElement, yElement)) {
            if (grid[xElement][yElement] == -1) {
                grid[xElement][yElement] = 7;
            }
            grid[xElement][yElement]+=3;
            if (grid[xElement][yElement] > MAX_VAL) grid[xElement][yElement] = MAX_VAL;

            // To fight noise, increment adjacent tiles by 1. This will be compensated by the probability grid
            // anyways.
            for (int i = -1; i <= 1; i++)
                for (int j = -1; j <= 1; j++)
                    if (i != 0 || j != 0)
                        if (grid[xElement][yElement] < MAX_VAL) grid[xElement][yElement]++;
        }
    }

    /**
     * Sets an element in the grid to 7. This is used if the system is uncertain if there is an object in one
     * particular area or not.
     * @param xElement the x grid coordinate (column)
     * @param yElement the y grid coordinate (row)
     */
    public void uncertain(int xElement, int yElement) {
        if (!outOfBounds(xElement, yElement)) {

            grid[xElement][yElement] = 7;
        }
    }

    /**
     * Sets an element in the frontier to true or false.
     * @param xElement the x grid coordinate (column)
     * @param yElement the y grid coordinate (row)
     * @param state True if frontier, False if not
     */
    public void setFrontier(int xElement, int yElement, boolean state) {
        if (!outOfBounds(xElement, yElement)) {

            frontierGrid[xElement][yElement] = state;
        }
    }

    /**
     * Gets the frontier state from a frontier grid element. Returns false if out of bounds.
     * @param xElement the x grid coordinate (column)
     * @param yElement the y grid coordinate (row)
     * @return state of element
     */
    public boolean getFrontier(int xElement, int yElement) {
        if (!outOfBounds(xElement, yElement)) {

            return frontierGrid[xElement][yElement];
        }
        return false;
    }

    /**
     * Gets the value of a map grid element. Returns MAX_VAL if out of bounds.
     * @param xElement the x grid coordinate (column)
     * @param yElement the y grid coordinate (row)
     * @return value of a map grid element
     */
    public int getValue(int xElement, int yElement) {
        if (!outOfBounds(xElement, yElement)) {

            return grid[xElement][yElement];
        }
        return MAX_VAL;
    }

    /**
     * Gets the probability of a map grid element. Returns 0 if out of bounds.
     * Probability is based on how many times the same value has been observed for a given tile.
     * @param xElement the x grid coordinate (column)
     * @param yElement the y grid coordinate (row)
     * @return probability of a map grid element
     */
    public double getProbability(int xElement, int yElement) {
        if (!outOfBounds(xElement, yElement)) {

            return probabilityGrid[xElement][yElement];
        }
        return 0;
    }

    /**
     * Updates the probability for a given grid element
     * @param xElement the x grid coordinate (column)
     * @param yElement the y grid coordinate (row)
     * @param value the observed value
     */
    public void updateProbability(int xElement, int yElement, int value) {
        if (!outOfBounds(xElement, yElement)) {

            // Decrease probability if not same
            if (prevGrid[xElement][yElement] != value) {
                probabilityGrid[xElement][yElement] -= 0.1;
            }
            // Increase probability if same
            else {
                probabilityGrid[xElement][yElement] += 0.2;
            }
            probabilityGrid[xElement][yElement]
                    = Math.max(0, Math.min(1,probabilityGrid[xElement][yElement]));
        }
    }

    /**
     * Translates world coordinates into grid coordinates
     * @param px x coord in world coordinates
     * @param py y coord in world coordinates
     * @return the (column,row) grid coordinates as elements 0 and 1
     */
    public int[] getGridIndex(double px, double py) {
        int x = (int)(px/Settings.GRID_ELEMENT_SIZE) + xOffset;
        int y = (int)(py/Settings.GRID_ELEMENT_SIZE) + yOffset;
        return new int[]{x,y};
    }

    /**
     * Translates position coordinates into grid coordinates
     * @param p Position in world coordinates
     * @return the (column,row) grid coordinates as elements 0 and 1
     */
    public int[] getGridIndex(Position p) {
        return getGridIndex(p.getX(), p.getY());
    }

    /**
     * Translates grid coordinates into world coordinates
     * @param coords grid coordinates, elements 0 for x 1 for y
     * @return world coordinates as a Position object
     */
    public Position getWorldCoordinates(int[] coords){
        double x = ((double)(coords[0] - xOffset))*Settings.GRID_ELEMENT_SIZE;
        double y = ((double)(coords[1] - yOffset))*Settings.GRID_ELEMENT_SIZE;
        return new Position(x,y);
    }

}
