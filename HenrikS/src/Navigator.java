import java.util.*;

/**
 * Navigator module that handles the deliberative control of the hybrid model.
 * Only the navigator has uses the map to make decisions where to move.
 * The navigator is dependent on the Pilot and the Cartographer.
 * @author Henrik Söderlund on 12/23/2017.
 */
public class Navigator extends Thread {

    // The Cartographer
    private Cartographer cartographer;
    // The Pilot
    private Pilot pilot;
    // A* Pathfinding object
    private Astar pathFinder;

    // Boolean to state if done mapping, setting this to true will turn off the navigator
    public boolean doneMapping = false;

    // Boolean to state if the navigator has found a path or not. It will only look for a path if this is False.
    boolean foundPath = false;
    // The destination of the navigator. The navigator will provide a destination along the path for the pilot.
    Position destination;
    // The path if the navigator has found one.
    LinkedList<Astar.Node> path;

    // Starting position of the robot, we pretend that it is always (0,0). This is only used to get back when
    // the map has been fully observed.
    Position startpos = new Position(0,0);

    // Banned destination points (for now) that did result in an open path (or something else unwanted).
    HashSet<Position> blacklist;

    // Lookahead distance along the path. This is used to give a destination point for the pilot.
    public static final double LOOKAHEAD_DISTANCE = 2; // lookahead distance
    // The minimum distance to the navigator's destination before it looks for an updated route.
    public static final double POINT_REACHED_THRESHOLD = 5;

    /**
     * Constructor of the navigator.
     * @param cartographer Cartographer
     * @param pilot Pilot
     */
    public Navigator(Cartographer cartographer, Pilot pilot) {
        this.cartographer=cartographer;
        this.pilot=pilot;

        pathFinder = new Astar();
        destination = new Position(0,0);
        path = new LinkedList<>();
        blacklist = new HashSet<>();
    }

    /**
     * This is automatically called by the thread after calling .start().
     */
    @Override
    public void run(){

        while (!doneMapping) {
            // Wait for the cartographer to be done mapping
            try {
                cartographer.join();
            } catch (InterruptedException ex) {
            }

            // Get current positon
            Position curPos = pilot.getPosition();

            // If not found path or close enough to destination, find new path.
            if (foundPath == false || distance(curPos, destination) <= POINT_REACHED_THRESHOLD) {
                foundPath = false;

                // Find closest point along the frontier and let it be the destination
                Position destPos = findClosestFrontier();
                destination = destPos;

                // Convert to grid coords
                int[] curCoords = cartographer.getMap().getGridIndex(curPos);
                int[] destCoords = cartographer.getMap().getGridIndex(destPos);

                // Find the path using A*
                try {
                    path = pathFinder.findPath(cartographer.getMap().grid,
                            curCoords[0], curCoords[1],
                            destCoords[0], destCoords[1]);
                } catch (Exception e) {
                    path = new LinkedList<>();
                }

                // If path is not open, say we found path, otherwise, blacklist the destination
                if (!path.isEmpty()) {
                    foundPath = true;
                    if (!blacklist.isEmpty())
                        blacklist.clear();
                } else {
                    blacklist.add(destPos);
                }

                // Set the path and destination for the cartographer to draw (if ShowMap is active)
                cartographer.setPath(path);
                cartographer.setDestination(destPos);
            }

            // If an object is on the path, look for a new path.
            for (Astar.Node node : path) {
                if (cartographer.getMap().grid[node.x][node.y] > 7)
                    foundPath = false;
            }

            // If emergency avoidance is true, path is invalid and we need a new pat
            if (pilot.emergencyAvoidance == true) {
                int[] coords = cartographer.getMap().getGridIndex(destination);
                cartographer.getMap().close(coords[0], coords[1]);
                foundPath = false;
                pilot.emergencyAvoidance = false;
            }

            // Set destination for the Pilot based on a lookahead distance along the path
            pilot.setDestination(getLookaheadPoint(curPos, destination, path));

            // Check if done mapping
            if (pilot.reachedDestination &&
                    destination.getDistanceTo(startpos) < 1 &&
                    cartographer.progress > 0.7) {
                doneMapping = true;
            }
        }

        System.out.println("Shutting down navigator...");
    }

    /**
     * Method for getting a point along a path, from one position to a destination
     * @param from Current position
     * @param to Final destination
     * @param path Path
     * @return Position along the path, a lookahead distance away from the current position to the destination.
     */
    private Position getLookaheadPoint (Position from, Position to, LinkedList<Astar.Node> path) {
        if (!path.isEmpty()) {
            // We want to begin the search at the end of the path.
            Stack<Astar.Node> tmp = new Stack<>();
            for (Astar.Node node : path) {
                tmp.add(node);
            }

            Astar.Node current = tmp.pop();
            Position position = cartographer.getMap().getWorldCoordinates(new int[]{current.x, current.y});
            // Look for the point
            while (distance(from, position) > LOOKAHEAD_DISTANCE && !tmp.isEmpty()) {
                current = tmp.pop();
                position = cartographer.getMap().getWorldCoordinates(new int[]{current.x, current.y});
            }
            return position;
        }
        return to;
    }

    /**
     * Finds a point in the frontier closest to the robot, given some restrictions to follow,
     * e.g. not being too close to an object and not being too small of a frontier.
     * @return Position found in the frontier that is closest to the robot
     */
    public Position findClosestFrontier() {
        GridMap map = cartographer.getMap();

        Position robotPosition = pilot.getPosition();

        Position destination = null;
        // Find frontier closest to current position
        for (int i = 0; i < map.frontierGrid.length; i++) {
            for (int j = 0; j < map.frontierGrid[0].length; j++) {
                if (map.frontierGrid[i][j] == true) {
                    // Frontier size around the current point (max 8)
                    int frontierSize = 1;
                    // Open tiles around the current point (max 8 but very unlikely)
                    int nextToOpen = 0;
                    for (int k = -1; k <= 1; k++)
                        for (int l = -1; l <= 1; l++)
                            if (k != 0 || l != 0) {
                                if (map.getFrontier(i+k,j+l) == true)
                                    frontierSize++;
                                if (map.getValue(i+k,j+l) == 0)
                                    nextToOpen++;
                            }

                    // Frontier size and adjacent open tiles has to be adequate
                    if (frontierSize > 3 && nextToOpen >= 3) {
                        Position currentDest = map.getWorldCoordinates(new int[]{i, j});
                        boolean invalidPos = false;
                        // It should not be blacklisted
                        for (Position pos : blacklist)
                            if (pos.getDistanceTo(currentDest) < 0.1)
                                invalidPos = true;
                        // If everything went well, set it as current destination
                        if (!invalidPos && (destination == null
                                || robotPosition.getDistanceTo(currentDest)
                                    < robotPosition.getDistanceTo(destination))) {

                            destination = currentDest;
                        }
                    }
                }
            }
        }

        // If not found any, go to (0,0)
        if (destination == null){
            blacklist.clear();
            return new Position(0,0);
        }
        return destination;
    }

    /**
     * Gets the euclidean distance from one point to another.
     * @param x1 x of point 1
     * @param y1 y of point 1
     * @param x2 x of point 2
     * @param y2 y of point 2
     * @return distance in meters
     */
    private double distance (double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow(x2-x1,2)+Math.pow(y2-y1,2));
    }

    /**
     * Gets the euclidean distance from one point to another.
     * @param from Position 1
     * @param to Position 2
     * @return distance in meters
     */
    private double distance(Position from, Position to) {
        return distance(from.getX(), from.getY(), to.getX(), to.getY());
    }

}
