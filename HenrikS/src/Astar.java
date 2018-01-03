import java.util.*;

/**
 * A* implementation for navigation.
 * @author Henrik Söderlund on 12/28/2017.
 */
public class Astar {

    /**
     * Node class used for A*
     */
    public class Node  {

        // Distance travelled to this node.
        double g = 0;
        // Distance from this node to goal.
        double h = 0;
        // Total distance.
        double f = 0;
        // Weighted cost of this node
        double w = 0;

        // Position of this nodew
        int x, y;

        // Arrived to from the following node
        Node fromNode = this;

        /**
         * Constuctor of node
         * @param x x coord
         * @param y y coord
         */
        Node (int x, int y) {
            this.x = x;
            this.y = y;
        }

        /**
         * Sets G cost for this node
         * @param from based on the node we came from
         */
        void setGCost (Node from) {
            g = from.g + distance(from, this);
            f = g + h + w;
        }

        /**
         * Sets the H cost for this node
         * @param x goal x
         * @param y goal y
         */
        void setHCost (int x, int y) {
            h = distance(this.x, this.y, x, y);
            f = g + h + w;
        }

        /**
         * Calculates G cost from a node to this node
         * @param from a node
         * @return G cost from a node to this node
         */
        double calcGCost (Node from) {
            return from.g + distance(from, this);
        }
    }

    // Boolean that enables one to cancel the algorithm
    public boolean done = false;

    // Open list
    LinkedList<Node> openList;
    // Closed list
    LinkedList<Node> closedList;

    // A* Representation of the map
    Node[][] map;

    /**
     * Method for getting the node with the lowest F cost in the open set.
     * @return Node with lowest F cost in open set
     */
    private Node lowestFInOpen() {
        if (!openList.isEmpty()) {
            Node lowest = openList.peek();
            for (Node node : openList) {
                if (lowest.f > node.f)
                    lowest = node;
            }
            return lowest;
        }
        return null;
    }

    /**
     * Gets the adjacent nodes from the current node
     * @param map Node map
     * @param current from this node
     * @return Linked list of adjacent nodes
     */
    private LinkedList<Node> getAdjacentNodes(Node[][] map, Node current) {
        LinkedList<Node> list = new LinkedList<>();
        for (int i = -1; i <= 1; i++)
            for (int j = -1; j <= 1; j++)
                if (i != 0 || j != 0) {
                    if (current.x+i >= 0 && current.x+i < map.length
                            && current.y+j >= 0 && current.y+j < map[0].length) {
                        if (map[current.x + i][current.y + j] != null) {
                            list.add(map[current.x + i][current.y + j]);
                        }
                    }
                }

        return list;
    }

    /**
     * Calculates the path from start to end, after the algorithm has found the best path
     * @param start Start node
     * @param end End node
     * @return Path from start to end
     */
    private LinkedList<Node> calcPath (Node start, Node end) {
        //System.out.println("CALC PATH");
        Stack<Node> stack = new Stack<>();
        stack.add(end);
        Node previous = end.fromNode;
        while (!previous.equals(start) && previous != null) {
            stack.add(previous);
            previous = previous.fromNode;
        }
        LinkedList<Node> path = new LinkedList<>();
        while (!stack.isEmpty())
            path.add(stack.pop());

        return path;
    }

    /**
     * Creates the map of Nodes based on a grid of elements with value 0-15 where
     * 0 is a guaranteed open tile and 15 is guaranteed a closed tile
     * @param grid primitive map
     */
    void createMap (int[][] grid) {
        map = new Node[grid.length][grid[0].length];
        for (int i = 0; i < map.length; i++) {
            for (int j = 0; j < map[0].length; j++) {
                boolean safeDistance = true;
                boolean validTile = true;
                if (grid[i][j] != 0) {
                    // Check if not next to open tile
                    boolean nextToOpen = false;
                    for (int k = -1; k <= 1; k++)
                        for (int l = -1; l <= 1; l++)
                            if (k != 0 || l != 0) {
                                if (i + k >= 0 && i + k < map.length
                                        && j + l >= 0 && j + l < map[0].length) {
                                    if (grid[i + k][j + l] == 0) {
                                        nextToOpen = true;
                                    }
                                }
                            }

                    // If not next to open, it is not a valid tile
                    if (!nextToOpen) {
                        validTile = false;
                    }
                } else {
                    // If it is an open tile, check if it is not too close to a closed tile
                    for (int k = -2; k <= 2; k++)
                        for (int l = -2; l <= 2; l++)
                            if (k != 0 || l != 0) {
                                if (i + k >= 0 && i + k < map.length
                                        && j + l >= 0 && j + l < map[0].length) {
                                    if (grid[i + k][j + l] > 7) {
                                        safeDistance = false;
                                    }
                                }
                            }
                }

                // Check if closed tile, then it is an invalid tile
                for (int k = -1; k <= 1; k++)
                    for (int l = -1; l <= 1; l++)
                        if (k != 0 || l != 0) {
                            if (i + k >= 0 && i + k < map.length
                                    && j + l >= 0 && j + l < map[0].length) {
                                if (grid[i + k][j + l] == GridMap.MAX_VAL) {
                                    validTile = false;
                                }
                            }
                        }

                // If close enough to being an open tile (<=7) and is a valid tile
                if (grid[i][j] <= 7 && validTile) {
                    // Create node
                    Node newNode = new Node(i, j);
                    // Set weighted cost based on safe distance
                    if (safeDistance)
                        newNode.w = grid[i][j] == -1 ? GridMap.MAX_VAL : grid[i][j];
                    else
                        newNode.w = GridMap.MAX_VAL;
                    // Add it to the Node map
                    map[i][j] = newNode;
                }
                // Otherwise, set to null
                else  {
                    map[i][j] = null;
                }
            }
        }
    }

    /**
     * The main A Star Algorithm.
     *
     * Finds the shortest path from the start pos to the end pos based
     * on the standard A* heuristic (f = h + g) and an additional weighted
     * cost w so it becomes (f = g + h + w). G and H are based on the diagonal distance.
     *
     * @param startX x where the path starts
     * @param startY y where the path starts
     * @param endX x where the path ends
     * @param endY y where the path ends
     * @return the path as calculated by the A Star algorithm
     */
    public final LinkedList<Node> findPath(int[][] grid, int startX, int startY, int endX, int endY) {
        createMap(grid);

        // Make sure the positions are within the map.
        while (startX < 0)
            startX++;
        while (startX >= map.length)
            startX--;
        while (startY < 0)
            startY++;
        while (startY >= map[0].length)
            startY--;

        // Create lists
        openList = new LinkedList<Node>();
        closedList = new LinkedList<Node>();

        // Add start node to open list
        Node startNode = map[startX][startY];
        openList.add(startNode);

        // Begin
        done = false;
        Node current;
        while (!done && !openList.isEmpty()) {
            current = lowestFInOpen(); // get node with lowest fCosts from openList
            closedList.add(current); // add current node to closed list
            openList.remove(current); // delete current node from open list

            if ((current.x == endX)
                    && (current.y == endY)) { // found goal
                return calcPath(startNode, current);
            }

            // for all adjacent nodes:
            LinkedList<Node> adjacentNodes = getAdjacentNodes(map, current);
            for (Node neighbour : adjacentNodes) {
                if (closedList.contains(neighbour)) {
                    continue;
                }

                // node is not in openList
                if (!openList.contains(neighbour)) {
                    // set current node as previous for this node
                    neighbour.fromNode = current;
                    // set h costs of this node, it will also compute F cost
                    neighbour.setHCost(endX, endY);
                    // set g costs of this node, it will also compute F cost
                    neighbour.setGCost(current);
                    // add node to openList
                    openList.add(neighbour);
                }
                // node is in openList
                else {
                    // costs from current node may be cheaper than previous costs
                    if (neighbour.g > neighbour.calcGCost(current)) {
                        // set current node as previous for this node
                        neighbour.fromNode = current;
                        // set g costs of this node, it will also compute F cost
                        neighbour.setGCost(current);
                    }
                }
            }

            // no path exists
            if (openList.isEmpty()) {
                // return open list
                return new LinkedList<Node>();
            }
        }
        return new LinkedList<Node>();
    }

    /**
     * Calculates the diagonal distance from P1 to P2.
     * @param x1 x of P1
     * @param y1 y of P1
     * @param x2 x of P2
     * @param y2 y of P2
     * @return diagonal distance
     */
    double distance (int x1, int y1, int x2, int y2) {
        int D = 1;  // Cost of moving non-diagonally
        int D2 = 2; // Cost of moving diagonally
        int dx = Math.abs(x1 - x2);
        int dy = Math.abs(y1 - y2);
        return D * (dx + dy) + (D2 - 2 * D) * Math.min(dx, dy);
    }

    /**
     * Calculates the diagonal distance from one Node to another.
     * @param from node
     * @param to another node
     * @return diagonal distance
     */
    double distance (Node from, Node to) {
        return distance(from.x, from.y, to.x, to.y);
    }


}
