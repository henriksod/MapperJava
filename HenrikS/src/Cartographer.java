import java.util.LinkedList;

/**
 * Cartographer module that takes care of mapping the environment that the robot's sensors sees.
 * @author Henrik Söderlund on 12/23/2017.
 */
public class Cartographer extends Thread {

    // GridMap object where the map is stored
    private final GridMap map;

    // Communication objects
    private LaserPropertiesResponse lpr;
    private RobotCommunication rCom;

    // Path and destination for drawing in the GUI (from Navigator)
    private LinkedList<Astar.Node> path;
    private Position destination;

    // A reference to the main thread (for interrupting)
    private Thread mainThread;

    // Boolean to state if done mapping, setting this to true will turn off the cartographer
    public boolean doneMapping = false;

    // Progress of mapping. Less unmapped areas result in more progress
    public double progress = 0;

    /**
     * Constructor of the Cartographer
     * @param mapCols Columns that makes up the grid map
     * @param mapRows Rows that makes up the grid map
     * @param startCol Starting column (offset x)
     * @param startRow Starting row (offset y)
     * @param rCom Robot communications object
     * @param mainThread Main thread object
     */
    public Cartographer(int mapCols, int mapRows, int startCol, int startRow,
                        RobotCommunication rCom, Thread mainThread) {
        this.mainThread = mainThread;
        this.lpr = new LaserPropertiesResponse();
        this.rCom = rCom;

        this.destination = new Position(0,0);
        this.path = new LinkedList<>();

        LocalizationResponse lr = new LocalizationResponse();
        try{
            this.rCom.getResponse(lr);
            this.rCom.getResponse(lpr);
        }catch(Exception e){
            e.printStackTrace();
        }

        this.map = new GridMap(mapCols, mapRows, startCol, startRow);
    }

    /**
     * Gets the GridMap object
     * @return GridMap object
     */
    public GridMap getMap() {
        return map;
    }

    /**
     * This is automatically called by the thread after calling .start().
     */
    @Override
    public void run(){

        // While not done mapping
        while (!doneMapping) {
            try {
                Thread.sleep(Settings.tickMS);

                LocalizationResponse lr = new LocalizationResponse();
                LaserEchoesResponse ler = new LaserEchoesResponse();

                rCom.getResponse(lr);
                rCom.getResponse(ler);

                // Update the map
                updateMap(lr, ler, lpr);

                // Tell the main thread that the update is done!
                mainThread.interrupt();

                // Compute progress
                int numUnmapped = 0;
                for (int i = 0; i < map.grid.length; i++) {
                    for (int j = 0; j < map.grid[0].length; j++) {
                        if (map.getValue(i, j) == -1) {
                            numUnmapped++;
                        }
                    }
                }
                progress = 1.0 - ((double) numUnmapped) / (map.grid.length * map.grid[0].length);
                System.out.println("Progress: " + progress);
            } catch (Exception e) {
                //e.printStackTrace();
            }
        }

        System.out.println("Shutting down cartographer...");
    }

    /**
     * Sets the destination for drawing
     * @param p Destination position
     */
    public synchronized void setDestination(Position p){
        this.destination = p;
    }

    /**
     * Sets the path for drawing
     * @param path Path
     */
    public synchronized void setPath(LinkedList<Astar.Node> path) { this.path = path; }

    /**
     * Updates elements of the map along an echo line.
     * @param robotPosition the position of the robot
     * @param echoAngle the angle of the laser relative to the robot, in radians.
     * @param echoLength the distance from the laser scanner to the obstacle.
     */
    public void updateLine(Position robotPosition, double echoAngle, double echoLength){
        double currentX = 0;
        double currentY = 0;
        double currentDistance = 0;

        do {
            // Incrementally increase the distance by the grid element size until the echo length has been reached
            currentDistance += Settings.GRID_ELEMENT_SIZE;

            // Compute the position in world coordinates
            currentX = robotPosition.getX() + currentDistance * Math.cos(echoAngle);
            currentY = robotPosition.getY() + currentDistance * Math.sin(echoAngle);

            // To grid coordinates
            int[] coords = map.getGridIndex(currentX, currentY);

            // We dont want to update this element if it is MAX_VAL and probability of it being so is 1.
            if (map.getValue(coords[0], coords[1]) == map.MAX_VAL
                    && map.getProbability(coords[0], coords[1]) == 1)
                break;

            // If not, we only update elements that are within a distance of 30 meters
            if (currentDistance < 30) {
                // Open element (decrease by 1)
                map.open(coords[0], coords[1]);
                // This is not a frontier
                map.setFrontier(coords[0], coords[1], false);

                // For all adjacent tiles, if it is an unmapped tile or uncertain, set as frontier
                for (int i = -1; i <= 1; i++)
                    for (int j = -1; j <= 1; j++)
                        if (i != 0 || j != 0) {
                            int val = map.getValue(coords[0]+i, coords[1]+j);
                            if (val <= 7 && val != 0) {
                                map.setFrontier(coords[0]+i, coords[1]+j, true);
                            }
                        }
            }
            // If distance is more than 30 meters and it is unmapped, set as uncertain
            else if (map.getValue(coords[0], coords[1]) != 0) {
                map.uncertain(coords[0], coords[1]);
            }

        }while (currentDistance <= echoLength);

        int[] finalCoords = map.getGridIndex(currentX, currentY);

        // At the end of the echo, if within 30 meters and not open
        if (map.getValue(finalCoords[0], finalCoords[1]) != 0
                && map.getProbability(finalCoords[0], finalCoords[1]) == 1 && echoLength < 30) {
            // increase value by 3
            map.close(finalCoords[0], finalCoords[1]);

            // No tile next to this tile should be a frontier, we also create some distance between frontiers and
            // dlosed tiles.
            for (int i = -3; i <= 3; i++)
                for (int j = -3; j <= 3; j++)
                    if (i != 0 || j != 0) {
                        map.setFrontier(finalCoords[0] + i, finalCoords[1] + j, false);
                    }
        }

        // Update probailities based on previous grid. If values are the same for one element, it will increase
        // its probability, if not the same it will decrease.
        for (int i = 0; i < map.grid.length; i++) {
            for (int j = 0; j < map.grid[0].length; j++) {
                map.updateProbability(i,j,map.getValue(i,j));
            }
        }

        // Set previous grid to current grid
        for (int i = 0; i < map.grid.length; i++) {
            for (int j = 0; j < map.grid[0].length; j++) {
                map.prevGrid[i][j] = map.grid[i][j];
            }
        }

    }

    /**
     * Updates the map with new laser echo data.
     * Assumes echoes are the distance from the robot position to the detected obstacle.
     * @param lr Localization object
     * @param ler LaserEchoes object
     * @param lpr LaserProperties object
     */
    public void updateMap(LocalizationResponse lr, LaserEchoesResponse ler, LaserPropertiesResponse lpr){
        // Get data from simulation
        double robotHeading = lr.getHeadingAngle();
        Position robotPosition = new Position(lr.getPosition());

        double[] laserEchoes = ler.getEchoes();
        double[] laserAngles = getAngles(lpr);

        // Update the map for each line of the laser scanner
        double angleRelative;
        for (int i=0; i<laserAngles.length-1; i++){
            angleRelative=laserAngles[i]+robotHeading;
            updateLine(robotPosition, angleRelative, laserEchoes[i]);
        }

        // Update the visual GUI map
        int[] robotCoords = map.getGridIndex(robotPosition);
        int[] destCoords = map.getGridIndex(destination);
        map.update(robotCoords[0], robotCoords[1], destCoords[0], destCoords[1], path);
    }

    /**
     * Metod for computing a list of angles for each laser beam.
     * @param lpr Laser properties
     * @return List of angles in radians relative to the robot heading
     */
    public double[] getAngles(LaserPropertiesResponse lpr) {
        // create a table of the right size
        int beamCount = (int) ((lpr.getEndAngle()-lpr.getStartAngle())/lpr.getAngleIncrement())+1;
        double[] angles = new double[beamCount];

        double increment = 1 * (Math.PI/180);
        angles[0] = lpr.getStartAngle();

        for (int i = 1; i<beamCount; i++){
            angles[i] = angles[i-1]+increment;
        }

        return angles;
    }
}
