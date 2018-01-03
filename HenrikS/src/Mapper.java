/**
 * The Mapper combines all modules that makes up the hybrid architecture.
 * It rus on the main thread and handles the runtime of the program.
 * @author Henrik Söderlund on 12/23/2017.
 */
public class Mapper {
    public static String HOST = "http://127.0.0.1";
    public static int PORT = 50000;
    public static int COLS = 100;
    public static int ROWS = 100;

    // Coordinates in the map where the robot starts at
    public static int STARTCOL = COLS/2;
    public static int STARTROW = ROWS/2;

    /**
     * Main method
     * @param args Program arguments. Usage example: http://127.0.0.1:50000 -30 -20 50 40
     */
    public static void main(String[] args){
        boolean invalid = true;

        // If providing 5 arguments, run as normal
        if (args.length == 5) {
            String[] connTokens = args[0].split(":");
            if (connTokens.length == 3) {
                // Get host and port from program arguments
                HOST = connTokens[0] + ":" + connTokens[1];
                PORT = Integer.parseInt(connTokens[2]);

                // Get corner coordinates
                double[] coords1 = new double[]{Double.parseDouble(args[1])/Settings.GRID_ELEMENT_SIZE,
                                                Double.parseDouble(args[2])/Settings.GRID_ELEMENT_SIZE};
                double[] coords2 = new double[]{Double.parseDouble(args[3])/Settings.GRID_ELEMENT_SIZE,
                                                Double.parseDouble(args[4])/Settings.GRID_ELEMENT_SIZE};

                // Convert into COLS and ROWS
                COLS = (int) (Math.abs(coords1[0]) + Math.abs(coords2[0]));
                ROWS = (int) (Math.abs(coords1[1]) + Math.abs(coords2[1]));

                // Compute offset from middle that is the starting position
                STARTCOL = (int) ((coords1[0] + coords2[0]) / 2.0)+COLS/2;
                STARTROW = (int) ((coords1[1] + coords2[1]) / 2.0)+ROWS/2;

                invalid = false;
            }
        }
        // If no arguments provided, run a quick test example
        else if (args.length == 0){
            System.out.println("----------");
            System.out.println("NO ARGUMENTS FOUND, STARTING EXAMPLE ENVIRONMENT");
            System.out.println("----------");
            invalid = false;
        }

        // No program arguments given, exit
        if (invalid == true){
            System.out.println("Invalid program argments!\n"
                    +"\tUsage example: http://127.0.0.1:50000 -30 -20 50 40\n"
                    +"\twhere the last four arguments define the two corner\n"
                    +"\tcoordinates of the square area to be mapped.\n"
                    +"\tThe first corner should be negative numbers and the second corner positive numbers.\n"
                    +"\nRunning the program without arguments starts an example environment.");
            System.exit(1);
        }

        System.out.println("Starting Communications Interface...");
        RobotCommunication rCom = new RobotCommunication(HOST, PORT);

        System.out.println("Starting Cartographer...");
        Cartographer cartographer = new Cartographer(COLS,ROWS,STARTCOL,STARTROW, rCom, Thread.currentThread());
        cartographer.start();

        System.out.println("Starting Pilot...");
        Pilot pilot = new Pilot(rCom);
        pilot.start();

        System.out.println("Starting Navigator...");
        Navigator navigator = new Navigator(cartographer,pilot);
        navigator.start();

        System.out.println("Begin Mapping...");
        while (!navigator.doneMapping) {
            try {
                navigator.join();
            } catch (InterruptedException ex) {
                navigator.interrupt();
            }
        }

        cartographer.doneMapping = true;
        try {
            cartographer.join();
        } catch (InterruptedException ex) {}
        pilot.doneMapping = true;
        try {
            pilot.join();
        } catch (InterruptedException ex) {}
        try {
            Thread.sleep(1000);
        } catch(InterruptedException e){}
        System.out.println("Mapping is done!");
    }
}
