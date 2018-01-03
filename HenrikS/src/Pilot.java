
/**
 * Pilot module that handles the reactive control of the hybrid model.
 * Only the pilot has direct control of the actuators.
 * @author Henrik Söderlund on 12/23/2017.
 */
public class Pilot extends Thread {

    private RobotCommunication robotComm;
    private LaserPropertiesResponse lpr;
    private LocalizationResponse lr;
    private DifferentialDriveRequest dr;
    private LaserEchoesResponse ler;

    // Pilot destination
    private Position destination;

    // Angular velocity PID controller
    private PID pidAngularVel;

    // Boolean to state if done mapping, setting this to true will turn off the pilot
    public boolean doneMapping = false;
    // Boolean used for emergency avoidance. If true, the robot will start backing and try to turn away from the obstale
    public boolean emergencyAvoidance = false;
    // Boolean to state if the current destination has been reached
    public boolean reachedDestination = false;

    // Parameters used by the pilot
    public static final double POINT_REACHED_THRESHOLD = 0.5;
    public static final double MAXIMUM_AVOIDANCE_DISTANCE = 5;
    public static final double EMERGENCY_AVOIDANCE_PERCETAGE = 0.9;
    public static final double AVOIDANCEFACTOR = 0.4;
    public static final double MAXIMUM_LINEAR_VELOCITY = 0.9;

    /**
     * Constructor of the Pilot.
     * @param robotComm Used to communicate with the web interface
     */
    public Pilot (RobotCommunication robotComm) {
        this.robotComm = robotComm;

        this.lpr = new LaserPropertiesResponse();
        this.lr = new LocalizationResponse();
        this.dr = new DifferentialDriveRequest();
        this.ler  = new LaserEchoesResponse();

        updateResponse(lpr);
        updateResponse(lr);

        destination = new Position(lr.getPosition());

        pidAngularVel = new PID(1.5, 0, 0);

    }

    /**
     * Method for updating response objects.
     * @param response Response object
     */
    public void updateResponse (Response response) {
        try{
            robotComm.getResponse(response);
        }catch (Exception e){
            e.printStackTrace();
        }
    }

    /**
     * Method for sending requests.
     * @param request Request object
     */
    public void sendRequest (Request request) {
        try{
            robotComm.putRequest(request);
        } catch (Exception e){
            e.printStackTrace();
        }
    }

    /**
     * Method for getting robot position from the simulation environment.
     * @return position
     */
    public Position getPosition () {
        updateResponse(lr);
        return new Position(lr.getPosition());
    }

    /**
     * Method for getting heading angle from the simulation environment.
     * @return heading
     */
    public double getHeading() {
        updateResponse(lr);
        return lr.getHeadingAngle();
    }

    /**
     * Method for setting the velocities of the robot.
     * @param linearVel linear velocity
     * @param angularVel angular velocity
     */
    private void setVelocities(double linearVel, double angularVel) {
        dr.setAngularSpeed(angularVel);
        dr.setLinearSpeed(linearVel);

        sendRequest(dr);
    }

    /**
     * Method that computes a value for obstacle avoidance used for anguar veloctity.
     * It takes the laser angles and distances to compute the avidance value based on the shortest beam.
     * @return Avoidance value
     */
    public double obstacleAvoidance() {
        updateResponse(ler);

        double[] laserEchoes = ler.getEchoes();
        double[] laserAngles = getAngles(lpr);

        // Find closest beam within maximum avoidance distance
        double closestLaserAngle = 0;
        double closestLaserDistance = 10000;
        for (int i = 0; i < laserAngles.length; i++) { // -90 to 90 degrees
            double ang = laserAngles[i];
            // Only look within 180 degrees
            if (ang > -Math.PI/2.0 && ang < Math.PI/2.0) {
                double dist = laserEchoes[i];
                if (dist < closestLaserDistance && dist <= MAXIMUM_AVOIDANCE_DISTANCE) {
                    closestLaserDistance = dist;
                    closestLaserAngle = ang;
                }
            }
        }

        // Compute avoidance value
        double avoidance = -Math.signum(closestLaserAngle)
                *(Math.cos(closestLaserAngle)+Math.abs(Math.sin(closestLaserAngle)))
                *(MAXIMUM_AVOIDANCE_DISTANCE/closestLaserDistance);

        // Return value scaled to how much avoidance we want
        return avoidance*AVOIDANCEFACTOR;
    }

    /**
     * Sets the destination of the pilot. It will try to travel to this point.
     * @param p Destination
     */
    public synchronized void setDestination(Position p){
        this.destination = p;
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

    /**
     * This is automatically called by the thread after calling .start().
     */
    @Override
    public void run() {
        while (!doneMapping) {
            Position robotPosition = getPosition();
            double robotDistance = robotPosition.getDistanceTo(destination);

            // Do the following until destination is reached
            while (robotDistance > POINT_REACHED_THRESHOLD){

                reachedDestination = false;

                try{
                    Thread.sleep(Settings.tickMS);
                }catch(InterruptedException e){}

                // Compute heading and bearing to get the angular error.
                double heading = getHeading();
                double bearing = robotPosition.getBearingTo(destination);
                double error = getAngleError(heading, bearing);

                // Compute angular velocity using the PID controller and the error.
                double angularVelocity = pidAngularVel.compute(error);
                // Set linear velocity. We don't change this.
                double linearVelocity = MAXIMUM_LINEAR_VELOCITY;

                // Compute reactive avoidance value.
                double avoidance = obstacleAvoidance();

                // Emergency stop?
                if (Math.abs(avoidance) > EMERGENCY_AVOIDANCE_PERCETAGE*MAXIMUM_AVOIDANCE_DISTANCE) {
                    linearVelocity = -0.1;
                    angularVelocity -= avoidance;
                    emergencyAvoidance = true;
                    try{
                        Thread.sleep(1000); // Drive half a second
                    }catch(InterruptedException e){
                        e.printStackTrace();
                    }
                }
                // Everything is fine, avoid objects like normal
                else {
                    emergencyAvoidance = false;
                    angularVelocity += avoidance;
                }

                // Set velocities.
                setVelocities(linearVelocity,angularVelocity);

                // Refresh data
                robotPosition = getPosition();
                robotDistance = robotPosition.getDistanceTo(destination);

            }

            setVelocities(0,0.1);
            reachedDestination = true;
        }

        setVelocities(0,0);
        System.out.println("Shutting down pilot...");
    }

    /**
     * Gets the angle error between two euler angles.
     * @param heading heading angle in radians, within the interval (-PI, PI]
     * @param bearing target angle in radians, within the interval (-PI, PI]
     * @return error angle in radians
     */
    public double getAngleError(double heading, double bearing) {
        double error = bearing - heading;
        return Math.abs(error) > Math.PI ?
                error % Math.PI - Math.signum(error)*Math.PI
                : error;
    }

}
