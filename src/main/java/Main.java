
import map.GlobalMap;
import map.Plot;
import mcl.MonteCarloLocalization;
import odometry.DefaultOdometry;
import odometry.Odometry;
import sensor.DefaultSensor;
import sensor.RayTracing;
import sensor.Sensor;

/**
 * The main class which runs the MCL algorithm.
 */
public class Main {
	private static final int NUM_PARTICLE_TO_DRAW = 2000;
	private static final boolean DRAW_LOGGED_POSE = false;				// Whether to directly draw the recorded pose in data
	
	/**
	 * The main method
	 * @param args	args[0] is the maximum number of particles to use 
	 */
	public static void main(String[] args) {
		
		int numParticle = Integer.parseInt(args[0]);					// Set the maximum number of particles to begin with
		boolean recordPosition = false;									// Whether to store the estimated robot poses in a list
		
		GlobalMap map = new GlobalMap();						
		Sensor sensorModel = new DefaultSensor();
		Odometry odometryModel = new DefaultOdometry();
		Plot plot = new Plot(map, NUM_PARTICLE_TO_DRAW, DRAW_LOGGED_POSE);
		
		// Load pre-computed expected laser reading values
		sensorModel.setExpectedSensorReadingFromCache(RayTracing.loadRayTracing(map));
		
		// Instantiate MonteCarloLocalization object and link sensor/odometry models
		MonteCarloLocalization mcl = new MonteCarloLocalization(plot, map, numParticle);
		mcl.setOdometryModel(odometryModel);
		mcl.setSensorModel(sensorModel);
		
		// Run MCL algorithm
		mcl.runMonteCarloLocalization(recordPosition);
	}
}
