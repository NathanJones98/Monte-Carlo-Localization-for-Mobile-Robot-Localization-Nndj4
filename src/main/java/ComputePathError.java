import java.util.List;

import map.GlobalMap;
import map.Plot;
import mcl.MonteCarloLocalization;
import mcl.Observation;
import odometry.CustomOdometry;
import odometry.Odometry;
import sensor.CustomSensor;
import sensor.RayTracing;
import sensor.Sensor;

public final class ComputePathError {

	/**
	 * Computes the root mean squared error between the path you obtained and the ground truth path.
	 * @return	The RMSE value
	 */
	public static double computeError() {
		System.out.println("\n\n===========Computing the error w.r.t. the ground truth path=============\n");
  
		int numParticle = 5000;									// Set the maximum number of particles to begin with
		boolean recordPosition = true;							// Store the estimated robot poses in a list
  
		GlobalMap map = new GlobalMap();						
		Sensor sensorModel = new CustomSensor();
		Odometry odometryModel = new CustomOdometry();
		Plot plot = new Plot(map, numParticle, false);
  
		// Load pre-computed expected laser reading values
		sensorModel.setExpectedSensorReadingFromCache(RayTracing.loadRayTracing(map));
  
		// Instantiate MonteCarloLocalization object and link sensor/odometry models
		MonteCarloLocalization mcl = new MonteCarloLocalization(plot, map, numParticle);
		mcl.setOdometryModel(odometryModel);
		mcl.setSensorModel(sensorModel);
		
		// Run MCL algorithm and record the path of the most probable particles
		List<double[]> trace = mcl.runMonteCarloLocalization(recordPosition);
  
		// Read in the ground truth path
		List<Observation> groundTruthList = Observation.loadRobotLogData("data/robotdata101.log");
  
		// Skip some initial time steps as it takes time to converge
		int tStart = 0;
		for (int t = 0; t < trace.size(); t++) {
			double tFromTrace = trace.get(t)[0];
			double timestamp = groundTruthList.get(0).timeStamp;
			if (tFromTrace == timestamp) {
				tStart = t;
				break;
			}
		}
  
		// Compute the root mean square error
		double error = 0;
		int count = 0;
		int endIndex = Math.min(trace.size() - tStart, groundTruthList.size());
		for (int t = 0; t < endIndex; t++) {
			double[] traceAtTimeT = trace.get(t + tStart);
			Observation trueObs = groundTruthList.get(t);
    
			double traceX = traceAtTimeT[1];
			double traceY = traceAtTimeT[2];
			double trueX = trueObs.robotPose[0];
			double trueY = trueObs.robotPose[1];

			error += Math.pow(traceX - trueX, 2) + Math.pow(traceY - trueY, 2);
			count++;
		}
  
		error = Math.sqrt(error) / count;
		System.out.println("The RMSE is " + error + "\n");
		return error;
	}
	
	public static void main(String[] args) {
		computeError();
	}
}
