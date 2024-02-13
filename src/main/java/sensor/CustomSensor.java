package sensor;



/**
 * This class implements the likelihood() method based on distributions you will choose to include.
 */
public class CustomSensor extends Sensor{
	/** 
	 * TODO: define any parameters you use as final constants here. 
	 */
	private static final double SIGMA_HIT = 67;
	private static final double WEIGHT_HIT = 0.64;
	
	private static final double SIGMA_HIT_TRI = 25;
	private static final double WEIGHT_HIT_TRI = 0.35;
	
	private static final double WEIGHT_HIT_UNI = 0.01;
	
	public CustomSensor() {
		super();
	}

	/**
	 * Computes the likelihood p(o_i | x) as a double array.
	 * Note that expectedObservations and actualObservations should be set up prior to calling this method.
	 */
	public double[] likelihood() {
		// TODO: implement the sensor model discussed during the lab.
		
		double[] weightedProbDensity = new double[actualObservations.length];
		double[] weightedProbDensityTri = new double[actualObservations.length];
		double[] weightedProbDensityUni = new double[actualObservations.length];
		double[] finalweightedprobdensity = new double[actualObservations.length];
		
		// Compute the Gaussian density reflecting small measurement noise
		for (int i = 0; i < actualObservations.length; i++) {
			
			//Gaussian
			double diff = actualObservations[i] - expectedObservations[i];
			weightedProbDensity[i] = WEIGHT_HIT * 1 / (Math.sqrt(2 * Math.PI) * SIGMA_HIT) * Math.exp(-1 * Math.pow(diff, 2) / (2 * Math.pow(SIGMA_HIT, 2)));
			
			//Triangular
			diff = actualObservations[i] - MAX_LASER_RANGE;
			diff = diff + (Math.sqrt(6) * SIGMA_HIT_TRI);
			
			if(Math.abs(diff)<Math.sqrt(6)*SIGMA_HIT_TRI) {
				weightedProbDensityTri[i] = WEIGHT_HIT_TRI *((Math.sqrt(6)*SIGMA_HIT_TRI-diff)/(6* Math.pow(SIGMA_HIT_TRI,2)));
			} else {
				weightedProbDensityTri[i] = 0;
			}
			
			//Uniform
			diff = actualObservations[i];
			if(diff < 0 || diff > MAX_LASER_RANGE) {
				weightedProbDensityUni[i] = 0;
			} else {
				weightedProbDensityUni[i] = WEIGHT_HIT_UNI;
			}
			
			finalweightedprobdensity[i] = weightedProbDensity[i] + weightedProbDensityTri[i] + weightedProbDensityUni[i];
		}
		
		return finalweightedprobdensity;
	}
}
