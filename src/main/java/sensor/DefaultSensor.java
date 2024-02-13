package sensor;

/**
 * This class is an example class which computes likelihood() method based simply on 
 * the Gaussian centered at the expected laser reading value.
 */
public class DefaultSensor extends Sensor{
	
	private static final double SIGMA_HIT = 50;
	private static final double WEIGHT_HIT = 1.0;

	public DefaultSensor() {
		super();
	}
	
	/**
	 * Computes p(o_i | x) as an array. Here, observations are modeled as Gaussians centered around the corresponding expected readings.
	 */
	public double[] likelihood() {
		double[] weightedProbDensity = new double[actualObservations.length];
		
		// Compute the Gaussian density reflecting small measurement noise
		for (int i = 0; i < actualObservations.length; i++) {
			double diff = actualObservations[i] - expectedObservations[i];
			weightedProbDensity[i] = WEIGHT_HIT * 1 / (Math.sqrt(2 * Math.PI) * SIGMA_HIT) * Math.exp(-1 * Math.pow(diff, 2) / (2 * Math.pow(SIGMA_HIT, 2)));
		}
	
		return weightedProbDensity;
	}
}
