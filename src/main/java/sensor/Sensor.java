package sensor;

/**
 * An abstract class that defines a sensor model, which is extended by BasicSensor and CustomSensor classes. 
 * Note that this class has a single abstract method, likelihood(). Other than that, other methods are shared by
 * the two subclasses (hence defined here).
 * <p>
 * Essentially, a sensor model is specified by probabilistic models (density functions) that you will define.   
 */
public abstract class Sensor {
	public static final double MAX_LASER_RANGE = 2000;	// You may change this if you like
	public static final int NUM_BINS = 120;				// How many bins to use for laser readings
	
	private double[][][] cachedLaserReading;			// Precomputed expected laser readings are saved 
	protected boolean init = true;
	protected double[] actualObservations;
	protected double[] expectedObservations;
	
	
	/**
	 * An abstract method which computes the likelihood. This method should be implemented in subclasses.
	 * You should make sure to set 'actualObservations' and 'expectedObservations' arrays correctly before calling this method.
	 *  
	 * @return	a double array consisting of likelihood values corresponding to observed laser readings
	 */
	public abstract double[] likelihood();
	
	
	/**
	 * Based on the independence assumption of observations, this method computes log p(o | x) = \sum_i log p(o_i | x) where
	 * log p(o_i | x) corresponds to one of laser readings
	 * 
	 * @return	the sum of log likelihoods
	 */
	public double sumOfLogLikelihood() {
		double[] likelihood = likelihood();
		double logLikelihood = 0;
		for (int i = 0; i < likelihood.length; i++) {
			logLikelihood += Math.log(likelihood[i]);
		}
		return logLikelihood;
	}
	
	
	/**
	 * Sets the array of expected readings which are precomputed for fast lookup.
	 * 
	 * @param cachedLaserReading	a cached array object containing expected laser reading values computed via Bresenham algorithm
	 */
	public void setExpectedSensorReadingFromCache(double[][][] cachedLaserReading) {
		this.cachedLaserReading = cachedLaserReading;
	}
	
	
	/**
	 * Sets the expectedObservations array. This will be necessary to compute the likelihood of observation.
	 * 
	 * @param expectedObservations 	an array object containing expected readings
	 */
	public void setExpectedObs(double[] expectedObservations) {
		this.expectedObservations = expectedObservations;
	}
	
	
	/**
	 * Sets the actualObservations array. This will be necessary to compute the likelihood of observation.
	 * 
	 * @param actualObservations 	an array object containing the observed laser readings
	 */
	public void setActualObs(double[] actualObservations) {
		this.actualObservations = actualObservations;
	}
	
	
	/**
	 * Returns the expected laser reading from the cached array given a pose on the map
	 */
	public double getExpectedObs(int[] pose) {
		return cachedLaserReading[pose[0]][pose[1]][pose[2]];
	}
}
