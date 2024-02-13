package mcl;

import java.util.*;

import map.GlobalMap;

/**
 * A class that defines a particle object. 
 */
public class Particle implements Comparable<Particle>{
	// Fixed parameters defining the standard deviations used to perturb a particle pose
	private static final double SIGMA_THETA_PERTURB = 0.2;
	private static final double SIGMA_POSITION_PERTURB = 2;
	private static GlobalMap globalMap;
	
	private double logWeight = 0;
	protected double weight = 1.0;
	private double[] pose = new double[3];	
	
	/**
	 * Instantiates a Particle object with an initial random pose.
	 */
	public Particle() {
		this.initPose();
	}
	
	/**
	 * Associates the GlobalMap object with all Particle objects.
	 * @param map	the GlobalMap object we use
	 */
	public static void setGlobalMap(GlobalMap map) {
		globalMap = map;
	}

	/**
	 * Initializes the pose of a particle.
	 */
	private void initPose() {		
		double theta = MonteCarloLocalization.RANDOM.nextDouble() * (4 * Math.PI) - 2 * Math.PI;
		double ub = 8000; 
		boolean validPose = false;
		
		// Particle should have a valid position defined by the map
		while (!validPose) {
			double x = MonteCarloLocalization.RANDOM.nextDouble() * (ub + 1);
			double y = MonteCarloLocalization.RANDOM.nextDouble() * (ub + 1);
			pose = new double[] {x, y, theta};
			validPose = isValidPosition();
		}
	}
	
	/**
	 * Checks if the current position of a particle is valid on the map.
	 * A position is valid if it can be occupied with probability > 0.8. 
	 */
	public boolean isValidPosition() {
		int x = (int) pose[0] / 10;
		int y = (int) pose[1] / 10;
		
		if (x < 0 || y < 0) {
			System.out.println(String.format("Unexpected initial position! (x, y): (%d, %d)", x, y));
			return false;
		}
		
		try {
			if (Double.parseDouble(globalMap.globalMapValues[x][y]) > 0.8) {
				return true;
			} else {
				return false;
			}
		} catch (Exception e) {
			return false;
		}
	}	
	
	/**
	 * Sets the log of weight.
	 */
	public void setLogWeight(double logw) {
		logWeight = logw;
	}

	/**
	 * Multiplicatively update weights; hence in log space, we add log of weights
	 */
	public void setLogWeight(double logw, boolean multiplicative) {
		if (multiplicative) {
			logWeight += logw;
		} else {
			setLogWeight(logw);
		}
	}
	
	/**
	 * Returns log weight
	 */
	public double getLogWeight() {
		return logWeight;
	}
	
	/**
	 * Returns weight
	 */
	public double getWeight() {
		return weight;
	}

	/**
	 * Sets the weight 
	 */
	public void setWeight(double w) {
		weight = w;
	}

	/**
	 * Returns a cloned Particle object of this Particle.
	 * By cloning we mean that the new Particle has the same pose and weight as its original.
	 * 
	 * @return	returns the cloned Particle of this Particle object with the same pose and weight
	 */
	public Particle clone() {
		Particle clone = new Particle();
		clone.setPose(this.pose);
		clone.setWeight(this.getWeight());
		clone.setLogWeight(this.getLogWeight());
		return clone;
	}
	
	/**
	 * Stochastically perturbs the pose of this particle 
	 */
	protected void perturb() {
		boolean newValidPose = false;
		double newCurrentX, newCurrentY, newCurrentTheta;
		double[] currentPose = Arrays.copyOf(pose, 3);
		
		while (!newValidPose) {
			// Perturb theta
			pose = currentPose;
			newCurrentTheta = pose[2] + SIGMA_THETA_PERTURB * MonteCarloLocalization.RANDOM.nextGaussian();
			newCurrentTheta = newCurrentTheta % (2 * Math.PI);
			
			// Perturb position
			newCurrentX = pose[0] + SIGMA_POSITION_PERTURB * MonteCarloLocalization.RANDOM.nextGaussian();
			newCurrentY = pose[1] + SIGMA_POSITION_PERTURB * MonteCarloLocalization.RANDOM.nextGaussian();
			pose = new double[] {newCurrentX, newCurrentY, newCurrentTheta};
			newValidPose = isValidPosition();
		}
	}
	
	/**
	 * Returns the pose of this particle
	 */
	public double[] getPose(){
		return pose;
	}
	
	/**
	 * Sets the pose of this particle
	 */
	public void setPose(double x, double y, double theta) {
		pose = new double[] {x, y, theta};
	}
	
	public void setPose(double[] newPose) {
		pose = Arrays.copyOf(newPose, 3);
	}
	
	/**
	 * Defines the format of string to be printed out
	 */
	public String toString() {		
		return String.format("x=%s y=%s theta=%s -- weight=%s", pose[0], pose[1], pose[2], weight);
	}
	
	/**
	 * Compares this Particle to another based on their weights. This is necessary for sorting particles.
	 * 
	 * @param other		another Particle object to compare with this Particle
	 * @return			0 if same; -1 if this < other; 1 if this > other
	 */
	@Override
	public int compareTo(Particle other) {
		if (this.weight == other.weight) 
			return 0;
		return this.weight < other.weight? -1 : 1;
	}
}
