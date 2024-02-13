package odometry;

/**
 * An abstract class defining an odometry model
 */
public abstract class Odometry {
	
	 /**
	 * Computes the updated pose of a particle given the current pose, previous pose in data, and the control signal.
	 * 
	 * @param	pose		the current pose of a particle which can be thought of as a pivot point
	 * @param 	prevPose	the previous pose recorded in a log file
	 * @param	control		an action that consists of {deltaX, deltaY and deltaTheta}
	 */	
	public abstract double[] updatePose(double[] pose, double[] prevPose, double[] control);	
}
