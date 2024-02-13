package odometry;

/**
 * This is an example of a subclass extending the Odometry abstract class. 
 */
public class DefaultOdometry extends Odometry {

	public DefaultOdometry() {}
	
	 /**
	 * Computes the updated pose of a particle given the current pose, previous pose in data, and the control signal.
	 * DefaultOdometry model computes the next pose in a very simple (and incorrect) way: that is, the components of 
	 * the given control signal are merely added to the current pose.
	 * 
	 * @param	pose		the current pose of a particle which can be thought of as a pivot point
	 * @param 	prevPose	the previous pose recorded in a log file
	 * @param	control		an action that consists of {deltaX, deltaY and deltaTheta}
	 */		
	public double[] updatePose(double[] pose, double[] prevPose, double[] control){			
		// The control signal represents the differences in X, Y, and theta values compared to the previously logged pose.
		double deltaX = control[0];
		double deltaY = control[1];
		double deltaTheta = control[2];
		
		double[] updatedPose = new double[3];
		
		// Update the pose by simply add the control signals
		updatedPose[0] = pose[0] + deltaX;
		updatedPose[1] = pose[1] + deltaY;
		updatedPose[2] = pose[2] + deltaTheta;
		
		return updatedPose;
	}
}
