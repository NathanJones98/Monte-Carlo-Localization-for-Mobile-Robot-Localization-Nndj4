package odometry;
import java.util.Random;

/**
 * A class that defines the movement of particles based on logged robot data.
 * TODO: add assignment description
 */
public class CustomOdometry extends Odometry {
	
	/** 
	 * TODO: define any parameters you use as final constants here.  
	 */
	private static double weights[] = {0.001, 0.001, 0.001, 0.001};
	double [] rot1 = new double[2];
	double [] rot2 = new double[2];
	double [] tran = new double[2];
	Random rand = new Random(0);
	
	public CustomOdometry() {};
	
	 /**
	 * Computes the updated pose of a particle given the current pose, previous pose in data, and the control signal.
	 * 
	 * @param	pose		the current pose of a particle which can be thought of as a pivot point
	 * @param 	prevPose	the previous pose recorded in a log file
	 * @param	control		an action that consists of {deltaX, deltaY and deltaTheta}
	 */	
	public double[] updatePose(double[] pose, double[] prevPose, double[] control){		
		// TODO: implement the odometry model discussed during the lab
		
		double[] return_double = new double[3];
		
		//using the formulas given
		rot1[0] = Math.atan2(control[1],control[0]) - prevPose[2];
		rot2[0] = Math.atan2(control[1],control[0]) - prevPose[2];
		tran[0] = Math.sqrt(Math.pow(control[0], 2)+Math.pow(control[1], 2));
		
		rot1[1] = rot1[0] - rand.nextGaussian() * (weights[0]*(Math.abs(rot1[0])) + weights[1]*(Math.abs(tran[0])));
		rot2[1] = rot2[0] - rand.nextGaussian() * (weights[0]*(Math.abs(rot2[0])) + weights[1]*(Math.abs(tran[0])));
		tran[1] = tran[0]  - rand.nextGaussian() * (weights[2]*(Math.abs(tran[0])) + weights[3]*(Math.abs(rot1[0])+Math.abs(tran[0])));
		
		return_double[0] = pose[0] + tran[1] *Math.cos(pose[2] + rot1[1]);
		return_double[1] = pose[1] + tran[1] *Math.sin(pose[2] + rot1[1]);
		return_double[2] = pose[2] + rot1[1] + rot2[1];
		
		return return_double;
	}
}
