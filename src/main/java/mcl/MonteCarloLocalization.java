package mcl;

import java.util.*;

import map.GlobalMap;
import map.Plot;
import odometry.Odometry;
import sensor.Sensor;
import util.Util;

/**
 * A class which runs the Monte Carlo Localization (MCL) algorithm.
 * 
 */
public class MonteCarloLocalization {
	
	public static final double LIKELIHOOD_SCALE = 0.90;				// Scale factor you may want to adjust
	private static final int RESAMPLE_PERIOD = 10;					// how frequently resample
	private static final int MIN_NUM_PARTICLE = 300;				// The minimum number of particles to keep around
	private static final int LOGNUM = 1;							// Use the robotdata1.log file
	public static final Random RANDOM = new Random(0);				// DO NOT MODIFY THIS
	private int numParticle;										// total number of particles
	private double[] prevLoggedRobotPose;
	
	private List<Observation> obsList;
	public List<double[]> trackedPosition = new ArrayList<>();
	
	private Plot plotObj = null;
	private Sensor sensorModel = null;
	private Odometry odometryModel = null;

	/**
	* TODO: since we are not using your Main.java for grading, you should make sure to correctly set 
	* default parameters (e.g. likelihoodScale)! 
	*/
 	public MonteCarloLocalization(Plot plot, GlobalMap globalMap, int numParticle) {
		// Set attributes
		this.plotObj = plot;
		this.numParticle = numParticle;
		
		// Particles live on this specific map
		Particle.setGlobalMap(globalMap);
	
		// Load data
		loadLoggedData(LOGNUM);
	}

	
	/**
	 * Runs the MCL algorithm for multiple time steps.
	 * Each Observation object contains the logged pose of the robot. 
	 * When observation.dataType is "L", 180 laser reading values are also given at that time step.  
	 * <p>
	 * Given an observation (type "O" or "L"),
	 * 		1) getControlSignal & getLaserReadings -- retrieve the control signal and laser readings
	 * 		2) updateParticles -- update poses and weights of particles; then, resample particles
	 * 		3) addParticlesIfNeeded -- apply some additional tricks to prevent early-convergence
	 * 		4) updatePlot -- plot the current particles on the map
	 * 
	 * *** DO NOT ALTER THE PARAMETER AND RETURN VALUES & TYPES *** 
	 * @param recordPosition	whether to record the track of the most probable particles
	 * @return 					when recordPosition == true, returns the list of the trace
	 */
	public List<double[]> runMonteCarloLocalization(boolean recordPosition) {		
		// Create initial particles
		List<Particle> particles = createParticles();
		
		// Iterate through time steps
		for (int t = 0; t < obsList.size(); t++) {
			Observation observation = obsList.get(t);
			
			// Get the control signal & laser observations from logged data
			double[] control = getControlSignal(observation);
			double[] laserObservations = getLaserReadings(observation);
			
			// Run a single iteration of MCL algorithm
			boolean resample = (observation.dataType.equals("L") && (t+1) % RESAMPLE_PERIOD == 0);
			particles = updateParticles(particles, control, laserObservations, resample);
			
			// Adaptively manage the number of particles
			particles = addParticlesIfNeeded(particles);

			// Sort particles according to their weights and plot on the map
			Collections.sort(particles);
			Particle p = particles.get(particles.size() - 1);				// Particle with the largest weight will be marked on the map
			
			// Track the position of the most probable particle
			if (recordPosition) {
				recordRobotPath(p, observation);
			} else {
				System.out.println(String.format("\t# of particles: %d, timestamp=%.4f particle=%s", particles.size(), observation.timeStamp, p));
			}

			plotObj.updatePlot(observation, particles);
			
			// Shuffle the particle list 
			Collections.shuffle(particles, RANDOM);
			
			prevLoggedRobotPose = observation.robotPose;
		}
		
		// 
		if (recordPosition) {
			return trackedPosition;
		} else {
			return null;
		}
	}
	

	/**
	 * The core method which constitutes a single iteration of MCL. 
	 * Given the list of current particles, control, and laser observations, the following steps should be implemented: 
	 * 		1) particle poses are updated based on the control signal;
	 * 		2) particles are reweighted based on the laser readings;
	 * 		3) weights are renormalized via the exp-normalize trick;
	 * 		4) particles are resampled according to their weights.
	 * <p>
	 * Note that when we don't resample, particle weights are multiplicatively updated.
	 * When resampling happens, all weights are reset to 1 (i.e. log weight = 0).
	 * 
	 * @param currParticles		a list of current particles 
	 * @param control			an array of control signals: {deltaX, deltaY, deltaTheta}
	 * @param laserObservations	an array of 180 laser readings 
	 * @param resample			whether to resample or not
	 * @return
	 */
	private List<Particle> updateParticles(List<Particle> currParticles, double[] control, double[] laserObservations, boolean resample) {
		List<Particle> candidateParticles = new ArrayList<>();
		
		// Loop through particles
		for (int i = 0; i < currParticles.size(); i++) {
			Particle p = currParticles.get(i);
			
			// Update the pose of particle given control
			updatePose(p, control);
			
			if (p.isValidPosition()) {
				// Let the sensor model compute the weight
				updateWeight(p, laserObservations);
				
				// Only keep particles with valid positions	
				candidateParticles.add(p);
			}
		}
		
		// Resample
		if (resample) {
			// Normalize weights using the exp-normalize trick
			normalizeWeights(candidateParticles, true);
			
			// Resample particles as per their weights
			List<Particle> newParticles = resampleParticles(candidateParticles);
			return newParticles;
		} 
		// Or simply return valid particles with updated weights and poses
		else {
			return candidateParticles;
		}
	}
	
	
	/**
	 * Updates the pose of this particle based on the odometry model and control signal
	 * 
	 * @param particle
	 * @param control
	 */
	public void updatePose(Particle particle, double[] control) {
		// TODO
		particle.setPose(odometryModel.updatePose(particle.getPose(), prevLoggedRobotPose, control));
	}
	
	
	/**
	 * Updates the weight of a particle given a single laser observation data (180 observations).
	 * <p>
	 * The exp-normalize trick for numerical stability should be implemented (instead of directly computing the likelihood, 
	 * you keep track of its log value).
	 * Also, see e.g. https://timvieira.github.io/blog/post/2014/02/11/exp-normalize-trick/
	 * <p>
	 * Make sure that the log weights are updated multiplicatively. 
	 * Also, as explained in the slides, you may want to reduce the impact of a single update by a factor of alpha such that
	 * 		weight = p(o | x)^{alpha}
	 * 		or, log weight = alpha * log p(o | x)
	 * 
	 * @param particle				a particle whose weight is to be updated
	 * @param laserObservations		a new array containing laser observations from a single time step
	 */
	public void updateWeight(Particle particle, double[] laserObservations) {
		// Update the weight only when laser readings are given
		if (laserObservations.length != 0) {
			// TODO:
			
			setObsArrayOfSensor(particle.getPose(), laserObservations);
			particle.setLogWeight(sensorModel.sumOfLogLikelihood() * LIKELIHOOD_SCALE, true);
			particle.setWeight(Math.exp(particle.getLogWeight()));
			
			//INDArray temp_observation[] = new INDArray[180];
			
			//for(int i = 0; i < 180; i++) {
				//temp_observation[i].add(laserObservations[i]);
			//}
			//temp_observation[] = sensorModel.likelihood();
			
		}
	}
	
	
	/**
	 * Normalizes weights of particles. When expNormalize == true, then we apply the exp-normalize trick.
	 *
	 * @param particles
	 * @param expNormalize
	 */
	public void normalizeWeights(List<Particle> particles, boolean expNormalize) {
		if (expNormalize) {
			// TODO: use the exp-normalize trick for normalizing weights
			
			int index_max = 0;
			for(int i = 0; i < particles.size(); i++) {
				if(particles.get(i).getLogWeight() > particles.get(index_max).getLogWeight()){
					index_max = i;
				}	
			}
			
			double b = particles.get(index_max).getLogWeight();
			double num = 0;
			double denom = 0;
			
			for(int i = 0; i < particles.size(); i++) {
				double temp = particles.get(i).getLogWeight() - b;
				denom = denom + Math.exp(temp);
			}
			
			for(int i = 0; i < particles.size(); i++) {
				double temp = particles.get(i).getLogWeight() - b;
				num = Math.exp(temp);
				
				particles.get(i).setWeight(num/denom);
			}
			
		} else {
			// Directly normalize weights
			normalizeWeights(particles);
		}
	}
	
	
	/**
	 * Sample with probabilities proportional to the weights of particles.
	 * This method returns a list whose ith element corresponds to how many times the ith sample has been sampled.
	 *  
	 * @param particles	a list of particles
	 * @return 			an Integer array of counts 
	 */
	public Integer[] sampleMultinomial(List<Particle> particles){
		// TODO
		int size = particles.size();
		
		Integer[] vals = new Integer[size];
		for(int i = 0; i < size; i++) {
			vals[i] = 0;
		}
		
		double random_num = 0;
		double weight = 0;
		
		for(int iter = 0; iter < size; iter++) {
			random_num = RANDOM.nextDouble();
			
			for(int i = 0; i < size; i++) {
				weight = particles.get(i).getWeight();
				if(random_num <= weight) {
					vals[i]++;
					i = size;
					//System.out.println(i);
				} else if (random_num > weight) {
					random_num -= weight;
				}
			}
		}
		return vals;
	}
	
	
	/**
	 * Samples new particles with probabilities proportional to their weight.
	 * Duplicate particles are suppressed unless there are fewer particles than targetNumParticle.
	 * For multiplicative update of weights, log of weight is reset to 0 (i.e. w = 1).
	 * 
	 * @return a list of selected (perturbed) particles
	 */
	private List<Particle> resampleParticles(List<Particle> particles){
		Particle p;
		List<Particle> newParticles = new ArrayList<>();
		
		// Sample indices of particles with replacement according to their weights
		Integer[] sampledIndexCounts = sampleMultinomial(particles);
		
		int newParticleCount = 0;
		for (int i = 0; i < particles.size(); i++) {
			newParticleCount ++;
			
			// How many times the ith particle is sampled?
			int count = sampledIndexCounts[i];
			
			// For sampled particles, suppress adding all duplicate particles
			while (count > 0) {
				if (count == 1) {
					// Include the particle at least once if count > 0
					p = particles.get(i);
					p.setLogWeight(0);
					newParticles.add(p);
				} 
				else if (count < 3 || newParticleCount < MIN_NUM_PARTICLE) {
					// Particle is cloned and perturbed if more than one should be added
					p = particles.get(i).clone();
					p.perturb();
					p.setLogWeight(0);
					newParticles.add(p);
				}
				count -= 1;
			}			
		}
		return newParticles;
	}
	
	
	/**
	 * Normalizes particle weights.
	 * 
	 * @param particles		the list of particles whose weights will be normalized
	 */
	private void normalizeWeights(List<Particle> particles) {
		double weightSum = 0;
		for (Particle p : particles) {
			weightSum += p.getWeight();
		}
		
		for (Particle p : particles) {
			p.setWeight(p.getWeight() / weightSum);
		}
	}
	
	
	/**
	 * This method adaptively manages the list of particles. When we have no particles left, 
	 * simply create particles of size numParticle. When the number of particles drop below targetNumParticle,
	 * we randomly select some particles which are cloned, perturbed and added to the particle list.
	 *  
	 * @param particles		the list of remaining particles
	 * @return				a list of particles
	 */
	private List<Particle> addParticlesIfNeeded(List<Particle> particles) {
		int numRemParticles = particles.size();
		if (numRemParticles == 0) {
			particles = createParticles();
			numRemParticles = numParticle;
		}
		
		while (numRemParticles < MIN_NUM_PARTICLE) {
			int index = RANDOM.nextInt(numRemParticles);
			Particle p = particles.get(index).clone();
			p.perturb();
			particles.add(p);
			numRemParticles++;
		}
		return particles;
	}
	
	
	/**
	 * Returns the control signal at a specific time step, which is computed by comparing the current odometry value and the previous one.
	 *  
	 * @param 	observation	an observation containing the logged robot pose 
	 * @return				the control signal: {deltaX, deltaY, deltaTheta}
	 */
	private double[] getControlSignal(Observation observation) {
		double[] currLoggedRobotPose = observation.robotPose;
		double[] control = new double[3];
		
		if (prevLoggedRobotPose == null) {
			prevLoggedRobotPose = currLoggedRobotPose;
		}
		
		for (int i = 0; i < currLoggedRobotPose.length; i++) {
			control[i] = currLoggedRobotPose[i] - prevLoggedRobotPose[i];
		}
		return control;
	}
	
	
	/**
	 * Returns the laser sensor readings of a specific time step.
	 * 
	 * @param 	observation	an observation containing the logged laser readings (observation.dataType should be "L")
	 * @return				an array consisting of laser readings
	 */
	private double[] getLaserReadings(Observation observation) {
		if (observation.dataType.equals("L")) {
			return observation.laserReadings;
		} else {
			return new double[0];
		}
		
	}
	

	/**
	 * 	Sets the observation arrays of Sensor object based on the current pose of particle. 
	 *  The expected laser readings must have been precomputed for theta \in [0, 2 * PI].
	 *	The recorded laser readings span the range of [theta - PI/2, theta + PI/2].
	 *	@param particlePose			the current pose of a particle
	 *	@param laserObservations	the laser readings at a specific time step
	 */
	private void setObsArrayOfSensor(double[] particlePose, double[] laserObservations){
		double expectedReading;
		int numBins = Sensor.NUM_BINS;							// [0, 2 * PI] range has been divided into discrete number of bins
		int numBinsConsidered =  numBins / 2;					// 180 degrees correspond to the half of bins
		
		// Get the laser readings
		double[] subsampledObservations = new double[numBinsConsidered];
		double[] expectedObservations = new double[numBinsConsidered];
		
		// Laser sensor is placed 25cm ahead of the robot
		double theta = particlePose[2];
		double xLaserLoc = particlePose[0] + 25 * Math.cos(theta);
		double yLaserLoc = particlePose[1] + 25 * Math.sin(theta);
		int xIndex = (int) Math.min(xLaserLoc / 10, 799);
		int yIndex = (int) Math.min(yLaserLoc / 10, 799);
		
		// Set the starting value of theta and get its bin id
		theta = theta - Math.PI / 2;
		int binId = Util.thetaToBinId(theta, Sensor.NUM_BINS);
		int iInc = 180 / numBinsConsidered;
		
		// Retrieve the precomputed expected reading and initialize the actual/expected observation arrays
		for (int i = 0; i < numBinsConsidered; i++) {
			expectedReading = sensorModel.getExpectedObs(new int[] {xIndex, yIndex, binId});
			subsampledObservations[i] = laserObservations[iInc * i];
			expectedObservations[i] = expectedReading;
			binId = (binId + 1) % numBins;
		}
		
		sensorModel.setExpectedObs(expectedObservations);
		sensorModel.setActualObs(subsampledObservations);
	}

	
	/**
	 * Creates numParticle number of Particle objects and returns the list of them.
	 */
	public List<Particle> createParticles() {
		List<Particle> particles = new ArrayList<>();
		for (int i = 0; i < numParticle; i++) {
			particles.add(new Particle());
		}
		return particles;
	}
	

	/**
	 * Store the path of the most probable particles (may change at every iteration) with timestamps.
	 */
	private void recordRobotPath(Particle p, Observation observation) {
		double t = observation.timeStamp;
		double[] pose = p.getPose();
		double[] trace = new double[] {t, pose[0], pose[1]};
		trackedPosition.add(trace);
	}
	
	
	/**
	 * Sets the sensor model -- THIS IS USED ONLY TO AID TESTING; DO NOT USE YOURSELF
	 */
	public void setSensorModel(Sensor sensorModel) {
		this.sensorModel = sensorModel;
	}
	
	
	/**
	 * Sets the odometry model -- THIS IS USED ONLY TO AID TESTING; DO NOT USE YOURSELF
	 */
	public void setOdometryModel(Odometry odometryModel) {
		this.odometryModel = odometryModel;
	}
	
		
	/**
	 * Loads robot data and processes them
	 */
	private void loadLoggedData(int logNum) {
		obsList = Observation.loadRobotLogData(String.format("data/robotdata%d.log", logNum));
	}
}
