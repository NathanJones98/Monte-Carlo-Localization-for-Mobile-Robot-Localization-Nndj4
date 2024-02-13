# Project 3: Monte-Carlo Localization for Mobile Robot Localization

(The original code was developed by Yudong (William) Xu.)

In this project, you will implement the Monte Carlo Localization algorithm described in the following paper: 
- [Sebastian Thrun, Dieter Fox, Wolfram Burgard, and Frank Dellaert. Robust Monte Carlo Localization for Mobile Robots. *Artificial Intelligence*, 128(1–2):99–141, 2001](https://www.ri.cmu.edu/pub_files/pub2/thrun_sebastian_2001_1/thrun_sebastian_2001_1.pdf).

You **must** understand *Section 1 and 2* of this paper (which will also be discussed in lab) for correct implementation. In essence, MCL approximately represents the posterior belief over the true pose of a mobile robot using *particles* and iteratively updates its belief by incorporating observations via sampling-importance-resampling. 

### Problem description

<img src="https://github.com/jihwan-jeong/figures-repo/blob/master/robot-localization.png" width="800">

Your robot is roaming around in a building with an odometer and a laser rangefinder. With the map of the building, the odometry and laser sensor data, your MCL algorithm will help the lost robot figure out its whereabouts.

A pose of the robot consists of its position (x, y) on the map and the direction that the robot is facing towards (theta) --- see the figure above. Now, starting from an unknown pose, the robot collects sensory data from its laser rangefinder (we interchangeably use the terms 'sensor' and 'rangefinder'). The sensor spans 180 degrees, i.e. from (theta - PI/2) to (theta + PI/2). Additionally, the odometer will tell you which control signal has been acted upon. We define the control signal as (deltaX, deltaY, deltaTheta) where

- deltaX = X_{t} - X_{t-1}
- deltaY = Y_{t} - Y_{t-1} 
- deltaTheta = theta_{t} - theta_{t-1}

when t is the current time step and t-1 is the one before. Although there are online versions of MCL, we are given pre-recorded poses of the robot in this project. Hence, we will look up the logged odometry and sensor data to compute the path of the robot in hindsight.

<p float="left">
<img src="https://github.com/jihwan-jeong/figures-repo/blob/master/pic0001.png" width="400" />
<img src="https://github.com/jihwan-jeong/figures-repo/blob/master/pic0002.png" width="400" />
</p>

Initially, particles are randomly sampled and scattered across the map with equal weights (the first figure above). At this stage, it is impossible to figure out which particles have similar poses as the actual robot. However, as we observe data, we re-weight each particle according to how *likely* it has resulted in the given observation. Based on this likelihood, we discard some particles and keep more probable particles around. Over time, the MCL algorithm would nicely approximate the true pose of the robot (the second figure above). 

In a more procedural sense, the MCL algorithm should perform the following steps at each time step:

1. Receive the control signal from the odometer;
2. Update the pose of every particle based on the control signal;
3. Update the weight of every particle using the laser rangefinder readings;
4. Normalize the weights of all particles;
5. Resample particles according to the normalized weights of particles.


**[Dataset]**

The map and the dataset you are given are the following files:

- instruct.txt - Format description of the map file and the data file
- robotdata{}.log - Five data logs of odometry and laser readings
- wean.dat.gz - The map where the robot moves around

In a robotdata{}.log file, you will find that laser measurements may (type "L") or may not (type "O") be recorded at each time step. Read instruct.txt closely to understand the meaning of each number in the log files and the map file.

**[Project description]**

Now, you will implement Step 2 through to Step 5 in this project. Successful implementation of **Question 1** will pass all the tests in [Tests.java](/src/test/java/Tests.java). Therefore, you will get the full marks (4pts) for passing these tests. Note however that there exists a very low (but nonzero) chance of failing to pass the *testMultinomialSampling* test even with a correct implementation. This is because the test is statistical in nature. In this case, we will manually grade your *sampleMultinomial* method. 

Additionally, your MCL algorithm should correctly converge (at least approximately) to the correct poses (or the correct path) of the robot. We will use the RMSE score to measure the discrepancy in the paths. You will find that you need to have sensible odometry and sensor models to get the MCL algorithm with reasonable performance. For the competitive portion in Q1(d) (2pts), we will look at your RMSE score.

In **Question 2**, you will implement odometry and sensor models (4pts). Your implementation will be evaluated via your performance on the RMSE score. Should you fail to get an error below the bar we set, we will manually grade your odometry/sensor models to see if you have correctly followed the content in the lab slides.

Please note that you will be referred back to the content of the MCL paper we cited at the beginning and the project slides (will be given out before lab) if you ask a question that is directly answered in the paper. If you are uncertain about specific parts of the algorithm, the sensor or the odometry models even after reading through the references, you may post a question on Piazza.

## Question 1. Complete the MCL algorithm [6pts]

You need to implement all major components of the MCL algorithm (namely, *updatePose*, *updateWeight*, *normalizeWeights*, *sampleMultinomial*) defined in the [MonteCarloLocalization.java](/src/main/java/mcl/MonteCarloLocalization.java) file.

(a) **[1pt]** Implement the *updatePose* method (see the 'Q1(a): updatePose' slide in the lab slides). You are provided with the *getControlSignal(observation)* method which returns the control signal by comparing the logged robot pose of the current time step and that of the previous time step. You should call the *updatePose* method of an odometry model (either DefaultOdometry or CustomOdometry if implemented) to compute the next pose of a particle. Then, make sure to **set** the pose of the particle with the new pose. This method can be implemented in just a few lines of code. Successful implementation must pass the *testUpdatePose()* test in Tests.java. 

(b) **[1pt]** Implement the *updateWeight* method (see the 'Q1(b): updateWeight' slide in the lab slides). According to Thrun et al. (2001), an weight of a particle `i` is defined as `w_t(i) = p(o_t|s_t, i)`, which is simply the likelihood of the observation `o_t` at the current pose `s_t`. At time `t`, `o_t` consists of multiple laser sensor readings and we assume that they are all mutually independent. Hence, if there are `M` number of laser readings at `t`, you compute `w_t(i) = p(o_t|s_t, i) = p(o_t1|s_t, i) * p(o_t2|s_t, i) * ... * p(o_tM|s_t, i)`. This value is computed for each and every particle (`i=1,...,N`), after which you normalize them. 

In practice, this procedure may incur numerical over/underflow. Therefore, you will instead keep track of the log-likelihood, `log w_t(i) = log p(o_t|s_t, i) = \sum_j log p(o_tj|s_t, i)`, of each particle (hence the log of weight). 

Furthermore, you will not completely replace the previous weight of a particle with the currently computed one when you **don't** resample. You should rather update the weight in a multiplicative way; that is, `w_t(i) = p(o_t|s_t, i) * w_{t-1}(i)`. In the log space, this update scheme corresponds to `log w_t(i) = log p(o_t|s_t, i) + log w_{t-1}(i)` (see the *setLogWeight* method in [Particle.java](/src/main/java/mcl/Particle.java)). When you **resample**, the weights should be reset to 1 (see the *resampleParticles* method and Equation 4.37 of [this](https://docs.ufpr.br/~danielsantos/ProbabilisticRobotics.pdf)).

Finally, you are going to multiply the LIKELIHOOD_SCALE constant (defined in MonteCarloLocalization.java) to the log of weight before setting it for each particle. This is to scale how much impact you would allow a single observation to have on the weight of the particle. See Section 6.3.4 of [this](https://docs.ufpr.br/~danielsantos/ProbabilisticRobotics.pdf)) and the lab slides for more detailed explanation.

Now, for the *updateWeight* method, you are given a particle and an array of laser readings. With this, you need to compute the log weight of the particle and set it in the multiplicative way as explained above. This method can be completed in a few lines of code (Note: make sure to call *setObsArrayOfSensor* method before computing weights for correct implementation). Successful implementation must pass the *testUpdateWeight* test in Tests.java.

(c) Implement the *normalizeWeights* and *sampleMultinomial* methods, which are necessary for resampling particles (see the slide titled 'Q1(c): normalizeWeights and sampleMultinomial' in the lab slides).

(c-1) **[1pt]** Implement the *normalizeWeights* method via the Exp-Normalize trick (see [this](https://timvieira.github.io/blog/post/2014/02/11/exp-normalize-trick/)). Remember that you will be tracking the log of weights as discussed in Q1(b). This trick is for the sake of numerical stability, which is useful when you work with probabilities. Make sure, however, that you call the *setWeight* method to update the weights of particles once you have computed the normalized weights. Successful implementation must pass the *testExpNormalize*  test in Tests.java.

(c-2) **[1pt]** Implement the *sampleMultinomial* method which returns an array of counts of particles. You should sample from the multinomial distribution defined by the normalized weights of particles (SHOULD BE NORMALIZED). The number of particles sampled should be the same as the number of particles in the particle list given to the method.

Note that the return value is not a list of particles but an array of integers. Elements of this array correspond to how many times particles are sampled. For example, assume there are 3 particles [p1, p2, p3] and we get [2, 1, 0] as the return value of this method. This means that p1 was sampled twice, p2 once, and none of p3 were sampled. 

It is very likely that successful implementation would pass the *testMultinomialSampling* test in Tests.java. However, as different JVMs may result in different implementations of Random, you may still fail to pass the test with a correctly implemented method (because the test is statistical in nature). Hence, should you fail to pass the test, we will manually grade your *sampleMultinomial* method.

(d) **[2pt]** Competitive portion

We will evaluate the performance of your MCL implementation by computing the RMSE of the predicted path you get with respect to the ground truth path of the robot. The ground truth path is defined in [robotdata101.log](/data/robotdata101.log), and you can check the animation of the correct path from [/animations/true_path.mp4](/animations/true_path.mp4). Your predicted path is determined by the pose of the most probable particle at each time step.

Note that we will fix the maximum number of particles at 5000 for evaluation although you can change `TARGET_NUM_PARTICLE` in MonteCarloLocalization.java. You can compute the error of your robot's path by running the [ComputePathError.java](/src/main/java/ComputePathError.java) file. Note that the performance is measured only for *robotdata1.log* file. When computing the error, we skip the first 160 time steps (~10sec) as the initial guesses can be far away from the correct position. 

As for the scoring, **you will get the full 2 points if you get the zero error. RMSE errors greater than 20** will result in 0 point for this competitive portion. Otherwise, you will get a linearly interpolated score.


## Question 2. Implement Sensor and Odometry Models [4pts]

In this part, you will implement odometry and sensor models that the MCL algorithm will use to compute the movement of particles as well as their weights. Since there is no single correct model, we will evaluate your implementation by the estimated path that your MCL algorithm produces. That is, you get the full 4 points for successfully identifying the true robot path, which will be determined by the error computed in the competitive portion explained just above. In the case your MCL implementation fails to provide a reasonable quality path, we will manually grade your odometry/sensor models according to how closely they come to implementing the models discussed in the lab slides.

(a) **[2pts]** Complete the odometry model discussed in the lab slides (see the slide titled 'Q2(a): CustomOdometry model') in [CustomOdometry.java](/src/main/java/odometry/CustomOdometry.java). Specifically, you need to implement the *updatePose* method in the file. You are given [DefaultOdometry.java](/src/main/java/odometry/DefaultOdometry.java) file which helps you understand the arguments and return values of the *updatePose* method. Note that you should make any of the parameters --- necessary for implementing the method --- as final constants in the CustomOdometry.java file. 

(b) **[2pts]** Complete the sensor model discussed in the lab slides (see the slide titled 'Q2(b): CustomSensor model') in [CustomSensor.java](/src/main/java/sensor/CustomSensor). Specifically, you need to implement the *likelihood* method in the class file. Unlike the DefaultSensor class which models the likelihood via a simple Gaussian, you will need to define *uniform* and *triangular* densities as will be dicussed during the lab. Again, you should make any of the parameters --- necessary for implementing the method --- as final constants in the CustomSensor.java file.

***

For computational efficiency, this project has introduced the following two measures:

(1) The expected laser rangefinder values are precomputed and cached. This file will be loaded and used as a lookup table.

(2) Instead of computing the expected laser readings for *all* possible thetas (which is impossible), we discretize 360 degrees (or 2\*PI radians) into some number of bins. Currently, we divided the range into 120 bins. Hence, you will find [range_array_120bin.npy.zip](/data/range_array_120bin.npy.zip) file. Unzip this file in the same directory (/data). Once you instantiate a Sensor object and a GlobalMap object, you should call 

```java
sensor.setExpectedSensorReadingFromCache(RayTracing.loadRayTracing(map));
```

to load the cached file. We have provided you with the ray tracing algorithm used for precomputing the expected values, but these are just for reference. DO NOT CHANGE THE NUMBER OF BINS as we will use 120 bins to evaluate your implementation. To use the cached values, see the *getExpectedObs(pos)* method defined in [Sensor.java](/src/main/java/sensor/Sensor.java).

## How to run:

**[Run MCL algorithm]**
<br>
We have provided the code for running your MCL algorithm in [Main.java](/src/main/java/Main.java). Make sure you unzip the range_array_120bin.npy.zip file before running. Note however that we will not use your Main.java file for evaluation. As mentioned, you are responsible for setting the default parameters in MonteCarloLocalization.java, CustomSensor.java and CustomOdometry.java files. During evaluation, we will simply make the following calls (as in [ComputePathError.java](/src/main/java/ComputePathError.java)):

```java
int numParticle = 5000;						// Set the maximum number of particles to begin with
boolean recordPosition = true;					// Store the estimated robot poses in a list
  
GlobalMap map = new GlobalMap();						
Sensor sensorModel = new CustomSensor();			// We use your CustomSensor for evaluation
Odometry odometryModel = new CustomOdometry();		// We use your CustomOdometry for evaluation
Plot plot = new Plot(map, numParticle, false);
  
// Load pre-computed expected laser reading values
sensorModel.setExpectedSensorReadingFromCache(RayTracing.loadRayTracing(map));
	
// Instantiate MonteCarloLocalization object and link sensor/odometry models
MonteCarloLocalization mcl = new MonteCarloLocalization(plot, map, numParticle);
mcl.setSensorModel(sensorModel);
mcl.setOdometryModel(odometryModel);
	
// Run MCL algorithm and record the path of the most probable particles
List<double[]> trace = mcl.runMonteCarloLocalization(recordPosition);
```

**[Make animation with the saved figures]** You will find that a snapshot of the map is saved after every iteration. To convert these files into an animation, you have the following two options. 

1. Run [ShowAnimation.java](/src/main/java/ShowAnimation.java).
2. Run the following command (change filename to something else) on your command line tool:

```
ffmpeg -r 60 -f image2 -s 1920x1080 -i savedFigs/pic%04d.png -vcodec libx264 -crf 25 -pix_fmt yuv420p animations/filename.mp4
```

If you get an error, check if *ffmpeg* is installed on your machine. 

**[Test your implementation]** You can run [Tests.java](/src/test/java/Tests.java) file on Eclipse for testing your code.

### Optional reading
See Chapter 5, 6 and 8 of the following text book:
- [Sebastian Thrun, Wolfram Burgard, and Dieter Fox. Probabilistic Robotics. *MIT Press*, 2005](https://docs.ufpr.br/~danielsantos/ProbabilisticRobotics.pdf)


### Submission notes
You should not change the variable names of any variables that we have provided you with. Also, make sure you have adjusted (if found necessary) the values of the following constants before submission (again, DO NOT CHANGE THE VARIABLE NAMES):

In MonteCarloLocalization.java

```java
LIKELIHOOD_SCALE;
MIN_NUM_PARTICLE;
RESAMPLE_PERIOD;
```

Also, you will need to set any parameters that you will have found necessary for correct implementation as final constants in the associated .java files. We will **not** be running your Main.java for evaluation.