import static org.junit.Assert.*;

import org.junit.Test;

import java.util.*;

import map.GlobalMap;
import map.Plot;
import odometry.*;
import sensor.*;
import mcl.*;

public class Tests {
	
	static int logNum = 1;
	static int numParticle = 20;
	static GlobalMap globalMap = new GlobalMap();
	static Sensor sensorModel = new DefaultSensor();
	static Odometry odometryModel = new DefaultOdometry();
	static Plot plot = new Plot(globalMap, numParticle, false);
	static MonteCarloLocalization mcl = new MonteCarloLocalization(plot, globalMap, numParticle);
	static List<Particle> particles = mcl.createParticles();
	
	
	/**
	 * Tests whether particle poses are correctly updated.
	 */
	@Test
	public void testUpdatePose() {
		double[] control = new double[] {10, 5, 0.5};
		Particle particle = particles.get(0);
		particle.setPose(new double[] {1145.9063027900747, 3704.1255051978055, 2.9024268161535804});
		
		mcl.setOdometryModel(odometryModel);
		mcl.updatePose(particle, control);
		
		double[] newPose = particle.getPose();
		double[] expected = new double[] {1155.9063027900747, 3709.1255051978055, 3.4024268161535804};
		
		for (int i = 0; i < 3; i++) {
			assertEquals(expected[i], newPose[i], 1e-10);
		}
	}
	
	
	/**
	 * Tests whether updateWeight method is correctly implemented. Note that we check the logWeight value.
	 */
	@Test
	public void testUpdateWeight() {

		Particle particle = particles.get(0);
		particle.setPose(new double[] {1145.9063027900747, 3704.1255051978055, 2.9024268161535804});
		
		double[] observations = new double[] {
				531,487,478,487,477,466,462,501,491,505,508,480,493,480,512,519,496,500,508,487,476,556,507,472,509,476,473,503,
				490,473,492,470,520,534,483,506,455,486,461,486,500,501,504,514,551,442,486,464,490,489,536,541,508,527,513,480,
				485,516,508,509,479,508,495,506,519,520,500,506,484,509,502,497,547,492,483,504,492,501,519,476,506,521,466,522,
				492,513,531,490,484,471,506,511,488,498,486,502,515,537,506,495,479,516,467,521,484,457,497,473,519,523,502,504,
				522,535,492,505,492,514,503,512,485,512,489,525,506,487,475,522,471,461,526,485,522,484,535,440,489,536,514,518,
				485,518,468,493,490,492,537,517,507,468,507,504,515,485,474,495,523,507,519,471,484,482,470,480,507,506,486,470,
				497,500,486,503,519,479,486,506,486,475,476,506
		}; 
											  
		sensorModel.setExpectedSensorReadingFromCache(RayTracing.loadRayTracing(globalMap));
		
		mcl.setSensorModel(sensorModel);
		mcl.updateWeight(particle, observations);
		double scaledLogWeight = particle.getLogWeight();
		double logWeight = scaledLogWeight / MonteCarloLocalization.LIKELIHOOD_SCALE;
		double expected = -625.4927368164062;
		assertEquals(expected, logWeight, 1e-4);
	}
	
	
	/**
	 * Tests whether the exp-normalize trick has been correctly implemented.
	 */
	@Test
	public void testExpNormalize() {
		double[] logWeight = new double[] {
				7.308781907032909, 4.100808114922017, 2.077148413097171, 
				3.3271705595951118, 9.677559094241207, 0.061171822657613006, 
				9.637047970232077, 9.398653887819098, 9.471949176631938, 
				9.370821488959697, 3.971743421847056, 3.4751802920311023, 
				2.9405703200403677, 5.064836273262351, 1.1596708803265776, 
				7.705358800791777, 6.5989270869342, 1.5674689056984625, 
				3.782020453210553, 1.3976268290375116
		};
		
		for (int i = 0; i < numParticle; i++) {
			double logW = logWeight[i];
			particles.get(i).setLogWeight(logW);
		}
		
		mcl.normalizeWeights(particles, true);
		
		double[] weights = new double[numParticle];
		for (int i = 0; i < numParticle; i++) {
			weights[i] = particles.get(i).getWeight();
		}
		
		double[] expected = new double[] {
				0.0204725457902844, 8.278783685463071E-4, 1.0942140662975753E-4, 3.8192669427083043E-4, 
				0.2187352175501777, 1.4573866418115912E-5, 0.21005109739151173, 0.16549760768195565, 
				0.17808340864486413, 0.16095492241145454, 7.276364580279494E-4, 4.4285323749212004E-4, 
				2.5946672079620346E-4, 0.002170894131955381, 4.371664889425236E-5, 0.030437081540474236, 
				0.0100666697170796, 6.572813291359538E-5, 6.01892335116289E-4, 5.546127113733753E-5
		};
		
		for (int i = 0; i < numParticle; i++) {
			assertEquals(expected[i], weights[i], 1e-4);			
		}
	}
	
	
	/**
	 * Tests whether you have correctly implemented the multinomial sampling method. 
	 * Although it's *very unlikely* that you fail this test with a correct implementation, it is not impossible. 
	 * Hence, passing this test will not guarantee you get the full point. 
	 * Should your method fails to pass this test, we will manually evaluate your sampling method for grading.  
	 */
	@Test
	public void testMultinomialSampling() {		
		Integer[] tempIndexCounts = new Integer[20];
		double[] sampledCounts = new double[20];
		double[] weightArray = new double[] {
				0.07158838902401185, 0.040166781603015504, 0.02034534763095368, 0.03258912133365722, 0.09479019541884982, 
				5.99168547293382E-4, 0.09439339522118967, 0.09205836203373328, 0.09277627805803164, 0.09178574798910005, 
				0.038902612884528474, 0.03403885378425501, 0.028802431745972332, 0.04960928839925213, 0.011358797016607096, 
				0.07547279839748405, 0.06463547078699385, 0.015353115639721937, 0.0370443057331725, 0.013689538752176501
		};
		
		for (int i = 0; i < weightArray.length; i++) {
			double wi = weightArray[i];
			particles.get(i).setWeight(wi);
			sampledCounts[i] = 0;
		}
		
		for (int i = 0; i < 10000; i++) {
			tempIndexCounts = mcl.sampleMultinomial(particles);
			for (int j = 0; j < numParticle; j++) {
				int newCount = tempIndexCounts[j];
				sampledCounts[j] += (double) newCount;
			}
		}
		
		double[] expected = new double[] {
				14379.0, 8074.0, 4109.0, 6568.0, 18687.0, 114.0, 18730.0, 18340.0, 18594.0, 18520.0, 
				7801.0, 6848.0, 5799.0, 9869.0, 2245.0, 15091.0, 13124.0, 3023.0, 7393.0, 2692.0
		};
		
		
		for (int i = 0; i < 20; i++) {
			assertEquals(expected[i], sampledCounts[i], 500);
		}
	}
}
