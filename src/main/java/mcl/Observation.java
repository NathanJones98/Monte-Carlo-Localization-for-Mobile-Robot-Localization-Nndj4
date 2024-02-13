package mcl;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.*;

import sensor.Sensor;
import util.Util;

/**
 * A class that loads, processes, and stores the robot data
 *
 * A single datum in "robotdata%d.log" comes in either one of the two types:
 * Type "O" -- contains only robot pose:
 *      x y theta - coordinates of the robot in standard odometry frame
 *      timestamp - timestamp of odometry reading (starts at ~ 0)
 * <p>
 * Type L -- contains robot pose, laser pose, and laser readings:
 *      x y theta - coodinates of the robot in standard odometry frame when laser reading was taken
 *      xl yl thetal - coordinates of the *laser* in standard odometry frame when the laser reading was taken
 *      1 .. 180 - 180 range readings of laser in cm. The 180 readings span 180 degrees *COUNTER-CLOCKWISE* just like angles.
 *      timestamp - timestamp of laser reading
 */
public class Observation {

	private static String filePath;
	private static List<String[]> rawAllObs;
	public static List<Observation> obsList;
	
	public String dataType;
	public double timeStamp;
	public double[] robotPose;
	public double[] laserPose;
	public double[] laserReadings;
	
	/**
	 * The constructor which receives a raw array of observation data as Strings.
	 * 
	 * @param rawObservation	an array of Strings corresponding to one time step
	 */
	public Observation(String[] rawObservation) {
		processSingleObservation(rawObservation);
	}
	
	/**
	 * Parses a raw array of Strings containing odometry and/or laser readings and stores them into Observation object.
     * Type L ['type', 'x', 'y', 'theta', 'xl', 'yl', 'thetal', r1 ~r180, 'ts']
     * Type O ['type', 'x', 'y', 'theta', 'ts']
     *  
	 * @param rawObservation	an array of Strings corresponding to one time step
	 */
	private void processSingleObservation(String[] rawObservation) {
		dataType = rawObservation[0];
		
		// Type "O"
		if (dataType.equals("O")) {
			timeStamp = Double.parseDouble(rawObservation[4]);
			robotPose = Util.convertDouble(Arrays.copyOfRange(rawObservation, 1, 4));
		} else {
			// Type "L"
			if (rawObservation.length < 188) {
				throw new IndexOutOfBoundsException("L data incomplete");
			}
			timeStamp = Double.parseDouble(rawObservation[187]);
			robotPose = Util.convertDouble(Arrays.copyOfRange(rawObservation, 1, 4));
			laserPose = Util.convertDouble(Arrays.copyOfRange(rawObservation, 4, 7));
			laserReadings = Util.convertDouble(Arrays.copyOfRange(rawObservation, 7, 187));
			
			// Trim laser readings that are larger than Sensor.MAX_LASER_RANGE
			for (int i = 0; i < 180; i++) {
				if (laserReadings[i] > Sensor.MAX_LASER_RANGE) {
					laserReadings[i] = Sensor.MAX_LASER_RANGE;
				}
			}
		}
	}
	
	/**
	 * Loads the logged robot data.
	 * 
	 * @param	logFilePath		the path to the file
	 * @return 	a list of Observation objects
	 */
	public static List<Observation> loadRobotLogData(String logFilePath) {
		filePath = logFilePath.intern();
		rawAllObs = new ArrayList<>();
		
		// Reads in data in String type
		try (BufferedReader br = new BufferedReader(new FileReader(filePath))){
			String line;
			while ((line = br.readLine()) != null) {
				String[] values = line.split(" ");
				rawAllObs.add(values);
			}
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		// Instantiate an Observation object per data sample
		obsList = new ArrayList<>();
		for (String[] rawObs : rawAllObs) {
			Observation obs = new Observation(rawObs);
			obsList.add(obs);
		}
		return obsList;
	}
}
