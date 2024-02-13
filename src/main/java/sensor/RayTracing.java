package sensor;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;

import map.GlobalMap;

/**
 * This class is used for precomputing and loading expected laser readings given the map of Wean hall.
 */
public class RayTracing {
	private static GlobalMap globalMap;
	
	public RayTracing() {
	}
	
	/**
	 * Returns the saved ray tracing results; if the file is not found, creates the file and returns it.
	 * Unless you want to use a different number of bins, you can just unzip .zip file in 'data/' and use it.
	 */
	public static double[][][] loadRayTracing(GlobalMap map) {	
		globalMap = map;
		int numBins = Sensor.NUM_BINS;
		
		String rayTracingFilePath = String.format("data/range_array_%dbin.dat", numBins);
		
		// Creates the expected laser reading array using a ray tracing algorithm
		createRayTracingCache(numBins, rayTracingFilePath);	
		
		// Loads the array from the created file
		try {
			FileInputStream fis = new FileInputStream(rayTracingFilePath);
			ObjectInputStream ois = new ObjectInputStream(fis);
			double[][][] cachedExpectedLaserReading = (double[][][]) ois.readObject();
			System.out.println("File succesfully loaded!");
			ois.close();
			return cachedExpectedLaserReading;
		} catch (ClassNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} 
		return null;
	}	
	
    /**
     * Creates a file that stores an array whose (i, j, k)th element corresponds to the expected laser reading
     * at the corresponding pixel on the map. 
     * 
     * @param numBins				the number of bins 
     * @param rayTracingFilePath 	the file path to which the created array will be stored
     */
	private static void createRayTracingCache(int numBins, String rayTracingFilePath) {
		// Check if the file already exists
		File rayTracingFile = new File(rayTracingFilePath);
		if (rayTracingFile.exists()) {
			System.out.print(String.format("Cached file (%s) found...", rayTracingFilePath));
			return;
		} else {
			System.out.print(String.format("Error: You should first unzip '%s' file!", rayTracingFilePath));
			System.exit(1);
		}
		
		double expectedReading;
		int width = 800;
		int height = 800;
		
		
		// Initialize the ray tracing array and theta values; [0, 2*PI] range of theta is divided into a few number of bins 
		double[][][] rayTracingArray = new double[800][800][numBins];
		double[] raycastDegrees = new double[120];
		double inc = 2 * Math.PI / (numBins-1);
		for (int i = 0; i < 120; i++) {
			raycastDegrees[i] = inc * i;
		}
		
		// Call rayTracingBresenham method to compute the expected reading and store in rayTracingArray
		for (int i = 0; i < width; i++) {
			for (int j = 0; j < height; j++) {
				for (int k = 0; k < numBins; k++) {
					expectedReading = rayTracingBresenham(i, j, raycastDegrees[k]);
					rayTracingArray[i][j][k] = expectedReading;
				}
			}
		}
		
		// Save the array
		System.out.println(String.format("Saving cached expected laser readings to %s...", rayTracingFilePath));
		try {
			FileOutputStream fos = new FileOutputStream(rayTracingFilePath);
			ObjectOutputStream oos = new ObjectOutputStream(fos);
			oos.writeObject(rayTracingArray);
			oos.close();
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	/*
	 * The ray casting algorithm based on the Bresenham's line algorithm (check on Wikipedia).
	 * Given a point (x, y) and a laser direction (theta), searches for the nearest object along the direction.
	 * Note that the maximum laser reading is 8183, and we consider a pixel to be occupied
	 * if the value on the pixel is smaller than 0.8.
	 */
	private static double rayTracingBresenham(int x, int y, double theta) {
		double minValFreeSpace = 0.7;
		int maxDist = 8183;
		double dist;
		maxDist = maxDist / 10;
		
		// Consider a line segment from (x0, y0) to (x1, y1) where (x1, y1) is the furthest possible point considering 
		// the maximum laser range (can be out of the map)
		int x0 = x, y0 = y;
		int x1 = x + (int)(maxDist * Math.cos(theta));
		int y1 = y + (int)(maxDist * Math.sin(theta));
		int sx, sy;
		
		// If (x0, y0) is occupied, just return 0
		if (Double.parseDouble(globalMap.globalMapValues[x][y]) < minValFreeSpace) {
			return 0;
		}
		
		// steep indicates whether slope (dy/dx) is larger than 1 (true)
		boolean steep = false;
		int dx = Math.abs(x1 - x);
		int dy = Math.abs(y1 - y);
		
		sx = (x1 - x > 0) ? 1 : -1;		// Direction matters for Bresenham algorithm 
		sy = (y1 - y > 0) ? 1 : -1;
		
		// Swap values if steep
		if (dy > dx) {
			steep = true;
			int temp;
			temp = y; y = x; x = temp;
			temp = dy; dy = dx; dx = temp;
			temp = sy; sy = sx; sx = temp;
		}
		
		// The difference variable
		int D = (2 * dy) - dx;
		
		try {
			// Increment x and check which pixel on the y axis the line passes through
			// When there is an object at (x, y), then compute the distance between (x, y) and (x0, y0) and return it
			for (int i = 0; i < dx; i++) {
				
				// If x and y have been swapped
				if (steep) {
					if (Double.parseDouble(globalMap.globalMapValues[y][x]) < minValFreeSpace) {
						dist = Math.sqrt(Math.pow(y - x0, 2) + Math.pow(x - y0, 2));
						return Math.min(dist, maxDist) * 10;
					} 
				} else {
					if (Double.parseDouble(globalMap.globalMapValues[x][y]) < minValFreeSpace) {
						dist = Math.sqrt(Math.pow(x - x0, 2) + Math.pow(y - y0, 2));
						return Math.min(dist, maxDist) * 10;
					}
				}
				
				if (D >= 0) {
					y += sy;
					D -= 2 * dx;
				}
				x += sx;
				D += 2 * dy;
			}
			if (steep) {
				dist = Math.sqrt(Math.pow(y - x0, 2) + Math.pow(x - y0, 2));
				return Math.min(dist, maxDist) * 10;
			} else {
				dist = Math.sqrt(Math.pow(x - x0, 2) + Math.pow(y - y0, 2));
				return Math.min(dist, maxDist) * 10;
			}
		} catch (IndexOutOfBoundsException e) {
			dist = Math.sqrt(Math.pow((y - x0), 2) + Math.pow((x - y0), 2));
			return Math.min(dist, maxDist) * 10;
		}
	}
}
