package map;

import java.io.*;

/**
 * A class that defines the map object. The map is in the form of a 800 x 800 matrix 
 * where each entry represents a location on the map. 
 *<p>
 * The value of each entry:
 *        -1  = don't know
 *<p>
 * any value in [0;1] is a probability for occupancy:
 * 		  e.g.
 *        1   = occupiable by robot with probability 1
 *        0   = unoccupiable with probability 1
 *        0.5 = occupiable with probability 0.5
 **/
public class GlobalMap {

	public String[][] globalMapValues;
	
	public GlobalMap() {	
		this("data/map/wean.dat");
		}
	
	public GlobalMap(String mapFilePath) {
		globalMapValues = new String[800][800];
		loadMap(mapFilePath);
	}
	
	/**
	 * Loads the map of Wean hall and saves it into 2D array of Strings.
	 * @param filePath	the file path to the map file ("data/map/wean.dat")
	 */
	private void loadMap(String filePath) {
		try (BufferedReader br = new BufferedReader(new FileReader(filePath))) {
			String line;
			int i = 0;
			while ((line = br.readLine()) != null) {
				String[] values = line.split(" ");
				globalMapValues[i] = values;
				i++;
			}
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		} 
	}
}
